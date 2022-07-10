#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import BSDFReSTIR;
import Utils.Helpers;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
import Utils.VirtualLight.VirtualLightContainer;
import Utils.VirtualLight.MegaTextureContainer;
import RenderPasses.GBufferUtils;
import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Lights.Lights;
import Scene.Shading;
import Scene.RaytracingInlineHelper;
import Experimental.Scene.Lights.LightCollection;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Experimental.Scene.Lights.EnvMapSampler;
import Experimental.Scene.Material.MaterialShading;
import Scene.Material.SimpleBRDF;
import Utils.Sampling.AliasTable;
import Utils.Debug.PixelDebug;
import Experimental.Scene.Material.Fresnel;

static const bool kUseRussianRoulette = true;
static const bool kUseEnvLight = USE_ENV_LIGHT;
static const bool kUseAnalyticLights = USE_ANALYTIC_LIGHTS;
static const bool kUseEmissiveLights = USE_EMISSIVE_LIGHTS;
static const bool kUseVirtualLights = USE_VIRTUAL_LIGHTS;

static const bool kUseNEE = true;
static const bool kUseMIS = true;

static const bool kVirtualLightUseNEE = USE_VIRTUAL_LIGHT_NEE;
static const bool kVirtualLightUsePreciseNEE = ENABLE_PRECISE_VIRTUAL_LIGHT_NEE;

static const bool kVirtualLightUseMIS = USE_VIRTUAL_LIGHT_MIS;

static const uint kBounceNum = _BOUNCE_NUM;
static const bool kOutputDepth = _OUTPUT_DEPTH;
static const bool kOutputOutgoingRadiance = _OUTPUT_OUTGOING_RADIANCE;

static const uint kShadingMode = _SHADING_MODE;

static const bool kUseTraditionalVXL = _USE_TRADITIONAL_VXL;
static const bool kUseRichVXL = _USE_RICH_VXL;
static const bool kUseVSL = _USE_VSL;

static const bool kUseMixtureBSDF = _USE_MIXTURE_BSDF;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    VirtualLightContainer gVirtualLights;
    AliasTable gVirtualLightFluxTable;
    GBuffer gGBuffer;
    MegaTextureContainer gSpecRadianceContainer;
    EmissiveLightSampler gEmissiveLightSampler;
    EmissiveLightSampler gVirtualLightBVHSampler;
    EnvMapSampler gEnvMapSampler;
    float gShortDistance;
    float gShortDistanceRange;
    float gShortDistanceNumVSLs;
    float gLongDistanceNumVSLs;
    bool gUseNewThresholds;
    float gMaxPathIntensity;
    uint  gNEESamples;
    bool  gUseRIS;
    float gRoughnessThreshold;
    bool  gUseVirtualLightBVH;

    ResamplingRuntimeParameters gParams;
    uint gInputBufferIndex;
    bool gMISWithPowerSampling;
    int gNumPowerSamples;
    int gNumBSDFSamples;
    int gCurrentVPLIndex;
}

RWStructuredBuffer<Reservoir> gReservoirs;
RWTexture2D<float4> gShadingOutput;

struct SimpleReservoir
{
    float3 selectedWeight;
    float weightSum;
    //bool isVirtualLightNEE;
};

SimpleReservoir emptySimpleReservoir()
{
    SimpleReservoir s;
    s.selectedWeight = 0.f;
    s.weightSum = 0.f;
    //s.isVirtualLightNEE = false;
    return s;
}

bool streamSample(
    inout SimpleReservoir reservoir,
    float3 weight,
    float random)
{
    // What's the current weight
    float risWeight = luminance(weight);
    // Update the weight sum
    reservoir.weightSum += risWeight;
    // Decide if we will randomly pick this sample
    bool selectSample = (random * reservoir.weightSum < risWeight);

    // If we did select this sample, update the relevant data.
    // New samples don't have visibility or age information, we can skip that.
    if (selectSample)
    {
        reservoir.selectedWeight = weight;
    }

    return selectSample;
}

struct SavedRay
{
    float3 org;
    float3 dir;
    float tMax;
}

void accumulateRadiance(inout float3 TotalRadiance, float3 PathRadiance, float maxPathIntensity)
{
	if (maxPathIntensity > 0)
	{
		// User asked for path contributions to be clamped to reduce fireflies.
		// Depending on how aggressive this value is, the image could be quite biased
		TotalRadiance += min(PathRadiance, maxPathIntensity);
	}
	else
	{
		// Just average values directly
		TotalRadiance += PathRadiance;
	}
}

float3 evalVirtualLightReSTIRNEE(uint2 pixelPos, ShadingData sd, SimpleBRDF simpleBRDF)
{
    float3 res = 0.f;
    Reservoir r = loadReservoir(gParams, gReservoirs, pixelPos, gInputBufferIndex);

    if (isValidReservoir(r))
    {
        float actualDis = length(r.position - sd.posW);
        float blendFactor = saturate((actualDis - gShortDistance) / gShortDistanceRange);
        float3 sampledDir = normalize(r.position - sd.posW);
        float3 rayOrigin = sd.computeNewRayOrigin();
        
        TriangleHit trianglehit;
        ShadingData nextSd;
        if (blendFactor > 0.f && traceScatterRay(rayOrigin, sampledDir, nextSd, trianglehit))
        {
            float3 wo = toLocal(sd.V, sd.N);
            float3 wi = toLocal(sampledDir, sd.N);
            float invPdf = getReservoirInvPdf(r);
            float3 brdfVal = simpleBRDF.eval(wo, wi) * invPdf;

            float3 radiance = gVirtualLights.readRadianceByPosition(sd.posW, nextSd.posW, nextSd.N, gSpecRadianceContainer);

            if (actualDis > gShortDistance)
            {
                res += blendFactor * brdfVal * radiance;
            }
        }
    }

    return res;
}

float3 evalVirtualLightingNEEPrecisely(ShadingData sd, SimpleBRDF simpleBRDF, inout SampleGenerator sg)
{
    int lightIndex = gVirtualLightFluxTable.sample(sampleNext2D(sg));
    float lightSelectionPdf = gVirtualLightFluxTable.getWeight(lightIndex) / gVirtualLightFluxTable.weightSum;

    // perform cone sampling
    float3 sampledDir;
    float conePdf;
    gVirtualLights.sampleVSL(lightIndex, sg, sd.posW, sd.N, sampledDir, conePdf);
    float3 wi = toLocal(sampledDir, sd.N);

    ShadingData nextSd;
    TriangleHit trianglehit;
    float3 vplPosition = gVirtualLights.getPosition(lightIndex);
    float virtualLightRadius = gVirtualLights.getVSLRadius(lightIndex);
    float3 rayOrigin = computeRayOrigin(sd.posW, dot(sd.faceN, sampledDir) >= 0 ? sd.faceN : -sd.faceN);
    float tmax = length(vplPosition - rayOrigin) + virtualLightRadius;

    if (castScene(rayOrigin, sampledDir, nextSd, trianglehit, tmax))
    {
        // if shorter than shortDistance, reject
        float actualDis = distance(nextSd.posW, sd.posW);
        if (actualDis <= gShortDistance)
        {
            return 0.0f;
        }

        bool isSampleValid = false;
        // confirm that we are inside the VSL and the VSL is not rejected by anti-leaking conditions
        if (insideSphere(nextSd.posW, vplPosition, virtualLightRadius))
        {
            float3 virtualLightN = gVirtualLights.getNormal(lightIndex);

#if _SHADING_NORMAL_CHECK
            if (!readRadianceCheckingNormal || dot(nextSd.N, virtualLightN) > 0.7f)
#endif
            {
                isSampleValid = true;
#if _SHADING_MATERIAL_CHECK
                const float roughnessThreshold = 0.5f;
                const float reflectivityThreshold = 0.5f;
                const float albedoThreshold = 0.5f;

                if (!compareRelativeDifference(nextSd.linearRoughness, gVirtualLights.getRoughness(lightIndex), roughnessThreshold) ||
                    !compareRelativeDifference(luminance(nextSd.specular), luminance(gVirtualLights.getSpecular(lightIndex)), reflectivityThreshold) ||
                    !compareRelativeDifference(luminance(nextSd.diffuse), luminance(gVirtualLights.getDiffuse(lightIndex)), albedoThreshold))
                {
                    isSampleValid = false;
                }
#endif
            }
        }

        if (!isSampleValid)
        {
            return 0.0f;
        }

        float3 wo = toLocal(sd.V, sd.N);
        float3 wi = toLocal(sampledDir, sd.N);

        float diffuseBrdfPdf = 0.f;
        float specularBrdfPdf = 0.f;
        float brdfPdf = 0.f;

        if (kUseMixtureBSDF || !gUseNewThresholds)
        {
            brdfPdf = simpleBRDF.evalPdf(wo, wi);
        }
        else
        {
            diffuseBrdfPdf = simpleBRDF.evalDiffusePdf(wo, wi);
            specularBrdfPdf = simpleBRDF.evalSpecularPdf(wo, wi);
            brdfPdf = simpleBRDF.pDiffuseReflection * diffuseBrdfPdf + simpleBRDF.pSpecularReflection * specularBrdfPdf;
        }

        float blendFactor = 1.f;
        float blendFactorDiffuse = 1.f;
        float blendFactorSpecular = 1.f;

        if (gUseNewThresholds)
        {
            float avgRadius2 = virtualLightRadius * virtualLightRadius; // to really match BSDF sampling blending factor, we need to perform neighbor search for the average radius
            // alternatively, we can compute a different blending factor for each light sampled by BSDF
            float estimatedNumVPLs = (actualDis * actualDis / abs(dot(nextSd.N, -sampledDir))) / avgRadius2;
            if (kUseMixtureBSDF)
            {
                estimatedNumVPLs /= brdfPdf;
                blendFactor = saturate(float(estimatedNumVPLs - gShortDistanceNumVSLs) / (gLongDistanceNumVSLs - gShortDistanceNumVSLs));
            }
            else
            {
                float estimatedNumVPLsForDiffuse = estimatedNumVPLs / diffuseBrdfPdf;
                blendFactorDiffuse = saturate(float(estimatedNumVPLsForDiffuse - gShortDistanceNumVSLs) / (gLongDistanceNumVSLs - gShortDistanceNumVSLs));
                float estimatedNumVPLsForSpecular = estimatedNumVPLs / specularBrdfPdf;
                blendFactorSpecular = saturate(float(estimatedNumVPLsForSpecular - gShortDistanceNumVSLs) / (gLongDistanceNumVSLs - gShortDistanceNumVSLs));
                blendFactor = blendFactorDiffuse + blendFactorSpecular;
            }
        }
        else
        {
            float minDis = gShortDistance;
            float maxDis = gShortDistance + gShortDistanceRange;
            blendFactor = saturate((actualDis - minDis) / gShortDistanceRange);
        }

        if (blendFactor == 0.f)
        {
            return 0.0f;
        }

        float misWeight = 1.0f;
        if (kVirtualLightUseMIS)
        {
            misWeight = evalMultiSampleMISRobust(conePdf * lightSelectionPdf, brdfPdf, gNEESamples, 1);
        }

        float3 brdfVal = 0;
        float3 diffuseBrdfVal = simpleBRDF.evalDiffuse(wo, wi);
        float3 specularBrdfVal = simpleBRDF.evalSpecular(wo, wi);

        if (kUseMixtureBSDF || !gUseNewThresholds)
            brdfVal = blendFactor * (diffuseBrdfVal + specularBrdfVal);
        else
        {
            brdfVal = diffuseBrdfVal * blendFactorDiffuse + specularBrdfVal * blendFactorSpecular;
        }

        if (sd.linearRoughness <= gRoughnessThreshold)
        {
            brdfVal *= simpleBRDF.pDiffuseReflection;
        }

        float invArea = 1.0f / (M_PI * virtualLightRadius * virtualLightRadius);
        return (1.0f / gNEESamples) * invArea * misWeight * brdfVal * gVirtualLights.readLe(lightIndex, sampledDir, gSpecRadianceContainer) / (conePdf * lightSelectionPdf);
    }

    return 0.0f;
}

float3 evalVXLNEENaive(ShadingData sd, SimpleBRDF simpleBRDF, inout SampleGenerator sg)
{
    uint lightIndex = gCurrentVPLIndex;
    float lightSelectionPdf = 1.0f / gVirtualLights.count;

    float3 virtualLightPos = gVirtualLights.getPosition(lightIndex);

    float3 toLight = normalize(virtualLightPos - sd.posW);
    float3 shadingOrigin = computeRayOrigin(sd.posW, dot(sd.N, toLight) >= 0 ? sd.N : -sd.N);
    float3 vplNormal = gVirtualLights.getNormal(lightIndex);
    float3 lightOrigin = computeRayOrigin(virtualLightPos, vplNormal);
    
    if (!traceShadowRay(shadingOrigin, toLight, distance(lightOrigin, shadingOrigin)))
    {
        return 0.0f;
    }

#if _USE_VSL
    const uint sampleCount = 16;
#else
    const uint sampleCount = 1;
#endif
    float3 result = 0.0f;
    for (uint i = 0; i < sampleCount; i++)
    {
        float3 sampledDir;
        float conePdf = 1.0f;
        if (kUseVSL)
        {
            bool coneSampleValid = gVirtualLights.isConeSamplingValid(lightIndex, sd.posW);
            if (coneSampleValid)
            {
                gVirtualLights.coneSamplingVSL(lightIndex, sampleNext2D(sg), sd.posW, sampledDir, conePdf);
            }
            else
            {
                sampledDir = cosineWeightedSampling(sampleNext2D(sg), sd.N, conePdf);
            }
        }
        else
        {
            sampledDir = normalize(virtualLightPos - sd.posW);
        }

        float3 wo = toLocal(sd.V, sd.N);
        float3 wi = toLocal(sampledDir, sd.N);
        if (wi.z <= 1e-6f)
        {
            continue;
        }

        float3 Le;
        if (kUseRichVXL)
        {
            Le = gVirtualLights.readLe(lightIndex, sampledDir, gSpecRadianceContainer);
        }
        else
        {
            Le = gVirtualLights.readLe(lightIndex, sampledDir);
        }


        float3 brdfVal = simpleBRDF.evalFr(wo, wi);
        if (sd.linearRoughness <= gRoughnessThreshold)
        {
            brdfVal *= simpleBRDF.pDiffuseReflection;
        }

        float radius = gVirtualLights.getVSLRadius(lightIndex);
        if (kUseVSL)
        {
            float invArea = 1.0f / (M_PI * radius * radius);
            float attenuation = invArea * saturate(dot(vplNormal, -sampledDir)) * wi.z;
            result += ((1.0f / sampleCount) * attenuation * brdfVal * Le / (lightSelectionPdf * conePdf));
        }
        else
        {
            float attenuation = simpliedCemAttenuation(length(virtualLightPos - sd.posW), 2.0 * radius) * saturate(dot(vplNormal, -sampledDir)) * wi.z;
            result += ((1.0f / sampleCount) * attenuation * brdfVal * Le / lightSelectionPdf);
        }

        if (i >= sampleCount)
        {
            break;
        }
    }
    return result;
}

float3 evalVXLNEE(ShadingData sd, SimpleBRDF simpleBRDF, inout SampleGenerator sg)
{
    uint lightIndex;
    float lightSelectionPdf;
    bool foundLight = false;
#if 1
    if (gUseVirtualLightBVH)
    {
        foundLight = gVirtualLightBVHSampler.sampleVirtualLight(sd.posW, sd.N, true, sampleNext1D(sg), lightIndex, lightSelectionPdf);
    }
    else
    {
        lightIndex = gVirtualLightFluxTable.sample(sampleNext2D(sg));
        lightSelectionPdf = gVirtualLightFluxTable.getWeight(lightIndex) / gVirtualLightFluxTable.weightSum;
        foundLight = true;
    }
#else
    gVirtualLights.sampleUniform(sampleNext1D(sg), lightIndex, lightSelectionPdf);
    foundLight = true;
#endif

    if (!foundLight) 
    {
        return 0.f;
    }

    float3 virtualLightPos = gVirtualLights.getPosition(lightIndex);

    float3 toLight = normalize(virtualLightPos - sd.posW);
    float3 shadingOrigin = computeRayOrigin(sd.posW, dot(sd.N, toLight) >= 0 ? sd.N : -sd.N);
    float3 vplNormal = gVirtualLights.getNormal(lightIndex);
    float3 lightOrigin = computeRayOrigin(virtualLightPos, vplNormal);
    
    if (!traceShadowRay(shadingOrigin, toLight, distance(lightOrigin, shadingOrigin)))
    {
        return 0.0f;
    }

#if _USE_VSL
    const uint sampleCount = 16;
#else
    const uint sampleCount = 1;
#endif
    float3 result = 0.0f;
    for (uint i = 0; i < sampleCount; i++)
    {
        float3 sampledDir;
        float conePdf = 1.0f;
        if (kUseVSL)
        {
            bool coneSampleValid = gVirtualLights.isConeSamplingValid(lightIndex, sd.posW);
            if (coneSampleValid)
            {
                gVirtualLights.coneSamplingVSL(lightIndex, sampleNext2D(sg), sd.posW, sampledDir, conePdf);
            }
            else
            {
                sampledDir = cosineWeightedSampling(sampleNext2D(sg), sd.N, conePdf);
            }
        }
        else
        {
            sampledDir = normalize(virtualLightPos - sd.posW);
        }

        float3 wo = toLocal(sd.V, sd.N);
        float3 wi = toLocal(sampledDir, sd.N);
        if (wi.z <= 1e-6f)
        {
            continue;
        }

        float3 Le;
        if (kUseRichVXL)
        {
            Le = gVirtualLights.readLe(lightIndex, sampledDir, gSpecRadianceContainer);
        }
        else
        {
            Le = gVirtualLights.readLe(lightIndex, sampledDir);
        }


        float3 brdfVal = simpleBRDF.evalFr(wo, wi);
        if (sd.linearRoughness <= gRoughnessThreshold)
        {
            brdfVal *= simpleBRDF.pDiffuseReflection;
        }

        float radius = gVirtualLights.getVSLRadius(lightIndex);
        if (kUseVSL)
        {
            float invArea = 1.0f / (M_PI * radius * radius);
            float attenuation = invArea * saturate(dot(vplNormal, -sampledDir)) * wi.z;
            result += ((1.0f / gNEESamples) * (1.0f / sampleCount) * attenuation * brdfVal * Le / (lightSelectionPdf * conePdf));
        }
        else
        {
            float attenuation = simpliedCemAttenuation(length(virtualLightPos - sd.posW), 2.0 * radius) * saturate(dot(vplNormal, -sampledDir)) * wi.z;
            result += ((1.0f / gNEESamples) * (1.0f / sampleCount) * attenuation * brdfVal * Le / lightSelectionPdf);
        }

        if (i >= sampleCount)
        {
            break;
        }
    }
    return result;
}

float3 evalVirtualLightingNEE(ShadingData sd, SimpleBRDF simpleBRDF, inout SampleGenerator sg
#if USE_RIS
    , inout SavedRay savedRay
#endif
)
{
    uint lightIndex;
    float lightSelectionPdf;
    bool foundLight = false;
#if 1
    if (gUseVirtualLightBVH)
    {
        foundLight = gVirtualLightBVHSampler.sampleVirtualLight(sd.posW, sd.N, true, sampleNext1D(sg), lightIndex, lightSelectionPdf);
    }
    else
    {
        lightIndex = gVirtualLightFluxTable.sample(sampleNext2D(sg));
        lightSelectionPdf = gVirtualLightFluxTable.getWeight(lightIndex) / gVirtualLightFluxTable.weightSum;
        foundLight = true;
    }
#else
    gVirtualLights.sampleUniform(sampleNext1D(sg), lightIndex, lightSelectionPdf);
    foundLight = true;
#endif

    if (!foundLight) return 0.f;

    float3 sampledDir;
    float conePdf;
    gVirtualLights.sampleVSL(lightIndex, sg, sd.posW, sd.N, sampledDir, conePdf);
    
    float3 wo = toLocal(sd.V, sd.N);
    float3 wi = toLocal(sampledDir, sd.N);

    float3 virtualLightPos = gVirtualLights.getPosition(lightIndex);
    float3 virtualLightNormal = gVirtualLights.getNormal(lightIndex);
    float virtualLightRadius = gVirtualLights.getRadius(lightIndex);

    // if out of disk, reject
    float t;
    if (!intersectDisk(-virtualLightNormal, virtualLightPos, virtualLightRadius, sd.posW, sampledDir, t))
    {
        return 0.0f;
    }

    // if shorter than shortDistance, reject
    float actualDis = t;

    float diffuseBrdfPdf = 0.f;
    float specularBrdfPdf = 0.f;
    float brdfPdf = 0.f;

    if (kUseMixtureBSDF || !gUseNewThresholds)
    {
        brdfPdf = simpleBRDF.evalPdf(wo, wi);
    }
    else
    {
        diffuseBrdfPdf = simpleBRDF.evalDiffusePdf(wo, wi);
        specularBrdfPdf = simpleBRDF.evalSpecularPdf(wo, wi);
        brdfPdf = simpleBRDF.pDiffuseReflection * diffuseBrdfPdf + simpleBRDF.pSpecularReflection * specularBrdfPdf;
    }


    float blendFactor = 1.f;
    float blendFactorDiffuse = 1.f;
    float blendFactorSpecular = 1.f;

    if (gUseNewThresholds)
    {
        float avgRadius2 = virtualLightRadius * virtualLightRadius; // to really match BSDF sampling blending factor, we need to perform neighbor search for the average radius
        // alternatively, we can compute a different blending factor for each light sampled by BSDF
        float estimatedNumVPLs = (actualDis * actualDis / abs(dot(virtualLightNormal, -sampledDir))) / avgRadius2;
        if (kUseMixtureBSDF)
        {
            estimatedNumVPLs /= brdfPdf;
            blendFactor = saturate(float(estimatedNumVPLs - gShortDistanceNumVSLs) / (gLongDistanceNumVSLs - gShortDistanceNumVSLs));
        }
        else
        {
            float estimatedNumVPLsForDiffuse = estimatedNumVPLs / diffuseBrdfPdf;
            blendFactorDiffuse = saturate(float(estimatedNumVPLsForDiffuse - gShortDistanceNumVSLs) / (gLongDistanceNumVSLs - gShortDistanceNumVSLs));
            float estimatedNumVPLsForSpecular = estimatedNumVPLs / specularBrdfPdf;
            blendFactorSpecular = saturate(float(estimatedNumVPLsForSpecular - gShortDistanceNumVSLs) / (gLongDistanceNumVSLs - gShortDistanceNumVSLs));
            blendFactor = blendFactorDiffuse + blendFactorSpecular;
        }
    }
    else
    {
        float minDis = gShortDistance;
        float maxDis = gShortDistance + gShortDistanceRange;
        blendFactor = saturate((actualDis - minDis) / gShortDistanceRange);
    }

    if (blendFactor == 0.f)
    {
        return 0.0f;
    }

    float3 shadingOrigin = computeRayOrigin(sd.posW, dot(sd.faceN, sampledDir) >= 0 ? sd.faceN : -sd.faceN);
    bool visible = true;

#if USE_RIS
    savedRay = { shadingOrigin, sampledDir, max(t - virtualLightRadius, 0.0f) };
#else
    visible = traceShadowRay(shadingOrigin, sampledDir, max(t - virtualLightRadius, 0.0f));
#endif

    if (!visible)
    {
        return 0.0f;
    }

    float misWeight = 1.0f;
    if (kVirtualLightUseMIS)
    {
        //misWeight = evalMISRobust(conePdf * lightSelectionPdf, brdfPdf);
        misWeight = evalMultiSampleMISRobust(conePdf * lightSelectionPdf, brdfPdf, gNEESamples, 1);
    }

    float3 brdfVal = 0;
    float3 diffuseBrdfVal = simpleBRDF.evalDiffuse(wo, wi);
    float3 specularBrdfVal = simpleBRDF.evalSpecular(wo, wi);

    if (kUseMixtureBSDF || !gUseNewThresholds)
        brdfVal = blendFactor * (diffuseBrdfVal + specularBrdfVal);
    else
    {
        brdfVal = diffuseBrdfVal * blendFactorDiffuse + specularBrdfVal * blendFactorSpecular;
    }

    if (sd.linearRoughness <= gRoughnessThreshold)
    {
        brdfVal *= simpleBRDF.pDiffuseReflection;
    }

    float invArea = 1.0f / (M_PI * virtualLightRadius * virtualLightRadius);
    return (1.0f / gNEESamples) * invArea * misWeight * brdfVal * gVirtualLights.readLe(lightIndex, sampledDir, gSpecRadianceContainer) / (conePdf * lightSelectionPdf);
}

float3 evalEmissiveLightingNEE(ShadingData sd, inout SampleGenerator sg, EmissiveLightSampler emissiveSampler)
{
    TriangleLightSample tls;
    if (emissiveSampler.sampleLight(sd.posW, sd.N, true, sg, tls))
    {
        SimpleBRDF simpleBRDF;
        simpleBRDF.setup(sd);

        float3 lightPos = computeRayOrigin(tls.posW, tls.normalW);
        float3 shadingOrigin = computeRayOrigin(sd.posW, dot(sd.faceN, lightPos - sd.posW) >= 0 ? sd.faceN : -sd.faceN);
        float3 toLight = lightPos - shadingOrigin;
        float lightDistance = length(toLight);
        float3 L = normalize(toLight);

        if (dot(L, sd.N) <= 1e-6f)
        {
            return 0.0f;
        }

        bool visible = traceShadowRay(shadingOrigin, L, lightDistance);
        if (!visible)
        {
            return float3(0.f);
        }

        float pdf = tls.pdf;
        float3 Le = tls.Le;

        float3 wo = toLocal(sd.V, sd.N);
        float3 wi = toLocal(L, sd.N);
        float misWeight = 1.0f;
        if (kUseMIS)
        {
            misWeight = evalMISRobust(pdf, simpleBRDF.evalPdf(wo, wi));
        }
        return misWeight * Le * simpleBRDF.eval(wo, wi) / (pdf);
    }
    
    return 0.0f;
}

float3 readOutgoingRadiance(ShadingData sd)
{
    float3 virtualLightRadiance = 0.0f;
    RayDesc searchRay;
    searchRay.Origin = sd.posW;
    searchRay.Direction = float3(0.0f, 1.0f, 0.0f);
    searchRay.TMin = 0.0f;
    searchRay.TMax = 0.0f;
    RayQuery<RAY_FLAG_NONE> rayQuery;
    rayQuery.TraceRayInline(gVirtualLights.as, RAY_FLAG_NONE, 0xff, searchRay);

    while(rayQuery.Proceed())
    {
        if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
        {
            uint lightIndex = rayQuery.CandidatePrimitiveIndex();
            float3 lightPosW = gVirtualLights.getPosition(lightIndex);
            float radius = gVirtualLights.getRadius(lightIndex);
            float disSquare = dot(sd.posW - lightPosW, sd.posW - lightPosW);
            if(disSquare <= radius * radius)
            {
                float3 L = -sd.V;
                {
                    float area = M_PI * radius * radius;
                    virtualLightRadiance += (gVirtualLights.readLe(lightIndex, L, gSpecRadianceContainer) / area);
                }
            }
        }
    }
    
    return virtualLightRadiance;
}


inline float3 UnifiedShading(uint2 pixelPos)
{
    float3 color = 0.0f;
    SampleGenerator sg = SampleGenerator.create(pixelPos.xy, gFrameIndex);

    ShadingData sd;
    if (!LoadShadingData(gGBuffer, pixelPos.xy, gViewportDims, gScene.camera, sd))
    {
        if (kUseEnvLight)
        {
            color = gScene.envMap.eval(gScene.camera.computeRayPinhole(pixelPos.xy, gViewportDims).dir);
        }
        return color;
    }
    
    if (kOutputOutgoingRadiance)
    {
        return readOutgoingRadiance(sd);
    }

    color += sd.emissive;

    SimpleBRDF simpleBRDF;
    simpleBRDF.setup(sd);

    float3 throughput = 1.0f;

    bool isDelta = false;
    bool isSmooth = false;
    float3 rayOrigin = sd.computeNewRayOrigin();
    float3 rayDirection;

    float depthVal = 0.0f;
    float depthThp = 1.0f;
    uint firstSampableDepth = 0;

    for (uint depth = 0; depth < kBounceNum; depth++)
    {
        if (kOutputDepth)
        {
            depthVal += depthThp;
        }

        if (kUseNEE && kUseEmissiveLights)
        {
            accumulateRadiance(color, throughput * evalEmissiveLightingNEE(sd, sg, gEmissiveLightSampler), (depth == 0 || !kUseVirtualLights) ? 1e30f : gMaxPathIntensity);
        }

        if ((kVirtualLightUseNEE || (kShadingMode == 1 && depth == 0)) && kUseVirtualLights)
        {
            // for RIS
#if USE_RIS
            SimpleReservoir reservoir = emptySimpleReservoir();
            SavedRay selectedShadowRay;
            SavedRay candidateShadowRay;

            for (uint lightId = 0; lightId < gNEESamples; lightId++)
            {
                float3 weight = evalVirtualLightingNEE(sd, simpleBRDF, sg, candidateShadowRay);
                if (streamSample(reservoir, weight, sampleNext1D(sg)))
                {
                    selectedShadowRay = candidateShadowRay;
                }
            }

            // verify visibility
            if (reservoir.weightSum > 0.f && traceShadowRay(selectedShadowRay.org, selectedShadowRay.dir, selectedShadowRay.tMax))
            {
                accumulateRadiance(color, throughput * reservoir.selectedWeight / luminance(reservoir.selectedWeight) * reservoir.weightSum, gMaxPathIntensity);
            }
#else

            if (kShadingMode == 1 && depth == 0)
            {
                accumulateRadiance(color, throughput * evalVirtualLightReSTIRNEE(pixelPos, sd, simpleBRDF), gMaxPathIntensity);
            }
            else if (kShadingMode == 2 && kUseTraditionalVXL)
            {
                accumulateRadiance(color, throughput * evalVXLNEENaive(sd, simpleBRDF, sg), gMaxPathIntensity);
            }
            else
            {
                for (uint lightId = 0; lightId < gNEESamples; lightId++)
                {
                    if (kUseTraditionalVXL)
                    {
                        accumulateRadiance(color, throughput * evalVXLNEE(sd, simpleBRDF, sg), gMaxPathIntensity);
                    }
                    else
                    {
                        if (kVirtualLightUsePreciseNEE)
                        {
                            accumulateRadiance(color, throughput * evalVirtualLightingNEEPrecisely(sd, simpleBRDF, sg), gMaxPathIntensity);
                        }
                        else
                        {
                            accumulateRadiance(color, throughput * evalVirtualLightingNEE(sd, simpleBRDF, sg), gMaxPathIntensity);
                        }
                    }
                }
            }
#endif
        }

        float pdfBrdf;
        float pdfSubBRDF;
        uint lobeType;
        bool valid = simpleBRDF.sample(sd, rayDirection, pdfBrdf, pdfSubBRDF, lobeType, sg);
        if (!valid || min(sd.NdotV, dot(rayDirection, sd.N)) < 1e-6f)
        {
            break;
        }
        isDelta = (lobeType == 3);
        isSmooth = (lobeType == 2 && sd.linearRoughness <= gRoughnessThreshold);
        if ((isDelta || isSmooth) && (firstSampableDepth == depth))
        {
            firstSampableDepth += 1u;
        }
        float3 wo = toLocal(sd.V, sd.N);
        float3 wi = toLocal(rayDirection, sd.N);
        float3 newThroughput;
        if (!isDelta)
        {
            if (kUseMixtureBSDF)
            {
                newThroughput = throughput * simpleBRDF.eval(wo, wi) / pdfBrdf;
            }
            else
            {
                if (lobeType == 1) newThroughput = throughput * simpleBRDF.evalDiffuse(wo, wi) / (simpleBRDF.pDiffuseReflection * pdfSubBRDF);
                else newThroughput = throughput * simpleBRDF.evalSpecular(wo, wi) / (simpleBRDF.pSpecularReflection * pdfSubBRDF);
            }
        }
        else
        {
            float woDotN = saturate(dot(rayDirection, sd.N));
            float3 F = evalFresnelSchlick(simpleBRDF.specularReflection.albedo, 1, woDotN);
            newThroughput = throughput * F / pdfBrdf;
        }

        if (kUseRussianRoulette && depth >= 5)
        {
            float continuationProb = sqrt(saturate(max(newThroughput.x, max(newThroughput.y, newThroughput.z)) / max(throughput.x, max(throughput.y, throughput.z))));
            if (continuationProb < 1)
            {
                if (sampleNext1D(sg) >= continuationProb)
                {
                    break;
                }
                throughput = newThroughput / continuationProb;
            }
            else
            {
                throughput = newThroughput;
            }
        }
        else
        {
            throughput = newThroughput;
        }

        TriangleHit trianglehit;
        ShadingData nextSd;
        bool hit = traceScatterRay(rayOrigin, rayDirection, nextSd, trianglehit);
        if (hit)
        {
            if (kUseVirtualLights && !isSmooth)
            {
                float actualDis = distance(nextSd.posW, sd.posW);
                float estimatedNumVPLs = 0;

                float minDis = gShortDistance;
                float maxDis = gShortDistance + gShortDistanceRange;
                float blendFactor = gUseNewThresholds ? 1.f : saturate((actualDis - minDis) / gShortDistanceRange);
                bool isVirtualLightSampleValid = false;
                if (isDelta)
                {
                    blendFactor = 0.0f;
                }
                else if (gUseNewThresholds || actualDis >= minDis)
                {
                    if (kVirtualLightUseMIS || !kVirtualLightUseNEE)
                    {
                        float3 virtualLightRadiance = 0.0f;
                        RayDesc searchRay;
                        searchRay.Origin = nextSd.posW;
                        searchRay.Direction = float3(0.0f, 1.0f, 0.0f);
                        searchRay.TMin = 0.0f;
                        searchRay.TMax = 0.0f;
                        RayQuery<RAY_FLAG_NONE> rayQuery;
                        rayQuery.TraceRayInline(gVirtualLights.as, RAY_FLAG_NONE, 0xff, searchRay);

                        isVirtualLightSampleValid = true;
                        uint validCount = 0;

                        float avgRadius2 = 0.f;

                        for (uint i = 0; i < maxSearchIterCount; i++)
                        {
                            if(!rayQuery.Proceed())
                            {
                                break;
                            }
                        
                            if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
                            {
                                uint lightIndex = rayQuery.CandidatePrimitiveIndex();
                                float3 lightPosW = gVirtualLights.getPosition(lightIndex);
                                float radius = gVirtualLights.getRadius(lightIndex);
                                float disSquare = dot(nextSd.posW - lightPosW, nextSd.posW - lightPosW);
                                if(disSquare <= radius * radius)
                                {
                                    avgRadius2 += radius * radius;

                                    float3 L = normalize(nextSd.posW - sd.posW);
                                    float3 virtualLightN = gVirtualLights.getNormal(lightIndex);

#if _SHADING_NORMAL_CHECK
                                    if(!readRadianceCheckingNormal || dot(nextSd.N, virtualLightN) > 0.7f)
#endif
                                    {
#if _SHADING_MATERIAL_CHECK
                                        // const float roughnessThreshold = 0.5f;
                                        // const float reflectivityThreshold = 0.5f;
                                        // const float albedoThreshold = 0.5f;

                                        // if (!compareRelativeDifference(nextSd.linearRoughness, gVirtualLights.getRoughness(lightIndex), roughnessThreshold))
                                        // {
                                        //     continue;
                                        // }

                                        // if (!compareRelativeDifference(luminance(nextSd.specular), luminance(gVirtualLights.getSpecular(lightIndex)), reflectivityThreshold))
                                        // {
                                        //     continue;
                                        // }

                                        // if (!compareRelativeDifference(luminance(nextSd.diffuse), luminance(gVirtualLights.getDiffuse(lightIndex)), albedoThreshold))
                                        // {
                                        //     continue;
                                        // }
#endif

                                        validCount++;
                                        if (depth == 0 && kShadingMode == 1)
                                        {
                                            break;
                                        }

                                        float area = M_PI * radius * radius;
                                        float misWeight = 1.0f;
                                        if (kVirtualLightUseMIS && kVirtualLightUseNEE && !isDelta)
                                        {
                                            float lightSelectionPdf;
                                            if (gUseVirtualLightBVH)
                                            {
                                                lightSelectionPdf = gVirtualLightBVHSampler.evalVirtualLightPdf(sd.posW, sd.N, true, lightIndex);
                                            }
                                            else
                                            {
                                                lightSelectionPdf = gVirtualLightFluxTable.getWeight(lightIndex) / gVirtualLightFluxTable.weightSum;
                                            }
                                            float conePdf = gVirtualLights.evalConeSamplingPdf(lightIndex, sd.posW, L);
                                            misWeight = evalMultiSampleMISRobust(pdfBrdf, conePdf * lightSelectionPdf, 1, gNEESamples);
                                        }
                                        virtualLightRadiance += (misWeight * gVirtualLights.readLe(lightIndex, L, gSpecRadianceContainer) / area);
                                    }
                                }
                            }
                        }


                        if (validCount == 0)
                        {
                            isVirtualLightSampleValid = false;
                        }
                        else if (gUseNewThresholds)
                        {
                            avgRadius2 /= validCount;
                            estimatedNumVPLs = (actualDis * actualDis / ((kUseMixtureBSDF ? pdfBrdf : pdfSubBRDF) * abs(dot(nextSd.N, -rayDirection)))) / avgRadius2;
                            blendFactor = saturate(float(estimatedNumVPLs - gShortDistanceNumVSLs) / (gLongDistanceNumVSLs - gShortDistanceNumVSLs));
                        }
                        
                        if (isVirtualLightSampleValid)
                        {
                            accumulateRadiance(color, blendFactor * throughput * virtualLightRadiance, gMaxPathIntensity);
                        }
                        else
                        {
                            blendFactor = 0.0f;
                        }
                    }
                }

                bool continueTrace = true;
                if ((gUseNewThresholds && estimatedNumVPLs > gLongDistanceNumVSLs || !gUseNewThresholds && actualDis > maxDis) && !isDelta && isVirtualLightSampleValid)
                {
                    continueTrace = false;
                }

                if (kUseMIS || !kUseNEE)
                {
                    float misWeight = 1.0f;
                    if (kUseMIS && kUseNEE && kUseEmissiveLights && !isDelta)
                    {
                        misWeight = evalMISRobust(pdfBrdf, gEmissiveLightSampler.evalPdf(sd.posW, sd.N, true, trianglehit));
                    }
                    accumulateRadiance(color, misWeight * throughput * nextSd.emissive, depth == 0 ? 1e30f : gMaxPathIntensity);
                }

                if (!continueTrace)
                {
                    break;    
                }

                throughput = (1.0f - blendFactor) * throughput;
                if (kOutputDepth)
                {
                    depthThp = (1.0f - blendFactor) * depthThp;
                }
            }
            else 
            {
                if (kUseMIS || !kUseNEE)
                {
                    float misWeight = 1.0f;
                    if (kUseMIS && kUseEmissiveLights && !isDelta)
                    {
                        misWeight = evalMISRobust(pdfBrdf, gEmissiveLightSampler.evalPdf(sd.posW, sd.N, true, trianglehit));
                    }
                    accumulateRadiance(color, misWeight * throughput * nextSd.emissive, 1e30f);
                }
            }
            rayOrigin = nextSd.computeNewRayOrigin();
            simpleBRDF.setup(nextSd);
            sd = nextSd;
        }
        else 
        {
            if (kUseEnvLight)
            {
                float3 Le = gScene.envMap.eval(rayDirection);
                accumulateRadiance(color, throughput * Le, gMaxPathIntensity);
            }

            break;
        }
    }

    if (kOutputDepth)
    {
        float3 goodColor = pow(float3(95.0/255.0, 1.0/255.0, 116.0/255.0), 2.2f);
        float3 badColor = pow(float3(87.0/255.0, 252.0/255.0, 157.0/255.0), 2.2f);
        uint legendRangeMax = 5;
        color = lerp(goodColor, badColor, (depthVal - 1.0f) / float(legendRangeMax));
    }

    return color;
}


[numthreads(16, 16, 1)]
void main(uint2 GlobalIndex : SV_DispatchThreadID, uint3 groupIndex: SV_GROUPID)
{
    if (any(GlobalIndex.xy >= gViewportDims)) 
    {
        return;
    }

    float3 res = UnifiedShading(GlobalIndex);
    gShadingOutput[GlobalIndex] = float4(res, 1.0);
}
