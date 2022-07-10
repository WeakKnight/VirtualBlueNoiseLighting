#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Helpers;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
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
import Types;
import Utils.Math.PackedFormats;

static const bool kUseRussianRoulette = true;
static const bool kUseEnvLight = USE_ENV_LIGHT;
static const bool kUseAnalyticLights = USE_ANALYTIC_LIGHTS;
static const bool kUseEmissiveLights = USE_EMISSIVE_LIGHTS;
static const uint kBounceNum = _BOUNCE_NUM;

static const bool kUseNEE = true;
static const bool kUseMIS = true;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    GBuffer gGBuffer;
    EmissiveLightSampler gEmissiveLightSampler;
    EnvMapSampler gEnvMapSampler;
    float gMaxPathIntensity;
    float gPhotonRadius;
    bool gOutputOutgoingRadiance;
}

StructuredBuffer<float3> gPhotonPositions;
StructuredBuffer<uint> gPhotonNormals;
StructuredBuffer<uint> gPhotonDirections;
StructuredBuffer<uint> gPhotonPowers;

Photon readPhoton(uint index)
{
    Photon result;
    result.position = gPhotonPositions[index];
    result.normal = decodeNormal2x16(gPhotonNormals[index]);
    result.direction = decodeNormal2x16(gPhotonDirections[index]);
    result.power = decodeLogLuvHDR(gPhotonPowers[index]);
    return result;
}

RaytracingAccelerationStructure gPhotonAS;
RWTexture2D<float4> gShadingOutput;
StructuredBuffer<PackedBoundingBox> gBoundingBoxBuffer;

void accumulateRadiance(inout float3 TotalRadiance, float3 PathRadiance)
{
	if (gMaxPathIntensity > 0)
	{
		// User asked for path contributions to be clamped to reduce fireflies.
		// Depending on how aggressive this value is, the image could be quite biased
		TotalRadiance += min(PathRadiance, gMaxPathIntensity);
	}
	else
	{
		// Just average values directly
		TotalRadiance += PathRadiance;
	}
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

static const bool kUsePhotonMap = true;

#if 0
bool GatherPhotonEnergy(ShadingData sd, SimpleBRDF simpleBRDF, out float3 color)
{
    color = 0.0f;
    RayDesc searchRay;
    searchRay.Origin = sd.posW;
    searchRay.Direction = float3(0.0f, 1.0f, 0.0f);
    searchRay.TMin = 0.0f;
    searchRay.TMax = 0.0f;
    RayQuery<RAY_FLAG_NONE> rayQuery;
    rayQuery.TraceRayInline(gPhotonAS, RAY_FLAG_NONE, 0xff, searchRay);

    // Need Density Estimation
    uint validIntersectionNum = 0u;
    float globalRadiusSquared = gPhotonRadius * gPhotonRadius;
    float maxDisSquared = 0.0f;
    while(rayQuery.Proceed())
    {
        if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
        {
            uint photonIndex = rayQuery.CandidatePrimitiveIndex();
            Photon photon = readPhoton(photonIndex);
            float disSquared = dot(photon.position - sd.posW, photon.position - sd.posW);
            if (disSquared < globalRadiusSquared)
            {
                if (dot(sd.N, photon.normal) > 0.7f)
                {
                    validIntersectionNum++;
                    maxDisSquared = max(maxDisSquared, disSquared);
                    float3 wo = toLocal(sd.V, sd.N);
                    float3 wi = toLocal(photon.direction, sd.N);
                    color += photon.power * simpleBRDF.evalFr(wo, wi);
                }
            }
        }
    }

    float normalziationTerm = 1.0f / (validIntersectionNum * M_PI * maxDisSquared);
    color = normalziationTerm * color;
    return validIntersectionNum > 0;
}
#else
bool GatherPhotonEnergy(ShadingData sd, SimpleBRDF simpleBRDF, out float3 color)
{
    color = 0.0f;
    uint photonNum = 0;
    RayDesc searchRay;
    searchRay.Origin = sd.posW;
    searchRay.Direction = float3(0.0f, 1.0f, 0.0f);
    searchRay.TMax = 0.0f;
    RayQuery<RAY_FLAG_NONE> rayQuery;
    rayQuery.TraceRayInline(gPhotonAS, RAY_FLAG_NONE, 0xff, searchRay);
    while(rayQuery.Proceed())
    {
        if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
        {
            uint photonIndex = rayQuery.CandidatePrimitiveIndex();
            Photon photon = readPhoton(photonIndex);
            float radius = 0.5f * (gBoundingBoxBuffer[photonIndex].maxPoint.x - gBoundingBoxBuffer[photonIndex].minPoint.x);
            float disSquared = dot(photon.position - sd.posW, photon.position - sd.posW);
            if (disSquared < (radius * radius))
            {
                if (dot(sd.N, photon.normal) > 0.7f && dot(photon.direction, sd.N) > 1e-6f)
                {
                    photonNum++;
                    float3 wo = toLocal(sd.V, sd.N);
                    float3 wi = toLocal(photon.direction, sd.N);
                    float invArea = 1.0f / (M_PI * radius * radius);
                    color += (invArea * photon.power * simpleBRDF.evalFr(wo, wi));
                }
            }
        }
    }
    
    return (photonNum > 0);
}
#endif

inline float3 ShadingWithPhotons(uint2 pixelPos)
{
    float3 color = 0.0f;
    SampleGenerator sg = SampleGenerator.create(pixelPos, gFrameIndex);
    ShadingData sd;
    if (!LoadShadingData(gGBuffer, pixelPos.xy, gViewportDims, gScene.camera, sd))
    {
        if (kUseEnvLight)
        {
            color = gScene.envMap.eval(gScene.camera.computeRayPinhole(pixelPos.xy, gViewportDims).dir);
        }
        return color;
    }

    SimpleBRDF simpleBRDF;
    simpleBRDF.setup(sd);

    if (gOutputOutgoingRadiance)
    {
        float3 photonEnergy;
        GatherPhotonEnergy(sd, simpleBRDF, photonEnergy);
        return photonEnergy;
    }

    color += sd.emissive;

    float3 throughput = 1.0f;
    float3 rayOrigin = sd.computeNewRayOrigin();
    float3 rayDirection;
    for (uint depth = 0; depth < kBounceNum; depth++)
    {
        if (kUseNEE && kUseEmissiveLights)
        {
            accumulateRadiance(color, throughput * evalEmissiveLightingNEE(sd, sg, gEmissiveLightSampler));
        }

        float pdfBrdf;
        uint lobeType;
        bool valid = simpleBRDF.sample(sd, rayDirection, pdfBrdf, lobeType, sg);
        if (!valid || min(sd.NdotV, dot(rayDirection, sd.N)) < 1e-6f)
        {
            break;
        }

        float3 wo = toLocal(sd.V, sd.N);
        float3 wi = toLocal(rayDirection, sd.N);
        float3 newThroughput;
        if (lobeType != 3u)
        {
            newThroughput = throughput * simpleBRDF.eval(wo, wi) / pdfBrdf;
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
            if (kUseMIS || !kUseNEE)
            {
                float misWeight = 1.0f;
                if (kUseMIS && kUseEmissiveLights && lobeType != 3u)
                {
                    misWeight = evalMISRobust(pdfBrdf, gEmissiveLightSampler.evalPdf(sd.posW, sd.N, true, trianglehit));
                }
                accumulateRadiance(color, misWeight * throughput * nextSd.emissive);
            }

            SimpleBRDF nextBRDF;
            nextBRDF.setup(nextSd);
            // Can Do Final Gathering
            if (kUsePhotonMap && lobeType == 1u)
            {
                float3 photonEnergy;
                bool valid = GatherPhotonEnergy(nextSd, nextBRDF, photonEnergy);
                if (valid)
                {
                    accumulateRadiance(color, throughput * photonEnergy);
                }
                break;
            }

            rayOrigin = nextSd.computeNewRayOrigin();
            simpleBRDF = nextBRDF;
            sd = nextSd;
        }
        else
        {
            if (kUseEnvLight)
            {
                float3 Le = gScene.envMap.eval(rayDirection);
                accumulateRadiance(color, throughput * Le);
            }

            break;
        }
    }
    return color;
}

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    if (any(dispatchThreadId.xy >= gViewportDims)) 
    {
        return;
    }

    float3 res = ShadingWithPhotons(dispatchThreadId.xy);
    gShadingOutput[dispatchThreadId.xy] = float4(res, 1.0);
}
