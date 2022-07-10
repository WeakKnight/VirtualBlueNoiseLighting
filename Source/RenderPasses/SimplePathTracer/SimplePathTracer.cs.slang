import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Lights.Lights;
import Scene.Shading;
import Experimental.Scene.Lights.LightCollection;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Experimental.Scene.Lights.EnvMapSampler;
import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;
import Scene.Material.SimpleBRDF;
import Utils.Color.ColorHelpers;
import RenderPasses.GBufferUtils;
import Experimental.Scene.Material.Fresnel;

static const bool kUseRussianRoulette = true;
static const bool kUseEnvLight = USE_ENV_LIGHT;
static const bool kUseAnalyticLights = USE_ANALYTIC_LIGHTS;
static const bool kUseEmissiveLights = USE_EMISSIVE_LIGHTS;
static const bool kUseNEE = true;
static const bool kUseMIS = true;

static const uint kBounceNum = _BOUNCE_NUM;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    GBuffer gGBuffer;
    EmissiveLightSampler gEmissiveLightSampler;
    EnvMapSampler gEnvMapSampler;
    float gMaxPathIntensity;
};

RWTexture2D<float4> gOutput;

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

float3 evalEmissiveLighting(ShadingData sd, inout SampleGenerator sg, EmissiveLightSampler emissiveSampler)
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

        if (dot(L, sd.N) <= kMinCosTheta)
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

[numthreads(16, 16, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if (any(launchIndex.xy >= gViewportDims))
    {
        return;
    }

    SampleGenerator sg = SampleGenerator.create(launchIndex.xy, gFrameIndex);
    ShadingData sd;
    if (!LoadShadingData(gGBuffer, launchIndex.xy, gViewportDims, gScene.camera, sd))
    {
        if (kUseEnvLight)
        {
            gOutput[launchIndex.xy] = float4(gScene.envMap.eval(gScene.camera.computeRayPinhole(launchIndex.xy, gViewportDims).dir), 1.0);
        }
        else
        {
            gOutput[launchIndex.xy] = 0.0f;
        }
        return;
    }
    float3 color = 0.0f;
    color += sd.emissive;
    
    SimpleBRDF simpleBRDF;
    simpleBRDF.setup(sd);

    
    float3 throughput = 1.0f;
    uint lobeType = 0;
    float3 rayOrigin = sd.computeNewRayOrigin();
    float3 rayDirection;

    for (uint depth = 0; depth < kBounceNum; depth++)
    {
        if (kUseNEE && kUseEmissiveLights)
        {
            accumulateRadiance(color, throughput * evalEmissiveLighting(sd, sg, gEmissiveLightSampler));
        }

        float pdfBrdf;
        bool valid = simpleBRDF.sample(sd, rayDirection, pdfBrdf, lobeType, sg);
        if (!valid || min(sd.NdotV, dot(rayDirection, sd.N)) < 1e-6f)
        {
            break;
        }

        float3 wo = toLocal(sd.V, sd.N);
        float3 wi = toLocal(rayDirection, sd.N);
        float3 newThroughput;
        if (lobeType != 3)
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
        float3 misPosW = sd.posW;
        float3 misN = sd.N;
        bool hit = traceScatterRay(rayOrigin, rayDirection, sd, trianglehit);
        if (hit)
        {
            rayOrigin = sd.computeNewRayOrigin();
            simpleBRDF.setup(sd);
        }

        if (kUseMIS || !kUseNEE)
        {
            // handle emissive hit
            if (hit)
            {
                float misWeight = 1.0f;
                if (kUseMIS && kUseEmissiveLights && lobeType != 3)
                {
                    misWeight = evalMISRobust(pdfBrdf, gEmissiveLightSampler.evalPdf(misPosW, misN, true, trianglehit));
                }
                accumulateRadiance(color, misWeight * throughput * sd.emissive);
            }
            // handle infinite hit
            else if (kUseEnvLight)
            {
                float3 Le = gScene.envMap.eval(rayDirection);
                accumulateRadiance(color, throughput * Le);
            }
        }

        if (!hit)
        {
            break;
        }
    }

    gOutput[launchIndex.xy] = float4(color, 1.0);
}
