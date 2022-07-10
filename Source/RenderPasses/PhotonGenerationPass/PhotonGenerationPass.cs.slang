import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Lights.Lights;
import Scene.Shading;
import Experimental.Scene.Lights.LightCollection;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;
import Utils.VirtualLight.VirtualLightContainer;
import Scene.RaytracingInlineHelper;
import Scene.Material.SimpleBRDF;
import Utils.Sampling.AliasTable;
import Experimental.Scene.Material.Fresnel;

static const bool kUseRussianRoulette = true;
static const uint kBounceNum = _BOUNCE_NUM;

cbuffer CB
{
    uint gFrameIndex;
    uint gPhotonPathCount;
    VirtualLightContainer gPhotons;
    AliasTable gEmissiveTriTable;
};

[numthreads(256, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
     if (launchIndex.x >= gPhotonPathCount) 
    {
        return;
    }

    SampleGenerator sg = SampleGenerator.create(uint2(launchIndex.x, 1), gFrameIndex);
    uint triIndex = gEmissiveTriTable.sample(sampleNext2D(sg));
    float triPdf = gEmissiveTriTable.getWeight(triIndex) / gEmissiveTriTable.weightSum;

    EmissiveTriangle emissiveTri = gScene.lightCollection.getTriangle(triIndex);
    float samplePdf = triPdf / emissiveTri.area;
    float3 barycentric = sample_triangle(sampleNext2D(sg));

    float3 samplePos = emissiveTri.getPosition(barycentric);
    float2 sampleUV = emissiveTri.getTexCoord(barycentric);
    float3 Le = gScene.getEmissive(emissiveTri.materialID, sampleUV, gScene.materialResources[emissiveTri.materialID]);
    float invPathCount = 1.0 / gPhotonPathCount;
    Le = Le / samplePdf * invPathCount;

    float3 rayDir = cosineWeightedSampling(sampleNext2D(sg), emissiveTri.normal);
    float3 rayOrigin = computeRayOrigin(samplePos, emissiveTri.normal);
    Le = Le * M_PI ;

    [loop]
    for (uint i = 0; i < kBounceNum; i++)
    {
        ShadingData sd;
        bool hit = traceScene(rayOrigin, rayDir, sd);
        if (!hit)
        {
            break;
        }

        uint lightIndex = gPhotons.pushLight(sd);
        gPhotons.directionBuffer[lightIndex] = -rayDir;
        gPhotons.incidentRadianceBuffer[lightIndex] = Le;

        SimpleBRDF brdf;
        brdf.setup(sd);

        float3 V;
        float pdf;
        uint lobeType;
        bool valid = brdf.sample(sd.N, sd.V, V, pdf, lobeType, sg);
        if(!valid || dot(V, sd.faceN) < 1e-6f || dot(sd.V, sd.N) < 1e-6f)
        {
            break;
        }

        float3 wi = toLocal(sd.V, sd.N);
        float3 wo = toLocal(V, sd.N);
        
        float3 newLe;
        if(lobeType != 3)
        {
            newLe = Le * brdf.evalFr(wo, wi) * saturate(wo.z) / pdf;
        }
        else
        {
            float3 H = (sd.V + V) * 0.5;
            float LDotH = saturate(dot(sd.V, H));
            float3 F = evalFresnelSchlick(brdf.specularReflection.albedo, 1, LDotH);
            newLe = Le * F / pdf;
        }

        if (kUseRussianRoulette && i >= 5)
        {
            float continuationProb = sqrt(saturate(max(newLe.x, max(newLe.y, newLe.z)) / max(Le.x, max(Le.y, Le.z))));
            if (continuationProb < 1)
            {
                if (sampleNext1D(sg) >= continuationProb)
                {
                    break;
                }
                Le = newLe / continuationProb;
            }
            else
            {
                Le = newLe;
            }
        }
        else
        {
            Le = newLe;
        }
        
        rayOrigin = computeRayOrigin(sd.posW, sd.faceN);    
        rayDir = V;
    }
}