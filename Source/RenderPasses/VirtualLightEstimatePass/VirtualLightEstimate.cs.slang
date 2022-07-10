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
import Scene.RaytracingInlineHelper;
import Utils.VirtualLight.VirtualLightContainer;
import Utils.VirtualLight.MegaTextureContainer;
import Utils.Sampling.AliasTable;
import Experimental.Scene.Material.MaterialShading;
import Experimental.Scene.Material.Microfacet;
import Experimental.Scene.Material.Fresnel;
import Scene.Material.SimpleBRDF;

/*
Macros:
_PER_FRAME_PATH_COUNT
_INV_PER_FRAME_PATH_COUNT
*/
#define NEIGHBOUR_SEARCH_NORMAL_TOLERANCE 0.1
static const uint kInvalidIndex = 0xffffffff;

static const bool kConvertIncomingToOutgoing = (_CONVERT_INCOMING_TO_OUTGOING == 1);
static const uint kPerFramePathCount = _PER_FRAME_PATH_COUNT;
static const bool kUseRussianRoulette = true;
static const uint kBounceNum = _BOUNCE_NUM;

cbuffer CB
{
    uint gFrameIndex;
    uint gCurrentPathIndex;
    uint gSeedCapacity;
    float gInvPathCount;
    VirtualLightContainer gVirtualLightContainer;
    MegaTextureContainer gSpecRadianceContainer;
    AliasTable gEmissiveTriTable;
};

StructuredBuffer<uint> gSeedBuffer;
RWByteAddressBuffer gFluxBuffer;

void interlockAddFlux(uint index, float flux)
{
    uint address = index * 4u;
    NvInterlockedAddFp32(gFluxBuffer, address, flux);
}

float calcFlux(float3 color)
{
    float flux = (color.r + color.g + color.b) / 3.0f;
    return flux;
}

void radianceTransfer(uint lightIndex, float3 vplNormal, float3 L, float3 Le, float weight)
{
    ShadingData virtualLightSd = gVirtualLightContainer.getShadingData(lightIndex);
    float3 wi = toLocal(L, vplNormal);

    float flux = 0.0;
    
    SimpleBRDF vplBrdf;
    vplBrdf.setup(virtualLightSd);

    const uint itemSize = gSpecRadianceContainer.perItemSizeHQ;
    const float stepSize = 1.0f / itemSize;

    uint type = gVirtualLightContainer.getType(lightIndex);

#if USE_SECOND_DIFFUSE_BRDF == 1
    if (type == 0)
    // Diffuse Estimation
#endif
    {
        float3 diffuse = weight * Le * vplBrdf.diffuseReflection.evalFr(0.0f, wi);
        gVirtualLightContainer.interlockAddDiffuseRadiance(lightIndex, diffuse);
        // We need multiply 2.0 * pi here because we integrate over solid angle
        flux += calcFlux(M_PI * diffuse * gSpecRadianceContainer.perItemSizeHQ * gSpecRadianceContainer.perItemSizeHQ);
    }

    // Specular Estimation
    if (type != 0)
    {
        uint itemSize;
        if (type == 3)
        {
            itemSize = gSpecRadianceContainer.perItemSizeHQ;
        }
        else if (type == 1)
        {
            itemSize = gSpecRadianceContainer.perItemSizeLQ;
        }
        const float stepSize = 1.0f / itemSize;
        uint texIndex = gVirtualLightContainer.getIndex(lightIndex);

        if (kConvertIncomingToOutgoing)
        {
            float3 weightedLe = weight * Le;
            if (type == 3)
            {
                gSpecRadianceContainer.interlockWritePhotonHQ(texIndex, wi, weightedLe);
            }
            else if (type == 1)
            {
                gSpecRadianceContainer.interlockWritePhotonLQ(texIndex, wi, weightedLe);
            }
        }
        else
        {
            for(int u = 0; u < itemSize; u++)
            {
                for(int v = 0; v < itemSize; v++)
                {
                    float3 specular = 0.0f;
                    float3 subFlux = 0.0f;
                    for(uint k = 0; k < 4; k++)
                    {
                        float3 wo = UV2HemisphereFloat(float2(u, v) + (plastic(k) - 0.5), stepSize);
#if USE_SECOND_DIFFUSE_BRDF == 1
                        float3 brdfValue = vplBrdf.evalFrWIthoutCheck(wo, wi);
#else
                        float3 brdfValue = vplBrdf.specularReflection.evalFr(wo, wi);
#endif
                        specular += 0.25f *  Le * brdfValue * weight;
                        subFlux += 2.0 * M_PI * 0.25f * saturate(wo.z) * specular; 
                    }

                    if (type == 3)
                    {
                        gSpecRadianceContainer.interlockAddColorHQ(texIndex, uint2(u, v), specular);
                        flux += calcFlux(subFlux);
                    }
                    else if (type == 1)
                    {
                        gSpecRadianceContainer.interlockAddColorLQ(texIndex, uint2(u, v), specular);
                        flux += ((gSpecRadianceContainer.perItemSizeHQ / gSpecRadianceContainer.perItemSizeLQ) * (gSpecRadianceContainer.perItemSizeHQ / gSpecRadianceContainer.perItemSizeLQ) * calcFlux(subFlux));
                    }
                }
            }
        }
    }

    interlockAddFlux(lightIndex, flux);
}

[numthreads(256, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if (launchIndex.x >= kPerFramePathCount) 
    {
        return;
    }

    uint seedIndex = (uint)((gCurrentPathIndex + launchIndex.x) / 2000) % gSeedCapacity;
    uint seed = gSeedBuffer[seedIndex];
    SampleGenerator sg = SampleGenerator.create(uint2(gCurrentPathIndex + launchIndex.x, seed), gFrameIndex);
    
    uint triIndex = gEmissiveTriTable.sample(sampleNext2D(sg));
    float triPdf = gEmissiveTriTable.getWeight(triIndex) / gEmissiveTriTable.weightSum;

    EmissiveTriangle emissiveTri = gScene.lightCollection.getTriangle(triIndex);
    float samplePdf = triPdf / emissiveTri.area;
    float3 barycentric = sample_triangle(sampleNext2D(sg));

    float3 samplePos = emissiveTri.getPosition(barycentric);
    float2 sampleUV = emissiveTri.getTexCoord(barycentric);
    float3 Le = gScene.getEmissive(emissiveTri.materialID, sampleUV, gScene.materialResources[emissiveTri.materialID]);
    Le = Le / samplePdf * gInvPathCount;

    float3 rayDir = cosineWeightedSampling(sampleNext2D(sg), emissiveTri.normal);
    float3 rayOrigin = computeRayOrigin(samplePos, emissiveTri.normal);
    Le = Le * M_PI ;
    
    [loop]
    for (int bounce = 0; bounce < kBounceNum; bounce++)
    {
        ShadingData sd;
        if (!traceScene(rayOrigin, rayDir, sd))
        {
            break;
        }

        RayDesc searchRay;
        searchRay.Origin = sd.posW;
        searchRay.Direction = float3(0.0f, 1.0f, 0.0f);
        searchRay.TMin = 0.0f;
        searchRay.TMax = 0.00001f;
        RayQuery<RAY_FLAG_NONE> rayQuery;

        float3 L = -rayDir;

#ifdef _ESTIMATION_FORMER_VERSION
        float searchRadius = gVirtualLightContainer.boundingBoxRadius;

        float totalWeight = 0.0f;
        {
            rayQuery.TraceRayInline(
                gVirtualLightContainer.as,
                RAY_FLAG_NONE,
                0xff,
                searchRay);

            while (rayQuery.Proceed())
            {
                if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
                {
                    uint AABBIndex = rayQuery.CandidatePrimitiveIndex();
                    PackedBoundingBox boundingBox = gVirtualLightContainer.boundingBoxBuffer[AABBIndex];
                    float3 vplOrigin = (boundingBox.minPoint + boundingBox.maxPoint) * 0.5f;
                    float dis = sqrt(dot(sd.posW - vplOrigin, sd.posW - vplOrigin));
                    // More Test
                    if(dis <= searchRadius)
                    {
                        float3 vplNormal = gVirtualLightContainer.getNormal(AABBIndex);
                        float3 disVec = sd.N - vplNormal;
                        if(dot(disVec, disVec) < NEIGHBOUR_SEARCH_NORMAL_TOLERANCE)
                        {
                            totalWeight = totalWeight + (1.0f - dis * dis / (searchRadius * searchRadius));
                        }
                    }
                }
            }
        }

        {
            rayQuery.TraceRayInline(
                gVirtualLightContainer.as,
                RAY_FLAG_NONE,
                0xff,
                searchRay);

            while (rayQuery.Proceed())
            {
                uint AABBIndex = rayQuery.CandidatePrimitiveIndex();
                PackedBoundingBox boundingBox = gVirtualLightContainer.boundingBoxBuffer[AABBIndex];
                float3 vplOrigin = (boundingBox.minPoint + boundingBox.maxPoint) * 0.5f;
                float dis = sqrt(dot(sd.posW - vplOrigin, sd.posW - vplOrigin));

                if(dis <= searchRadius)
                {   
                    float3 vplNormal = gVirtualLightContainer.getNormal(AABBIndex);
                    float3 disVec = sd.N - vplNormal;
                    if(dot(disVec, disVec) < NEIGHBOUR_SEARCH_NORMAL_TOLERANCE)
                    {
                        float weight = 1.0f - dis * dis / (searchRadius * searchRadius);
                        weight = weight / totalWeight;
                        
                        radianceTransfer(AABBIndex, vplNormal, L, Le, weight);
                    }
                }
            }
        }
#else
        /*
        if intersect more than 8 times, clamp
        */
        uint intersectIndicis[8] = {kInvalidIndex, kInvalidIndex, kInvalidIndex, kInvalidIndex, kInvalidIndex, kInvalidIndex, kInvalidIndex, kInvalidIndex};
        uint validIntersectionNum = 0u;
        rayQuery.TraceRayInline(gVirtualLightContainer.as, RAY_FLAG_NONE, 0xff, searchRay);
        float maxDisSquare = 0.0f;

        while(rayQuery.Proceed())
        {
            if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
            {
                uint lightIndex = rayQuery.CandidatePrimitiveIndex();
                float3 posW = gVirtualLightContainer.getPosition(lightIndex);
                float radius = gVirtualLightContainer.getRadius(lightIndex);
                float disSquare = dot(sd.posW - posW, sd.posW - posW);
                if(disSquare <= radius * radius)
                {
                    float3 virtualLightN = gVirtualLightContainer.getNormal(lightIndex);
                    if(dot(sd.N, virtualLightN) > 0.7f && dot(virtualLightN, L) > 1e-6f)
                    {
                        // const float roughnessThreshold = 0.5f;
                        // const float reflectivityThreshold = 0.5f;
                        // const float albedoThreshold = 0.5f;

                        // if (!compareRelativeDifference(sd.linearRoughness, gVirtualLightContainer.getRoughness(lightIndex), roughnessThreshold))
                        // {
                        //     continue;
                        // }

                        // if (!compareRelativeDifference(luminance(sd.specular), luminance(gVirtualLightContainer.getSpecular(lightIndex)), reflectivityThreshold))
                        // {
                        //     continue;
                        // }

                        // if (!compareRelativeDifference(luminance(sd.diffuse), luminance(gVirtualLightContainer.getDiffuse(lightIndex)), albedoThreshold))
                        // {
                        //     continue;
                        // }

                        intersectIndicis[validIntersectionNum] = lightIndex;
                        validIntersectionNum++;
                        maxDisSquare = max(maxDisSquare, disSquare);

                        if (validIntersectionNum == 8)
                        {
                            rayQuery.Abort();
                        }
                    }
                }
            }
        }

        /*
        Add a little to incorporate the farthest virtual light
        */
        maxDisSquare = min(maxDisSquare * 1.05, maxDisSquare + 0.001);
        // maxDisSquare = maxDisSquare + 0.001f;

        /*
        Compute Total Weight
        */
        float weightSum = 0.0f;
        for (uint i = 0; i < validIntersectionNum; i++)
        {
            uint index = intersectIndicis[i];
            float3 posW = gVirtualLightContainer.getPosition(index);
            float disSquare = dot(sd.posW - posW, sd.posW - posW);
            weightSum = weightSum + (1.0f - disSquare / (maxDisSquare));
        }

        /*
        Do Radiance Transfer
        */
        for (uint i = 0; i < validIntersectionNum; i++)
        {
            uint lightIndex = intersectIndicis[i];

            float3 posW = gVirtualLightContainer.getPosition(lightIndex);
            float3 vplNormal = gVirtualLightContainer.getNormal(lightIndex);
            float disSquare = dot(sd.posW - posW, sd.posW - posW);
            float weight = (1.0f - disSquare / (maxDisSquare)) / weightSum;

            radianceTransfer(lightIndex, vplNormal, L, Le, weight);
        }
#endif
        /*
        Continue Tracing
        */
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

        if (kUseRussianRoulette && bounce >= 5)
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
