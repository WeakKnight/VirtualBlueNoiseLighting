/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "NRD.hlsli"
#include "STL.hlsli"
#include "RELAX_Diffuse_SpatialVarianceEstimation.resources.hlsli"

NRD_DECLARE_CONSTANTS

#include "NRD_Common.hlsli"
NRD_DECLARE_SAMPLERS
#include "RELAX_Common.hlsli"

#define THREAD_GROUP_SIZE 16
#define SKIRT 2

NRD_DECLARE_INPUT_TEXTURES
NRD_DECLARE_OUTPUT_TEXTURES

groupshared float4 sharedDiffuseAnd2ndMoment[THREAD_GROUP_SIZE + SKIRT * 2][THREAD_GROUP_SIZE + SKIRT * 2];
groupshared float4 sharedNormalViewZ[THREAD_GROUP_SIZE + SKIRT * 2][THREAD_GROUP_SIZE + SKIRT * 2];

float computeDepthWeight(float depthCenter, float depthP, float phiDepth)
{
    return 1;
}

float computeNormalWeight(float3 normalCenter, float3 normalP, float phiNormal)
{
    return phiNormal == 0.0f ? 1.0f : pow(saturate(dot(normalCenter, normalP)), phiNormal);
}

[numthreads(THREAD_GROUP_SIZE, THREAD_GROUP_SIZE, 1)]
NRD_EXPORT void NRD_CS_MAIN(uint3 dispatchThreadId : SV_DispatchThreadId, uint3 groupThreadId : SV_GroupThreadId, uint3 groupId : SV_GroupId)
{
    const int2 ipos = dispatchThreadId.xy;

    // Populating shared memory
    uint linearThreadIndex = groupThreadId.y * THREAD_GROUP_SIZE + groupThreadId.x;
    uint newIdxX = linearThreadIndex % (THREAD_GROUP_SIZE + SKIRT * 2);
    uint newIdxY = linearThreadIndex / (THREAD_GROUP_SIZE + SKIRT * 2);

    uint blockXStart = groupId.x * THREAD_GROUP_SIZE;
    uint blockYStart = groupId.y * THREAD_GROUP_SIZE;

    // First stage
    int ox = newIdxX;
    int oy = newIdxY;
    int xx = blockXStart + newIdxX - SKIRT;
    int yy = blockYStart + newIdxY - SKIRT;

    float4 diffuse = 0;
    float3 normal = 0;
    float viewZ = 0;

    if ((xx >= 0) && (yy >= 0) && (xx < gResolution.x) && (yy < gResolution.y))
    {
        diffuse = gDiffuseIllumination[int2(xx, yy)];
        normal = NRD_FrontEnd_UnpackNormalAndRoughness(gNormalRoughness[int2(xx, yy)]).rgb;
        viewZ = gViewZ[int2(xx, yy)] / NRD_FP16_VIEWZ_SCALE;
    }
    sharedDiffuseAnd2ndMoment[oy][ox] = diffuse;
    sharedNormalViewZ[oy][ox] = float4(normal, viewZ);

    // Second stage
    linearThreadIndex += THREAD_GROUP_SIZE * THREAD_GROUP_SIZE;
    newIdxX = linearThreadIndex % (THREAD_GROUP_SIZE + SKIRT * 2);
    newIdxY = linearThreadIndex / (THREAD_GROUP_SIZE + SKIRT * 2);

    ox = newIdxX;
    oy = newIdxY;
    xx = blockXStart + newIdxX - SKIRT;
    yy = blockYStart + newIdxY - SKIRT;

    diffuse = 0;
    normal = 0;
    viewZ = 0;

    if (linearThreadIndex < (THREAD_GROUP_SIZE + SKIRT * 2) * (THREAD_GROUP_SIZE + SKIRT * 2))
    {
        if ((xx >= 0) && (yy >= 0) && (xx < gResolution.x) && (yy < gResolution.y))
        {
            diffuse = gDiffuseIllumination[int2(xx, yy)];
            normal = NRD_FrontEnd_UnpackNormalAndRoughness(gNormalRoughness[int2(xx, yy)]).rgb;
            viewZ = gViewZ[int2(xx, yy)] / NRD_FP16_VIEWZ_SCALE;
        }
        sharedDiffuseAnd2ndMoment[oy][ox] = diffuse;
        sharedNormalViewZ[oy][ox] = float4(normal, viewZ);
    }

    // Ensuring all the writes to shared memory are done by now
    GroupMemoryBarrierWithGroupSync();

    //
    // Shared memory is populated now and can be used for filtering
    //

    int2 sharedMemoryCenterIndex = groupThreadId.xy + int2(SKIRT, SKIRT);

    // Repacking normal and roughness to prev normal roughness to be used in the next frame
    float4 normalRoughness = NRD_FrontEnd_UnpackNormalAndRoughness(gNormalRoughness[ipos]);
    gOutNormalRoughness[ipos] = PackPrevNormalRoughness(normalRoughness);

    // Using diffuse history length for spatial variance estimation
    float historyLength = 255.0 * gHistoryLength[ipos];

    float4 centerDiffuseAnd2ndMoment = sharedDiffuseAnd2ndMoment[sharedMemoryCenterIndex.y][sharedMemoryCenterIndex.x];
    float3 centerDiffuseIllumination = centerDiffuseAnd2ndMoment.rgb;
    float centerDiffuse1stMoment = STL::Color::Luminance(centerDiffuseIllumination);
    float centerDiffuse2ndMoment = centerDiffuseAnd2ndMoment.a;

    [branch]
    if (historyLength >= float(gHistoryThreshold))
    {
        // If we have enough temporal history available,
        // we pass illumination data unmodified
        // and calculate variance based on temporally accumulated moments
        float diffuseVariance = centerDiffuse2ndMoment - centerDiffuse1stMoment * centerDiffuse1stMoment;

        gOutDiffuseIlluminationAndVariance[ipos] = float4(centerDiffuseIllumination, diffuseVariance);
        return;
    }

    float4 centerNormalViewZ = sharedNormalViewZ[sharedMemoryCenterIndex.y][sharedMemoryCenterIndex.x];
    float3 centerNormal = centerNormalViewZ.xyz;
    float centerViewZ = centerNormalViewZ.a;

    // Early out if linearZ is beyond denoising range
    [branch]
    if (centerViewZ > gDenoisingRange)
    {
        return;
    }

    float sumWDiffuseIllumination = 0;
    float3 sumDiffuseIllumination = 0;

    float sumDiffuse1stMoment = 0;
    float sumDiffuse2ndMoment = 0;

    // Compute first and second moment spatially. This code also applies cross-bilateral
    // filtering on the input illumination.
    [unroll]
    for (int cy = -2; cy <= 2; cy++)
    {
        [unroll]
        for (int cx = -2; cx <= 2; cx++)
        {
            int2 sharedMemoryIndex = groupThreadId.xy + int2(SKIRT + cx, SKIRT + cy);

            // Fetching sample data
            float3 sampleNormal = sharedNormalViewZ[sharedMemoryIndex.y][sharedMemoryIndex.x].rgb;

            float4 sampleDiffuse = sharedDiffuseAnd2ndMoment[sharedMemoryIndex.y][sharedMemoryIndex.x];
            float3 sampleDiffuseIllumination = sampleDiffuse.rgb;
            float sampleDiffuse1stMoment = STL::Color::Luminance(sampleDiffuseIllumination);
            float sampleDiffuse2ndMoment = sampleDiffuse.a;


            // Calculating weights
            float depthW = 1.0;// TODO: should we take in account depth here?
            float normalW = computeNormalWeight(centerNormal, sampleNormal, gPhiNormal);
            float diffuseW = normalW * depthW;

            // Accumulating
            sumWDiffuseIllumination += diffuseW;
            sumDiffuseIllumination += sampleDiffuseIllumination.rgb * diffuseW;
            sumDiffuse1stMoment += sampleDiffuse1stMoment * diffuseW;
            sumDiffuse2ndMoment += sampleDiffuse2ndMoment * diffuseW;
        }
    }

    // Clamp sum to >0 to avoid NaNs.
    sumWDiffuseIllumination = max(sumWDiffuseIllumination, 1e-6f);

    sumDiffuseIllumination /= sumWDiffuseIllumination;
    sumDiffuse1stMoment /= sumWDiffuseIllumination;
    sumDiffuse2ndMoment /= sumWDiffuseIllumination;

    // compute variance using the first and second moments
    float diffuseVariance = abs(sumDiffuse2ndMoment - sumDiffuse1stMoment * sumDiffuse1stMoment);

    // give the variance a boost for the first frames
    float boost = max(1.0, 4.0 / (historyLength + 1.0));
    diffuseVariance *= boost;

    gOutDiffuseIlluminationAndVariance[ipos] = float4(sumDiffuseIllumination, diffuseVariance);
}
