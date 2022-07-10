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
#include "RELAX_DiffuseSpecular_DisocclusionFix.resources.hlsli"

NRD_DECLARE_CONSTANTS

#include "NRD_Common.hlsli"
NRD_DECLARE_SAMPLERS
#include "RELAX_Common.hlsli"

NRD_DECLARE_INPUT_TEXTURES
NRD_DECLARE_OUTPUT_TEXTURES

// Helper functions
float3 getCurrentWorldPos(int2 pixelPos, float depth)
{
    float2 uv = ((float2)pixelPos + float2(0.5, 0.5)) * gInvRectSize * 2.0 - 1.0;
    return depth * (gFrustumForward.xyz + gFrustumRight.xyz * uv.x - gFrustumUp.xyz * uv.y);
}

float getGeometryWeight(float3 centerWorldPos, float3 centerNormal, float3 sampleWorldPos, float centerLinearZ)
{
    float distanceToCenterPointPlane = abs(dot(sampleWorldPos - centerWorldPos, centerNormal));
    return (distanceToCenterPointPlane / (centerLinearZ + 1e-6) > gDisocclusionThreshold) ? 0.0 : 1.0;
}

float getDiffuseNormalWeight(float3 centerNormal, float3 pointNormal)
{
    return pow(max(0.01, dot(centerNormal, pointNormal)), max(gDisocclusionFixEdgeStoppingNormalPower, 0.01));
}

float GetRadius(float numFramesInHistory)
{
    return gMaxRadius / (numFramesInHistory + 1.0);
}

float getSpecularLobeHalfAngle(float roughness)
{
    // Defines a cone angle, where micro-normals are distributed
    float r2 = roughness * roughness;
    float r3 = roughness * r2;
    return 3.141592 * r2 / (1.0 + 0.5 * r2 + r3);
}

float2 getNormalWeightParams(float roughness, float numFramesInHistory)
{
    // This is the main parameter - cone angle
    float angle = 0.33 * getSpecularLobeHalfAngle(roughness);
    return float2(angle, 1.0);
}

float getSpecularRWeight(float2 params0, float3 v0, float3 v)
{
    float cosa = saturate(dot(v0, v));
    float a = STL::Math::AcosApprox(cosa);
    a = 1.0 - STL::Math::SmoothStep(0.0, params0.x, a);
    return saturate(1.0 + (a - 1.0) * params0.y);
}

//
// Main
//

[numthreads(8, 8, 1)]
NRD_EXPORT void NRD_CS_MAIN(uint3 dispatchThreadId : SV_DispatchThreadId)
{
    const int2 ipos = int2(dispatchThreadId.xy);

    float centerViewZ = gViewZFP16[ipos] / NRD_FP16_VIEWZ_SCALE;

    // Early out if linearZ is beyond denoising range
    [branch]
    if (centerViewZ > gDenoisingRange)
    {
        return;
    }

    float historyLength = 255.0 * gSpecularAndDiffuseHistoryLength[ipos].g; // Using diffuse history length to control disocclusion fix
    float4 specularIlluminationAnd2ndMoment = gSpecularIllumination[ipos];
    float4 diffuseIlluminationAnd2ndMoment = gDiffuseIllumination[ipos];
    float4 specularIlluminationResponsive = gSpecularIlluminationResponsive[ipos];
    float4 diffuseIlluminationResponsive = gDiffuseIlluminationResponsive[ipos];

    // Pass through the input data if no disocclusion detected
    [branch]
    if (historyLength > gFramesToFix)
    {
        gOutSpecularIllumination[ipos] = specularIlluminationAnd2ndMoment;
        gOutDiffuseIllumination[ipos] = diffuseIlluminationAnd2ndMoment;
        gOutSpecularIlluminationResponsive[ipos] = specularIlluminationResponsive;
        gOutDiffuseIlluminationResponsive[ipos] = diffuseIlluminationResponsive;
        return;
    }

    // Unpacking the rest of center data
    float4 centerNormalRoughness = NRD_FrontEnd_UnpackNormalAndRoughness(gNormalRoughness[ipos]);
    float3 centerNormal = centerNormalRoughness.rgb;
    float centerRoughness = centerNormalRoughness.a;
    float3 centerWorldPos = getCurrentWorldPos(ipos, centerViewZ);
    float3 centerV = normalize(centerWorldPos);
    float3 centerR = reflect(centerV, centerNormal);
    float2 normalWeightParams = getNormalWeightParams(centerRoughness, historyLength);

    // Running sparse cross-bilateral filter
    float4 specularIlluminationAnd2ndMomentSum = specularIlluminationAnd2ndMoment;
    float4 diffuseIlluminationAnd2ndMomentSum = diffuseIlluminationAnd2ndMoment;

    float diffuseWSum = 1;
    float specularWSum = 1;

    float r = GetRadius(historyLength);

    [unroll]
    for (int j = -2; j <= 2; j++)
    [unroll]
    for (int i = -2; i <= 2; i++)
    {
        int dx = (int)(i * r);
        int dy = (int)(j * r);

        int2 samplePosInt = (int2)ipos + int2(dx, dy);

        bool isInside = all(samplePosInt >= int2(0, 0)) && all(samplePosInt < gResolution);
        if ((i == 0) && (j == 0)) continue;

        float3 sampleNormal = NRD_FrontEnd_UnpackNormalAndRoughness(gNormalRoughness[samplePosInt]).rgb;

        float sampleViewZ = gViewZFP16[samplePosInt] / NRD_FP16_VIEWZ_SCALE;

        // Edge stopping functions:
        // ..normal
        float diffuseW = getDiffuseNormalWeight(centerNormal, sampleNormal);

        // ..geometry
        float3 sampleWorldPos = getCurrentWorldPos(samplePosInt, sampleViewZ);
        float geometryWeight = getGeometryWeight(centerWorldPos, centerNormal, sampleWorldPos, centerViewZ);
        float specularW = geometryWeight;
        diffuseW *= geometryWeight;
        diffuseW *= isInside ? 1.0 : 0;

        // ..specular lobe
        float3 sampleV = normalize(sampleWorldPos);
        float3 sampleR = reflect(sampleV, sampleNormal);
        specularW *= getSpecularRWeight(normalWeightParams, sampleR, centerR);
        specularW *= isInside ? 1.0 : 0;

        // Summing up diffuse result
        if (diffuseW > 1e-4)
        {
            float4 sampleDiffuseIlluminationAnd2ndMoment = gDiffuseIllumination[samplePosInt];
            diffuseIlluminationAnd2ndMomentSum += sampleDiffuseIlluminationAnd2ndMoment * diffuseW;
            diffuseWSum += diffuseW;
        }

        // Summing up specular result
        if (specularW > 1e-4)
        {
            float4 sampleSpecularIlluminationAnd2ndMoment = gSpecularIllumination[samplePosInt];
            specularIlluminationAnd2ndMomentSum += sampleSpecularIlluminationAnd2ndMoment * specularW;
            specularWSum += specularW;
        }
    }

    float4 outSpecularIlluminationAnd2ndMoment = specularIlluminationAnd2ndMomentSum / specularWSum;
    float4 outDiffuseIlluminationAnd2ndMoment = diffuseIlluminationAnd2ndMomentSum / diffuseWSum;

    // Writing out the results
    gOutSpecularIllumination[ipos] = outSpecularIlluminationAnd2ndMoment;
    gOutDiffuseIllumination[ipos] = outDiffuseIlluminationAnd2ndMoment;
    gOutSpecularIlluminationResponsive[ipos] = outSpecularIlluminationAnd2ndMoment;
    gOutDiffuseIlluminationResponsive[ipos] = outDiffuseIlluminationAnd2ndMoment;
}
