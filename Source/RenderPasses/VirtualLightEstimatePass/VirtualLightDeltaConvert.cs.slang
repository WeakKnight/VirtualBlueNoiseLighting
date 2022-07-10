import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;
import Utils.VirtualLight.VirtualLightContainer;
import Utils.VirtualLight.MegaTextureContainer;
import Scene.Material.SimpleBRDF;
import Scene.Shading;
import Experimental.Scene.Material.Microfacet;
import Experimental.Scene.Material.Fresnel;

static const bool kTreatHighlyGlossyAsDelta = _TREAT_HIGHLY_GLOSSY_AS_DELTA;
static const float kHighlyGlossyThreshold = _HIGHLY_GLOSSY_THRESHOLD;

cbuffer CB
{
    uint gCount;
    uint gProcessedCount;
    VirtualLightContainer gVirtualLights;
    MegaTextureContainer gIncidentContainer;
};

RWByteAddressBuffer gConvertedBuffer;
RWByteAddressBuffer gFluxBuffer;
RWByteAddressBuffer gSolidAngleLUTBuffer;

float calcFlux(float3 color)
{
    float flux = (color.r + color.g + color.b) / 3.0f;
    return flux;
}

void interlockAddFlux(uint index, float flux)
{
    uint address = index * 4u;
    NvInterlockedAddFp32(gFluxBuffer, address, flux);
}

float readSolidAngleWeight(uint2 uv, uint itemSize)
{
    uint address = (uv.y * itemSize + uv.x) * 4u;
    return asfloat(gSolidAngleLUTBuffer.Load(address));
}

[numthreads(1, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    uint texIndex = gProcessedCount + launchIndex.x;
    if (texIndex >= gCount)
    {
        return;
    }
    
    uint lightIndex = 0;
    if (TEXTURE_TYPE == 3)
    {
        lightIndex = gVirtualLights.getVirtualLightIndexHQ(texIndex);
    }
    else if (TEXTURE_TYPE == 1)
    {
        lightIndex = gVirtualLights.getVirtualLightIndexLQ(texIndex);
    }
    
    ShadingData virtualLightSd = gVirtualLights.getShadingData(lightIndex);
    const uint perItemSize = TEXTURE_PER_ITEM_SIZE;
    const float stepSize = 1.0f / (float)perItemSize;

    if (kTreatHighlyGlossyAsDelta)
    {
        if (virtualLightSd.linearRoughness >= kHighlyGlossyThreshold)
        {
            return;
        }
        virtualLightSd.linearRoughness = 0.0f;
    }
    else
    {
        if (virtualLightSd.linearRoughness > 0.08f)
        {
            return;
        }
    }

    float totalFlux = 0.0f;

    // if (virtualLightSd.linearRoughness > 0.08f)
    // {
    //     const uint sampleCount = 4u;
    //     const float sampleWeight = 1.0f / float(sampleCount);
    //     SampleGenerator sg = SampleGenerator.create(uint2(0u, 0u), texIndex);
    //     for (uint i = 0; i < perItemSize; i++)
    //     {
    //         for (uint j = 0; j < perItemSize; j++)
    //         {
    //             float3 Le = 0.0f;
    //             if (TEXTURE_TYPE == 3)
    //             {
    //                 Le = gIncidentContainer.readColor(texIndex, uint2(i, j), perItemSize, gIncidentContainer.dataBufferHQ);
    //             }
    //             else if (TEXTURE_TYPE == 1)
    //             {
    //                 Le = gIncidentContainer.readColor(texIndex, uint2(i, j), perItemSize, gIncidentContainer.dataBufferLQ);
    //             }
    //             float3 wi = UVToHemisphere(uint2(i, j), stepSize);
    //             virtualLightSd.V = fromLocal(wi, virtualLightSd.N);
    //             SimpleBRDF vplBrdf;
    //             vplBrdf.setup(virtualLightSd);

    //             for (uint k = 0; k < sampleCount; k++)
    //             {
    //                 float pdfBrdf;
    //                 uint lobeType;
    //                 float3 wo;
    //                 float3 weight;
    //                 bool valid = vplBrdf.specularReflection.sample(wi, wo, pdfBrdf, weight, lobeType, sg);
    //                 if (!valid)
    //                 {
    //                     continue;
    //                 }

    //                 float2 uv = sphericalCoordinateToUV(wo);
    //                 float2 floatUV = uv * perItemSize;
    //                 floatUV = float2(clamp(floatUV.x, 0.0, perItemSize - 1.0), clamp(floatUV.y, 0.0, perItemSize - 1.0));
    //                 floatUV = floor(floatUV);

    //                 uint2 reflectedUV = uint2(floatUV.x, floatUV.y);

    //                 float3 brdfValue = vplBrdf.specularReflection.evalFr(wi, wo);

    //                 float normalizationFactor = readSolidAngleWeight(reflectedUV, perItemSize);

    //                 float3 result = (sampleWeight * Le * brdfValue / (pdfBrdf * normalizationFactor));

    //                 gIncidentContainer.interlockAddColor(texIndex, reflectedUV, result, perItemSize, gConvertedBuffer);
    //                 totalFlux += calcFlux(2.0 * M_PI * saturate(wo.z) * result);
    //             }
    //         }
    //     }
    // }
    // else
    {
        SimpleBRDF vplBrdf;
        vplBrdf.setup(virtualLightSd);

        for (uint i = 0; i < perItemSize; i++)
        {
            for (uint j = 0; j < perItemSize; j++)
            {
                uint2 reflectedUV = uint2(perItemSize - 1 - i, perItemSize - 1 - j);
                float3 wi = UVToHemisphere(uint2(i, j), stepSize);
                float3 F = evalFresnelSchlick(vplBrdf.specularReflection.albedo, 1, wi.z);
                float3 Le = 0.0f;
                if (TEXTURE_TYPE == 3)
                {
                    Le = gIncidentContainer.readColor(texIndex, uint2(i, j), perItemSize, gIncidentContainer.dataBufferHQ);
                }
                else if (TEXTURE_TYPE == 1)
                {
                    Le = gIncidentContainer.readColor(texIndex, uint2(i, j), perItemSize, gIncidentContainer.dataBufferLQ);
                }
                float normalizationFactor = readSolidAngleWeight(reflectedUV, perItemSize); // 2.0 * M_PI * wi.z / 256.0;
                float3 res = Le * F / normalizationFactor;
                gIncidentContainer.interlockAddColor(texIndex, reflectedUV, res, perItemSize, gConvertedBuffer);
                totalFlux += calcFlux(2.0 * M_PI * wi.z * Le * F);
            }
        }
    }

    interlockAddFlux(lightIndex, totalFlux);
}
