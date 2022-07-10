import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;
import Utils.VirtualLight.VirtualLightContainer;
import Utils.VirtualLight.MegaTextureContainer;
import Scene.Material.SimpleBRDF;
import Scene.Shading;

static const bool kTreatHighlyGlossyAsDelta = _TREAT_HIGHLY_GLOSSY_AS_DELTA;
static const float kHighlyGlossyThreshold = _HIGHLY_GLOSSY_THRESHOLD;

cbuffer CB
{
    uint gCount;
    uint gProcessedCount;
    uint gTexType;
    VirtualLightContainer gVirtualLights;
    MegaTextureContainer gIncidentContainer;
};

RWByteAddressBuffer gConvertedBuffer;
RWByteAddressBuffer gFluxBuffer;

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

void AddFlux(uint index, float flux)
{
    uint address = index * 4u;
    gFluxBuffer.Store(address, asuint(flux));
}

groupshared float3 sharedMem[TEXTURE_PER_ITEM_SIZE * TEXTURE_PER_ITEM_SIZE];

[numthreads(1, TEXTURE_PER_ITEM_SIZE, TEXTURE_PER_ITEM_SIZE)]
void main(uint3 launchIndex : SV_DispatchThreadID, uint IndexOfThreadInGroup : SV_GroupIndex)
{
    uint texIndex = gProcessedCount + launchIndex.x;
    if (texIndex >= gCount)
    {
        return;
    }
    
    uint lightIndex = gVirtualLights.getVirtualLightIndexHQ(texIndex);
    ShadingData virtualLightSd = gVirtualLights.getShadingData(lightIndex);

    uint u = launchIndex.y;
    uint v = launchIndex.z;

    float stepSize = 1.0f / (float)TEXTURE_PER_ITEM_SIZE;
    float3 wo = UVToHemisphere(uint2(u, v), stepSize);
    virtualLightSd.V = fromLocal(wo, virtualLightSd.N);

    if (kTreatHighlyGlossyAsDelta)
    {
        if (virtualLightSd.linearRoughness < kHighlyGlossyThreshold)
        {
            virtualLightSd.linearRoughness = 0.0f;
        }
    }

    SimpleBRDF vplBrdf;
    vplBrdf.setup(virtualLightSd);

    // if (vplBrdf.specularReflection.alpha <= 0.0f)
    // {
    //     return;
    // }

    if (TEXTURE_TYPE == 3)
    {
        sharedMem[u + v * TEXTURE_PER_ITEM_SIZE] = gIncidentContainer.readColor(texIndex, uint2(u, v), TEXTURE_PER_ITEM_SIZE, gIncidentContainer.dataBufferHQ);
    }
    else if (TEXTURE_TYPE == 1)
    {
        sharedMem[u + v * TEXTURE_PER_ITEM_SIZE] = gIncidentContainer.readColor(texIndex, uint2(u, v), TEXTURE_PER_ITEM_SIZE, gIncidentContainer.dataBufferLQ);
    }

    GroupMemoryBarrierWithGroupSync();

    float fluxSum = 0.0;
    float3 result = 0.0;

// Importance Sample
#if 0
    SampleGenerator sg = SampleGenerator.create(launchIndex.yz, texIndex);
    const uint sampleCount = TEXTURE_PER_ITEM_SIZE * TEXTURE_PER_ITEM_SIZE;
    const float normalizationTerm = 1.0f / float(sampleCount);
    for (uint i = 0; i < sampleCount; i++)
    {
        float pdfBrdf;
        uint lobeType;
        float3 rayDirection;
        bool valid = vplBrdf.sample(virtualLightSd, rayDirection, pdfBrdf, lobeType, sg);
        if (!valid || dot(rayDirection, virtualLightSd.faceN) < 1e-6f)
        {
            continue;
        }

        float3 wi = toLocal(rayDirection, virtualLightSd.N);
        float3 Le;
        if (TEXTURE_TYPE == 3)
        {
            Le = gIncidentContainer.readColorBiliearHQ(texIndex, wi);
        }
        else if (TEXTURE_TYPE == 1)
        {
            Le = gIncidentContainer.readColorBiliearLQ(texIndex, wi);
        }

#if USE_SECOND_DIFFUSE_BRDF == 1
        float3 brdfValue = vplBrdf.evalFrWIthoutCheck(wo, wi);
#else
        float3 brdfValue = vplBrdf.specularReflection.evalFr(wo, wi);    
#endif
        result += (normalizationTerm * Le * brdfValue / pdfBrdf);
        fluxSum += calcFlux(2.0 * M_PI * saturate(wo.z) * result);
    }

// Uniform Sample
#else 
    for (uint i = 0; i < TEXTURE_PER_ITEM_SIZE; i++)
    {
        for (uint j = 0; j < TEXTURE_PER_ITEM_SIZE; j++)
        {
            float3 Le = sharedMem[i + j * TEXTURE_PER_ITEM_SIZE];
            for (uint k = 0; k < 4; k++)
            {
                float3 wo = UV2HemisphereFloat(float2(u, v) + (plastic(k) - 0.5f), stepSize);
                float3 wi = UVToHemisphere(uint2(i, j), stepSize);
#if USE_SECOND_DIFFUSE_BRDF == 1
                float3 brdfValue = vplBrdf.evalFrWIthoutCheck(wo, wi);
#else
                float3 brdfValue = vplBrdf.specularReflection.evalFr(wo, wi);    
#endif
                float3 subResult = 0.25f * Le * brdfValue;
                result += subResult;
                fluxSum += calcFlux(2.0 * M_PI  * saturate(wo.z) * subResult);
            }
        }
    }
#endif

    gIncidentContainer.AddColor(texIndex, uint2(u, v), result, TEXTURE_PER_ITEM_SIZE, gConvertedBuffer);
    interlockAddFlux(lightIndex, fluxSum);
    // sharedMem2[IndexOfThreadInGroup] = fluxSum;
    
    // // wait until everything is transfered from device memory to shared memory
    // GroupMemoryBarrierWithGroupSync();

    // // hard-coded for 256 threads
    // if (TEXTURE_PER_ITEM_SIZE == 32)
    // {
    //     if (IndexOfThreadInGroup < 512)
    //         sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 512];
    //     GroupMemoryBarrierWithGroupSync();

    //     if (IndexOfThreadInGroup < 256)
    //         sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 256];
    //     GroupMemoryBarrierWithGroupSync();

    //     if (IndexOfThreadInGroup < 128)
    //         sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 128];
    //     GroupMemoryBarrierWithGroupSync();

    //     if (IndexOfThreadInGroup < 64)
    //         sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 64];
    //     GroupMemoryBarrierWithGroupSync();

    //     if (IndexOfThreadInGroup < 32) sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 32];
    //     if (IndexOfThreadInGroup < 16) sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 16];
    //     if (IndexOfThreadInGroup < 8) sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 8];
    //     if (IndexOfThreadInGroup < 4) sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 4];
    //     if (IndexOfThreadInGroup < 2)   sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 2];
    //     if (IndexOfThreadInGroup < 1)   sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 1];

    //     // Have the first thread write out to the output
    //     if (IndexOfThreadInGroup == 0)
    //     {
    //         AddFlux(lightIndex, sharedMem2[0]);
    //     }
    // }
    // else if (TEXTURE_PER_ITEM_SIZE == 16)
    // {
    //     if (IndexOfThreadInGroup < 128)
    //         sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 128];
    //     GroupMemoryBarrierWithGroupSync();

    //     if (IndexOfThreadInGroup < 64)
    //         sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 64];
    //     GroupMemoryBarrierWithGroupSync();

    //     if (IndexOfThreadInGroup < 32) sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 32];
    //     if (IndexOfThreadInGroup < 16) sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 16];
    //     if (IndexOfThreadInGroup < 8) sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 8];
    //     if (IndexOfThreadInGroup < 4) sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 4];
    //     if (IndexOfThreadInGroup < 2)   sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 2];
    //     if (IndexOfThreadInGroup < 1)   sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 1];

    //     // Have the first thread write out to the output
    //     if (IndexOfThreadInGroup == 0)
    //     {
    //         AddFlux(lightIndex, sharedMem2[0]);
    //     }
    // }
    // else if (TEXTURE_PER_ITEM_SIZE == 8)
    // {
    //     if (IndexOfThreadInGroup < 32) sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 32];
    //     if (IndexOfThreadInGroup < 16) sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 16];
    //     if (IndexOfThreadInGroup < 8) sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 8];
    //     if (IndexOfThreadInGroup < 4) sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 4];
    //     if (IndexOfThreadInGroup < 2)   sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 2];
    //     if (IndexOfThreadInGroup < 1)   sharedMem2[IndexOfThreadInGroup] += sharedMem2[IndexOfThreadInGroup + 1];

    //     // Have the first thread write out to the output
    //     if (IndexOfThreadInGroup == 0)
    //     {
    //         AddFlux(lightIndex, sharedMem2[0]);
    //     }
    // }

    //interlockAddFlux(lightIndex, calcFlux(subFlux));
}
