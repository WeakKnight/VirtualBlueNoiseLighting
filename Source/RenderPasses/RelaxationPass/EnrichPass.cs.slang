#define _AS_PHOTONS

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
import Utils.Sampling.AliasTable;
import Utils.VirtualLight.VirtualLightContainer;
import Utils.VirtualLight.MegaTextureContainer;
import Scene.Material.SimpleBRDF;

#define NEIGHBOUR_SEARCH_NORMAL_TOLERANCE 0.7

cbuffer PerFrameCB
{
    MegaTextureContainer gSpecRadianceContainer;
    uint gIndex;
    float gRadiusSquared;
    float gTotalWeight;
    uint gActualPhotonCount;

    VirtualLightContainer gPhotons;
    VirtualLightContainer gVirtualLights;
};

StructuredBuffer<uint> gIndicesBuffer;
StructuredBuffer<float> gPrsBuffer;
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

[numthreads(8, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if (launchIndex.x >= gActualPhotonCount) 
    {
        return;
    }

    uint photonIndex = gIndicesBuffer[launchIndex.x];

    ShadingData vplSd = gVirtualLights.getShadingData(gIndex);
    float3 vplPos = vplSd.posW;
    float3 vplNormal = vplSd.N;

    SimpleBRDF vplBrdf;
    vplBrdf.setup(vplSd);

    float3 photonPos = gPhotons.getPosition(photonIndex);
    float dis = sqrt(dot(vplPos - photonPos, vplPos - photonPos));
    float3 photonNormal = gPhotons.getNormal(photonIndex);

    if(dot(photonNormal, vplNormal) > NEIGHBOUR_SEARCH_NORMAL_TOLERANCE)
    {
        float prs = gPrsBuffer[photonIndex];
        if (prs < 1e-4f)
        {
            return;
        }
        float3 Le = gPhotons.incidentRadianceBuffer[photonIndex] / prs;

        float flux = 0.0;

        float weight;
        if(gRadiusSquared < 1e-6f || gTotalWeight < 1e-6f)
        {
            weight = 1.0 / float(gActualPhotonCount);
        }
        else
        {
            weight = 1.0f - dis * dis / (gRadiusSquared);
            weight = weight / gTotalWeight;
        }

        if (weight > 1.0f)
        {
            weight = 0.0f;
        }

        float3 L = gPhotons.directionBuffer[photonIndex];
        float3 wi = toLocal(L, vplNormal);

        uint type = gVirtualLights.getType(gIndex);
#if USE_SECOND_DIFFUSE_BRDF == 1
        if (type == 0)
        // Diffuse Estimation
#endif
        {
            float3 diffuse = weight * Le * vplBrdf.diffuseReflection.evalFr(0.0f, wi);
            gVirtualLights.interlockAddDiffuseRadiance(gIndex, diffuse);
            // We need multiply 2.0 * pi here because we integrate over solid angle
            flux += calcFlux(M_PI * diffuse * gSpecRadianceContainer.perItemSizeHQ * gSpecRadianceContainer.perItemSizeHQ);
        }

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
            uint texIndex = gVirtualLights.getIndex(gIndex);

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

        interlockAddFlux(gIndex, flux);
    }
}
