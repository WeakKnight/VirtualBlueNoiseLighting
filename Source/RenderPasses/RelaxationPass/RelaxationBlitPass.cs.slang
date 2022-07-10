import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;
import Utils.VirtualLight.VirtualLightContainer;

cbuffer CB
{
    bool gUseDMaxForASBuilding;
    float gRadiusScalerForASBuilding;
    VirtualLightContainer gPhotons;
    VirtualLightContainer gVirtualLights;
};

StructuredBuffer<uint> gIndices;
StructuredBuffer<float> gDMaxs;

[numthreads(256, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if(launchIndex.x >= gVirtualLights.count)
    {
        return;
    }

    uint lightIndex = launchIndex.x;
    float3 posW = gVirtualLights.getPosition(lightIndex);

    float3 radius = float3(gUseDMaxForASBuilding? gRadiusScalerForASBuilding * gDMaxs[lightIndex]: gVirtualLights.boundingBoxRadius);
    gVirtualLights.boundingBoxBuffer[lightIndex].minPoint = posW - radius;
    gVirtualLights.boundingBoxBuffer[lightIndex].maxPoint = posW + radius;

    uint photonIndex = gIndices[lightIndex];

    gVirtualLights.setNormal(lightIndex, gPhotons.getNormal(photonIndex));
    gVirtualLights.setFaceNormal(lightIndex, gPhotons.getFaceNormal(photonIndex));
    gVirtualLights.setDiffuse(lightIndex, gPhotons.getDiffuse(photonIndex));
    gVirtualLights.setSpecular(lightIndex, gPhotons.getSpecular(photonIndex));
    gVirtualLights.setMetalRoughness(lightIndex, gPhotons.getMetallic(photonIndex), gPhotons.getRoughness(photonIndex));

    gVirtualLights.allocateTextureIndex(lightIndex);
}
