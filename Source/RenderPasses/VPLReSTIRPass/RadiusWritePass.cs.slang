import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;
import Utils.VirtualLight.VirtualLightContainer;

cbuffer CB
{
    VirtualLightContainer gVirtualLights;
};

StructuredBuffer<float> gDMaxs;

[numthreads(256, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if(launchIndex.x >= gVirtualLights.count)
    {
        return;
    }

    uint VPLIndex = launchIndex.x;
    float3 posW = gVirtualLights.getPosition(VPLIndex);
    float3 radius = float3(max(gDMaxs[VPLIndex], 0.00001f));
    gVirtualLights.boundingBoxBuffer[VPLIndex].minPoint = posW - radius;
    gVirtualLights.boundingBoxBuffer[VPLIndex].maxPoint = posW + radius;
}
