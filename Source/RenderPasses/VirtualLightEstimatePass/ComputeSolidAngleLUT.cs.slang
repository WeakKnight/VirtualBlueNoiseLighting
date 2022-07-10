import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;

#define NV_SHADER_EXTN_SLOT u63
#define NV_SHADER_EXTN_REGISTER_SPACE space0
#include "NVAPI/nvHLSLExtns.h"

cbuffer CB
{
    uint gItemSize;
    uint gSampleCount;
}

RWByteAddressBuffer gLUTBuffer;

float2 sphericalCoordinateToUV(float3 dir)
{
    float3 p = normalize(dir);
    float2 uv;
    p /= dot(float3(1.0f), abs(p));
    return 0.5 * float2(p.x - p.y + 1.0, -p.x - p.y + 1.0);
}

void writeBuffer(uint2 uv, float cosTheta, float weight)
{
    uint address = (uv.y * gItemSize + uv.x) * 4u;
    NvInterlockedAddFp32(gLUTBuffer, address, weight * 2.0 * cosTheta * M_PI / (float)gSampleCount);

}

[numthreads(256, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    SampleGenerator sg = SampleGenerator.create(uint2(launchIndex.x, 1), 1);
    float pdf;
    float3 sampledDir = HemisphereSampling(sampleNext2D(sg), float3(0.0f, 0.0f, 1.0f), pdf);

    float2 uv = sphericalCoordinateToUV(-sampledDir);

    float2 floatUV = uv * gItemSize;
    floatUV = float2(clamp(floatUV.x, 0.0, gItemSize - 1.0), clamp(floatUV.y, 0.0, gItemSize - 1.0));
    floatUV = floor(floatUV);
    writeBuffer(uint2(floatUV.x, floatUV.y), sampledDir.z, 1.f);

    //float2 floatUV = uv * gItemSize - 0.5;
    //floatUV = float2(clamp(floatUV.x, 0.0, gItemSize - 1.0), clamp(floatUV.y, 0.0, gItemSize - 1.0));

    //uint2 topLeft = floor(floatUV);
    //uint2 bottomRight = topLeft + uint2(1, 1);
    //bottomRight = uint2(min(bottomRight.x, gItemSize - 1), min(bottomRight.y, gItemSize - 1));

    //uint top = topLeft.y;
    //uint left = topLeft.x;
    //uint bottom = bottomRight.y;
    //uint right = bottomRight.x; 

    //float2 fracPos = floatUV - floor(floatUV);
    //
    //float areaTopLeft = (1.0 - fracPos.x) * (1.0 - fracPos.y);
    //float areaTopRight = fracPos.x * (1.0 - fracPos.y);
    //float areaBottomLeft = (1.0 - fracPos.x) * fracPos.y;
    //float areaBottomRight = fracPos.x * fracPos.y;

    //writeBuffer(uint2(left, top), areaTopLeft);
    //writeBuffer(uint2(right, top), areaTopRight);
    //writeBuffer(uint2(left, bottom), areaBottomLeft);
    //writeBuffer(uint2(right, bottom), areaBottomRight);
}
