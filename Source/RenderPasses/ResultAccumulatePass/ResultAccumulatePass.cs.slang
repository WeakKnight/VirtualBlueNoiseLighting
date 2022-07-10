cbuffer PerFrameCB
{
    uint2   gResolution;
    uint    gAccumCount;
}

// Input data to accumulate and accumulated output.
Texture2D<float4>   gCurFrame;
RWTexture2D<float4> gOutputFrame;

// Last frame data, format depends on accumulation mode.
RWTexture2D<float4> gLastFrameSum;      // If mode is Single

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    if (any(dispatchThreadId.xy >= gResolution)) return;
    const uint2 pixelPos = dispatchThreadId.xy;
    const float4 curColor = gCurFrame[pixelPos];

    // Fetch previous sum and compute the new sum.
    float4 sum = gLastFrameSum[pixelPos] + curColor;
    float4 output = sum / (gAccumCount + 1);

    gLastFrameSum[pixelPos] = sum;
    gOutputFrame[pixelPos] = output;
}
