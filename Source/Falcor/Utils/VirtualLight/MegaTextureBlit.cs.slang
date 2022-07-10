cbuffer CB
{
    uint gItemSizeHQ;
    uint gItemSizeLQ;
    uint gCount;
}

Buffer<uint> gTypeBuffer;

StructuredBuffer<uint> gIndexBuffer;

ByteAddressBuffer gSourceHQ;
RWTexture2DArray<float4> gContainerHQ;

ByteAddressBuffer gSourceLQ;
RWTexture2DArray<float4> gContainerLQ;

/*
VPL Count
*/
[numthreads(256, 1, 1)]
void main(uint threadIndex : SV_DispatchThreadID)
{
    if (threadIndex >= gCount)
    {
        return;
    }

    uint type = gTypeBuffer[threadIndex];

    /*
    None Check
    */
    if (type == 0u)
    {
        return;
    }

    uint texIndex = gIndexBuffer[threadIndex];
    uint layer = texIndex / 10000u;
    uint index = texIndex - layer * 10000u;
    uint y = index / 100u;
    uint x = index - y * 100u;

    if (type == 3u)
    {
        uint itemSizeAfterDilation = gItemSizeHQ + 2u;
        uint2 baseUV = uint2(x * itemSizeAfterDilation, y * itemSizeAfterDilation);

        for (uint u = 0u; u < gItemSizeHQ; u++)
        {
            for(uint v = 0u; v < gItemSizeHQ; v++)
            {
                uint address = (gItemSizeHQ * gItemSizeHQ * texIndex + v * gItemSizeHQ + u) * 12u;
                float3 col = asfloat(gSourceHQ.Load3(address));
                uint2 uv = baseUV + uint2(u, v) + uint2(1u, 1u);
                gContainerHQ[uint3(uv, layer)] = float4(col, 1.0f);
            }
        }
    }
    else if (type == 1u)
    {
        uint itemSizeAfterDilation = gItemSizeLQ + 2u;
        uint2 baseUV = uint2(x * itemSizeAfterDilation, y * itemSizeAfterDilation);

        for (uint u = 0u; u < gItemSizeLQ; u++)
        {
            for(uint v = 0u; v < gItemSizeLQ; v++)
            {
                uint address = (gItemSizeLQ * gItemSizeLQ * texIndex + v * gItemSizeLQ + u) * 12u;
                float3 col = asfloat(gSourceLQ.Load3(address));
                uint2 uv = baseUV + uint2(u, v) + uint2(1u, 1u);
                gContainerLQ[uint3(uv, layer)] = float4(col, 1.0f);
            }
        }
    }
}
