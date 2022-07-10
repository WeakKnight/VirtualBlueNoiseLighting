cbuffer CB
{
    uint gTileSampleNum;
    uint2 gTileDims;
};

RWStructuredBuffer<uint> gTileVirtualLightContainer;
RWByteAddressBuffer gTileVirtualLightWeights;

[numthreads(16, 16, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if (any(launchIndex.xy >= gTileDims))
    {
        return;
    }

    uint2 blockIndex = launchIndex.xy;
    uint tileBaseIndex = (gTileSampleNum + 1) * (blockIndex.y * gTileDims.x + blockIndex.x);
    uint firstEmptySampleIndex = 0;
    uint tileSampleCount = gTileVirtualLightContainer[tileBaseIndex + gTileSampleNum];

    for (uint i = 0; i < tileSampleCount; i++)
    {
        uint tileIndex = tileBaseIndex + i;
        uint lightIndex = gTileVirtualLightContainer[tileIndex];
        gTileVirtualLightWeights.InterlockedAdd(lightIndex * 4u, 1);
    }
}
