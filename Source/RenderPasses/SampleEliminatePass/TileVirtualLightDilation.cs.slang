cbuffer CB
{
    uint gTileSampleNum;
    uint2 gTileDims;
};

StructuredBuffer<uint> gTileVirtualLightContainer;
RWStructuredBuffer<uint> gOutput;

[numthreads(16, 16, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if (any(launchIndex.xy >= gTileDims))
    {
        return;
    }

    uint2 blockIndex = launchIndex.xy;
    
    uint validLIghtIndex = 0xffffffff;
    uint i = 0;
    // Pass through
    for (i = 0; i < gTileSampleNum; i++)
    {
        uint tileIndex = gTileSampleNum * (blockIndex.y * gTileDims.x + blockIndex.x) + i;
        uint lightIndex = gTileVirtualLightContainer[tileIndex];
        if (lightIndex != 0xffffffff)
        {
            gOutput[tileIndex] = lightIndex;
            if (validLIghtIndex == 0xffffffff)
            {
                validLIghtIndex = lightIndex;
            }            
        }
    }

    // Use The First Valid Light Index Fill Unvalid Index
    for (i = 0; i < gTileSampleNum; i++)
    {
        uint tileIndex = gTileSampleNum * (blockIndex.y * gTileDims.x + blockIndex.x) + i;
        uint lightIndex = gTileVirtualLightContainer[tileIndex];
        if (lightIndex == 0xffffffff && validLIghtIndex != 0xffffffff)
        {
            gOutput[tileIndex] = validLIghtIndex;       
        }
    }

    //// Dilation
    //{
    //    uint tileIndex = gTileSampleNum * (blockIndex.y * gTileDims.x + blockIndex.x) + 0;
    //    uint lightIndex = gTileVirtualLightContainer[tileIndex];
    //    int2 neighbourOffset[8] = { int2(0, 1), int2(1, 0), int2(1, 1), int2(0, -1), int2(-1, 0), int2(-1, -1), int2(-1, 1), int2(1, -1) };
    //    // If first light is still unvalid
    //    if (lightIndex == 0xffffffff)
    //    {
    //        for (i = 0; i < 8; i++)
    //        {
    //            int2 neighbourBlockIndex = blockIndex + neighbourOffset[i];
    //            if (any(neighbourBlockIndex >= gTileDims) || any(neighbourBlockIndex < 0))
    //            {
    //                continue;
    //            }
    //            uint neighbourTileIndex = gTileSampleNum * (neighbourBlockIndex.y * gTileDims.x + neighbourBlockIndex.x) + 0;
    //            uint neighbourLightIndex = gTileVirtualLightContainer[neighbourTileIndex];
                
    //            // Use First Valid Neighbour To Fill Whole Tile
    //            if (neighbourLightIndex != 0xffffffff)
    //            {
    //                for (int j = 0; j < gTileSampleNum; j++)
    //                {
    //                    gOutput[tileIndex + j] = neighbourLightIndex;
    //                }
    //                break;
    //            }
    //        }
    //    }
    //}
}
