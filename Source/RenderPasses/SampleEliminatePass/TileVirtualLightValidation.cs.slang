cbuffer CB
{
    uint gTileSampleNum;
    uint2 gTileDims;
    // VirtualLightContainer gInitialVirtualLights;
    // VirtualLightContainer gSampleEliminatedVirtualLights;
};

/*
Key original index
Value new Index
*/
StructuredBuffer<uint> gHashMap;

RWStructuredBuffer<uint> gTileVirtualLightContainer;

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
        uint oldIndex = gTileVirtualLightContainer[tileIndex];
        uint newIndex = gHashMap[oldIndex];
        if (newIndex != 0xffffffff)
        {
            gTileVirtualLightContainer[tileIndex] = newIndex;
        }
        // This Sample Got Removed By Sample Elimination, Use RTX Neighbour Search To Fill It
        // else
        // {
        //     float closetDis = 99999999.9f;
        //     uint closetIndex = 0xffffffff;

        //     float3 posW = gInitialVirtualLights.getPosition(oldIndex);

        //     RayDesc searchRay;
        //     searchRay.Origin = posW;
        //     searchRay.Direction = float3(0.0f, 1.0f, 0.0f);
        //     searchRay.TMin = 0.0f;
        //     searchRay.TMax = 0.00001f;
        //     RayQuery<RAY_FLAG_NONE> rayQuery;

        //     rayQuery.TraceRayInline(gSampleEliminatedVirtualLights.as, RAY_FLAG_NONE, 0xff, searchRay);
        //     for (uint i = 0; i < 8; i++)
        //     {
        //         if(!rayQuery.Proceed())
        //         {
        //             break;
        //         }
            
        //         if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
        //         {
        //             uint lightIndex = rayQuery.CandidatePrimitiveIndex();
        //             float3 searchedPosW = gSampleEliminatedVirtualLights.getPosition(lightIndex);
        //             float dis =  dot(searchedPosW - posW, searchedPosW - posW);
        //             if (dis < closetDis)
        //             {
        //                 closetDis = dis;
        //                 closetIndex = lightIndex;
        //             }
        //         }

        //         if (i == 7)
        //         {
        //             rayQuery.Abort();
        //         }
        //     }

        //     gTileVirtualLightContainer[tileIndex] = closetIndex;
        // }
    }
}
