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
import Scene.RaytracingInlineHelper;
import Utils.VirtualLight.VirtualLightContainer;
import Utils.VirtualLight.MegaTextureContainer;
import RenderPasses.GBufferUtils;
#include "Utils/VirtualLight/MegaTextureConfig.slangh"

static const uint kInvalidIndex = 0xffffffff;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    /*
    Solid Circle Radius
    */
    float gRadius;
    VirtualLightContainer gVirtualLightContainer;
    MegaTextureContainer gSpecRadianceContainer;
    /*
    0: Uniform Solid Circle Mode
    1: Adaptive Solid Circle Mode
    2: Ring Mode
    3: Estimation Heat Map
    4: Adaptive Diffuse Solid Circle Mode
    5: Tile Virtual Light Visualization
    */
    uint gVisType;
    GBuffer gGBuffer;
    uint gTileSize;
    uint gTileSampleNum;
    uint2 gTileDims;
    uint2 gMegaTextureBaseIndex;
    uint gMegaTexturePitch;
    float gMegaTextureExposure;
    float gExposure;
    bool gTonemapped;
    uint2 gSelectedPixel;
    bool gNeedUpdate;

    uint gMaxBSDFSearchCount;
};

RWTexture2D<float4> gTotalSum;
RWTexture2D<float4> gOutput;
RWTexture2D<uint> gSampleNum;
RWStructuredBuffer<uint> gTileVirtualLightContainer;
RWTexture2D<float4> gMegaTextureItem;

bool isNaN(float f)
{
    uint u = asuint(f) & ~0x80000000u; // clear out the sign bit
    return (u > 0x7F800000);           // greater than Inf is NaN
}

float3 toneMapAces(float3 color)
{
    // Cancel out the pre-exposure mentioned in
    // https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/

    color *= 0.6;

    float A = 2.51;
    float B = 0.03;
    float C = 2.43;
    float D = 0.59;
    float E = 0.14;

    color = saturate((color*(A*color+B))/(color*(C*color+D)+E));
    return color;
}

[numthreads(16, 16, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if (any(launchIndex.xy >= gViewportDims)) 
    {
        return;
    }

    float3 result = 0.0f;

    ShadingData sd;
    bool valid = LoadShadingData(gGBuffer, launchIndex.xy, gViewportDims, gScene.camera, sd);
    if (!valid)
    {
        gOutput[launchIndex.xy] = float4(0.0f, 0.0f, 0.0f, 1.0f);
    }

    if (gVisType == 8)
    {
        float3 col = 0.0;
        RayDesc searchRay;
        searchRay.Origin = sd.posW;
        searchRay.Direction = float3(0.0f, 1.0f, 0.0f);
        searchRay.TMin = 0.0f;
        searchRay.TMax = 0.00001f;
        RayQuery<RAY_FLAG_NONE> rayQuery;
        // uint intersectIndicis[8] = {kInvalidIndex, kInvalidIndex, kInvalidIndex, kInvalidIndex, kInvalidIndex, kInvalidIndex, kInvalidIndex, kInvalidIndex};
        rayQuery.TraceRayInline(gVirtualLightContainer.as, RAY_FLAG_NONE, 0xff, searchRay);
        for (uint i = 0; i < 8; i++)
        {
            if(!rayQuery.Proceed())
            {
                break;
            }
        
            if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
            {
                uint lightIndex = rayQuery.CandidatePrimitiveIndex();
                float3 posW = gVirtualLightContainer.getPosition(lightIndex);
                float radius = gVirtualLightContainer.getRadius(lightIndex);
                float disSquare = dot(sd.posW - posW, sd.posW - posW);
                if(disSquare <= radius * radius)
                {
                    float3 L = posW - sd.posW;
                    float3 virtualLightN = gVirtualLightContainer.getNormal(lightIndex);
                    if(dot(sd.N, virtualLightN) > 0.65f)
                    {
                        col += gVirtualLightContainer.readLe(lightIndex, -L, gSpecRadianceContainer);
                    }
                }
            }

            if (i == 7)
            {
                rayQuery.Abort();
            }
        }

        result = col;
        gSampleNum[launchIndex.xy] += 1;
    }
    else if (gVisType == 7)
    {
        float3 cameraPos = gScene.camera.getPosition();
        RayDesc cameraRay = gScene.camera.computeRayPinhole(launchIndex.xy, gViewportDims).toRayDesc();
        RayQuery<RAY_FLAG_NONE> rayQuery;

        rayQuery.TraceRayInline(
            gVirtualLightContainer.as,
            RAY_FLAG_NONE, // OR'd with template flags above
            0xff, // InstanceInclusionMask
            cameraRay);

        float closetDistance = 1000000.0f;
        while (rayQuery.Proceed())
        {
            if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
            {
                uint lightIndex = rayQuery.CandidatePrimitiveIndex();
                PackedBoundingBox boundingBox = gVirtualLightContainer.boundingBoxBuffer[lightIndex];
                float3 vplOrigin = (boundingBox.minPoint + boundingBox.maxPoint) * 0.5f;
                float radius = 0.5f * (boundingBox.maxPoint.x - boundingBox.minPoint.x);
                float t;
                if (intersectRaySphere(cameraRay.Origin, cameraRay.Direction, vplOrigin, radius, t))
                {
                    if (closetDistance > t)
                    {
                        closetDistance = t;
                        rayQuery.CommitProceduralPrimitiveHit(t);
                    }
                }
            }
        }

        if(rayQuery.CommittedStatus() == COMMITTED_PROCEDURAL_PRIMITIVE_HIT)
        {
            float3 posW = cameraRay.Origin + rayQuery.CommittedRayT() * cameraRay.Direction;
            uint vplIndex = rayQuery.CommittedPrimitiveIndex();
            PackedBoundingBox boundingBox = gVirtualLightContainer.boundingBoxBuffer[vplIndex];
            float3 vplOrigin = (boundingBox.minPoint + boundingBox.maxPoint) * 0.5f;
            result = gMegaTextureExposure * gVirtualLightContainer.readLe(vplIndex, -normalize(posW - vplOrigin), gSpecRadianceContainer);
            gSampleNum[launchIndex.xy] += 1;

            uint type = gVirtualLightContainer.typeBuffer[vplIndex];
            if (gNeedUpdate && type == 3)
            {
                if (gSelectedPixel.x == launchIndex.x && gSelectedPixel.y == launchIndex.y)
                {
                    uint texIndex = gVirtualLightContainer.indexBuffer[vplIndex];
                    for (uint u = 0; u < gSpecRadianceContainer.perItemSizeHQ; u++)
                    {
                        for (uint v = 0; v < gSpecRadianceContainer.perItemSizeHQ; v++)
                        {
                            gMegaTextureItem[uint2(u,v)] = float4(toneMapAces(pow(2.0f, gExposure) * gMegaTextureExposure * gSpecRadianceContainer.readColor(texIndex, uint2(u,v), gSpecRadianceContainer.perItemSizeHQ, gSpecRadianceContainer.dataBufferHQ)), 1.0f);
                        }
                    }
                }
            }
        }
    }
    else if (gVisType == 6)
    {
#if ENABLE_SOFTWARE_BILINEAR
        uint2 megaTex2DIndex = gMegaTextureBaseIndex + launchIndex.xy / gSpecRadianceContainer.perItemSizeHQ;
        uint2 megaTexUV = launchIndex.xy - gSpecRadianceContainer.perItemSizeHQ * megaTex2DIndex;
        uint megaTexIndex = megaTex2DIndex.y * gMegaTexturePitch + megaTex2DIndex.x;
        if (megaTexIndex < gVirtualLightContainer.texCountHQ)
        {
            float3 radiance = gMegaTextureExposure * gSpecRadianceContainer.readColor(megaTexIndex, megaTexUV, gSpecRadianceContainer.perItemSizeHQ, gSpecRadianceContainer.dataBufferHQ);
            if (isNaN(radiance.x) || isNaN(radiance.y) || isNaN(radiance.z))
            {
                radiance = float3(2.0, 0.0, 0.0);
            }
            result = radiance;
            gSampleNum[launchIndex.xy] += 1;
        }
#endif
    }
    else if (gVisType == 5)
    {
        uint2 blockIndex = launchIndex.xy / gTileSize;
        float3 col = 0.0f;
        uint baseIndex = (gTileSampleNum + 1) * (blockIndex.y * gTileDims.x + blockIndex.x);
        uint sampleNum = gTileVirtualLightContainer[baseIndex + gTileSampleNum];
        for (uint i = 0; i < sampleNum; i++)
        {
            uint tileIndex = baseIndex + i;
            uint vplIndex = gTileVirtualLightContainer[tileIndex];
            if (vplIndex != 0xffffffff)
            {
                float3 vplPosition = gVirtualLightContainer.getPosition(vplIndex);
                float3 L = normalize(vplPosition - sd.posW);
                col += gVirtualLightContainer.readLe(vplIndex, L, gSpecRadianceContainer);
            }
        }
        sampleNum = max(sampleNum, 1);
        result = gMegaTextureExposure * col / sampleNum;
        gSampleNum[launchIndex.xy] += 1;
    }
    else
    {
        float3 posW = sd.posW;
        if (valid)
        {
            RayDesc dummyRay;
            dummyRay.Origin = posW;
            dummyRay.Direction = float3(0.0, 1.0, 0.0);
            dummyRay.TMin = 0.0f;
            dummyRay.TMax = 0.0000001f;

            RayQuery<RAY_FLAG_NONE> rayQuery;

            rayQuery.TraceRayInline(
                gVirtualLightContainer.as,
                RAY_FLAG_NONE, // OR'd with template flags above
                0xff, // InstanceInclusionMask
                dummyRay);

            uint hitCount = 0;
            while (rayQuery.Proceed())
            {
                if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
                {
                    uint AABBIndex = rayQuery.CandidatePrimitiveIndex();
                    PackedBoundingBox boundingBox = gVirtualLightContainer.boundingBoxBuffer[AABBIndex];
                    float3 vplOrigin = (boundingBox.minPoint + boundingBox.maxPoint) * 0.5f;
                    float radius = 0.5f * (boundingBox.maxPoint.x - boundingBox.minPoint.x);
                    float3 origin = posW;
                    float dis = sqrt(dot(origin - vplOrigin, origin - vplOrigin));
                    uint type = gVirtualLightContainer.typeBuffer[AABBIndex];
                    float3 visColor = 1.0;
                    // if (type == 3)
                    // {
                    //     visColor = float3(0.0, 0.3, 1.0); 
                    // }
                    if (dis <= radius)
                    {
                        hitCount += 1;
                    }
                    if (gVisType == 0)
                    {
                        if (dis < gRadius)
                        {
                            result = visColor;
                            gSampleNum[launchIndex.xy] += 1;
                            rayQuery.Abort();
                        }
                    }
                    else if (gVisType == 1)
                    {
                        if (dis <= radius)
                        {
                            result = visColor;
                            gSampleNum[launchIndex.xy] += 1;
                            rayQuery.Abort();
                        }
                    }
                    else if (gVisType == 2)
                    {
                        if (dis <= radius && dis >= (radius - 0.001f))
                        {
                            result = visColor;
                            gSampleNum[launchIndex.xy] += 1;
                            rayQuery.Abort();
                        }
                    }
                    else if (gVisType == 4)
                    {
                        if (dis <= radius)
                        {
                            result = gVirtualLightContainer.readDiffuseRadiance(AABBIndex);
                            gSampleNum[launchIndex.xy] += 1;
                            rayQuery.Abort();
                        }
                    }
                }
            }

            if (gVisType == 3)
            {
                float3 goodColor = pow(float3(95.0/255.0, 1.0/255.0, 116.0/255.0), 2.2f);
                float3 badColor = pow(float3(87.0/255.0, 252.0/255.0, 157.0/255.0), 2.2f);
                // gMaxBSDFSearchCount
                if (hitCount > gMaxBSDFSearchCount)
                {
                    result = badColor;
                }
                else
                {
                    result = goodColor;
                }
                gSampleNum[launchIndex.xy] += 1;
            }
        }
    }

    gTotalSum[launchIndex.xy] += float4(result, 1.0f);

    float3 outputVal = gTotalSum[launchIndex.xy].xyz / float(max(gSampleNum[launchIndex.xy], 1u));
    outputVal = pow(2.0f, gExposure) * outputVal;
    if (gTonemapped)
    {
        outputVal = toneMapAces(outputVal);
    }
    gOutput[launchIndex.xy] = float4(outputVal, 1.0f);
}
