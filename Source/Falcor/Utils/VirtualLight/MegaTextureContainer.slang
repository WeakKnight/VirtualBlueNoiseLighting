import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;
import Utils.VirtualLight.TexelUtil;
#include "Utils/VirtualLight/MegaTextureConfig.slangh"

#define NV_SHADER_EXTN_SLOT u63
#define NV_SHADER_EXTN_REGISTER_SPACE space0
#include "NVAPI/nvHLSLExtns.h"

#ifdef _WRITE_MEGA_TEXTURE
#define ADDRESS_BUFFER RWByteAddressBuffer
#else
#define ADDRESS_BUFFER ByteAddressBuffer
#endif

float2 sphericalCoordinateToUV(float3 dir)
{
    float3 p = normalize(dir);
    float2 uv;
    p /= dot(float3(1.0f), abs(p));
    return 0.5 * float2(p.x - p.y + 1.0, -p.x - p.y + 1.0);
}

float3 UVToHemisphere(uint2 uvPos, float stepSize)
{
    float2 uv = float2(uvPos.x * stepSize + 0.5 * stepSize, uvPos.y * stepSize + 0.5 * stepSize);
    float2 xy = float2((uv.x - uv.y), (1.0f - uv.y - uv.x));
    
    float2 absXY = abs(xy);
    float3 pPrime = float3(xy.x, xy.y, 1.0 - absXY.x - absXY.y);
    
    return normalize(pPrime * dot(float3(1.0f, 1.0f, 1.0f), abs(pPrime)));
}

float3 UV2HemisphereFloat(float2 uvPos, float stepSize)
{
    float2 uv = float2(uvPos.x * stepSize + 0.5 * stepSize, uvPos.y * stepSize + 0.5 * stepSize);
    float2 xy = float2((uv.x - uv.y), (1.0f - uv.y - uv.x));
    
    float2 absXY = abs(xy);
    float3 pPrime = float3(xy.x, xy.y, 1.0 - absXY.x - absXY.y);
    
    return normalize(pPrime * dot(float3(1.0f, 1.0f, 1.0f), abs(pPrime)));
}

struct MegaTextureContainer
{
    uint perItemSizeHQ;
    ADDRESS_BUFFER dataBufferHQ;
    Texture2DArray<float4> containerHQ;

    uint perItemSizeLQ;
    ADDRESS_BUFFER dataBufferLQ;
    Texture2DArray<float4> containerLQ;

    SamplerState linearSampler;

#ifdef _WRITE_MEGA_TEXTURE

    void interlockAddColor(uint itemIndex, uint2 uv, float3 color, uint perItemSize, RWByteAddressBuffer dataBuffer)
    {
        uint address = itemIndex * perItemSize * perItemSize * 12u + (uv.y * perItemSize + uv.x) * 12u;
        NvInterlockedAddFp32(dataBuffer, address + 0, color.r);
        NvInterlockedAddFp32(dataBuffer, address + 4, color.g);
        NvInterlockedAddFp32(dataBuffer, address + 8, color.b);
    }

    void AddColor(uint itemIndex, uint2 uv, float3 color, uint perItemSize, RWByteAddressBuffer dataBuffer)
    {
        uint address = itemIndex * perItemSize * perItemSize * 12u + (uv.y * perItemSize + uv.x) * 12u;
        dataBuffer.Store3(address, asuint(color));
    }

    void interlockAddColorHQ(uint itemIndex, uint2 uv, float3 color)
    {
        interlockAddColor(itemIndex, uv, color, perItemSizeHQ, dataBufferHQ);
    }

    void interlockAddColorLQ(uint itemIndex, uint2 uv, float3 color)
    {
        interlockAddColor(itemIndex, uv, color, perItemSizeLQ, dataBufferLQ);
    }

    void interlockWritePhotonHQ(uint itemIndex, float3 wi, float3 photonEnergy)
    {
        float2 floatUV = sphericalCoordinateToUV(wi) * float(perItemSizeHQ) - float2(0.5);
        floatUV = float2(clamp(floatUV.x, 0.0, (perItemSizeHQ - 1.0)), clamp(floatUV.y, 0.0, (perItemSizeHQ - 1.0)));

        uint2 topLeft = floor(floatUV);
        uint2 bottomRight = topLeft + uint2(1, 1);
        bottomRight = uint2(min(bottomRight.x, perItemSizeHQ - 1), min(bottomRight.y, perItemSizeHQ - 1));

        uint top = topLeft.y;
        uint left = topLeft.x;
        uint bottom = bottomRight.y;
        uint right = bottomRight.x; 

        float2 fracPos = floatUV - floor(floatUV);
        
        float areaTopLeft = (1.0 - fracPos.x) * (1.0 - fracPos.y);
        float areaTopRight = fracPos.x * (1.0 - fracPos.y);
        float areaBottomLeft = (1.0 - fracPos.x) * fracPos.y;
        float areaBottomRight = fracPos.x * fracPos.y;

        interlockAddColorHQ(itemIndex, uint2(left, top), areaTopLeft * photonEnergy);
        interlockAddColorHQ(itemIndex, uint2(right, top), areaTopRight * photonEnergy);
        interlockAddColorHQ(itemIndex, uint2(left, bottom), areaBottomLeft * photonEnergy);
        interlockAddColorHQ(itemIndex, uint2(right, bottom), areaBottomRight * photonEnergy);
    }

    void interlockWritePhotonLQ(uint itemIndex, float3 wi, float3 photonEnergy)
    {
        float2 floatUV = sphericalCoordinateToUV(wi) * float(perItemSizeLQ) - float2(0.5);
        floatUV = float2(clamp(floatUV.x, 0.0, (perItemSizeLQ - 1.0)), clamp(floatUV.y, 0.0, (perItemSizeLQ - 1.0)));

        uint2 topLeft = floor(floatUV);
        uint2 bottomRight = topLeft + uint2(1, 1);
        bottomRight = uint2(min(bottomRight.x, perItemSizeLQ - 1), min(bottomRight.y, perItemSizeLQ - 1));

        uint top = topLeft.y;
        uint left = topLeft.x;
        uint bottom = bottomRight.y;
        uint right = bottomRight.x; 

        float2 fracPos = floatUV - floor(floatUV);
        
        float areaTopLeft = (1.0 - fracPos.x) * (1.0 - fracPos.y);
        float areaTopRight = fracPos.x * (1.0 - fracPos.y);
        float areaBottomLeft = (1.0 - fracPos.x) * fracPos.y;
        float areaBottomRight = fracPos.x * fracPos.y;

        interlockAddColorLQ(itemIndex, uint2(left, top), areaTopLeft * photonEnergy);
        interlockAddColorLQ(itemIndex, uint2(right, top), areaTopRight * photonEnergy);
        interlockAddColorLQ(itemIndex, uint2(left, bottom), areaBottomLeft * photonEnergy);
        interlockAddColorLQ(itemIndex, uint2(right, bottom), areaBottomRight * photonEnergy);
    }

#endif
        
    float3 readColor(uint itemIndex, uint2 uv, uint perItemSize, ADDRESS_BUFFER dataBuffer)
    {
        uint address = itemIndex * perItemSize * perItemSize * 12u + (uv.y * perItemSize + uv.x) * 12u;
        float3 result = asfloat(dataBuffer.Load3(address));
        return result;
    }

    float3 readColor(uint itemIndex, uint2 uv, uint perItemSize, Texture2DArray<float4> texArr)
    {
        uint layer = itemIndex / 10000;
        uint index1D = itemIndex - layer * 10000;
        uint rowIndex = index1D / 100;
        uint columnIndex = index1D - rowIndex * 100;
        uint2 baseIndex2d = (perItemSize + 2) * uint2(columnIndex, rowIndex) + uint2(1, 1);
        return texArr[uint3(baseIndex2d + uv, layer)].rgb;
    }

    float3 readColorBiliear(uint itemIndex, float2 uv, uint perItemSize, ADDRESS_BUFFER dataBuffer)
    {
        float2 floatUV = uv * perItemSize - 0.5;
        floatUV = float2(clamp(floatUV.x, 0.0, perItemSize - 1.0), clamp(floatUV.y, 0.0, perItemSize - 1.0));
        
        uint2 topLeft = floor(floatUV);
        uint2 bottomRight = topLeft + uint2(1, 1);
        bottomRight = uint2(min(bottomRight.x, perItemSize - 1), min(bottomRight.y, perItemSize - 1));

        float3 colTopLeft = readColor(itemIndex, uint2(topLeft.x, topLeft.y), perItemSize, dataBuffer);
        float3 colTopRight = readColor(itemIndex, uint2(bottomRight.x, topLeft.y), perItemSize, dataBuffer);
        float3 colBottomLeft = readColor(itemIndex, uint2(topLeft.x, bottomRight.y), perItemSize, dataBuffer);
        float3 colBottomRight = readColor(itemIndex, uint2(bottomRight.x, bottomRight.y), perItemSize, dataBuffer);

        float2 fracPos = floatUV - floor(floatUV);
        
        float areaTopLeft = (1.0 - fracPos.x) * (1.0 - fracPos.y);
        float areaTopRight = fracPos.x * (1.0 - fracPos.y);
        float areaBottomLeft = (1.0 - fracPos.x) * fracPos.y;
        float areaBottomRight = fracPos.x * fracPos.y;

        return areaTopLeft * colTopLeft + areaTopRight * colTopRight + areaBottomLeft * colBottomLeft + areaBottomRight * colBottomRight;
    }

    float3 readColorBiliear(uint itemIndex, float3 dir, uint perItemSize, ADDRESS_BUFFER dataBuffer)
    {
        float2 uv = sphericalCoordinateToUV(dir);
        return readColorBiliear(itemIndex, uv, perItemSize, dataBuffer);
    }

    float3 readColorBiliearTex(uint itemIndex, float3 dir, uint perItemSize, Texture2DArray<float4> texArr)
    {
        const uint dilatedSize = (perItemSize + 2);

        float2 uv = sphericalCoordinateToUV(dir);
        uint layer = itemIndex / 10000;
        uint index1D = itemIndex - layer * 10000;
        uint rowIndex = index1D / 100;
        uint columnIndex = index1D - rowIndex * 100;
        uint2 baseIndex2d = dilatedSize * uint2(columnIndex, rowIndex) + uint2(1, 1);

        const float texelSize = 1.0f / (dilatedSize * 100);
        float2 baseUV = float2(texelSize * baseIndex2d.x, texelSize * baseIndex2d.y);

        float2 finalUV = baseUV + ((float)perItemSize / (dilatedSize * 100.0f)) * uv;
        return texArr.SampleLevel(linearSampler, float3(finalUV, layer), 0.f).rgb;
    }

    float3 readColorBiliearHQ(uint itemIndex, float3 dir)
    {
#if ENABLE_SOFTWARE_BILINEAR
        return readColorBiliear(itemIndex, dir, perItemSizeHQ, dataBufferHQ);
#else
        return readColorBiliearTex(itemIndex, dir, perItemSizeHQ, containerHQ);
#endif
    }

    float3 readColorBiliearLQ(uint itemIndex, float3 dir)
    {
#if ENABLE_SOFTWARE_BILINEAR
        return readColorBiliear(itemIndex, dir, perItemSizeLQ, dataBufferLQ);
#else
        return readColorBiliearTex(itemIndex, dir, perItemSizeLQ, containerLQ);
#endif
    }
};
