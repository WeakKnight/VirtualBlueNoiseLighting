import Scene.Scene;
import Scene.Shading;
import Scene.HitInfo;
import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Math.MathHelpers;
import Utils.Sampling.SampleGenerator;
import Utils.Color.ColorHelpers;
import Utils.Math.PackedFormats;

static const bool kUseVBuffer = true;

struct GBuffer
{
    Texture2D<float4> _TexAlbedo;
    Texture2D<float> _TexMetallic;
    Texture2D<float4> _TexPosition;
    Texture2D<float4> _TexEmissive;
    Texture2D<float4> _TexShadingNormal;
    Texture2D<float4> _TexFaceNormal;
    Texture2D<float4> _TexLinearZAndNormal;
    Texture2D<float2> _TexMotion;
    Texture2D<float4> _TexMotionW;
    Texture2D<PackedHitInfo> _TexVBuffer;
    Texture2D<float4> _TexNormalWRoughnessMaterialID;

    Texture2D<float4> _TexPreLinearZAndNormal;

    inline float2 GetMotionVector(uint2 texCoord)
    {
        return _TexMotion[texCoord].rg;
    }

    inline float2 GetDepthAndDriv(uint2 texCoord, bool temporal = false)
    {
        if (temporal)
        {
            return _TexPreLinearZAndNormal[texCoord].rg;
        }
        else 
        {
            return _TexLinearZAndNormal[texCoord].rg;
        }
    }

    inline float3 GetNormal(uint2 texCoord, bool temporal = false)
    {
        if (temporal)
        {
            return oct_to_ndir_snorm(_TexPreLinearZAndNormal[texCoord].zw);
        }
        else
        {
            return _TexShadingNormal[texCoord].xyz;
        }
    }

    bool IsReprjValid(uint2 texCoord, uint2 texCoordNeighbour, bool isTemporalReprj, float normalThreshold = 0.65, float depthThreshold = 0.1)
    {
        uint2 texDim;
        uint levels;
        _TexAlbedo.GetDimensions(0, texDim.x, texDim.y, levels);
        bool insideScreen = (texCoordNeighbour.x >= 0 && texCoordNeighbour.y >= 0 && texCoordNeighbour.x < texDim.x && texCoordNeighbour.y < texDim.y);
        if(!insideScreen)
        {
            return false;
        }

        float2 depth = GetDepthAndDriv(texCoord);
        float3 normal = GetNormal(texCoord);

        float2 depthNeighbour = GetDepthAndDriv(texCoordNeighbour, isTemporalReprj);
        float3 normalNeighbour = GetNormal(texCoordNeighbour, isTemporalReprj);
        
        if(!compareRelativeDifference(depth.x, depthNeighbour.x, depthThreshold))
        {
            return false;
        }

        // check normals for compatibility
        if (dot(normal, normalNeighbour) < normalThreshold) 
        {
            return false;
        }

        return true;
    }
}

bool IsRenderingDataValid(GBuffer pGBuffer, uint2 texCoord)
{
    return pGBuffer._TexPosition[texCoord].a > 0.5;
}

bool PrepareRenderingData(GBuffer pGBuffer, uint2 pixel, uint2 frameDim, const Camera camera, out ShadingData sd, out HitInfo hit)
{
    sd = {};

    float3 rayDir = camera.computeRayPinhole(pixel, frameDim).dir;
    bool valid = false;

    if (pGBuffer._TexEmissive[pixel].a > 0.1f)
    {
        if (hit.decode(pGBuffer._TexVBuffer[pixel]))
        {
            // Evaluate Falcor's material parameters at the hit point.
            // TODO: Implement texLOD to enable texture filtering in prepareShadingData().
            VertexData v = gScene.getVertexData(hit);
            const uint materialID = gScene.getMaterialID(hit.instanceID);
            sd = prepareShadingData(v, materialID, gScene.materials[materialID], gScene.materialResources[materialID], -rayDir, 0.f);
            adjustShadingNormal(sd, v);
            valid = true;
        }
    }

    sd.V = -rayDir;
    return valid;
}

bool PrepareRenderingData(GBuffer pGBuffer, uint2 texCoord, float3 cameraPos, out ShadingData sd)
{
    sd.posW = pGBuffer._TexPosition[texCoord].xyz;
    sd.V = normalize(cameraPos - sd.posW);
    sd.N = pGBuffer.GetNormal(texCoord);
    sd.NdotV = dot(sd.N, sd.V);
    sd.doubleSided = pGBuffer._TexEmissive[texCoord].a > 0.5f;
    sd.faceN = pGBuffer._TexFaceNormal[texCoord].xyz;
    sd.frontFacing = pGBuffer._TexFaceNormal[texCoord].w > 0.5f;
    if (sd.frontFacing)
    {
        sd.emissive = pGBuffer._TexEmissive[texCoord].rgb;
    }
    else
    {
        sd.emissive = 0.0f;
    }
    sd.metallic = pGBuffer._TexMetallic[texCoord].r;
    sd.diffuse = pGBuffer._TexAlbedo[texCoord].rgb;
    sd.opacity = pGBuffer._TexAlbedo[texCoord].a;
    
    float4 speuclarRoughness = Unpack_R8G8B8A8_Gamma_UFLOAT(asuint(pGBuffer._TexPosition[texCoord].w));
    sd.specular = speuclarRoughness.rgb;
    sd.linearRoughness = speuclarRoughness.a;

    sd.ggxAlpha = max(0.0064, sd.linearRoughness * sd.linearRoughness);

    if (!sd.frontFacing && sd.doubleSided)
    {
        sd.N = -sd.N;
        sd.NdotV = -sd.NdotV;
    }

    return pGBuffer._TexEmissive[texCoord].a > 0.1f;
}

bool LoadShadingData(GBuffer pGBuffer, uint2 pixel, uint2 frameDim, const Camera camera, out ShadingData sd)
{
    if (kUseVBuffer)
    {
        HitInfo hit;
        return PrepareRenderingData(pGBuffer, pixel, frameDim, camera, sd, hit);
    }
    else
    {
        return PrepareRenderingData(pGBuffer, pixel, camera.getPosition(), sd);
    }
}

bool AreMaterialsSimilar(ShadingData a, ShadingData b)
{
    const float roughnessThreshold = 0.5;
    const float reflectivityThreshold = 0.5;
    const float albedoThreshold = 0.5;

    if (!compareRelativeDifference(a.linearRoughness, b.linearRoughness, roughnessThreshold))
        return false;

    if (!compareRelativeDifference(luminance(a.specular), luminance(b.specular), reflectivityThreshold))
        return false;
    
    if (!compareRelativeDifference(luminance(a.diffuse), luminance(b.diffuse), albedoThreshold))
        return false;

    return true;
}