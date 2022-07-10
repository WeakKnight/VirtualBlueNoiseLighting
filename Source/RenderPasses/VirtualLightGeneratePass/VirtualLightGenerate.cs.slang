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
import Utils.VirtualLight.VirtualLightContainer;
import Experimental.Scene.Material.MaterialShading;
import Scene.RaytracingInlineHelper;
import RenderPasses.GBufferUtils;
import Scene.Material.SimpleBRDF;
import Experimental.Scene.Material.Fresnel;
import RenderPasses.GBufferUtils;

static bool kContinueDeltaBounce = true;

cbuffer CB
{
    uint gFrameIndex;
    uint2 gViewportDims;
    VirtualLightContainer gVirtualLightContainer;
    uint gTileSize;
    uint gTileSampleNum;
    uint2 gTileDims;
    bool gDirectLightingOnly;
    GBuffer gGBuffer;
}

RWStructuredBuffer<uint> gTileVirtualLightContainer;

[numthreads(16, 16, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if (any(launchIndex.xy >= gTileDims)) 
    {
        return;
    }

    SampleGenerator sg = SampleGenerator.create(launchIndex.xy, gFrameIndex);
    uint baseIndex = (gTileSampleNum + 1) * (launchIndex.y * gTileDims.x + launchIndex.x);
    uint firstEmptySampleIndex = 0;

    float3 throughPut = float3(1.0f, 1.0f, 1.0f);

    for (uint i = 0; i < gTileSampleNum; i++)
    {
        uint2 pixelOffset = uint2(min(gTileSize * sampleNext1D(sg), gTileSize - 1), min(gTileSize * sampleNext1D(sg), gTileSize - 1));
        uint2 pixelPos = launchIndex.xy * gTileSize + pixelOffset;
        float2 p = (pixelPos + float2(0.5f, 0.5f)) / float2(gViewportDims.xy);
        CameraRay ray = gScene.camera.sampleRayPinhole(p);
        
        float3 rayDir = ray.dir;
        float3 rayOrigin = ray.origin;
        uint lobeType = 0;
        bool isDelta = false;
        for (int j = 0; j < 4; j++)
        {
            ShadingData sd;
            if (j == 0)
            {
                bool valid = LoadShadingData(gGBuffer, pixelPos, gViewportDims, gScene.camera, sd);
                if (!valid)
                {
                    break;
                }
            }
            else
            {
                bool hit = traceScene(rayOrigin, rayDir, sd);
                if (!hit)
                {
                    break;
                }
            }
            SimpleBRDF simpleBRDF;
            simpleBRDF.setup(sd);
            
            if (j == 1)
            {
                if (gDirectLightingOnly)
                {
                    if (kContinueDeltaBounce && isDelta)
                    {
                    }
                    else
                    {
                        uint lightIndex = gVirtualLightContainer.pushLight(sd);
                        gVirtualLightContainer.throughputBuffer[lightIndex] = 0.333 * (throughPut.x + throughPut.y + throughPut.z);
                        
                        gTileVirtualLightContainer[baseIndex + firstEmptySampleIndex] = lightIndex;
                        firstEmptySampleIndex++;

                        break;
                    }
                }
                else
                {
                    uint lightIndex = gVirtualLightContainer.pushLight(sd);
                    gVirtualLightContainer.throughputBuffer[lightIndex] = 0.333 * (throughPut.x + throughPut.y + throughPut.z);
                    
                    gTileVirtualLightContainer[baseIndex + firstEmptySampleIndex] = lightIndex;
                    firstEmptySampleIndex++;
                }
            }
            else if (j > 1)
            {
                if (lobeType == (uint)LOBE_TYPE::DIFFUSE)
                {
                    break;
                }
                
                if (sampleNext1D(sg) < simpleBRDF.pSpecularReflection)
                {
                    uint lightIndex = gVirtualLightContainer.pushLight(sd);
                    gVirtualLightContainer.throughputBuffer[lightIndex] = 0.333 * (throughPut.x + throughPut.y + throughPut.z);
                    break;
                }
            }
            
            float pdf;
            bool valid = simpleBRDF.sample(sd, rayDir, pdf, lobeType, sg);
            if(!valid || dot(rayDir, sd.faceN) < 1e-6f)
            {
                break;
            }

            isDelta = (lobeType == 3);

            float3 wi = toLocal(rayDir, sd.N);
            float3 wo = toLocal(sd.V, sd.N);
            if(lobeType == 3)
            {
                float3 H = (sd.V + rayDir) * 0.5;
                float woDotH = saturate(dot(sd.V, H));
                float3 F = evalFresnelSchlick(simpleBRDF.specularReflection.albedo, 1, woDotH);
                throughPut = throughPut * F / simpleBRDF.pSpecularReflection;
            }
            else
            {
                throughPut = throughPut * simpleBRDF.eval(wo, wi) / pdf;
            }
            
            rayOrigin = computeRayOrigin(sd.posW, sd.faceN);
        }
    }

    gTileVirtualLightContainer[baseIndex + gTileSampleNum] = firstEmptySampleIndex;
}
