#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Sampling.ReSTIR;
import Utils.Helpers;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
import Utils.VirtualLight.VirtualLightContainer;
import Utils.VirtualLight.MegaTextureContainer;
import RenderPasses.GBufferUtils;
import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Lights.Lights;
import Scene.Shading;
import Scene.RaytracingInlineHelper;
import Experimental.Scene.Lights.LightCollection;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Experimental.Scene.Material.MaterialShading;
import Scene.Material.SimpleBRDF;
import Experimental.Scene.Material.Fresnel;
import Utils.Sampling.AliasTable;
import LightTreeSampler;
import Utils.Debug.PixelDebug;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    VirtualLightContainer gVirtualLights;
    GBuffer gGBuffer;
    MegaTextureContainer gSpecRadianceContainer;
    AliasTable gFluxTable;
    AliasTable gEmissiveTriTable;
    uint gLeafStartIndex;
    uint gNumFinalConeSamples;
    uint gMaxLightSamples;
}

StructuredBuffer<uint> gTileVirtualLightContainer;
StructuredBuffer<uint> gTileVirtualLightWeights;
StructuredBuffer<LightNode> gNodeBuffer;

Texture2D<float4>   gRawShadingOutput;
RWTexture2D<float4> gShadingOutput;

[numthreads(16, 16, 1)]
void main(uint2 GlobalIndex : SV_DispatchThreadID, uint3 groupIndex: SV_GROUPID)
{
    if (any(GlobalIndex.xy >= gViewportDims)) 
    {
        return;
    }
    
    float4 rawShadingoutput = gRawShadingOutput[GlobalIndex.xy];
    gShadingOutput[GlobalIndex.xy] = rawShadingoutput;
    
    float3 res = 0.0;
    float3 throughput = float3(1.0f);
    SampleGenerator sg = SampleGenerator.create(GlobalIndex, gFrameIndex + 13 * 13);
    float3 rayOrigin;
    float3 inputDirection;

    {
        ShadingData sd;
        bool valid = PrepareRenderingData(gGBuffer, GlobalIndex.xy, gScene.camera.getPosition(), sd);
        if(!valid || (sd.linearRoughness * sd.linearRoughness > 0.0064f))
        {
           return;
        }
        else
        {
            SimpleBRDF brdf;
            brdf.setup(sd); 
            if(dot(sd.V, sd.N) < 1e-6f)
            {
                return;
            }
            float3 wo = toLocal(sd.V, sd.N);
            float3 wi = float3(-wo.x, -wo.y, wo.z);
            inputDirection = fromLocal(wi, sd.N);
            {
                float LDotH = saturate(dot(inputDirection, sd.N));
                float3 F = evalFresnelSchlick(brdf.specularReflection.albedo, 1, LDotH);
                throughput = throughput * F;
            }
            rayOrigin = computeRayOrigin(sd.posW, sd.frontFacing ? sd.faceN : -sd.faceN);
        }
    }
    
    
    for(int i = 0; i < 4; i++ )
    {
        ShadingData sd;
        bool hit = traceScene(rayOrigin, inputDirection, sd);
        if(!hit)
        break;
        if(dot(sd.V, sd.N) < 1e-6f)
        {
            break;
        }
        rayOrigin = computeRayOrigin(sd.posW, sd.frontFacing ? sd.faceN : -sd.faceN);
        {
            Reservoir r = emptyReservoir();

            float3 selectedL = 0.0f;
            for (int i = 0; i < 4; i++)
            {
                uint lightIndex = gFluxTable.sample(sampleNext2D(sg));
                float lightPdf = gFluxTable.getWeight(lightIndex) / gFluxTable.weightSum;
                float lightInvPdf = 1.0f / lightPdf;
                float3 lightPos = gVirtualLights.getPosition(lightIndex);

                float LPdf;
                float3 L;
                bool coneSampleValid = gVirtualLights.isConeSamplingValid(lightIndex, rayOrigin);
                if (coneSampleValid)
                {
                    gVirtualLights.coneSamplingVSL(lightIndex, sampleNext2D(sg), rayOrigin, L, LPdf);
                }
                else
                {
                    L = cosineWeightedSampling(sampleNext2D(sg), sd.N, LPdf);
                }
                
                float3 Le = gVirtualLights.readLe(lightIndex, L, gSpecRadianceContainer);
                float targetPdf = luminance(gVirtualLights.shadeSurfaceWithVSL(lightIndex, sd, Le, L)) * lightInvPdf / LPdf;
                if (streamSample(r, lightIndex, gVirtualLights.getUV(i, L), sampleNext1D(sg), targetPdf, lightInvPdf / LPdf))
                {
                    selectedL = L;
                }
            }

            finalizeResampling(r, 1.0, r.M);
            r.M = 1;

            if (isValidReservoir(r))
            {
                uint lightIndex = getReservoirLightIndex(r);
                float3 lightPos = gVirtualLights.getPosition(lightIndex);
                if (!isOccluded(rayOrigin, lightPos))
                {
                    // float3 L = gVirtualLights.getDirection(lightIndex, getReservoirSampleUV(r));
                    float3 L = selectedL;
                    float3 Le = gVirtualLights.readLe(lightIndex, L, gSpecRadianceContainer);
                    res += throughput * gVirtualLights.shadeSurfaceWithVSL(lightIndex, sd, Le, L) * getReservoirInvPdf(r) ;
                }
            }
        }
        {
            uint triIndex = gEmissiveTriTable.sample(sampleNext2D(sg));
            float pdf = gEmissiveTriTable.getWeight(triIndex) / gEmissiveTriTable.weightSum;

            TriangleLightSample lightSample;
            bool valid = sampleTriangle(sd.posW, triIndex, sampleNext2D(sg), lightSample);
            if (valid)
            {
                float invPdf = (1.0 / lightSample.pdf) * (1.0 / pdf);
                float3 L = lightSample.dir;
                float3 Le = lightSample.Le;
                float3 V = sd.V;

                if (!isOccluded(rayOrigin, lightSample.posW))
                {
                    SimpleBRDF brdf;
                    brdf.setup(sd);
                    float3 wi = toLocal(L, sd.N);
                    float3 wo = toLocal(V, sd.N);
                    float3 brdfValue = brdf.eval(wo, wi);
                    res += throughput *Le * brdfValue * invPdf;
                }
            }
        }

        if(sd.linearRoughness * sd.linearRoughness > 0.0064f)
        {
            res += throughput * sd.emissive;
            break;        
        }
        else
        {
            {
                SimpleBRDF brdf;
                brdf.setup(sd); 
                float3 wo = toLocal(sd.V, sd.N);
                float3 wi = float3(-wo.x, -wo.y, wo.z);
                {
                    float LDotH = saturate(dot(sd.V, sd.N));
                    float3 F = evalFresnelSchlick(brdf.specularReflection.albedo, 1, LDotH);
                    throughput = throughput * F;
                }
                inputDirection = fromLocal(wi, sd.N);
            }
            continue;
        }
       
    }
    
    gShadingOutput[GlobalIndex.xy].xyz += res;

}
