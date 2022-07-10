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
import Utils.Sampling.AliasTable;
import Utils.Debug.PixelDebug;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    ResamplingRuntimeParameters gParams;
    VirtualLightContainer gVirtualLights;
    GBuffer gGBuffer;
    uint gInputBufferIndex;
    AliasTable gEmissiveTriTable;
    AliasTable gFluxTable;
    uint gShadingMode;
    bool gEnableDirectLighting;
}

RWStructuredBuffer<Reservoir> gReservoirs;
RWTexture2D<float4> gShadingOutput;

float3 evalDirectLighting(inout SampleGenerator sg, ShadingData sd)
{
    float3 res = 0.0;
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
        float3 posWOrigin = computeRayOrigin(sd.posW, sd.faceN);
        if (!isOccluded(posWOrigin, lightSample.posW, 0.00001f))
        {
            SimpleBRDF brdf;
            brdf.setup(sd);
            float3 wi = toLocal(L, sd.N);
            float3 wo = toLocal(V, sd.N);
            float3 brdfValue = brdf.eval(wo, wi);
            res += Le * brdfValue * invPdf;
        }
    }
    return res;
}

[numthreads(16, 16, 1)]
void main(uint2 GlobalIndex : SV_DispatchThreadID, uint3 groupIndex: SV_GROUPID)
{
    if (any(GlobalIndex.xy >= gViewportDims)) 
    {
        return;
    }

    float3 res = 0.0;

    SampleGenerator sg = SampleGenerator.create(GlobalIndex, gFrameIndex);
    ShadingData sd;
    bool valid = PrepareRenderingData(gGBuffer, GlobalIndex.xy, gScene.camera.getPosition(), sd);
    float3 rayOrigin = computeRayOrigin(sd.posW, sd.frontFacing ? sd.faceN : -sd.faceN);
    if (valid)
    {
        SimpleBRDF simpleBRDF;
        simpleBRDF.setup(sd);

        res += sd.emissive;
        if (gEnableDirectLighting)
        {
            res += evalDirectLighting(sg, sd);
        }

        // Reservoir r = loadReservoir(gParams, gReservoirs, GlobalIndex.xy, gInputBufferIndex);
        // if (isValidReservoir(r))
        // {
        //     uint lightIndex = getReservoirLightIndex(r);
        //     float3 rayOrigin = computeRayOrigin(sd.posW, sd.frontFacing ? sd.faceN : -sd.faceN);
        //     float3 lightPos = gVirtualLights.getPosition(lightIndex);
        //     if (!isOccluded(rayOrigin, lightPos))
        //     {
        //         float LPdf; 
        //         res += getReservoirInvPdf(r) * gVirtualLights.coneComputeShadingPointRadianceForVPL(sd, sg, lightIndex, true);
        //     }
        // }

        uint vplIndex = gFluxTable.sample(sampleNext2D(sg));
        float vplInvPdf = gFluxTable.weightSum / gFluxTable.getWeight(vplIndex);
        res += gVirtualLights.evalVPL(vplIndex, sd) * vplInvPdf;

        if (any(isnan(res)) || any(isinf(res)))
        {
            res = 0.0;
        }
    }

    gShadingOutput[GlobalIndex] = float4(res, 1.0);
}
