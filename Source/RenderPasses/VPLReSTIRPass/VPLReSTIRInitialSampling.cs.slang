#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Sampling.ReSTIR;
import Utils.Helpers;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
import Utils.VirtualLight.VirtualLightContainer;
import RenderPasses.GBufferUtils;
import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Lights.Lights;
import Scene.Shading;
import Scene.RaytracingInlineHelper;
import Scene.Material.SimpleBRDF;
import Experimental.Scene.Material.MaterialShading;
import Experimental.Scene.Lights.LightCollection;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Utils.Sampling.AliasTable;
import Utils.Debug.PixelDebug;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    ResamplingRuntimeParameters gParams;
    VirtualLightContainer gVirtualLights;
    GBuffer gGBuffer;
    uint gOutputBufferIndex;
    AliasTable gFluxTable;
}

RWStructuredBuffer<Reservoir> gReservoirs;

[numthreads(16, 16, 1)]
void main(uint2 GlobalIndex : SV_DispatchThreadID, uint3 groupIndex: SV_GROUPID)
{
    if (any(GlobalIndex.xy >= gViewportDims)) 
    {
        return;
    }

    static const uint numInitialSamples = 8;
#ifdef _VSL_EVALUATION
    static const bool kUseVSL = true;
#else
    static const bool kUseVSL = false;
#endif

    SampleGenerator sg = SampleGenerator.create(GlobalIndex, gFrameIndex + 1 * 13);
    Reservoir r = emptyReservoir();

    ShadingData sd;
    bool valid = PrepareRenderingData(gGBuffer, GlobalIndex.xy, gScene.camera.getPosition(), sd);
    if (valid)
    {
        float selectedLPdf = 1.f;
        for(uint i = 0; i < numInitialSamples; i++)
        {
            uint lightIndex = gFluxTable.sample(sampleNext2D(sg));
            float lightInvPdf = gFluxTable.weightSum / gFluxTable.getWeight(lightIndex);

            float2 VSLrand = sampleNext2D(sg);
            float ConePdf = 1.f;
            float3 ConeDir = 0.f;
            float targetPdf = luminance(gVirtualLights.coneComputeShadingPointRadianceForVPL(sd, sg, lightIndex));
            float MISTotalProb = numInitialSamples * ConePdf;
            if (streamSample(r, lightIndex, VSLrand, sampleNext1D(sg), targetPdf, lightInvPdf / MISTotalProb))
            {
                selectedLPdf = ConePdf;
            }
        }

        if (isValidReservoir(r))
        {
            float3 rayOrigin = computeRayOrigin(sd.posW, sd.frontFacing ? sd.faceN : -sd.faceN);
            uint lightIndex = getReservoirLightIndex(r);
            float3 lightPos = gVirtualLights.getPosition(lightIndex);

            if (isOccluded(rayOrigin, lightPos))
            {
                r.lightData = 0;
                r.weightSum = 0;
            }
        }

        r.targetPdf /= selectedLPdf; // convert to primary sample space (assuming cone sampling technique) for reusing random numbers

        r.M = 1;
        finalizeResampling(r, 1.0, r.M);
    }

    storeReservoir(gParams, r, gReservoirs, GlobalIndex.xy, gOutputBufferIndex);
}
