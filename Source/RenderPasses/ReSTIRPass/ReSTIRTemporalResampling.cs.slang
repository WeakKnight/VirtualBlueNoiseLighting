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
import Experimental.Scene.Material.MaterialShading;
import Experimental.Scene.Lights.LightCollection;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    ResamplingRuntimeParameters gParams;
    VirtualLightContainer gVirtualLights;
    MegaTextureContainer gSpecRadianceContainer;
    GBuffer gGBuffer;
    GBuffer gPrevFrameGBuffer;
    uint gInputBufferIndex;
    uint gHistoryBufferIndex;
    uint gOutputBufferIndex;
    bool gPointSampling;
    uint gNumConeSamples;
    uint gNumMinConeSamples;
    uint maxHistoryLength;
}

static const float depthThreshold = 0.1f;
static const float normalThreshold = 0.5f;

RWStructuredBuffer<Reservoir> gReservoirs;

[numthreads(16, 16, 1)]
void main(uint2 GlobalIndex : SV_DispatchThreadID, uint3 groupIndex: SV_GROUPID)
{
    if (any(GlobalIndex.xy >= gViewportDims)) 
    {
        return;
    }

    uint2 pixelPosition = GlobalIndex.xy;
    SampleGenerator sg = SampleGenerator.create(GlobalIndex, gFrameIndex + 2 * 13);
    Reservoir temporalResult = emptyReservoir();
    
    ShadingData sd;
    if (LoadShadingData(gGBuffer, GlobalIndex.xy, gViewportDims, gScene.camera, sd))
    {
        Reservoir curSample = loadReservoir(gParams, gReservoirs, pixelPosition, gInputBufferIndex);

        int historyLimit = min(Reservoir::c_MaxM, maxHistoryLength * curSample.M);
        int selectedLightPrevID = -1;
        if (isValidReservoir(curSample))
        {
            selectedLightPrevID = getReservoirLightIndex(curSample);
        }

        Reservoir state = emptyReservoir();
        combineReservoirs(state, curSample, 0.5f, curSample.targetPdf);

        float2 motion = gGBuffer.GetMotionVector(pixelPosition);
        motion.xy += (sampleNext2D(sg) - 0.5);
        float2 reprojectedSamplePosition = float2(pixelPosition) + motion.xy;
        int2 prevPos = int2(round(reprojectedSamplePosition));

        ShadingData neighborSd;
        bool foundNeighbor = false;
        const float radius = 4;

        for (int i = 0; i < 9; i++)
        {
            int2 offset = 0;
            if(i > 0)
            {
                offset.x = int((sampleNext1D(sg) - 0.5) * radius);
                offset.y = int((sampleNext1D(sg) - 0.5) * radius);
            }

            int2 idx = prevPos + offset;

            if (!LoadShadingData(gPrevFrameGBuffer, idx, gViewportDims, gScene.camera, neighborSd))
            {
                continue;
            }

            if (!gGBuffer.IsReprjValid(pixelPosition, idx, true, normalThreshold, depthThreshold))
            {
                continue;
            }

            prevPos = idx;
            foundNeighbor = true;
            break;
        }

        if (foundNeighbor)
        {
            Reservoir prevSample = loadReservoir(gParams, gReservoirs, prevPos, gHistoryBufferIndex);
            prevSample.M = min(prevSample.M, historyLimit);

            //float2 prevUV = unpackUV(prevSample.uvData);

            float weightAtCurrent = 0.0f;
            //if (isValidReservoir(prevSample))
            //{
            //    uint lightIndex = getReservoirLightIndex(prevSample);
            //    float LPdf; float3 dummy;
            //    float targetPdf;
            //    if (gPointSampling)
            //    {
            //        targetPdf = luminance(gVirtualLights.coneSampleShadingPointRadiance(sd, prevUV, lightIndex, LPdf, dummy, gSpecRadianceContainer));
            //    }
            //    else
            //    {
            //        targetPdf = luminance(gVirtualLights.coneComputeShadingPointRadiance(sd, sg, gNumMinConeSamples, gNumConeSamples, lightIndex, gSpecRadianceContainer));
            //        LPdf = 1.f;
            //    }

            //    weightAtCurrent = targetPdf / LPdf; // incoprate Jacobian
            //}
            weightAtCurrent = prevSample.targetPdf;

            //prevSample.uvData = packUV(prevUV);

            if (isinf(prevSample.weightSum) || isnan(prevSample.weightSum)) prevSample.weightSum = 0.f;

            combineReservoirs(state, prevSample, sampleNext1D(sg), weightAtCurrent);
        }

        //TODO: bias correction?

        finalizeResampling(state, 1.0, state.M);
        temporalResult = state;
    }

    storeReservoir(gParams, temporalResult, gReservoirs, GlobalIndex.xy, gOutputBufferIndex);
}
