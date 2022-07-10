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
    uint gInputBufferIndex;
    uint gOutputBufferIndex;
}

static const uint numSamples = 2;
static const uint numDisocclusionBoostSamples = 2;
static const uint targetHistoryLength = 20;
static const float samplingRadius = 10.0f;
static const float depthThreshold = 0.1f;
static const float normalThreshold = 0.5f;
static const uint neighborOffsetMask = 8191;

RWStructuredBuffer<Reservoir> gReservoirs;
Buffer<float2> gNeighborOffsetBuffer;

[numthreads(16, 16, 1)]
void main(uint2 GlobalIndex : SV_DispatchThreadID, uint3 groupIndex: SV_GROUPID)
{
    if (any(GlobalIndex.xy >= gViewportDims)) 
    {
        return;
    }
    uint2 pixelPosition = GlobalIndex.xy;
    SampleGenerator sg = SampleGenerator.create(GlobalIndex, gFrameIndex + 3 * 13);
    Reservoir spatialResult = emptyReservoir();
    
    ShadingData sd;
    if (PrepareRenderingData(gGBuffer, pixelPosition, gScene.camera.getPosition(), sd))
    {
        Reservoir centerSample = loadReservoir(gParams, gReservoirs, pixelPosition, gInputBufferIndex);
        uint centerLightIndex = getReservoirLightIndex(centerSample);

        Reservoir state = emptyReservoir();
        
        int selected = -1;
        uint selectedLight;
        if (isValidReservoir(centerSample))
        {
            selectedLight = getReservoirLightIndex(centerSample);
        }

        combineReservoirs(state, centerSample, 0.5f, centerSample.targetPdf);

        int numSpatialSamples = numSamples;
        if (centerSample.M < targetHistoryLength)
        {
            numSpatialSamples = max(numSpatialSamples, numDisocclusionBoostSamples);
        }
        numSpatialSamples = min(numSpatialSamples, 32);

        uint startIdx = sampleNext1D(sg) * neighborOffsetMask;
        uint cachedResult = 0;
        int i;
        float m_c = 1.f;
        int validSamples = 1;
        for (i = 0; i < numSpatialSamples; i++)
        {
            uint sampleIdx = (startIdx + i) & neighborOffsetMask;
            int2 spatialOffset = int2(float2(gNeighborOffsetBuffer[sampleIdx].xy) * samplingRadius);
            int2 idx = pixelPosition + spatialOffset;

            ShadingData neighborSd;
            if (!PrepareRenderingData(gGBuffer, idx, gScene.camera.getPosition(), neighborSd))
            {
                continue;
            }
            
            if (!gGBuffer.IsReprjValid(pixelPosition, idx, false, normalThreshold, depthThreshold))
            {
                continue;
            }

            if (!AreMaterialsSimilar(sd, neighborSd))
            {
                continue;
            }

            cachedResult |= (1u << uint(i));
            Reservoir neighborSample = loadReservoir(gParams, gReservoirs, idx, gInputBufferIndex);
            validSamples++;

            if (isValidReservoir(neighborSample))
            {
                uint lightIndex = getReservoirLightIndex(neighborSample);
                float targetPdf = luminance(gVirtualLights.coneComputeShadingPointRadianceForVPL(sd, sg, lightIndex));
                if (combineReservoirs(state, neighborSample, sampleNext1D(sg), targetPdf))
                {
                    selected = i;
                    selectedLight = lightIndex;
                }
            }
            else
            {
                state.M += neighborSample.M;
            }
        }

        if (isValidReservoir(state))
        {
            float pi = state.targetPdf;
            float piSum = state.targetPdf * centerSample.M;
            uint count = centerSample.M;
            for (i = 0; i < numSpatialSamples; i++)
            {
                if ((cachedResult & (1u << uint(i))) == 0) 
                {
                    continue;
                }
                uint sampleIdx = (startIdx + i) & neighborOffsetMask;
                int2 idx = pixelPosition + int2(float2(gNeighborOffsetBuffer[sampleIdx].xy) * samplingRadius);
            
                ShadingData neighborSd;
                PrepareRenderingData(gGBuffer, idx, gScene.camera.getPosition(), neighborSd);

                float targetPdf = luminance(gVirtualLights.coneComputeShadingPointRadianceForVPL(neighborSd, sg, selectedLight));
                float ps = targetPdf;
                
                float3 rayOrigin = computeRayOrigin(neighborSd.posW, neighborSd.frontFacing ? neighborSd.faceN : -neighborSd.faceN);
                if (isOccluded(rayOrigin, gVirtualLights.getPosition(selectedLight)))
                {
                    ps = 0;
                }

                Reservoir neighborSample = loadReservoir(gParams, gReservoirs, idx, gInputBufferIndex);
                if (ps > 0)
                {
                    count += neighborSample.M;
                }

                pi = (selected == i ? ps : pi);
                piSum += ps * neighborSample.M;
            }
            finalizeResampling(state, pi, piSum);
        }
        else
        {
            finalizeResampling(state, 1.f, state.M);
        }

        spatialResult = state;
    }

    storeReservoir(gParams, spatialResult, gReservoirs, GlobalIndex.xy, gOutputBufferIndex);
}
