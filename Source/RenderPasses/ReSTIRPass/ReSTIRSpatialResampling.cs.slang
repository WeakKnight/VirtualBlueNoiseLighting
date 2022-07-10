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
    uint gInputBufferIndex;
    uint gOutputBufferIndex;
    bool gPointSampling;
    uint gNumConeSamples;
    uint gNumMinConeSamples;
    bool gPairwiseMIS;
    bool gBiasCorrectionForReuse;
}

static const uint numSamples = 4;
static const uint numDisocclusionBoostSamples = 4;
static const uint targetHistoryLength = 20;
static const float samplingRadius = 10;
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
    if (LoadShadingData(gGBuffer, pixelPosition, gViewportDims, gScene.camera, sd))
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

        if (!gPairwiseMIS)
        {
            combineReservoirs(state, centerSample, 0.5f, centerSample.targetPdf);
        }

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
            if (!LoadShadingData(gGBuffer, idx, gViewportDims, gScene.camera, neighborSd))
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

            if (gPairwiseMIS)
            {
                float centerSampleTargetPdfAtNeighbor;

                bool occluded = false;
#if !defined(_VSL_EVALUATION) || !SCATTER_VISIBILITY
                if (gBiasCorrectionForReuse)
                {
                    float3 rayOrigin = computeRayOrigin(neighborSd.posW, neighborSd.frontFacing ? neighborSd.faceN : -neighborSd.faceN);
                    occluded = isOccluded(rayOrigin, gVirtualLights.getPosition(centerLightIndex));
                }
#endif

                if (occluded) centerSampleTargetPdfAtNeighbor = 0.f;
                else
                {
                    if (gPointSampling)
                    {
                        float LPdf; float3 dummy;
                        centerSampleTargetPdfAtNeighbor = luminance(gVirtualLights.coneSampleShadingPointRadiance(neighborSd, unpackUV(centerSample.uvData), centerLightIndex, LPdf, dummy, gSpecRadianceContainer));
                        centerSampleTargetPdfAtNeighbor /= LPdf;
                    }
                    else
                    {
                        centerSampleTargetPdfAtNeighbor = luminance(gVirtualLights.coneComputeShadingPointRadiance(neighborSd, sg, gNumMinConeSamples, gNumConeSamples, centerLightIndex, gSpecRadianceContainer));
                    }
                }

                m_c += 1 - (centerSampleTargetPdfAtNeighbor / (centerSample.targetPdf / numSpatialSamples + centerSampleTargetPdfAtNeighbor));
            }

            if (isValidReservoir(neighborSample))
            {
                uint lightIndex = getReservoirLightIndex(neighborSample);

                bool occluded = false;

#if !defined(_VSL_EVALUATION) || !SCATTER_VISIBILITY
                if (gPairwiseMIS && gBiasCorrectionForReuse)
                {
                    float3 rayOrigin = sd.computeNewRayOrigin();
                    occluded = isOccluded(rayOrigin, gVirtualLights.getPosition(lightIndex));
                }
#endif

                float targetPdf;

                if (occluded) targetPdf = 0.f;
                else
                {
                    if (gPointSampling)
                    {
                        float LPdf; float3 dummy;
                        targetPdf = luminance(gVirtualLights.coneSampleShadingPointRadiance(sd, unpackUV(neighborSample.uvData), lightIndex, LPdf, dummy, gSpecRadianceContainer));
                        targetPdf /= LPdf;// incoprate Jacobian
                    }
                    else
                    {
                        targetPdf = luminance(gVirtualLights.coneComputeShadingPointRadiance(sd, sg, gNumMinConeSamples, gNumConeSamples, lightIndex, gSpecRadianceContainer));
                    }
                }

                float m_n = 1.f;
                if (gPairwiseMIS)
                {
                    m_n = neighborSample.targetPdf / (neighborSample.targetPdf + targetPdf / numSpatialSamples);
                    m_n /= neighborSample.M;
                    if (isnan(m_n) || isinf(m_n)) m_n = 0.f;
                }

                if (combineReservoirs(state, neighborSample, sampleNext1D(sg), targetPdf, m_n))
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

        if (gPairwiseMIS)
        {
            m_c /= centerSample.M;
            if (isnan(m_c) || isinf(m_c)) m_c = 0.f;
            combineReservoirs(state, centerSample, sampleNext1D(sg), centerSample.targetPdf, m_c);
            finalizeResampling(state, 1.f, validSamples);
        }
        else if (isValidReservoir(state) && gBiasCorrectionForReuse)
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
                LoadShadingData(gGBuffer, idx, gViewportDims, gScene.camera, neighborSd);

                float targetPdf;
                if (gPointSampling)
                {
                    float LPdf; float3 dummy;
                    targetPdf = luminance(gVirtualLights.coneSampleShadingPointRadiance(neighborSd, unpackUV(state.uvData), selectedLight, LPdf, dummy, gSpecRadianceContainer));
                    targetPdf /= LPdf;
                }
                else
                {
                    targetPdf = luminance(gVirtualLights.coneComputeShadingPointRadiance(neighborSd, sg, gNumMinConeSamples, gNumConeSamples, selectedLight, gSpecRadianceContainer));
                }

                float ps = targetPdf;
                
                float3 rayOrigin = neighborSd.computeNewRayOrigin();
#if !defined(_VSL_EVALUATION) || !SCATTER_VISIBILITY
                if (isOccluded(rayOrigin, gVirtualLights.getPosition(selectedLight)))
                {
                    ps = 0;
                }
#endif

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
