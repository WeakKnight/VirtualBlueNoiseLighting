#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import BSDFReSTIR;
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
import Scene.Material.SimpleBRDF;
import Experimental.Scene.Material.MaterialShading;
import Experimental.Scene.Lights.LightCollection;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Utils.Debug.PixelDebug;

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
}

static const uint numSamples = 2;
static const uint numDisocclusionBoostSamples = 2;
static const uint targetHistoryLength = 20;
static const float samplingRadius = 20.0f;
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
        SimpleBRDF simpleBRDF;
        simpleBRDF.setup(sd);

        Reservoir centerSample = loadReservoir(gParams, gReservoirs, pixelPosition, gInputBufferIndex);
        Reservoir state = emptyReservoir();
        
        int selected = -1;
        float3 selectedPos;
        float3 selectedNormal;
        float selectedJacobianTerm = 1;
        float selectedJacobian = 1;
        if (isValidReservoir(centerSample))
        {
            selectedPos = centerSample.position;
            selectedNormal = centerSample.normal;

            float3 distNeighbor = sd.posW - selectedPos;
            selectedJacobianTerm = dot(distNeighbor, distNeighbor) / max(0.f, dot(normalize(distNeighbor), selectedNormal));
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

            if (isValidReservoir(neighborSample))
            {
                float3 samplePos = neighborSample.position;
                float3 sampleNormal = neighborSample.normal;
                float3 sampledDir = normalize(neighborSample.position - sd.posW);
                float3 wo = toLocal(sd.V, sd.N);
                float3 wi = toLocal(sampledDir, sd.N);

                if (isOccluded(sd.computeNewRayOrigin(), neighborSample.position, 0.001))
                {
                    state.M += neighborSample.M;
                }
                else
                {
                    float3 Le = gVirtualLights.readRadianceByPosition(sd.posW, samplePos, sampleNormal, gSpecRadianceContainer);

                    float3 res = simpleBRDF.eval(wo, wi) * Le;
                    float targetPdf = luminance(res);

                    // compute jacobian
                    float3 distNeighbor = neighborSd.posW - neighborSample.position;
                    float3 distCurrent = sd.posW - neighborSample.position;
                    float neighborJacobianTerm = dot(distNeighbor, distNeighbor) / max(0.f, dot(normalize(distNeighbor), sampleNormal));
                    float currentJacobianTerm = dot(distCurrent, distCurrent) / max(0.f, dot(normalize(distCurrent), sampleNormal));
                    float jacobian = neighborJacobianTerm / currentJacobianTerm;
                    if (isnan(jacobian) || isinf(jacobian)) jacobian = 0.f;

                    if (combineReservoirs(state, neighborSample, sampleNext1D(sg), targetPdf, jacobian))
                    {
                        selected = i;
                        selectedPos = neighborSample.position;
                        selectedNormal = neighborSample.normal;
                        selectedJacobianTerm = neighborJacobianTerm;
                        selectedJacobian = jacobian;
                    }
                }
            }
            else
            {
                state.M += neighborSample.M;
            }
        }

        if (isValidReservoir(state))
        {
            float pi = state.targetPdf * selectedJacobian;
            float piSum = state.targetPdf * selectedJacobian * centerSample.M;
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

                float3 sampledDir = normalize(selectedPos - neighborSd.posW);
                ShadingData nextSd;
                float ps = 0.0f;

                if (!isOccluded(neighborSd.computeNewRayOrigin(), selectedPos, 0.001))
                {
                    float3 wo = toLocal(neighborSd.V, neighborSd.N);
                    float3 wi = toLocal(sampledDir, neighborSd.N);
                    float3 Le = gVirtualLights.readRadianceByPosition(neighborSd.posW, selectedPos, selectedNormal, gSpecRadianceContainer);
                    float3 res = simpleBRDF.eval(wo, wi) * Le;
                    ps = luminance(res);

                    //if (traceScene(rayOrigin, sampledDir, nextSd))
                    //{
                    //    float3 wo = toLocal(neighborSd.V, neighborSd.N);
                    //    float3 wi = toLocal(sampledDir, neighborSd.N);
                    //    
                    //    float3 Le = gVirtualLights.readRadianceByPosition(neighborSd.posW, nextSd.posW, nextSd.N, gSpecRadianceContainer);
                    //    float3 res = simpleBRDF.eval(wo, wi) * Le;
                    //    ps = luminance(res);
                    //}

                    // compute jacobian
                    float3 distCurrent = neighborSd.posW - selectedPos;
                    float currentJacobianTerm = dot(distCurrent, distCurrent) / max(0.f, dot(normalize(distCurrent), selectedNormal));
                    float jacobian = selectedJacobianTerm / currentJacobianTerm;
                    if (isnan(jacobian) || isinf(jacobian)) jacobian = 0.f;
                    ps *= jacobian;

                    Reservoir neighborSample = loadReservoir(gParams, gReservoirs, idx, gInputBufferIndex);

                    if (ps > 0)
                    {
                        count += neighborSample.M;
                        piSum += ps * neighborSample.M;
                    }
                }

                pi = (selected == i ? ps : pi);
            }

            finalizeResampling(state, pi, piSum);
        }

        spatialResult = state;
    }

    storeReservoir(gParams, spatialResult, gReservoirs, GlobalIndex.xy, gOutputBufferIndex);
}
