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
import Experimental.Scene.Material.MaterialShading;
import Experimental.Scene.Lights.LightCollection;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Utils.Debug.PixelDebug;
import Scene.Material.SimpleBRDF;

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
}

static const int maxHistoryLength = 2;
static const float depthThreshold = 0.01f;
static const float normalThreshold = 0.8f;

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
    printSetPixel(GlobalIndex);

    ShadingData sd;
    if (LoadShadingData(gGBuffer, GlobalIndex.xy, gViewportDims, gScene.camera, sd))
    {
        SimpleBRDF simpleBRDF;
        simpleBRDF.setup(sd);

        Reservoir curSample = loadReservoir(gParams, gReservoirs, pixelPosition, gInputBufferIndex);
        
        int historyLimit = min(Reservoir::c_MaxM, maxHistoryLength * curSample.M);

        Reservoir state = emptyReservoir();
        combineReservoirs(state, curSample, 0.5f, curSample.targetPdf);

        float3 selectedPos = curSample.position;
        float3 selectedNormal = curSample.normal;
        float3 distNeighbor = sd.posW - selectedPos;
        float selectedJacobianTerm = dot(distNeighbor, distNeighbor) / max(0.f, dot(normalize(distNeighbor), selectedNormal));

        float selectedJacobian = 1;
        bool selectedPrev = false;

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

            // if (!gGBuffer.IsReprjValid(pixelPosition, idx, true, normalThreshold, depthThreshold))
            // {
            //    continue;
            // }

            prevPos = idx;
            foundNeighbor = true;
            break;
        }

        Reservoir prevSample;

        if (foundNeighbor)
        {
            prevSample = loadReservoir(gParams, gReservoirs, prevPos, gHistoryBufferIndex);
            prevSample.M = min(prevSample.M, historyLimit);

            // temporal resampling

            if (isValidReservoir(prevSample))
            {
                float3 samplePos = prevSample.position;
                float3 sampleNormal = prevSample.normal;
                float3 sampledDir = normalize(prevSample.position - sd.posW);
                float3 wo = toLocal(sd.V, sd.N);
                float3 wi = toLocal(sampledDir, sd.N);

                // skip visibility test (probably consistent)
                float3 Le = gVirtualLights.readRadianceByPosition(sd.posW, samplePos, sampleNormal, gSpecRadianceContainer);
                float3 res = simpleBRDF.eval(wo, wi) * Le;
                float targetPdf = luminance(res);

                // compute jacobian
                float3 distNeighbor = neighborSd.posW - prevSample.position;
                float3 distCurrent = sd.posW - prevSample.position;
                float neighborJacobianTerm = dot(distNeighbor, distNeighbor) / max(0.f, dot(normalize(distNeighbor), sampleNormal));
                float currentJacobianTerm = dot(distCurrent, distCurrent) / max(0.f, dot(normalize(distCurrent), sampleNormal));
                float jacobian = neighborJacobianTerm / currentJacobianTerm;
                if (isnan(jacobian) || isinf(jacobian)) jacobian = 0.f;

                if (isinf(prevSample.weightSum) || isnan(prevSample.weightSum)) prevSample.weightSum = 0.f;
                if (combineReservoirs(state, prevSample, sampleNext1D(sg), targetPdf, jacobian))
                {
                    selectedJacobianTerm = neighborJacobianTerm;
                    selectedJacobian = jacobian;
                    selectedPrev = true;
                }

                // no resampling
                //float weightAtCurrent = prevSample.targetPdf;
                //combineReservoirs(state, prevSample, sampleNext1D(sg), weightAtCurrent);
            }
            else
            {
                state.M += prevSample.M;
            }
        }

        //TODO: bias correction?

        if (isValidReservoir(state))
        {
            float pi = state.targetPdf * selectedJacobian;
            float piSum = state.targetPdf * selectedJacobian * curSample.M;
            uint count = curSample.M;

            if (selectedPrev)
            {
                pi = prevSample.targetPdf;
                piSum += pi * prevSample.M;
                count += prevSample.M;
            }
            else if (foundNeighbor)
            {
                float3 sampledDir = normalize(selectedPos - neighborSd.posW);
                float ps = 0.0f;

                if (!isOccluded(neighborSd.computeNewRayOrigin(), selectedPos, 0.001))
                {
                    SimpleBRDF prevBRDF;
                    prevBRDF.setup(neighborSd);

                    float3 wo = toLocal(neighborSd.V, neighborSd.N);
                    float3 wi = toLocal(sampledDir, neighborSd.N);
                    float3 Le = gVirtualLights.readRadianceByPosition(neighborSd.posW, selectedPos, selectedNormal, gSpecRadianceContainer);
                    float3 res = prevBRDF.eval(wo, wi) * Le;
                    ps = luminance(res);

                    float3 distCurrent = neighborSd.posW - selectedPos;
                    float currentJacobianTerm = dot(distCurrent, distCurrent) / max(0.f, dot(normalize(distCurrent), selectedNormal));
                    float jacobian = selectedJacobianTerm / currentJacobianTerm;
                    if (isnan(jacobian) || isinf(jacobian)) jacobian = 0.f;
                    ps *= jacobian;
                }

                //ShadingData nextSd;
                //if (traceScene(rayOrigin, sampledDir, nextSd))
                //{
                //    SimpleBRDF prevBRDF;
                //    prevBRDF.setup(neighborSd);

                //    float3 wo = toLocal(neighborSd.V, neighborSd.N);
                //    float3 wi = toLocal(sampledDir, neighborSd.N);
                //    float3 Le = gVirtualLights.readRadianceByPosition(neighborSd.posW, nextSd.posW, nextSd.N, gSpecRadianceContainer);
                //    float3 res = prevBRDF.eval(wo, wi) * Le;
                //    ps = luminance(res);

                //    float3 distCurrent = neighborSd.posW - selectedPos;
                //    float currentJacobianTerm = dot(distCurrent, distCurrent) / max(0.f, dot(normalize(distCurrent), selectedNormal));
                //    float jacobian = selectedJacobianTerm / currentJacobianTerm;
                //    if (isnan(jacobian) || isinf(jacobian)) jacobian = 0.f;
                //    ps *= jacobian;
                //}

                if (ps > 0) count += prevSample.M;
                piSum += ps * prevSample.M;
            }

            finalizeResampling(state, pi, piSum);
        }

        // biased
        //finalizeResampling(state, 1.0, state.M);

        temporalResult = state;
    }

    storeReservoir(gParams, temporalResult, gReservoirs, GlobalIndex.xy, gOutputBufferIndex);
}
