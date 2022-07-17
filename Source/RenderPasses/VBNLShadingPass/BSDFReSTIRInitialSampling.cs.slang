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
import Utils.Sampling.AliasTable;
import Utils.Debug.PixelDebug;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    ResamplingRuntimeParameters gParams;
    VirtualLightContainer gVirtualLights;
    GBuffer gGBuffer;
    MegaTextureContainer gSpecRadianceContainer;
    uint gOutputBufferIndex;
    AliasTable gFluxTable;
    bool gMISWithPowerSampling;
    int gNumPowerSamples;
    int gNumBSDFSamples;
}

RWStructuredBuffer<Reservoir> gReservoirs;


[numthreads(16, 16, 1)]
void main(uint2 GlobalIndex : SV_DispatchThreadID, uint3 groupIndex: SV_GROUPID)
{
    if (any(GlobalIndex.xy >= gViewportDims)) 
    {
        return;
    }

    SampleGenerator sg = SampleGenerator.create(GlobalIndex, gFrameIndex + 1 * 13);
    Reservoir r = emptyReservoir();

    printSetPixel(GlobalIndex);
    ShadingData sd;
    bool valid = LoadShadingData(gGBuffer, GlobalIndex.xy, gViewportDims, gScene.camera, sd);
    if (valid)
    {
        SimpleBRDF simpleBRDF;
        simpleBRDF.setup(sd);
        float3 wo = toLocal(sd.V, sd.N);
        float3 rayOrigin = sd.computeNewRayOrigin();

        // generate power samples
        if (gMISWithPowerSampling)
        {
            for (int powerSampleId = 0; powerSampleId < gNumPowerSamples; powerSampleId++)
            {
                int lightIndex = gFluxTable.sample(sampleNext2D(sg));
                float lightInvPdf = gFluxTable.weightSum / gFluxTable.getWeight(lightIndex);

                // perform cone sampling
                float pdfBrdf;
                float3 sampledDir;
                float conePdf;
                gVirtualLights.sampleVSL(lightIndex, sg, sd.posW, sd.N, sampledDir, conePdf);
                float3 wi = toLocal(sampledDir, sd.N);

                ShadingData nextSd;
                TriangleHit trianglehit;
                float3 vplPosition = gVirtualLights.getPosition(lightIndex);
                float vplRadius = gVirtualLights.getVSLRadius(lightIndex);
                float tmax = length(vplPosition - rayOrigin) + vplRadius;

                if (castScene(rayOrigin, sampledDir, nextSd, trianglehit, tmax))
                {
                    uint lightsFound = 0;
                    uint foundLightIndces[8];
                    float3 Le = gVirtualLights.readRadianceByPosition(sd.posW, nextSd.posW, nextSd.N, vplPosition, vplRadius, lightsFound, foundLightIndces, gSpecRadianceContainer);
                    float3 res = simpleBRDF.eval(wo, wi) * Le;
                    float targetPdf = luminance(res);

                    if (targetPdf > 0)
                    {
                        // compute source PDF
                        float pdf_sum = 0.f;
                        for (int j = 0; j < lightsFound; j++)
                        {
                            int curLightID = foundLightIndces[j];

                            float lightPowerPdf = gFluxTable.getWeight(curLightID) / gFluxTable.weightSum;
                            // cone pdf
                            float lightConePdf = gVirtualLights.evaluateVirtualLightSamplingPDF(sd, curLightID, sampledDir);

                            pdf_sum += lightPowerPdf * lightConePdf * gNumPowerSamples;
                        }

                        // also add BSDF sampling PDF
                        pdf_sum += simpleBRDF.evalPdf(wo, wi) * gNumBSDFSamples;

                        streamSample(r, nextSd.posW, nextSd.N, sampleNext1D(sg), targetPdf, 1.0 / pdf_sum);
                    }
                }
            }
        }


        // generate BSDF samples

        for (uint bsdfSampleId = 0; bsdfSampleId < gNumBSDFSamples; bsdfSampleId++)
        {
            float pdfBrdf;
            float3 sampledDir;
            uint lobeType;
            bool suc = simpleBRDF.sample(sd, sampledDir, pdfBrdf, lobeType, sg);
            float3 wi = toLocal(sampledDir, sd.N);
            if (!suc || min(sd.NdotV, dot(sampledDir, sd.N)) < 1e-6f)
            {
                r.M += 1;
            }
            else
            {
                ShadingData nextSd;
                if (traceScene(rayOrigin, sampledDir, nextSd))
                {
                    uint lightsFound = 0;
                    uint foundLightIndces[8];
                    float3 Le = gVirtualLights.readRadianceByPosition(sd.posW, nextSd.posW, nextSd.N, float3(0), -1.f, lightsFound, foundLightIndces, gSpecRadianceContainer);
                    float3 res = simpleBRDF.eval(wo, wi) * Le;
                    float targetPdf = luminance(res);

                    if (targetPdf > 0)
                    {
                        // compute source PDF
                        float pdf_sum = 0.f;

                        if (gMISWithPowerSampling)
                        {
                            for (int j = 0; j < lightsFound; j++)
                            {
                                int curLightID = foundLightIndces[j];
                                float lightPowerPdf = gFluxTable.getWeight(curLightID) / gFluxTable.weightSum;
                                // cone pdf
                                float lightConePdf = gVirtualLights.evaluateVirtualLightSamplingPDF(sd, curLightID, sampledDir);

                                pdf_sum += lightPowerPdf * lightConePdf * gNumPowerSamples;
                            }
                        }
                        // also add BSDF sampling PDF
                        pdf_sum += pdfBrdf * gNumBSDFSamples;

                        streamSample(r, nextSd.posW, nextSd.N, sampleNext1D(sg), targetPdf, 1.0 / pdf_sum);
                    }
                }
            }
        }

        r.M = 1;
        finalizeResampling(r, 1.0, r.M);
    }

    storeReservoir(gParams, r, gReservoirs, GlobalIndex.xy, gOutputBufferIndex);
}
