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
    bool MISForVSL;
    bool useSelfGenSamples;
    uint tileSearchRadius;
    uint numInitialSamples;
    uint numTileVPLSamples;
    AliasTable gFluxTable;
    uint gTileSampleNum;
    uint gTileSize;
    uint2 gTileDims;
    bool gPointSampling;
    uint gNumConeSamples;
    uint gNumMinConeSamples;
}

StructuredBuffer<uint> gTileVirtualLightContainer;
StructuredBuffer<uint> gTileVirtualLightWeights;

StructuredBuffer<uint2> gRISBuffer;
RWStructuredBuffer<Reservoir> gReservoirs;

[numthreads(16, 16, 1)]
void main(uint2 GlobalIndex : SV_DispatchThreadID, uint3 groupIndex: SV_GROUPID)
{
    if (any(GlobalIndex.xy >= gViewportDims)) 
    {
        return;
    }

    printSetPixel(GlobalIndex);

    SampleGenerator sg = SampleGenerator.create(GlobalIndex, gFrameIndex + 1 * 13);
    SampleGenerator groupSG = SampleGenerator.create(groupIndex.xy, gFrameIndex + 1 * 13);
    
    uint tileIndex = min(sampleNext1D(groupSG) * gParams.tileCount, gParams.tileCount - 1);
    uint risBufferBase = tileIndex * gParams.tileSize;
    
    Reservoir r = emptyReservoir();
    
    ShadingData sd;
    bool valid = LoadShadingData(gGBuffer, GlobalIndex.xy, gViewportDims, gScene.camera, sd);

#ifdef _PRE_SAMPLING
    static const bool kUsePreSampling = true;
#else
    static const bool kUsePreSampling = false;
#endif

#ifdef _VSL_EVALUATION
    static const bool kUseVSL = true;
#else
    static const bool kUseVSL = false;
#endif

    if (valid)
    {
        float selectedLPdf = 1.f;
        bool selectedSelfGeneratedVPL = false;

        for(uint i = 0; i < numInitialSamples; i++)
        {
            uint lightIndex = 0xffffffff;
            float lightInvPdf = 0.0f;

            if (kUsePreSampling)
            {
                uint risSampleIndex = risBufferBase + min(sampleNext1D(sg) * gParams.tileSize, gParams.tileSize - 1);
                lightIndex = gRISBuffer[risSampleIndex].x;
                lightInvPdf = asfloat(gRISBuffer[risSampleIndex].y);
            }
            else
            {
#ifdef _UNIFORM_SAMPLING
                uint lightIndex = min(sampleNext1D(sg) * gVirtualLights.count, gVirtualLights.count - 1);
                float lightInvPdf = gVirtualLights.count;
#else
                lightIndex = gFluxTable.sample(sampleNext2D(sg));
                lightInvPdf = gFluxTable.weightSum / gFluxTable.getWeight(lightIndex);
#endif
            }

            //  cone sampling
            {
                float2 VSLrand = sampleNext2D(sg);
                float ConePdf;
                float3 ConeDir;
                float targetPdf;
                if (gPointSampling)
                {
                    targetPdf = luminance(gVirtualLights.coneSampleShadingPointRadiance(sd, VSLrand, lightIndex, ConePdf, ConeDir, gSpecRadianceContainer));
                }
                else
                {
                    targetPdf = luminance(gVirtualLights.coneComputeShadingPointRadiance(sd, sg, gNumMinConeSamples, gNumConeSamples, lightIndex, gSpecRadianceContainer));
                    ConePdf = 1.f;
                    ConeDir = 0.f;
                }

                // MIS with tile sampling
                float tilePDF = 0;

                if (useSelfGenSamples)
                {
                    tilePDF = gVirtualLights.evaluateTileSamplingPDF(GlobalIndex.xy, lightIndex, gTileSize, tileSearchRadius, gTileDims, gTileSampleNum, gTileVirtualLightContainer);
                }

                float MISTotalProb = (numTileVPLSamples * tilePDF * lightInvPdf + numInitialSamples) * ConePdf;

                if (streamSample(r, lightIndex, VSLrand, sampleNext1D(sg), targetPdf, lightInvPdf / MISTotalProb))
                {
                    selectedLPdf = ConePdf;
                }
            }
        }

        // Self-Gen samples (Tile Sampling)
        if (useSelfGenSamples)
        {
            for (uint i = 0; i < numTileVPLSamples; i++)
            {
                float tilePDF = 0.0f;
                uint lightIndex = 0xffffffff;
                bool valid = gVirtualLights.sampleFromTile(sampleNext3D(sg), GlobalIndex.xy, gTileSize, tileSearchRadius, gTileDims, gTileSampleNum, gTileVirtualLightContainer, lightIndex, tilePDF);

                if (valid)
                {
                    float lightInvPdf = gFluxTable.weightSum / gFluxTable.getWeight(lightIndex);
                    float2 VSLrand = sampleNext2D(sg);
                    float ConePdf;
                    float3 ConeDir;
                    float targetPdf;

                    if (gPointSampling)
                    {
                        targetPdf = luminance(gVirtualLights.coneSampleShadingPointRadiance(sd, VSLrand, lightIndex, ConePdf, ConeDir, gSpecRadianceContainer));
                    }
                    else
                    {
                        targetPdf = luminance(gVirtualLights.coneComputeShadingPointRadiance(sd, sg, gNumMinConeSamples, gNumConeSamples, lightIndex, gSpecRadianceContainer));
                        ConePdf = 1.f;
                        ConeDir = 0.f;
                    }

                    float weight = 0.f;
                    if (isinf(lightInvPdf))
                    {
                        weight = 1.f / (numTileVPLSamples * tilePDF * ConePdf);
                    }
                    else
                    {
                        float MISTotalProb = (numTileVPLSamples * tilePDF * lightInvPdf + numInitialSamples) * ConePdf;
                        weight = lightInvPdf / MISTotalProb;
                    }

                    if (streamSample(r, lightIndex, VSLrand, sampleNext1D(sg), targetPdf, weight))
                    {
                        selectedLPdf = ConePdf;
                        selectedSelfGeneratedVPL = true;
                    }
                }
            }
        }

        // skip shadow rays for self-gen samples since it's likely not occluded (does not introduce bias)
        if (isValidReservoir(r) && !selectedSelfGeneratedVPL)
        {
            float3 rayOrigin = sd.computeNewRayOrigin();
            uint lightIndex = getReservoirLightIndex(r);
            float3 lightPos = gVirtualLights.getPosition(lightIndex);
#if !defined(_VSL_EVALUATION) || !SCATTER_VISIBILITY
            if (isOccluded(rayOrigin, lightPos))
            {
                r.lightData = 0;
                r.weightSum = 0;
            }
#endif
        }

        r.targetPdf /= selectedLPdf; // convert to primary sample space (assuming cone sampling technique) for reusing random numbers

        r.M = 1;
        finalizeResampling(r, 1.0, r.M);
    }

    storeReservoir(gParams, r, gReservoirs, GlobalIndex.xy, gOutputBufferIndex);
}
