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
import LightTreeSampler;
import Utils.Debug.PixelDebug;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    ResamplingRuntimeParameters gParams;
    VirtualLightContainer gVirtualLights;
    GBuffer gGBuffer;
    MegaTextureContainer gSpecRadianceContainer;
    uint gInputBufferIndex;
    bool gEnableDirectLighting;
    bool gPointSampling;
    uint gShadingMode;
    uint gVisibilityMode;
    uint gTileSampleNum;
    AliasTable gFluxTable;
    AliasTable gEmissiveTriTable;
    uint2 gTileDims;
    uint gNumFinalConeSamples;
    uint gNumMinFinalConeSamples;
    uint gLeafStartIndex;
    uint gMaxLightSamples;
    bool gImproveCorner;
    EmissiveLightSampler gEmissiveLightSampler;
}

StructuredBuffer<uint> gTileVirtualLightContainer;
StructuredBuffer<uint> gTileVirtualLightWeights;
StructuredBuffer<LightNode> gNodeBuffer;

RWStructuredBuffer<Reservoir> gReservoirs;
RWTexture2D<float4> gShadingOutput;

float3 evalEmissiveLighting(ShadingData sd, inout SampleGenerator sg, EmissiveLightSampler emissiveSampler)
{
    TriangleLightSample tls;
    if (emissiveSampler.sampleLight(sd.posW, sd.N, true, sg, tls))
    {
        SimpleBRDF simpleBRDF;
        simpleBRDF.setup(sd);

        float3 lightPos = computeRayOrigin(tls.posW, tls.normalW);
        float3 shadingOrigin = computeRayOrigin(sd.posW, dot(sd.faceN, lightPos - sd.posW) >= 0 ? sd.faceN : -sd.faceN);
        float3 toLight = lightPos - shadingOrigin;
        float lightDistance = length(toLight);
        float3 L = normalize(toLight);

        if (dot(L, sd.N) <= 1e-6f)
        {
            return 0.0f;
        }

        bool visible = traceShadowRay(shadingOrigin, L, lightDistance);
        if (!visible)
        {
            return float3(0.f);
        }

        float pdf = tls.pdf;
        float3 Le = tls.Le;

        float3 wo = toLocal(sd.V, sd.N);
        float3 wi = toLocal(L, sd.N);
        float misWeight = 1.0f;
        return misWeight * Le * simpleBRDF.eval(wo, wi) / (pdf);
    }
    
    return 0.0f;
}

float3 biasCompensation(inout SampleGenerator sg, in ShadingData sd, uint lightIndex, float invPdf)
{
    float3 res = 0.0;
    float3 throughput = 1.0f;
    bool terminated = false;
    for (uint i = 0; i < 3; i++)
    {
        if(terminated)
        {
            break;
        }
        
        SimpleBRDF simpleBRDF;
        simpleBRDF.setup(sd);
        float3 sampledDir;
        float pdf;
        uint lobeType;
        bool valid = simpleBRDF.sample(sd, sampledDir, pdf, lobeType, sg);

        if(!valid || dot(sampledDir, sd.faceN) < 1e-6f)
        {
            terminated = true;
        }
        else
        {
            float3 posWOrigin = computeRayOrigin(sd.posW, sd.faceN);
            float3 wi = toLocal(sampledDir, sd.N);
            float3 wo = toLocal(sd.V, sd.N);
            throughput = throughput * simpleBRDF.eval(wo, wi) / pdf;

            ShadingData sd_next;
            TriangleHit trianglehit;
            if(castScene(posWOrigin, sampledDir, sd_next, trianglehit, 0.5))
            {
                // Bias Happen
                if(GeometryTerm(sd_next.posW, sd_next.N, sd.posW, sd.N) > GeometryB())
                {
                    throughput *= GeometryTermResidual(sd_next.posW, sd_next.N, sd.posW, sd.N);
                    res += throughput * sd_next.emissive;
                    res += throughput * evalEmissiveLighting(sd_next, sg, gEmissiveLightSampler);
                    float3 L = normalize(gVirtualLights.getPosition(lightIndex) - sd_next.posW);
                    float3 Le = gVirtualLights.readLe(lightIndex, L, gSpecRadianceContainer);
                    res += throughput * gVirtualLights.shadeSurface(lightIndex, sd_next, Le) * invPdf;
                }
                else
                {
                    terminated = true;
                }
            }
            else
            {
                terminated = true;
            }
            sd = sd_next;
        }
    }
    return res;
}

float3 readRadianceByPosition(float3 shadingPosition, ShadingData sd)
{
    float3 Le = 0.0f;

    RayDesc searchRay;
    searchRay.Origin = sd.posW;
    searchRay.Direction = float3(0.0f, 1.0f, 0.0f);
    searchRay.TMin = 0.0f;
    searchRay.TMax = 0.00001f;
    RayQuery<RAY_FLAG_NONE> rayQuery;
    rayQuery.TraceRayInline(gVirtualLights.as, RAY_FLAG_NONE, 0xff, searchRay);
    for (uint i = 0; i < 8; i++)
    {
        if(!rayQuery.Proceed())
        {
            break;
        }
    
        if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
        {
            uint lightIndex = rayQuery.CandidatePrimitiveIndex();
            float3 posW = gVirtualLights.getPosition(lightIndex);
            float radius = gVirtualLights.getRadius(lightIndex);
            float disSquare = dot(sd.posW - posW, sd.posW - posW);
            if(disSquare <= radius * radius)
            {
                float3 L = sd.posW - shadingPosition;
                float3 virtualLightN = gVirtualLights.getNormal(lightIndex);
                if(dot(sd.N, virtualLightN) > 0.65f)
                {
                    float area = M_PI * radius * radius;
                    Le += (gVirtualLights.readLe(lightIndex, L, gSpecRadianceContainer) / area);
                }
            }
        }

        if (i == 7)
        {
            rayQuery.Abort();
        }
    }

    return Le;
}

float3 brdfSamplingVirtualLights(inout SampleGenerator sg, ShadingData sd)
{
    SimpleBRDF simpleBRDF;
    simpleBRDF.setup(sd);

    float3 res = 0.0f;

    float pdfBrdf;
    float3 sampledDir;
    uint lobeType;
    bool suc = simpleBRDF.sample(sd, sampledDir, pdfBrdf, lobeType, sg);
    float3 wo = toLocal(sd.V, sd.N);
    float3 wi = toLocal(sampledDir, sd.N);
    float3 throughput = simpleBRDF.eval(wo, wi) / pdfBrdf;
    if (!suc || min(sd.NdotV, dot(sampledDir, sd.N)) < 1e-6f)
    {
    }
    else
    {
        ShadingData nextSd;
        float3 rayOrigin = sd.computeNewRayOrigin();
        if (traceScene(rayOrigin, sampledDir, nextSd))
        {
            float3 disVec = nextSd.posW - sd.posW;
            float minDis = 0.2f + 0.2f * sampleNext1D(sg);
            if (dot(disVec, disVec) < (minDis * minDis))
            {
               res += throughput * evalEmissiveLighting(nextSd, sg, gEmissiveLightSampler);
               res += throughput * secondaryBrdfSamplingVirtualLights(sg, nextSd, minDis);
            }
            else
            {
                float3 Le = readRadianceByPosition(sd.posW, nextSd);
                res += throughput * Le;
            }
        }
    }

    return res;
}

float3 brdfSamplingVirtualLightsShort(inout SampleGenerator sg, ShadingData sd)
{
    SimpleBRDF simpleBRDF;
    simpleBRDF.setup(sd);

    float3 res = 0.0f;

    float pdfBrdf;
    float3 sampledDir;
    uint lobeType;
    bool suc = simpleBRDF.sample(sd, sampledDir, pdfBrdf, lobeType, sg);
    float3 wo = toLocal(sd.V, sd.N);
    float3 wi = toLocal(sampledDir, sd.N);
    float3 throughput = simpleBRDF.eval(wo, wi) / pdfBrdf;
    if (!suc || min(sd.NdotV, dot(sampledDir, sd.N)) < 1e-6f)
    {
    }
    else
    {
        ShadingData nextSd;
        float3 rayOrigin = sd.computeNewRayOrigin();
        if (traceScene(rayOrigin, sampledDir, nextSd))
        {
            float3 disVec = nextSd.posW - sd.posW;
            float minDis = 0.2f + 0.2f * sampleNext1D(sg);
            if (dot(disVec, disVec) < (minDis * minDis))
            {
               res += throughput * evalEmissiveLighting(nextSd, sg, gEmissiveLightSampler);
               res += throughput * secondaryBrdfSamplingVirtualLights(sg, nextSd, minDis);
            }
        }
    }

    return res;
}

float3 secondaryBrdfSamplingVirtualLights(inout SampleGenerator sg, ShadingData sd, float minDis)
{
    SimpleBRDF simpleBRDF;
    simpleBRDF.setup(sd);

    float3 res = 0.0f;

    float pdfBrdf;
    float3 sampledDir;
    uint lobeType;
    bool suc = simpleBRDF.sample(sd, sampledDir, pdfBrdf, lobeType, sg);
    float3 wo = toLocal(sd.V, sd.N);
    float3 wi = toLocal(sampledDir, sd.N);
    float3 throughput = simpleBRDF.eval(wo, wi) / pdfBrdf;

    if (!suc || dot(sampledDir, sd.faceN) < 1e-6f)
    {
    }
    else
    {
        ShadingData nextSd;
        float3 rayOrigin = computeRayOrigin(sd.posW, sd.frontFacing ? sd.faceN : -sd.faceN);
        if (traceScene(rayOrigin, sampledDir, nextSd))
        {
            float3 disVec = nextSd.posW - sd.posW;
            if (dot(disVec, disVec) > (minDis * minDis))
            {
                float3 Le = readRadianceByPosition(sd.posW, nextSd);
                res += throughput * Le;
            }
            else
            {
                res += throughput * evalEmissiveLighting(nextSd, sg, gEmissiveLightSampler);
            }
        }
    }

    return res;
}

static const bool kUseRussianRoulette = true;
static const bool kUseNEE = true;
static const bool kUseMIS = true;
static const uint kBounceNum = 256u;
static const float gRoughnessThreshold = 0.16f;

float3 groundtruthEvaluation(uint2 pixelPos)
{
    float3 color = 0.0f;
    SampleGenerator sg = SampleGenerator.create(pixelPos, gFrameIndex);
    ShadingData sd;
    bool valid = LoadShadingData(gGBuffer, pixelPos, gViewportDims, gScene.camera, sd);
    if (!valid)
    {
        return 0.0f;
    }

//     float3 rayOrigin = sd.computeNewRayOrigin();
//     float3 res = 0.0f;
//     uint lightIndex = gFluxTable.sample(sampleNext2D(sg));
//     float pdf = gFluxTable.getWeight(lightIndex) / gFluxTable.weightSum;
//     float3 lightPos = gVirtualLights.getPosition(lightIndex);
    
//     res += sd.emissive;

//     if (gEnableDirectLighting)
//     {            
//         res += evalEmissiveLighting(sd, sg, gEmissiveLightSampler);
//     }

//     if (pdf > 0.0f)
//     {
//         if (gImproveCorner)
//         {
//             res += brdfSamplingVirtualLightsShort(sg, sd);
//         }
//         float3 lightPos = gVirtualLights.getPosition(lightIndex);
//         if (!isOccluded(rayOrigin, lightPos))
//         {
// #ifdef _VSL_EVALUATION
//             #ifdef _USE_BIAS_COMPENSATION
//             if (sd.linearRoughness > 0.7)
//             {
//                 float invPdf = 1.0f / pdf;
//                 float3 L = normalize(lightPos - rayOrigin);
//                 float3 Le = gVirtualLights.readLe(lightIndex, L, gSpecRadianceContainer);
//                 res += gVirtualLights.shadeSurface(lightIndex, sd, Le) * invPdf;

            
//                 res += biasCompensation(sg, sd, lightIndex, invPdf);
            
//             }
//             else
//             #endif
//             {
//                 float3 disVec = lightPos - sd.posW;
//                 float minDis = 0.2f + 0.2f * sampleNext1D(sg);
//                 if (gImproveCorner && (dot(disVec, disVec) < (minDis * minDis)))
//                 {
//                 }
//                 else
//                 {
//                     for (uint i = 0; i < 16; i++)
//                     {
//                         float LPdf;
//                         float3 L;
//                         float2 VSLrand = sampleNext2D(sg); // randomly sample
//                         gVirtualLights.sampleVSL(lightIndex, VSLrand, sd.posW, sd.N, L, LPdf);

//                         if (LPdf > 0.0)
//                         {
//                             float3 Le = gVirtualLights.readLe(lightIndex, L, gSpecRadianceContainer);
//                             res += 0.0625f * gVirtualLights.shadeSurfaceWithVSL(lightIndex, sd, Le, L) / LPdf / pdf ;
//                         }
//                     }
//                 }
//             }
// #else
//             float3 L = normalize(lightPos - rayOrigin);
//             float3 Le = gVirtualLights.readLe(lightIndex, L, gSpecRadianceContainer);
//             res += gVirtualLights.shadeSurface(lightIndex, sd, Le) / pdf ;
// #endif
//         }
//     }
//     return res;
    color += sd.emissive;

    SimpleBRDF simpleBRDF;
    simpleBRDF.setup(sd);

    float3 throughput = 1.0f;

    bool isDelta = false;
    bool isSmooth = false;
    float3 rayOrigin = sd.computeNewRayOrigin();
    float3 rayDirection;

    float depthVal = 0.0f;
    float depthThp = 1.0f;
    uint firstSampableDepth = 0;
    
    for (uint depth = 0; depth < kBounceNum; depth++)
    {
        // Sample Emissive Light
        color += throughput * evalEmissiveLighting(sd, sg, gEmissiveLightSampler);

        // Sample Virtual Light
        {
            uint lightIndex = gFluxTable.sample(sampleNext2D(sg));
            float pdf = gFluxTable.getWeight(lightIndex) / gFluxTable.weightSum;
            float3 lightPos = gVirtualLights.getPosition(lightIndex);
            float3 L = normalize(lightPos - rayOrigin);
            if (!isOccluded(rayOrigin, lightPos))
            {
                float3 Le = gVirtualLights.readLe(lightIndex, L, gSpecRadianceContainer);
                color += gVirtualLights.shadeSurface(lightIndex, sd, Le) / pdf;
            }
        }

        break;
    }

    return color;
}

[numthreads(16, 16, 1)]
void main(uint2 GlobalIndex : SV_DispatchThreadID, uint3 groupIndex: SV_GROUPID)
{
    if (any(GlobalIndex.xy >= gViewportDims)) 
    {
        return;
    }

    printSetPixel(GlobalIndex);

    float3 res = 0.0;
    if (gShadingMode == 0)
    {
        res = groundtruthEvaluation(GlobalIndex);
        gShadingOutput[GlobalIndex] = float4(res, 1.0);
        return;
    }

    SampleGenerator sg = SampleGenerator.create(GlobalIndex, gFrameIndex);
    ShadingData sd;
    bool valid = LoadShadingData(gGBuffer, GlobalIndex.xy, gViewportDims, gScene.camera, sd);
    float3 rayOrigin = sd.computeNewRayOrigin();
    if (valid)
    {
        res += sd.emissive;

        if (gEnableDirectLighting)
        {            
            res += evalEmissiveLighting(sd, sg, gEmissiveLightSampler);
        }

        // RIS
        if (gShadingMode == 1)
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
                float3 rayOrigin = computeRayOrigin(sd.posW, sd.frontFacing ? sd.faceN : -sd.faceN);
                float3 lightPos = gVirtualLights.getPosition(lightIndex);
                if (!isOccluded(rayOrigin, lightPos))
                {
                    // float3 L = gVirtualLights.getDirection(lightIndex, getReservoirSampleUV(r));
                    float3 L = selectedL;
                    float3 Le = gVirtualLights.readLe(lightIndex, L, gSpecRadianceContainer);
                    res += gVirtualLights.shadeSurfaceWithVSL(lightIndex, sd, Le, L) * getReservoirInvPdf(r) ;
                }
            }
        }
        else if (gShadingMode == 4)
        {
            res += brdfSamplingVirtualLights(sg, sd);
        }
        // ReSTIR
        else if (gShadingMode == 2)
        {
            if (gImproveCorner)
            {
                res += brdfSamplingVirtualLightsShort(sg, sd);
            }
            Reservoir r = loadReservoir(gParams, gReservoirs, GlobalIndex.xy, gInputBufferIndex);
            print(r.M);
            if (isValidReservoir(r))
            {
                uint lightIndex = getReservoirLightIndex(r);
                float3 rayOrigin = computeRayOrigin(sd.posW, sd.frontFacing ? sd.faceN : -sd.faceN);
                float3 lightPos = gVirtualLights.getPosition(lightIndex);
                float3 disVec = lightPos - sd.posW;
                float minDis = 0.2f + 0.2f * sampleNext1D(sg);
                bool useVirtualLights = (dot(disVec, disVec) >= (minDis * minDis)) || !gImproveCorner;
                if ((gVisibilityMode == 0 || gVisibilityMode == 2) && useVirtualLights)
                {
#if !defined(_VSL_EVALUATION) || !SCATTER_VISIBILITY
                    if (!isOccluded(rayOrigin, lightPos))
#endif
                    {
                        #ifdef _USE_BIAS_COMPENSATION
                        if (sd.linearRoughness > 0.7 && !(gVisibilityMode == 2 && gPointSampling)) // somehow this mode does not work with bias compensation...
                        {
                            float3 L = normalize(lightPos - rayOrigin);
                            float3 Le = gVirtualLights.readLe(lightIndex, L, gSpecRadianceContainer);
                            float invPdf = getReservoirInvPdf(r);
                            res += gVirtualLights.shadeSurface(lightIndex, sd, Le) * invPdf;
                            res += biasCompensation(sg, sd, lightIndex, invPdf);
                        }
                        else
                        #endif
                        {
                            float LPdf; float3 dummy;
                            float3 F;
                            if (gPointSampling)
                            {
                                F = gVirtualLights.coneSampleShadingPointRadiance(sd, unpackUV(r.uvData), lightIndex, LPdf, dummy, gSpecRadianceContainer);
                            }
                            else
                            {
                                F = gVirtualLights.coneComputeShadingPointRadianceRandom(sd, sg, gNumMinFinalConeSamples, gNumFinalConeSamples, lightIndex, gSpecRadianceContainer);
                                LPdf = 1.f;
                            }

                            float3 F_ = F / LPdf; // incoprate Jacobian
                            res += F_ * getReservoirInvPdf(r);
                        }
                    }
#if !defined(_VSL_EVALUATION) || !SCATTER_VISIBILITY
                    else // set next frame's temporal reservoir's weight to 0
                    {
                        setReservoirWeight(gParams, gReservoirs, GlobalIndex.xy, gInputBufferIndex, 0);
                    }
#endif
                }
                else //Point-To-Sphere visibility
                {
                    // not implemented!
                }
            }
        }
        else if (gShadingMode == 3)
        {
            // lightcut sampling
            LightCutsSample lightSamples[MAX_LIGHT_SAMPLES];
            sampleCut(sd.posW, sd.N, gMaxLightSamples, sg, lightSamples, gLeafStartIndex, gNodeBuffer);

            float3 indirectRes = 0.f;
            for (int i = 0; i < gMaxLightSamples; i++)
            {
                // light tree sampling
                //LightCutsSample lightSamples;
                //sampleLightTree(1, sd.posW, sd.N, sg, lightSamples, gLeafStartIndex, gNodeBuffer);

                int lightID = lightSamples[i].index;
                float pdf = lightSamples[i].prob;

                if (lightID == -1) continue;

                float3 lightPos = gVirtualLights.getPosition(lightID);

                if (gVisibilityMode == 0 || gVisibilityMode == 2) //Point-To-Point
                {
#if !defined(_VSL_EVALUATION) || !SCATTER_VISIBILITY
                    if (!isOccluded(rayOrigin, lightPos))
#endif
                    {
                        float3 F = gVirtualLights.coneComputeShadingPointRadianceRandom(sd, sg, gNumMinFinalConeSamples, gNumFinalConeSamples, lightID, gSpecRadianceContainer);
                        indirectRes += F / pdf;
                    }
                }
                else //Point-To-Sphere 
                {
                    float solidAngle;
                    bool coneSampleValid = gVirtualLights.isConeSamplingValid(lightID, sd.posW, solidAngle);
                    int sampleCount = gNumMinFinalConeSamples + int((gNumFinalConeSamples - gNumMinFinalConeSamples) * solidAngle / (2 * M_PI));
                    bool visibility = true;
                    float3 F = 0.f;

                    // shoot shadow ray
                    if (!coneSampleValid)
                    {
                        visibility = !isOccluded(rayOrigin, lightPos);
                    }

                    if (visibility)
                    {
                        for (int i = 0; i < sampleCount; i++)
                        {
                            float3 L;
                            float LPdf;

                            float2 VSLrand = sampleNext2D(sg); // randomly sample
                            if (coneSampleValid)
                            {
                                gVirtualLights.coneSamplingVSL(lightID, VSLrand, sd.posW, L, LPdf);
                                float vslRadius = gVirtualLights.getVSLRadius(lightID);
                                float t;
                                if (intersectRaySphere(rayOrigin, L, lightPos, vslRadius, t))
                                {
                                    float3 targetPos = rayOrigin + t * L;
                                    visibility = !isOccluded(rayOrigin, targetPos);
                                }
                            }
                            else // hemispherical sampling
                            {
                                L = HemisphereSampling(VSLrand, sd.N, LPdf);
                            }

                            if (visibility)
                            {
                                float3 Le = gVirtualLights.readLe(lightID, L, gSpecRadianceContainer);
                                if (LPdf > 0.0)
                                {
                                    F += gVirtualLights.shadeSurfaceWithVSL(lightID, sd, Le, L) / (sampleCount * LPdf);
                                }
                            }
                        }
                    }
                    indirectRes += F / pdf;
                }
            }

            // light tree sampling
            //indirectRes /= gMaxLightSamples;

            res += indirectRes;
        }

        if (any(isnan(res)))
        {
            res = 0.0;
        }
    }

    gShadingOutput[GlobalIndex] = float4(res, 1.0);
}
