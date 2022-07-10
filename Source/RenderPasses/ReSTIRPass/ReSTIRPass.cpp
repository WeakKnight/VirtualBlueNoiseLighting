/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#include "ReSTIRPass.h"
#include "Utils/VirtualLight/VirtualLightContainer.h"
#include "RenderPasses/GBufferPass.h"
#include "Utils/VirtualLight/MegaTextureContainer.h"
#include "Utils/Sampling/AliasTable.h"
#include "Utils/Debug/PixelDebug.h"
#include "Experimental/Scene/Lights/LightBVHSampler.h"

namespace
{
    const char kDummy[] = "dummy";
    const char kDummyInput[] = "input";
    const char kDummyOutput[] = "output";
    const char kDesc[] = "ReSTIR Pass";

    const char kShadingMode[] = "GroundtruthMode";

    const char kEnableDirectLighting[] = "enableDirectLighting";
    const char kEnableVSLEvaluation[] = "enableVSLEvaluation";
    const char kEnableTemporalResampling[] = "enableTemporalResampling";
    const char kEnableSpatialResampling[] = "enableSpatialResampling";
    const char kEnablePresampling[] = "enablePresampling";
    const char kNeedUsageCount[] = "needUsageCount";
    const char kVSLRadiusFactor[] = "VSLRadiusFactor";
    const char kDoSpecularGathering[] = "enableSpecularGathering";
    const char kUseTileSampling[] = "useTileSampling";
    const char kTileSearchRadius[] = "tileSearchRadius";
    const char kBiasCompensation[] = "biasCompensation";
    const char kMaxHistoryLength[] = "maxHistoryLength";
    const char kImproveCorner[] = "improveCorner";
    const char kVisibilityMode[] = "visibilityMode";
    const char kNumMinConeSamples[] = "numMinConeSamples";

    const std::string kDicInitialVirtualLights = "initialVirtualLights";
    const std::string kDicSampleEliminatedVirtualLights = "sampleEliminatedVirtualLights";
    const std::string kDicCurVirtualLights = "curVirtualLights";
    const std::string kDicSpecularRadianceContainer = "specularRadianceContainer";
    const std::string kDicFluxBuffer = "fluxBuffer";
    const std::string kDicFluxTable = "fluxTable";
    const std::string kDicRadianceReady = "radianceReady";
    const std::string kGBuffer = "GBuffer";
    const std::string kGBufferPrevFrame = "GBufferPrevFrame";
    const std::string kReservoirBuffer = "reservoirBuffer";
    const std::string kEmissiveTriBase = "EMISSIVE_TRI_BASE";
    const std::string kEmissiveTriBaseVal = "3000000";
    const std::string kEmissiveTriangleFluxTable = "emissiveTriangleFluxTable";

    const std::string kDicTileDim = "tileDim";
    const std::string kDicTileSize = "tileSize";
    const std::string kDicTileSampleNum = "tileSampleNum";
    const std::string kDicTileVirtualLightContainer = "tileVirtualLightContainer";
    const std::string kDicTileVirtualLightWeights = "tileVirtualLightWeights";

    const char kFillTileHoles[] = "fillTileHoles";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

static void regPythonApi(pybind11::module& m)
{
    pybind11::class_<ReSTIRPass, RenderPass, ReSTIRPass::SharedPtr> pass(m, "ReSTIRPass");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("ReSTIRPass", kDesc, ReSTIRPass::create);
    ScriptBindings::registerBinding(regPythonApi);
}

void computePdfTextureSize(uint32_t maxItems, uint32_t& outWidth, uint32_t& outHeight, uint32_t& outMipLevels)
{
    // Compute the size of a power-of-2 rectangle that fits all items, 1 item per pixel
    double textureWidth = std::max(1.0, ceil(sqrt(double(maxItems))));
    textureWidth = exp2(ceil(log2(textureWidth)));
    double textureHeight = std::max(1.0, ceil(maxItems / textureWidth));
    textureHeight = exp2(ceil(log2(textureHeight)));
    double textureMips = std::max(1.0, log2(std::max(textureWidth, textureHeight)));

    outWidth = uint32_t(textureWidth);
    outHeight = uint32_t(textureHeight);
    outMipLevels = uint32_t(textureMips);
}

void FillNeighborOffsetBuffer(uint8_t* buffer)
{
    int R = 250;
    const float phi2 = 1.0f / 1.3247179572447f;
    uint32_t num = 0;
    float u = 0.5f;
    float v = 0.5f;
    while (num < 8192 * 2) {
        u += phi2;
        v += phi2 * phi2;
        if (u >= 1.0f) u -= 1.0f;
        if (v >= 1.0f) v -= 1.0f;

        float rSq = (u - 0.5f) * (u - 0.5f) + (v - 0.5f) * (v - 0.5f);
        if (rSq > 0.25f)
            continue;

        buffer[num++] = int8_t((u - 0.5f) * R);
        buffer[num++] = int8_t((v - 0.5f) * R);
    }
}

ReSTIRPass::SharedPtr ReSTIRPass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new ReSTIRPass);
    for (const auto& [key, value] : dict)
    {
        if (key == kEnableTemporalResampling)
        {
            pPass->mEnableTemporalResampling = value;
        }
        else if (key == kEnableSpatialResampling)
        {
            pPass->mEnableSpatialResampling = value;
        }
        else if (key == kEnableVSLEvaluation)
        {
            pPass->mEnableVSLEvaluation = value;
        }
        else if (key == kShadingMode)
        {
            pPass->mShadingMode = value;
        }
        else if (key == kEnableDirectLighting)
        {
            pPass->mEnableDirectLighting = value;
        }
        else if (key == kEnablePresampling)
        {
            pPass->mEnablePresampling = value;
        }
        else if (key == kVSLRadiusFactor)
        {
            pPass->mVSLRadiusFactor = value;
        }
        else if (key == kDoSpecularGathering)
        {
            pPass->mEnableSpecularGathering = value;
        }
        else if (key == kTileSearchRadius)
        {
            pPass->mTileSearchRadius = value;
        }
        else if (key == kBiasCompensation)
        {
            pPass->mEnableBiasCompensation = false;
        }
        else if (key == kUseTileSampling)
        {
            pPass->mUseSelfGenSamples = value;
        }
        else if (key == kMaxHistoryLength)
        {
            pPass->mMaxHistoryLength = value;
        }
        else if (key == kImproveCorner)
        {
            pPass->mImproveCorner = value;
        }
        else if (key == kVisibilityMode)
        {
            pPass->mVisibilityMode = value;
        }
        else if (key == kNumMinConeSamples)
        {
            pPass->mNumMinConeSamples = value;
        }
    }

    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);
    pPass->mpPixelDebug = PixelDebug::create();

   /* Program::Desc prepareEmissiveTrianglesPassDesc;
    prepareEmissiveTrianglesPassDesc.addShaderLibrary("RenderPasses/ReSTIRPass/ReSTIRPrepareEmissiveTriangles.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpPrepareEmissiveTrianglesPass = ComputePass::create(prepareEmissiveTrianglesPassDesc, Program::DefineList(), false);*/

    Program::Desc prepareLightPassDesc;
    prepareLightPassDesc.addShaderLibrary("RenderPasses/ReSTIRPass/ReSTIRPrepareLights.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpPrepareLightsPass = ComputePass::create(prepareLightPassDesc, Program::DefineList(), false);

    Program::Desc preSamplingPassDesc;
    preSamplingPassDesc.addShaderLibrary("RenderPasses/ReSTIRPass/ReSTIRPreSampling.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpPreSamplingPass = ComputePass::create(preSamplingPassDesc, Program::DefineList(), false);

    Program::Desc initialSamplingPassDesc;
    initialSamplingPassDesc.addShaderLibrary("RenderPasses/ReSTIRPass/ReSTIRInitialSampling.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpInitialSamplingPass = ComputePass::create(initialSamplingPassDesc, Program::DefineList(), false);

    Program::Desc temporalResamplingPassDesc;
    temporalResamplingPassDesc.addShaderLibrary("RenderPasses/ReSTIRPass/ReSTIRTemporalResampling.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpTemporalResamplingPass = ComputePass::create(temporalResamplingPassDesc, Program::DefineList(), false);

    Program::Desc spatialResamplingPassDesc;
    spatialResamplingPassDesc.addShaderLibrary("RenderPasses/ReSTIRPass/ReSTIRSpatialResampling.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpSpatialResamplingPass = ComputePass::create(spatialResamplingPassDesc, Program::DefineList(), false);

    Program::Desc restirShadingPassDesc;
    restirShadingPassDesc.addShaderLibrary("RenderPasses/ReSTIRPass/ReSTIRShading.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpShadingPass = ComputePass::create(restirShadingPassDesc, Program::DefineList(), false);

    Program::Desc SpecularGatheringPassDesc;
    SpecularGatheringPassDesc.addShaderLibrary("RenderPasses/ReSTIRPass/ReSTIRSpecularGathering.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpSpecularGatheringPass = ComputePass::create(SpecularGatheringPassDesc, Program::DefineList(), false);

    return pPass;
}

std::string ReSTIRPass::getDesc()
{
    return kDesc;
}

Dictionary ReSTIRPass::getScriptingDictionary()
{
    Dictionary d;
    d[kEnableVSLEvaluation] = mEnableVSLEvaluation;
    d[kEnableTemporalResampling] = mEnableTemporalResampling;
    d[kEnableSpatialResampling] = mEnableSpatialResampling;
    d[kShadingMode] = mShadingMode;
    d[kEnableDirectLighting] = mEnableDirectLighting;
    d[kEnablePresampling] = mEnablePresampling;
    d[kVSLRadiusFactor] = mVSLRadiusFactor;
    d[kDoSpecularGathering] = mEnableSpecularGathering;
    d[kTileSearchRadius] = mTileSearchRadius;
    d[kBiasCompensation] = mEnableBiasCompensation;
    d[kMaxHistoryLength] = mMaxHistoryLength;
    d[kImproveCorner] = mImproveCorner;
    d[kVisibilityMode] = mVisibilityMode;
    d[kNumMinConeSamples] = mNumMinConeSamples;
    d[kUseTileSampling] = mUseSelfGenSamples;
    return d;
}

RenderPassReflection ReSTIRPass::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "useless dummy Input");
    reflector.addOutput(kDummyOutput, "shading Output").bindFlags(ResourceBindFlags::RenderTarget | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource).format(ResourceFormat::RGBA32Float);
    return reflector;
}

void ReSTIRPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
    {
        return;
    }

    GBufferPass::SharedPtr gBuffer = renderData.getDictionary()[kGBuffer];
    GBufferPass::SharedPtr gBufferPrev = renderData.getDictionary()[kGBufferPrevFrame];
    if (gBuffer == nullptr || gBufferPrev == nullptr)
    {
        return;
    }

    if (gpState->currentState != 3)
    {
        return;
    }

    mpPixelDebug->beginFrame(pRenderContext, renderData.getDefaultTextureDims());

    prepareLights(pRenderContext);

    if (mNeedClear)
    {
        mpScene->getCamera()->setShutterSpeed(mpScene->getCamera()->getShutterSpeed() + 1.0f);
        mNeedClear = false;
    }

    if (mNeedRecompile)
    {
        recompile();
    }

    VirtualLightContainer::SharedPtr sampleEliminatedVirtualLights = renderData.getDictionary()[kDicSampleEliminatedVirtualLights];
    MegaTextureContainer::SharedPtr specularRadianceContainer = renderData.getDictionary()[kDicSpecularRadianceContainer];
    AliasTable::SharedPtr emissiveTriangleFluxTable = renderData.getDictionary()[kEmissiveTriangleFluxTable];

    Buffer::SharedPtr fluxBuffer = renderData.getDictionary()[kDicFluxBuffer];

    bool fillTileHoles = true;
    if (renderData.getDictionary().keyExists(kFillTileHoles))
    {
        fillTileHoles = renderData.getDictionary()[kFillTileHoles];
    }

    AliasTable::SharedPtr fluxTable = nullptr;
    if (renderData.getDictionary().keyExists(kDicFluxTable))
    {
        fluxTable = renderData.getDictionary()[kDicFluxTable];
    }

    bool radianceReady = renderData.getDictionary()[kDicRadianceReady];
    if (sampleEliminatedVirtualLights == nullptr
        || specularRadianceContainer == nullptr
        || fluxTable == nullptr
        || emissiveTriangleFluxTable == nullptr
        || !radianceReady)
    {
        mpPixelDebug->endFrame(pRenderContext);
        return;
    }

    // build light tree if doesn't exist
    if ((mShadingMode == 3 || mEnableSpecularGathering) && mRequestRebuildTree)
    {
        Shader::DefineList defines;
        defines.add("UNBALANCED_TREE");

        if (mUseBalancedTree) mpShadingPass->getProgram()->removeDefines(defines);
        else mpShadingPass->getProgram()->addDefines(defines);
        mpShadingPass->setVars(nullptr);

        if (mUseBalancedTree) mpSpecularGatheringPass->getProgram()->removeDefines(defines);
        else mpSpecularGatheringPass->getProgram()->addDefines(defines);
        mpSpecularGatheringPass->setVars(nullptr);
    }

    if ((mShadingMode == 3 || mEnableSpecularGathering) &&mRequestRebuildTree)
    {
        if (!mLightTree) mLightTree = LightTree::Create();
        // need to have a oracle of rebuilding the tree
        mLightTree->BuildCPU(pRenderContext, sampleEliminatedVirtualLights.get(), fluxBuffer, mUseBalancedTree, mVSLRadiusFactor, mbalancedTreeQuantLevel, mUnbalancedTreeSeed);
        mRequestRebuildTree = false;

    }

    Buffer::SharedPtr tileVirtualLightContainer = renderData.getDictionary()[kDicTileVirtualLightContainer];
    uint2 tileDims = renderData.getDictionary()[kDicTileDim];
    uint tileSize = renderData.getDictionary()[kDicTileSize];
    uint tileSampleNum = renderData.getDictionary()[kDicTileSampleNum];

    Buffer::SharedPtr tileVirtualLightWeights = renderData.getDictionary()[kDicTileVirtualLightWeights];

    if (tileVirtualLightContainer == nullptr || tileVirtualLightWeights == nullptr)
    {
        debugBreak();
        return; // should not be nullptr here
    }

    if (mpNeighborOffsetBuffer == nullptr)
    {
        std::vector<uint8_t> offsets;
        offsets.resize(8192 * 2);
        FillNeighborOffsetBuffer(offsets.data());
        mpNeighborOffsetBuffer = Buffer::createTyped(ResourceFormat::RG8Snorm, 8192, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, offsets.data());
        mpNeighborOffsetBuffer->setName("ReSTIR: Neighbor Offset Buffer");
    }

    if (gpState->currentState != gpState->prevState)
    {
        mpRISBuffer = Buffer::createStructured(sizeof(float2), 512 * 128);
        mpRISBuffer->setName("ReSTIR: RIS Buffer");
    }

    uint32_t renderWidth = gpFramework->getTargetFbo()->getWidth();
    uint32_t renderHeight = gpFramework->getTargetFbo()->getHeight();
    uint32_t renderWidthBlocks = (renderWidth + 16 - 1) / 16;
    uint32_t renderHeightBlocks = (renderHeight + 16 - 1) / 16;
    uint reservoirBlockRowPitch = renderWidthBlocks * (16 * 16);
    uint reservoirArrayPitch = reservoirBlockRowPitch * renderHeightBlocks;

    const uint32_t reservoirLayers = 2;
    if (gpState->currentState != gpState->prevState)
    {
        /*
        存两个array layer
        */
        mpReservoirBuffer = Buffer::createStructured(mpInitialSamplingPass["gReservoirs"], reservoirArrayPitch * reservoirLayers);
        mpReservoirBuffer->setName("ReSTIR: Reservoir Buffer");
    }

    float oldVSLRadiusFactor = sampleEliminatedVirtualLights->getRadiusFactorForVSL();
    if (oldVSLRadiusFactor != mVSLRadiusFactor)
    {
        sampleEliminatedVirtualLights->setRadiusFactorForVSL(mVSLRadiusFactor);
        mRequestRebuildTree = true;
    }

    if (mOptionChanged)
    {
        {
            Shader::DefineList defines;
            defines.add("SCATTER_VISIBILITY", mVisibilityMode == 2 ? "1" : "0");
            addDefineToAllPasses(defines);
        }

        {
            Shader::DefineList defines;
            defines.add("_UNIFORM_SAMPLING");
            if (mUniformSampling)
            {
                addDefineToAllPasses(defines);
            }
            else
            {
                removeDefineFromAllPasses(defines);
            }
        }
        {
            Shader::DefineList defines;
            defines.add("_VSL_EVALUATION");
            if (mEnableVSLEvaluation)
            {
                addDefineToAllPasses(defines);
            }
            else
            {
                removeDefineFromAllPasses(defines);
            }
        }

        {
            Shader::DefineList defines;
            defines.add("_PRE_SAMPLING");
            if (mEnablePresampling)
            {
                addDefineToAllPasses(defines);
            }
            else
            {
                removeDefineFromAllPasses(defines);
            }
        }

        {
            Shader::DefineList defines;
            defines.add("_FILL_TILE_HOLES");
            if (fillTileHoles)
            {
                addDefineToAllPasses(defines);
            }
            else
            {
                removeDefineFromAllPasses(defines);
            }
        }

        /*{
            Shader::DefineList defines;
            defines.add("_USE_BIAS_COMPENSATION");
            if (mEnableBiasCompensation)
            {
                addDefineToAllPasses(defines);
            }
            else
            {
                removeDefineFromAllPasses(defines);
            }
        }*/
        /*
        Trigger Update
        */
        mpScene->getCamera()->setFocalDistance(100.0f + std::sinf(gpFramework->getGlobalClock().getFrame()));
        mOptionChanged = false;
    }

    uint32_t shadeInputBufferIndex = 0;
    int numSubFrames = mSubFrameMode ? mNumSubFrames : 1;
    if (mSubFrameMode) mEnableTemporalResampling = true;

    if (mShadingMode == 2)
    {
        bool savedmEnableTemporalResampling = mEnableTemporalResampling;
        for (int subframeId = 0; subframeId < numSubFrames; subframeId++)
        {
            mLastFrameOutputReservoir = mCurrentFrameOutputReservoir;
            uint32_t initialOutputBufferIndex = !mLastFrameOutputReservoir;
            uint32_t temporalInputBufferIndex = mLastFrameOutputReservoir;
            uint32_t temporalOutputBufferIndex = initialOutputBufferIndex;
            uint32_t spatialInputBufferIndex = temporalOutputBufferIndex;
            uint32_t spatialOutputBufferIndex = !spatialInputBufferIndex;
            shadeInputBufferIndex = mEnableSpatialResampling ? spatialOutputBufferIndex : temporalOutputBufferIndex;
            mCurrentFrameOutputReservoir = shadeInputBufferIndex;
            if (subframeId == 0 && mSubFrameMode) mEnableTemporalResampling = false;
            else
            {
                mEnableTemporalResampling = savedmEnableTemporalResampling;
            }
            /*
            Presampling Dispatch
            */
            if (mEnablePresampling)
            {
                PROFILE("Pre Sampling");
                //mpPixelDebug->prepareProgram(mpPreSamplingPass->getProgram(), mpPreSamplingPass->getRootVar());
                auto cb = mpPreSamplingPass["CB"];
                cb["gFrameIndex"] = numSubFrames * (uint)gpFramework->getGlobalClock().getFrame() + subframeId;
                cb["gParams"]["tileSize"] = 512u;
                cb["gParams"]["tileCount"] = 128u;
                cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
                cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
                sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
                fluxTable->setShaderData(cb["gFluxTable"]);
                mpPreSamplingPass["gScene"] = mpScene->getParameterBlock();
                mpPreSamplingPass["gRISBuffer"] = mpRISBuffer;

                mpPreSamplingPass->execute(pRenderContext, uint3(512, 128, 1));
            }

            /*
            Initial Sampling Dispatch
            */
            {
                PROFILE("Initial Sampling");
                mpPixelDebug->prepareProgram(mpInitialSamplingPass->getProgram(), mpInitialSamplingPass->getRootVar());
                auto cb = mpInitialSamplingPass["CB"];
                cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
                cb["gFrameIndex"] = numSubFrames * (uint)gpFramework->getGlobalClock().getFrame() + subframeId;
                cb["gParams"]["tileSize"] = 512u;
                cb["gParams"]["tileCount"] = 128u;
                cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
                cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
                cb["gOutputBufferIndex"] = initialOutputBufferIndex;
                cb["MISForVSL"] = mMISForVSL;
                cb["useSelfGenSamples"] = mUseSelfGenSamples;

                cb["tileSearchRadius"] = mTileSearchRadius;
                cb["numInitialSamples"] = mNumInitialSamples;
                cb["numTileVPLSamples"] = mNumTileVPLSamples;

                cb["gTileSampleNum"] = tileSampleNum;
                cb["gTileDims"] = tileDims;
                cb["gTileSize"] = tileSize;

                cb["gNumMinConeSamples"] = mNumMinConeSamples;
                cb["gNumConeSamples"] = mNumConeSamples;
                cb["gPointSampling"] = mPointSampling;


                mpInitialSamplingPass["gTileVirtualLightWeights"] = tileVirtualLightWeights;
                mpInitialSamplingPass["gTileVirtualLightContainer"] = tileVirtualLightContainer;

                fluxTable->setShaderData(cb["gFluxTable"]);

                //cb["gVirtualLightPdfTextureSize"] = uint2(mpVirtualLightPDFTexture->getWidth(), mpVirtualLightPDFTexture->getHeight());
                sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
                gBuffer->SetShaderData(cb["gGBuffer"]);
                specularRadianceContainer->setShaderData(cb["gSpecRadianceContainer"]);

                mpScene->setRaytracingShaderData(pRenderContext, mpInitialSamplingPass->getRootVar());
                //mpInitialSamplingPass["gVirtualLightPdfTexture"] = mpVirtualLightPDFTexture;
                mpInitialSamplingPass["gRISBuffer"] = mpRISBuffer;
                mpInitialSamplingPass["gReservoirs"] = mpReservoirBuffer;

                mpInitialSamplingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
            }

            /*
            Temporal Resampling Dispatch
            */
            if (mEnableTemporalResampling)
            {
                PROFILE("Temporal Resampling");
                //mpPixelDebug->prepareProgram(mpTemporalResamplingPass->getProgram(), mpTemporalResamplingPass->getRootVar());
                auto cb = mpTemporalResamplingPass["CB"];
                cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
                cb["gFrameIndex"] = numSubFrames * (uint)gpFramework->getGlobalClock().getFrame() + subframeId;
                cb["gParams"]["tileSize"] = 512u;
                cb["gParams"]["tileCount"] = 128u;
                cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
                cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
                cb["gInputBufferIndex"] = initialOutputBufferIndex;
                cb["gHistoryBufferIndex"] = temporalInputBufferIndex;
                cb["gOutputBufferIndex"] = temporalOutputBufferIndex;
                cb["gNumMinConeSamples"] = mNumMinConeSamples;
                cb["gNumConeSamples"] = mNumConeSamples;
                cb["gPointSampling"] = mPointSampling;
                cb["maxHistoryLength"] = mSubFrameMode && !mUseMaxHistoryForSubframeMode ? 99999999 : mMaxHistoryLength;

                sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
                gBuffer->SetShaderData(cb["gGBuffer"]);
                gBufferPrev->SetShaderData(cb["gPrevFrameGBuffer"]); // use the actual prev frame G-buffer
                specularRadianceContainer->setShaderData(cb["gSpecRadianceContainer"]);

                mpScene->setRaytracingShaderData(pRenderContext, mpTemporalResamplingPass->getRootVar());
                mpTemporalResamplingPass["gReservoirs"] = mpReservoirBuffer;

                mpTemporalResamplingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
            }

            /*
            Spatial Resampling Dispatch
            */
            if (mEnableSpatialResampling)
            {
                PROFILE("Spatial Resampling");
                //mpPixelDebug->prepareProgram(mpSpatialResamplingPass->getProgram(), mpSpatialResamplingPass->getRootVar());
                auto cb = mpSpatialResamplingPass["CB"];
                cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
                cb["gFrameIndex"] = numSubFrames * (uint)gpFramework->getGlobalClock().getFrame() + subframeId;
                cb["gParams"]["tileSize"] = 512u;
                cb["gParams"]["tileCount"] = 128u;
                cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
                cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
                cb["gInputBufferIndex"] = spatialInputBufferIndex;
                cb["gOutputBufferIndex"] = spatialOutputBufferIndex;
                cb["gNumMinConeSamples"] = mNumMinConeSamples;
                cb["gNumConeSamples"] = mNumConeSamples;
                cb["gPointSampling"] = mPointSampling;
                cb["gPairwiseMIS"] = mPairwiseMIS;
                cb["gBiasCorrectionForReuse"] = mBiasCorrectionForReuse;

                sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
                gBuffer->SetShaderData(cb["gGBuffer"]);
                specularRadianceContainer->setShaderData(cb["gSpecRadianceContainer"]);

                mpScene->setRaytracingShaderData(pRenderContext, mpSpatialResamplingPass->getRootVar());
                mpSpatialResamplingPass["gReservoirs"] = mpReservoirBuffer;
                mpSpatialResamplingPass["gNeighborOffsetBuffer"] = mpNeighborOffsetBuffer;

                mpSpatialResamplingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
            }

        }
    }

    /*
    Shading Dispatch
    */
    {
        PROFILE("Sample Shading");
        mpPixelDebug->prepareProgram(mpShadingPass->getProgram(), mpShadingPass->getRootVar());
        auto cb = mpShadingPass["CB"];
        cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
        cb["gParams"]["tileSize"] = 512u;
        cb["gParams"]["tileCount"] = 128u;
        cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
        cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
        cb["gInputBufferIndex"] = shadeInputBufferIndex;
        cb["gEnableDirectLighting"] = mEnableDirectLighting;
        cb["gShadingMode"] = mShadingMode;
        cb["gTileSampleNum"] = tileSampleNum;
        cb["gTileDims"] = tileDims;
        fluxTable->setShaderData(cb["gFluxTable"]);
        emissiveTriangleFluxTable->setShaderData(cb["gEmissiveTriTable"]);
        cb["gPointSampling"] = mPointSampling;
        cb["gNumFinalConeSamples"] = mNumFinalConeSamples;
        cb["gNumMinFinalConeSamples"] = mNumMinFinalConeSamples;
        cb["gVisibilityMode"] = mVisibilityMode;
        cb["gImproveCorner"] = mImproveCorner;

        if (mpEmissiveSampler)
        {
            mpEmissiveSampler->setShaderData(cb["gEmissiveLightSampler"]);
        }

        if (mLightTree && mShadingMode == 3)
        {
            cb["gLeafStartIndex"] = mLightTree->GetLeafStartIndex();
            cb["gMaxLightSamples"] = mNumInitialSamples;
            mpShadingPass["gNodeBuffer"] = mLightTree->GetNodeBuffer();

        }

        mpShadingPass["gTileVirtualLightWeights"] = tileVirtualLightWeights;
        mpShadingPass["gTileVirtualLightContainer"] = tileVirtualLightContainer;

        sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
        gBuffer->SetShaderData(cb["gGBuffer"]);
        specularRadianceContainer->setShaderData(cb["gSpecRadianceContainer"]);

        mpScene->setRaytracingShaderData(pRenderContext, mpShadingPass->getRootVar());
        mpShadingPass["gReservoirs"] = mpReservoirBuffer;
        //mpShadingPass["gEmissiveTrianglePdfTexture"] = emissiveTrianglePDFTexture;

        mpShadingOutput = renderData[kDummyOutput]->asTexture();

        if (mpRawShadingOutput == nullptr)
        {
            if (mpShadingOutput == nullptr)
            {
                logError("No Output Texture");
            }
            else
            {
                mpRawShadingOutput = Texture::create2D(mpShadingOutput->getWidth(), mpShadingOutput->getHeight(), ResourceFormat::RGBA32Float,  1, 1, nullptr, Resource::BindFlags::AllColorViews);
            }
        }

        if (mEnableSpecularGathering)
        {
            mpShadingPass["gShadingOutput"] = mpRawShadingOutput;
        }
        else
        {
            mpShadingPass["gShadingOutput"] = mpShadingOutput;
        }
        mpShadingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    mpPixelDebug->endFrame(pRenderContext);

    /*
    Specular Gathering Dispatch
    */

    if(mEnableSpecularGathering && mpRawShadingOutput != nullptr)
    {

        auto cb = mpSpecularGatheringPass["CB"];
        cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();

        cb["gNumFinalConeSamples"] = mNumFinalConeSamples;
        fluxTable->setShaderData(cb["gFluxTable"]);
        emissiveTriangleFluxTable->setShaderData(cb["gEmissiveTriTable"]);

        if (mLightTree && mShadingMode == 3)
        {
            cb["gLeafStartIndex"] = mLightTree->GetLeafStartIndex();
            cb["gMaxLightSamples"] = mNumInitialSamples;
            mpSpecularGatheringPass["gNodeBuffer"] = mLightTree->GetNodeBuffer();
        }
  
        mpSpecularGatheringPass["gRawShadingOutput"] = mpRawShadingOutput;
        mpSpecularGatheringPass["gShadingOutput"] = mpShadingOutput;

        sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
        gBuffer->SetShaderData(cb["gGBuffer"]);
        specularRadianceContainer->setShaderData(cb["gSpecRadianceContainer"]);
        mpScene->setRaytracingShaderData(pRenderContext, mpSpecularGatheringPass->getRootVar());

        mpSpecularGatheringPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    gpState->setState(3);
}

void ReSTIRPass::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;
    dirty |= widget.checkbox("improve corner", mImproveCorner);
    dirty |= widget.checkbox("sub frame mode", mSubFrameMode);
    dirty |= widget.var("Num sub frames", mNumSubFrames, 1u, 100u);
    dirty |= widget.checkbox("Use Max History for Subframe mode", mUseMaxHistoryForSubframeMode);
    dirty |= widget.checkbox("Temporal Resampling", mEnableTemporalResampling);
    dirty |= widget.var("Temporal History Length", mMaxHistoryLength, 1u, 20u);
    dirty |= widget.checkbox("Spatial Resampling", mEnableSpatialResampling);
    dirty |= widget.checkbox("VSL Evaluation", mEnableVSLEvaluation);
    dirty |= widget.checkbox("Enable Direct Lighting", mEnableDirectLighting);
    //dirty |= widget.checkbox("MIS for VSL Sampling", mMISForVSL);
    dirty |= widget.checkbox("Use Self-Gen Samples", mUseSelfGenSamples);

    dirty |= widget.var("Search Radius", mTileSearchRadius, 0u, 16u);
    dirty |= widget.var("Num Tile VPL Samples", mNumTileVPLSamples, 1u, 16u);

    dirty |= widget.var("Num Initial Samples", mNumInitialSamples, 0u, 32u);
    dirty |= widget.checkbox("VSL Point Sampling", mPointSampling);
    dirty |= widget.var("VSL Min Cone Samples", mNumMinConeSamples, 1u, 32u);
    dirty |= widget.var("VSL Max Cone Samples", mNumConeSamples, mNumMinConeSamples, 32u);
    dirty |= widget.var("VSL Min Final Cone Samples", mNumMinFinalConeSamples, 1u, 200u);
    dirty |= widget.var("VSL Max Final Cone Samples", mNumFinalConeSamples, mNumMinFinalConeSamples, 200u);
    dirty |= widget.checkbox("Enable Presampling", mEnablePresampling);
    dirty |= widget.checkbox("Uniform Sampling In RIS", mUniformSampling);
    dirty |= widget.checkbox("Bias Correction For Reuse", mBiasCorrectionForReuse);
    dirty |= widget.checkbox("Pairwise MIS", mPairwiseMIS);

    bool specularGatheringChanged = widget.checkbox("Specuar Gathering", mEnableSpecularGathering);
    dirty |= specularGatheringChanged;
    if (specularGatheringChanged && mEnableSpecularGathering)
    {
        mRequestRebuildTree = true;
    }

    dirty |= widget.var("VSL Radius Factor", mVSLRadiusFactor, 0.0001f, 1000.0f);
    //dirty |= widget.checkbox("BiasCompensation", mEnableBiasCompensation);

    Gui::DropdownList shadingModeList;
    shadingModeList.push_back({ 0, "Groundtruth" });
    shadingModeList.push_back({ 1, "RIS" });
    shadingModeList.push_back({ 2, "ReSTIR" });
    shadingModeList.push_back({ 3, "SLC" });
    shadingModeList.push_back({ 4, "RichPhotonmapping" });

    bool changeShadingMode = widget.dropdown("Shading Mode", shadingModeList, mShadingMode);

    dirty |= changeShadingMode;

    if (changeShadingMode && mShadingMode == 3) mRequestRebuildTree = true;

    Gui::DropdownList visibilityModeList;
    visibilityModeList.push_back({ 0, "PointToPoint" });
    visibilityModeList.push_back({ 1, "PointToSphere" });
    visibilityModeList.push_back({ 2, "ScatterRay" });

    bool changeVisibilityMode = widget.dropdown("Visibility Mode", visibilityModeList, mVisibilityMode);

    dirty |= changeVisibilityMode;


    bool changeTreeType= widget.checkbox("Balanced Tree", mUseBalancedTree);

    dirty |= changeTreeType;

    mRequestRebuildTree |= changeTreeType;

    bool changeSeed = widget.var("Unbalanced Tree Seed", mUnbalancedTreeSeed, 1u, 1000u);

    dirty |= changeSeed;

    mRequestRebuildTree |= changeSeed;

    bool changeQuantLevel = widget.var("Balanced Tree Quant Level", mbalancedTreeQuantLevel, 1u, 16u);

    dirty |= changeQuantLevel;

    mRequestRebuildTree |= changeQuantLevel;

    if (auto logGroup = widget.group("Logging"))
    {
        // Pixel debugger.
        mpPixelDebug->renderUI(logGroup);
    }

    if (dirty) mOptionChanged = true;
}

void ReSTIRPass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpScene)
    {
        recompile();
    }
}

void ReSTIRPass::addDefineToAllPasses(Shader::DefineList defines)
{
    mpPrepareLightsPass->getProgram()->addDefines(defines);
    mpPrepareLightsPass->setVars(nullptr); // Trigger recompile

    mpPreSamplingPass->getProgram()->addDefines(defines);
    mpPreSamplingPass->setVars(nullptr); // Trigger recompile

    mpInitialSamplingPass->getProgram()->addDefines(defines);
    mpInitialSamplingPass->setVars(nullptr);

    mpTemporalResamplingPass->getProgram()->addDefines(defines);
    mpTemporalResamplingPass->setVars(nullptr);

    mpSpatialResamplingPass->getProgram()->addDefines(defines);
    mpSpatialResamplingPass->setVars(nullptr);

    mpShadingPass->getProgram()->addDefines(defines);
    mpShadingPass->setVars(nullptr);



    mpSpecularGatheringPass->getProgram()->addDefines(defines);
    mpSpecularGatheringPass->setVars(nullptr);

}

void ReSTIRPass::removeDefineFromAllPasses(Shader::DefineList defines)
{
    mpPrepareLightsPass->getProgram()->removeDefines(defines);
    mpPrepareLightsPass->setVars(nullptr); // Trigger recompile

    mpPreSamplingPass->getProgram()->removeDefines(defines);
    mpPreSamplingPass->setVars(nullptr); // Trigger recompile

    mpInitialSamplingPass->getProgram()->removeDefines(defines);
    mpInitialSamplingPass->setVars(nullptr);

    mpTemporalResamplingPass->getProgram()->removeDefines(defines);
    mpTemporalResamplingPass->setVars(nullptr);

    mpSpatialResamplingPass->getProgram()->removeDefines(defines);
    mpSpatialResamplingPass->setVars(nullptr);

    mpShadingPass->getProgram()->removeDefines(defines);
    mpShadingPass->setVars(nullptr);

    mpSpecularGatheringPass->getProgram()->addDefines(defines);
    mpSpecularGatheringPass->setVars(nullptr);
}

bool ReSTIRPass::prepareLights(RenderContext* pRenderContext)
{
    bool lightingChanged = false;

    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::RenderSettingsChanged))
    {
        lightingChanged = true;
        mNeedRecompile = true;
    }

    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::EnvMapChanged))
    {
        mpEnvMapSampler = nullptr;
        lightingChanged = true;
        mNeedRecompile = true;
    }

    if (mpScene->useEnvLight())
    {
        if (!mpEnvMapSampler)
        {
            mpEnvMapSampler = EnvMapSampler::create(pRenderContext, mpScene->getEnvMap());
            lightingChanged = true;
            mNeedRecompile = true;
        }
    }
    else
    {
        if (mpEnvMapSampler)
        {
            mpEnvMapSampler = nullptr;
            lightingChanged = true;
            mNeedRecompile = true;
        }
    }

    if (mpScene->getRenderSettings().useEmissiveLights)
    {
        mpScene->getLightCollection(pRenderContext);
    }

    if (mpScene->useEmissiveLights())
    {
        if (!mpEmissiveSampler)
        {
            const auto& pLights = mpScene->getLightCollection(pRenderContext);
            assert(pLights && pLights->getActiveLightCount() > 0);

            mpEmissiveSampler = LightBVHSampler::create(pRenderContext, mpScene);

            lightingChanged = true;
            mNeedRecompile = true;
        }
    }
    else
    {
        if (mpEmissiveSampler)
        {
            mpEmissiveSampler = nullptr;
            lightingChanged = true;
            mNeedRecompile = true;
        }
    }

    if (mpEmissiveSampler)
    {
        lightingChanged |= mpEmissiveSampler->update(pRenderContext);
    }

    return lightingChanged;
}

void ReSTIRPass::recompile()
{
    Shader::DefineList defines = mpScene->getSceneDefines();
    defines.add(mpSampleGenerator->getDefines());
    defines.add("_MS_DISABLE_ALPHA_TEST");
    defines.add("_DEFAULT_ALPHA_TEST");
    defines.add(kEmissiveTriBase, kEmissiveTriBaseVal);
    defines.add("SCATTER_VISIBILITY", mVisibilityMode == 2 ? "1" : "0");
    if (mpEmissiveSampler != nullptr)
    {
        defines.add(mpEmissiveSampler->getDefines());
    }
    if (mEnableVSLEvaluation)
    {
        defines.add("_VSL_EVALUATION");
    }

    if (mEnablePresampling)
    {
        defines.add("_PRE_SAMPLING");
    }

    if (mEnableBiasCompensation)

    {
        //defines.add("_USE_BIAS_COMPENSATION");
    }

    addDefineToAllPasses(defines);
}


