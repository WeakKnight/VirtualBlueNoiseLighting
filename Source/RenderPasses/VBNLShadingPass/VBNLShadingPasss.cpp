#include "VBNLShadingPass.h"
#include "Utils/VirtualLight/MegaTextureContainer.h"
#include "Utils/VirtualLight/VirtualLightContainer.h"
#include "Utils/Sampling/AliasTable.h"
#include "Utils/Debug/PixelDebug.h"
#include "RenderPasses/GBufferPass.h"
#include "Experimental/Scene/Lights/LightBVHSampler.h"

namespace
{
    const char kDummy[] = "dummy";
    const char kDummyInput[] = "input";
    const char kDummyOutput[] = "output";
    const char kDesc[] = "Insert pass description here";

    const char kShortDistance[] = "shortDistance";
    const char kShortDistanceRange[] = "shortDistanceRange";

    const char kShortDistanceNumVSLs[] = "shortDistanceNumVSLs";
    const char kLongDistanceNumVSLs[] = "longDistanceNumVSLs";

    const char kUseNewThresholds[] = "useNewThresholds";

    const char kMaxPathIntensity[] = "MaxPathIntensity";
    const char kBounceNum[] = "BounceNum";

    const char kUseVirtualLight[] = "UseVirtualLight";
    const char kVirtualLightUseNEE[] = "VirtualLightUseNEE";
    const char kEnablePreciseVirtualLightNEE[] = "EnablePreciseVirtualLightNEE";
    const char kVirtualLightUseMIS[] = "VirtualLightUseMIS";
    const char kRoughnessThreshold[] = "RoughnessThreshold";
    const char kUseRIS[] = "UseRIS";
    const char kNEESamples[] = "NEESamples";

    const char kUseVirtualLightBVH[] = "UseVirtualLightBVH";

    const char kEnableTemporalResampling[] = "enableTemporalResampling";
    const char kEnableSpatialResampling[] = "enableSpatialResampling";
    const char kShadingMode[] = "shadingMode";
    const char kMISWithPowerSampling[] = "MISWithPowerSampling";
    const char kFinalMISWithPowerSampling[] = "FinalMISWithPowerSampling";

    const char kShadingNormalCheck[] = "ShadingNormalCheck";
    const char kShadingMaterialCheck[] = "ShadingMaterialCheck";

    const char kUseTraditionalVXL[] = "UseTraditionalVXL";
    const char kUseRichVXL[] = "UseRichVXL";
    const char kUseVSL[] = "UseVSL";
    const char kVSLRadiusScaler[] = "VSLRadiusScaler";

    const char kMaxBSDFSearchCount[] = "MaxBSDFSearchCount";

    const std::string kDicFluxTable = "fluxTable";
    const std::string kDicPhotons = "photons";
    const std::string kDicSampleEliminatedVirtualLights = "sampleEliminatedVirtualLights";
    const std::string kDicSpecularRadianceContainer = "specularRadianceContainer";
    const std::string kDicCurVirtualLights = "curVirtualLights";
    const std::string kDicMaxBSDFSearchCount = "MaxBSDFSearchCount";

    const std::string kGBuffer = "GBuffer";
    const std::string kGBufferPrevFrame = "GBufferPrevFrame";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

static void regPythonApi(pybind11::module& m)
{
    pybind11::class_<VBNLShadingPass, RenderPass, VBNLShadingPass::SharedPtr> pass(m, "VBNLShadingPass");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary & lib)
{
    lib.registerClass("VBNLShadingPass", kDesc, VBNLShadingPass::create);
    ScriptBindings::registerBinding(regPythonApi);
}

void FillNeighborOffsetBuffer(uint8_t* buffer)
{
    int R = 250;
    const float phi2 = 1.0f / 1.3247179572447f;
    uint32_t num = 0;
    float u = 0.5f;
    float v = 0.5f;
    while (num < 8192 * 2)
    {
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

VBNLShadingPass::SharedPtr VBNLShadingPass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new VBNLShadingPass);
    for (const auto& [key, value] : dict)
    {
        if (key == kShortDistance)
        {
            pPass->mShortDistance = value;
        }
        else if (key == kShortDistanceRange)
        {
            pPass->mShortDistanceRange = value;
        }
        else if (key == kShortDistanceNumVSLs)
        {
            pPass->mShortDistanceNumVSLs = value;
        }
        else if (key == kLongDistanceNumVSLs)
        {
            pPass->mLongDistanceNumVSLs = value;
        }
        else if (key == kUseNewThresholds)
        {
            pPass->mUseNewThresholds = value;
        }
        else if (key == kMaxPathIntensity)
        {
            pPass->mMaxPathIntensity = value;
        }
        else if (key == kBounceNum)
        {
            pPass->mBounceNum = value;
        }
        else if (key == kVirtualLightUseNEE)
        {
            pPass->mUseVirtualLightNEE = value;
        }
        else if (key == kEnablePreciseVirtualLightNEE)
        {
            pPass->mEnablePreciseVirtualLightNEE = value;
        }
        else if (key == kVirtualLightUseMIS)
        {
            pPass->mUseVirtualLightMIS = value;
        }
        else if (key == kRoughnessThreshold)
        {
            pPass->mRoughnessThreshold = value;
        }
        else if (key == kUseRIS)
        {
            pPass->mUseRIS = value;
        }
        else if (key == kNEESamples)
        {
            pPass->mNEESamples = value;
        }
        else if (key == kUseVirtualLight)
        {
            pPass->mUseVirtualLight = value;
        }
        else if (key == kUseVirtualLightBVH)
        {
            pPass->mUseVirtualLightBVH = value;
        }
        else if (key == kEnableSpatialResampling)
        {
            pPass->mEnableSpatialResampling = value;
        }
        else if (key == kEnableTemporalResampling)
        {
            pPass->mEnableTemporalResampling = value;
        }
        else if (key == kShadingMode)
        {
            pPass->mShadingMode = value;
        }
        else if (key == kShadingNormalCheck)
        {
            pPass->mShadingNormalCheck = value;
        }
        else if (key == kShadingMaterialCheck)
        {
            pPass->mShadingMaterialCheck = value;
        }
        else if (key == kUseTraditionalVXL)
        {
            pPass->mUseTraditionalVXL = value;
        }
        else if (key == kUseRichVXL)
        {
            pPass->mUseRichVXL = value;
        }
        else if (key == kUseVSL)
        {
            pPass->mUseVSL = value;
        }
        else if (key == kVSLRadiusScaler)
        {
            pPass->mVSLRadiusScaler = value;
        }
        else if (key == kMaxBSDFSearchCount)
        {
            pPass->mMaxBSDFSearchCount = value;
        }
        else if (key == kMISWithPowerSampling)
        {
            pPass->mMISWithPowerSampling = value;
        }
    }

    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);
    pPass->mpPixelDebug = PixelDebug::create();

    Program::Desc restirShadingPassDesc;
    restirShadingPassDesc.addShaderLibrary("RenderPasses/VBNLShadingPass/VBNLShading.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpShadingPass = ComputePass::create(restirShadingPassDesc, Program::DefineList(), false);

    Program::Desc initialSamplingPassDesc;
    initialSamplingPassDesc.addShaderLibrary("RenderPasses/VBNLShadingPass/BSDFReSTIRInitialSampling.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpInitialSamplingPass = ComputePass::create(initialSamplingPassDesc, Program::DefineList(), false);

    Program::Desc temporalResamplingPassDesc;
    temporalResamplingPassDesc.addShaderLibrary("RenderPasses/VBNLShadingPass/BSDFReSTIRTemporalResampling.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpTemporalResamplingPass = ComputePass::create(temporalResamplingPassDesc, Program::DefineList(), false);

    Program::Desc spatialResamplingPassDesc;
    spatialResamplingPassDesc.addShaderLibrary("RenderPasses/VBNLShadingPass/BSDFReSTIRSpatialResampling.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpSpatialResamplingPass = ComputePass::create(spatialResamplingPassDesc, Program::DefineList(), false);

    return pPass;
}

std::string VBNLShadingPass::getDesc()
{
    return kDesc;
}

Dictionary VBNLShadingPass::getScriptingDictionary()
{
    Dictionary d;
    d[kShortDistance] = mShortDistance;
    d[kShortDistanceRange] = mShortDistanceRange;
    d[kShortDistanceNumVSLs] = mShortDistanceNumVSLs;
    d[kLongDistanceNumVSLs] = mLongDistanceNumVSLs;
    d[kUseNewThresholds] = mUseNewThresholds;

    d[kMaxPathIntensity] = mMaxPathIntensity;
    d[kBounceNum] = mBounceNum;
    d[kVirtualLightUseNEE] = mUseVirtualLightNEE;
    d[kEnablePreciseVirtualLightNEE] = mEnablePreciseVirtualLightNEE;
    d[kVirtualLightUseMIS] = mUseVirtualLightMIS;
    d[kRoughnessThreshold] = mRoughnessThreshold;
    d[kUseRIS] = mUseRIS;
    d[kNEESamples] = mNEESamples;

    d[kEnableSpatialResampling] = mEnableSpatialResampling;
    d[kEnableTemporalResampling] = mEnableTemporalResampling;
    d[kShadingMode] = mShadingMode;
    d[kMISWithPowerSampling] = mMISWithPowerSampling;
    d[kFinalMISWithPowerSampling] = mFinalMISWithPowerSampling;

    d[kShadingNormalCheck] = mShadingNormalCheck;
    d[kShadingMaterialCheck] = mShadingMaterialCheck;

    d[kUseTraditionalVXL] = mUseTraditionalVXL;
    d[kUseRichVXL] = mUseRichVXL;
    d[kUseVSL] = mUseVSL;
    d[kVSLRadiusScaler] = mVSLRadiusScaler;

    d[kMaxBSDFSearchCount] = mMaxBSDFSearchCount;

    return d;
}

RenderPassReflection VBNLShadingPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "useless dummy Input");
    reflector.addOutput(kDummyOutput, "shading Output").bindFlags(ResourceBindFlags::RenderTarget | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource).format(ResourceFormat::RGBA32Float);
    return reflector;
}

void VBNLShadingPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
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

    AliasTable::SharedPtr fluxTable = nullptr;
    if (!fluxTable)
    {
        if (renderData.getDictionary().keyExists(kDicFluxTable))
        {
            fluxTable = renderData.getDictionary()[kDicFluxTable];
        }
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
        renderData.getDictionary()[kDicMaxBSDFSearchCount] = mMaxBSDFSearchCount;
        recompile();
    }

    VirtualLightContainer::SharedPtr sampleEliminatedVirtualLights = nullptr;
    if (renderData.getDictionary().keyExists(kDicSampleEliminatedVirtualLights))
    {
        sampleEliminatedVirtualLights = renderData.getDictionary()[kDicSampleEliminatedVirtualLights];
    }
    else
    {
        sampleEliminatedVirtualLights = renderData.getDictionary()[kDicPhotons];
        renderData.getDictionary()[kDicSampleEliminatedVirtualLights] = sampleEliminatedVirtualLights;

        Buffer::SharedPtr radianceBuffer = sampleEliminatedVirtualLights->getIncidentRadianceBuffer();
        Buffer::SharedPtr readBuffer = Buffer::create(radianceBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read, nullptr);
        pRenderContext->copyBufferRegion(readBuffer.get(), 0, radianceBuffer.get(), 0, radianceBuffer->getSize());
        pRenderContext->flush(true);
        uint lightCount = sampleEliminatedVirtualLights->getCount();
        std::vector<float> weights(lightCount);

        float3* colorData = (float3*)readBuffer->map(Buffer::MapType::Read);
        for (size_t i = 0; i < lightCount; i++)
        {
            float3 col = colorData[i];
            weights[i] = (col.x + col.y + col.z);
        }
        readBuffer->unmap();

        auto fluxBuffer = Buffer::createStructured(sizeof(float), sampleEliminatedVirtualLights->getCount(), ResourceBindFlags::ShaderResource, Buffer::CpuAccess::None, weights.data());
        fluxBuffer->setName("EstimatePass: Flux Buffer");

        sampleEliminatedVirtualLights->setFluxBuffer(fluxBuffer);
        std::mt19937 rng;
        fluxTable = AliasTable::create(weights, rng);
        renderData.getDictionary()[kDicFluxTable] = fluxTable;
    }

    MegaTextureContainer::SharedPtr specularRadianceContainer = nullptr;
    if (renderData.getDictionary().keyExists(kDicSpecularRadianceContainer))
    {
        specularRadianceContainer = renderData.getDictionary()[kDicSpecularRadianceContainer];
    }

    mpVirtualLights = sampleEliminatedVirtualLights;
    if (specularRadianceContainer != nullptr)
    {
        mMegaTexItemSize = specularRadianceContainer->GetPerItemSizeHQ();
    }

    recordUserInteraction();

    // build light BVH
    if (!mpVirtualLightEmissiveSampler || (mVirtualLightEmissiveSamplerOptionsChanged && mUseVirtualLightBVH))
    {
        mVirtualLightEmissiveSamplerOptionsChanged = false;
        mpScene->setVirtualLightCollection(sampleEliminatedVirtualLights);
        sampleEliminatedVirtualLights->prepareReadBuffers(pRenderContext);
        if (!mpVirtualLightEmissiveSampler)
        {
            LightBVHSampler::Options options;
            //options.buildOptions.splitHeuristicSelection = LightBVHBuilder::SplitHeuristic::Equal;
            mpVirtualLightEmissiveSampler = LightBVHSampler::create(pRenderContext, mpScene, options, true);
        }
        mpVirtualLightEmissiveSampler->update(pRenderContext);
        sampleEliminatedVirtualLights->clearReadBuffers(pRenderContext);
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
        mpReservoirBuffer = Buffer::createStructured(mpShadingPass["gReservoirs"], reservoirArrayPitch * reservoirLayers);
        mpReservoirBuffer->setName("ReSTIR: Reservoir Buffer");

        logInfo("[===Time Stats===]Total Preparation Time Is " + std::to_string(gpState->time));
    }

    mLastFrameOutputReservoir = mCurrentFrameOutputReservoir;

    uint32_t initialOutputBufferIndex = !mLastFrameOutputReservoir;
    uint32_t temporalInputBufferIndex = mLastFrameOutputReservoir;
    uint32_t temporalOutputBufferIndex = initialOutputBufferIndex;
    uint32_t spatialInputBufferIndex = temporalOutputBufferIndex;
    uint32_t spatialOutputBufferIndex = !spatialInputBufferIndex;
    uint32_t shadeInputBufferIndex = mEnableSpatialResampling ? spatialOutputBufferIndex : temporalOutputBufferIndex;

    mCurrentFrameOutputReservoir = shadeInputBufferIndex;

    if (mShadingMode == 1)
    {
        /*
        Initial Sampling Dispatch
        */
        {
            PROFILE("Initial Sampling");
            mpPixelDebug->prepareProgram(mpInitialSamplingPass->getProgram(), mpInitialSamplingPass->getRootVar());
            auto cb = mpInitialSamplingPass["CB"];
            cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
            cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
            cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
            cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
            cb["gOutputBufferIndex"] = initialOutputBufferIndex;
            cb["gMISWithPowerSampling"] = mMISWithPowerSampling;
            cb["gNumBSDFSamples"] = mNumBSDFSamples;
            cb["gNumPowerSamples"] = mNumPowerSamples;

            //cb["gVirtualLightPdfTextureSize"] = uint2(mpVirtualLightPDFTexture->getWidth(), mpVirtualLightPDFTexture->getHeight());
            sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
            gBuffer->SetShaderData(cb["gGBuffer"]);
            specularRadianceContainer->setShaderData(cb["gSpecRadianceContainer"]);
            fluxTable->setShaderData(cb["gFluxTable"]);

            mpScene->setRaytracingShaderData(pRenderContext, mpInitialSamplingPass->getRootVar());
            mpInitialSamplingPass["gReservoirs"] = mpReservoirBuffer;

            mpInitialSamplingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        }

        /*
        Temporal Resampling Dispatch
        */
        if (mEnableTemporalResampling)
        {
            PROFILE("Temporal Resampling");
            mpPixelDebug->prepareProgram(mpTemporalResamplingPass->getProgram(), mpTemporalResamplingPass->getRootVar());
            auto cb = mpTemporalResamplingPass["CB"];
            cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
            cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
            cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
            cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
            cb["gInputBufferIndex"] = initialOutputBufferIndex;
            cb["gHistoryBufferIndex"] = temporalInputBufferIndex;
            cb["gOutputBufferIndex"] = temporalOutputBufferIndex;

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
            mpPixelDebug->prepareProgram(mpSpatialResamplingPass->getProgram(), mpSpatialResamplingPass->getRootVar());
            auto cb = mpSpatialResamplingPass["CB"];
            cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
            cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
            cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
            cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
            cb["gInputBufferIndex"] = spatialInputBufferIndex;
            cb["gOutputBufferIndex"] = spatialOutputBufferIndex;

            sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
            gBuffer->SetShaderData(cb["gGBuffer"]);
            specularRadianceContainer->setShaderData(cb["gSpecRadianceContainer"]);

            mpScene->setRaytracingShaderData(pRenderContext, mpSpatialResamplingPass->getRootVar());
            mpSpatialResamplingPass["gReservoirs"] = mpReservoirBuffer;
            mpSpatialResamplingPass["gNeighborOffsetBuffer"] = mpNeighborOffsetBuffer;

            mpSpatialResamplingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        }
    }

    if (mShadingMode == 2)
    {
        mCurrentVPLIndex += 1;
        if (mCurrentVPLIndex >= sampleEliminatedVirtualLights->getCount())
        {
            mCurrentVPLIndex = 0;
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
        cb["gUseNewThresholds"] = mUseNewThresholds;
        cb["gShortDistance"] = mShortDistance;
        cb["gShortDistanceRange"] = mShortDistanceRange;
        cb["gShortDistanceNumVSLs"] = mShortDistanceNumVSLs;
        cb["gLongDistanceNumVSLs"] = mLongDistanceNumVSLs;

        cb["gMaxPathIntensity"] = mMaxPathIntensity;
        cb["gNEESamples"] = mNEESamples;
        cb["gUseRIS"] = mUseRIS;
        cb["gRoughnessThreshold"] = mRoughnessThreshold;
        cb["gUseVirtualLightBVH"] = mUseVirtualLightBVH;

        //
        cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
        cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
        cb["gInputBufferIndex"] = shadeInputBufferIndex;
        cb["gMISWithPowerSampling"] = mFinalMISWithPowerSampling;
        cb["gNumBSDFSamples"] = mFinalNumBSDFSamples;
        cb["gNumPowerSamples"] = mFinalNumPowerSamples;
        if (mShadingMode == 2)
        {
            cb["gCurrentVPLIndex"] = mCurrentVPLIndex;
        }
        mpShadingPass["gReservoirs"] = mpReservoirBuffer;
        //

        fluxTable->setShaderData(cb["gVirtualLightFluxTable"]);

        if (mpEmissiveSampler)
        {
            mpEmissiveSampler->setShaderData(cb["gEmissiveLightSampler"]);
        }
        if (mpEnvMapSampler)
        {
            mpEnvMapSampler->setShaderData(cb["gEnvMapSampler"]);
        }
        if (mpVirtualLightEmissiveSampler)
        {
            mpVirtualLightEmissiveSampler->setShaderData(cb["gVirtualLightBVHSampler"]);
        }

        sampleEliminatedVirtualLights->setRadiusFactorForVSL(mVSLRadiusScaler);
        sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
        gBuffer->SetShaderData(cb["gGBuffer"]);
        if (specularRadianceContainer)
        {
            specularRadianceContainer->setShaderData(cb["gSpecRadianceContainer"]);
        }

        mpScene->setRaytracingShaderData(pRenderContext, mpShadingPass->getRootVar());

        mpShadingOutput = renderData[kDummyOutput]->asTexture();

        mpShadingPass["gShadingOutput"] = mpShadingOutput;
        mpShadingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    mpPixelDebug->endFrame(pRenderContext);

    gpState->setState(3);

    replayUserInteraction();

    if (mCaptureAfterCertainTime && mTimer.getDuration() >= mTimeForCapture)
    {
        renderData.getDictionary()["freezeOutput"] = true;
    }
    else
        renderData.getDictionary()["freezeOutput"] = false;
}

void VBNLShadingPass::renderUI(Gui::Widgets& widget)
{
    if (auto logGroup = widget.group("Logging"))
    {
        // Pixel debugger.
        mpPixelDebug->renderUI(logGroup);
    }

    if (auto virtualLightGroup = widget.group("Virtual Light Stats"))
    {
        if (mpVirtualLights)
        {
            mpVirtualLights->renderUI(virtualLightGroup, mMegaTexItemSize);
        }
    }

    if (mpVirtualLightEmissiveSampler)
    {
        if (auto emissiveGroup = widget.group("Emissive sampler options"))
        {
            if (mpVirtualLightEmissiveSampler->renderUI(emissiveGroup))
            {
                mVirtualLightEmissiveSamplerOptions = std::static_pointer_cast<LightBVHSampler>(mpVirtualLightEmissiveSampler)->getOptions();
                mVirtualLightEmissiveSamplerOptionsChanged = true;
                mNeedRecompile = true;
            }
        }
    }

    if (auto group = widget.group("Recording"))
    {
        if (!mCaptureAfterCertainTime)
        {
            if (widget.button("Capture After Certain Time"))
            {
                mCaptureAfterCertainTime = true;
                mNeedClear = true;
                mTimer.startDurationFromCurrentTime();
            }
        }
        else
        {
            if (widget.button("Finish Capture"))
            {
                mCaptureAfterCertainTime = false;
                mNeedClear = true;
            }
        }

        widget.var("Render time for capture (ms)", mTimeForCapture, 1.f, 100000.f);

        if (widget.button(mRecordUserInteraction ? "Stop Record" : "Record User Interaction"))
        {
            mRecordUserInteraction = !mRecordUserInteraction;
            mRecordedTotalFrameTime = 0.f;
            if (!mRecordUserInteraction)
            {
                if (mUserInteractionOutputFile.is_open())
                    mUserInteractionOutputFile.close();
            }
        }

        if (widget.button("Load User Input File"))
        {
            mCameraTimeSequence.clear();
            mCameraUpSequence.clear();
            mCameraTargetSequence.clear();
            mCameraPositionSequence.clear();
            std::string filename;
            FileDialogFilterVec txtFile;
            txtFile.push_back({ "txt", "txt file" });
            if (openFileDialog(txtFile, filename))
            {
                std::ifstream f(filename);
                // load camera positions and targets
                while (!f.eof())
                {
                    double t;
                    f >> t;
                    mCameraTimeSequence.push_back(t);
                    if (f.fail() || f.eof()) break;
                    float x, y, z;
                    f >> x >> y >> z;
                    mCameraPositionSequence.push_back(float3(x, y, z));
                    f >> x >> y >> z;
                    mCameraTargetSequence.push_back(float3(x, y, z));
                    f >> x >> y >> z;
                    mCameraUpSequence.push_back(float3(x, y, z));
                }
                std::cout << "Loaded " << mCameraPositionSequence.size() << " frames user input" << std::endl;

                f.close();
            }
        }

        if (widget.button(mReplayUserInteraction ? "Stop Playing User Input" : "Play Loaded User Input"))
        {
            mReplayUserInteraction = !mReplayUserInteraction;
            mRecordedTotalFrameTime = 0.f;
        }
    }

    Gui::DropdownList shadingModeList;
    shadingModeList.push_back({ 0, "Unified" });
    shadingModeList.push_back({ 1, "ReSTIR" });
    if (mUseTraditionalVXL)
    {
        shadingModeList.push_back({ 2, "Naive" });
    }
    mNeedRecompile |= widget.dropdown("Shading Mode", shadingModeList, mShadingMode);
    mNeedRecompile |= widget.checkbox("Use Mixture BSDF", mUseMixtureBSDF);

    mNeedRecompile |= widget.checkbox("Use Virtual Light", mUseVirtualLight);
    if (mUseVirtualLight)
    {
        mNeedRecompile |= widget.checkbox("Use Traditional VXL", mUseTraditionalVXL);
        if (mUseTraditionalVXL)
        {
            mNeedRecompile |= widget.checkbox("Use VSL", mUseVSL);
            if (mUseVSL)
            {
                mNeedClear |= widget.var("VSL Radius Scaler", mVSLRadiusScaler);
            }
            mNeedRecompile |= widget.checkbox("Use RichVXL", mUseRichVXL);
        }
        else
        {
            mNeedClear |= widget.checkbox("Use New Thresholds", mUseNewThresholds);
            mNeedClear |= widget.var("Short Distance", mShortDistance, 0.0f, 10.0f);
            mNeedClear |= widget.var("Short Distance Range", mShortDistanceRange, 0.0f, 10.0f);
            mNeedClear |= widget.var("Short Distance Num VSLs", mShortDistanceNumVSLs, 0.0f, 10000.f);
            mNeedClear |= widget.var("Long Distance Num VSLs", mLongDistanceNumVSLs, 0.0f, 10000.f);

            mNeedRecompile |= widget.checkbox("Use Virtual Light NEE", mUseVirtualLightNEE);
            if (mUseVirtualLightNEE)
            {
                mNeedRecompile |= widget.checkbox("Enable Precise Virtual Light NEE", mEnablePreciseVirtualLightNEE);
            }
            mNeedRecompile |= widget.checkbox("Use Virtual Light MIS", mUseVirtualLightMIS);
            mNeedRecompile |= widget.checkbox("Shading Normal Check", mShadingNormalCheck);
            mNeedRecompile |= widget.checkbox("Shading Material Check", mShadingMaterialCheck);
            mNeedRecompile |= widget.checkbox("Use RIS", mUseRIS);
            mNeedRecompile |= widget.var("Max BSDF Search Count", mMaxBSDFSearchCount);
        }

        mNeedClear |= widget.var("Roughness Threshold", mRoughnessThreshold, 0.f, 1.0f);
        mNeedRecompile |= widget.var("NEE Samples", mNEESamples);
        mNeedRecompile |= widget.checkbox("Use Virtual Light BVH", mUseVirtualLightBVH);
    }

    mNeedRecompile |= widget.checkbox("Output Depth", mOutputDepth);
    mNeedRecompile |= widget.checkbox("Output Outgoing Radiance", mOutputOutgoingRadiance);

    mNeedRecompile |= widget.var("Bounce Num", mBounceNum);
    mNeedClear |= widget.var("Max Path Intensity", mMaxPathIntensity);

    if (mShadingMode == 1u)
    {
        if (auto group = widget.group("ReSTIR"))
        {
            mNeedRecompile |= widget.checkbox("MIS with Power Sampling", mMISWithPowerSampling);
            mNeedRecompile |= widget.var("Num BSDF Samples", mNumBSDFSamples, 0u, 32u);
            mNeedRecompile |= widget.var("Num Power Samples", mNumPowerSamples, 0u, 32u);

            mNeedRecompile |= widget.checkbox("Temporal Resampling", mEnableTemporalResampling);
            mNeedRecompile |= widget.checkbox("Spatial Resampling", mEnableSpatialResampling);

            mNeedRecompile |= widget.checkbox("Final MIS with Power Sampling", mFinalMISWithPowerSampling);
            mNeedRecompile |= widget.var("Final Num BSDF Samples", mFinalNumBSDFSamples, 1u, 32u);
            mNeedRecompile |= widget.var("Final Num Power Samples", mFinalNumPowerSamples, 1u, 32u);
        }
    }
}

void VBNLShadingPass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpNeighborOffsetBuffer == nullptr)
    {
        std::vector<uint8_t> offsets;
        offsets.resize(8192 * 2);
        FillNeighborOffsetBuffer(offsets.data());
        mpNeighborOffsetBuffer = Buffer::createTyped(ResourceFormat::RG8Snorm, 8192, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, offsets.data());
        mpNeighborOffsetBuffer->setName("ReSTIR: Neighbor Offset Buffer");
    }

    if (mpScene)
    {
        recompile();
    }
}

bool VBNLShadingPass::onKeyEvent(const KeyboardEvent& keyEvent)
{
    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == KeyboardEvent::Key::R)
    {
        mReplayUserInteraction = true;
        mRecordedTotalFrameTime = 0.f;
        return true;
    }
    return false;
}

void VBNLShadingPass::onHotReload(HotReloadFlags reloaded)
{
    recompile();
}

bool VBNLShadingPass::prepareLights(RenderContext* pRenderContext)
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

void VBNLShadingPass::recompile()
{
    if (!mUseVirtualLight)
    {
        mShadingMode = 0u;
    }

    if (mUseTraditionalVXL)
    {
        if (mShadingMode == 1)
        {
            mShadingMode = 0u;
        }
        mShortDistance = 0.0f;
        mShortDistanceRange = 0.0f;
        mUseVirtualLightNEE = true;
        mUseVirtualLightMIS = false;
        mUseRIS = false;
    }

    Shader::DefineList defines = mpScene->getSceneDefines();
    defines.add(mpSampleGenerator->getDefines());
    defines.add("_MS_DISABLE_ALPHA_TEST");
    defines.add("_DEFAULT_ALPHA_TEST");
    defines.add("USE_ENV_LIGHT", mpScene->useEnvLight() ? "1" : "0");
    defines.add("USE_ANALYTIC_LIGHTS", mpScene->useAnalyticLights() ? "1" : "0");
    defines.add("USE_EMISSIVE_LIGHTS", mpScene->useEmissiveLights() ? "1" : "0");

    defines.add("USE_VIRTUAL_LIGHTS", mUseVirtualLight ? "1" : "0");
    defines.add("USE_VIRTUAL_LIGHT_NEE", mUseVirtualLightNEE ? "1" : "0");
    defines.add("ENABLE_PRECISE_VIRTUAL_LIGHT_NEE", mEnablePreciseVirtualLightNEE ? "1" : "0");
    defines.add("USE_VIRTUAL_LIGHT_MIS", mUseVirtualLightMIS ? "1" : "0");

    defines.add("_BOUNCE_NUM", std::to_string(mBounceNum));

    defines.add("_OUTPUT_DEPTH", mOutputDepth ? "1" : "0");
    defines.add("_OUTPUT_OUTGOING_RADIANCE", mOutputOutgoingRadiance ? "1" : "0");
    defines.add("_VSL_EVALUATION");
    defines.add("SCATTER_VISIBILITY, 1");

    defines.add("USE_RIS", mUseRIS ? "1" : "0");

    defines.add("_SHADING_MODE", std::to_string(mShadingMode));

    defines.add("_SHADING_NORMAL_CHECK", mShadingNormalCheck ? "1" : "0");
    defines.add("_SHADING_MATERIAL_CHECK", mShadingMaterialCheck ? "1" : "0");

    defines.add("_USE_TRADITIONAL_VXL", mUseTraditionalVXL ? "1" : "0");
    defines.add("_USE_RICH_VXL", mUseRichVXL ? "1" : "0");
    defines.add("_USE_VSL", mUseVSL ? "1" : "0");
    defines.add("_USE_MIXTURE_BSDF", mUseMixtureBSDF ? "1" : "0");

    defines.add("_MAX_BSDF_SEARCH_COUNT", std::to_string(mMaxBSDFSearchCount));

    if (mpEmissiveSampler)
    {
        defines.add(mpEmissiveSampler->getDefines());
    }

    mpShadingPass->getProgram()->addDefines(defines);
    mpShadingPass->setVars(nullptr); // Trigger recompile

    mpInitialSamplingPass->getProgram()->addDefines(defines);
    mpInitialSamplingPass->setVars(nullptr); // Trigger recompile

    mpTemporalResamplingPass->getProgram()->addDefines(defines);
    mpTemporalResamplingPass->setVars(nullptr); // Trigger recompile

    mpSpatialResamplingPass->getProgram()->addDefines(defines);
    mpSpatialResamplingPass->setVars(nullptr); // Trigger recompile

    mNeedClear = true;
    mNeedRecompile = false;
}

void VBNLShadingPass::recordUserInteraction()
{
    if (mRecordUserInteraction)
    {
        if (!mUserInteractionOutputFile.is_open())
        {
            std::time_t result = std::time(nullptr);
            char filename_char[100];
            // will be saved under "Mogwai/"
            std::strftime(filename_char, 100, "%Y-%m-%d-%H-%M-%S", std::localtime(&result));
            std::string filename(filename_char);
            mUserInteractionFileName = filename;
            filename += "_CamCapture.txt";
            mUserInteractionOutputFile.open(filename);
        }

        float3 target = mpScene->getCamera()->getTarget();
        float3 pos = mpScene->getCamera()->getPosition();
        float3 up = mpScene->getCamera()->getUpVector();

        // current time ?
        mUserInteractionOutputFile << mRecordedTotalFrameTime << std::endl;
        mUserInteractionOutputFile << pos.x << " " << pos.y << " " << pos.z << std::endl;
        mUserInteractionOutputFile << target.x << " " << target.y << " " << target.z << std::endl;
        mUserInteractionOutputFile << up.x << " " << up.y << " " << up.z << std::endl << std::endl;
        mRecordedTotalFrameTime += gpFramework->getFrameRate().getLastFrameTime();
    }
    else
    {
        if (mUserInteractionOutputFile.is_open())
        {
            mRecordedTotalFrameTime = 0.0;
            mUserInteractionOutputFile.close();
        }
    }
}

void VBNLShadingPass::replayUserInteraction()
{
    if (mReplayUserInteraction)
    {
        mRecordedTotalFrameTime += gpFramework->getFrameRate().getLastFrameTime();

        // find closest time
        uint32_t index = (uint32_t)(std::lower_bound(mCameraTimeSequence.begin(), mCameraTimeSequence.end(), mRecordedTotalFrameTime) - mCameraTimeSequence.begin());
        if (index >= mCameraTimeSequence.size())
            mReplayUserInteraction = false;
        else
        {
            float3 prevPosition = mpScene->getCamera()->getPosition();
            float3 prevTarget = mpScene->getCamera()->getTarget();
            float3 prevUp = mpScene->getCamera()->getUpVector();

            mpScene->getCamera()->setPosition(mCameraPositionSequence[index]);
            mpScene->getCamera()->setTarget(mCameraTargetSequence[index]);
            mpScene->getCamera()->setUpVector(mCameraUpSequence[index]);
        }
    }
}

