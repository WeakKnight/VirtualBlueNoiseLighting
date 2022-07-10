#pragma once
#include "Falcor.h"
#include "FalcorExperimental.h"
#include "Utils/Sampling/SampleGenerator.h"
#include "Experimental/Scene/Lights/LightBVHSampler.h"
#include "Utils/Timing/CpuTimer.h"

using namespace Falcor;

class BSDFReSTIRPass : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<BSDFReSTIRPass>;

    static SharedPtr create(RenderContext* pRenderContext = nullptr, const Dictionary& dict = {});

    virtual std::string getDesc() override;
    virtual Dictionary getScriptingDictionary() override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pContext, const CompileData& compileData) override {}
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { return mpPixelDebug->onMouseEvent(mouseEvent); }
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override;
    virtual void onHotReload(HotReloadFlags reloaded);
private:
    BSDFReSTIRPass() = default;

    bool prepareLights(RenderContext* pRenderContext);
    void recompile();

    void recordUserInteraction();
    void replayUserInteraction();

    EmissiveLightSampler::SharedPtr mpEmissiveSampler;
    EnvMapSampler::SharedPtr mpEnvMapSampler;
    bool mNeedRecompile = true;
    bool mNeedClear = false;

    Scene::SharedPtr mpScene;

    SampleGenerator::SharedPtr mpSampleGenerator;
    PixelDebug::SharedPtr               mpPixelDebug;                    ///< Utility class for pixel debugging (print in shaders).

    ComputePass::SharedPtr mpShadingPass;

    LightBVHSampler::SharedPtr mpVirtualLightEmissiveSampler;
    LightBVHSampler::Options mVirtualLightEmissiveSamplerOptions;
    bool mVirtualLightEmissiveSamplerOptionsChanged = false;

    float mShortDistance = 0.05f;
    float mShortDistanceRange = 0.01f;

    float mShortDistanceNumVSLs = 40;
    float mLongDistanceNumVSLs = 80;

    bool mUseNewThresholds = true;

    float mMaxPathIntensity = 0.0f;
    uint32_t mBounceNum = 256u;
    uint32_t mNEESamples = 1u;
    bool mUseRIS = false;
    bool mUseVirtualLightBVH = false;

    bool mUseVirtualLight = true;
    bool mUseVirtualLightNEE = true;
    bool mEnablePreciseVirtualLightNEE = false;
    bool mUseVirtualLightMIS = true;
    float mRoughnessThreshold = 0.12f;

    bool mOutputDepth = false;
    bool mOutputOutgoingRadiance = false;

    bool mShadingNormalCheck = true;
    bool mShadingMaterialCheck = false;

    bool mUseTraditionalVXL = false;
    bool mUseVSL = true;
    bool mUseRichVXL = true;

    bool mUseMixtureBSDF = true;

    Texture::SharedPtr mpShadingOutput;

    //// user interaction
    std::vector<double> mCameraTimeSequence;
    std::vector<float3> mCameraTargetSequence;
    std::vector<float3> mCameraUpSequence;
    std::vector<float3> mCameraPositionSequence;
    bool mReplayUserInteraction = false;
    bool                            mRecordUserInteraction = false;
    double                          mRecordedTotalFrameTime = 0.0;
    std::string                     mUserInteractionFileName = "";
    std::ofstream                   mUserInteractionOutputFile;

    //// capture after certain render time
    bool mCaptureAfterCertainTime = false;
    CpuTimer mTimer;
    float mTimeForCapture = 1000.f;

    uint mShadingMode = 0;

    //// ReSTIR related
    ComputePass::SharedPtr mpInitialSamplingPass;
    ComputePass::SharedPtr mpSpatialResamplingPass;
    ComputePass::SharedPtr mpTemporalResamplingPass;
    ComputePass::SharedPtr mpReSTIRShadingPass;

    Buffer::SharedPtr mpNeighborOffsetBuffer;

    bool mEnableSpatialResampling = false;
    bool mMISWithPowerSampling = false;
    bool mEnableTemporalResampling = false;
    uint mLastFrameOutputReservoir = 0;
    uint mCurrentFrameOutputReservoir = 0;
    uint mNumBSDFSamples = 1;
    uint mNumPowerSamples = 1;
    bool mFinalMISWithPowerSampling = false;
    uint mFinalNumBSDFSamples = 1;
    uint mFinalNumPowerSamples = 1;
    Buffer::SharedPtr mpReservoirBuffer;

    VirtualLightContainer::SharedPtr mpVirtualLights;
    uint mMegaTexItemSize = 0u;

    float mVSLRadiusScaler = 1.0f;
    uint mMaxBSDFSearchCount = 16;

    uint mCurrentVPLIndex = 0;
};
