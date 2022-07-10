#pragma once
#include "Falcor.h"
#include "FalcorExperimental.h"
#include "Utils/Sampling/SampleGenerator.h"

using namespace Falcor;

class SimplePathTracer : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<SimplePathTracer>;

    static SharedPtr create(RenderContext* pRenderContext = nullptr, const Dictionary& dict = {});

    virtual std::string getDesc() override;
    virtual Dictionary getScriptingDictionary() override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pContext, const CompileData& compileData) override {}
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { return false; }
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override { return false; }
    void onHotReload(HotReloadFlags reloaded) override;

private:
    SimplePathTracer() = default;

    bool prepareLights(RenderContext* pRenderContext);
    void recompile();

    EmissiveLightSampler::SharedPtr mpEmissiveSampler;
    EnvMapSampler::SharedPtr mpEnvMapSampler;
    bool mNeedRecompile = true;
    bool mNeedClear = false;

    Scene::SharedPtr mpScene;
    ComputePass::SharedPtr  mpComputePass;
    SampleGenerator::SharedPtr mpSampleGenerator;

    float mMaxPathIntensity = 0.0f;
    uint32_t mBounceNum = 256u;
};
