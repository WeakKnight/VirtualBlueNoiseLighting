#include "SimplePathTracer.h"
#include "RenderPasses/GBufferPass.h"
#include "Utils/Sampling/AliasTable.h"
#include "Experimental/Scene/Lights/LightBVHSampler.h"

namespace
{
    const char kDummy[] = "dummy";
    const char kDummyInput[] = "input";
    const char kDummyOutput[] = "output";
    const char kDesc[] = "Insert pass description here";
    const char kSampleCount[] = "sampleCount";
    const char kMaxPathIntensity[] = "MaxPathIntensity";
    const char kBounceNum[] = "BounceNum";

    const std::string kDicInitialVirtualLights = "initialVirtualLights";
    const std::string kDicSampleEliminatedVirtualLights = "sampleEliminatedVirtualLights";
    const std::string kDicCurVirtualLights = "curVirtualLights";
    const std::string kGBuffer = "GBuffer";
    const std::string kReservoirBuffer = "reservoirBuffer";
    const std::string kEmissiveTriangleFluxTable = "emissiveTriangleFluxTable";
}

extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

static void regPythonApi(pybind11::module& m)
{
    pybind11::class_<SimplePathTracer, RenderPass, SimplePathTracer::SharedPtr> pass(m, "SimplePathTracer");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary & lib)
{
    lib.registerClass("SimplePathTracer", kDesc, SimplePathTracer::create);
    ScriptBindings::registerBinding(regPythonApi);
}

SimplePathTracer::SharedPtr SimplePathTracer::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new SimplePathTracer);
    for (const auto& [key, value] : dict)
    {
        if (key == kMaxPathIntensity)
        {
            pPass->mMaxPathIntensity = value;
        }
        else if (key == kBounceNum)
        {
            pPass->mBounceNum = value;
        }
    }
    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    Program::Desc passDesc;
    passDesc.addShaderLibrary("RenderPasses/SimplePathTracer/SimplePathTracer.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpComputePass = ComputePass::create(passDesc, Program::DefineList(), false);

    return pPass;
}

std::string SimplePathTracer::getDesc()
{
    return kDesc;
}

Dictionary SimplePathTracer::getScriptingDictionary()
{
    Dictionary d;
    d[kMaxPathIntensity] = mMaxPathIntensity;
    d[kBounceNum] = mBounceNum;
    return d;
}

RenderPassReflection SimplePathTracer::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "useless dummy Input");
    reflector.addOutput(kDummyOutput, "shading Output").bindFlags(ResourceBindFlags::RenderTarget | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource).format(ResourceFormat::RGBA32Float);
    return reflector;
}

void SimplePathTracer::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
    {
        return;
    }

    GBufferPass::SharedPtr gBuffer = renderData.getDictionary()[kGBuffer];
    if (gBuffer == nullptr)
    {
        return;
    }

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

    auto cb = mpComputePass["CB"];
    cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
    cb["gMaxPathIntensity"] = mMaxPathIntensity;

    gBuffer->SetShaderData(cb["gGBuffer"]);
    if (mpEmissiveSampler)
    {
        mpEmissiveSampler->setShaderData(cb["gEmissiveLightSampler"]);
    }
    if (mpEnvMapSampler)
    {
        mpEnvMapSampler->setShaderData(cb["gEnvMapSampler"]);
    }

    mpScene->setRaytracingShaderData(pRenderContext, mpComputePass->getRootVar());
    Texture::SharedPtr shadingOutput = renderData[kDummyOutput]->asTexture();
    if (shadingOutput == nullptr)
    {
        logError("No Output Texture");
    }
    mpComputePass["gOutput"] = shadingOutput;
    mpComputePass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
}

void SimplePathTracer::renderUI(Gui::Widgets& widget)
{
    mNeedRecompile |= widget.var("Bounce Num", mBounceNum);
    mNeedClear |= widget.var("Max Path Intensity", mMaxPathIntensity);
}

void SimplePathTracer::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpScene)
    {
        recompile();
    }
}

void SimplePathTracer::onHotReload(HotReloadFlags reloaded)
{
    recompile();
}

bool SimplePathTracer::prepareLights(RenderContext* pRenderContext)
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

void SimplePathTracer::recompile()
{
    Shader::DefineList defines = mpScene->getSceneDefines();
    defines.add(mpSampleGenerator->getDefines());
    defines.add("_MS_DISABLE_ALPHA_TEST");
    defines.add("_DEFAULT_ALPHA_TEST");
    defines.add("USE_ENV_LIGHT", mpScene->useEnvLight() ? "1" : "0");
    defines.add("USE_ANALYTIC_LIGHTS", mpScene->useAnalyticLights() ? "1" : "0");
    defines.add("USE_EMISSIVE_LIGHTS", mpScene->useEmissiveLights() ? "1" : "0");
    defines.add("_BOUNCE_NUM", std::to_string(mBounceNum));
    if (mpEmissiveSampler)
    {
        defines.add(mpEmissiveSampler->getDefines());
    }

    mpComputePass->getProgram()->addDefines(defines);
    mpComputePass->setVars(nullptr);

    mNeedRecompile = false;
    mNeedClear = true;
}


