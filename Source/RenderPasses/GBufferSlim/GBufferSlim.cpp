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
#include "GBufferSlim.h"


namespace
{
    const char kDummy[] = "dummy";
    const char kDummyInput[] = "input";
    const char kDummyOutput[] = "output";
    const char kNormalRoughnessMaterialId[] = "normalRoughnessMaterialId";
    const char kMotionVec[] = "motionVec";
    const char kMotionVecW[] = "motionVecW";
    const char kDesc[] = "Slim GBuffer";
    const std::string kDicInitialVirtualLights = "initialVirtualLights";
    const std::string kDicSampleEliminatedVirtualLights = "sampleEliminatedVirtualLights";
    const std::string kDicCurVirtualLights = "curVirtualLights";
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
    pybind11::class_<GBufferSlim, RenderPass, GBufferSlim::SharedPtr> pass(m, "GBufferSlim");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("GBufferSlim", kDesc, GBufferSlim::create);
    ScriptBindings::registerBinding(regPythonApi);
}

GBufferSlim::SharedPtr GBufferSlim::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new GBufferSlim);
    for (const auto& [key, value] : dict)
    {
    }
    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);
    return pPass;
}

std::string GBufferSlim::getDesc()
{
    return kDesc;
}

Dictionary GBufferSlim::getScriptingDictionary()
{
    Dictionary d;
    return d;
}

RenderPassReflection GBufferSlim::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addOutput(kDummyOutput, "useless dummy Output");
    reflector.addOutput(kMotionVec, "motionVec").bindFlags(ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource).format(ResourceFormat::RG32Float);
    reflector.addOutput(kMotionVecW, "motionVecW").bindFlags(ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource).format(ResourceFormat::RGBA16Float);
    reflector.addOutput(kNormalRoughnessMaterialId, "normalRoughnessMaterialId").bindFlags(ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource).format(ResourceFormat::RGB10A2Unorm);
    return reflector;
}

void GBufferSlim::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    // auto& pTexture = renderData["src"]->asTexture();
    if (mpScene == nullptr)
    {
        return;
    }

    auto motionVecTex = renderData[kMotionVec]->asTexture();
    auto motionVecWTex = renderData[kMotionVecW]->asTexture();
    auto normalRoughnessMaterialIdTex = renderData[kNormalRoughnessMaterialId]->asTexture();

    if (mpGBufferPass == nullptr)
    {
        mpGBufferPass = GBufferPass::Create(mpScene);
        mpGBufferPass->SetMotionTexture(motionVecTex);
        mpGBufferPass->SetMotionWTexture(motionVecWTex);
        mpGBufferPass->SetNormalRoughnessMaterialIdTexture(normalRoughnessMaterialIdTex);

        mpGBufferPassAnother = GBufferPass::Create(mpScene);
        mpGBufferPassAnother->SetMotionTexture(motionVecTex);
        mpGBufferPassAnother->SetMotionWTexture(motionVecWTex);
        mpGBufferPassAnother->SetNormalRoughnessMaterialIdTexture(normalRoughnessMaterialIdTex);
    }

    if (mpGBufferPass->GetMotionTexture()->getWidth() != gpFramework->getTargetFbo()->getWidth() || mpGBufferPass->GetMotionTexture()->getHeight() != gpFramework->getTargetFbo()->getHeight())
    {
        mpGBufferPass->Resize(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        mpGBufferPassAnother->Resize(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    if (gpFramework->getGlobalClock().getFrame() % 2 == 0)
    {
        mpGBufferPass->Execute(pRenderContext);
        renderData.getDictionary()[kGBuffer] = mpGBufferPass;
        renderData.getDictionary()[kGBufferPrevFrame] = mpGBufferPassAnother;
    }
    else
    {
        mpGBufferPassAnother->Execute(pRenderContext);
        renderData.getDictionary()[kGBuffer] = mpGBufferPassAnother;
        renderData.getDictionary()[kGBufferPrevFrame] = mpGBufferPass;
    }
}

void GBufferSlim::renderUI(Gui::Widgets& widget)
{
}

void GBufferSlim::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpScene)
    {
        Shader::DefineList defines = mpScene->getSceneDefines();
        defines.add(mpSampleGenerator->getDefines());
        defines.add("_MS_DISABLE_ALPHA_TEST");
        defines.add("_DEFAULT_ALPHA_TEST");

        mpSamplePattern = HaltonSamplePattern::create(0U);
        mpScene->getCamera()->setPatternGenerator(mpSamplePattern, float2(1.0f / gpFramework->getTargetFbo()->getWidth(), 1.0f / gpFramework->getTargetFbo()->getHeight()));
    }
}


