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
#include "ResultAccumulatePass.h"


namespace
{
    const char kDummy[] = "dummy";
    const char kDummyInput[] = "input";
    const char kDummyOutput[] = "output";
    const char kDesc[] = "Insert pass description here";
    const std::string kDicInitialVirtualLights = "initialVirtualLights";
    const std::string kDicSampleEliminatedVirtualLights = "sampleEliminatedVirtualLights";
    const std::string kDicCurVirtualLights = "curVirtualLights";
    const std::string kGBuffer = "GBuffer";
    const std::string kReservoirBuffer = "reservoirBuffer";

    const char kInputChannel[] = "input";
    const char kOutputChannel[] = "output";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

static void regPythonApi(pybind11::module& m)
{
    pybind11::class_<ResultAccumulatePass, RenderPass, ResultAccumulatePass::SharedPtr> pass(m, "ResultAccumulatePass");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("ResultAccumulatePass", kDesc, ResultAccumulatePass::create);
    ScriptBindings::registerBinding(regPythonApi);
}

ResultAccumulatePass::SharedPtr ResultAccumulatePass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new ResultAccumulatePass);
    for (const auto& [key, value] : dict)
    {
    }
    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    Program::Desc passDesc;
    passDesc.addShaderLibrary("RenderPasses/ResultAccumulatePass/ResultAccumulatePass.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpComputePass = ComputePass::create(passDesc, Program::DefineList(), false);

    return pPass;
}

std::string ResultAccumulatePass::getDesc()
{
    return kDesc;
}

Dictionary ResultAccumulatePass::getScriptingDictionary()
{
    Dictionary d;
    return d;
}

RenderPassReflection ResultAccumulatePass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput(kInputChannel, "Input data to be temporally accumulated").bindFlags(ResourceBindFlags::ShaderResource);
    reflector.addOutput(kOutputChannel, "Output data that is temporally accumulated").bindFlags(ResourceBindFlags::RenderTarget | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource).format(ResourceFormat::RGBA32Float);
    return reflector;
}

void ResultAccumulatePass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    // auto& pTexture = renderData["src"]->asTexture();
    if (mpScene == nullptr)
    {
        return;
    }

    Texture::SharedPtr pSrc = renderData[kInputChannel]->asTexture();
    Texture::SharedPtr pDst = renderData[kOutputChannel]->asTexture();
    const uint2 resolution = uint2(pSrc->getWidth(), pSrc->getHeight());

    if (mpLastFrameSum == nullptr)
    {
        mpLastFrameSum = Texture::create2D(resolution.x, resolution.y, ResourceFormat::RGBA32Float, 1, 1, nullptr, Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess);
    }

    if (mAccumCount == 0)
    {
        pRenderContext->clearUAV(mpLastFrameSum->getUAV().get(), float4(0.f));
    }
        
    if (mNeedAccum)
    {
        mpComputePass["PerFrameCB"]["gResolution"] = resolution;
        mpComputePass["PerFrameCB"]["gAccumCount"] = mAccumCount++;
        mpComputePass["gCurFrame"] = pSrc;
        mpComputePass["gOutputFrame"] = pDst;

        // Bind accumulation buffers. Some of these may be nullptr's.
        mpComputePass["gLastFrameSum"] = mpLastFrameSum;

        mpComputePass->execute(pRenderContext, resolution.x, resolution.y, 1);
        mNeedAccum = false;
    }
}

void ResultAccumulatePass::renderUI(Gui::Widgets& widget)
{
    widget.text("Accum Count: " + std::to_string(mAccumCount));
    if (widget.button("Accum"))
    {
        mNeedAccum = true;
    }
}

void ResultAccumulatePass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpScene)
    {
        Shader::DefineList defines = mpScene->getSceneDefines();
        defines.add(mpSampleGenerator->getDefines());
        defines.add("_MS_DISABLE_ALPHA_TEST");
        defines.add("_DEFAULT_ALPHA_TEST");

        mpComputePass->getProgram()->addDefines(defines);
        mpComputePass->setVars(nullptr); // Trigger recompile
    }
}


