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
#include "PrepareLights.h"


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
    const std::string kEmissiveTriangleFluxTable = "emissiveTriangleFluxTable";
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

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

static void regPythonApi(pybind11::module& m)
{
    pybind11::class_<PrepareLights, RenderPass, PrepareLights::SharedPtr> pass(m, "PrepareLights");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("PrepareLights", kDesc, PrepareLights::create);
    ScriptBindings::registerBinding(regPythonApi);
}

PrepareLights::SharedPtr PrepareLights::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new PrepareLights);
    for (const auto& [key, value] : dict)
    {
    }
    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);
    Program::Desc passDesc;
    passDesc.addShaderLibrary("RenderPasses/PrepareLights/PrepareLights.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpComputePass = ComputePass::create(passDesc, Program::DefineList(), false);
    return pPass;
}

std::string PrepareLights::getDesc()
{
    return kDesc;
}

Dictionary PrepareLights::getScriptingDictionary()
{
    Dictionary d;
    return d;
}

RenderPassReflection PrepareLights::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "useless dummy Input");
    reflector.addOutput(kDummyOutput, "useless dummy Output");
    return reflector;
}

void PrepareLights::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    // auto& pTexture = renderData["src"]->asTexture();
    if (mpScene == nullptr)
    {
        return;
    }

    if (mpEmissiveTriangleTable == nullptr)
    {
        auto lightCollection = mpScene->getLightCollection(pRenderContext);
        lightCollection->prepareSyncCPUData(pRenderContext);
        auto lightData = lightCollection->getMeshLightTriangles();
        std::vector<float> fluxList;
        fluxList.resize(lightData.size(), 0.0f);
        for (int i = 0; i < lightData.size(); i++)
        {
            fluxList[i] = lightData[i].flux;
        }
        std::mt19937 rng;
        mpEmissiveTriangleTable = AliasTable::create(fluxList, rng);
    }

    renderData.getDictionary()[kEmissiveTriangleFluxTable] = mpEmissiveTriangleTable;
}

void PrepareLights::renderUI(Gui::Widgets& widget)
{
}

void PrepareLights::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
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


