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
#include "PhotonGenerationPass.h"
#include <tbb/parallel_for.h>
#include "cyCore.h"
#include "cyPointCloud.h"

namespace
{
    const char kDummy[] = "dummy";
    const char kDummyInput[] = "input";
    const char kDummyOutput[] = "output";
    const char kDesc[] = "Photon Generation";

    const char kState[] = "state";
    const char kStateStep[] = "stateStep";

    const std::string kDicPhotons = "photons";
    const char kPhotonPathCount[] = "PhotonPathCount";
    const char kBounceNum[] = "BounceNum";

    const char kComputeVSLRadius[] = "ComputeVSLRadius";
    const char kKNNRadiusNeighbour[] = "KNNRadiusNeighbour";
    const char kRadius[] = "Radius";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

static void regPythonApi(pybind11::module& m)
{
    pybind11::class_<PhotonGenerationPass, RenderPass, PhotonGenerationPass::SharedPtr> pass(m, "PhotonGenerationPass");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("PhotonGenerationPass", kDesc, PhotonGenerationPass::create);
    ScriptBindings::registerBinding(regPythonApi);
}

PhotonGenerationPass::SharedPtr PhotonGenerationPass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new PhotonGenerationPass);
    for (const auto& [key, value] : dict)
    {
        if (key == kPhotonPathCount)
        {
            pPass->mPhotonPathCount = value;
        }
        else if (key == kState)
        {
            pPass->mState = value;
        }
        else if (key == kStateStep)
        {
            pPass->mStateStep = value;
        }
        else if (key == kBounceNum)
        {
            pPass->mBounceNum = value;
        }
        else if (key == kComputeVSLRadius)
        {
            pPass->mComputeVSLRadius = value;
        }
        else if (key == kRadius)
        {
            pPass->mRadius = value;
        }
        else if (key == kKNNRadiusNeighbour)
        {
            pPass->mKNNRadiusNeighbour = value;
        }
    }
    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    Program::Desc radiusWritePassDesc;
    radiusWritePassDesc.addShaderLibrary("RenderPasses/PhotonGenerationPass/RadiusWritePass.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpRadiusWritePass = ComputePass::create(radiusWritePassDesc, Program::DefineList(), false);

    Program::Desc desc;
    desc.addShaderLibrary("RenderPasses/PhotonGenerationPass/PhotonGenerationPass.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpGenerationPass = ComputePass::create(desc, Program::DefineList(), false);

    return pPass;
}

std::string PhotonGenerationPass::getDesc()
{
    return kDesc;
}

Dictionary PhotonGenerationPass::getScriptingDictionary()
{
    Dictionary d;
    d[kPhotonPathCount] = mPhotonPathCount;
    d[kState] = mState;
    d[kStateStep] = mStateStep;
    d[kBounceNum] = mBounceNum;
    d[kComputeVSLRadius] = mComputeVSLRadius;
    d[kRadius] = mRadius;
    d[kKNNRadiusNeighbour] = mKNNRadiusNeighbour;
    return d;
}

RenderPassReflection PhotonGenerationPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "input");
    reflector.addOutput(kDummyOutput, "output");
    return reflector;
}

void PhotonGenerationPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    // auto& pTexture = renderData["src"]->asTexture();
    if (mpScene == nullptr)
    {
        return;
    }

    int currentState = gpState->currentState;
    if (currentState != mState)
    {
        return;
    }

    mTimer.update();

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
    if (mpPhotons == nullptr)
    {
        mpPhotons = VirtualLightContainer::create(20u * mPhotonPathCount, mRadius, true);

        ShaderVar cb = mpGenerationPass["CB"];

        cb["gPhotonPathCount"] = mPhotonPathCount;
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();

        mpEmissiveTriangleTable->setShaderData(cb["gEmissiveTriTable"]);
        mpPhotons->setShaderData(cb["gPhotons"]);

        mpScene->setRaytracingShaderData(pRenderContext, mpGenerationPass->getRootVar());

        mpGenerationPass->execute(pRenderContext, uint3(mPhotonPathCount, 1, 1));
    }
    mpPhotons->updateCounterToCPU(pRenderContext);

    if (mComputeVSLRadius)
    {
        std::vector<float> radiuses;
        uint lightCount = mpPhotons->getCount();
        radiuses.resize(lightCount);

        Buffer::SharedPtr positionBuffer = mpPhotons->getPositionBuffer();
        Buffer::SharedPtr positionReadBuffer = Buffer::create(positionBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read, nullptr);
        pRenderContext->copyBufferRegion(positionReadBuffer.get(), 0, positionBuffer.get(), 0, positionBuffer->getSize());
        pRenderContext->flush(true);

        float3* positions = (float3*)positionReadBuffer->map(Buffer::MapType::Read);

        cy::PointCloud<float3, float, 3> positionCloud;
        positionCloud.Build(lightCount, positions);
        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, lightCount),
            [&](const tbb::blocked_range<size_t>& r)
        {
            for (size_t i = r.begin(); i < r.end(); i++)
            {
                float3 position = positions[i];
                std::vector<cy::PointCloud<float3, float, 3>::PointInfo> pointInfos;
                pointInfos.resize(mKNNRadiusNeighbour);
                int resCount = positionCloud.GetPoints(position, mKNNRadiusNeighbour, pointInfos.data());
                if (resCount <= 0.0)
                {
                    std::cout << "KNNRadiusPass: No Neighbour Found!!!" << std::endl;
                }
                float longest = 0.0f;
                for (uint j = 0; j < resCount; j++)
                {
                    float dis = sqrt(pointInfos[j].distanceSquared);
                    if (dis > longest)
                    {
                        longest = dis;
                    }
                }
                assert(longest >= 0.0f);
                if (longest < 0.0f)
                {
                    logError("We should not have neggative radius");
                }

                longest = std::max(0.000001f, longest);
                float r = sqrt(longest);

                radiuses[i] = r;
            }
        }
        );

        Buffer::SharedPtr dMaxBuffer = Buffer::createStructured(sizeof(float), radiuses.size(), ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource, Buffer::CpuAccess::None, radiuses.data());
        dMaxBuffer->setName("VPLReSTIR: dMax Buffer");

        auto cb = mpRadiusWritePass["CB"];
        mpPhotons->setShaderData(cb["gVirtualLights"]);
        mpRadiusWritePass["gDMaxs"] = dMaxBuffer;
        mpRadiusWritePass->execute(pRenderContext, lightCount, 1, 1);
        pRenderContext->flush(true);
        positionReadBuffer->unmap();
    }

    mpPhotons->buildAS(pRenderContext);

    logInfo("actual photon count: " + std::to_string(mpPhotons->getCount()));
    mTimer.update();
    logInfo("[===Time Stats===]Photonmap Generation Takes " + std::to_string(mTimer.delta()) + "seconds");
    gpState->time += mTimer.delta();

    renderData.getDictionary()[kDicPhotons] = mpPhotons;

    gpState->setState(mState + mStateStep);
}

void PhotonGenerationPass::renderUI(Gui::Widgets& widget)
{
    widget.var("Photon Path Count", mPhotonPathCount, 1u, 80000000u);
    widget.var("Bounce Num", mBounceNum, 1u, 256u);
    widget.var("State", mState, -1, 100);
    widget.var("StateStep", mStateStep, 0, 100);
    widget.checkbox("Compute VSL Radius", mComputeVSLRadius);
    if (mComputeVSLRadius)
    {
        widget.var("KNN Search Neighbor", mKNNRadiusNeighbour, 1u, 20u);
    }
    widget.var("Radius", mRadius,0.0f, 10.0f);

    if (mpPhotons)
    {
        if (auto virtualLightGroup = widget.group("Virtual Light Stats"))
        {
            mpPhotons->renderUI(virtualLightGroup, 0u);
        }

        widget.text("Photon Light Count:" + std::to_string(mpPhotons->getCount()));
    }
}

void PhotonGenerationPass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpScene)
    {
        Shader::DefineList defines = mpScene->getSceneDefines();
        defines.add(mpSampleGenerator->getDefines());
        defines.add("_MS_DISABLE_ALPHA_TEST");
        defines.add("_DEFAULT_ALPHA_TEST");
        defines.add("_BOUNCE_NUM", std::to_string(mBounceNum));

        mpGenerationPass->getProgram()->addDefines(defines);
        mpGenerationPass->setVars(nullptr); // Trigger recompile

        mpRadiusWritePass->getProgram()->addDefines(defines);
        mpRadiusWritePass->setVars(nullptr); // Trigger recompile
    }
}


