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
#include "VirtualLightGeneratePass.h"
#include "RenderPasses/GBufferPass.h"
#include <tbb/parallel_for.h>
#include "cyCore.h"
#include "cyPointCloud.h"

struct PackedBoundingBox
{
    float3 minPoint;
    float3 maxPoint;
    float Pad0;
    float Pad1;
};

namespace
{
    const char kDummy[] = "dummy";
    const char kDummyInput[] = "input";
    const char kDummyOutput[] = "output";
    const char kDesc[] = "Place initial virtual light samples";
    const char kTileSize[] = "TileSize";
    const char kTileSampleNum[] = "TileSampleNum";
    const char kBoundBoxRadius[] = "boundBoxRadius";

    const char kStartFromLights[] = "StartFromLights";
    const char kStartFromView[] = "StartFromView";
    const char kPhotonPathCount[] = "PhotonPathCount";

    const char kDirectLightingOnly[] = "DirectLightingOnly";

    const char kNeedRelaxation[] = "NeedRelaxation";

    const std::string kGBuffer = "GBuffer";
    const std::string kDicInitialVirtualLights = "initialVirtualLights";
    const std::string kDicCurVirtualLights = "curVirtualLights";
    const std::string kDicTileDim = "tileDim";
    const std::string kDicTileSize = "tileSize";
    const std::string kDicTileSampleNum = "tileSampleNum";
    const std::string kDicTileVirtualLightContainer = "tileVirtualLightContainer";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("VirtualLightGeneratePass", kDesc, VirtualLightGeneratePass::create);
}

VirtualLightGeneratePass::SharedPtr VirtualLightGeneratePass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new VirtualLightGeneratePass);
    for (const auto& [key, value] : dict)
    {
        if (key == kTileSize)
        {
            pPass->mTileSize = value;
        }
        else if (key == kTileSampleNum)
        {
            pPass->mTileSampleNum = value;
        }
        else if (key == kBoundBoxRadius)
        {
            pPass->mBoundBoxRadius = value;
        }
        else if (key == kStartFromLights)
        {
            pPass->mStartFromLights = value;
        }
        else if (key == kPhotonPathCount)
        {
            pPass->mPhotonPathCount = value;
        }
        else if (key == kDirectLightingOnly)
        {
            pPass->mDirectLightingOnly = value;
        }
        else if (key == kStartFromView)
        {
            pPass->mStartFromView = value;
        }
        else if (key == kNeedRelaxation)
        {
            pPass->mNeedRelaxation = value;
        }
    }

    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    Program::Desc desc;
    desc.addShaderLibrary("RenderPasses/VirtualLightGeneratePass/VirtualLightGenerate.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpComputePass = ComputePass::create(desc, Program::DefineList(), false);

    Program::Desc startFromLightsDesc;
    startFromLightsDesc.addShaderLibrary("RenderPasses/VirtualLightGeneratePass/VirtualLightGenerateFromLights.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpStartFromLightsComputePass = ComputePass::create(startFromLightsDesc, Program::DefineList(), false);
    
    return pPass;
}

std::string VirtualLightGeneratePass::getDesc()
{
    return kDesc;
}

Dictionary VirtualLightGeneratePass::getScriptingDictionary()
{
    Dictionary d;
    d[kTileSize] = mTileSize;
    d[kTileSampleNum] = mTileSampleNum;
    d[kBoundBoxRadius] = mBoundBoxRadius;
    d[kStartFromLights] = mStartFromLights;
    d[kPhotonPathCount] = mPhotonPathCount;
    d[kDirectLightingOnly] = mDirectLightingOnly;
    d[kStartFromView] = mStartFromView;
    d[kNeedRelaxation] = mNeedRelaxation;
    return d;
}

RenderPassReflection VirtualLightGeneratePass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "input");
    reflector.addOutput(kDummyOutput, "output");
    return reflector;
}

void VirtualLightGeneratePass::execute(RenderContext* pRenderContext, const RenderData& renderData)
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

    if (gpState->currentState == -1)
    {
        gpState->setState(0);
    }

    if (gpState->currentState != 0)
    {
        return;
    }

    mTimer.update();

    if (gpState->currentState != gpState->prevState)
    {
        uint capacity = 0;

        if (mStartFromView)
        {
            capacity += mTileSampleNum * ceil(gpFramework->getTargetFbo()->getWidth() * gpFramework->getTargetFbo()->getHeight() / (float)(mTileSize * mTileSize));
        }

        if (mStartFromLights)
        {
            capacity += mPhotonPathCount * 4u;
        }

        if (!mDirectLightingOnly)
        {
            capacity *= 2;
        }

        mpVirtualLightContainer = VirtualLightContainer::create(capacity, mBoundBoxRadius);
    }

    if (gpState->currentState != gpState->prevState)
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

    uint tileWidth = gpFramework->getTargetFbo()->getWidth() / mTileSize;
    uint tileHeight = gpFramework->getTargetFbo()->getHeight() / mTileSize;
    if (gpState->currentState != gpState->prevState)
    {
        if (gpFramework->getTargetFbo()->getWidth() % mTileSize != 0 || gpFramework->getTargetFbo()->getHeight() % mTileSize != 0)
        {
            logInfo("invalid size");
            debugBreak();
        }

        std::vector<uint> initData;
        if (!mStartFromView)
        {
            initData.resize(tileWidth * tileHeight * (mTileSampleNum + 1), 0u);
        }
        else
        {
            initData.resize(tileWidth * tileHeight * (mTileSampleNum + 1), 0xffffffffu);
        }
        mpTileVirtualLightContainer = Buffer::createStructured(sizeof(uint), tileWidth * tileHeight * (mTileSampleNum + 1), ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource, Buffer::CpuAccess::None, initData.data());
        mpTileVirtualLightContainer->setName("Tile Virtual Light Container");
    }

    // set virtual light container by global dictionary
    renderData.getDictionary()[kDicInitialVirtualLights] = mpVirtualLightContainer;
    // set cur virtual light container for later usage
    renderData.getDictionary()[kDicCurVirtualLights] = mpVirtualLightContainer;

    renderData.getDictionary()[kDicTileVirtualLightContainer] = mpTileVirtualLightContainer;
    renderData.getDictionary()[kDicTileDim] = uint2(tileWidth, tileHeight);
    renderData.getDictionary()[kDicTileSampleNum] = mTileSampleNum;
    renderData.getDictionary()[kDicTileSize] = mTileSize;

    if (mStartFromLights)
    {
        ShaderVar cb = mpStartFromLightsComputePass["CB"];
        
        cb["gPhotonPathCount"] = mPhotonPathCount;
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();

        mpEmissiveTriangleTable->setShaderData(cb["gEmissiveTriTable"]);
        mpVirtualLightContainer->setShaderData(cb["gVirtualLightContainer"]);

        mpScene->setRaytracingShaderData(pRenderContext, mpStartFromLightsComputePass->getRootVar());

        mpStartFromLightsComputePass->execute(pRenderContext, uint3(mPhotonPathCount, 1, 1));
    }

    if (mStartFromView)
    {
        ShaderVar cb = mpComputePass["CB"];
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
        cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        cb["gTileSize"] = mTileSize;
        cb["gTileSampleNum"] = mTileSampleNum;
        cb["gTileDims"] = uint2(tileWidth, tileHeight);
        cb["gDirectLightingOnly"] = mDirectLightingOnly;
        gBuffer->SetShaderData(cb["gGBuffer"]);

        mpVirtualLightContainer->setShaderData(cb["gVirtualLightContainer"]);
        mpComputePass["gTileVirtualLightContainer"] = mpTileVirtualLightContainer;
        mpScene->setRaytracingShaderData(pRenderContext, mpComputePass->getRootVar());

        mpComputePass->execute(pRenderContext, uint3(tileWidth, tileHeight, 1));
    }

    mpVirtualLightContainer->updateCounterToCPU(pRenderContext);

    if (mNeedRelaxation)
    {
        auto snappers = VirtualLightContainer::create(mPhotonPathCount * 4u, mBoundBoxRadius);

        ShaderVar cb = mpStartFromLightsComputePass["CB"];

        cb["gPhotonPathCount"] = mPhotonPathCount;
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();

        mpEmissiveTriangleTable->setShaderData(cb["gEmissiveTriTable"]);
        snappers->setShaderData(cb["gVirtualLightContainer"]);

        mpScene->setRaytracingShaderData(pRenderContext, mpStartFromLightsComputePass->getRootVar());

        mpStartFromLightsComputePass->execute(pRenderContext, uint3(mPhotonPathCount, 1, 1));

        snappers->updateCounterToCPU(pRenderContext);

        relaxation(pRenderContext, snappers);
    }
    mpVirtualLightContainer->buildAS(pRenderContext);

    mTimer.update();

    logInfo("actual initial virtual light or importon count: " + std::to_string(mpVirtualLightContainer->getCount()));
    logInfo("[===Time Stats===]Initial Virtual Light or Importon Generation Takes " + std::to_string(mTimer.delta()) + "seconds");
    gpState->time += mTimer.delta();
    gpState->setState(1);
}

void VirtualLightGeneratePass::renderUI(Gui::Widgets& widget)
{
    widget.checkbox("Start From Lights", mStartFromLights);
    widget.var("Photon Path Count", mPhotonPathCount, 1u, 1000000u);

    widget.checkbox("Start From View", mStartFromView);
    widget.var("Tile Size", mTileSize, 1u, 20u);
    widget.var("Tile Sample Num", mTileSampleNum, 1u, 256u);
    widget.checkbox("Direct Lighting Only", mDirectLightingOnly);

    widget.var("Bound Box Radius", mBoundBoxRadius, 0.0f, 1.0f);

    widget.checkbox("Need Relaxation", mNeedRelaxation);

    if (mpVirtualLightContainer)
    {
        mpVirtualLightContainer->renderUI(widget);
    }

    if (widget.button("Prepare Data"))
    {
        gpState->setState(0);
    }
}

void VirtualLightGeneratePass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
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

        mpStartFromLightsComputePass->getProgram()->addDefines(defines);
        mpStartFromLightsComputePass->setVars(nullptr); // Trigger recompile
    }
}

# define M_PI           3.14159265358979323846
# define NearestNeighborCount 7
void VirtualLightGeneratePass::relaxation(RenderContext* pRenderContext, VirtualLightContainer::SharedPtr snappers)
{
    logInfo("snapper count:" + std::to_string(snappers->getCount()));
    Buffer::SharedPtr positionReadBuffer = Buffer::create(mpVirtualLightContainer->getPositionBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Write);
    pRenderContext->copyBufferRegion(positionReadBuffer.get(), 0, mpVirtualLightContainer->getPositionBuffer().get(), 0, mpVirtualLightContainer->getPositionBuffer()->getSize());

    Buffer::SharedPtr snappersPositionReadBuffer = Buffer::create(snappers->getPositionBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
    pRenderContext->copyBufferRegion(snappersPositionReadBuffer.get(), 0, snappers->getPositionBuffer().get(), 0, snappers->getPositionBuffer()->getSize());

    pRenderContext->flush(true);

    float3* mappedPositions = (float3*)positionReadBuffer->map(Buffer::MapType::Write);
    float3* snapperPositions = (float3*)snappersPositionReadBuffer->map(Buffer::MapType::Read);

    cy::PointCloud<float3, float, 3> PhotonCloud;
    PhotonCloud.Build(snappers->getCount(), snapperPositions);

    uint mIterationCount = 20;
    cy::PointCloud<float3, float, 3> candidateCloud;
    for (uint i = 0; i < mIterationCount; i++)
    {
        // Rebuild The Tree
        candidateCloud.Build(mpVirtualLightContainer->getCount(), mappedPositions);

        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, mpVirtualLightContainer->getCount()),
            [&](const tbb::blocked_range<size_t>& r) {
            for (size_t j = r.begin(); j < r.end(); ++j)
            {
                float3 pos = mappedPositions[j];
                cy::PointCloud<float3, float, 3>::PointInfo pointInfos[NearestNeighborCount + 1];
                bool res = candidateCloud.GetPoints(pos, NearestNeighborCount + 1, pointInfos);
                assert(res);

                float radiusSquared = 0.0f;
                for (uint k = 0; k < NearestNeighborCount + 1; k++)
                {
                    float disSquared = pointInfos[k].distanceSquared;
                    if (disSquared > radiusSquared)
                    {
                        radiusSquared = disSquared;
                    }
                }

                float3 deltaX = float3(0.0f, 0.0f, 0.0f);
                for (uint k = 0; k < NearestNeighborCount + 1; k++)
                {
                    float disSquared = pointInfos[k].distanceSquared;
                    if (disSquared < radiusSquared)
                    {
                        float3 item = (1.0f / float(NearestNeighborCount)) * (pos - pointInfos[k].pos) * (sqrt(radiusSquared) / (sqrt(disSquared) + 1e-6f) - 1.0f);
                        deltaX += item;
                    }
                }

                pos += 2.5f * deltaX;
                mappedPositions[j] = pos;
            }
        }
        );

        //snap to nearest photons
        if (i == (mIterationCount - 1))
        {
            for (uint j = 0; j < mpVirtualLightContainer->getCount(); j++)
            {
                float3 pos = mappedPositions[j];
                cy::PointCloud<float3, float, 3>::PointInfo pointInfos[1];
                bool res = PhotonCloud.GetPoints(pos, 1, pointInfos);
                assert(res);
                mappedPositions[j] = pointInfos[0].pos;
            }
        }
        else
        {
            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, mpVirtualLightContainer->getCount()),
                [&mappedPositions, &PhotonCloud](const tbb::blocked_range<size_t>& r)
            {
                for (size_t j = r.begin(); j < r.end(); ++j)
                {
                    float3 pos = mappedPositions[j];
                    cy::PointCloud<float3, float, 3>::PointInfo pointInfos[1];
                    bool res = PhotonCloud.GetPoints(pos, 1, pointInfos);
                    assert(res);
                    mappedPositions[j] = pointInfos[0].pos;
                }
            });
        }
    }

    mpVirtualLightContainer->getPositionBuffer()->setBlob(mappedPositions, 0, mpVirtualLightContainer->getPositionBuffer()->getSize());
    positionReadBuffer->unmap();
    snappersPositionReadBuffer->unmap();
}
