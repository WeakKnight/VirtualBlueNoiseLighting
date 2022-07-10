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
#include "RelaxationPass.h"

#include <tbb/parallel_for.h>
#include "cyCore.h"
#include "cyPointCloud.h"
#include "Utils/Math/PackedFormats.h"

# define M_PI           3.14159265358979323846
# define NearestNeighborCount 7

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
    const char kDesc[] = "Relaxation Pass";
    const char kTargetCount[] = "targetCount";
    const char kRadius[] = "radius";
    const char kRadiusScalerForASBuilding[] = "radiusScalerForASBuilding";
    const char kUseDMaxForASBuilding[] = "useDMaxForASBuilding";

    const char kTextureItemSize[] = "Texture Item Size";
    const char kMegaTextureCapacity[] = "MegaTexture Capacity";
    const char kTextureItemSizeLQ[] = "Texture Item Size LQ";
    const char kMegaTextureCapacityLQ[] = "MegaTexture Capacity LQ";

    const char kRelaxationFactor[] = "Relaxation Factor";

    const char kKNNRadiusScaler[] = "KNNRadiusScaler";
    const char kStandardEnrich[] = "standardEnrich";

    const std::string kDicInitialVirtualLights = "initialVirtualLights";
    const std::string kDicSampleEliminatedVirtualLights = "sampleEliminatedVirtualLights";
    const std::string kDicCurVirtualLights = "curVirtualLights";
    const std::string kGBuffer = "GBuffer";
    const std::string kReservoirBuffer = "reservoirBuffer";

    const std::string kDicPhotons = "photons";
    const std::string kDicSpecularRadianceContainer = "specularRadianceContainer";
    const std::string kDicFluxBuffer = "fluxBuffer";
    const std::string kDicRadianceReady = "radianceReady";
    const std::string kDicFluxTable = "fluxTable";

    const std::string kDicTileDim = "tileDim";
    const std::string kDicTileSize = "tileSize";
    const std::string kDicTileSampleNum = "tileSampleNum";
    const std::string kDicTileVirtualLightWeights = "tileVirtualLightWeights";

    const char kMinPrs[] = "MinPrs";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

static void regPythonApi(pybind11::module& m)
{
    pybind11::class_<RelaxationPass, RenderPass, RelaxationPass::SharedPtr> pass(m, "RelaxationPass");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary & lib)
{
    lib.registerClass("RelaxationPass", kDesc, RelaxationPass::create);
    ScriptBindings::registerBinding(regPythonApi);
}

RelaxationPass::SharedPtr RelaxationPass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new RelaxationPass);
    for (const auto& [key, value] : dict)
    {
        if (key == kTargetCount)
        {
            pPass->mTargetCount = value;
        }
        else if (key == kRadius)
        {
            pPass->mRadius = value;
        }
        else if (key == kRadiusScalerForASBuilding)
        {
            pPass->mRadiusScalerForASBuilding = value;
        }
        else if (key == kUseDMaxForASBuilding)
        {
            pPass->mUseDMaxForASBuilding = value;
        }
        else if (key == kTextureItemSize)
        {
            pPass->mTextureItemSize = value;
        }
        else if (key == kMegaTextureCapacity)
        {
            pPass->mMegaTextureCapacity = value;
        }
        else if (key == kTextureItemSizeLQ)
        {
            pPass->mTextureItemSizeLQ = value;
        }
        else if (key == kMegaTextureCapacityLQ)
        {
            pPass->mMegaTextureCapacityLQ = value;
        }
        else if (key == kRelaxationFactor)
        {
            pPass->mRelaxationFactor = value;
        }
        else if (key == kStandardEnrich)
        {
            pPass->mUseStandardEnrich = value;
        }
        else if (key == kKNNRadiusScaler)
        {
            pPass->mKNNRadiusScaler = value;
        }
        else if (key == kMinPrs)
        {
            pPass->mMinPrs = value;
        }
    }
    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    Program::Desc desc;
    desc.addShaderLibrary("RenderPasses/RelaxationPass/RelaxationBlitPass.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpBlitPass = ComputePass::create(desc, Program::DefineList(), false);

    Program::Desc enrichDesc;
    enrichDesc.addShaderLibrary("RenderPasses/RelaxationPass/EnrichPass.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpEnrichPass = ComputePass::create(enrichDesc, Program::DefineList(), false);

    Program::Desc solidAnglePassDesc;
    solidAnglePassDesc.addShaderLibrary("RenderPasses/VirtualLightEstimatePass/ComputeSolidAngleLUT.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpLUTPass = ComputePass::create(solidAnglePassDesc, Program::DefineList(), false);

    Program::Desc convertPassDesc;
    convertPassDesc.addShaderLibrary("RenderPasses/VirtualLightEstimatePass/VirtualLightConvert.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpConvertPass = ComputePass::create(convertPassDesc, Program::DefineList(), false);
    pPass->mpConvertPassLQ = ComputePass::create(convertPassDesc, Program::DefineList(), false);

    Program::Desc deltaConvertPassDesc;
    deltaConvertPassDesc.addShaderLibrary("RenderPasses/VirtualLightEstimatePass/VirtualLightDeltaConvert.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpDeltaConvertPass = ComputePass::create(deltaConvertPassDesc, Program::DefineList(), false);
    pPass->mpDeltaConvertPassLQ = ComputePass::create(deltaConvertPassDesc, Program::DefineList(), false);

    return pPass;
}

std::string RelaxationPass::getDesc()
{
    return kDesc;
}

Dictionary RelaxationPass::getScriptingDictionary()
{
    Dictionary d;
    d[kTargetCount] = mTargetCount;
    d[kRadius] = mRadius;
    d[kRadiusScalerForASBuilding] = mRadiusScalerForASBuilding;
    d[kUseDMaxForASBuilding] = mUseDMaxForASBuilding;
    d[kTextureItemSize] = mTextureItemSize;
    d[kMegaTextureCapacity] = mMegaTextureCapacity;
    d[kTextureItemSizeLQ] = mTextureItemSizeLQ;
    d[kMegaTextureCapacityLQ] = mMegaTextureCapacityLQ;
    d[kRelaxationFactor] = mRelaxationFactor;
    d[kStandardEnrich] = mUseStandardEnrich;
    d[kKNNRadiusScaler] = mKNNRadiusScaler;
    d[kMinPrs] = mMinPrs;
    return d;
}

RenderPassReflection RelaxationPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "input");
    reflector.addOutput(kDummyOutput, "output");
    return reflector;
}

#define NEIGHBOUR_SEARCH_NORMAL_TOLERANCE 0.65

void RelaxationPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    // auto& pTexture = renderData["src"]->asTexture();
    if (mpScene == nullptr)
    {
        return;
    }

    if (gpState->currentState != 2)
    {
        mCurrentPhotonPathCount = 0;
        mNeedFinalize = true;
        return;
    }

    if (gpState->currentState != gpState->prevState)
    {
        mTimer.update();
        if (mpEmissiveTriTable == nullptr)
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
            mpEmissiveTriTable = AliasTable::create(fluxList, rng);
        }

        mpSpecularRadianceContainer = MegaTextureContainer::create(mMegaTextureCapacity, mTextureItemSize, mMegaTextureCapacityLQ, mTextureItemSizeLQ);

        mpSolidAngleLUT = Buffer::create((size_t)mTextureItemSize * (size_t)mTextureItemSize * sizeof(uint));
        mpSolidAngleLUT->setName("SolidAngleLUT");

        mpSolidAngleLUTLQ = Buffer::create((size_t)mTextureItemSizeLQ * (size_t)mTextureItemSizeLQ * sizeof(uint));
        mpSolidAngleLUTLQ->setName("SolidAngleLUTLQ");

        const uint sampleCount = 100000;

        {
            auto cb = mpLUTPass["CB"];
            cb["gItemSize"] = mTextureItemSize;
            cb["gSampleCount"] = sampleCount;
            mpLUTPass["gLUTBuffer"] = mpSolidAngleLUT;
            mpLUTPass->execute(pRenderContext, sampleCount, 1, 1);
            pRenderContext->flush(true);
        }

        {
            auto cb = mpLUTPass["CB"];
            cb["gItemSize"] = mTextureItemSizeLQ;
            cb["gSampleCount"] = sampleCount;
            mpLUTPass["gLUTBuffer"] = mpSolidAngleLUTLQ;
            mpLUTPass->execute(pRenderContext, sampleCount, 1, 1);
            pRenderContext->flush(true);
        }

        VirtualLightContainer::SharedPtr photonContainer = renderData.getDictionary()[kDicPhotons];
        VirtualLightContainer::SharedPtr importonContainer = renderData.getDictionary()[kDicInitialVirtualLights];
        uint32_t targetCount = std::min(mTargetCount, photonContainer->getCount());
        targetCount = std::max(1u, targetCount);

        Buffer::SharedPtr PhotonPositionReadBuffer = Buffer::create(photonContainer->getPositionBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
        pRenderContext->copyBufferRegion(PhotonPositionReadBuffer.get(), 0, photonContainer->getPositionBuffer().get(), 0, photonContainer->getPositionBuffer()->getSize());

        Buffer::SharedPtr PhotonNormalReadBuffer = Buffer::create(photonContainer->getNormalBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
        pRenderContext->copyBufferRegion(PhotonNormalReadBuffer.get(), 0, photonContainer->getNormalBuffer().get(), 0, photonContainer->getNormalBuffer()->getSize());

        Buffer::SharedPtr ImportonPositionReadBuffer = Buffer::create(importonContainer->getPositionBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
        pRenderContext->copyBufferRegion(ImportonPositionReadBuffer.get(), 0, importonContainer->getPositionBuffer().get(), 0, importonContainer->getPositionBuffer()->getSize());

        Buffer::SharedPtr ImportonNormalReadBuffer = Buffer::create(importonContainer->getNormalBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
        pRenderContext->copyBufferRegion(ImportonNormalReadBuffer.get(), 0, importonContainer->getNormalBuffer().get(), 0, importonContainer->getNormalBuffer()->getSize());

        Buffer::SharedPtr ImportonValueReadBuffer = Buffer::create(importonContainer->getThroughputBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
        pRenderContext->copyBufferRegion(ImportonValueReadBuffer.get(), 0, importonContainer->getThroughputBuffer().get(), 0, importonContainer->getThroughputBuffer()->getSize());
        pRenderContext->flush(true);

        float3* mappedPhotonPositions = (float3*)PhotonPositionReadBuffer->map(Buffer::MapType::Read);
        float3* mappedImportonPositions = (float3*)ImportonPositionReadBuffer->map(Buffer::MapType::Read);

        uint* mappedPhotonNormals = (uint*)PhotonNormalReadBuffer->map(Buffer::MapType::Read);
        uint* mappedImportonNormals = (uint*)ImportonNormalReadBuffer->map(Buffer::MapType::Read);

        float* mappedImportonThroughPut = (float*)ImportonValueReadBuffer->map(Buffer::MapType::Read);

        std::vector<double> weightsContainer;
        std::vector<float> photonRaidusContainer;
        weightsContainer.resize(photonContainer->getCount());
        photonRaidusContainer.resize(photonContainer->getCount());

        cy::PointCloud<float3, float, 3> PhotonCloud;
        PhotonCloud.Build(photonContainer->getCount(), mappedPhotonPositions);

        cy::PointCloud<float3, float, 3> ImportonCloud;
        ImportonCloud.Build(importonContainer->getCount(), mappedImportonPositions);

        mLocalTimer.begin("Compute Importon Weight");
        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, photonContainer->getCount()),
            [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i < r.end(); ++i)
            {
                float3 pos = mappedPhotonPositions[i];
                float3 normal = decodeNormal2x16(mappedPhotonNormals[i]);
                cy::PointCloud<float3, float, 3>::PointInfo pointInfos[NearestNeighborCount];
                int actualCount = ImportonCloud.GetPoints(pos, NearestNeighborCount, pointInfos);
                if (actualCount != NearestNeighborCount)
                {
                    std::cout << "This Should Not Happen" << std::endl;
                }

                double weight = 0.0f;

                float radiusSquared = 0.0f;
                // find the max disSquared
                {
                    for (uint j = 0; j < actualCount; j++)
                    {
                        float disSquared = pointInfos[j].distanceSquared;
                        if (disSquared > radiusSquared)
                        {
                            radiusSquared = disSquared;
                        }

                    }

                    if (radiusSquared <= 0.0f)
                    {
                        logError("Line 345 This Should Not Happen");
                    }

                }

                for (uint j = 0; j < actualCount; j++)
                {
                    float disSquared = pointInfos[j].distanceSquared;
                    if (disSquared < radiusSquared)
                    {
                        uint32_t index = pointInfos[j].index;
                        float a = std::max(0.0f, mappedImportonThroughPut[index]);
                        float3 importonNormal = decodeNormal2x16(mappedImportonNormals[index]);
                        if ((dot(normal, importonNormal) > NEIGHBOUR_SEARCH_NORMAL_TOLERANCE) && (a > 0.0f))
                        {
                            weight += ((double)a * (2.0 / (M_PI * (double)(radiusSquared + 1e-6f))) * std::max(0.0, std::min(1.0, 1.0 - (double)disSquared / (double)(radiusSquared + 1e-6f))));
                        }
                    }
                }

                /*if (weight <= 0.0f)
                {
                    logInfo("index " + std::to_string(i) + " radiusSquared is" + std::to_string(radiusSquared));
                    logInfo("index " + std::to_string(i) + " actualCount is" + std::to_string(actualCount));
                    for (uint j = 0; j < actualCount; j++)
                    {
                        logInfo("index " + std::to_string(i) + ":" + "j: " + std::to_string(j) + " disSquared is" + std::to_string(pointInfos[j].distanceSquared));
                        float disSquared = pointInfos[j].distanceSquared;
                        if (disSquared < radiusSquared)
                        {
                            uint32_t index = pointInfos[j].index;
                            float a = mappedImportonThroughPut[index];
                            logInfo("index " + std::to_string(i) + ":""j: " + std::to_string(j) + " a is" + std::to_string(a));
                            float3 importonNormal = decodeNormal2x16(mappedImportonNormals[index]);
                            if ((dot(normal, importonNormal) > NEIGHBOUR_SEARCH_NORMAL_TOLERANCE) && (a > 0.0f))
                            {
                                logInfo("index " + std::to_string(i) + ":""j: " + std::to_string(j) + "Normal Check Pass");
                                weight += ((double)a * (2.0 / (M_PI * (double)(radiusSquared + 1e-6f))) * std::max(0.0, std::min(1.0, 1.0 - (double)disSquared / (double)(radiusSquared + 1e-6f))));
                            }
                        }
                    }
                    logError("index " + std::to_string(i) + " Line 367 This Should Not Happen");
                }*/
                weightsContainer[i] = weight;
            }
        }
        );
        mLocalTimer.end();

        mLocalTimer.begin("Compute Prs");
        double weightSum = 0.0f;
        for (uint i = 0; i < photonContainer->getCount(); i++)
        {
            weightSum += weightsContainer[i];
        }
        double averageWeight = weightSum / double(photonContainer->getCount());
        std::cout << "Average Weight " << averageWeight << std::endl;
        double ratio = float(photonContainer->getCount()) / float(targetCount);

        float minPrs = mMinPrs;
        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, photonContainer->getCount()),
            [&weightsContainer, &ratio, &averageWeight, minPrs](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i < r.end(); ++i)
            {
                double prs = std::min(1.0, weightsContainer[i] / (ratio * averageWeight));
                // avoid too small prs which can blow up the whole rendering result.
                prs = std::max((double)minPrs, prs);
                weightsContainer[i] = prs;
            }
        }
        );
        mLocalTimer.end();

        mLocalTimer.begin("Reject Sampling");
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0, 1);//uniform distribution between 0 and 1

        std::vector<float3> candidatePositions;
        candidatePositions.reserve(targetCount);
        std::vector<uint32_t> candidateIndices;
        candidateIndices.reserve(targetCount);

        bool acceptAll = false;
        if (targetCount >= photonContainer->getCount())
        {
            acceptAll = true;
        }

        for (uint i = 0; i < photonContainer->getCount(); i++)
        {
            float r = dis(gen);
            if ((r < weightsContainer[i]) || acceptAll)
            {
                //accpet it
                candidatePositions.push_back(mappedPhotonPositions[i]);
            }
            mPrsArr.push_back(weightsContainer[i]);
        }
        mLocalTimer.end();

        std::cout << "Target Count Is " << targetCount << std::endl;
        std::cout << "Photon Count Is " << photonContainer->getCount() << std::endl;
        std::cout << "Actual Relaxation Count Is " << candidatePositions.size() << std::endl;

        pRenderContext->flush(true);


        cy::PointCloud<float3, float, 3> candidateCloud;
        for (uint i = 0; i < mIterationCount; i++)
        {
            mLocalTimer.begin("Relaxation Iteration" + std::to_string(i));
            // Rebuild The Tree
            candidateCloud.Build(candidatePositions.size(), candidatePositions.data());

            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, candidatePositions.size()),
                [&](const tbb::blocked_range<size_t>& r) {
                for (size_t j = r.begin(); j < r.end(); ++j)
                {
                    float3 pos = candidatePositions[j];
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

                    pos += mRelaxationFactor * deltaX;
                    candidatePositions[j] = pos;
                }
            }
            );

            //snap to nearest photons
            if (i == (mIterationCount - 1))
            {
                for (uint j = 0; j < candidatePositions.size(); j++)
                {
                    float3 pos = candidatePositions[j];
                    cy::PointCloud<float3, float, 3>::PointInfo pointInfos[1];
                    bool res = PhotonCloud.GetPoints(pos, 1, pointInfos);
                    assert(res);
                    candidatePositions[j] = pointInfos[0].pos;
                    candidateIndices.push_back(pointInfos[0].index);
                }
            }
            else
            {
                tbb::parallel_for(
                    tbb::blocked_range<size_t>(0, candidatePositions.size()),
                    [&candidatePositions, &PhotonCloud](const tbb::blocked_range<size_t>& r)
                {
                    for (size_t j = r.begin(); j < r.end(); ++j)
                    {
                        float3 pos = candidatePositions[j];
                        cy::PointCloud<float3, float, 3>::PointInfo pointInfos[1];
                        bool res = PhotonCloud.GetPoints(pos, 1, pointInfos);
                        assert(res);
                        candidatePositions[j] = pointInfos[0].pos;
                    }
                });
            }
            mLocalTimer.end();
        }

        assert(candidateIndices.size() == candidatePositions.size());

        PhotonPositionReadBuffer->unmap();
        ImportonPositionReadBuffer->unmap();
        ImportonValueReadBuffer->unmap();
        PhotonNormalReadBuffer->unmap();
        ImportonNormalReadBuffer->unmap();

        mpVirtualLightContainer = VirtualLightContainer::create(candidatePositions.size() * 2u, mRadius);
        mpVirtualLightContainer->getPositionBuffer()->setBlob(candidatePositions.data(), 0, candidatePositions.size() * sizeof(float3));
        mpVirtualLightContainer->setCount(pRenderContext, candidatePositions.size());

        mpFluxBuffer = Buffer::createStructured(sizeof(float), mpVirtualLightContainer->getCount());
        mpFluxBuffer->setName("RelaxationPass: Flux Buffer");

        mpVirtualLightContainer->setFluxBuffer(mpFluxBuffer);

        mLocalTimer.begin("Compute VSL Radius");
        std::vector<float> radiuses;
        radiuses.resize(candidatePositions.size());
        candidateCloud.Build(candidatePositions.size(), candidatePositions.data());
        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, candidatePositions.size()),
            [&](const tbb::blocked_range<size_t>& r)
        {
            for (size_t i = r.begin(); i < r.end(); i++)
            {
                float3 position = candidatePositions[i];
                std::vector<cy::PointCloud<float3, float, 3>::PointInfo> pointInfos;
                pointInfos.resize(mKNNRadiusScaler);
                int resCount = candidateCloud.GetPoints(position, mKNNRadiusScaler, pointInfos.data());
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

                radiuses[i] = sqrt(longest);
            }
        }
        );
        mLocalTimer.end();

        Buffer::SharedPtr dMaxBuffer = Buffer::createStructured(sizeof(float), radiuses.size(), ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource, Buffer::CpuAccess::None, radiuses.data());
        dMaxBuffer->setName("SE: dMax Buffer");
        Buffer::SharedPtr indicesBuffer = Buffer::createStructured(sizeof(uint), candidateIndices.size(), ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource, Buffer::CpuAccess::None, candidateIndices.data());
        indicesBuffer->setName("SE: Indices Buffer");
        {
            auto cb = mpBlitPass["CB"];
            cb["gUseDMaxForASBuilding"] = mUseDMaxForASBuilding;
            cb["gRadiusScalerForASBuilding"] = mRadiusScalerForASBuilding;
            photonContainer->setShaderData(cb["gPhotons"]);
            mpVirtualLightContainer->setShaderData(cb["gVirtualLights"]);
            mpBlitPass["gIndices"] = indicesBuffer;
            mpBlitPass["gDMaxs"] = dMaxBuffer;
            mpBlitPass->execute(pRenderContext, uint3(candidateIndices.size(), 1, 1));
        }

        mpVirtualLightContainer->updateCounterToCPU(pRenderContext);
        mpVirtualLightContainer->updateTextureCounterToCPU(pRenderContext);
        if (mMegaTextureCapacity < mpVirtualLightContainer->getTexCountHQ())
        {
            logError("mega texture capacity is not enough, should be more than " + std::to_string(mpVirtualLightContainer->getTexCountHQ()));
        }
        mpVirtualLightContainer->buildAS(pRenderContext);

        mTimer.update();
        logInfo("[===Time Stats===]Selection And Relaxation Takes " + std::to_string(mTimer.delta()) + "seconds");
        gpState->time += mTimer.delta();
        if (mUseStandardEnrich)
        {
            enrich(pRenderContext, photonContainer.get());
        }

        mTimer.update();
        logInfo("[===Time Stats===]Enriching Virtual Lights Takes " + std::to_string(mTimer.delta()) + "seconds");
        gpState->time += mTimer.delta();
    }

    if (mUseStandardEnrich)
    {
        gotoNextState(pRenderContext, renderData);
    }
    else
    {
        float invPathCount = 1.0 / mPhotonPathCount;
        if (mCurrentPhotonPathCount >= mPhotonPathCount && mNeedFinalize)
        {
            const uint perFrameProcessingCount = mPerFrameProcessingCount;
            PROFILE("Convert Processing");
            size_t dataBufferSize = mpSpecularRadianceContainer->GetDataBufferHQ()->getSize();
            auto convertedBuffer = Buffer::create(dataBufferSize);
            convertedBuffer->setName("MegaTexture: Converted Outgoing Radiance");

            uint texCount = mpVirtualLightContainer->getTexCountHQ();
            uint processedCount = 0;

            while (true)
            {
                if (processedCount >= texCount)
                {
                    break;
                }
                ShaderVar cb = mpConvertPass["CB"];
                cb["gCount"] = texCount;
                cb["gProcessedCount"] = processedCount;

                mpVirtualLightContainer->setShaderData(cb["gVirtualLights"]);
                mpSpecularRadianceContainer->setShaderData(cb["gIncidentContainer"]);

                mpConvertPass["gConvertedBuffer"] = convertedBuffer;
                mpConvertPass["gFluxBuffer"] = mpFluxBuffer;
                mpConvertPass->execute(pRenderContext, uint3(perFrameProcessingCount, mpSpecularRadianceContainer->GetPerItemSizeHQ(), mpSpecularRadianceContainer->GetPerItemSizeHQ()));

                pRenderContext->flush(true);

                processedCount += perFrameProcessingCount;
                std::cout << "Incident Radiance Processed" << processedCount << std::endl;
            }

            processedCount = 0;
            while (true)
            {
                if (processedCount >= texCount)
                {
                    break;
                }
                ShaderVar cb = mpDeltaConvertPass["CB"];
                cb["gCount"] = texCount;
                cb["gProcessedCount"] = processedCount;

                mpVirtualLightContainer->setShaderData(cb["gVirtualLights"]);
                mpSpecularRadianceContainer->setShaderData(cb["gIncidentContainer"]);

                mpDeltaConvertPass["gConvertedBuffer"] = convertedBuffer;
                mpDeltaConvertPass["gFluxBuffer"] = mpFluxBuffer;
                mpDeltaConvertPass["gSolidAngleLUTBuffer"] = mpSolidAngleLUT;
                mpDeltaConvertPass->execute(pRenderContext, uint3(perFrameProcessingCount, 1, 1));

                pRenderContext->flush(true);

                processedCount += perFrameProcessingCount;
                std::cout << "Delta Incident Radiance Processed" << processedCount << std::endl;
            }
            mpSpecularRadianceContainer->setDataBufferHQ(convertedBuffer);

            dataBufferSize = mpSpecularRadianceContainer->GetDataBufferLQ()->getSize();
            auto convertedBufferLQ = Buffer::create(dataBufferSize);
            convertedBufferLQ->setName("MegaTexture: Converted Outgoing Radiance LQ");
            texCount = mpVirtualLightContainer->getTexCountLQ();
            processedCount = 0;
            while (true)
            {
                if (processedCount >= texCount)
                {
                    break;
                }
                ShaderVar cb = mpConvertPass["CB"];
                cb["gCount"] = texCount;
                cb["gProcessedCount"] = processedCount;

                mpVirtualLightContainer->setShaderData(cb["gVirtualLights"]);
                mpSpecularRadianceContainer->setShaderData(cb["gIncidentContainer"]);

                mpConvertPassLQ["gConvertedBuffer"] = convertedBufferLQ;
                mpConvertPassLQ["gFluxBuffer"] = mpFluxBuffer;
                mpConvertPassLQ->execute(pRenderContext, uint3(perFrameProcessingCount, mpSpecularRadianceContainer->GetPerItemSizeLQ(), mpSpecularRadianceContainer->GetPerItemSizeLQ()));

                pRenderContext->flush(true);

                processedCount += perFrameProcessingCount;
                std::cout << "Incident Radiance LQ Processed" << processedCount << std::endl;
            }

            processedCount = 0;
            while (true)
            {
                if (processedCount >= texCount)
                {
                    break;
                }
                ShaderVar cb = mpDeltaConvertPass["CB"];
                cb["gCount"] = texCount;
                cb["gProcessedCount"] = processedCount;

                mpVirtualLightContainer->setShaderData(cb["gVirtualLights"]);
                mpSpecularRadianceContainer->setShaderData(cb["gIncidentContainer"]);

                mpDeltaConvertPass["gConvertedBuffer"] = convertedBufferLQ;
                mpDeltaConvertPass["gFluxBuffer"] = mpFluxBuffer;
                mpDeltaConvertPass["gSolidAngleLUTBuffer"] = mpSolidAngleLUTLQ;
                mpDeltaConvertPass->execute(pRenderContext, uint3(perFrameProcessingCount, 1, 1));

                pRenderContext->flush(true);

                processedCount += perFrameProcessingCount;
                std::cout << "Delta Incident Radiance Processed" << processedCount << std::endl;
            }
            mpSpecularRadianceContainer->setDataBufferLQ(convertedBufferLQ);

            /*
            Trigger Update
           */
            mpScene->getCamera()->setFocalDistance(100.0f);
            gotoNextState(pRenderContext, renderData);

            mNeedFinalize = false;
            return;
        }

        gpState->setState(2);
    }
}

void RelaxationPass::renderUI(Gui::Widgets& widget)
{
    widget.var("Target Count", mTargetCount, 1u, 1000000u);
    widget.var("Radius", mRadius, 0.000001f, 10.0f);
    widget.var("RadiusScalerForASBuilding", mRadiusScalerForASBuilding, 0.0f, 10.0f);
    widget.checkbox("UseDMaxForASBuilding", mUseDMaxForASBuilding);

    widget.var("Mega Texture Item Size", mTextureItemSize, 8u, 64u);
    widget.var("MegaTexture Capacity", mMegaTextureCapacity, 10000u, 1000000u);
    widget.var("Mega Texture Item Size LQ", mTextureItemSizeLQ, 8u, 64u);
    widget.var("MegaTexture Capacity LQ", mMegaTextureCapacityLQ, 10000u, 1000000u);

    widget.var("Relaxation Factor", mRelaxationFactor, 0.01f, 10.0f);

    widget.var("Photon Path Count", mPhotonPathCount, 1u, 100000000u);

    widget.var("KNNRadiusScaler", mKNNRadiusScaler, 1, 20);
    widget.checkbox("Standard Enrich", mUseStandardEnrich);

    if (mpVirtualLightContainer)
    {
        widget.text("VirtualLightCount: " + std::to_string(mpVirtualLightContainer->getCount()));
        widget.text("Textured Light Count: " + std::to_string(mpVirtualLightContainer->getTexCountHQ()));
        widget.text("Textured Light Count LQ: " + std::to_string(mpVirtualLightContainer->getTexCountLQ()));
    }

    widget.var("Min Prs", mMinPrs);
}

void RelaxationPass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpScene)
    {
        Shader::DefineList defines = mpScene->getSceneDefines();
        defines.add(mpSampleGenerator->getDefines());
        defines.add("_MS_DISABLE_ALPHA_TEST");
        defines.add("_DEFAULT_ALPHA_TEST");
        defines.add("_WRITE_MEGA_TEXTURE");
        defines.add("_PATH_COUNT", std::to_string(mPhotonPathCount));
        defines.add("_INV_PATH_COUNT", std::to_string(1.0f / (float)mPhotonPathCount));
        //defines.add("_SEPERATE_BRDF");
        defines.add("_TREAT_HIGHLY_GLOSSY_AS_DELTA", mTreatHighlyGlossyAsDelta ? "1" : "0");
        defines.add("_HIGHLY_GLOSSY_THRESHOLD", std::to_string(mHighlyGlossyThreshold));

        mpLUTPass->getProgram()->addDefines(defines);
        mpLUTPass->setVars(nullptr); // Trigger recompile

        mpConvertPass->getProgram()->addDefines(defines);
        mpConvertPass->getProgram()->addDefine("TEXTURE_PER_ITEM_SIZE", std::to_string(mTextureItemSize));
        mpConvertPass->getProgram()->addDefine("TEXTURE_TYPE", std::to_string(3));
        mpConvertPass->setVars(nullptr); // Trigger recompile

        mpDeltaConvertPass->getProgram()->addDefines(defines);
        mpDeltaConvertPass->getProgram()->addDefine("TEXTURE_PER_ITEM_SIZE", std::to_string(mTextureItemSize));
        mpDeltaConvertPass->getProgram()->addDefine("TEXTURE_TYPE", std::to_string(3));
        mpDeltaConvertPass->setVars(nullptr); // Trigger recompile

        mpConvertPassLQ->getProgram()->addDefines(defines);
        mpConvertPassLQ->getProgram()->addDefine("TEXTURE_PER_ITEM_SIZE", std::to_string(mTextureItemSizeLQ));
        mpConvertPassLQ->getProgram()->addDefine("TEXTURE_TYPE", std::to_string(1));
        mpConvertPassLQ->setVars(nullptr); // Trigger recompile

        mpDeltaConvertPassLQ->getProgram()->addDefines(defines);
        mpDeltaConvertPassLQ->getProgram()->addDefine("TEXTURE_PER_ITEM_SIZE", std::to_string(mTextureItemSizeLQ));
        mpDeltaConvertPassLQ->getProgram()->addDefine("TEXTURE_TYPE", std::to_string(1));
        mpDeltaConvertPassLQ->setVars(nullptr); // Trigger recompile

        mpBlitPass->getProgram()->addDefines(defines);
        mpBlitPass->setVars(nullptr); // Trigger recompile

        mpEnrichPass->getProgram()->addDefines(defines);
        mpEnrichPass->setVars(nullptr); // Trigger recompile
    }
}

#define MAX_PHOTON_SIZE 2048
void RelaxationPass::enrich(RenderContext* pRenderContext, VirtualLightContainer* photonContainer)
{
    pRenderContext->flush(true);
    Buffer::SharedPtr m_PrsBuffer = Buffer::createStructured(sizeof(float), mPrsArr.size(), Resource::BindFlags::UnorderedAccess | Resource::BindFlags::ShaderResource, Buffer::CpuAccess::None, mPrsArr.data());

    uint m_ToalIndexCount = mpVirtualLightContainer->getCount();
    //read GPU buffers
    Buffer::SharedPtr PhotonPositionReadBuffer = Buffer::create(photonContainer->getPositionBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
    pRenderContext->copyBufferRegion(PhotonPositionReadBuffer.get(), 0, photonContainer->getPositionBuffer().get(), 0, photonContainer->getPositionBuffer()->getSize());

    Buffer::SharedPtr ImportonPositionReadBuffer = Buffer::create(mpVirtualLightContainer->getPositionBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
    pRenderContext->copyBufferRegion(ImportonPositionReadBuffer.get(), 0, mpVirtualLightContainer->getPositionBuffer().get(), 0, mpVirtualLightContainer->getPositionBuffer()->getSize());

    pRenderContext->flush(true);

    float3* mappedPhotonPositions = (float3*)PhotonPositionReadBuffer->map(Buffer::MapType::Read);
    float3* mappedImportonPositions = (float3*)ImportonPositionReadBuffer->map(Buffer::MapType::Read);

    std::vector<float> weightsContainer;
    std::vector<uint> indexContainer;
    weightsContainer.resize((NearestNeighborCount - 1) * mpVirtualLightContainer->getCount());
    indexContainer.resize((NearestNeighborCount - 1) * mpVirtualLightContainer->getCount());

    cy::PointCloud<float3, float, 3> PhotonCloud;
    PhotonCloud.Build(photonContainer->getCount(), mappedPhotonPositions);

    cy::PointCloud<float3, float, 3> importonCloud;
    importonCloud.Build(mpVirtualLightContainer->getCount(), mappedImportonPositions);

    std::vector<uint> indices;
    indices.resize(1024);
    Buffer::SharedPtr indiciesBuffer = Buffer::createStructured(sizeof(uint), MAX_PHOTON_SIZE, ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource, Buffer::CpuAccess::None, nullptr, true);

    mLocalTimer.begin("Search Radiuses Computation");
    std::vector<float> searchRadiuses;
    searchRadiuses.resize(mpVirtualLightContainer->getCount());
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, searchRadiuses.size()),
        [&searchRadiuses, &importonCloud, &mappedImportonPositions](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i < r.end(); ++i)
        {
            float searchRadius = 0.0f;
            size_t searchCount = 3;
            std::vector<cy::PointCloud<float3, float, 3>::PointInfo> importonInfos;
            do
            {
                importonInfos.resize(searchCount);
                float3 pos = mappedImportonPositions[i];
                int actualFind = importonCloud.GetPoints(pos, searchCount, importonInfos.data());
                if (actualFind != searchCount)
                {
                    logError("Actual Find != Search Count! This Should Not Happen!!");
                }
                for (uint j = 0; j < searchCount; j++)
                {
                    float dis = sqrt(importonInfos[j].distanceSquared);
                    if (searchRadius < dis)
                    {
                        searchRadius = dis;
                    }
                }

                searchCount = searchCount * 2llu;
            } while (searchRadius <= 0.0f);
            searchRadiuses[i] = searchRadius;
        }
    }
    );
    mLocalTimer.end();

    std::vector<float> totalWeights;
    totalWeights.resize(mpVirtualLightContainer->getCount());

    std::vector<uint> totalIndices;
    totalIndices.resize(MAX_PHOTON_SIZE * mpVirtualLightContainer->getCount());

    std::vector<float> actualRadiuseSquares;
    actualRadiuseSquares.resize(mpVirtualLightContainer->getCount());

    std::vector<uint> actualPhotonCounts;
    actualPhotonCounts.resize(mpVirtualLightContainer->getCount());

    mLocalTimer.begin("Compute Photon Weight");
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, searchRadiuses.size()),
        [&actualPhotonCounts, &actualRadiuseSquares, &totalWeights, &searchRadiuses, &totalIndices, &PhotonCloud, &importonCloud, &mappedImportonPositions](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i < r.end(); ++i)
        {
            float actualRadiusSquared = 0.0f;
            int actualPhotonCount = 0;
            float3 pos = mappedImportonPositions[i];
            float searchRadius = searchRadiuses[i];
            std::vector<cy::PointCloud<float3, float, 3>::PointInfo> photonInfos;
            photonInfos.resize(MAX_PHOTON_SIZE);
            //cy::PointCloud<float3, float, 3>::PointInfo photonInfos[MAX_PHOTON_SIZE];
            //do
            //{
            actualPhotonCount = PhotonCloud.GetPoints(pos, searchRadius, MAX_PHOTON_SIZE, photonInfos.data());
            for (uint j = 0; j < actualPhotonCount; j++)
            {
                if (actualRadiusSquared < photonInfos[j].distanceSquared)
                {
                    actualRadiusSquared = photonInfos[j].distanceSquared;
                }
            }
            //searchRadius = searchRadius * 2.0f;
        //} while (actualRadiusSquared <= 1e-6f || (actualRadiusSquared <= 0.01f && actualPhotonCount <= 28u));

            actualPhotonCounts[i] = actualPhotonCount;
            actualRadiuseSquares[i] = actualRadiusSquared;

            float totalWeight = 0.0f;
            for (uint j = 0; j < actualPhotonCount; j++)
            {
                totalWeight += 1.0f - (photonInfos[j].distanceSquared / (actualRadiusSquared));
                totalIndices[i * MAX_PHOTON_SIZE + j] = photonInfos[j].index;
            }
            totalWeights[i] = totalWeight;
            if (actualPhotonCount <= 0u)
            {
                logInfo("index " + std::to_string(i) + " wrong photon weight" + std::to_string(totalWeight));
                logInfo("index " + std::to_string(i) + "actualPhotonCount" + std::to_string(actualPhotonCount));
                logInfo("index " + std::to_string(i) + "actualRadiusSquared" + std::to_string(actualRadiusSquared));
                logInfo("index " + std::to_string(i) + "searchPos is:" + std::to_string(pos.x) + ", " + std::to_string(pos.y) + ", " + std::to_string(pos.z));
                for (uint j = 0; j < actualPhotonCount; j++)
                {
                    logInfo("index " + std::to_string(i) + "photon position is: " + std::to_string(photonInfos[j].pos.x) + "," + std::to_string(photonInfos[j].pos.x) + "," + std::to_string(photonInfos[j].pos.z));
                }
                logError("index " + std::to_string(i) + "Wrong Data!!");
            }
        }
    }
    );
    mLocalTimer.end();

    mLocalTimer.begin("Actual Radiance Estmation");
    for (uint i = 0; i < mpVirtualLightContainer->getCount(); i++)
    {
        indiciesBuffer->setBlob(totalIndices.data() + i * MAX_PHOTON_SIZE, 0, actualPhotonCounts[i] * sizeof(uint));

        auto cb = mpEnrichPass["PerFrameCB"];
        mpSpecularRadianceContainer->setShaderData(cb["gSpecRadianceContainer"]);
        cb["gIndex"] = i;
        cb["gRadiusSquared"] = actualRadiuseSquares[i];
        float totalWeight = totalWeights[i];
        if (isnan(totalWeight) || isinf(totalWeight))
        {
            totalWeight = 0.0f;
        }
        cb["gTotalWeight"] = totalWeight;
        cb["gActualPhotonCount"] = actualPhotonCounts[i];
        if (actualPhotonCounts[i] > MAX_PHOTON_SIZE)
        {
            logInfo("Too Many Photon, This Should Not Happen");
        }

        photonContainer->setShaderData(cb["gPhotons"]);
        mpVirtualLightContainer->setShaderData(cb["gVirtualLights"]);

        mpEnrichPass["gPrsBuffer"] = m_PrsBuffer;
        mpEnrichPass["gIndicesBuffer"] = indiciesBuffer;
        mpEnrichPass["gFluxBuffer"] = mpFluxBuffer;

        pRenderContext->resourceBarrier(m_PrsBuffer.get(), Resource::State::UnorderedAccess);
        pRenderContext->resourceBarrier(indiciesBuffer.get(), Resource::State::UnorderedAccess);
        pRenderContext->resourceBarrier(mpSpecularRadianceContainer->GetDataBufferHQ(), Resource::State::UnorderedAccess);
        pRenderContext->resourceBarrier(mpVirtualLightContainer->getPositionBuffer().get(), Resource::State::UnorderedAccess);

        mpEnrichPass->execute(pRenderContext, uint3(MAX_PHOTON_SIZE, 1, 1));

        pRenderContext->resourceBarrier(m_PrsBuffer.get(), Resource::State::UnorderedAccess);
        pRenderContext->resourceBarrier(indiciesBuffer.get(), Resource::State::UnorderedAccess);
        pRenderContext->resourceBarrier(mpSpecularRadianceContainer->GetDataBufferHQ(), Resource::State::UnorderedAccess);
        pRenderContext->resourceBarrier(mpVirtualLightContainer->getPositionBuffer().get(), Resource::State::UnorderedAccess);

        pRenderContext->flush(true);

        if (i % 1000 == 0)
        {
            std::cout << "Rich Virtual Light " << i << " Finished" << std::endl;
            gpDevice->flushAndSync();
        }
    }

    PhotonPositionReadBuffer->unmap();
    ImportonPositionReadBuffer->unmap();

    pRenderContext->flush(true);
    mLocalTimer.end();
}

void RelaxationPass::gotoNextState(RenderContext* pRenderContext, const RenderData& renderData)
{
    {
        Buffer::SharedPtr readBuffer = Buffer::create(mpFluxBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read, nullptr);
        pRenderContext->copyBufferRegion(readBuffer.get(), 0, mpFluxBuffer.get(), 0, mpFluxBuffer->getSize());
        pRenderContext->flush(true);

        uint lightCount = mpVirtualLightContainer->getCount();
        std::vector<float> weights(lightCount);

        float* fluxData = (float*)readBuffer->map(Buffer::MapType::Read);
        for (size_t i = 0; i < mpVirtualLightContainer->getCount(); i++)
        {
            weights[i] = fluxData[i];
        }
        readBuffer->unmap();
        std::mt19937 rng;
        mpFluxTable = AliasTable::create(weights, rng);
    }

    renderData.getDictionary()[kDicFluxTable] = mpFluxTable;

    renderData.getDictionary()[kDicRadianceReady] = true;

    renderData.getDictionary()[kDicFluxBuffer] = mpFluxBuffer;

    renderData.getDictionary()[kDicSpecularRadianceContainer] = mpSpecularRadianceContainer;

    renderData.getDictionary()[kDicSampleEliminatedVirtualLights] = mpVirtualLightContainer;
    renderData.getDictionary()[kDicCurVirtualLights] = mpVirtualLightContainer;

    uint2 tileDims = renderData.getDictionary()[kDicTileDim];
    uint tileSize = renderData.getDictionary()[kDicTileSize];
    uint tileSampleNum = renderData.getDictionary()[kDicTileSampleNum];

    if (!renderData.getDictionary().keyExists(kDicTileVirtualLightWeights))
    {
        renderData.getDictionary()[kDicTileVirtualLightWeights] = Buffer::createStructured(sizeof(uint), tileDims.x * tileDims.y * (tileSampleNum + 1));
    }

    gpState->setState(3);
}


