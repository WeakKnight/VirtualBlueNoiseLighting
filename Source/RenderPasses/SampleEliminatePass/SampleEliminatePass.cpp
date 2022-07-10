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
#include "SampleEliminatePass.h"
#include "cyCore.h"
#include "cyHeap.h"
#include "cyPointCloud.h"
#include <tbb/parallel_for.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/atomic.h>
#include <tbb/parallel_sort.h>
#include <limits>
#include <set>
#include "nanoflann.hpp"

Timer timer;


namespace
{
    const char kDummy[] = "dummy";
    const char kDummyInput[] = "input";
    const char kDummyOutput[] = "output";
    const char kDesc[] = "get a blue-noise-distributed set of samples";
    const char kRatio[] = "ratio";
    const char kRadiusSearchRange[] = "radiusSearchRange";
    const char kRadiusSearchCount[] = "radiusSearchCount";
    const char kRadius[] = "radius";
    const char kUniformSE[] = "uniformSE";
    const char kUseSE[] = "useSE";
    const char kRadiusScalerForASBuilding[] = "radiusScalerForASBuilding";
    const char kUseDMaxForASBuilding[] = "useDMaxForASBuilding";
    const char kUseParallelSE[] = "useParallelSE";
    const char kFillTileHoles[] = "fillTileHoles";

    const std::string kDicInitialVirtualLights = "initialVirtualLights";
    const std::string kDicSampleEliminatedVirtualLights = "sampleEliminatedVirtualLights";
    const std::string kDicCurVirtualLights = "curVirtualLights";

    const std::string kDicTileDim = "tileDim";
    const std::string kDicTileSampleNum = "tileSampleNum";
    const std::string kDicTileVirtualLightContainer = "tileVirtualLightContainer";
    const std::string kDicTileVirtualLightWeights = "tileVirtualLightWeights";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

static void regPythonApi(pybind11::module& m)
{
    pybind11::class_<SampleEliminatePass, RenderPass, SampleEliminatePass::SharedPtr> pass(m, "SampleEliminatePass");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary & lib)
{
    lib.registerClass("SampleEliminatePass", kDesc, SampleEliminatePass::create);
    ScriptBindings::registerBinding(regPythonApi);
}

SampleEliminatePass::SharedPtr SampleEliminatePass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new SampleEliminatePass);
    for (const auto& [key, value] : dict)
    {
        if (key == kRatio)
        {
            pPass->mRatio = value;
        }
        else if (key == kRadiusSearchRange)
        {
            pPass->mRadiusSearchRange = value;
        }
        else if (key == kRadiusSearchCount)
        {
            pPass->mRadiusSearchCount = value;
        }
        else if (key == kRadius)
        {
            pPass->mRadius = value;
        }
        else if (key == kUniformSE)
        {
            pPass->mUniformSE = value;
        }
        else if (key == kUseSE)
        {
            pPass->mUseSE = value;
        }
        else if (key == kRadiusScalerForASBuilding)
        {
            pPass->mRadiusScalerForASBuilding = value;
        }
        else if (key == kUseDMaxForASBuilding)
        {
            pPass->mUseDMaxForASBuilding = value;
        }
        else if (key == kUseParallelSE)
        {
            pPass->mUseParallelSE = value;
        }
        else if (key == kFillTileHoles)
        {
            pPass->mFillTileHoles = value;
        }
    }
    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);
    Program::Desc desc;
    desc.addShaderLibrary("RenderPasses/SampleEliminatePass/SampleEliminate.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpComputePass = ComputePass::create(desc, Program::DefineList(), false);

    Program::Desc validationDesc;
    validationDesc.addShaderLibrary("RenderPasses/SampleEliminatePass/TileVirtualLightValidation.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpValidationPass = ComputePass::create(validationDesc, Program::DefineList(), false);

    Program::Desc countDesc;
    countDesc.addShaderLibrary("RenderPasses/SampleEliminatePass/TileVirtualLightCountWeight.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpCountWeightPass = ComputePass::create(countDesc, Program::DefineList(), false);

    //Program::Desc dilationDesc;
    //dilationDesc.addShaderLibrary("RenderPasses/SampleEliminatePass/TileVirtualLightDilation.cs.slang").csEntry("main").setShaderModel("6_5");
    //pPass->mpDilationPass = ComputePass::create(dilationDesc, Program::DefineList(), false);

    return pPass;
}

std::string SampleEliminatePass::getDesc()
{
    return kDesc;
}

Dictionary SampleEliminatePass::getScriptingDictionary()
{
    Dictionary d;
    d[kRatio] = mRatio;
    d[kRadiusSearchRange] = mRadiusSearchRange;
    d[kRadiusSearchCount] = mRadiusSearchCount;
    d[kRadius] = mRadius;
    d[kUniformSE] = mUniformSE;
    d[kUseSE] = mUseSE;
    d[kRadiusScalerForASBuilding] = mRadiusScalerForASBuilding;
    d[kUseDMaxForASBuilding] = mUseDMaxForASBuilding;
    d[kUseParallelSE] = mUseParallelSE;
    d[kFillTileHoles] = mFillTileHoles;
    return d;
}

RenderPassReflection SampleEliminatePass::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "useless dummy input");
    reflector.addOutput(kDummyOutput, "useless dummy output");
    return reflector;
}

void SampleEliminatePass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
    {
        return;
    }

    if (gpState->currentState != 1)
    {
        return;
    }

    renderData.getDictionary()[kFillTileHoles] = mFillTileHoles;

    VirtualLightContainer::SharedPtr initialVirtualLights = renderData.getDictionary()[kDicInitialVirtualLights];
    if (initialVirtualLights == nullptr)
    {
        debugBreak(); // should not be nullptr here
        return;
    }

    Buffer::SharedPtr tileVirtualLightContainer = renderData.getDictionary()[kDicTileVirtualLightContainer];
    uint2 tileDims = renderData.getDictionary()[kDicTileDim];
    uint tileSampleNum = renderData.getDictionary()[kDicTileSampleNum];
    if (tileVirtualLightContainer == nullptr)
    {
        debugBreak();
        return; // should not be nullptr here
    }

    if (gpState->currentState != gpState->prevState)
    {
        uint targetCount = (float)initialVirtualLights->getCount() * mRatio;
        if (!mUseSE)
        {
            mUseParallelSE = false;
            targetCount = initialVirtualLights->getCount() - 1;
        }

        std::vector<uint32_t> outputIndices;
        outputIndices.reserve(targetCount);
        std::vector<float3> outputPositions;
        outputPositions.reserve(targetCount);
        std::vector<float> dmaxs;
        dmaxs.reserve(targetCount);
        eliminatie(pRenderContext, initialVirtualLights, targetCount, outputIndices, outputPositions, dmaxs);

        targetCount = outputIndices.size();
        mpSampleEliminatedVirtualLights = VirtualLightContainer::create(targetCount, mRadius);
        mpSampleEliminatedVirtualLights->getPositionBuffer()->setBlob(outputPositions.data(), 0, outputPositions.size() * sizeof(float3));
        mpSampleEliminatedVirtualLights->setCount(pRenderContext, targetCount);
        pRenderContext->flush(true);

        Buffer::SharedPtr hashMapBuffer = Buffer::createStructured(sizeof(uint), initialVirtualLights->getCount(), ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, mpSEHashMap.data());
        hashMapBuffer->setName("SE: HashMap");

        // Copy Data
        {
            ShaderVar cb = mpComputePass["CB"];
            cb["gUseDMaxForASBuilding"] = mUseDMaxForASBuilding;
            cb["gRadiusScalerForASBuilding"] = mRadiusScalerForASBuilding;
            initialVirtualLights->setShaderData(cb["gInitialVirtualLights"]);
            mpSampleEliminatedVirtualLights->setShaderData(cb["gSampleEliminatedVirtualLights"]);

            Buffer::SharedPtr indicesBuffer = Buffer::createStructured(sizeof(uint), outputIndices.size(), ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource, Buffer::CpuAccess::None, outputIndices.data());
            indicesBuffer->setName("SE: Indices Buffer");
            Buffer::SharedPtr dMaxBuffer = Buffer::createStructured(sizeof(float), dmaxs.size(), ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource, Buffer::CpuAccess::None, dmaxs.data());
            dMaxBuffer->setName("SE: dMax Buffer");
            mpComputePass["gIndices"] = indicesBuffer;
            mpComputePass["gDMaxs"] = dMaxBuffer;
            mpComputePass["gHashMap"] = hashMapBuffer;

            mpComputePass->execute(pRenderContext, uint3(targetCount, 1, 1));
            pRenderContext->flush(true);
            mpSampleEliminatedVirtualLights->buildAS(pRenderContext);
        }

        mpSampleEliminatedVirtualLights->updateTextureCounterToCPU(pRenderContext);

        // Validate Tile Virtual Light Container
        {
            ShaderVar cb = mpValidationPass["CB"];
            cb["gTileSampleNum"] = tileSampleNum;
            cb["gTileDims"] = tileDims;
            //initialVirtualLights->setShaderData(cb["gInitialVirtualLights"]);
            //mpSampleEliminatedVirtualLights->setShaderData(cb["gSampleEliminatedVirtualLights"]);

            mpValidationPass["gHashMap"] = hashMapBuffer;
            mpValidationPass["gTileVirtualLightContainer"] = tileVirtualLightContainer;
            mpValidationPass->execute(pRenderContext, uint3(tileDims, 1));
        }

        // Dilation
       /* std::vector<uint> initTileData;
        initTileData.resize(tileDims.x * tileDims.y * tileSampleNum, 0xffffffff);
        {
            ShaderVar cb = mpDilationPass["CB"];
            cb["gTileSampleNum"] = tileSampleNum;
            cb["gTileDims"] = tileDims;

            auto outputBuffer = Buffer::create(tileVirtualLightContainer->getSize(), ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource, Buffer::CpuAccess::None, initTileData.data());
            mpDilationPass["gTileVirtualLightContainer"] = tileVirtualLightContainer;
            mpDilationPass["gOutput"] = outputBuffer;
            mpDilationPass->execute(pRenderContext, uint3(tileDims, 1));
            pRenderContext->flush(true);
            pRenderContext->copyBufferRegion(tileVirtualLightContainer.get(), 0, outputBuffer.get(), 0, tileVirtualLightContainer->getSize());
            pRenderContext->flush(true);
        }*/

        if (mpTileVirtualLightWeights == nullptr)
        {
            mpTileVirtualLightWeights = Buffer::createStructured(sizeof(uint), mpSampleEliminatedVirtualLights->getCount());
            mpTileVirtualLightWeights->setName("TileVirtualLightWeights");

            ShaderVar cb = mpCountWeightPass["CB"];
            cb["gTileSampleNum"] = tileSampleNum;
            cb["gTileDims"] = tileDims;
            //initialVirtualLights->setShaderData(cb["gInitialVirtualLights"]);
            //mpSampleEliminatedVirtualLights->setShaderData(cb["gSampleEliminatedVirtualLights"]);

            mpCountWeightPass["gTileVirtualLightWeights"] = mpTileVirtualLightWeights;
            mpCountWeightPass["gTileVirtualLightContainer"] = tileVirtualLightContainer;
            mpCountWeightPass->execute(pRenderContext, uint3(tileDims, 1));
        }
    }

    renderData.getDictionary()[kDicTileVirtualLightWeights] = mpTileVirtualLightWeights;
    renderData.getDictionary()[kDicCurVirtualLights] = mpSampleEliminatedVirtualLights;
    renderData.getDictionary()[kDicSampleEliminatedVirtualLights] = mpSampleEliminatedVirtualLights;

    gpState->setState(2);
}

void SampleEliminatePass::renderUI(Gui::Widgets& widget)
{
    if (mpSampleEliminatedVirtualLights != nullptr)
    {
        widget.text("Final Virtual Light Count: " + std::to_string(mpSampleEliminatedVirtualLights->getCount()));
        widget.text("Final Mega Texture Item Count: " + std::to_string(mpSampleEliminatedVirtualLights->getTexCountHQ()));
        widget.text("Final Mega Texture Item LQ Count: " + std::to_string(mpSampleEliminatedVirtualLights->getTexCountLQ()));
    }

    widget.var("Ratio", mRatio, 0.02f, 1.0f);
    widget.var("RadiusSearchRange", mRadiusSearchRange, 0.0f, 1.0f);
    widget.var("RadiusSearchCount", mRadiusSearchCount, 0u, 1000u);
    widget.var("Radius", mRadius, 0.0f, 1.0f);
    widget.checkbox("UniformSE", mUniformSE);
    widget.checkbox("UseSE", mUseSE);
    widget.var("RadiusScalerForASBuilding", mRadiusScalerForASBuilding, 0.0f, 10.0f);
    widget.checkbox("UseDMaxForASBuilding", mUseDMaxForASBuilding);
    widget.checkbox("Parallel SE", mUseParallelSE);
    widget.checkbox("Fill Tile Holes", mFillTileHoles);
}

void SampleEliminatePass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
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

        mpValidationPass->getProgram()->addDefines(defines);
        mpValidationPass->setVars(nullptr); // Trigger recompile

        mpCountWeightPass->getProgram()->addDefines(defines);
        mpCountWeightPass->setVars(nullptr); // Trigger recompile

        //mpDilationPass->getProgram()->addDefines(defines);
        //mpDilationPass->setVars(nullptr); // Trigger recompile
    }
}

void SampleEliminatePass::eliminatie(RenderContext* pRenderContext, VirtualLightContainer::SharedPtr initialVirtualLights, uint targetCount, std::vector<uint>& outputIndices, std::vector<float3>& outputPositions, std::vector<float>& dmaxs)
{
    uint inputCount = initialVirtualLights->getCount();

    mpSEHashMap.resize(inputCount, 0xffffffffu);

    Buffer::SharedPtr positionReadBuffer = Buffer::create(initialVirtualLights->getPositionBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
    pRenderContext->copyBufferRegion(positionReadBuffer.get(), 0, initialVirtualLights->getPositionBuffer().get(), 0, initialVirtualLights->getPositionBuffer()->getSize());
    pRenderContext->flush(true);
    float3* inputPositions = (float3*)positionReadBuffer->map(Buffer::MapType::Read);

#if 0
    Buffer::SharedPtr metalRoughnessReadBuffer = Buffer::create(initialVirtualLights->getMetalRoughnessBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
    pRenderContext->copyBufferRegion(metalRoughnessReadBuffer.get(), 0, initialVirtualLights->getMetalRoughnessBuffer().get(), 0, initialVirtualLights->getMetalRoughnessBuffer()->getSize());
    pRenderContext->flush(true);

    std::vector<float> roughnesses;
    roughnesses.resize(inputCount, 0.0f);
    float2* metalRoushnesses = (float2*)metalRoughnessReadBuffer->map(Buffer::MapType::Read);
    for (int i = 0; i < inputCount; i++)
    {
        roughnesses[i] = metalRoushnesses[i].y;
    }
    metalRoughnessReadBuffer->unmap();
#endif

    logInfo("ready to SE, initialCount: " + std::to_string(inputCount) + "  targetCount:  " + std::to_string(targetCount) + " .");
    if (mUseParallelSE)
    {
        timer.begin("[===Time Stats===]Sample Elimination");
        Timer tempTimer;
        tempTimer.begin("Compute Adaptive Radius");
        float ratio = float(inputCount) / float(targetCount);
        float3 boundingBoxMin(std::numeric_limits<float>::max());
        float3 boundingBoxMax(-std::numeric_limits<float>::max());
        std::vector<uint> inputIndices;
        inputIndices.reserve(inputCount);
        // build the large boundingbox
        {
            for (int i = 0; i < inputCount; i++)
            {
                float3 thisPoint = inputPositions[i];

                if (thisPoint.x < boundingBoxMin.x) boundingBoxMin.x = thisPoint.x;
                if (thisPoint.y < boundingBoxMin.y) boundingBoxMin.y = thisPoint.y;
                if (thisPoint.z < boundingBoxMin.z) boundingBoxMin.z = thisPoint.z;

                if (thisPoint.x > boundingBoxMax.x) boundingBoxMax.x = thisPoint.x;
                if (thisPoint.y > boundingBoxMax.y) boundingBoxMax.y = thisPoint.y;
                if (thisPoint.z > boundingBoxMax.z) boundingBoxMax.z = thisPoint.z;

                inputIndices.emplace_back(i);
            }
        }
        Partition mp(boundingBoxMin, boundingBoxMax, 2, ratio);


        PointCloud<float> cloud;
        cloud.pts.resize(inputCount);
        memcpy(cloud.pts.data(), inputPositions, inputCount * sizeof(float3));

        // construct a kd-tree index:
        using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<float, PointCloud<float>>,
            PointCloud<float>, 3 /* dim */
        >;

        my_kd_tree_t index(3 /*dim*/, cloud, { 20 /* max leaf */ });

        // cy::PointCloud<float3, float, 3, uint> kdtree;
        // kdtree.Build(inputCount, inputPositions);
        auto getPoissonDiskRadius = [&](float3 pos)
        {
            int radiusSerchCount = mRadiusSearchCount > 256 ? 256 : mRadiusSearchCount;
            uint actualCount = radiusSerchCount;
            float actualSquaredRadius;

            size_t                num_results = radiusSerchCount;
            std::vector<uint32_t> ret_index(num_results);
            std::vector<float>    out_dist_sqr(num_results);

            num_results = index.knnSearch(
                &pos.x, num_results, &ret_index[0], &out_dist_sqr[0]);


            if (mRadiusSearchRange >= out_dist_sqr.back())
            {
                auto upper = std::upper_bound(out_dist_sqr.begin(), out_dist_sqr.end(), mRadiusSearchRange);
                int index = std::distance(out_dist_sqr.begin(), upper) - 1;
                index = index > 0 ? index : 0;
                actualSquaredRadius = out_dist_sqr.at(index);
                actualCount = index;
            }
            else
            {
                actualSquaredRadius = out_dist_sqr.back();
            }

            //kdtree.PointsSearchExtRadiusFirst(pos, mRadiusSearchCount, mRadiusSearchRange, actualCount, actualSquaredRadius);
            float sampleDomainArea = ratio * cy::Pi<float>() * actualSquaredRadius / (float)actualCount;
            float result = sqrt(sampleDomainArea / (2.0 * sqrt(3.0)));
            result = std::max(std::min(mRadius, result), 1e-6f);
            return result;
        };
        std::vector<float> dMaxList(inputCount);
        std::vector<float> reverseDMaxList(inputCount);
        tbb::parallel_for(0u, inputCount, 1u, [&](uint i)
            {
                if (mUniformSE)
                {
                    dMaxList[i] = 2.0f * mRadius;
                }
                else
                {
                    dMaxList[i] = 2.0f * getPoissonDiskRadius(inputPositions[i]);
                }
                reverseDMaxList[i] = dMaxList[i];
            });
        tempTimer.end();

        tempTimer.begin("Elimination with multiple Partition");
        //start elimination
        std::vector<uint> tempIndices1, tempIndices2, targetCountEachGrid1, targetCountEachGrid2, targetCountEachGrid3;
        eliminateWithPartition(2, 0.8f, inputCount, dMaxList, reverseDMaxList, mp, inputIndices, inputPositions, tempIndices1);
        mp.setRatio(float(tempIndices1.size()) / float(targetCount));
        eliminateWithPartition(1, 0.8f, inputCount, dMaxList, reverseDMaxList, mp, tempIndices1, inputPositions, tempIndices2);
        mp.setRatio(float(tempIndices2.size()) / float(targetCount));
        eliminateWithPartition(0, 1.00f, inputCount, dMaxList, reverseDMaxList, mp, tempIndices2, inputPositions, outputIndices, &outputPositions, &dmaxs);
        logInfo("Actual Count" + std::to_string(outputPositions.size()));
        tempTimer.end();

        timer.end();
        gpState->time += timer.delta();
    }
    else
    {
        timer.begin("Sample Elimination");
        cy::PointCloud<float3, float, 3, uint> kdtree;
        kdtree.Build(inputCount, inputPositions);
        float ratio = float(inputCount) / float(targetCount);
        auto getPoissonDiskRadius = [&](float3 pos)
        {
            uint actualCount;
            float actualSquaredRadius;
            kdtree.PointsSearchExtRadiusFirst(pos, mRadiusSearchCount, mRadiusSearchRange, actualCount, actualSquaredRadius);
            float sampleDomainArea = ratio * cy::Pi<float>() * actualSquaredRadius / (float)actualCount;
            float result = sqrt(sampleDomainArea / (2.0 * sqrt(3.0)));
            result = std::max(std::min(mRadius, result), 1e-6f);
            return result;
        };
        std::vector<float> dMaxList(inputCount);
        std::vector<float> reverseDMaxList(inputCount);

        uint remainCount = inputCount;

        if (remainCount > targetCount)
        {
            tbb::parallel_for(0u, inputCount, 1u, [&](uint i)
                {
                    if (mUniformSE)
                    {
                        dMaxList[i] = 2.0f * mRadius;
                    }
                    else
                    {
                        dMaxList[i] = 2.0f * getPoissonDiskRadius(inputPositions[i]);
                    }
                    reverseDMaxList[i] = dMaxList[i];
                });
        }
        else
        {
            tbb::parallel_for(0u, inputCount, 1u, [&](uint i)
                {
                    float3 position = inputPositions[i];
                    std::vector<cy::PointCloud<float3, float, 3>::PointInfo> pointInfos;
                    pointInfos.resize(5);
                    int resCount = kdtree.GetPoints(position, 5, pointInfos.data());
                    if (resCount <= 0)
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

                    dMaxList[i] = longest;
                    reverseDMaxList[i] = longest;
                });
        }


        auto weightFunction = [&](float d2, float dMax)
        {
            float d = sqrt(d2);
            const float alpha = 8.0f;
            return std::pow(1.0 - d / dMax, alpha);
        };
        std::vector<float> weights(inputCount, 0.0f);
        for (uint index = 0; index < inputCount; index++)
        {
            float3 point = inputPositions[index];
            float dMax = dMaxList[index];
            kdtree.GetPoints(point, dMaxList[index], [&](uint i, float3 const& p, float d2, float&)
                {
                    if (i >= inputCount)
                    {
                        return;
                    }
                    if (i != index)
                    {
                        float weight = weightFunction(d2, dMax);
                        weights[index] += weight;
                        if (reverseDMaxList[i] < dMax)
                        {
                            reverseDMaxList[i] = dMax;
                        }
                    }
                });
        }
        cy::Heap<float, uint> maxHeap;
        maxHeap.SetDataPointer(weights.data(), inputCount);
        maxHeap.Build();

        while (remainCount > targetCount)
        {
            uint index = maxHeap.GetTopItemID();
            maxHeap.Pop();

            float3 point = inputPositions[index];
            kdtree.GetPoints(point, reverseDMaxList[index], [&](uint i, float3 const& p, float d2, float&)
                {
                    if (i > inputCount)
                    {
                        return;
                    }
                    if (i != index)
                    {
                        float dMax = dMaxList[i];
                        if (dMax * dMax > d2)
                        {
                            float weight = weightFunction(d2, dMax);
                            weights[i] -= weight;
                            maxHeap.MoveItemDown(i);
                        }
                    }
                });
            remainCount--;
        }
        for (uint i = 0; i < targetCount; i++)
        {
            outputIndices.emplace_back(maxHeap.GetIDFromHeap(i));
            outputPositions.emplace_back(inputPositions[outputIndices[i]]);
            dmaxs.emplace_back(dMaxList[outputIndices[i]]);
        }
        logInfo("Actual Count" + std::to_string(outputPositions.size()));
        timer.end();
        gpState->time += timer.delta();
    }

    if (mFillTileHoles)
    {
        std::vector<bool> removedSamples;
        removedSamples.resize(inputCount, true);

        for (uint i = 0; i < outputIndices.size(); i++)
        {
            removedSamples[outputIndices[i]] = false;
        }

        std::vector<uint> removedSampleList;
        removedSampleList.reserve(inputCount - outputIndices.size());
        for (uint i = 0; i < inputCount; i++)
        {
            if (removedSamples[i])
            {
                removedSampleList.emplace_back(i);
            }
        }
        assert(removedSampleList.size() == (inputCount - outputIndices.size()));
        assert(outputIndices.size() == outputPositions.size());

        cy::PointCloud<float3, float, 3, uint> kdtreeAfterSE;
        kdtreeAfterSE.Build(outputIndices.size(), outputPositions.data());
        for (uint i = 0; i < removedSampleList.size(); i++)
        {
            uint oldIndex = removedSampleList[i];
            float3 position = inputPositions[oldIndex];
            uint newIndex;
            bool valid = kdtreeAfterSE.GetClosestIndex(position, newIndex);
            assert(valid);
            mpSEHashMap[oldIndex] = newIndex;
        }
    }

    positionReadBuffer->unmap();
}

void SampleEliminatePass::eliminateWithPartition(int level, float percent, int inputCount, std::vector<float>& inputDmax, std::vector<float>& inputReverseDmax, Partition& mp, std::vector<uint>& inputIndices, float3* pointCloud, std::vector<uint>& outputIndices, std::vector<float3>* outputPositions, std::vector<float>* dMaxArray)
{
    Timer tempTimer;
    tempTimer.begin("Elimination with Level" + std::to_string(level));
    mp.setLevel(level);
    uint numTotalPoints = inputCount;
    uint currentNumOfPoints = inputIndices.size();
    uint numOfCells = (1 << level) * (1 << level) * (1 << level);

    //global indices
    std::vector<std::vector<uint>> partitionIndices;
    partitionIndices.resize(numOfCells);
    //positions
    std::vector< std::vector<float3>> partitionPoints;
    partitionPoints.resize(numOfCells);
    //weights
    std::vector<std::vector<float>> partitionWeights;
    partitionWeights.resize(numOfCells);
    //kdtrees
    std::vector<cy::PointCloud<float3, float, 3, uint>> kdtrees;
    kdtrees.resize(numOfCells);

    //generating partitions 
    for (int i = 0; i < numOfCells; i++)
    {
        partitionIndices[i].reserve(int((float)currentNumOfPoints / numOfCells) + 32);
        partitionPoints[i].reserve(int((float)currentNumOfPoints / numOfCells) + 32);
    }
    const bool useUniformPartition = false;

    Timer tempTimer1;
    tempTimer1.begin("Generate Partition");
    if (useUniformPartition)
    {
        for (uint index = 0; index < currentNumOfPoints; index++)
        {
            int globalIndex = inputIndices[index];
            float3 point = pointCloud[globalIndex];
            int3 gridindex = mp.putIntoPartition(point);
            int linearIndex = mp.linearIndexOfPartition(gridindex);
            partitionIndices[linearIndex].emplace_back(globalIndex);
            partitionPoints[linearIndex].emplace_back(point);
        }

    }
    else
    {
        if (numOfCells == 1)
        {
            for (uint index = 0; index < currentNumOfPoints; index++)
            {
                int globalIndex = inputIndices[index];
                float3 point = pointCloud[globalIndex];
                int3 gridindex = mp.putIntoPartition(point);
                int linearIndex = mp.linearIndexOfPartition(gridindex);
                partitionIndices[linearIndex].emplace_back(globalIndex);
                partitionPoints[linearIndex].emplace_back(point);
            }
        }
        else if (numOfCells == 8)
        {
            std::vector<std::vector<uint>> levels;
            levels.resize(14);
            int levelsIndex = 0;
            for (uint level = 1; level < 2; level++)
            {
                uint cellNumberOffset = 1 << level;
                for (int i = levelsIndex; i < levelsIndex + cellNumberOffset; i++)
                {
                    levels[i].reserve(int(currentNumOfPoints / (1 << level)) + 32);
                }
                levelsIndex += cellNumberOffset;
            }
            levelsIndex = 0;
            mp.NonUniformPartitionParallel(pointCloud, inputIndices, levels[0], levels[1]);
            for (uint level = 1; level < 2; level++)
            {
                uint cellNumberOffset = 1 << level;
                int nextLevelIndex = levelsIndex + cellNumberOffset;
                for (int i = 0; i < cellNumberOffset; i++)
                {
                    mp.NonUniformPartitionParallel(pointCloud, levels[levelsIndex + i], levels[nextLevelIndex + 2 * i], levels[nextLevelIndex + 2 * i + 1]);
                }
                levelsIndex += cellNumberOffset;
            }
            for (int i = 0; i < 4; i++)
            {
                mp.NonUniformPartitionParallel(pointCloud, levels[2 + i], partitionIndices[2 * i], partitionIndices[2 * i + 1]);
            }
            tbb::parallel_for(0, (int)partitionIndices.size(), 1, [&](int i)
                {
                    for (int j = 0; j < partitionIndices.at(i).size(); j++)
                    {
                        int globalIndex = partitionIndices[i].at(j);
                        partitionPoints[i].emplace_back(pointCloud[globalIndex]);
                    }
                }
            );
        }
        else if (numOfCells == 64)
        {
            std::vector<std::vector<uint>> levels;
            levels.resize(62);
            int levelsIndex = 0;
            for (uint level = 1; level < 5; level++)
            {
                uint cellNumberOffset = 1 << level;
                for (int i = levelsIndex; i < levelsIndex + cellNumberOffset; i++)
                {
                    levels[i].reserve(int(currentNumOfPoints / (1 << level)) + 32);
                }
                levelsIndex += cellNumberOffset;
            }
            levelsIndex = 0;
            mp.NonUniformPartitionParallel(pointCloud, inputIndices, levels[0], levels[1]);
            for (uint level = 1; level < 5; level++)
            {
                uint cellNumberOffset = 1 << level;
                int nextLevelIndex = levelsIndex + cellNumberOffset;
                for (int i = 0; i < cellNumberOffset; i++)
                {
                    mp.NonUniformPartitionParallel(pointCloud, levels[levelsIndex + i], levels[nextLevelIndex + 2 * i], levels[nextLevelIndex + 2 * i + 1]);
                }
                levelsIndex += cellNumberOffset;
            }
            for (int i = 0; i < 32; i++)
            {
                auto NonUniformPartitionFunc = [i, &pointCloud, &levels, &partitionIndices, &mp]() {
                    mp.NonUniformPartition(pointCloud, levels[30 + i], partitionIndices[2 * i], partitionIndices[2 * i + 1]);
                };
                mJobSystem.dispatchTask(NonUniformPartitionFunc);
            }
            mJobSystem.finish();
            tbb::parallel_for(0, (int)partitionIndices.size(), 1, [&](int i)
                {
                    for (int j = 0; j < partitionIndices.at(i).size(); j++)
                    {
                        int globalIndex = partitionIndices[i].at(j);
                        partitionPoints[i].emplace_back(pointCloud[globalIndex]);
                    }
                }
            );

        }
    }
    tempTimer1.end();
    //initialization phase 2
    Timer tempTimer2;
    tempTimer2.begin("Elimination");
    std::vector<std::vector<float>> dMaxList;
    std::vector<std::vector<float>> reverseDMaxList;
    dMaxList.resize(numOfCells);
    reverseDMaxList.resize(numOfCells);
    for (int index = 0; index < numOfCells; index++)
    {
        dMaxList[index].resize(partitionPoints[index].size());
        reverseDMaxList[index].resize(partitionPoints[index].size());
        partitionWeights[index].resize(partitionPoints[index].size());
    }
    //generate dMaxList and reverseDMaxList
    for (int j = 0; j < numOfCells; j++)
    {
        uint count = partitionPoints[j].size();
        tbb::parallel_for(0u, count, 1u, [&](uint i)
            {
                int globalIndex = partitionIndices[j][i];
                dMaxList[j][i] = inputDmax[globalIndex];
                reverseDMaxList[j][i] = inputReverseDmax[globalIndex];
            });
    }

    tbb::parallel_for(0, (int)numOfCells, 1, [&](int j)
        {
            kdtrees[j].Build(partitionPoints[j].size(), partitionPoints[j].data());
        }
    );
    //generate weight
    auto weightFunction = [&](float d2, float dMax)
    {
        float d = sqrt(d2);
        const float alpha = 8.0f;
        return std::pow(1.0 - d / dMax, alpha);
    };
    for (int j = 0; j < numOfCells; j++)
    {
        uint count = partitionPoints[j].size();
        tbb::parallel_for(0u, count, 1u, [&](uint index)
            {
                float3 point = partitionPoints[j][index];
                float dMax = dMaxList[j][index];
                kdtrees[j].GetPoints(point, dMax, [&](uint i, float3 const& p, float d2, float&)
                    {
                        if (i >= count)
                        {
                            return;
                        }
                        if (i != index)
                        {
                            float weight = weightFunction(d2, dMax);
                            partitionWeights[j][index] += weight;
                            if (reverseDMaxList[j][i] < dMax)
                            {
                                reverseDMaxList[j][i] = dMax;
                            }
                        }
                    });
            });
    }
    //construct heaps
    std::vector<cy::Heap<float, uint>> maxHeaps;
    maxHeaps.resize(numOfCells);
    for (int index = 0; index < numOfCells; index++)
    {
        maxHeaps[index].SetDataPointer(partitionWeights[index].data(), partitionPoints[index].size());
        maxHeaps[index].Build();
    }

    for (int grid = 0; grid < static_cast<int>(numOfCells); grid++)
    {
        // sample elimination
        auto eliminationWithinAGrid = [grid, percent, &partitionPoints, &mp, &maxHeaps, &kdtrees, &reverseDMaxList, &dMaxList,
            &weightFunction, &partitionWeights]() {
            int gridTargetCount = static_cast<int>(partitionPoints[grid].size() / mp.getRatio());

            int gridInputCount = static_cast<int>(partitionPoints[grid].size());

            while (
                static_cast<int>(maxHeaps[grid].NumItemsInHeap()) > int((1.0f - percent) * gridInputCount + percent * gridTargetCount) - 1)
            {
                uint index = maxHeaps[grid].GetTopItemID();
                maxHeaps[grid].Pop();

                float3 point = partitionPoints[grid].at(index);
                uint maxCount = static_cast<uint>(partitionPoints[grid].size());
                kdtrees[grid].GetPoints(point, reverseDMaxList[grid][index], [&](uint i, float3 const& p, float d2, float&) {
                    if (i > maxCount)
                    {
                        return;
                    }
                    if (i != index)
                    {
                        float dMax = dMaxList[grid][i];
                        if (dMax * dMax > d2)
                        {
                            float weight = static_cast<float>(weightFunction(d2, dMax));
                            partitionWeights[grid].at(i) -= weight;
                            maxHeaps[grid].MoveItemDown(i);
                        }
                    }
                    });
            }
        };
        mJobSystem.dispatchTask(eliminationWithinAGrid);
    }
    mJobSystem.finish();
    //output results
    for (uint i = 0; i < numOfCells; i++)
    {
        uint num = maxHeaps[i].NumItemsInHeap();
        for (int j = 0; j < num; j++)
        {
            uint localIndex = maxHeaps[i].GetIDFromHeap(j);
            outputIndices.emplace_back(partitionIndices[i][localIndex]);
            if (outputPositions != nullptr && dMaxArray != nullptr)
            {
                outputPositions->emplace_back(partitionPoints[i][localIndex]);
                dMaxArray->emplace_back(dMaxList[i][localIndex]);
            }

        }
    }
    tempTimer2.end();
    tempTimer.end();

}


//for non-uniform grid partition
    // Input: Point cloud, indices  ****** Output: two point cloud, two index arrays
void Partition::NonUniformPartitionParallel(float3* pointCloud, std::vector<uint>& inputIndices, std::vector<uint>& ouputIndicesLeft, std::vector<uint>& ouputIndicesRight, bool printInfo)
{
    int pointCount = inputIndices.size();
    std::vector<float3> pointCloudInCell;
    pointCloudInCell.resize(pointCount);

    Timer tempTimer;
    if (printInfo)
        tempTimer.begin("NonUniformPartition Stage1");

    tbb::parallel_for(0, pointCount, 1, [&](int i)
        {
            pointCloudInCell.at(i) = (pointCloud[inputIndices.at(i)]);
        });

    std::vector<float> xValues(pointCount);
    std::vector<float> yValues(pointCount);
    std::vector<float> zValues(pointCount);

    tbb::parallel_for(0, pointCount, 1, [&](int i)
        {
            xValues.at(i) = pointCloudInCell[i].x;
        });
    tbb::parallel_for(0, pointCount, 1, [&](int i)
        {
            yValues.at(i) = pointCloudInCell[i].y;
        });
    tbb::parallel_for(0, pointCount, 1, [&](int i)
        {
            zValues.at(i) = pointCloudInCell[i].z;
        });

    if (printInfo)
        tempTimer.end();


    if (printInfo)
        tempTimer.begin("NonUniformPartition Stage2");

    tbb::parallel_sort(xValues.begin(), xValues.end(), std::less<float>());
    tbb::parallel_sort(yValues.begin(), yValues.end(), std::less<float>());
    tbb::parallel_sort(zValues.begin(), zValues.end(), std::less<float>());

    float xSize = xValues.at(xValues.size() - 1) - xValues.at(0);
    float ySize = yValues.at(yValues.size() - 1) - yValues.at(0);
    float zSize = zValues.at(zValues.size() - 1) - zValues.at(0);

    float xMin = xValues.at(0);
    float yMin = yValues.at(0);
    float zMin = zValues.at(0);


    int medianIndex = int(pointCount / 2);

    float xPlane = xValues.at(medianIndex);
    float yPlane = yValues.at(medianIndex);
    float zPlane = zValues.at(medianIndex);


    float xPercent = (xPlane - xMin) / std::max(xSize, 1e-3f * diagonalVector.x);
    float yPercent = (yPlane - yMin) / std::max(ySize, 1e-3f * diagonalVector.y);
    float zPercent = (zPlane - zMin) / std::max(zSize, 1e-3f * diagonalVector.z);

    float disToXPlane = std::abs(xPercent - 0.5f);
    float disToYPlane = std::abs(yPercent - 0.5f);
    float disToZPlane = std::abs(zPercent - 0.5f);

    if (printInfo)
        tempTimer.end();


    if (printInfo)
        tempTimer.begin("NonUniformPartition Stage3");
    // saparate with xPlane
    if (disToXPlane <= disToYPlane && disToXPlane <= disToZPlane)
    {
        for (int i = 0; i < pointCount; i++)
        {
            if (pointCloudInCell.at(i).x > xPlane)
            {
                ouputIndicesLeft.emplace_back(inputIndices[i]);
            }
            else
            {
                ouputIndicesRight.emplace_back(inputIndices[i]);
            }

        }

    }
    else
    {
        //saparate with yPlane
        if (disToYPlane <= disToZPlane)
        {
            for (int i = 0; i < pointCount; i++)
            {
                if (pointCloudInCell.at(i).y > yPlane)
                {
                    ouputIndicesLeft.emplace_back(inputIndices[i]);
                }
                else
                {
                    ouputIndicesRight.emplace_back(inputIndices[i]);
                }

            }

        }
        //saparate with zPlane
        else
        {
            for (int i = 0; i < pointCount; i++)
            {
                if (pointCloudInCell.at(i).z > zPlane)
                {
                    ouputIndicesLeft.emplace_back(inputIndices[i]);
                }
                else
                {
                    ouputIndicesRight.emplace_back(inputIndices[i]);
                }

            }

        }
    }

    if (printInfo)
        tempTimer.end();
}


