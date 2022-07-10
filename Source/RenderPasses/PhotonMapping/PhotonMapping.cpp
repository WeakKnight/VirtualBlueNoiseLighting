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
#include "PhotonMapping.h"
#include "Experimental/Scene/Lights/LightBVHSampler.h"
#include "RenderPasses/GBufferPass.h"
#include "cyCore.h"
#include "cyHeap.h"
#include "cyPointCloud.h"
#include <tbb/parallel_for.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/atomic.h>
#include <tbb/parallel_sort.h>

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

    const char kRadiusScaler[] = "RadiusScaler";
    const char kMaxPathIntensity[] = "MaxPathIntensity";
    const char kPhotonPathCount[] = "PhotonPathCount";
    const char kBounceNum[] = "BounceNum";
    const char kPhotonRadius[] = "PhotonRadius";
    const char kUseAdaptiveRadius[] = "UseAdaptiveRadius";
    const char kNeighborIndex[] = "NeighborIndex";
    const char kPhotonLiveness[] = "PhotonLiveness";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

static void regPythonApi(pybind11::module& m)
{
    pybind11::class_<PhotonMapping, RenderPass, PhotonMapping::SharedPtr> pass(m, "PhotonMapping");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("PhotonMapping", kDesc, PhotonMapping::create);
    ScriptBindings::registerBinding(regPythonApi);
}

PhotonMapping::SharedPtr PhotonMapping::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new PhotonMapping());
    for (const auto& [key, value] : dict)
    {
        if (key == kBounceNum)
        {
            pPass->mBounceNum = value;
        }
        else if (key == kPhotonPathCount)
        {
            pPass->mPhotonTraceParames.photonPathCount = value;
        }
        else if (key == kPhotonRadius)
        {
            pPass->mPhotonTraceParames.radius = value;
        }
        else if (key == kMaxPathIntensity)
        {
            pPass->mMaxPathIntensity = value;
        }
        else if (key == kRadiusScaler)
        {
            pPass->mRadiusScaler = value;
        }
        else if (key == kUseAdaptiveRadius)
        {
            pPass->mUseAdaptiveRadius = value;
        }
        else if (key == kNeighborIndex)
        {
            pPass->mNeighborIndex = value;
        }
        else if (key == kPhotonLiveness)
        {
            pPass->mPhotonLiveness = value;
        }
        else
        {
            logWarning("Unknown field '" + key + "' in a PhotonMapping dictionary");
        }
    }
    
    return pPass;
}

std::string PhotonMapping::getDesc()
{
    return kDesc;
}

Dictionary PhotonMapping::getScriptingDictionary()
{
    Dictionary d;
    d[kBounceNum] = mBounceNum;
    d[kPhotonPathCount] = mPhotonTraceParames.photonPathCount;
    d[kPhotonRadius] = mPhotonTraceParames.radius;
    d[kMaxPathIntensity] = mMaxPathIntensity;
    d[kRadiusScaler] = mRadiusScaler;
    d[kUseAdaptiveRadius] = mUseAdaptiveRadius;
    d[kNeighborIndex] = mNeighborIndex;
    d[kPhotonLiveness] = mPhotonLiveness;
    return d;
}

RenderPassReflection PhotonMapping::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "useless dummy Input");
    reflector.addOutput(kDummyOutput, "shading Output").bindFlags(ResourceBindFlags::RenderTarget | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource).format(ResourceFormat::RGBA32Float);
    return reflector;
}

void PhotonMapping::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
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

    mPhotonTraceParames.frameIndex = gpFramework->getGlobalClock().getFrame();

    if (mPhotonLiveness > 0 && (mPhotonTraceParames.frameIndex % mPhotonLiveness == 0))
    {
        pRenderContext->clearUAVCounter(mpPhotonPositions, 0);
        mPhotonGenerated = false;
    }

    if (!mPhotonGenerated)
    {
        mLocalTimer.startDurationFromCurrentTime();
        tracePhotons(pRenderContext, renderData);
        mPhotonGenerated = true;
        mLocalTimer.update();
        logInfo("[===Time Stats===]Photon Generation " + std::to_string(mLocalTimer.delta()) + "seconds");
    }

    shading(pRenderContext, renderData);

    if (mCaptureAfterCertainTime && mTimer.getDuration() >= mTimeForCapture)
    {
        renderData.getDictionary()["freezeOutput"] = true;
    }
    else
        renderData.getDictionary()["freezeOutput"] = false;
}

void PhotonMapping::renderUI(Gui::Widgets& widget)
{
    if (!mCaptureAfterCertainTime)
    {
        if (widget.button("Capture After Certain Time"))
        {
            mCaptureAfterCertainTime = true;
            mNeedClear = true;
            mTimer.startDurationFromCurrentTime();
        }
    }
    else
    {
        if (widget.button("Finish Capture"))
        {
            mCaptureAfterCertainTime = false;
            mNeedClear = true;
        }
    }

    widget.var("Render time for capture (ms)", mTimeForCapture, 1.f, 100000.f);

    widget.var("Photon Path Count", mPhotonTraceParames.photonPathCount);
    widget.var("Photon Radius", mPhotonTraceParames.radius);
    widget.text("Photon Count: " + std::to_string(mPhotonCount));
    mNeedClear |= widget.checkbox("Output Outgoing Radiance", mOutputOutgoingRadiance);
    mNeedClear |= widget.var("Max Path Intensity", mMaxPathIntensity);
    widget.var("Photon Liveness", mPhotonLiveness);

    if (auto statsGroup = widget.group("Stats"))
    {
        float memSizeInMB = 14llu * sizeof(float) * mPhotonCount / float(1024 * 1024);
        widget.text("Total Memory Size Is " + std::to_string(memSizeInMB) + "mb");
    }
}

void PhotonMapping::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpScene)
    {
        recompile();
    }

    if constexpr ((sizeof(Photon) != 48))
    {
        logError("Wrong Photon Size");
    }

    mCapacity = mPhotonTraceParames.photonPathCount * 18u;
    mpPhotonPositions = Buffer::createStructured(sizeof(float3), mCapacity);
    mpPhotonNormals = Buffer::createStructured(sizeof(uint), mCapacity);
    mpPhotonDirections = Buffer::createStructured(sizeof(uint), mCapacity);
    mpPhotonPowers = Buffer::createStructured(sizeof(uint), mCapacity);

    mpBoundingBoxBuffer = Buffer::createStructured(sizeof(PackedBoundingBox), mCapacity);
}


PhotonMapping::PhotonMapping()
{
    mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_TINY_UNIFORM);

    Program::Desc tracePhotonPassDesc;
    tracePhotonPassDesc.addShaderLibrary("RenderPasses/PhotonMapping/PhotonTracePass.cs.slang").csEntry("main").setShaderModel("6_5");
    mpPhotonTracePass = ComputePass::create(tracePhotonPassDesc, Program::DefineList(), false);

    Program::Desc shadingPassDesc;
    shadingPassDesc.addShaderLibrary("RenderPasses/PhotonMapping/ShadingPass.cs.slang").csEntry("main").setShaderModel("6_5");
    mpShadingPass = ComputePass::create(shadingPassDesc, Program::DefineList(), false);
}

bool PhotonMapping::prepareLights(RenderContext* pRenderContext)
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

        if (!mpEmissiveTable || is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightCollectionChanged))
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
            mpEmissiveTable = AliasTable::create(fluxList, rng);
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

        if (mpEmissiveTable)
        {
            mpEmissiveTable = nullptr;
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

void PhotonMapping::recompile()
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

    mpPhotonTracePass->getProgram()->addDefines(defines);
    mpPhotonTracePass->setVars(nullptr); // Trigger recompile

    mpShadingPass->getProgram()->addDefines(defines);
    mpShadingPass->setVars(nullptr); // Trigger recompile

    mNeedClear = true;
    mNeedRecompile = false;
}

void PhotonMapping::tracePhotons(RenderContext* pRenderContext, const RenderData& renderData)
{
    ShaderVar cb = mpPhotonTracePass["CB"];

    cb["frameIndex"] = mPhotonTraceParames.frameIndex;
    cb["photonPathCount"] = mPhotonTraceParames.photonPathCount;
    cb["radius"] = mPhotonTraceParames.radius;

    mpEmissiveTable->setShaderData(cb["gEmissiveTriTable"]);
    mpPhotonTracePass["gPhotonPositions"] = mpPhotonPositions;
    mpPhotonTracePass["gPhotonNormals"] = mpPhotonNormals;
    mpPhotonTracePass["gPhotonDirections"] = mpPhotonDirections;
    mpPhotonTracePass["gPhotonPowers"] = mpPhotonPowers;
    mpPhotonTracePass["gBoundingBoxBuffer"] = mpBoundingBoxBuffer;

    mpScene->setRaytracingShaderData(pRenderContext, mpPhotonTracePass->getRootVar());

    mpPhotonTracePass->execute(pRenderContext, uint3(mPhotonTraceParames.photonPathCount, 1, 1));

    // Update Photon Count
    {
        Buffer::SharedPtr counterReadBuffer = Buffer::create(sizeof(uint), ResourceBindFlags::None, Buffer::CpuAccess::Read);
        Buffer::SharedPtr counterBuffer = mpPhotonPositions->getUAVCounter();

        pRenderContext->copyBufferRegion(counterReadBuffer.get(), 0, counterBuffer.get(), 0, counterBuffer->getSize());
        pRenderContext->flush(true);

        uint* data = (uint*)counterReadBuffer->map(Buffer::MapType::Read);
        mPhotonCount = data[0];
        counterReadBuffer->unmap();

        if (mCapacity < mPhotonCount)
        {
            logError("Photon Light Capacity Is Not Enough, Should Be Bigger Than " + std::to_string(mPhotonCount));
        }
    }

    //// Read Position And Construct Adaptive Radius
    //{
    //    Buffer::SharedPtr positionReadBuffer = Buffer::create(mpPhotonPositions->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
    //    pRenderContext->copyBufferRegion(positionReadBuffer.get(), 0, mpPhotonPositions.get(), 0, mpPhotonPositions->getSize());
    //    pRenderContext->flush(true);
    //    float3* inputPositions = (float3*)positionReadBuffer->map(Buffer::MapType::Read);

    //    cy::PointCloud<float3, float, 3, uint> kdtree;
    //    kdtree.Build(mPhotonCount, inputPositions);
    //    float ratio = 1.0f;
    //    auto getPoissonDiskRadius = [&](float3 pos)
    //    {
    //        float result = 0.0f;
    //        if (mUseAdaptiveRadius)
    //        {
    //            uint actualCount;
    //            float actualSquaredRadius;
    //            kdtree.PointsSearchExtRadiusFirst(pos, 150, 0.15f, actualCount, actualSquaredRadius);
    //            float sampleDomainArea = ratio * cy::Pi<float>() * actualSquaredRadius / (float)actualCount;
    //            result = sqrt(sampleDomainArea / (2.0 * sqrt(3.0)));
    //        }
    //        else
    //        {
    //            const uint neighborCount = mNeighborIndex;
    //            std::vector<cy::PointCloud<float3, float, 3>::PointInfo> pointInfos(neighborCount);
    //            int actualCount = kdtree.GetPoints(pos, neighborCount, pointInfos.data());
    //            if (actualCount != neighborCount)
    //            {
    //                logError("Should Not Happen");
    //            }
    //            for (uint i = 0; i < neighborCount; i++)
    //            {
    //                float dis = sqrt(pointInfos[i].distanceSquared);
    //                if (result < dis)
    //                {
    //                    result = dis;
    //                }
    //            }
    //        }

    //        result = std::max(std::min(mPhotonTraceParames.radius, result), 1e-6f);
    //        return result;
    //    };
    //    std::vector<float> photonRadiusList(mPhotonCount);

    //    tbb::parallel_for(0u, mPhotonCount, 1u, [&](uint i)
    //    {
    //        photonRadiusList[i] = mRadiusScaler * getPoissonDiskRadius(inputPositions[i]);
    //    });
    //    
    //    std::vector<PackedBoundingBox> packedBoundingBoxes(mPhotonCount);
    //    tbb::parallel_for(0u, mPhotonCount, 1u, [&](uint i)
    //    {
    //        packedBoundingBoxes[i].minPoint = inputPositions[i] - photonRadiusList[i];
    //        packedBoundingBoxes[i].maxPoint = inputPositions[i] + photonRadiusList[i];
    //        packedBoundingBoxes[i].pad0 = 0u;
    //        packedBoundingBoxes[i].pad1 = 0u;
    //    });

    //    positionReadBuffer->unmap();

    //    mpBoundingBoxBuffer->setBlob(packedBoundingBoxes.data(), 0u, sizeof(PackedBoundingBox) * packedBoundingBoxes.size());
    //}

    // Build AS
    if (!mpAccelerationStructureBuilder)
    {
        mpAccelerationStructureBuilder = BoundingBoxAccelerationStructureBuilder::Create(mpBoundingBoxBuffer);
    }
    mpAccelerationStructureBuilder->BuildAS(pRenderContext, mPhotonCount, 1);
}

void PhotonMapping::shading(RenderContext* pRenderContext, const RenderData& renderData)
{
    GBufferPass::SharedPtr gBuffer = renderData.getDictionary()[kGBuffer];

    PROFILE("Sample Shading");
    auto cb = mpShadingPass["CB"];
    cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
    cb["gMaxPathIntensity"] = mMaxPathIntensity;
    cb["gOutputOutgoingRadiance"] = mOutputOutgoingRadiance;
    cb["gPhotonRadius"] = mPhotonTraceParames.radius;

    if (mpEmissiveSampler)
    {
        mpEmissiveSampler->setShaderData(cb["gEmissiveLightSampler"]);
    }
    if (mpEnvMapSampler)
    {
        mpEnvMapSampler->setShaderData(cb["gEnvMapSampler"]);
    }

    gBuffer->SetShaderData(cb["gGBuffer"]);

    mpScene->setRaytracingShaderData(pRenderContext, mpShadingPass->getRootVar());
    mpShadingPass["gShadingOutput"] = renderData[kDummyOutput]->asTexture();
    mpShadingPass["gPhotonPositions"] = mpPhotonPositions;
    mpShadingPass["gPhotonNormals"] = mpPhotonNormals;
    mpShadingPass["gPhotonDirections"] = mpPhotonDirections;
    mpShadingPass["gPhotonPowers"] = mpPhotonPowers;
    mpShadingPass["gBoundingBoxBuffer"] = mpBoundingBoxBuffer;

    mpAccelerationStructureBuilder->SetRaytracingShaderData(mpShadingPass->getRootVar(), "gPhotonAS", 1u);

    mpShadingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
}
