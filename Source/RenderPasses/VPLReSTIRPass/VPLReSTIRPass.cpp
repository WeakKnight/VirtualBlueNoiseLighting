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
#include "VPLReSTIRPass.h"
#include "RenderPasses/GBufferPass.h"
#include "Utils/VirtualLight/VirtualLightContainer.h"

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
    const char kDesc[] = "VPLReSTIRPass";
    const std::string kDicInitialVirtualLights = "initialVirtualLights";
    const std::string kDicSampleEliminatedVirtualLights = "sampleEliminatedVirtualLights";
    const std::string kDicCurVirtualLights = "curVirtualLights";
    const std::string kGBuffer = "GBuffer";
    const std::string kEmissiveTriangleFluxTable = "emissiveTriangleFluxTable";
    const std::string kReservoirBuffer = "reservoirBuffer";
    const std::string kDicPhotons = "photons";

    const char kKNNRadiusNeighbour[] = "KNNRadiusNeighbour";
    const char kEnableDirectLighting[] = "enableDirectLighting";
    const char kEnableTemporalResampling[] = "enableTemporalResampling";
    const char kEnableSpatialResampling[] = "enableSpatialResampling";
    const char kShadingMode[] = "shadingMode";
    const char kEnableVSL[] = "enableVSL";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

static void regPythonApi(pybind11::module& m)
{
    pybind11::class_<VPLReSTIRPass, RenderPass, VPLReSTIRPass::SharedPtr> pass(m, "VPLReSTIRPass");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("VPLReSTIRPass", kDesc, VPLReSTIRPass::create);
    ScriptBindings::registerBinding(regPythonApi);
}

VPLReSTIRPass::SharedPtr VPLReSTIRPass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new VPLReSTIRPass);
    for (const auto& [key, value] : dict)
    {
        if (key == kEnableDirectLighting)
        {
            pPass->mEnableDirectLighting = value;
        }
        else if (key == kEnableSpatialResampling)
        {
            pPass->mEnableSpatialResampling = value;
        }
        else if (key == kEnableTemporalResampling)
        {
            pPass->mEnableTemporalResampling = value;
        }
        else if (key == kShadingMode)
        {
            pPass->mShadingMode = value;
        }
        else if (key == kKNNRadiusNeighbour)
        {
            pPass->mKNNRadiusNeighbour = value;
        }
        else if (key == kEnableVSL)
        {
            pPass->mEnableVSL = value;
        }
    }

    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    Program::Desc radiusWritePassDesc;
    radiusWritePassDesc.addShaderLibrary("RenderPasses/VPLReSTIRPass/RadiusWritePass.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpRadiusWritePass = ComputePass::create(radiusWritePassDesc, Program::DefineList(), false);

    Program::Desc initialSamplingPassDesc;
    initialSamplingPassDesc.addShaderLibrary("RenderPasses/VPLReSTIRPass/VPLReSTIRInitialSampling.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpInitialSamplingPass = ComputePass::create(initialSamplingPassDesc, Program::DefineList(), false);

    Program::Desc temporalResamplingPassDesc;
    temporalResamplingPassDesc.addShaderLibrary("RenderPasses/VPLReSTIRPass/VPLReSTIRTemporalResampling.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpTemporalResamplingPass = ComputePass::create(temporalResamplingPassDesc, Program::DefineList(), false);

    Program::Desc spatialResamplingPassDesc;
    spatialResamplingPassDesc.addShaderLibrary("RenderPasses/VPLReSTIRPass/VPLReSTIRSpatialResampling.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpSpatialResamplingPass = ComputePass::create(spatialResamplingPassDesc, Program::DefineList(), false);

    Program::Desc restirShadingPassDesc;
    restirShadingPassDesc.addShaderLibrary("RenderPasses/VPLReSTIRPass/VPLReSTIRShadingPass.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpShadingPass = ComputePass::create(restirShadingPassDesc, Program::DefineList(), false);

    return pPass;
}

std::string VPLReSTIRPass::getDesc()
{
    return kDesc;
}

Dictionary VPLReSTIRPass::getScriptingDictionary()
{
    Dictionary d;
    d[kEnableDirectLighting] = mEnableDirectLighting;
    d[kEnableSpatialResampling] = mEnableSpatialResampling;
    d[kEnableTemporalResampling] = mEnableTemporalResampling;
    d[kShadingMode] = mShadingMode;
    d[kKNNRadiusNeighbour] = mKNNRadiusNeighbour;
    d[kEnableVSL] = mEnableVSL;
    return d;
}

RenderPassReflection VPLReSTIRPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "useless dummy Input");
    reflector.addOutput(kDummyOutput, "shading Output").bindFlags(ResourceBindFlags::RenderTarget | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource).format(ResourceFormat::RGBA32Float);
    return reflector;
}

void FillNeighborOffsetBuffer(uint8_t* buffer)
{
    int R = 250;
    const float phi2 = 1.0f / 1.3247179572447f;
    uint32_t num = 0;
    float u = 0.5f;
    float v = 0.5f;
    while (num < 8192 * 2) {
        u += phi2;
        v += phi2 * phi2;
        if (u >= 1.0f) u -= 1.0f;
        if (v >= 1.0f) v -= 1.0f;

        float rSq = (u - 0.5f) * (u - 0.5f) + (v - 0.5f) * (v - 0.5f);
        if (rSq > 0.25f)
            continue;

        buffer[num++] = int8_t((u - 0.5f) * R);
        buffer[num++] = int8_t((v - 0.5f) * R);
    }
}

void VPLReSTIRPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    // auto& pTexture = renderData["src"]->asTexture();
    if (mpScene == nullptr)
    {
        return;
    }

    GBufferPass::SharedPtr gBuffer = renderData.getDictionary()[kGBuffer];
    if (gBuffer == nullptr)
    {
        return;
    }

    if (gpState->currentState != 0)
    {
        return;
    }

    if (mOptionChanged)
    {
        {
            Shader::DefineList defines;
            defines.add("_VSL_EVALUATION");
            if (mEnableVSL)
            {
                addDefineToAllPasses(defines);
            }
            else
            {
                removeDefineFromAllPasses(defines);
            }
        }

        /*
        Trigger Update
        */
        mpScene->getCamera()->setFocalDistance(100.0f + std::sinf(gpFramework->getGlobalClock().getFrame()));
        mOptionChanged = false;
    }

    if (mpNeighborOffsetBuffer == nullptr)
    {
        std::vector<uint8_t> offsets;
        offsets.resize(8192 * 2);
        FillNeighborOffsetBuffer(offsets.data());
        mpNeighborOffsetBuffer = Buffer::createTyped(ResourceFormat::RG8Snorm, 8192, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, offsets.data());
        mpNeighborOffsetBuffer->setName("ReSTIR: Neighbor Offset Buffer");
    }

    VirtualLightContainer::SharedPtr virtualLights = renderData.getDictionary()[kDicPhotons];
    if (mpFluxTable == nullptr)
    {
        Buffer::SharedPtr radianceBuffer = virtualLights->getIncidentRadianceBuffer();
        Buffer::SharedPtr readBuffer = Buffer::create(radianceBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read, nullptr);
        pRenderContext->copyBufferRegion(readBuffer.get(), 0, radianceBuffer.get(), 0, radianceBuffer->getSize());
        pRenderContext->flush(true);
        uint lightCount = virtualLights->getCount();
        std::vector<float> weights(lightCount);

        float3* colorData = (float3*)readBuffer->map(Buffer::MapType::Read);
        for (size_t i = 0; i < lightCount; i++)
        {
            float3 col = colorData[i];
            weights[i] = (col.x + col.y + col.z);
        }
        readBuffer->unmap();
        std::mt19937 rng;
        mpFluxTable = AliasTable::create(weights, rng);
    }
    AliasTable::SharedPtr emissiveTriangleFluxTable = renderData.getDictionary()[kEmissiveTriangleFluxTable];

    uint32_t renderWidth = gpFramework->getTargetFbo()->getWidth();
    uint32_t renderHeight = gpFramework->getTargetFbo()->getHeight();
    uint32_t renderWidthBlocks = (renderWidth + 16 - 1) / 16;
    uint32_t renderHeightBlocks = (renderHeight + 16 - 1) / 16;
    uint reservoirBlockRowPitch = renderWidthBlocks * (16 * 16);
    uint reservoirArrayPitch = reservoirBlockRowPitch * renderHeightBlocks;

    if (gpState->currentState != gpState->prevState)
    {
        /*
        存两个array layer
        */
        const uint32_t reservoirLayers = 2;
        mpReservoirBuffer = Buffer::createStructured(mpShadingPass["gReservoirs"], reservoirArrayPitch * reservoirLayers);
        mpReservoirBuffer->setName("ReSTIR: Reservoir Buffer");

        /*
        Compute VSL Radius
        */
        std::vector<float> radiuses;
        uint lightCount = virtualLights->getCount();
        radiuses.resize(lightCount);

        Buffer::SharedPtr positionBuffer = virtualLights->getPositionBuffer();
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
        virtualLights->setShaderData(cb["gVirtualLights"]);
        mpRadiusWritePass["gDMaxs"] = dMaxBuffer;
        mpRadiusWritePass->execute(pRenderContext, lightCount, 1, 1);
        pRenderContext->flush(true);
        positionReadBuffer->unmap();
    }

    mLastFrameOutputReservoir = mCurrentFrameOutputReservoir;

    uint32_t initialOutputBufferIndex = !mLastFrameOutputReservoir;
    uint32_t temporalInputBufferIndex = mLastFrameOutputReservoir;
    uint32_t temporalOutputBufferIndex = initialOutputBufferIndex;
    uint32_t spatialInputBufferIndex = temporalOutputBufferIndex;
    uint32_t spatialOutputBufferIndex = !spatialInputBufferIndex;
    uint32_t shadeInputBufferIndex = mEnableSpatialResampling ? spatialOutputBufferIndex : temporalOutputBufferIndex;

    mCurrentFrameOutputReservoir = shadeInputBufferIndex;

    /*
    Initial Sampling Dispatch
    */
    {
        PROFILE("Initial Sampling");
        auto cb = mpInitialSamplingPass["CB"];
        cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
        cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
        cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
        cb["gOutputBufferIndex"] = initialOutputBufferIndex;

        virtualLights->setShaderData(cb["gVirtualLights"]);
        gBuffer->SetShaderData(cb["gGBuffer"]);
        mpFluxTable->setShaderData(cb["gFluxTable"]);

        mpScene->setRaytracingShaderData(pRenderContext, mpInitialSamplingPass->getRootVar());
        mpInitialSamplingPass["gReservoirs"] = mpReservoirBuffer;

        mpInitialSamplingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    /*
    Temporal Resampling Dispatch
    */
    if (mEnableTemporalResampling)
    {
        PROFILE("Temporal Resampling");
        auto cb = mpTemporalResamplingPass["CB"];
        cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
        cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
        cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
        cb["gInputBufferIndex"] = initialOutputBufferIndex;
        cb["gHistoryBufferIndex"] = temporalInputBufferIndex;
        cb["gOutputBufferIndex"] = temporalOutputBufferIndex;

        virtualLights->setShaderData(cb["gVirtualLights"]);
        gBuffer->SetShaderData(cb["gGBuffer"]);

        mpScene->setRaytracingShaderData(pRenderContext, mpTemporalResamplingPass->getRootVar());
        mpTemporalResamplingPass["gReservoirs"] = mpReservoirBuffer;

        mpTemporalResamplingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    /*
    Spatial Resampling Dispatch
    */
    if (mEnableSpatialResampling)
    {
        PROFILE("Spatial Resampling");
        auto cb = mpSpatialResamplingPass["CB"];
        cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
        cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
        cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
        cb["gInputBufferIndex"] = spatialInputBufferIndex;
        cb["gOutputBufferIndex"] = spatialOutputBufferIndex;

        virtualLights->setShaderData(cb["gVirtualLights"]);
        gBuffer->SetShaderData(cb["gGBuffer"]);

        mpScene->setRaytracingShaderData(pRenderContext, mpSpatialResamplingPass->getRootVar());
        mpSpatialResamplingPass["gReservoirs"] = mpReservoirBuffer;
        mpSpatialResamplingPass["gNeighborOffsetBuffer"] = mpNeighborOffsetBuffer;

        mpSpatialResamplingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    /*
    Shading Dispatch
    */
    {
        PROFILE("Sample Shading");
        auto cb = mpShadingPass["CB"];
        cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
        cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
        cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
        cb["gInputBufferIndex"] = shadeInputBufferIndex;
        cb["gShadingMode"] = mShadingMode;
        cb["gEnableDirectLighting"] = mEnableDirectLighting;

        emissiveTriangleFluxTable->setShaderData(cb["gEmissiveTriTable"]);
        virtualLights->setShaderData(cb["gVirtualLights"]);
        gBuffer->SetShaderData(cb["gGBuffer"]);
        mpFluxTable->setShaderData(cb["gFluxTable"]);

        mpScene->setRaytracingShaderData(pRenderContext, mpShadingPass->getRootVar());
        mpShadingPass["gReservoirs"] = mpReservoirBuffer;

        mpShadingOutput = renderData[kDummyOutput]->asTexture();

        mpShadingPass["gShadingOutput"] = mpShadingOutput;
        mpShadingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    gpState->setState(0);
}

void VPLReSTIRPass::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;
    Gui::DropdownList shadingModeList;
    shadingModeList.push_back({ 0, "Groundtruth" });
    shadingModeList.push_back({ 1, "ReSTIR" });
    //widget.dropdown("Shading Mode", shadingModeList, mShadingMode);
    widget.checkbox("Direct Lighting", mEnableDirectLighting);
    widget.checkbox("Temporal Resampling", mEnableTemporalResampling);
    widget.checkbox("Spatial Resampling", mEnableSpatialResampling);

    widget.var("KNN Radius Neighbour", mKNNRadiusNeighbour, 2u, 20u);

    dirty |= widget.checkbox("VSL Evaluation", mEnableVSL);

    if (dirty)
    {
        mOptionChanged = true;
    }
}

void VPLReSTIRPass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpScene)
    {
        Shader::DefineList defines = mpScene->getSceneDefines();
        defines.add(mpSampleGenerator->getDefines());
        defines.add("_MS_DISABLE_ALPHA_TEST");
        defines.add("_DEFAULT_ALPHA_TEST");
        if (mEnableVSL)
        {
            defines.add("_VSL_EVALUATION");
        }

        addDefineToAllPasses(defines);
    }
}

void VPLReSTIRPass::addDefineToAllPasses(Shader::DefineList defines)
{
    mpRadiusWritePass->getProgram()->addDefines(defines);
    mpRadiusWritePass->setVars(nullptr); // Trigger recompile

    mpInitialSamplingPass->getProgram()->addDefines(defines);
    mpInitialSamplingPass->setVars(nullptr);

    mpTemporalResamplingPass->getProgram()->addDefines(defines);
    mpTemporalResamplingPass->setVars(nullptr);

    mpSpatialResamplingPass->getProgram()->addDefines(defines);
    mpSpatialResamplingPass->setVars(nullptr);

    mpShadingPass->getProgram()->addDefines(defines);
    mpShadingPass->setVars(nullptr);
}

void VPLReSTIRPass::removeDefineFromAllPasses(Shader::DefineList defines)
{
    mpRadiusWritePass->getProgram()->removeDefines(defines);
    mpRadiusWritePass->setVars(nullptr); // Trigger recompile

    mpInitialSamplingPass->getProgram()->removeDefines(defines);
    mpInitialSamplingPass->setVars(nullptr);

    mpTemporalResamplingPass->getProgram()->removeDefines(defines);
    mpTemporalResamplingPass->setVars(nullptr);

    mpSpatialResamplingPass->getProgram()->removeDefines(defines);
    mpSpatialResamplingPass->setVars(nullptr);

    mpShadingPass->getProgram()->removeDefines(defines);
    mpShadingPass->setVars(nullptr);
}


