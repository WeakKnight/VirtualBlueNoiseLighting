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
#include "VirtualLightEstimatePass.h"
#include "Utils/VirtualLight/VirtualLightContainer.h"
#include "Utils/VirtualLight/MegaTextureConfig.slangh"
#include "pcg32.h"

namespace
{
    const char kDummy[] = "dummy";
    const char kDummyInput[] = "input";
    const char kDummyOutput[] = "output";
    const char kDesc[] = "Insert pass description here";

    const char kPhotonPathCount[] = "Photon Path Count";
    const char kPerFramePhotonPathCount[] = "Per Frame Path Count";

    const char kTextureItemSize[] = "Texture Item Size";
    const char kMegaTextureCapacity[] = "MegaTexture Capacity";

    const char kTextureItemSizeLQ[] = "Texture Item Size LQ";
    const char kMegaTextureCapacityLQ[] = "MegaTexture Capacity LQ";

    const char kConvertIncomingToOutgoing[] = "Convert Incoming To Outgoing";
    const char kFormerVersion[] = "Former Version";
    const char kSeperateBRDF[] = "Seperate BRDF";
    const char kPerFrameProcessingCount[] = "PerFrameProcessingCount";

    const char kBounceNum[] = "BounceNum";

    const char kTreatHighlyGlossyAsDelta[] = "TreatHighlyGlossyAsDelta";
    const char kHighlyGlossyThreshold[] = "HighlyGlossyThreshold";

    const std::string kDicInitialVirtualLights = "initialVirtualLights";
    const std::string kDicSampleEliminatedVirtualLights = "sampleEliminatedVirtualLights";
    const std::string kDicCurVirtualLights = "curVirtualLights";
    const std::string kDicSpecularRadianceContainer = "specularRadianceContainer";
    const std::string kDicFluxBuffer = "fluxBuffer";
    const std::string kDicRadianceReady = "radianceReady";
    const std::string kDicFluxTable = "fluxTable";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

static void regPythonApi(pybind11::module& m)
{
    pybind11::class_<VirtualLightEstimatePass, RenderPass, VirtualLightEstimatePass::SharedPtr> pass(m, "VirtualLightEstimatePass");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary & lib)
{
    lib.registerClass("VirtualLightEstimatePass", kDesc, VirtualLightEstimatePass::create);
    ScriptBindings::registerBinding(regPythonApi);
}

VirtualLightEstimatePass::SharedPtr VirtualLightEstimatePass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new VirtualLightEstimatePass);
    for (const auto& [key, value] : dict)
    {
        if (key == kPhotonPathCount)
        {
            pPass->mPhotonPathCount = value;
        }
        else if (key == kPerFramePhotonPathCount)
        {
            pPass->mPerFramePhotonPathCount = value;
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
        else if (key == kConvertIncomingToOutgoing)
        {
            pPass->mConvertIncomingToOutgoing = value;
        }
        else if (key == kFormerVersion)
        {
            pPass->mFormerVersion = value;
        }
        else if (key == kSeperateBRDF)
        {
            pPass->mSeperateBRDF = value;
        }
        else if (key == kPerFrameProcessingCount)
        {
            pPass->mPerFrameProcessingCount = value;
        }
        else if (key == kBounceNum)
        {
            pPass->mBounceNum = value;
        }
        else if (key == kTreatHighlyGlossyAsDelta)
        {
            pPass->mTreatHighlyGlossyAsDelta = value;
        }
        else if (key == kHighlyGlossyThreshold)
        {
            pPass->mHighlyGlossyThreshold = value;
        }
    }
    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    Program::Desc solidAnglePassDesc;
    solidAnglePassDesc.addShaderLibrary("RenderPasses/VirtualLightEstimatePass/ComputeSolidAngleLUT.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpLUTPass = ComputePass::create(solidAnglePassDesc, Program::DefineList(), false);

    Program::Desc estimatePassDesc;
    estimatePassDesc.addShaderLibrary("RenderPasses/VirtualLightEstimatePass/VirtualLightEstimate.cs.slang").csEntry("main").setShaderModel("6_5").setCompilerFlags(Shader::CompilerFlags::FloatingPointModePrecise);
    pPass->mpEstimatePass = ComputePass::create(estimatePassDesc, Program::DefineList(), false);

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

std::string VirtualLightEstimatePass::getDesc()
{
    return kDesc;
}

Dictionary VirtualLightEstimatePass::getScriptingDictionary()
{
    Dictionary d;
    d[kPhotonPathCount] = mPhotonPathCount;
    d[kPerFramePhotonPathCount] = mPerFramePhotonPathCount;
    d[kTextureItemSize] = mTextureItemSize;
    d[kMegaTextureCapacity] = mMegaTextureCapacity;
    d[kTextureItemSizeLQ] = mTextureItemSizeLQ;
    d[kMegaTextureCapacityLQ] = mMegaTextureCapacityLQ;
    d[kConvertIncomingToOutgoing] = mConvertIncomingToOutgoing;
    d[kFormerVersion] = mFormerVersion;
    d[kSeperateBRDF] = mSeperateBRDF;
    d[kBounceNum] = mBounceNum;
    d[kTreatHighlyGlossyAsDelta] = mTreatHighlyGlossyAsDelta;
    d[kHighlyGlossyThreshold] = mHighlyGlossyThreshold;

    return d;
}

RenderPassReflection VirtualLightEstimatePass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "useless dummy Input");
    reflector.addOutput(kDummyOutput, "useless dummy Output");
    return reflector;
}

void VirtualLightEstimatePass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
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

    VirtualLightContainer::SharedPtr sampleEliminatedVirtualLights = renderData.getDictionary()[kDicSampleEliminatedVirtualLights];
    if (sampleEliminatedVirtualLights == nullptr)
    {
        debugBreak(); // should not be nullptr here
    }

    /*
    Construct Alias Table For Emissive Triangles
    */
    if (gpState->currentState != gpState->prevState)
    {
        auto lightCollection = mpScene->getLightCollection(pRenderContext);
        auto emissiveTris = lightCollection->getMeshLightTriangles();
        std::vector<float> weights(emissiveTris.size());
        for (size_t i = 0; i < emissiveTris.size(); i++)
        {
            weights[i] = emissiveTris[i].flux;
        }
        std::mt19937 rng;
        mpEmissiveTriTable = AliasTable::create(weights, rng);
        //mTimer.update();
    }

    if (gpState->currentState != gpState->prevState)
    {
        mpFluxBuffer = Buffer::createStructured(sizeof(float), sampleEliminatedVirtualLights->getCount());
        mpFluxBuffer->setName("EstimatePass: Flux Buffer");
    }

    if (gpState->currentState != gpState->prevState)
    {
        mpSpecularRadianceContainer = MegaTextureContainer::create(mMegaTextureCapacity, mTextureItemSize, mMegaTextureCapacityLQ, mTextureItemSizeLQ);
        if (mMegaTextureCapacity < sampleEliminatedVirtualLights->getTexCountHQ())
        {
            logError("mega texture capacity is not enough, should be more than " + std::to_string(sampleEliminatedVirtualLights->getTexCountHQ()));
        }

        mpSolidAngleLUT = Buffer::create((size_t)mTextureItemSize * (size_t)mTextureItemSize * sizeof(uint));
        mpSolidAngleLUT->setName("SolidAngleLUT");

#ifdef _MULTI_RES_MEGA_TEXTURE
        mpSolidAngleLUTLQ = Buffer::create((size_t)mTextureItemSizeLQ * (size_t)mTextureItemSizeLQ * sizeof(uint));
        mpSolidAngleLUTLQ->setName("SolidAngleLUT");
#endif

        const uint sampleCount = 100000;

        {
            auto cb = mpLUTPass["CB"];
            cb["gItemSize"] = mTextureItemSize;
            cb["gSampleCount"] = sampleCount;
            mpLUTPass["gLUTBuffer"] = mpSolidAngleLUT;
            mpLUTPass->execute(pRenderContext, sampleCount, 1, 1);
            pRenderContext->flush(true);
        }

#if _MULTI_RES_MEGA_TEXTURE
        {
            auto cb = mpLUTPass["CB"];
            cb["gItemSize"] = mTextureItemSizeLQ;
            cb["gSampleCount"] = sampleCount;
            mpLUTPass["gLUTBuffer"] = mpSolidAngleLUTLQ;
            mpLUTPass->execute(pRenderContext, sampleCount, 1, 1);
            pRenderContext->flush(true);
        }
#endif
    }

    if (gpState->currentState != gpState->prevState)
    {
        mpFluxTable = nullptr;
    }

    renderData.getDictionary()[kDicFluxBuffer] = mpFluxBuffer;
    renderData.getDictionary()[kDicSpecularRadianceContainer] = mpSpecularRadianceContainer;

    if (mCurrentPhotonPathCount >= mPhotonPathCount)
    {
        if (mNeedFinalize)
        {
            if (mConvertIncomingToOutgoing)
            {
                mStageTimer.begin("[===Time Stats===]Radiance Convert");
                const uint perFrameProcessingCount = mPerFrameProcessingCount;
                PROFILE("Convert Processing");
                size_t dataBufferSize = mpSpecularRadianceContainer->GetDataBufferHQ()->getSize();
                auto convertedBuffer = Buffer::create(dataBufferSize);
                convertedBuffer->setName("MegaTexture: Converted Outgoing Radiance");

                uint texCount = sampleEliminatedVirtualLights->getTexCountHQ();
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

                    sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
                    mpSpecularRadianceContainer->setShaderData(cb["gIncidentContainer"]);

                    mpConvertPass["gConvertedBuffer"] = convertedBuffer;
                    mpConvertPass["gFluxBuffer"] = mpFluxBuffer;
                    mpConvertPass->execute(pRenderContext, uint3(perFrameProcessingCount, mpSpecularRadianceContainer->GetPerItemSizeHQ(), mpSpecularRadianceContainer->GetPerItemSizeHQ()));

                    processedCount += perFrameProcessingCount;
                    if (processedCount >= texCount)
                    {
                        std::cout << "Incident Radiance Processed" << processedCount << std::endl;
                        pRenderContext->flush(true);
                    }
                    else if ((processedCount / perFrameProcessingCount) % 256 == 0)
                    {
                        std::cout << "Incident Radiance Processed" << processedCount << std::endl;
                        pRenderContext->flush();
                    }
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

                    sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
                    mpSpecularRadianceContainer->setShaderData(cb["gIncidentContainer"]);

                    mpDeltaConvertPass["gConvertedBuffer"] = convertedBuffer;
                    mpDeltaConvertPass["gFluxBuffer"] = mpFluxBuffer;
                    mpDeltaConvertPass["gSolidAngleLUTBuffer"] = mpSolidAngleLUT;
                    mpDeltaConvertPass->execute(pRenderContext, uint3(perFrameProcessingCount, 1, 1));

                    processedCount += perFrameProcessingCount;
                    if (processedCount >= texCount)
                    {
                        std::cout << "Delta Incident Radiance Processed" << processedCount << std::endl;
                        pRenderContext->flush(true);
                    }
                    else if ((processedCount / perFrameProcessingCount) % 256 == 0)
                    {
                        std::cout << "Delta Incident Radiance Processed" << processedCount << std::endl;
                        pRenderContext->flush();
                    }
                }
                mpSpecularRadianceContainer->setDataBufferHQ(convertedBuffer);

#if _MULTI_RES_MEGA_TEXTURE
                dataBufferSize = mpSpecularRadianceContainer->GetDataBufferLQ()->getSize();
                auto convertedBufferLQ = Buffer::create(dataBufferSize);
                convertedBufferLQ->setName("MegaTexture: Converted Outgoing Radiance LQ");
                texCount = sampleEliminatedVirtualLights->getTexCountLQ();
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

                    sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
                    mpSpecularRadianceContainer->setShaderData(cb["gIncidentContainer"]);

                    mpConvertPassLQ["gConvertedBuffer"] = convertedBufferLQ;
                    mpConvertPassLQ["gFluxBuffer"] = mpFluxBuffer;
                    mpConvertPassLQ->execute(pRenderContext, uint3(perFrameProcessingCount, mpSpecularRadianceContainer->GetPerItemSizeLQ(), mpSpecularRadianceContainer->GetPerItemSizeLQ()));

                    processedCount += perFrameProcessingCount;
                    if (processedCount >= texCount)
                    {
                        std::cout << "Incident Radiance LQ Processed" << processedCount << std::endl;
                        pRenderContext->flush(true);
                    }
                    else if ((processedCount / perFrameProcessingCount) % 256 == 0)
                    {
                        std::cout << "Incident Radiance LQ Processed" << processedCount << std::endl;
                        pRenderContext->flush();
                    }
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

                    sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLights"]);
                    mpSpecularRadianceContainer->setShaderData(cb["gIncidentContainer"]);

                    mpDeltaConvertPass["gConvertedBuffer"] = convertedBufferLQ;
                    mpDeltaConvertPass["gFluxBuffer"] = mpFluxBuffer;
                    mpDeltaConvertPass["gSolidAngleLUTBuffer"] = mpSolidAngleLUTLQ;
                    mpDeltaConvertPass->execute(pRenderContext, uint3(perFrameProcessingCount, 1, 1));

                    processedCount += perFrameProcessingCount;
                    if (processedCount >= texCount)
                    {
                        std::cout << "Delta Incident Radiance Processed" << processedCount << std::endl;
                        pRenderContext->flush(true);
                    }
                    else if ((processedCount / perFrameProcessingCount) % 256 == 0)
                    {
                        std::cout << "Delta Incident Radiance Processed" << processedCount << std::endl;
                        pRenderContext->flush();
                    }
                }
                mpSpecularRadianceContainer->setDataBufferLQ(convertedBufferLQ);
#endif
                mStageTimer.end();
                gpState->time += mStageTimer.delta();
            }

            /*
            Trigger Update
           */
            mpScene->getCamera()->setFocalDistance(100.0f);

            if (!ENABLE_SOFTWARE_BILINEAR)
            {
                mpSpecularRadianceContainer->blitToTexture(pRenderContext, sampleEliminatedVirtualLights->getCount(), sampleEliminatedVirtualLights->getTypeBuffer(), sampleEliminatedVirtualLights->getIndexBuffer());
            }

            if (mpFluxTable == nullptr)
            {
                Buffer::SharedPtr readBuffer = Buffer::create(mpFluxBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read, nullptr);
                pRenderContext->copyBufferRegion(readBuffer.get(), 0, mpFluxBuffer.get(), 0, mpFluxBuffer->getSize());
                pRenderContext->flush(true);

                uint lightCount = sampleEliminatedVirtualLights->getCount();
                std::vector<float> weights(lightCount);

                float* fluxData = (float*)readBuffer->map(Buffer::MapType::Read);
                for (size_t i = 0; i < sampleEliminatedVirtualLights->getCount(); i++)
                {
                    weights[i] = fluxData[i];
                }
                readBuffer->unmap();
                std::mt19937 rng;
                mpFluxTable = AliasTable::create(weights, rng);
            }

            renderData.getDictionary()[kDicFluxTable] = mpFluxTable;

            gpDevice->flushAndSync();

           /* mTimer.update();
            logInfo("Radiance Estimation Takes " + std::to_string(mTimer.delta()) + "seconds");*/
            mNeedFinalize = false;
        }

        renderData.getDictionary()[kDicRadianceReady] = true;
        gpState->setState(3);
        return;
    }

    /*
    Estimate
    */
    float invPathCount = 1.0 / mPhotonPathCount;
    ShaderVar cb = mpEstimatePass["CB"];
    cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
    cb["gInvPathCount"] = invPathCount;
    sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLightContainer"]);
    mpSpecularRadianceContainer->setShaderData(cb["gSpecRadianceContainer"]);
    mpEmissiveTriTable->setShaderData(cb["gEmissiveTriTable"]);
    mpScene->setRaytracingShaderData(pRenderContext, mpEstimatePass->getRootVar());
    mpEstimatePass["gFluxBuffer"] = mpFluxBuffer;

    const uint seedCapacity = mPhotonPathCount / 2000;
    pcg32 sg = pcg32(gpFramework->getGlobalClock().getFrame());
    std::vector<uint> seeds(seedCapacity, 0);
    for (uint i = 0; i < seedCapacity; i++)
    {
        seeds[i] = sg.nextUInt();
    }
    Buffer::SharedPtr seedBuffer = Buffer::createStructured(sizeof(uint), seedCapacity, ResourceBindFlags::ShaderResource, Buffer::CpuAccess::None, seeds.data());
    mpEstimatePass["gSeedBuffer"] = seedBuffer;
    cb["gSeedCapacity"] = seedCapacity;

    mStageTimer.begin("[===Time Stats===]Photon Tracing");

    uint iterNum = 0;
    while (mCurrentPhotonPathCount < mPhotonPathCount)
    {
        cb["gCurrentPathIndex"] = mCurrentPhotonPathCount;
        mpEstimatePass->execute(pRenderContext, uint3(mPerFramePhotonPathCount, 1, 1));
        mCurrentPhotonPathCount += mPerFramePhotonPathCount;
        //pRenderContext->flush(true);
        iterNum++;
        if (iterNum >= 1024)
        {
            iterNum = 0;
            pRenderContext->flush(true);
        }
    }

    pRenderContext->flush(true);

    mStageTimer.end();
    gpState->time += mStageTimer.delta();

    renderData.getDictionary()[kDicRadianceReady] = false;

    sampleEliminatedVirtualLights->setFluxBuffer(mpFluxBuffer);

    gpState->setState(2);
}

void VirtualLightEstimatePass::renderUI(Gui::Widgets& widget)
{
    widget.var("Photon Path Count", mPhotonPathCount);
    widget.var("Bounce Num", mBounceNum, 1u, 256u);
    widget.var("Per Frame Path Count", mPerFramePhotonPathCount);
    widget.text("Current Path Count: " + std::to_string(mCurrentPhotonPathCount));
    widget.var("Mega Texture Item Size", mTextureItemSize, 8u, 64u);
    widget.var("MegaTexture Capacity", mMegaTextureCapacity, 10000u, 1000000u);
    widget.var("Mega Texture Item Size LQ", mTextureItemSizeLQ, 8u, 64u);
    widget.var("MegaTexture Capacity LQ", mMegaTextureCapacityLQ, 10000u, 1000000u);
    widget.checkbox("Incident To Outgoing", mConvertIncomingToOutgoing);

    widget.checkbox("Treat Highly Glossy As Delta", mTreatHighlyGlossyAsDelta);
    widget.var("Highly Glossy Thredshold", mHighlyGlossyThreshold, 0.0f, 1.0f);
    //widget.checkbox("Use Former Version", mFormerVersion);
    //widget.checkbox("Seperate BRDF", mSeperateBRDF);
}

void VirtualLightEstimatePass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpScene)
    {
        Shader::DefineList defines = mpScene->getSceneDefines();
        defines.add(mpSampleGenerator->getDefines());
        defines.add("_MS_DISABLE_ALPHA_TEST");
        defines.add("_DEFAULT_ALPHA_TEST");
        defines.add("_PATH_COUNT", std::to_string(mPhotonPathCount));
        defines.add("_PER_FRAME_PATH_COUNT", std::to_string(mPerFramePhotonPathCount));
        defines.add("_WRITE_MEGA_TEXTURE");
        if (mFormerVersion)
        {
            defines.add("_ESTIMATION_FORMER_VERSION");
        }
        if (mSeperateBRDF)
        {
            defines.add("_SEPERATE_BRDF");
        }
        defines.add("_BOUNCE_NUM", std::to_string(mBounceNum));
        defines.add("_TREAT_HIGHLY_GLOSSY_AS_DELTA", mTreatHighlyGlossyAsDelta ? "1" : "0");
        defines.add("_HIGHLY_GLOSSY_THRESHOLD", std::to_string(mHighlyGlossyThreshold));

        mpLUTPass->getProgram()->addDefines(defines);
        mpLUTPass->setVars(nullptr); // Trigger recompile

        mpEstimatePass->getProgram()->addDefines(defines);
        mpEstimatePass->getProgram()->addDefine("_CONVERT_INCOMING_TO_OUTGOING", mConvertIncomingToOutgoing ? "1" : "0");
        mpEstimatePass->setVars(nullptr); // Trigger recompile

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
    }
}


