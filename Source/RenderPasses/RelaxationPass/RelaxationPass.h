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
#pragma once
#include "Falcor.h"
#include "FalcorExperimental.h"
#include "Utils/Sampling/SampleGenerator.h"
#include "Utils/VirtualLight/VirtualLightContainer.h"
#include "Utils/VirtualLight/MegaTextureContainer.h"
#include "Utils/Sampling/AliasTable.h"
#include "Timer.h"

using namespace Falcor;

class RelaxationPass : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<RelaxationPass>;

    /** Create a new render pass object.
        \param[in] pRenderContext The render context.
        \param[in] dict Dictionary of serialized parameters.
        \return A new object, or an exception is thrown if creation failed.
    */
    static SharedPtr create(RenderContext* pRenderContext = nullptr, const Dictionary& dict = {});

    virtual std::string getDesc() override;
    virtual Dictionary getScriptingDictionary() override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pContext, const CompileData& compileData) override {}
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { return false; }
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override { return false; }

private:
    RelaxationPass() = default;

    void enrich(RenderContext* pRenderContext, VirtualLightContainer* photonContainer);
    void gotoNextState(RenderContext* pRenderContext, const RenderData& renderData);

    Scene::SharedPtr mpScene;
    ComputePass::SharedPtr  mpBlitPass;
    ComputePass::SharedPtr  mpEnrichPass;
    VirtualLightContainer::SharedPtr mpVirtualLightContainer;
    SampleGenerator::SharedPtr mpSampleGenerator;

    MegaTextureContainer::SharedPtr mpSpecularRadianceContainer;

    AliasTable::SharedPtr mpFluxTable;
    Buffer::SharedPtr mpFluxBuffer;

    float mRelaxationFactor = 1.0f;

    float mRadiusScalerForASBuilding = 1.5f;
    bool  mUseDMaxForASBuilding = false;

    uint mTextureItemSize = 8;
    uint mMegaTextureCapacity = 100000;

    uint mTextureItemSizeLQ = 8;
    uint mMegaTextureCapacityLQ = 100000;

    uint mIterationCount = 20u;
    std::vector<float> mPrsArr;
    uint mTargetCount = 1000u;
    float mRadius = 0.03f;

    int mKNNRadiusScaler = 5;
    bool mUseStandardEnrich = true;

    /* Our Radiance Estimation*/
    uint mPhotonPathCount = 1650000;
    uint mPerFramePhotonPathCount = 5000;
    uint mCurrentPhotonPathCount = 0;

    uint mPerFrameProcessingCount = 128u;

    ComputePass::SharedPtr mpConvertPass;
    ComputePass::SharedPtr mpDeltaConvertPass;
    ComputePass::SharedPtr mpConvertPassLQ;
    ComputePass::SharedPtr mpDeltaConvertPassLQ;
    bool mTreatHighlyGlossyAsDelta = true;
    float mHighlyGlossyThreshold = 0.16f;

    AliasTable::SharedPtr mpEmissiveTriTable;

    ComputePass::SharedPtr mpLUTPass;
    Buffer::SharedPtr mpSolidAngleLUT;
    Buffer::SharedPtr mpSolidAngleLUTLQ;

    bool mNeedFinalize = true;

    Timer mLocalTimer;
    CpuTimer mTimer;
    float mMinPrs = 0.0f;
};
