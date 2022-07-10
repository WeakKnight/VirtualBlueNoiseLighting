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
#include "LinearBinaryTree.h"

using namespace Falcor;

class ReSTIRPass : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<ReSTIRPass>;

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
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { return mpPixelDebug->onMouseEvent(mouseEvent); }
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override { return false; }

private:
    ReSTIRPass() = default;

    void addDefineToAllPasses(Shader::DefineList defineList);
    void removeDefineFromAllPasses(Shader::DefineList defineList);

    bool prepareLights(RenderContext* pRenderContext);
    void recompile();

    EmissiveLightSampler::SharedPtr mpEmissiveSampler;
    EnvMapSampler::SharedPtr mpEnvMapSampler;
    bool mNeedRecompile = true;
    bool mNeedClear = false;

    Scene::SharedPtr mpScene;
    ComputePass::SharedPtr mpPrepareEmissiveTrianglesPass;
    ComputePass::SharedPtr mpPrepareLightsPass;
    ComputePass::SharedPtr mpPreSamplingPass;
    ComputePass::SharedPtr mpInitialSamplingPass;
    ComputePass::SharedPtr mpSpatialResamplingPass;
    ComputePass::SharedPtr mpTemporalResamplingPass;
    ComputePass::SharedPtr mpShadingPass;
    ComputePass::SharedPtr mpSpecularGatheringPass;

    float mVSLRadiusFactor = 1.0f;

    Buffer::SharedPtr mpNeighborOffsetBuffer;

    SampleGenerator::SharedPtr mpSampleGenerator;
    Buffer::SharedPtr mpReservoirBuffer;
    Buffer::SharedPtr mpRISBuffer;
    PixelDebug::SharedPtr               mpPixelDebug;                    ///< Utility class for pixel debugging (print in shaders).
    Texture::SharedPtr mpShadingOutput;
    Texture::SharedPtr mpRawShadingOutput;

    LightTree::SharedPtr mLightTree;

    uint mLastFrameOutputReservoir = 0;
    uint mCurrentFrameOutputReservoir = 0;

    uint mShadingMode = 3;
    uint mVisibilityMode = 0;
    bool mEnableDirectLighting = true;
    bool mEnableVSLEvaluation = false;
    bool mEnableSpatialResampling = false;
    bool mEnableTemporalResampling = false;
    bool mEnablePresampling = true;
    bool mEnableSpecularGathering = false;
    bool mEnableBiasCompensation = false;

    bool mMISForVSL = false;
    bool mUseSelfGenSamples = false;
    uint mNumInitialSamples = 4;
    uint mTileSearchRadius = 0;
    uint mNumTileVPLSamples = 1;

    uint mMaxHistoryLength = 2u;

    bool mUseMaxHistoryForSubframeMode = true;
    uint mNumSubFrames = 20u;

    bool mSubFrameMode = false;

    bool mPointSampling = false;
    uint mNumMinConeSamples = 1;
    uint mNumConeSamples = 16;
    uint mNumMinFinalConeSamples = 1;
    uint mNumFinalConeSamples = 100;

    bool mPairwiseMIS = true;
    bool mBiasCorrectionForReuse = true;

    bool mEmissiveTrianglesNeedUpdate = true;

    float mUsage = 1.0f;

    bool mOptionChanged = false;

    bool mRequestRebuildTree = true;
    bool mUseBalancedTree = false;

    bool mUniformSampling = false;

    bool mImproveCorner = false;

    uint mUnbalancedTreeSeed = 123;
    uint mbalancedTreeQuantLevel = 16;
};
