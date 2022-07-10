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
#include "Utils/Sampling/AliasTable.h"
#include "Types.slang"
#include "Utils/VirtualLight/BoundingBoxAccelerationStructureBuilder.h"

using namespace Falcor;

class PhotonMapping : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<PhotonMapping>;

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

    virtual void onHotReload(HotReloadFlags reloaded) override
    {
        recompile();
    }

private:
    PhotonMapping();

    bool prepareLights(RenderContext* pRenderContext);
    void recompile();
    void tracePhotons(RenderContext* pRenderContext, const RenderData& renderData);
    void shading(RenderContext* pRenderContext, const RenderData& renderData);

    AliasTable::SharedPtr mpEmissiveTable;
    EmissiveLightSampler::SharedPtr mpEmissiveSampler;
    EnvMapSampler::SharedPtr mpEnvMapSampler;

    Buffer::SharedPtr mpPhotonPositions;
    Buffer::SharedPtr mpPhotonNormals;
    Buffer::SharedPtr mpPhotonDirections;
    Buffer::SharedPtr mpPhotonPowers;

    Buffer::SharedPtr mpBoundingBoxBuffer;
    BoundingBoxAccelerationStructureBuilder::SharedPtr mpAccelerationStructureBuilder;

    bool mNeedRecompile = true;
    bool mNeedClear = false;
    bool mPhotonGenerated = false;
    bool mOutputOutgoingRadiance = false;

    //// capture after certain render time
    bool mCaptureAfterCertainTime = false;
    CpuTimer mTimer;
    CpuTimer mLocalTimer;

    float mTimeForCapture = 1000.f;

    float mMaxPathIntensity = 0.0f;

    uint32_t mPhotonCount = 0u;
    uint32_t mBounceNum = 256u;
    uint32_t mCapacity = 0u;

    PhotonTraceParams mPhotonTraceParames;

    Scene::SharedPtr mpScene;
    ComputePass::SharedPtr  mpPhotonTracePass;
    ComputePass::SharedPtr  mpShadingPass;
    SampleGenerator::SharedPtr mpSampleGenerator;

    float mRadiusScaler = 1.0f;
    bool mUseAdaptiveRadius = false;
    uint mNeighborIndex = 20;

    int mPhotonLiveness = 20;
};