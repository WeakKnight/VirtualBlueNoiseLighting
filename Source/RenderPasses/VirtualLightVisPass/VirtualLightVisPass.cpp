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
#include "VirtualLightVisPass.h"
#include "Utils/VirtualLight/VirtualLightContainer.h"
#include "RenderPasses/GBufferPass.h"
#include "Utils/VirtualLight/MegaTextureContainer.h"

namespace
{
    const char kDummy[] = "dummy";
    const char kDesc[] = "Insert pass description here";
    const char kRadius[] = "radius";
    const char kVisMode[] = "visMode";
    const char kVisType[] = "visType";

    const char kOutputChannel[] = "output";

    const char kEnable[] = "enable";

    const std::string kDicInitialVirtualLights = "initialVirtualLights";
    const std::string kDicSampleEliminatedVirtualLights = "sampleEliminatedVirtualLights";
    const std::string kDicCurVirtualLights = "curVirtualLights";
    const std::string kDicSpecularRadianceContainer = "specularRadianceContainer";

    const std::string kGBuffer = "GBuffer";

    const std::string kDicTileDim = "tileDim";
    const std::string kDicTileSize = "tileSize";
    const std::string kDicTileSampleNum = "tileSampleNum";
    const std::string kDicTileVirtualLightContainer = "tileVirtualLightContainer";
    const std::string kDicMaxBSDFSearchCount = "MaxBSDFSearchCount";
}

struct PackedBoundingBox
{
    float3 minPoint;
    float3 maxPoint;
    float Pad0;
    float Pad1;
};

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("VirtualLightVisPass", kDesc, VirtualLightVisPass::create);
}

VirtualLightVisPass::SharedPtr VirtualLightVisPass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new VirtualLightVisPass);
    for (const auto& [key, value] : dict)
    {
        if (key == kRadius)
        {
            pPass->mRadius = value;
        }
        else if (key == kVisMode)
        {
            pPass->mVisMode = value;
        }
        else if (key == kVisType)
        {
            pPass->mVisType = value;
        }
        else if (key == kEnable)
        {
            pPass->mEnable = value;
        }
    }

    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    Program::Desc desc;
    desc.addShaderLibrary("RenderPasses/VirtualLightVisPass/VirtualLightVis.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpComputePass = ComputePass::create(desc, Program::DefineList(), false);

    return pPass;
}

std::string VirtualLightVisPass::getDesc()
{
    return kDesc;
}

Dictionary VirtualLightVisPass::getScriptingDictionary()
{
    Dictionary d;
    d[kRadius] = mRadius;
    d[kVisMode] = mVisMode;
    d[kVisType] = mVisType;
    d[kEnable] = mEnable;
    return d;
}

RenderPassReflection VirtualLightVisPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput(kDummy, "");
    reflector.addOutput(kOutputChannel, "output texture").bindFlags(ResourceBindFlags::RenderTarget | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource).format(ResourceFormat::RGBA32Float);
    return reflector;
}

void VirtualLightVisPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (!mEnable)
    {
        return;
    }

    if (mpScene == nullptr)
    {
        return;
    }

    GBufferPass::SharedPtr gBuffer = renderData.getDictionary()[kGBuffer];
    if (gBuffer == nullptr)
    {
        return;
    }

    if (!renderData.getDictionary().keyExists(kDicSampleEliminatedVirtualLights))
    {
        return;
    }

    if (!renderData.getDictionary().keyExists(kDicSpecularRadianceContainer))
    {
        return;
    }

    uint32_t maxBSDFSearchCount = 16;
    if (renderData.getDictionary().keyExists(kDicMaxBSDFSearchCount))
    {
        maxBSDFSearchCount = renderData.getDictionary()[kDicMaxBSDFSearchCount];
    }

    Texture::SharedPtr pDst = renderData[kOutputChannel]->asTexture();

    if (!mpTotalSum || !mpSampleCount)
    {
        mpTotalSum = Texture::create2D(pDst->getWidth(), pDst->getHeight(), ResourceFormat::RGBA32Float, 1u, 1u, nullptr, ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource);
        mpSampleCount = Texture::create2D(pDst->getWidth(), pDst->getHeight(), ResourceFormat::R32Uint, 1u, 1u, nullptr, ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource);
    }

    if (mpScene)
    {
        auto sceneUpdates = mpScene->getUpdates();
        if ((sceneUpdates & ~Scene::UpdateFlags::CameraPropertiesChanged) != Scene::UpdateFlags::None)
        {
            pRenderContext->clearUAV(mpTotalSum->getUAV().get(), float4(0.0f));
            pRenderContext->clearUAV(mpSampleCount->getUAV().get(), uint4(0u));
        }
        if (is_set(sceneUpdates, Scene::UpdateFlags::CameraPropertiesChanged))
        {
            auto excluded = Camera::Changes::Jitter | Camera::Changes::History;
            auto cameraChanges = mpScene->getCamera()->getChanges();
            if ((cameraChanges & ~excluded) != Camera::Changes::None)
            {
                pRenderContext->clearUAV(mpTotalSum->getUAV().get(), float4(0.0f));
                pRenderContext->clearUAV(mpSampleCount->getUAV().get(), uint4(0u));
            }
        }
        if (mNeedClear)
        {
            pRenderContext->clearUAV(mpTotalSum->getUAV().get(), float4(0.0f));
            pRenderContext->clearUAV(mpSampleCount->getUAV().get(), uint4(0u));
            mNeedClear = false;
        }
    }

    VirtualLightContainer::SharedPtr initialVirtualLights = renderData.getDictionary()[kDicInitialVirtualLights];
    VirtualLightContainer::SharedPtr sampleEliminatedVirtualLights = renderData.getDictionary()[kDicSampleEliminatedVirtualLights];
    VirtualLightContainer::SharedPtr curVirtualLights = renderData.getDictionary()[kDicCurVirtualLights];
    MegaTextureContainer::SharedPtr specRadianceContainer = renderData.getDictionary()[kDicSpecularRadianceContainer];

    if (mpMegaTextureItem == nullptr)
    {
        mpMegaTextureItem = Texture::create2D(specRadianceContainer->GetPerItemSizeHQ(), specRadianceContainer->GetPerItemSizeHQ(), ResourceFormat::RGBA8Unorm, 1, 1, nullptr, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess);
    }

    VirtualLightContainer::SharedPtr seletedVirtualLights;
    switch (mVisMode)
    {
    case 0:
        seletedVirtualLights = curVirtualLights;
        break;
    case 1:
        seletedVirtualLights = initialVirtualLights;
        break;
    case 2:
        seletedVirtualLights = sampleEliminatedVirtualLights;
        break;
    }

    if (seletedVirtualLights == nullptr
        || specRadianceContainer == nullptr)
    {
        debugBreak(); // should not be nullptr here
        return;
    }

    Buffer::SharedPtr tileVirtualLightContainer = renderData.getDictionary()[kDicTileVirtualLightContainer];
    uint2 tileDims = renderData.getDictionary()[kDicTileDim];
    uint tileSampleNum = renderData.getDictionary()[kDicTileSampleNum];
    uint tileSize = renderData.getDictionary()[kDicTileSize];
    if (tileVirtualLightContainer == nullptr)
    {
        debugBreak();
        return; // should not be nullptr here
    }

    ShaderVar cb = mpComputePass["CB"];
    cb["gViewportDims"] = uint2(pDst->getWidth(), pDst->getHeight());
    cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
    cb["gRadius"] = mRadius;
    cb["gVisType"] = mVisType;
    seletedVirtualLights->setShaderData(cb["gVirtualLightContainer"]);
    specRadianceContainer->setShaderData(cb["gSpecRadianceContainer"]);
    cb["gTileSampleNum"] = tileSampleNum;
    cb["gTileDims"] = tileDims;
    cb["gTileSize"] = tileSize;
    cb["gMegaTextureBaseIndex"] = uint2(mMegaTextureBaseIndexX, mMegaTextureBaseIndexY);
    cb["gMegaTexturePitch"] = mMegaTexturePitch;
    cb["gMegaTextureExposure"] = mMegaTextureExposure;
    gBuffer->SetShaderData(cb["gGBuffer"]);
    cb["gExposure"] = mExposure;
    cb["gTonemapped"] = mTonemapped;
    cb["gSelectedPixel"] = mSelectedPixel;
    cb["gNeedUpdate"] = mNeedUpdate;
    cb["gMaxBSDFSearchCount"] = maxBSDFSearchCount;

    if (mpMegaTextureItem)
    {
        mpComputePass["gMegaTextureItem"] = mpMegaTextureItem;
    }
    if (mNeedUpdate)
    {
        mNeedUpdate = false;
    }

    mpComputePass["gTileVirtualLightContainer"] = tileVirtualLightContainer;
    mpScene->setRaytracingShaderData(pRenderContext, mpComputePass->getRootVar());

    mpComputePass["gOutput"] = pDst;
    mpComputePass["gSampleNum"] = mpSampleCount;
    mpComputePass["gTotalSum"] = mpTotalSum;

    mpComputePass->execute(pRenderContext, uint3(pDst->getWidth(), pDst->getHeight(), 1));
}

static std::string sOutputDir;

void VirtualLightVisPass::renderUI(Gui::Widgets& widget)
{
    widget.checkbox("Enable", mEnable);

    mNeedClear |= widget.var("Vis Radius", mRadius, 0.0f, 1.0f);
    Gui::DropdownList visModes;
    visModes.push_back({ 0, "Current Samples" });
    visModes.push_back({ 1, "Initial Samples" });
    visModes.push_back({ 2, "Sample Eliminated Samples" });
    mNeedClear |= widget.dropdown("Vis Mode", visModes, mVisMode);

    Gui::DropdownList visTypes;
    visTypes.push_back({ 0, "Uniform Solid Circle" });
    visTypes.push_back({ 1, "Adaptive Solid Circle" });
    visTypes.push_back({ 2, "Ring" });
    visTypes.push_back({ 3, "Clamping Safety Map" });
    visTypes.push_back({ 4, "Adaptive Diffuse Solid Circle" });
    visTypes.push_back({ 5, "Tile Virtual Light Visualization" });
    visTypes.push_back({ 6, "Vis Mega Texture" });
    visTypes.push_back({ 7, "Vis Mega Texture 3D" });
    visTypes.push_back({ 8, "Vis Splatted Outgoing Radiance" });

    mNeedClear |= widget.dropdown("Vis Type", visTypes, mVisType);

    if (mVisType == 6)
    {
        mNeedClear |= widget.var("Mega Texture Base X", mMegaTextureBaseIndexX);
        mNeedClear |= widget.var("Mega Texture Base Y", mMegaTextureBaseIndexY);
        mNeedClear |= widget.var("Mega Texture Pitch", mMegaTexturePitch);
    }

    mNeedClear |= widget.var("Mega Texture Exposure", mMegaTextureExposure);

    widget.checkbox("Tonemapped", mTonemapped);
    widget.var("Exposure", mExposure);

    if (mpMegaTextureItem)
    {
        widget.image("MegaTexItem", mpMegaTextureItem, float2(128.0f, 128.0f));
        
        widget.textbox("OutputDir", sOutputDir, Gui::TextFlags::FitWindow);
        if (widget.button("capture"))
        {
            mpMegaTextureItem->captureToFile(0u, 0u, sOutputDir + "\\megaTex.png", Bitmap::FileFormat::PngFile);
        }
    }
}

void VirtualLightVisPass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
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
    }
}

bool VirtualLightVisPass::onMouseEvent(const MouseEvent& mouseEvent)
{
    if (mouseEvent.type == MouseEvent::Type::RightButtonDown)
    {
        float2 cursorPos = mouseEvent.pos * float2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        mSelectedPixel = (uint2)glm::clamp(cursorPos, float2(0.f), float2(gpFramework->getTargetFbo()->getWidth() - 1, gpFramework->getTargetFbo()->getHeight() - 1));
        mNeedUpdate = true;
    }

    return false;
}
