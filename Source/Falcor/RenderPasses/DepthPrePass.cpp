#include "stdafx.h"
#include "DepthPrePass.h"

namespace Falcor
{
    DepthPrePass::SharedPtr DepthPrePass::Create(Scene::SharedPtr scene)
    {
        DepthPrePass::SharedPtr result = SharedPtr(new DepthPrePass());

        result->mpScene = scene;
        result->Resize(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        result->Init();

        return result;
    }

    void DepthPrePass::Resize(uint32_t width, uint32_t height)
    {
        Fbo::Desc desc;
        desc.setDepthStencilTarget(ResourceFormat::D32Float);

        m_Fbo = Fbo::create2D(width, height, desc);
    }

    void DepthPrePass::Execute(RenderContext* renderContext)
    {
        Camera::SharedPtr camera = mpScene->getCamera();

        m_GraphicsState->setFbo(m_Fbo);

        renderContext->clearDsv(m_Fbo->getDepthStencilView().get(), 1, 0);
        mpScene->rasterize(renderContext, m_GraphicsState.get(), m_GraphicsVars.get(), Scene::RenderFlags::UserRasterizerState);
    }

    void DepthPrePass::Init()
    {
        RasterizerState::Desc rasterStateDesc;
        rasterStateDesc.setCullMode(RasterizerState::CullMode::None);
        m_RasterizerState = RasterizerState::create(rasterStateDesc);

        Program::Desc desc;
        desc.addShaderLibrary("Falcor/RenderPasses/DepthPass.ps.slang").psEntry("main");

        Program::DefineList defines;
        defines.add(mpScene->getSceneDefines());
        GraphicsProgram::SharedPtr pProgram = GraphicsProgram::create(desc, defines);

        m_GraphicsState = GraphicsState::create();
        m_GraphicsState->setRasterizerState(m_RasterizerState);
        m_GraphicsState->setProgram(pProgram);

        m_GraphicsVars = GraphicsVars::create(pProgram->getReflector());
    }
}
