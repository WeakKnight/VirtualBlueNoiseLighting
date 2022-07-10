#pragma once
#include <memory>
#include "DepthPrePass.h"

namespace Falcor
{
    class dlldecl GBufferPass : public std::enable_shared_from_this<GBufferPass>
    {
    public:
        using SharedPtr = std::shared_ptr<GBufferPass>;
        static GBufferPass::SharedPtr Create(Scene::SharedPtr scene);

        void Resize(uint32_t width, uint32_t height);

        void Execute(RenderContext* renderContext);

        void SetShaderData(const ShaderVar& var) const;

        inline Texture::SharedPtr GetShadingNormalTexture() { return m_Fbo->getColorTexture(3); }

        inline Texture::SharedPtr GetMotionTexture() { return m_MotionTexture; }
        inline void SetMotionTexture(Texture::SharedPtr tex)
        {
            m_MotionTexture = tex;
        }

        inline Texture::SharedPtr GetMotionWTexture() { return m_MotionWTexture; }
        inline void SetMotionWTexture(Texture::SharedPtr tex)
        {
            m_MotionWTexture = tex;
        }

        inline Texture::SharedPtr GetNormalRoughnessMaterialIdTexture()
        {
            return m_NormalRoughnessMaterialIdTexture;
        }
        inline void SetNormalRoughnessMaterialIdTexture(Texture::SharedPtr tex) { m_NormalRoughnessMaterialIdTexture = tex; }

        inline Texture::SharedPtr GetDepthTexture() { return m_Fbo->getDepthStencilTexture(); }
        inline Texture::SharedPtr GetLinearNormalTexture() { return m_Fbo->getColorTexture(6); }
        inline Texture::SharedPtr GetPrevLinearNormalTexture() { return m_InternalPreviousLinearZAndNormalRT; }

    private:

        void InitPipelineResource();

        Scene::SharedPtr mpScene;

        Fbo::SharedPtr                  m_Fbo;
        Texture::SharedPtr              m_MotionTexture;
        Texture::SharedPtr              m_MotionWTexture;
        Texture::SharedPtr              m_HitInfoTexture;
        Texture::SharedPtr              m_InternalPreviousLinearZAndNormalRT;
        Texture::SharedPtr              m_NormalRoughnessMaterialIdTexture;

        /*
        For Rasterized GBuffer
        */
        GraphicsProgram::SharedPtr      m_Program = nullptr;
        GraphicsVars::SharedPtr         m_ProgramVars = nullptr;
        GraphicsState::SharedPtr        m_GraphicsState = nullptr;

        RasterizerState::SharedPtr      m_RasterizerState = nullptr;
        DepthStencilState::SharedPtr    m_DepthStencilState = nullptr;

        //FullScreenPass::SharedPtr       m_PackLinearZAndNormal;

        DepthPrePass::SharedPtr            m_DepthPass;
    };
}

