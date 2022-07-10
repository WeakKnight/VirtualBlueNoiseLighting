#pragma once

namespace Falcor
{
    class dlldecl DepthPrePass : public std::enable_shared_from_this<DepthPrePass>
    {
    public:
        using SharedPtr = std::shared_ptr<DepthPrePass>;
        static DepthPrePass::SharedPtr Create(Scene::SharedPtr scene);
        void Resize(uint32_t width, uint32_t height);
        void Execute(RenderContext* renderContext);

        inline Texture::SharedPtr GetDepthTexture() const
        {
            return m_Fbo->getDepthStencilTexture();
        }

    private:
        void Init();

        Scene::SharedPtr mpScene;
        Fbo::SharedPtr m_Fbo;
        GraphicsState::SharedPtr m_GraphicsState;
        GraphicsVars::SharedPtr m_GraphicsVars;
        RasterizerState::SharedPtr m_RasterizerState;
    };
}
