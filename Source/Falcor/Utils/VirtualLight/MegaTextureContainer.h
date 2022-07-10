#pragma once

namespace Falcor
{
    class dlldecl MegaTextureContainer
    {
    public:
        using SharedPtr = std::shared_ptr<MegaTextureContainer>;
        static SharedPtr create(uint32_t capacityHQ, uint perItemSizeHQ, uint32_t capacityLQ, uint perItemSizeLQ);
        void setShaderData(const ShaderVar& var) const;

        void blitToTexture(RenderContext* pRenderContext, uint vplCount, Buffer::SharedPtr typeBuffer, Buffer::SharedPtr indexBuffer);

        inline Buffer* GetDataBufferHQ() const
        {
            return mpDataBufferHQ.get();
        }

        inline void setDataBufferHQ(Buffer::SharedPtr newDataBuffer)
        {
            mpDataBufferHQ = newDataBuffer;
        }

        inline Texture::SharedPtr GetTextureHQ() const
        {
            return mpTexContainerHQ;
        }

        inline uint32_t GetPerItemSizeHQ() const
        {
            return mPerItemSizeHQ;
        }

        inline Buffer* GetDataBufferLQ() const
        {
            return mpDataBufferLQ.get();
        }

        inline void setDataBufferLQ(Buffer::SharedPtr newDataBuffer)
        {
            mpDataBufferLQ = newDataBuffer;
        }

        inline Texture::SharedPtr GetTextureLQ() const
        {
            return mpTexContainerLQ;
        }

        inline uint32_t GetPerItemSizeLQ() const
        {
            return mPerItemSizeLQ;
        }

    private:
        MegaTextureContainer(uint32_t capacityHQ = 100000, uint perItemSizeHQ = 16, uint32_t capacityLQ = 400000, uint perItemSizeLQ = 8);

        uint32_t mCapacityHQ = 100000u;
        uint32_t mPerItemSizeHQ = 16u;
        Buffer::SharedPtr mpDataBufferHQ;
        Texture::SharedPtr mpTexContainerHQ;

        uint32_t mCapacityLQ = 400000u;
        uint32_t mPerItemSizeLQ = 8u;
        Buffer::SharedPtr mpDataBufferLQ;
        Texture::SharedPtr mpTexContainerLQ;

        Sampler::SharedPtr mpLinearSampler;

        ComputePass::SharedPtr mpBlitPass;
    };
}
