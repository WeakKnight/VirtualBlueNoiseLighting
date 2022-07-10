#include "stdafx.h"
#include "MegaTextureContainer.h"

namespace Falcor
{
    MegaTextureContainer::SharedPtr MegaTextureContainer::create(uint32_t capacityHQ, uint perItemSizeHQ, uint32_t capacityLQ, uint perItemSizeLQ)
    {
        return SharedPtr(new MegaTextureContainer(capacityHQ, perItemSizeHQ, capacityLQ, perItemSizeLQ));
    }

    void MegaTextureContainer::setShaderData(const ShaderVar& var) const
    {
        var["perItemSizeHQ"] = mPerItemSizeHQ;
        if (mpDataBufferHQ != nullptr)
        {
            var["dataBufferHQ"] = mpDataBufferHQ;
        }
        if (mpTexContainerHQ != nullptr)
        {
            var["containerHQ"] = mpTexContainerHQ;
        }

        var["perItemSizeLQ"] = mPerItemSizeLQ;
        if (mpDataBufferLQ != nullptr)
        {
            var["dataBufferLQ"] = mpDataBufferLQ;
        }
        if (mpTexContainerLQ != nullptr)
        {
            var["containerLQ"] = mpTexContainerLQ;
        }

        var["linearSampler"] = mpLinearSampler;
    }

    void MegaTextureContainer::blitToTexture(RenderContext* pRenderContext, uint vplCount, Buffer::SharedPtr typeBuffer, Buffer::SharedPtr indexBuffer)
    {
        /*
        * Blit HQ
        */
        {
            const uint layerNumHQ = mCapacityHQ / 10000u;
            const uint itemSizeAfterDilationHQ = mPerItemSizeHQ + 2;
            if (mpTexContainerHQ == nullptr)
            {
                mpTexContainerHQ = Texture::create2D(100 * itemSizeAfterDilationHQ, 100 * itemSizeAfterDilationHQ, ResourceFormat::RGBA32Float, layerNumHQ, 1, nullptr, ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource);
                mpTexContainerHQ->setName("MegaTexture HQ");
            }

            const uint layerNumLQ = mCapacityLQ / 10000u;
            const uint itemSizeAfterDilationLQ = mPerItemSizeLQ + 2;
            if (mpTexContainerLQ == nullptr)
            {
                mpTexContainerLQ = Texture::create2D(100 * itemSizeAfterDilationLQ, 100 * itemSizeAfterDilationLQ, ResourceFormat::RGBA32Float, layerNumLQ, 1, nullptr, ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource);
                mpTexContainerLQ->setName("MegaTexture LQ");
            }

            auto cb = mpBlitPass["CB"];
            cb["gItemSizeHQ"] = mPerItemSizeHQ;
            cb["gItemSizeLQ"] = mPerItemSizeLQ;
            cb["gCount"] = vplCount;

            mpBlitPass["gTypeBuffer"] = typeBuffer;
            mpBlitPass["gIndexBuffer"] = indexBuffer;

            mpBlitPass["gSourceHQ"] = mpDataBufferHQ;
            mpBlitPass["gContainerHQ"] = mpTexContainerHQ;

            mpBlitPass["gSourceLQ"] = mpDataBufferLQ;
            mpBlitPass["gContainerLQ"] = mpTexContainerLQ;
        }

        pRenderContext->resourceBarrier(mpTexContainerHQ.get(), Resource::State::ShaderResource);
        pRenderContext->resourceBarrier(mpTexContainerLQ.get(), Resource::State::ShaderResource);
        /*
        Todo Clean Data
        */
        //mpDataBuffer = nullptr;
    }
    
    MegaTextureContainer::MegaTextureContainer(uint32_t capacityHQ, uint perItemSizeHQ, uint32_t capacityLQ, uint perItemSizeLQ):
        mCapacityHQ(capacityHQ),
        mPerItemSizeHQ(perItemSizeHQ),
        mCapacityLQ(capacityLQ),
        mPerItemSizeLQ(perItemSizeLQ)
    {
        /* RGB Float Stride = 12 */
        mpDataBufferHQ = Buffer::create(static_cast<size_t>(12) * static_cast<size_t>(mPerItemSizeHQ) * static_cast<size_t>(mPerItemSizeHQ) * static_cast<size_t>(mCapacityHQ));
        mpDataBufferHQ->setName("MegaTextureContainer: Data Buffer HQ");

        mpDataBufferLQ = Buffer::create(static_cast<size_t>(12) * static_cast<size_t>(mPerItemSizeLQ) * static_cast<size_t>(mPerItemSizeLQ) * static_cast<size_t>(mCapacityLQ));
        mpDataBufferLQ->setName("MegaTextureContainer: Data Buffer LQ");

        Sampler::Desc linearDesc;
        linearDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Point);
        linearDesc.setAddressingMode(Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp);
        mpLinearSampler = Sampler::create(linearDesc);

        Program::Desc passDesc;
        passDesc.addShaderLibrary("Falcor/Utils/VirtualLight/MegaTextureBlit.cs.slang").csEntry("main").setShaderModel("6_5");
        mpBlitPass = ComputePass::create(passDesc, Program::DefineList());
    }
}
