#pragma once
#include "BoundingBoxAccelerationStructureBuilder.h"

namespace Falcor
{
    class dlldecl VirtualLightContainer
    {
    public:
        using SharedPtr = std::shared_ptr<VirtualLightContainer>;
        using SharedConstPtr = std::shared_ptr<const VirtualLightContainer>;
        static SharedPtr create(uint32_t capacity, float boundingBoxRadius, bool asPhotons = false);
        void setShaderData(const ShaderVar& var) const;
        void setRayTracingData(const ShaderVar& var, const std::string name) const;
        void updateCounterToCPU(RenderContext* renderContext);
        void updateTextureCounterToCPU(RenderContext* renderContext);

        void buildAS(RenderContext* renderContext);

        void setCount(RenderContext* renderContext, uint count);
        uint32_t getCount() const { return mCount; }

        inline uint32_t getTexCountHQ() const { return mTexCountHQ; }
        inline uint32_t getTexCountLQ() const { return mTexCountLQ; }

        float getBoundingBoxRadius() const { return mBoundingBoxRadius; }
        Buffer::SharedPtr getPositionBuffer() const { return  mpPositionBuffer; }
		Buffer::SharedPtr getNormalBuffer() const { return  mpNormalBuffer; }

        Buffer::SharedPtr getTypeBuffer() const { return mpTypeBuffer; }
        Buffer::SharedPtr getIndexBuffer() const { return mpIndexBuffer; }
        Buffer::SharedPtr getBoundingBoxBuffer() const { return mpBoundBoxBuffer; }
        Buffer::SharedPtr getDiffuseRadianceBuffer() const { return mpDiffuseRadianceBuffer; }
        Buffer::SharedPtr getThroughputBuffer() const { return mpThroughputBuffer; }
        Buffer::SharedPtr getIncidentRadianceBuffer() const { return mpIncidentRadianceBuffer; }
        Buffer::SharedPtr getMetalRoughnessBuffer() const { return mpMetalRoughnessBuffer; }

        void setRadiusFactorForVSL(float val) { mRadiusFactorForVSL = val; }
        float getRadiusFactorForVSL() const { return mRadiusFactorForVSL; }

        void setFluxBuffer(Buffer::SharedPtr fluxBuffer) { mpFluxBuffer = fluxBuffer; }
        Buffer::SharedPtr getFluxBuffer() const { return mpFluxBuffer; }

        Buffer::SharedPtr getBoundingBoxReadBuffer() const { return mpBoundingBoxReadBuffer; }
        Buffer::SharedPtr getFluxReadBuffer() const { return mpFluxReadBuffer; }
        Buffer::SharedPtr getNormalReadBuffer() const { return mpNormalReadBuffer; }

        void renderUI(Gui::Widgets& widget, uint32_t texSize = 16u);

        void prepareReadBuffers(RenderContext* renderContext)
        {
            mpBoundingBoxReadBuffer = Buffer::create(mpBoundBoxBuffer->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
            renderContext->copyBufferRegion(mpBoundingBoxReadBuffer.get(), 0, mpBoundBoxBuffer.get(), 0, mpBoundBoxBuffer->getSize());

            mpNormalReadBuffer = Buffer::create(mpNormalBuffer->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
            renderContext->copyBufferRegion(mpNormalReadBuffer.get(), 0, mpNormalBuffer.get(), 0, mpNormalBuffer->getSize());

            mpFluxReadBuffer = Buffer::create(mpFluxBuffer->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
            renderContext->copyBufferRegion(mpFluxReadBuffer.get(), 0, mpFluxBuffer.get(), 0, mpFluxBuffer->getSize());

            renderContext->flush(true);
        }

        void clearReadBuffers(RenderContext* renderContext)
        {
            mpBoundingBoxReadBuffer = nullptr;
            mpNormalReadBuffer = nullptr;
            mpFluxReadBuffer = nullptr;
        }

    private:
        VirtualLightContainer(uint32_t capacity, float boundingBoxRadius, bool asPhotons);
        uint32_t mCapacity;
        uint32_t mCount;
        uint32_t mTexCountHQ;
        uint32_t mTexCountLQ;
        float mRadiusFactorForVSL = 1.0f;

        float mBoundingBoxRadius;
        bool mHaveAS = false;
        Buffer::SharedPtr mpPositionBuffer;
        Buffer::SharedPtr mpNormalBuffer;
        Buffer::SharedPtr mpFaceNormalBuffer;
        Buffer::SharedPtr mpDiffuseBuffer;
        Buffer::SharedPtr mpSpecularBuffer;
        Buffer::SharedPtr mpMetalRoughnessBuffer;
        Buffer::SharedPtr mpFluxBuffer;

        Buffer::SharedPtr mpDiffuseRadianceBuffer;

        Buffer::SharedPtr mpTypeBuffer;
        Buffer::SharedPtr mpIndexBuffer;

        Buffer::SharedPtr mpReverseIndexBufferHQ;
        Buffer::SharedPtr mpCounterHQ;

        Buffer::SharedPtr mpReverseIndexBufferLQ;
        Buffer::SharedPtr mpCounterLQ;

        Buffer::SharedPtr mpThroughputBuffer;

        Buffer::SharedPtr mpBoundBoxBuffer;
        BoundingBoxAccelerationStructureBuilder::SharedPtr mpAccelerationStructureBuilder;

        // Photon Only
        bool mAsPhoton = false;
        Buffer::SharedPtr mpDirectionBuffer;
        Buffer::SharedPtr mpIncidentRadianceBuffer;

        // Read buffers
        Buffer::SharedPtr mpBoundingBoxReadBuffer;
        //Buffer::SharedPtr mpPositionReadBuffer;
        Buffer::SharedPtr mpNormalReadBuffer;
        Buffer::SharedPtr mpFluxReadBuffer;
    };
}

