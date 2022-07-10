#include "stdafx.h"
#include "VirtualLightContainer.h"

namespace Falcor
{
    VirtualLightContainer::SharedPtr VirtualLightContainer::create(uint32_t capacity, float boundingBoxRadius, bool asPhotons)
    {
        return SharedPtr(new VirtualLightContainer(capacity, boundingBoxRadius, asPhotons));
    }

    void VirtualLightContainer::setShaderData(const ShaderVar& var) const
    {
        var["positionBuffer"] = mpPositionBuffer;
        var["normalBuffer"] = mpNormalBuffer;
        var["faceNormalBuffer"] = mpFaceNormalBuffer;
        var["diffuseBuffer"] = mpDiffuseBuffer;
        var["specularBuffer"] = mpSpecularBuffer;
        var["metalRoughnessBuffer"] = mpMetalRoughnessBuffer;
        var["diffuseRadianceBuffer"] = mpDiffuseRadianceBuffer;

        var["indexBuffer"] = mpIndexBuffer;
        var["typeBuffer"] = mpTypeBuffer;
        var["reverseIndexBufferHQ"] = mpReverseIndexBufferHQ;
        var["counterHQ"] = mpCounterHQ;
        var["reverseIndexBufferLQ"] = mpReverseIndexBufferLQ;
        var["counterLQ"] = mpCounterLQ;

        var["throughputBuffer"] = mpThroughputBuffer;

        var["boundingBoxBuffer"] = mpBoundBoxBuffer;
        if (mHaveAS)
        {
            mpAccelerationStructureBuilder->SetRaytracingShaderData(var, "as", 1);
        }
        var["count"] = mCount;
        var["texCountHQ"] = mTexCountHQ;
        var["texCountLQ"] = mTexCountLQ;
        var["radiusFactorForVSL"] = mRadiusFactorForVSL;
        var["boundingBoxRadius"] = mBoundingBoxRadius;

        if (mAsPhoton)
        {
            var["directionBuffer"] = mpDirectionBuffer;
            var["incidentRadianceBuffer"] = mpIncidentRadianceBuffer;
        }
    }

    void VirtualLightContainer::setRayTracingData(const ShaderVar& var, const std::string name) const
    {
        mpAccelerationStructureBuilder->SetRaytracingShaderData(var, name, 1);
    }

    void VirtualLightContainer::updateCounterToCPU(RenderContext* renderContext)
    {
        Buffer::SharedPtr counterReadBuffer = Buffer::create(sizeof(uint), ResourceBindFlags::None, Buffer::CpuAccess::Read);
        Buffer::SharedPtr counterBuffer = mpPositionBuffer->getUAVCounter();

        renderContext->copyBufferRegion(counterReadBuffer.get(), 0, counterBuffer.get(), 0, counterBuffer->getSize());
        renderContext->flush(true);

        uint* data = (uint*)counterReadBuffer->map(Buffer::MapType::Read);
        mCount = data[0];
        counterReadBuffer->unmap();

        if (mCapacity < mCount)
        {
            logError("Virtual Light Capacity Is Not Enough, Should Be Bigger Than " + std::to_string(mCount));
        }
    }

    void VirtualLightContainer::updateTextureCounterToCPU(RenderContext* renderContext)
    {
        {
            Buffer::SharedPtr counterReadBuffer = Buffer::create(sizeof(uint), ResourceBindFlags::None, Buffer::CpuAccess::Read);
            Buffer::SharedPtr counterBuffer = mpCounterHQ->getUAVCounter();

            renderContext->copyBufferRegion(counterReadBuffer.get(), 0, counterBuffer.get(), 0, counterBuffer->getSize());
            renderContext->flush(true);

            uint* data = (uint*)counterReadBuffer->map(Buffer::MapType::Read);
            mTexCountHQ = data[0];
            counterReadBuffer->unmap();
        }

        {
            Buffer::SharedPtr counterReadBuffer = Buffer::create(sizeof(uint), ResourceBindFlags::None, Buffer::CpuAccess::Read);
            Buffer::SharedPtr counterBuffer = mpCounterLQ->getUAVCounter();

            renderContext->copyBufferRegion(counterReadBuffer.get(), 0, counterBuffer.get(), 0, counterBuffer->getSize());
            renderContext->flush(true);

            uint* data = (uint*)counterReadBuffer->map(Buffer::MapType::Read);
            mTexCountLQ = data[0];
            counterReadBuffer->unmap();
        }
    }

    void VirtualLightContainer::buildAS(RenderContext* renderContext)
    {
        mpAccelerationStructureBuilder->BuildAS(renderContext, mCount, 1);
        mHaveAS = true;
    }

    void VirtualLightContainer::setCount(RenderContext* renderContext, uint count)
    {
        renderContext->clearUAVCounter(mpPositionBuffer, count);
        mCount = count;
    }

    void VirtualLightContainer::renderUI(Gui::Widgets& widget, uint32_t texSize)
    {
        widget.text("Count:" + std::to_string(mCount));
        widget.text("Tex Count:" + std::to_string(mTexCountHQ));

        float totalImportonSizeInMBs = sizeof(float) * 5llu * static_cast<size_t>(mCount) / float(1024llu * 1024llu);
        widget.text("Importon Size:" + std::to_string(totalImportonSizeInMBs) + "mb");

        float photonSizeInMBs = sizeof(float) * 8llu * static_cast<size_t>(mCount) / float(1024llu * 1024llu);
        widget.text("Photon Size:" + std::to_string(photonSizeInMBs) + "mb");

        float totalVirtualLightSizeInMBs = sizeof(float) * 36llu * static_cast<size_t>(mCount) / float(1024llu * 1024llu);
        widget.text("Virtual Light Size:" + std::to_string(totalVirtualLightSizeInMBs) + "mb");

        float totalTexSizeInMBs = sizeof(float4) * static_cast<size_t>(texSize) * static_cast<size_t>(texSize) * static_cast<size_t>(mTexCountHQ) / float(1024llu * 1024llu);
        widget.text("Mega Texture Size:" + std::to_string(totalTexSizeInMBs) + "mb");

        widget.text("Total Size:" + std::to_string(totalVirtualLightSizeInMBs + totalTexSizeInMBs) + "mb");
    }

    VirtualLightContainer::VirtualLightContainer(uint32_t capacity, float boundingBoxRadius, bool asPhotons) :
        mCapacity(capacity),
        mCount(0),
        mBoundingBoxRadius(boundingBoxRadius),
        mAsPhoton(asPhotons)
    {
        mpPositionBuffer = Buffer::createStructured(sizeof(float3), mCapacity);
        mpPositionBuffer->setName("Virutal Light Container: Position Buffer");

        mpNormalBuffer = Buffer::createStructured(sizeof(uint), mCapacity);
        mpNormalBuffer->setName("Virutal Light Container: Normal Buffer");

        mpFaceNormalBuffer = Buffer::createStructured(sizeof(uint), mCapacity);
        mpFaceNormalBuffer->setName("Virutal Light Container: Face Normal Buffer");

        mpDiffuseBuffer = Buffer::createStructured(sizeof(float3), mCapacity);
        mpDiffuseBuffer->setName("Virutal Light Container: Diffuse Buffer");

        mpSpecularBuffer = Buffer::createStructured(sizeof(float3), mCapacity);
        mpSpecularBuffer->setName("Virutal Light Container: Specular Buffer");

        mpMetalRoughnessBuffer = Buffer::createStructured(sizeof(float2), mCapacity);
        mpMetalRoughnessBuffer->setName("Virutal Light Container: Metal Roughness Buffer");

        mpDiffuseRadianceBuffer = Buffer::createStructured(sizeof(float3), mCapacity);
        mpDiffuseRadianceBuffer->setName("Virutal Light Container: Diffuse Radiance Buffer");

        mpTypeBuffer = Buffer::createTyped(ResourceFormat::R8Uint, mCapacity);
        mpTypeBuffer->setName("Virutal Light Container: TypeBuffer");

        mpIndexBuffer = Buffer::createStructured(sizeof(uint), mCapacity);
        mpIndexBuffer->setName("Virutal Light Container: IndexBuffer");

        mpThroughputBuffer = Buffer::createStructured(sizeof(float), mCapacity);
        mpThroughputBuffer->setName("Virutal Light Container: ThroughputBuffer");

        mpReverseIndexBufferHQ = Buffer::createStructured(sizeof(uint), mCapacity);
        mpReverseIndexBufferHQ->setName("Virtual Light Container: Reverse Link HQ");

        mpCounterHQ = Buffer::createStructured(sizeof(uint), 1);
        mpCounterHQ->setName("Virutal Light Container: CounterHQ");

        mpReverseIndexBufferLQ = Buffer::createStructured(sizeof(uint), mCapacity);
        mpReverseIndexBufferLQ->setName("Virtual Light Container: Reverse Link LQ");

        mpCounterLQ = Buffer::createStructured(sizeof(uint), 1);
        mpCounterLQ->setName("Virutal Light Container: CounterLQ");

        mpBoundBoxBuffer = Buffer::createStructured(sizeof(float) * 8, mCapacity);
        mpBoundBoxBuffer->setName("Virutal Light Container: Bound Box Buffer");

        mpAccelerationStructureBuilder = BoundingBoxAccelerationStructureBuilder::Create(mpBoundBoxBuffer);

        if (mAsPhoton)
        {
            mpDirectionBuffer = Buffer::createStructured(sizeof(float3), mCapacity);
            mpDirectionBuffer->setName("Virutal Light Container: Incident Direction Buffer");

            mpIncidentRadianceBuffer = Buffer::createStructured(sizeof(float3), mCapacity);
            mpIncidentRadianceBuffer->setName("Virutal Light Container: Incident Radiance Buffer");
        }
    }
}
