#pragma once

namespace Falcor
{
    class dlldecl BoundingBoxAccelerationStructureBuilder
    {
    public:
        using SharedPtr = std::shared_ptr<BoundingBoxAccelerationStructureBuilder>;

        static SharedPtr Create(Buffer::SharedPtr pBoundingBoxBuffer);

        void BuildAS(RenderContext* pContext, uint32_t boxCount, uint32_t rayTypeCount);

        void SetRaytracingShaderData(const ShaderVar& var, const std::string name, uint32_t rayTypeCount);

    private:

        void InitGeomDesc(uint32_t boxCount);

        void BuildBlas(RenderContext* pContext);

        void FillInstanceDesc(std::vector<D3D12_RAYTRACING_INSTANCE_DESC>& instanceDescs, uint32_t rayCount, bool perMeshHitEntry);

        void BuildTlas(RenderContext* pContext, uint32_t rayCount, bool perMeshHitEntry);

        Buffer::SharedPtr m_BoundingBoxBuffer;

        struct BlasData
        {
            D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO prebuildInfo;
            D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS buildInputs;
            std::vector<D3D12_RAYTRACING_GEOMETRY_DESC> geomDescs;

            uint64_t blasByteSize = 0;                      ///< Size of the final BLAS.
            uint64_t blasByteOffset = 0;                    ///< Offset into the BLAS buffer to where it is stored.
            uint64_t scratchByteOffset = 0;                 ///< Offset into the scratch buffer to use for rebuilds.
        };

        struct TlasData
        {
            Buffer::SharedPtr pTlas;
            ShaderResourceView::SharedPtr pSrv;             ///< Shader Resource View for binding the TLAS
            Buffer::SharedPtr pInstanceDescs;               ///< Buffer holding instance descs for the TLAS
        };

        std::vector<D3D12_RAYTRACING_INSTANCE_DESC> mInstanceDescs; ///< Shared between TLAS builds to avoid reallocating CPU memory
        std::unordered_map<uint32_t, TlasData> mTlasCache;  ///< Top Level Acceleration Structure for scene data cached per shader ray count
        Buffer::SharedPtr mpTlasScratch;                    ///< Scratch buffer used for TLAS builds. Can be shared as long as instance desc count is the same, which for now it is.
        D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO mTlasPrebuildInfo; ///< This can be reused as long as the number of instance descs doesn't change.

        std::vector<BlasData> mBlasData;    ///< All data related to the VPLs' BLASes.

        bool mRebuildBlas = true;
        Buffer::SharedPtr mpBlas;           ///< Buffer containing all BLASes.
        Buffer::SharedPtr mpBlasScratch;    ///< Scratch buffer used for BLAS builds.
    };
}
