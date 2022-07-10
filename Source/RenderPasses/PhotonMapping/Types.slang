#pragma once
#include "Utils/HostDeviceShared.slangh"
BEGIN_NAMESPACE_FALCOR

struct Photon
{
    float3 position;
    float3 direction;
    float3 normal;
    float3 power;
};

struct PhotonTraceParams
{
    uint frameIndex = 0;
    uint photonPathCount = 0u;
    float radius = 0.015f;
};

struct PackedBoundingBox
{
    float3 minPoint;
    float3 maxPoint;
    float pad0;
    float pad1;
};

END_NAMESPACE_FALCOR
