import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Lights.Lights;
import Scene.Shading;
import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;

bool traceScene(float3 rayOrigin, float3 rayDir, out ShadingData sd)
{
    RayDesc ray;
	ray.Origin = rayOrigin;
	ray.Direction = rayDir;
	ray.TMin = 0.0f;
	ray.TMax = 1.0e38f;

    RayQuery<RAY_FLAG_NONE> rayQuery;

    rayQuery.TraceRayInline(
        gRtScene,
        RAY_FLAG_NONE,
        0xff,
        ray);

    while (rayQuery.Proceed())
    {
        if (rayQuery.CandidateType() == CANDIDATE_NON_OPAQUE_TRIANGLE)
        {
            // Alpha test for non-opaque geometry.
            HitInfo hit = getCandidateTriangleHit(rayQuery);
            VertexData v = gScene.getVertexData(hit);
            uint materialID = gScene.getMaterialID(hit.instanceID);

            if (alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f)) 
            {
                continue;
            }

            rayQuery.CommitNonOpaqueTriangleHit();
        }
    }

    if(rayQuery.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
    {
        HitInfo hit = getCommittedTriangleHit(rayQuery);

        uint meshInstanceID = hit.instanceID;
        uint materialID = gScene.getMaterialID(meshInstanceID);

        float3 barycentrics = float3(1.f - hit.barycentrics.x - hit.barycentrics.y, hit.barycentrics.x, hit.barycentrics.y);
        VertexData v = gScene.getVertexData(hit.instanceID, hit.primitiveIndex, barycentrics);
        sd = prepareShadingData(v, materialID, gScene.materials[materialID], gScene.materialResources[materialID], -rayQuery.WorldRayDirection(), 0.f);
        return true;
    }

    return false;
}

bool traceSceneWithTMax(float3 rayOrigin, float3 rayDir, float tmax, out ShadingData sd)
{
    RayDesc ray;
	ray.Origin = rayOrigin;
	ray.Direction = rayDir;
	ray.TMin = 0.0f;
	ray.TMax = tmax;

    RayQuery<RAY_FLAG_NONE> rayQuery;

    rayQuery.TraceRayInline(
        gRtScene,
        RAY_FLAG_NONE,
        0xff,
        ray);

    while (rayQuery.Proceed())
    {
        if (rayQuery.CandidateType() == CANDIDATE_NON_OPAQUE_TRIANGLE)
        {
            // Alpha test for non-opaque geometry.
            HitInfo hit = getCandidateTriangleHit(rayQuery);
            VertexData v = gScene.getVertexData(hit);
            uint materialID = gScene.getMaterialID(hit.instanceID);

            if (alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f)) 
            {
                continue;
            }

            rayQuery.CommitNonOpaqueTriangleHit();
        }
    }

    if(rayQuery.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
    {
        HitInfo hit = getCommittedTriangleHit(rayQuery);

        uint meshInstanceID = hit.instanceID;
        uint materialID = gScene.getMaterialID(meshInstanceID);

        float3 barycentrics = float3(1.f - hit.barycentrics.x - hit.barycentrics.y, hit.barycentrics.x, hit.barycentrics.y);
        VertexData v = gScene.getVertexData(hit.instanceID, hit.primitiveIndex, barycentrics);
        sd = prepareShadingData(v, materialID, gScene.materials[materialID], gScene.materialResources[materialID], -rayQuery.WorldRayDirection(), 0.f);
        return true;
    }

    return false;
}

bool castScene(float3 rayOrigin, float3 rayDir, out ShadingData sd, out TriangleHit trianglehit, float tmax = 1.0e38f)
{
    RayDesc ray;
	ray.Origin = rayOrigin;
	ray.Direction = rayDir;
	ray.TMin = 0.0f;
	ray.TMax = tmax;

    RayQuery<RAY_FLAG_NONE> rayQuery;

    rayQuery.TraceRayInline(
        gRtScene,
        RAY_FLAG_NONE,
        0xff,
        ray);

    while (rayQuery.Proceed())
    {
        if (rayQuery.CandidateType() == CANDIDATE_NON_OPAQUE_TRIANGLE)
        {
            // Alpha test for non-opaque geometry.
            HitInfo hit = getCandidateTriangleHit(rayQuery);
            VertexData v = gScene.getVertexData(hit);
            uint materialID = gScene.getMaterialID(hit.instanceID);

            if (alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f)) 
            {
                continue;
            }

            rayQuery.CommitNonOpaqueTriangleHit();
        }
    }

    if(rayQuery.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
    {
        HitInfo hit = getCommittedTriangleHit(rayQuery);

        uint meshInstanceID = hit.instanceID;
        uint materialID = gScene.getMaterialID(meshInstanceID);

        float3 barycentrics = float3(1.f - hit.barycentrics.x - hit.barycentrics.y, hit.barycentrics.x, hit.barycentrics.y);
        VertexData v = gScene.getVertexData(hit.instanceID, hit.primitiveIndex, barycentrics);
        sd = prepareShadingData(v, materialID, gScene.materials[materialID], gScene.materialResources[materialID], -rayQuery.WorldRayDirection(), 0.f);

        trianglehit.triangleIndex = gScene.lightCollection.getTriangleIndex(hit.instanceID, hit.primitiveIndex);
        trianglehit.posW = sd.posW;
        trianglehit.normalW = sd.frontFacing ? sd.faceN : -sd.faceN;
        return true;
    }

    return false;
}

bool isOccluded(float3 origin, float3 lightPosW, float bias = 0.0f)
{
    RayDesc ray;
	ray.Origin = origin;
	ray.Direction = normalize(lightPosW - origin);
	ray.TMin = 0.0f;
	ray.TMax = length(lightPosW - origin) - bias;

    RayQuery<RAY_FLAG_CULL_NON_OPAQUE> rayQuery;
        //RayQuery<RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES> rayQuery;

    rayQuery.TraceRayInline(
        gRtScene,
        RAY_FLAG_CULL_NON_OPAQUE, // OR'd with template flags above
        0xff, // InstanceInclusionMask
        ray);

    while (rayQuery.Proceed())
    {
        if (rayQuery.CandidateType() == CANDIDATE_NON_OPAQUE_TRIANGLE)
        {
            // Alpha test for non-opaque geometry.
            HitInfo hit = getCandidateTriangleHit(rayQuery);
            VertexData v = gScene.getVertexData(hit);
            uint materialID = gScene.getMaterialID(hit.instanceID);

            if (alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f)) 
            {
                continue;
            }

            rayQuery.CommitNonOpaqueTriangleHit();
            rayQuery.Abort();
        }
    }

    if(rayQuery.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
    {
        return true;
    }
    else if(rayQuery.CommittedStatus() == COMMITTED_NOTHING)
    {
        return false;
    }

    return false;
}
