import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Lights.Lights;
import Scene.Shading;
import Experimental.Scene.Lights.LightCollection;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;
import Scene.RaytracingInlineHelper;
import Utils.VirtualLight.VirtualLightContainer;
import Utils.VirtualLight.MegaTextureContainer;
import Utils.Sampling.AliasTable;
import Experimental.Scene.Material.MaterialShading;
import Experimental.Scene.Material.Microfacet;
import Experimental.Scene.Material.Fresnel;
import Scene.Material.SimpleBRDF;

cbuffer CB
{
    uint gFrameIndex;
    uint gPhotonPathCount;
    VirtualLightContainer gVirtualLightContainer;
    AliasTable gEmissiveTriTable;
}

[numthreads(256, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if (launchIndex.x >= gPhotonPathCount) 
    {
        return;
    }
    
    SampleGenerator sg = SampleGenerator.create(uint2(launchIndex.x, 1), gFrameIndex);
    uint triIndex = gEmissiveTriTable.sample(sampleNext2D(sg));
    EmissiveTriangle emissiveTri = gScene.lightCollection.getTriangle(triIndex);
    float3 barycentric = sample_triangle(sampleNext2D(sg));
    float3 samplePos = emissiveTri.getPosition(barycentric);
    float3 rayDir = cosineWeightedSampling(sampleNext2D(sg), emissiveTri.normal);
    float3 rayOrigin = computeRayOrigin(samplePos, emissiveTri.normal);

    for (int bounce = 0; bounce < 4; bounce++)
    {
        ShadingData sd;
        if (!traceScene(rayOrigin, rayDir, sd))
        {
            break;
        }

        int lightIndex = gVirtualLightContainer.pushLight(sd);

        /*
        Continue Tracing
        */
        SimpleBRDF brdf;
        brdf.setup(sd);

        float3 V;
        float pdf;
        uint lobeType;
        bool valid = brdf.sample(sd.N, sd.V, V, pdf, lobeType, sg);
        if(!valid || dot(V, sd.faceN) < 1e-6f || dot(sd.V, sd.N) < 1e-6f)
        {
            break;
        }

        rayOrigin = computeRayOrigin(sd.posW, sd.faceN);    
        rayDir = V;
    }
}
