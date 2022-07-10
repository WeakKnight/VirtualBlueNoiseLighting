#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Sampling.ReSTIR;
import Utils.Math.MathHelpers;
import Utils.VirtualLight.VirtualLightContainer;
import Scene.Scene;
import Scene.Lights.Lights;
import Scene.Shading;
import Experimental.Scene.Lights.LightCollection;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Utils.Sampling.AliasTable;

cbuffer CB
{
    uint gFrameIndex;
    AliasTable gFluxTable;
    VirtualLightContainer gVirtualLights;
    ResamplingRuntimeParameters gParams;
}

Texture2D<float> gVirtualLightPdfTexture; 
Texture2D<float> gEmissiveTrianglePdfTexture;
RWStructuredBuffer<uint2> gRISBuffer;

/*
Dispatch Dimension
*/
[numthreads(256, 1, 1)] 
void main(uint2 GlobalIndex : SV_DispatchThreadID) 
{
    uint risBufferPtr = GlobalIndex.x + GlobalIndex.y * gParams.tileSize;
    
    SampleGenerator sg = SampleGenerator.create(GlobalIndex, gFrameIndex);
    /*
    Sample Virtual Lights
    */
#ifdef _UNIFORM_SAMPLING
    uint lightIndex = min(sampleNext1D(sg) * gVirtualLights.count, gVirtualLights.count - 1);
    float lightInvPdf = gVirtualLights.count;
#else
    uint lightIndex = gFluxTable.sample(sampleNext2D(sg));
    float lightInvPdf = gFluxTable.weightSum / gFluxTable.getWeight(lightIndex);
#endif

    if (gFluxTable.getWeight(lightIndex) > 0)
    {
        gRISBuffer[risBufferPtr] = uint2(lightIndex, asuint(lightInvPdf));
    }
    else
    {
        gRISBuffer[risBufferPtr] = uint2(0, 0);
    }
}