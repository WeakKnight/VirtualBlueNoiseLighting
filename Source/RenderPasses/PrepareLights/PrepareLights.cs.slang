import Utils.Math.MathHelpers;
import Scene.Scene;
import Scene.Lights.Lights;
import Scene.Shading;
import Experimental.Scene.Lights.LightCollection;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;

cbuffer CB
{
    uint gTriangleCount;
}

RWTexture2D<float> gEmissiveTrianglePdfTexture; 

[numthreads(256, 1, 1)]
void main(uint dispatchThreadId : SV_DispatchThreadID, uint groupThreadId : SV_GroupThreadID)
{
    if(dispatchThreadId >= gTriangleCount)
    {
        return;
    }

    uint2 pdfTexturePosition = linearIndexToZCurve(dispatchThreadId);    
    gEmissiveTrianglePdfTexture[pdfTexturePosition] = gScene.lightCollection.fluxData[dispatchThreadId].flux;
}
