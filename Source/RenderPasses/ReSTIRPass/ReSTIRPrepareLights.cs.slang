import Utils.Math.MathHelpers;
import Utils.VirtualLight.VirtualLightContainer;
import Utils.VirtualLight.MegaTextureContainer;

cbuffer CB
{
    VirtualLightContainer gVirtualLights;
}

StructuredBuffer<float> gFluxBuffer;
RWTexture2D<float> gVirtualLightPdfTexture; 

[numthreads(256, 1, 1)]
void main(uint dispatchThreadId : SV_DispatchThreadID, uint groupThreadId : SV_GroupThreadID)
{
    if(dispatchThreadId >= gVirtualLights.count)
    {
        return;
    }

    uint2 pdfTexturePosition = linearIndexToZCurve(dispatchThreadId);
    float flux = gFluxBuffer[dispatchThreadId];
    gVirtualLightPdfTexture[pdfTexturePosition] = flux;
}