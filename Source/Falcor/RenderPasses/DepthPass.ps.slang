import Scene.Raster;

void main(VSOut vOut, uint triangleIndex : SV_PrimitiveID) : SV_TARGET
{
    // Calling prepareShadingData() to discard pixels that fail alpha test. The pixel shader has no other side effects.
    float3 viewDir = normalize(gScene.camera.getPosition() - vOut.posW);
    prepareShadingData(vOut, triangleIndex, viewDir);
}
