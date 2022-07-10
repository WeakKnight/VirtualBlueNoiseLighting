import Scene.Shading;
import Scene.Raster;
import Scene.HitInfo;
import Utils.Helpers;
import Scene.TextureSampler;
import Utils.Math.MathHelpers;
import Utils.Math.PackedFormats;
#include "Scene/Material/MaterialDefines.slangh"

cbuffer PerFrameCB
{
    float2  frameSize;
    float3  cameraPos;
};

RWTexture2D<float2> gMotionVectors;
RWTexture2D<float4> gMotionVectorW;
RWTexture2D<PackedHitInfo> gVBuffer;
RWTexture2D<float4> gNormalWRoughnessMaterialID;

struct GBuffer 
{
	float4 albedo    : SV_Target0;
    float metallic: SV_Target1;
    float4 positon   : SV_Target2;
    float4 normal    : SV_Target3;
    float4 emissive  : SV_Target4;
    float4 faceN     : SV_Target5;
    float4 linearZAndNormal: SV_Target6;
};

VSOut vsMain(VSIn vIn)
{
    VSOut vOut;
    float4x4 worldMat = gScene.getWorldMatrix(vIn.meshInstanceID);
    float4 posW = mul(float4(vIn.pos, 1.f), worldMat);
    vOut.posW = posW.xyz;
    vOut.posH = mul(posW, gScene.camera.getViewProj());

    vOut.meshInstanceID = vIn.meshInstanceID;
    vOut.materialID = gScene.getMaterialID(vIn.meshInstanceID);

    vOut.texC = vIn.texC;
    vOut.normalW = mul(vIn.unpack().normal, gScene.getInverseTransposeWorldMatrix(vIn.meshInstanceID));
    float4 tangent = vIn.unpack().tangent;
    vOut.tangentW = float4(mul(tangent.xyz, (float3x3)gScene.getWorldMatrix(vIn.meshInstanceID)), tangent.w);

    // Compute the vertex position in the previous frame.
    float3 prevPos = vIn.pos;
    MeshInstanceData meshInstance = gScene.getMeshInstance(vIn.meshInstanceID);
    if (meshInstance.hasDynamicData())
    {
        uint dynamicVertexIndex = gScene.meshes[meshInstance.meshID].dynamicVbOffset + vIn.vertexID;
        prevPos = gScene.prevVertices[dynamicVertexIndex].position;
    }
    float4 prevPosW = mul(float4(prevPos, 1.f), gScene.getPrevWorldMatrix(vIn.meshInstanceID));
    vOut.prevPosH = prevPosW;
    return vOut;
}

[earlydepthstencil]
GBuffer psMain(VSOut vOut, uint triangleIndex : SV_PrimitiveID, float3 barycentrics : SV_Barycentrics)
{
    int2 ipos = int2(vOut.posH.xy);

    float3 faceNormal = gScene.getFaceNormalW(vOut.meshInstanceID, triangleIndex);
    VertexData v = prepareVertexData(vOut, faceNormal);

    if (alphaTest(v, gScene.materials[vOut.materialID], gScene.materialResources[vOut.materialID])) 
    {
        discard;
    }

    float3 viewDir = normalize(gScene.camera.getPosition() - vOut.posW);
    ShadingData sd = prepareShadingData(vOut, triangleIndex, viewDir);

    adjustShadingNormal(sd, v);

    // ImplicitLodTextureSampler lod = { };
    // MaterialData md = gScene.materials[vOut.materialID];
    // MaterialResources mr = gScene.materialResources[vOut.materialID];
    // float4 baseColor = sampleTexture(mr.baseColor, mr.samplerState, v.texC, md.baseColor, EXTRACT_DIFFUSE_TYPE(md.flags), lod);

    float packedSpecular = asfloat(Pack_R8G8B8A8_Gamma_UFLOAT(float4(sd.specular, sd.linearRoughness)));

    GBuffer gBufOut;
	gBufOut.albedo     = float4(sd.diffuse, sd.opacity);
    gBufOut.metallic = sd.metallic;
    gBufOut.positon    = float4(sd.posW, packedSpecular);
    gBufOut.normal     = float4(sd.N, length(sd.posW - cameraPos));
    gBufOut.emissive   = float4(sd.emissive, sd.doubleSided? 1.0f: 0.2f);
    gBufOut.faceN      = float4(sd.faceN, sd.frontFacing? 1.0f: 0.0f);                  
    
    // Motion Vectors
    const float2 pixelPos = ipos + float2(0.5, 0.5); // Current sample in pixel coords.
    float4 prevPosW = vOut.prevPosH;
    const float4 prevPosH = mul(float4(prevPosW.xyz, 1.0f), gScene.camera.data.prevViewProjMatNoJitter);
    const float2 mv = calcMotionVector(pixelPos, prevPosH, frameSize) + float2(gScene.camera.data.jitterX, -gScene.camera.data.jitterY); // Remove camera jitter from motion vector
    gMotionVectors[ipos] = mv;
    gMotionVectorW[ipos] = float4(prevPosW.xyz - sd.posW, 0.f);

    // Pos and Normal's Derivatives 
    // gPosNormalFwidth[ipos] = float2(length(fwidth(sd.posW)), length(fwidth(sd.N)));

    // Linear z and its derivative
    float linearZ = vOut.posH.z * vOut.posH.w;
    float2 linearZAndDeriv = float2(linearZ, max(abs(ddx(linearZ)), abs(ddy(linearZ))));
    // gLinearZAndDeriv[ipos] = linearZAndDeriv;
    float2 packedNormal = ndir_to_oct_snorm(gBufOut.normal.xyz);
    gBufOut.linearZAndNormal = float4(linearZAndDeriv, packedNormal);

    HitInfo hit;
    hit.type = InstanceType::TriangleMesh;
    hit.instanceID = vOut.meshInstanceID;
    hit.primitiveIndex = triangleIndex;
    hit.barycentrics = barycentrics.yz;
    gVBuffer[ipos] = hit.encode();

    float2 octNormal = ndir_to_oct_unorm(sd.N);
    float roughness = sd.linearRoughness;
    float materialID = 0.f;
    gNormalWRoughnessMaterialID[ipos] = float4(octNormal, roughness, materialID);
    
    return gBufOut;
}
