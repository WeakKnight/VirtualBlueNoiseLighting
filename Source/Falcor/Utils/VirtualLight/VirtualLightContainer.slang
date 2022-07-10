#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;
import Scene.Scene;
import Scene.Shading;
import Scene.HitInfo;
import Experimental.Scene.Material.BxDFTypes;
import Experimental.Scene.Material.MaterialShading;
import Scene.Material.SimpleBRDF;
import Utils.Helpers;
import Utils.VirtualLight.MegaTextureContainer;
import Scene.RaytracingInlineHelper;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Utils.Debug.PixelDebug;

#ifdef _MAX_BSDF_SEARCH_COUNT
static const uint maxSearchIterCount = _MAX_BSDF_SEARCH_COUNT;
#else
static const uint maxSearchIterCount = 16;
#endif
static const bool readRadianceByLightPosition = false;
static const bool readRadianceCheckingNormal = true;

struct PackedBoundingBox
{
    float3 minPoint;
    float3 maxPoint;
    float Pad0;
    float Pad1;
};

float simpliedCemAttenuation(float d, float r)
{
    const float scaler = d * d + r * r;
    return 2.0 / (scaler + d * sqrt(scaler)); 
}

float GeometryB()
{
    return 30.0f;
}

float GeometryTerm(float3 vplPos, float3 vplNormal, float3 shadingPos, float3 shadingNormal)
{
    float3 dirToShading = normalize(shadingPos - vplPos);
    float3 dirToVPL = -dirToShading;
    float geometryValue = saturate(dot(vplNormal, dirToShading)) * saturate(dot(shadingNormal, dirToVPL)) / dot(vplPos - shadingPos, vplPos - shadingPos);
    return geometryValue;
}

float GeometryTermPrime(float3 vplPos, float3 vplNormal, float3 shadingPos, float3 shadingNormal)
{
    float geometryValue = GeometryTerm(vplPos, vplNormal, shadingPos, shadingNormal);
    const float b = GeometryB();
    return min(geometryValue, b);
}

float GeometryTermResidual(float3 vplPos, float3 vplNormal, float3 shadingPos, float3 shadingNormal)
{
    const float b = GeometryB();
    float geometryTerm = GeometryTerm(vplPos, vplNormal, shadingPos, shadingNormal);
    return max(geometryTerm - b, 0.0) / geometryTerm;
}

struct VirtualLightContainer
{
    RWStructuredBuffer<float3> positionBuffer;
    RWStructuredBuffer<uint> normalBuffer;
    RWStructuredBuffer<uint> faceNormalBuffer;
    RWStructuredBuffer<float3> diffuseBuffer;
    RWStructuredBuffer<float3> specularBuffer; 
    RWStructuredBuffer<float2> metalRoughnessBuffer;
    RWByteAddressBuffer diffuseRadianceBuffer;

    RWStructuredBuffer<float3> directionBuffer;
    RWStructuredBuffer<float3> incidentRadianceBuffer;

    /*
    R8 Uint, 0 None, 1 LQ, 2 MQ, 3 HQ
    */
    RWBuffer<uint> typeBuffer;
    RWStructuredBuffer<uint> indexBuffer;
    RWStructuredBuffer<uint> reverseIndexBufferHQ;
    RWStructuredBuffer<uint> counterHQ;
    RWStructuredBuffer<uint> reverseIndexBufferLQ;
    RWStructuredBuffer<uint> counterLQ;

    RWStructuredBuffer<float> throughputBuffer;

    RWStructuredBuffer<PackedBoundingBox> boundingBoxBuffer;
    RaytracingAccelerationStructure as;
    float boundingBoxRadius;
    uint count;
    uint texCountHQ;
    uint texCountLQ;
    float radiusFactorForVSL;

    float3 evalVPL(uint i, ShadingData sd)
    {
        float3 Le = incidentRadianceBuffer[i];
        ShadingData vplSd = getShadingData(i);        
        float3 LL = directionBuffer[i];
        float3 VL = -normalize(vplSd.posW - sd.posW);
        float3 wiL = toLocal(LL, vplSd.N);
        float3 woL = toLocal(VL, vplSd.N);
        SimpleBRDF vplBRDF;
        vplBRDF.setup(vplSd);
        float3 flux = Le * vplBRDF.evalFrWIthoutCheck(woL, wiL);

        SimpleBRDF simpleBRDF;
        simpleBRDF.setup(sd);

        float3 wo = toLocal(sd.V, sd.N);
        float3 wi = toLocal(-VL, sd.N);
        float radius = getVSLRadius(i);
        float attenuation = simpliedCemAttenuation(length(vplSd.posW - sd.posW), 2.0 * radius) * saturate(dot(vplSd.N, VL)) * wi.z;
        if (min(wo.z, wi.z) < 1e-6f)
        {
            attenuation = 0.0f;
        }

        float3 brdfValue = simpleBRDF.evalFr(wo, wi);
  
        return flux * brdfValue * attenuation;
    }

    float3 readLe(uint i, float3 L)
    {
        float3 Le = incidentRadianceBuffer[i];
        ShadingData vplSd = getShadingData(i);        
        float3 LL = directionBuffer[i];
        float3 VL = -L;
        float3 wiL = toLocal(LL, vplSd.N);
        float3 woL = toLocal(VL, vplSd.N);
        SimpleBRDF vplBRDF;
        vplBRDF.setup(vplSd);
        float3 flux = Le * vplBRDF.evalFrWIthoutCheck(woL, wiL);

        return flux;
    }

    float3 coneComputeShadingPointRadianceForVPL(ShadingData sd, inout SampleGenerator sg, int lightIndex, bool useRandom = false)
    {
        const int minSampleCount = 1; 
        int sampleCount = 32;
        
#ifdef _VSL_EVALUATION
        float solidAngle;
        bool coneSampleValid = isConeSamplingValid(lightIndex, sd.posW, solidAngle);
        sampleCount = minSampleCount + int((sampleCount - minSampleCount) * solidAngle / (2 * M_PI));
        float3 sum = 0.f;

        float LPdf;
        for (int i = 0; i < sampleCount; i++)
        {
            float3 L;
            float2 VSLrand;
            if (useRandom)
            {
                VSLrand = sampleNext2D(sg);
            }
            else
            {
                VSLrand = getHammersley(i, sampleCount);
            }

            if (coneSampleValid)
            {
                coneSamplingVSL(lightIndex, VSLrand, sd.posW, L, LPdf);
            }
            else // hemispherical sampling
            {
                L = HemisphereSampling(VSLrand, sd.N, LPdf);
            }

            float3 Le = readLe(lightIndex, L);
            
            sum += shadeSurfaceWithVSL(lightIndex, sd, Le, L);
        }

        return sum / (sampleCount * LPdf);
#else
        return evalVPL(lightIndex, sd);
#endif
    }

    uint pushLight(ShadingData sd)
    {
        uint ind = positionBuffer.IncrementCounter();
        setShadingData(ind, sd);

        float3 radius = float3(boundingBoxRadius);
        boundingBoxBuffer[ind].minPoint = sd.posW - radius;
        boundingBoxBuffer[ind].maxPoint = sd.posW + radius;

        return ind;
    }

    void allocateTextureIndex(uint ind)
    {
        float3 specular = getSpecular(ind);
        float linearRoughness = getRoughness(ind);

        if ((specular.x + specular.y + specular.z < 0.0001f))
        {
            typeBuffer[ind] = 0;
        }
        else if (linearRoughness >= 2.2)
        {
            typeBuffer[ind] = 1;
            uint texIndex = counterLQ.IncrementCounter();
            indexBuffer[ind] = texIndex;
            reverseIndexBufferLQ[texIndex] = ind;
        }
        else
        {
            typeBuffer[ind] = 3;
            uint texIndex = counterHQ.IncrementCounter();
            indexBuffer[ind] = texIndex;
            reverseIndexBufferHQ[texIndex] = ind;
        }
    }

    void interlockAddDiffuseRadiance(uint i, float3 radiance)
    {
        uint address = i * 12u;
        NvInterlockedAddFp32(diffuseRadianceBuffer, address + 0, radiance.r);
        NvInterlockedAddFp32(diffuseRadianceBuffer, address + 4, radiance.g);
        NvInterlockedAddFp32(diffuseRadianceBuffer, address + 8, radiance.b);
    }

    float3 readDiffuseRadiance(uint i)
    {
        uint address = i * 12u;
        float3 result = asfloat(diffuseRadianceBuffer.Load3(address));
        return result;
    }

    float3 readSpecLe(uint i, float3 L, MegaTextureContainer megaTexutre)
    {
        uint type = typeBuffer[i];
        if (type == 0)
        {
            return 0.0f;
        }

        float3 vplNormal = getNormal(i);
        if (dot(-L, vplNormal) <= 1e-6f)
        {
            return 0.0f;
        }

        float3 res = 0.0f;
        uint texIndex = indexBuffer[i];
        if (type == 3)
        {
            res = megaTexutre.readColorBiliearHQ(texIndex, toLocal(-L, vplNormal));
        }
        else if (type == 1)
        {
            res = megaTexutre.readColorBiliearLQ(texIndex, toLocal(-L, vplNormal));
        }
        return res;
    }

    float3 readLe(uint i, float3 L, MegaTextureContainer megaTexutre)
    {
        // return readSpecLe(i, L, megaTexutre);
        return readDiffuseRadiance(i) + readSpecLe(i, L, megaTexutre);
    }

    float3 shadeSurface(uint i, ShadingData sd, float3 Le)
    {
        float3 lightPos = getPosition(i);
        float3 lightN = getNormal(i);
        float3 L = normalize(lightPos - sd.posW);
        float3 V = sd.V;
        
        SimpleBRDF brdf;
        brdf.setup(sd);
        float3 wi = toLocal(L, sd.N);
        float3 wo = toLocal(V, sd.N);
        float3 brdfValue = brdf.evalFr(wo, wi);

#ifndef _USE_BIAS_COMPENSATION
        float attenuation = simpliedCemAttenuation(length(lightPos - sd.posW), 2.0 * getVSLRadius(i)) * saturate(dot(lightN, -L)) * wi.z;
        if (min(wo.z, wi.z) < 1e-6f)
        {
            attenuation = 0.0f;
        }
#else
        float attenuation = GeometryTermPrime(lightPos, lightN, sd.posW, sd.N);
#endif

        return Le * brdfValue * attenuation;
    }

    inline uint getType(uint i)
    {
        return typeBuffer[i];
    }

    inline uint getIndex(uint i)
    {
        return indexBuffer[i];
    }

    inline uint getVirtualLightIndexHQ(uint texIndex)
    {
        return reverseIndexBufferHQ[texIndex];
    }

    inline uint getVirtualLightIndexLQ(uint texIndex)
    {
        return reverseIndexBufferLQ[texIndex];
    }

    /*
    Shading Data Getter/Setter
    */
    inline ShadingData getShadingData(uint i)
    {
        ShadingData sd;
        sd.posW = getPosition(i);
        sd.N = getNormal(i);
        sd.faceN = getFaceNormal(i);
        sd.diffuse = getDiffuse(i);
        sd.specular = getSpecular(i);
        sd.metallic = getMetallic(i);
        sd.linearRoughness = getRoughness(i);
        // TODO Support Delta Transport
        // if((sd.linearRoughness * sd.linearRoughness) < 0.0064f)
        // {
        //     sd.linearRoughness = 0.081;
        // }
        sd.ggxAlpha = sd.linearRoughness * sd.linearRoughness;

        sd.V = 0.0f;
        sd.T = 0.0f;
        sd.B = 0.0f;
        sd.uv = 0.0f;
        sd.NdotV = 0.0f;
        sd.frontFacing = true;
        sd.doubleSided = false;
        sd.materialID = 0;
        sd.opacity = 1.0f;
        sd.emissive = 0.0f;
        sd.IoR = 1.5f;
        sd.eta = 1.0f;
        sd.activeLobes = (uint)LobeType::All;

        return sd;
    }

    inline void setShadingData(uint i, ShadingData sd)
    {
        float3 rayOrigin = computeRayOrigin(sd.posW, sd.frontFacing ? sd.faceN : -sd.faceN);
        setPosition(i, rayOrigin);
        setNormal(i, sd.N);
        setFaceNormal(i, sd.faceN);
        setDiffuse(i, sd.diffuse);
        setSpecular(i, sd.specular);
        setMetalRoughness(i, sd.metallic, sd.linearRoughness);
    }

    /*
    Pos Getter/Setter
    */
    inline float3 getPosition(uint i)
    {
        return positionBuffer[i];
    }
    inline void setPosition(uint i, float3 pos)
    {
        positionBuffer[i] = pos;
    }

    /*
    Shading Normal Getter/Setter
    */
    inline float3 getNormal(uint i)
    {
        return decodeNormal2x16(normalBuffer[i]);
    }
    inline void setNormal(uint i, float3 normal)
    {
        normalBuffer[i] = encodeNormal2x16(normal);
    }

    /*
    Face Normal Getter/Setter
    */
    inline float3 getFaceNormal(uint i)
    {
        return decodeNormal2x16(faceNormalBuffer[i]);
    }
    inline void setFaceNormal(uint i, float3 normal)
    {
        faceNormalBuffer[i] = encodeNormal2x16(normal);
    }

    /*
    Diffuse Getter/Setter
    */
    inline float3 getDiffuse(uint i)
    {
        return diffuseBuffer[i];
    }
    inline void setDiffuse(uint i, float3 diffuse)
    {
        diffuseBuffer[i] = diffuse;
    }

    /*
    Specular Getter/Setter
    */
    inline float3 getSpecular(uint i)
    {
        return specularBuffer[i];
    }
    inline void setSpecular(uint i, float3 specular)
    {
        specularBuffer[i] = specular;
    }
    
    /*
    MetalRoughness Getter/Setter
    */
    inline float getMetallic(uint i)
    {
        return metalRoughnessBuffer[i].x;
    }
    inline float getRoughness(uint i)
    {
        return metalRoughnessBuffer[i].y;
    }
    inline void setMetalRoughness(uint i, float metalic, float roughness)
    {
        metalRoughnessBuffer[i] = float2(metalic, roughness);
    }

    inline float getRadius(uint i)
    {
        return (boundingBoxBuffer[i].maxPoint.x - boundingBoxBuffer[i].minPoint.x) * 0.5f;
    }

    inline float getVSLRadius(uint i)
    {
        return max(getRadius(i) * radiusFactorForVSL, 1e-6f);
    }

    bool isConeSamplingValid(uint i, float3 posW)
    {
        float3 vplPos = getPosition(i);
        float radius = getVSLRadius(i);

        // float sinThethaSquare = radius * radius / dot(vplPos - posW, vplPos - posW);
        // if(sinThethaSquare < 1.0f)
        if((radius * radius) < dot(vplPos - posW, vplPos - posW))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool isConeSamplingValid(uint i, float3 posW, out float solidAngle)
    {
        float3 vplPos = getPosition(i);
        float radius = getVSLRadius(i);
        float r2 = radius * radius;
        float d2 = dot(vplPos - posW, vplPos - posW);

        if (r2 < d2)
        {
            float sinThethaSquare = r2 / d2;
            float cosThetaMax = sqrt(max(0.0f, 1.0f - sinThethaSquare));
            solidAngle = 2.0f * M_PI * (1.0f - cosThetaMax);
            return true;
        }
        else
        {
            solidAngle = 2.0f * M_PI;
            return false;
        }
    }

    // give direction, return random numbers
    float2 reverseConeSamplingVSL(float3 LDir, float3 vplPos, float cosThetaMax, float3 posW)
    {
        float3 normal = normalize(vplPos - posW);
        float3 UpVector = abs(normal.z) < 0.999 ? float3(0, 0, 1) : float3(1, 0, 0);
        float3 TangentX = normalize(cross(UpVector, normal));
        float3 TangentY = normalize(cross(normal, TangentX));

        float cosTheta = dot(LDir, normal);
        float sinTheta = sqrt(max(0.0f, 1.0f - cosTheta * cosTheta));
        float x = (1-cosTheta) / (1 - cosThetaMax);

        float3 LProj = (LDir - cosTheta*normal) / sinTheta;
        float y = atan2(dot(LProj, TangentY), dot(LProj, TangentX)) / (2.0f * M_PI);
        if (sinTheta == 0.f) y = 0.f;

        return float2(x,y);
    }

    float2 reverseCosineWeightedSampling(float3 LDir, float3 N)
    {
        float3 w = N;
        float3 u = 0.0;
        if (abs(w.x) > 0.1)
        {
            u = normalize(cross(float3(0.0, 1.0, 0.0), w));
        }
        else
        {
            u = normalize(cross(float3(1.0, 0.0, 0.0), w));
        }
        float3 v = cross(w, u);

        float cosTheta = dot(LDir, w);
        float y = 1-cosTheta*cosTheta;
        float sinTheta = sqrt(max(0.0f, y));
        float3 LProj = (LDir - cosTheta * w) / sinTheta;
        float x = atan2(dot(LProj, v), dot(LProj, u)) / (2.0f * M_PI);
        if (sinTheta == 0.f) x = 0.f;
        return float2(x,y);
    }

    void sampleVSL(uint index, float2 XY, float3 posW, float3 posNormal, out float3 LDir, out float pdf)
    {
        float3 vplPos = getPosition(index);
        float radius = getVSLRadius(index);
        float sinThethaSquare = radius * radius / dot(vplPos - posW, vplPos - posW);
        
        float oa = length(posW - vplPos);
        float2 ab = float2(radius, oa);
        float2 ac = float2(-radius, oa);

        float cosThetaMax;
        if(dot(ab, ac) > 0)
        {
            cosThetaMax = sqrt(max(0.0f, 1.0f - sinThethaSquare));
        }
        else
        {
            cosThetaMax = max(0.0, dot(ab, ac)); //isn't dot(ab,ac) <= 0?
        }

        float uniformConePdf = 1.0f / (2.0f * M_PI * (1.0f - cosThetaMax));
        pdf = uniformConePdf;
            
        float cosTheta = (1.0f - XY.x) + XY.x * cosThetaMax;
        float sinTheta = sqrt(max(0.0f, 1.0f - cosTheta * cosTheta));
        float phi = XY.y * 2.0f * M_PI;

        float3 normal = normalize(vplPos - posW);
        float3 UpVector = abs(normal.z) < 0.999 ? float3(0, 0, 1) : float3(1, 0, 0);
        float3 TangentX = normalize(cross(UpVector, normal));
        float3 TangentY = normalize(cross(normal, TangentX));

        LDir = cos(phi) * sinTheta * TangentX + sin(phi) * sinTheta * TangentY + cosTheta * normal;

    }

    void sampleVSL(uint index, inout SampleGenerator sg, float3 posW, float3 posNormal, out float3 LDir, out float pdf)
    {
        float2 XY = sampleNext2D(sg);
        sampleVSL(index, XY, posW, posNormal, LDir, pdf);
    }

    void coneSamplingVSL(uint i, float2 XY, float3 posW, out float3 LDir, out float pdf)
    {
        float3 vplPos = getPosition(i);
        float radius = getVSLRadius(i);

        float sinThethaSquare = radius * radius / dot(vplPos - posW, vplPos - posW);
        float cosThetaMax = sqrt(max(0.0f, 1.0f - sinThethaSquare));

        float uniformConePdf = 1.0f / (2.0f * M_PI * (1.0f - cosThetaMax));
        pdf = uniformConePdf;
            
        float cosTheta = (1.0f - XY.x) + XY.x * cosThetaMax;
        float sinTheta = sqrt(max(0.0f, 1.0f - cosTheta * cosTheta));
        float phi = XY.y * 2.0f * M_PI;

        float3 normal = normalize(vplPos - posW);
        float3 UpVector = abs(normal.z) < 0.999 ? float3(0, 0, 1) : float3(1, 0, 0);
        float3 TangentX = normalize(cross(UpVector, normal));
        float3 TangentY = normalize(cross(normal, TangentX));

        LDir = cos(phi) * sinTheta * TangentX + sin(phi) * sinTheta * TangentY + cosTheta * normal;
    }

    float evalConeSamplingPdf(uint i, float3 posW, float3 L)
    {
        float3 vplPos = getPosition(i);
        float radius = getVSLRadius(i);

        float sinThethaSquare = radius * radius / dot(vplPos - posW, vplPos - posW);
        float cosThetaMax = sqrt(max(0.0f, 1.0f - sinThethaSquare));
        if(dot(L, normalize(vplPos - posW)) >  cosThetaMax)
        {
            return 1.0f / (2.0f * M_PI * (1.0f - cosThetaMax));
        }
        else
        {
            return 0.0f;
        }
    }

    float evalSamplingPdf(uint i, float3 posW, float3 N, float3 L)
    {
        bool valid = isConeSamplingValid(i, posW);
        if(valid)
        {
            return evalConeSamplingPdf(i, posW, L);
        }
        else
        {
            return 0.5f * M_1_PI;
        }
    }

    float3 shadeSurfaceWithVSL(uint i, ShadingData sd, float3 Le, float3 L)
    {
        float radius = getVSLRadius(i);
        float3 vplNormal = getNormal(i);
        float invArea = 1.0f / (M_PI * radius * radius);

#if 1
        const float minDistance = 0.001f;
        const float minDistanceSquare = minDistance * minDistance;
        float3 toShadingPos = sd.posW - getPosition(i);
        if (dot(toShadingPos, toShadingPos) < minDistanceSquare)
        {
            return 0.0f;
        }
#endif
        SimpleBRDF brdf;
        brdf.setup(sd);        
        float3 wo = toLocal(sd.V, sd.N);
        float3 wi = toLocal(L, sd.N); 
        float3 brdfVal = brdf.eval(wo, wi);

        return brdfVal * Le * invArea* saturate(dot(vplNormal, -L));
    }


    float3 shadeSurfaceWithVSLNoCosine(uint i, ShadingData sd, float3 Le, float3 L)
    {
        float radius = getVSLRadius(i);
        float3 vplNormal = getNormal(i);
        float invArea = 1.0f / (M_PI * radius * radius);

#if 1
        const float minDistance = 0.001f;
        const float minDistanceSquare = minDistance * minDistance;
        float3 toShadingPos = sd.posW - getPosition(i);
        if (dot(toShadingPos, toShadingPos) < minDistanceSquare)
        {
            return 0.0f;
        }
#endif
        SimpleBRDF brdf;
        brdf.setup(sd);
        float3 wo = toLocal(sd.V, sd.N);
        float3 wi = toLocal(L, sd.N);
        float3 brdfVal = brdf.eval(wo, wi);

        return brdfVal * Le * invArea;
    }

// Polar Coord
#if 0
    float2 getUV(uint i, float3 L)
    {
        float3 vplNormal = getNormal(i);
        float radius = getVSLRadius(i);

        float right = float3(1.0, 0.0, 0.0);


        return 0.0f;
    }
    float3 getDirection(uint i, float2 uv)
    {
        return 0.0f;
    }
// Sphere Coord
#else
    float2 getUV(uint i, float3 L)
    {
        float3 vplNormal = getNormal(i);
        return cartesian_to_spherical_unorm(toLocal(L, vplNormal));
    }
    float3 getDirection(uint i, float2 uv)
    {
        float3 vplNormal = getNormal(i);
        float3 lightDir = spherical_unorm_to_cartesian(uv);
        return fromLocal(lightDir, vplNormal);
    }
#endif

    float evaluateVirtualLightSamplingPDF(ShadingData sd, int lightIndex, float3 L)
    {
#ifdef _VSL_EVALUATION
        return evalSamplingPdf(lightIndex, sd.posW, sd.N, L);
#else
        return 1.f;
#endif
    }


    float3 coneComputeShadingPointRadianceRandom(ShadingData sd, inout SampleGenerator sg, int minSampleCount, int sampleCount, int lightIndex, MegaTextureContainer specRadianceContainer)
    {
        return coneComputeShadingPointRadiance(sd, sg, minSampleCount, sampleCount, lightIndex, specRadianceContainer, true);
    }

    float3 coneComputeShadingPointRadiance(ShadingData sd, inout SampleGenerator sg, int minSampleCount, int sampleCount, int lightIndex, MegaTextureContainer specRadianceContainer, bool useRandom=false)
    {
#ifdef _VSL_EVALUATION
        float solidAngle;
        bool coneSampleValid = isConeSamplingValid(lightIndex, sd.posW, solidAngle);
        sampleCount = minSampleCount + int((sampleCount - minSampleCount) * solidAngle / (2*M_PI));
        float3 sum = 0.f;

#if SCATTER_VISIBILITY
        float vslRadius = getVSLRadius(lightIndex);
        float3 lightPos = getPosition(lightIndex);
        float tmax = length(lightPos - sd.posW) + vslRadius;
        float3 lightNormal = getNormal(lightIndex);
        float3 rayOrigin = computeRayOrigin(sd.posW, sd.frontFacing ? sd.faceN : -sd.faceN);
        float LPdf;

        for (int i = 0; i < sampleCount; i++)
        {
            float3 L;

            float2 VSLrand;

            if (useRandom) VSLrand = sampleNext2D(sg);
            else VSLrand = getHammersley(i, sampleCount);// plastic(sampleCount * lightIndex + i); //

            bool validDirection = true;
            if (!coneSampleValid)
            {
                L = HemisphereSampling(VSLrand, sd.N, LPdf);
            }
            else
            {
                coneSamplingVSL(lightIndex, VSLrand, sd.posW, L, LPdf);
                validDirection = dot(L, sd.N) > 0;
            }

            if (validDirection)
            {
                TriangleHit triangleHit;
                ShadingData nextSd;

                bool hit = castScene(rayOrigin, L, nextSd, triangleHit, tmax);
                if (hit)
                {
                    if (insideSphere(nextSd.posW, lightPos, vslRadius) && dot(nextSd.N, lightNormal) > 0.65f)
                    {
                        float3 Le = readLe(lightIndex, L, specRadianceContainer);
                        sum += shadeSurfaceWithVSLNoCosine(lightIndex, sd, Le, L);
                    }
                }
            }
        }
#else
        float LPdf;
        for (int i = 0; i < sampleCount; i++)
        {
            float3 L;

            float2 VSLrand;

            if (useRandom) VSLrand = sampleNext2D(sg);
            else VSLrand = getHammersley(i, sampleCount);// plastic(sampleCount * lightIndex + i); //

            if (coneSampleValid)
            {
                coneSamplingVSL(lightIndex, VSLrand, sd.posW, L, LPdf);
            }
            else // hemispherical sampling
            {
                L = HemisphereSampling(VSLrand, sd.N, LPdf);
            }

            float3 Le = readLe(lightIndex, L, specRadianceContainer);
            
            sum += shadeSurfaceWithVSL(lightIndex, sd, Le, L);
        }
#endif

        return sum / (sampleCount * LPdf);
#else
        float3 lightPos = getPosition(lightIndex);
        float3 L = normalize(lightPos - sd.posW);
        float3 Le = readLe(lightIndex, L, specRadianceContainer);
        return shadeSurface(lightIndex, sd, Le);
#endif
    }


    // cone sampling
    float3 coneSampleShadingPointRadiance(ShadingData sd, float2 VSLrand, int lightIndex, out float outPdf, out float3 outDir, MegaTextureContainer specRadianceContainer)
    {
#ifdef _VSL_EVALUATION
        float LPdf;
        float3 L;

        bool coneSampleValid = isConeSamplingValid(lightIndex, sd.posW);

#if SCATTER_VISIBILITY
        float vslRadius = getVSLRadius(lightIndex);
        float3 lightPos = getPosition(lightIndex);
        float tmax = length(lightPos - sd.posW) + vslRadius;
        float3 lightNormal = getNormal(lightIndex);
        float3 rayOrigin = computeRayOrigin(sd.posW, sd.frontFacing ? sd.faceN : -sd.faceN);
        float3 sum = 0.f;

        bool validDirection = true;
        if (!coneSampleValid)
        {
            L = HemisphereSampling(VSLrand, sd.N, LPdf);
        }
        else
        {
            coneSamplingVSL(lightIndex, VSLrand, sd.posW, L, LPdf);
            validDirection = dot(L, sd.N) > 0;
        }

        outPdf = LPdf;
        outDir = L;
        if (LPdf <= 0.0f)
        {
            return 0.f;
        }

        if (validDirection)
        {
            TriangleHit triangleHit;
            ShadingData nextSd;

            bool hit = castScene(rayOrigin, L, nextSd, triangleHit, tmax);
            if (hit)
            {
                if (insideSphere(nextSd.posW, lightPos, vslRadius) && dot(nextSd.N, lightNormal) > 0.65f)
                {
                    float3 Le = readLe(lightIndex, L, specRadianceContainer);
                    sum = shadeSurfaceWithVSLNoCosine(lightIndex, sd, Le, L);
                }
            }
        }

        return sum;
#else

        if (coneSampleValid)
        {
            coneSamplingVSL(lightIndex, VSLrand, sd.posW, L, LPdf);
        }
        else // hemispherical sampling
        {
            L = HemisphereSampling(VSLrand, sd.N, LPdf);
        }

        outPdf =  LPdf;
        outDir = L;
        if (LPdf <= 0.0f)
        {
            return 0.f;
        }

        float3 Le = readLe(lightIndex, L, specRadianceContainer);
        return shadeSurfaceWithVSL(lightIndex, sd, Le, L);
#endif

#else
        float3 lightPos = getPosition(lightIndex);
        float3 L = normalize(lightPos - sd.posW);
        float3 Le = readLe(lightIndex, L, specRadianceContainer);
        outDir = L;
        outPdf = 1;
        return shadeSurface(lightIndex, sd, Le);
#endif
    }

    float evaluateTileSamplingPDF(uint2 pixelPos, uint lightIndex, uint2 tileSize, uint2 tileDims, uint tileCapacity, StructuredBuffer<uint> tileContainer)
    {
        uint2 tilePos = pixelPos / tileSize;
        uint tileBaseIndex = (tileCapacity + 1) * (tilePos.y * tileDims.x + tilePos.x);
        uint tileSampleCount = tileContainer[tileBaseIndex + tileCapacity];

        bool found = false;
        for (int i = 0; i < tileSampleCount; i++)
        {
            if (lightIndex == tileContainer[tileBaseIndex + i])
            {
                found = true;
                break;
            }
        }

        return found ? 1.0f / tileSampleCount : 0.f;
    }

    float evaluateTileSamplingPDF(uint2 pixelPos, uint lightIndex, uint2 tileSize, uint searchRadius, uint2 tileDims, uint tileCapacity, StructuredBuffer<uint> tileContainer)
    {
        uint2 centerTile = pixelPos / tileSize;
        uint2 tileOffset = centerTile - searchRadius;
        uint2 tileEnds = tileOffset + searchRadius;
        tileOffset = max(uint2(0, 0), tileOffset);
        tileEnds = min(tileDims - 1, tileEnds);
        uint2 searchDims = tileEnds - tileOffset + 1;

        float pdf = 0.f;

        for (int i = 0; i < searchDims.y; i++)
        {
            for (int j = 0; j < searchDims.x; j++)
            {
                uint2 tilePos = tileOffset + uint2(j, i);
                uint tileBaseIndex = (tileCapacity + 1) * (tilePos.y * tileDims.x + tilePos.x);
                uint tileSampleCount = tileContainer[tileBaseIndex + tileCapacity];
                for (int k = 0; k < tileSampleCount; k++)
                {
                    uint curLightIndex = tileContainer[tileBaseIndex + k];
                    if (lightIndex == curLightIndex)
                    {
                        float weight = 1.f;// tileWeights[lightIndex];
                        pdf += weight * (1.0f / (searchDims.x * searchDims.y)) * (1.0f / tileSampleCount);

                        // this assumes no repeated VPLs
                        //pdf = weight * (1.0f / (searchDims.x * searchDims.y)) * (1.0f / tileSampleCount); 
                        //break;
                    }
                }
            }
        }

        return pdf;
    }

    uint getTileSampleCount(uint2 pixelPos, uint tileSize, uint2 tileDims, uint tileCapacity, StructuredBuffer<uint> tileContainer)
    {
        uint2 tilePos = pixelPos / tileSize;
        uint tileBaseIndex = (tileCapacity + 1) * (tilePos.y * tileDims.x + tilePos.x);
        uint tileSampleCount = tileContainer[tileBaseIndex + tileCapacity];
        return tileSampleCount;
    }

    bool sampleFromTile(float3 random, uint2 pixelPos, uint2 tileSize, uint searchRadius, uint2 tileDims, uint tileCapacity, StructuredBuffer<uint> tileContainer, out uint lightIndex, out float pdf)
    {
        uint2 centerTile = pixelPos / tileSize;
        uint2 tileOffset = centerTile - searchRadius;
        uint2 tileEnds = tileOffset + searchRadius;
        tileOffset = max(uint2(0, 0), tileOffset);
        tileEnds = min(uint2(tileDims - 1), tileEnds);

        uint2 searchDims = tileEnds - tileOffset + 1;
        uint2 tilePos = tileOffset + uint2(min(random.x * searchDims.x, searchDims.x - 1), min(random.y * searchDims.y, searchDims.y - 1));
        uint tileBaseIndex = (tileCapacity + 1) * (tilePos.y * tileDims.x + tilePos.x);
        uint tileSampleCount = tileContainer[tileBaseIndex + tileCapacity];
        uint sampleIndex = uint(min(random.z * tileSampleCount, tileSampleCount - 1));
        lightIndex = tileContainer[tileBaseIndex + sampleIndex];

        pdf = 0.f;

        if (tileSampleCount != 0)
        {
            pdf = evaluateTileSamplingPDF(pixelPos, lightIndex, tileSize, searchRadius, tileDims, tileCapacity, tileContainer);

            // this assumes no repeated VPLs
            //float weight = 1.f;
            //pdf = weight * (1.0f / (searchDims.x * searchDims.y)) * (1.0f / tileSampleCount);

            return true;
        }

        return false;
    }


    bool sampleFromTile(float3 random, uint2 tileDims, uint tileCapacity, StructuredBuffer<uint> tileContainer, StructuredBuffer<uint> tileWeights, out uint lightIndex, out float pdf)
    {
        uint2 tilePos = uint2(min(random.x * tileDims.x, tileDims.x - 1), min(random.y * tileDims.y, tileDims.y - 1));
        uint tileBaseIndex = (tileCapacity + 1) * (tilePos.y * tileDims.x + tilePos.x);
        uint tileSampleCount = tileContainer[tileBaseIndex + tileCapacity];
        uint sampleIndex = uint(min(random.z * tileSampleCount, tileSampleCount - 1));

        lightIndex = tileContainer[tileBaseIndex + sampleIndex];
        if (tileSampleCount != 0)
        {
            float weight = tileWeights[lightIndex];
            pdf = weight * (1.0f / (tileDims.x * tileDims.y)) * (1.0f / tileSampleCount);
            return true;
        }
        return false;
    }

    /*
    random: ramdom number [0, 1]
    pixelPos: 
    tileSize:
    tileDims: tile dimension
    tileCapcity: tile ray sample number

    return valid or not
    */
    bool sampleFromTile(float random, uint2 pixelPos, uint tileSize, uint2 tileDims, uint tileCapacity, StructuredBuffer<uint> tileContainer, out uint lightIndex, out float pdf)
    {
        uint2 tilePos = pixelPos / tileSize;
        uint tileBaseIndex = (tileCapacity + 1) * (tilePos.y * tileDims.x + tilePos.x);
        uint tileSampleCount = tileContainer[tileBaseIndex + tileCapacity];
        uint sampleIndex = uint(min(random * tileSampleCount, tileSampleCount - 1));
        pdf = 0.f;

        lightIndex = tileContainer[tileBaseIndex + sampleIndex];
        if (tileSampleCount != 0)
        {
            pdf = (1.0f / tileSampleCount);
            return true;
        }
        return false;
    }

    void sampleUniform(float random, out uint lightIndex, out float pdf)
    {
        lightIndex = uint(min(random * count, count - 1));
        pdf = (1.0f / count);
    }

    float3 readRadianceByPosition(float3 shadingPosition, float3 searchPos, float3 searchNormal, float3 vslPosition, float vslRadius, inout uint lightsFound, inout uint lightIndices[8], MegaTextureContainer megaTextureContainer)
    {
        lightsFound = 0;
        float3 Le = 0.0f;

        // if we are inside the VSL? (or a general query)
        if (vslRadius < 0.f || insideSphere(searchPos, vslPosition, vslRadius))
        {
            RayDesc searchRay;
            searchRay.Origin = searchPos;
            searchRay.Direction = float3(0.0f, 1.0f, 0.0f);
            searchRay.TMin = 0.0f;
            searchRay.TMax = 0.00001f;
            RayQuery<RAY_FLAG_NONE> rayQuery;
            rayQuery.TraceRayInline(as, RAY_FLAG_NONE, 0xff, searchRay);

            for (uint i = 0; i < maxSearchIterCount; i++)
            {
                if (!rayQuery.Proceed())
                {
                    break;
                }

                if (rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
                {
                    uint lightIndex = rayQuery.CandidatePrimitiveIndex();
                    float3 posW = getPosition(lightIndex);
                    float radius = getVSLRadius(lightIndex);
                    float disSquare = dot(searchPos - posW, searchPos - posW);
                    if (disSquare <= radius * radius)
                    {
                        lightIndices[lightsFound++] = lightIndex;
                        float3 L;
                        if (readRadianceByLightPosition)
                        {
                            L = posW - shadingPosition;
                        }
                        else
                        {
                            L = searchPos - shadingPosition;
                        }
                        float3 virtualLightN = getNormal(lightIndex);
                        if (!readRadianceCheckingNormal || dot(searchNormal, virtualLightN) > 0.7f)
                        {
                            float area = M_PI * radius * radius;
                            Le += (readLe(lightIndex, L, megaTextureContainer) / area);
                        }
                    }
                }

                if (i == (maxSearchIterCount - 1))
                {
                    rayQuery.Abort();
                }
            }
        }

        return Le;
    }

    float3 readRadianceByPositionExt(ShadingData sd, ShadingData nextSd, float3 vslPosition, float vslRadius, inout uint lightsFound, inout uint lightIndices[8], MegaTextureContainer megaTextureContainer)
    {
        lightsFound = 0;
        float3 Le = 0.0f;

        // if we are inside the VSL? (or a general query)
        if (vslRadius < 0.f || insideSphere(nextSd.posW, vslPosition, vslRadius))
        {
            RayDesc searchRay;
            searchRay.Origin = nextSd.posW;
            searchRay.Direction = float3(0.0f, 1.0f, 0.0f);
            searchRay.TMin = 0.0f;
            searchRay.TMax = 0.0f;
            RayQuery<RAY_FLAG_NONE> rayQuery;
            rayQuery.TraceRayInline(as, RAY_FLAG_NONE, 0xff, searchRay);

            for (uint i = 0; i < maxSearchIterCount; i++)
            {
                if (!rayQuery.Proceed())
                {
                    break;
                }

                if (rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
                {
                    uint lightIndex = rayQuery.CandidatePrimitiveIndex();
                    float3 lightPosW = getPosition(lightIndex);
                    float radius = getVSLRadius(lightIndex);
                    float disSquare = dot(nextSd.posW - lightPosW, nextSd.posW - lightPosW);
                    if (disSquare <= radius * radius)
                    {
                        // const float roughnessThreshold = 0.5f;
                        // const float reflectivityThreshold = 0.5f;
                        // const float albedoThreshold = 0.5f;

                        // if (!compareRelativeDifference(nextSd.linearRoughness, getRoughness(lightIndex), roughnessThreshold))
                        // {
                        //     continue;
                        // }

                        // if (!compareRelativeDifference(luminance(nextSd.specular), luminance(getSpecular(lightIndex)), reflectivityThreshold))
                        // {
                        //     continue;
                        // }

                        // if (!compareRelativeDifference(luminance(nextSd.diffuse), luminance(getDiffuse(lightIndex)), albedoThreshold))
                        // {
                        //     continue;
                        // }

                        float3 virtualLightN = getNormal(lightIndex);
                        if (!readRadianceCheckingNormal || dot(nextSd.N, virtualLightN) > 0.7f)
                        {
                            lightIndices[lightsFound++] = lightIndex;

                            float3 L = nextSd.posW - sd.posW;
                            float area = M_PI * radius * radius;
                            Le += (readLe(lightIndex, L, megaTextureContainer) / area);
                        }
                    }
                }

                if (i == (maxSearchIterCount - 1))
                {
                    rayQuery.Abort();
                }
            }
        }

        return Le;
    }


    float3 readRadianceByPosition(float3 shadingPosition, float3 searchPos, float3 searchNormal, MegaTextureContainer megaTextureContainer)
    {
        float3 Le = 0.0f;

        RayDesc searchRay;
        searchRay.Origin = searchPos;
        searchRay.Direction = float3(0.0f, 1.0f, 0.0f);
        searchRay.TMin = 0.0f;
        searchRay.TMax = 0.00001f;
        RayQuery<RAY_FLAG_NONE> rayQuery;
        rayQuery.TraceRayInline(as, RAY_FLAG_NONE, 0xff, searchRay);
        for (uint i = 0; i < maxSearchIterCount; i++)
        {
            if(!rayQuery.Proceed())
            {
                break;
            }
        
            if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
            {
                uint lightIndex = rayQuery.CandidatePrimitiveIndex();
                float3 posW = getPosition(lightIndex);
                float radius = getVSLRadius(lightIndex);
                float disSquare = dot(searchPos - posW, searchPos - posW);
                if(disSquare <= radius * radius)
                {
                    float3 L;
                    if (readRadianceByLightPosition)
                    {
                        L = posW - shadingPosition;
                    }
                    else
                    {
                        L = searchPos - shadingPosition;
                    }
                    float3 virtualLightN = getNormal(lightIndex);
                    if(!readRadianceCheckingNormal || dot(searchNormal, virtualLightN) > 0.7f)
                    {
                        float area = M_PI * radius * radius;
                        Le += (readLe(lightIndex, L, megaTextureContainer) / area);
                    }
                }
            }

            if (i == (maxSearchIterCount - 1))
            {
                rayQuery.Abort();
            }
        }

        return Le;
    }
};

inline float3 shadeSurfaceWithVSL(float3 Le, float3 vplNormal, float3 L, float radius)
{
    float invArea = 1.0f / (M_PI * radius * radius);
    return Le * invArea* saturate(dot(vplNormal, -L));
}

//cone sampling functions
bool isConeSamplingValid(float3 posW, float3 vplPos, float radius)
{
    float sinThethaSquare = radius * radius / dot(vplPos - posW, vplPos - posW);
    if(sinThethaSquare < 1.0f)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void coneSamplingVSL(float2 XY, float3 posW, float3 vplPos, float radius, out float3 LDir, out float pdf)
{
    float sinThethaSquare = radius * radius / dot(vplPos - posW, vplPos - posW);
    float cosThetaMax = sqrt(max(0.0f, 1.0f - sinThethaSquare));

    float uniformConePdf = 1.0f / (2.0f * M_PI * (1.0f - cosThetaMax));
    pdf = uniformConePdf;
        
    float cosTheta = (1.0f - XY.x) + XY.x * cosThetaMax;
    float sinTheta = sqrt(max(0.0f, 1.0f - cosTheta * cosTheta));
    float phi = XY.y * 2.0f * M_PI;

    float3 normal = normalize(vplPos - posW);
    float3 UpVector = abs(normal.z) < 0.999 ? float3(0, 0, 1) : float3(1, 0, 0);
    float3 TangentX = cross(UpVector, normal);
    float3 TangentY = cross(normal, TangentX);

    LDir = cos(phi) * sinTheta * TangentX + sin(phi) * sinTheta * TangentY + cosTheta * normal;
}

float evalConeSamplingPdf(float3 posW, float3 vplPos,float radius,float3 lightDir)
{
    float sinThethaSquare = radius * radius / dot(vplPos - posW, vplPos - posW);
    float cosThetaMax = sqrt(max(0.0f, 1.0f - sinThethaSquare));
    if(dot(lightDir, normalize(vplPos - posW)) >  cosThetaMax)
    {
        return 1.0f / (2.0f * M_PI * (1.0f - cosThetaMax));
    }
    else
    {
        return 0.0f;
    }
}
