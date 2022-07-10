import Utils.Sampling.SampleGenerator;
import Utils.Debug.PixelDebug;
#include "LightTreeDefinition.slangh"

struct LightCutsSample
{
    int index;
    float prob;
};

struct LightNode
{
    float3 boundMin;
    float intensity;
    float3 boundMax;
    int ID;
#ifdef LIGHT_CONE
    float4 cone;
#endif
};

static const int MAX_LIGHT_SAMPLES = 32;

inline float normalizedWeights(float l2_0, float l2_1, float intensGeom0, float intensGeom1)
{
	float ww0 = l2_1 * intensGeom0;
	float ww1 = l2_0 * intensGeom1;
	return ww0 / (ww0 + ww1);
};

inline float SquaredDistanceToClosestPoint(float3 p, float3 boundMin, float3 boundMax)
{
	float3 d = min(max(p, boundMin), boundMax) - p;
	return dot(d, d);
}

inline float SquaredDistanceToFarthestPoint(float3 p, float3 boundMin, float3 boundMax)
{
	float3 d = max(abs(boundMin - p), abs(boundMax - p));
	return dot(d, d);
}

float MaxDistAlong(float3 p, float3 dir, float3 boundMin, float3 boundMax)
{
	float3 dir_p = dir * p;
	float3 mx0 = dir * boundMin - dir_p;
	float3 mx1 = dir * boundMax - dir_p;
	return max(mx0[0], mx1[0]) + max(mx0[1], mx1[1]) + max(mx0[2], mx1[2]);
}

float GeomTermBoundApprox(float3 p, float3 N, float3 boundMin, float3 boundMax)
{
	float nrm_max = MaxDistAlong(p, N, boundMin, boundMax);
	if (nrm_max <= 0) return 0.0f;
	float3 d = min(max(p, boundMin), boundMax) - p;
	float3 tng = d - dot(d, N) * N;
	float hyp2 = dot(tng, tng) + nrm_max * nrm_max;
	return nrm_max * rsqrt(hyp2);
}

inline float AbsMinDistAlong(float3 p, float3 dir, float3 boundMin, float3 boundMax)
{
    bool hasPositive = false;
    bool hasNegative = false;
    float a = dot(dir, float3(boundMin.x, boundMin.y, boundMin.z) - p);
    float b = dot(dir, float3(boundMin.x, boundMin.y, boundMax.z) - p);
    float c = dot(dir, float3(boundMin.x, boundMax.y, boundMin.z) - p);
    float d = dot(dir, float3(boundMin.x, boundMax.y, boundMax.z) - p);
    float e = dot(dir, float3(boundMax.x, boundMin.y, boundMin.z) - p);
    float f = dot(dir, float3(boundMax.x, boundMin.y, boundMax.z) - p);
    float g = dot(dir, float3(boundMax.x, boundMax.y, boundMin.z) - p);
    float h = dot(dir, float3(boundMax.x, boundMax.y, boundMax.z) - p);
    hasPositive = a > 0 || b > 0 || c > 0 || d > 0 || e > 0 || f > 0 || g > 0 || h > 0;
    hasNegative = a < 0 || b < 0 || c < 0 || d < 0 || e < 0 || f < 0 || g < 0 || h < 0;
    if (hasPositive && hasNegative) return 0.f;
    else return min(min(min(abs(a), abs(b)), min(abs(c), abs(d))), min(min(abs(e), abs(f)), min(abs(g), abs(h))));
}

inline void CoordinateSystem_(float3 v1, out float3 v2, out float3 v3)
{
    if (abs(v1.x) > abs(v1.y)) v2 = float3(-v1.z, 0, v1.x) / sqrt(v1.x * v1.x + v1.z * v1.z);
    else v2 = float3(0, v1.z, -v1.y) / sqrt(v1.y * v1.y + v1.z * v1.z);
    v3 = normalize(cross(v1, v2));
}

inline float GeomTermBound(float3 p, float3 N, float3 boundMin, float3 boundMax)
{
    float nrm_max = MaxDistAlong(p, N, boundMin, boundMax);
    if (nrm_max <= 0) return 0.0f;
    float3 T, B;
    CoordinateSystem_(N, T, B);
    float y_amin = AbsMinDistAlong(p, T, boundMin, boundMax);
    float z_amin = AbsMinDistAlong(p, B, boundMin, boundMax);
    float hyp2 = y_amin * y_amin + z_amin * z_amin + nrm_max * nrm_max;
    return nrm_max * rsqrt(hyp2);
}


bool leftChildWeight(float3 shadingPos, float3 normal, int child0, int child1, inout float prob0, StructuredBuffer<LightNode> gNodeBuffer)
{
    // deal with bogus lights
    float intensity0 = gNodeBuffer[child0].intensity;
    float intensity1 = gNodeBuffer[child1].intensity;

    if (intensity0 == 0.0f)
	{
		if (intensity1 == 0.0f) 
        {
            return false;
        }
		prob0 = 0.0f;
		return true;
	}
	else if (intensity1 == 0.0f)
	{
		prob0 = 1.0f;
		return true;
	}
     
    float3 leftBoundMin = gNodeBuffer[child0].boundMin;
	float3 leftBoundMax = gNodeBuffer[child0].boundMax;
	float3 rightBoundMin = gNodeBuffer[child1].boundMin;
	float3 rightBoundMax = gNodeBuffer[child1].boundMax;

	// Compute the weights
	float geomTerm0 = GeomTermBound(shadingPos, normal, leftBoundMin, leftBoundMax);
	float geomTerm1 = GeomTermBound(shadingPos, normal, rightBoundMin, rightBoundMax);

#ifdef LIGHT_CONE
    float4 leftCone = gNodeBuffer[child0].cone;
    float4 rightCone = gNodeBuffer[child1].cone;
    float3 c0r_boundMin = 2 * shadingPos - leftBoundMax;
    float3 c0r_boundMax = 2 * shadingPos - leftBoundMin;
    float3 c1r_boundMin = 2 * shadingPos - rightBoundMax;
    float3 c1r_boundMax = 2 * shadingPos - rightBoundMin;
    float cos0 = GeomTermBound(shadingPos, leftCone.xyz, c0r_boundMin, c0r_boundMax);
    float cos1 = GeomTermBound(shadingPos, rightCone.xyz, c1r_boundMin, c1r_boundMax);
    geomTerm0 *= max(0.f, cos(max(0.f, acos(cos0) - leftCone.w)));
    geomTerm1 *= max(0.f, cos(max(0.f, acos(cos1) - rightCone.w)));
#endif

    if (geomTerm0 + geomTerm1 == 0.0f) 
    {
        return false;
    }

	if (geomTerm0 == 0.0f)
	{
		prob0 = 0.0f;
		return true;
	}
	else if (geomTerm1 == 0.0f)
	{
		prob0 = 1.0f;
		return true;
	}

    float intensGeom0 = gNodeBuffer[child0].intensity * geomTerm0;
	float intensGeom1 = gNodeBuffer[child1].intensity * geomTerm1;

    //deal with singularity
	float l2_min0 = SquaredDistanceToClosestPoint(shadingPos, leftBoundMin, leftBoundMax);
	float l2_min1 = SquaredDistanceToClosestPoint(shadingPos, rightBoundMin, rightBoundMax);
    float l2_max0 = SquaredDistanceToFarthestPoint(shadingPos, leftBoundMin, leftBoundMax);
	float l2_max1 = SquaredDistanceToFarthestPoint(shadingPos, rightBoundMin, rightBoundMax);

    float w_min0 = l2_min0 == 0.0f && l2_min1 == 0.0f ? intensGeom0 / (intensGeom0 + intensGeom1) : normalizedWeights(l2_min0, l2_min1, intensGeom0, intensGeom1);
	float w_max0 = normalizedWeights(l2_max0, l2_max1, intensGeom0, intensGeom1);
    prob0 = 0.5f * (w_max0 + w_min0);

    return true;
}

void sampleLightTree(uint nodeID, float3 shadingPos, float3 normal, inout SampleGenerator sg, out LightCutsSample lightCutsSample, uint leafStartIndex, StructuredBuffer<LightNode> gNodeBuffer)
{
    //begin with the root ndoe    
    double accumulateProb = 1.0;

    lightCutsSample.index = -1;

	while (nodeID < leafStartIndex) 
    {
        //print(321);
        //print(nodeID);

#ifdef UNBALANCED_TREE
        uint leftChild = gNodeBuffer[nodeID].ID;
        if (leftChild >= leafStartIndex) {
            break;
        }
#else
        uint leftChild = nodeID << 1;
#endif
        uint rightChild = leftChild + 1;

        float prob0;
		if (leftChildWeight(shadingPos, normal, leftChild, rightChild, prob0, gNodeBuffer)) 
		{
			if (sampleNext1D(sg) < prob0) 
			{
				nodeID = leftChild;
				accumulateProb *= prob0;
			}
			else 
			{
				nodeID = rightChild;
				accumulateProb *= (1.0f - prob0);
			}
		}
		else 
        {
			return;
		}

    }

#ifdef UNBALANCED_TREE
    lightCutsSample.index = gNodeBuffer[nodeID].ID - leafStartIndex;
#else
    lightCutsSample.index = gNodeBuffer[nodeID].ID;
#endif
    lightCutsSample.prob = (float)accumulateProb;
}


float errorFunction(int NodeID, const float3 posW, const float3 normalW, float SR2, uint leafStartIndex, StructuredBuffer<LightNode> gNodeBuffer)
{
    bool IsBLASInTwoLevelTree = false;

    LightNode node;

    // if it's leaf
#ifndef UNBALANCED_TREE
    if (NodeID >= leafStartIndex)
    {
        return 0;
    }
#endif
    node = gNodeBuffer[NodeID];
#ifdef UNBALANCED_TREE
    if (node.ID >= leafStartIndex)
    {
        return 0;
    }
#endif

    float dlen2 = SquaredDistanceToClosestPoint(posW, node.boundMin, node.boundMax);

    if (dlen2 < SR2) dlen2 = SR2; // bound the distance

    float atten = rcp(dlen2);

    atten *= GeomTermBound(posW, normalW, node.boundMin, node.boundMax);

#ifdef LIGHT_CONE
    {
        float3 nr_boundMin = 2 * posW - node.boundMax;
        float3 nr_boundMax = 2 * posW - node.boundMin;
        float cos0 = GeomTermBound(posW, node.cone.xyz, nr_boundMin, nr_boundMax);
        atten *= max(0.f, cos(max(0.f, acos(cos0) - node.cone.w)));
    }
#endif

    // this error function simplify treat BRDF bound as 1
    float colorIntens = node.intensity;
    float res = atten * colorIntens;
    return res;
};

struct OneLevelLightHeapSimpleData
{
    int NodeID;
    float error;
};


// TODO: implement LIGHT_CONE bound and BRDF bound
void sampleCut(const float3 posW, const float3 normalW, int maxLightSamples, inout SampleGenerator sg,
            inout LightCutsSample lightCutsSamples[MAX_LIGHT_SAMPLES], uint gLeafStartIndex, StructuredBuffer<LightNode> gNodeBuffer)
{
    // select a cut first
    int numLights = 1;

#define NodeID index
#define Error prob
#define heap lightCutsSamples
    //OneLevelLightHeapSimpleData heap[MAX_LIGHT_SAMPLES + 1];
    heap[0].NodeID = 1;
    heap[0].Error = 1e27;
    int maxId = 0;// 1;
    int lightcutNodes[MAX_LIGHT_SAMPLES];
    lightcutNodes[0] = 1;

    static const float errorLimit = 0.001f; // to tune

    float SR2 = errorLimit * length(gNodeBuffer[1].boundMax - gNodeBuffer[1].boundMin);
    SR2 *= SR2;

    while (numLights < maxLightSamples)
    {
        int id = maxId;
        int nodeID = heap[id].NodeID;

#ifdef UNBALANCED_TREE
        int pChild = gNodeBuffer[nodeID].ID;
#else
        int pChild = nodeID << 1;
#endif
        int sChild = pChild + 1;

        lightcutNodes[id] = pChild;
        heap[id].NodeID = pChild;
        heap[id].Error = errorFunction(pChild, posW, normalW, SR2, gLeafStartIndex, gNodeBuffer);

        // check bogus light
        if (gNodeBuffer[sChild].intensity > 0)
        {
            numLights++;
            lightcutNodes[numLights - 1] = sChild;
            heap[numLights - 1].NodeID = sChild;
            heap[numLights - 1].Error = errorFunction(sChild, posW, normalW, SR2, gLeafStartIndex, gNodeBuffer);
        }

        // find maxId
        float maxError = -1e10;
        for (int i = 0; i < numLights; i++)
        {
            if (heap[i].Error > maxError)
            {
                maxError = heap[i].Error;
                maxId = i;
            }
        }
        if (maxError <= 0) break;
    }

    //// write lightcut nodes
    int startId = 0;
    int endId = numLights;

    for (int i = 0; i < numLights; i++)
    {
        int nodeID = lightcutNodes[i];
        LightCutsSample lightSample;
        sampleLightTree(nodeID, posW, normalW, sg, lightSample, gLeafStartIndex, gNodeBuffer);
        lightCutsSamples[i] = lightSample;
    }
}

