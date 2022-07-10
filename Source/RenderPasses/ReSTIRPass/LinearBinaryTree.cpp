#include "LinearBinaryTree.h"
#include <intrin.h>
#include <math.h>
#include "CPULightCuts.h"
#include "Utils/Math/PackedFormats.h"

// #define USE_TEST_DATA

struct MortonPair
{
    MortonPair(uint64_t pId, uint pMortenCode)
    {
        id = pId;
        mortenCode = pMortenCode;
    }

    uint64_t id;
    uint mortenCode;
};

struct MortonComparer
{
    inline bool operator() (const MortonPair& a, const MortonPair& b)
    {
        return (a.mortenCode < b.mortenCode);
    }
};

struct PackedBoundingBox
{
    float3 minPoint;
    float3 maxPoint;
    float Pad0;
    float Pad1;
    float3 getCenter() {
        return 0.5f * (minPoint + maxPoint);
    }
    float3 getHalfExtent() {
        return 0.5f * (maxPoint - minPoint);
    }
};


uint64_t Part1By2_64(uint64_t w) {
    w &= 0x00000000001fffff;
    w = (w | w << 32) & 0x001f00000000ffff;
    w = (w | w << 16) & 0x001f0000ff0000ff;
    w = (w | w << 8) & 0x010f00f00f00f00f;
    w = (w | w << 4) & 0x10c30c30c30c30c3;
    w = (w | w << 2) & 0x1249249249249249;
    return w;
}

uint64_t EncodeMorton3_64(uint64_t x, uint64_t y, uint64_t z)
{
    return (Part1By2_64(z) << 2) + (Part1By2_64(y) << 1) + Part1By2_64(x);
}

// "Insert" two 0 bits after each of the 10 low bits of x
uint Part1By2(uint x)
{
    x &= 0x000003ff;                  // x = ---- ---- ---- ---- ---- --98 7654 3210
    x = (x ^ (x << 16)) & 0xff0000ff; // x = ---- --98 ---- ---- ---- ---- 7654 3210
    x = (x ^ (x << 8)) & 0x0300f00f; // x = ---- --98 ---- ---- 7654 ---- ---- 3210
    x = (x ^ (x << 4)) & 0x030c30c3; // x = ---- --98 ---- 76-- --54 ---- 32-- --10
    x = (x ^ (x << 2)) & 0x09249249; // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
    return x;
}

uint EncodeMorton3(uint x, uint y, uint z)
{
    return (Part1By2(z) << 2) + (Part1By2(y) << 1) + Part1By2(x);
}

LightTree::SharedPtr LightTree::Create()
{
    LightTree::SharedPtr result = LightTree::SharedPtr(new LightTree());
    result->Init();
    return result;
}

void LightTree::BuildCPU(RenderContext* renderContext, VirtualLightContainer* pVPLContainer, Buffer::SharedPtr fluxBuffer, bool useBalancedTree, float VSLradius, uint balancedTreeQuantLevel, uint seed)
{
    if (useBalancedTree)
        BuildBalancedCPU(renderContext, pVPLContainer->getBoundingBoxBuffer(), pVPLContainer->getNormalBuffer(), fluxBuffer, pVPLContainer->getCount(), VSLradius, balancedTreeQuantLevel);
    else
        BuildUnbalancedCPU(renderContext, pVPLContainer->getBoundingBoxBuffer(), pVPLContainer->getNormalBuffer(), fluxBuffer, pVPLContainer->getCount(), VSLradius, seed);
}

void LightTree::BuildBalancedCPU(RenderContext* renderContext, Buffer::SharedPtr boundingBoxBuffer, Buffer::SharedPtr normalBuffer, Buffer::SharedPtr fluxBuffer, uint count, float VSLradius, uint balancedTreeQuantLevel)
{
    m_CPUData.clear();

    renderContext->flush(true);

#ifndef USE_TEST_DATA
    uint vplCount = count;
    uint treeLevelCount = int(std::ceil(std::log2(vplCount))) + 1;

    m_LeafStartIndex = pow(2, treeLevelCount - 1);

    Buffer::SharedPtr bboxReadBuffer = Buffer::create(boundingBoxBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read);
    Buffer::SharedPtr colorReadBuffer = Buffer::create(fluxBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read);
    Buffer::SharedPtr normalReadBuffer = Buffer::create(normalBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read);

    renderContext->copyBufferRegion(bboxReadBuffer.get(), 0, boundingBoxBuffer.get(), 0, boundingBoxBuffer->getSize());
    renderContext->copyBufferRegion(colorReadBuffer.get(), 0, fluxBuffer.get(), 0, fluxBuffer->getSize());
    renderContext->copyBufferRegion(normalReadBuffer.get(), 0, normalBuffer.get(), 0, normalBuffer->getSize());

    renderContext->flush(true);

    PackedBoundingBox* bboxs = (PackedBoundingBox*)bboxReadBuffer->map(Buffer::MapType::Read);
    float* flux = (float*)colorReadBuffer->map(Buffer::MapType::Read);
    uint* normal = (uint*)normalReadBuffer->map(Buffer::MapType::Read);

#else
    uint vplCount = 7;
    uint treeLevelCount = int(std::ceil(std::log2(vplCount))) + 1;

    // Build Fake Data
    float3* positions = new float3[7];
    float3* colors = new float3[7];

    for (uint i = 0; i < vplCount; i++)
    {
        positions[i] = float3(i, i, i);
        colors[i] = float3(i, i, i);
    }
#endif

    // Compute Total Bounding Box
    AABB bounds;
    for (uint i = 0; i < vplCount; i++)
    {
        float3 center = bboxs[i].getCenter();
        float3 halfExtent = bboxs[i].getHalfExtent();
        bounds.include(center - VSLradius * halfExtent);
        bounds.include(center + VSLradius * halfExtent);
    }

    // Gen Morton Code
    std::vector<MortonPair> sortedVPLIDs;
    sortedVPLIDs.reserve(vplCount);
    for (uint i = 0; i < vplCount; i++)
    {
        float3 pos = 0.5f * (bboxs[i].maxPoint + bboxs[i].minPoint);
        float3 normedRatio = (pos - bounds.minPoint) / bounds.extent();
        uint quantRes = pow(2, balancedTreeQuantLevel);
        uint3 quantitizedPos = uint3(quantRes * normedRatio.x, quantRes * normedRatio.y, quantRes * normedRatio.z);
        //uint mortenCode = EncodeMorton3(quantitizedPos.x, quantitizedPos.y, quantitizedPos.z);
        uint64_t mortenCode = EncodeMorton3_64(quantitizedPos.x, quantitizedPos.y, quantitizedPos.z);
        sortedVPLIDs.push_back(MortonPair(i, mortenCode));
    }

    // Sort
    std::sort(sortedVPLIDs.begin(), sortedVPLIDs.end(), MortonComparer());

    /*
    Build Tree
    */
    uint totalNodeCount = pow(2, treeLevelCount) - 1;
    m_CPUData.reserve(totalNodeCount);

    auto isLeaf = [=](uint index)
    {
        return index < pow(2, treeLevelCount - 1);
    };

    // Push Leaf Nodes
    for (uint i = 0; i < pow(2, treeLevelCount - 1); i++)
    {
        LightNode node;
        if (i < vplCount)
        {
            uint vplIndex = sortedVPLIDs[i].id;
            node.ID = vplIndex;

            float3 center = bboxs[vplIndex].getCenter();
            float3 halfExtent = bboxs[vplIndex].getHalfExtent();

            node.boundMin = center - VSLradius * halfExtent;
            node.boundMax = center + VSLradius * halfExtent;
            node.intensity = flux[vplIndex];
#ifdef LIGHT_CONE
            node.cone = float4(decodeNormal2x16(normal[vplIndex]), 0);
#endif
        }
        else
        {
            node.ID = -1;
            node.boundMin = float3(std::numeric_limits<float>::infinity());
            node.boundMax = float3(-std::numeric_limits<float>::infinity());
            node.intensity = 0.0f;
        }

        m_CPUData.push_back(node);
    }

    // Build Parent Nodes
    uint lastLevelStartIndex = 0;
    for (uint i = 2; i <= treeLevelCount; i++)
    {
        uint thisLevelCount = pow(2, treeLevelCount - i);
        for (uint j = 0; j < thisLevelCount; j++)
        {
            LightNode leftNode = m_CPUData[lastLevelStartIndex + uint(j * 2) + 0];
            LightNode rightNode = m_CPUData[lastLevelStartIndex + uint(j * 2) + 1];

            LightNode node;
            node.ID = -1;
            node.boundMin = min(leftNode.boundMin, rightNode.boundMin);
            node.boundMax = max(leftNode.boundMax, rightNode.boundMax);
            node.intensity = leftNode.intensity + rightNode.intensity;
#ifdef LIGHT_CONE
            node.cone = LightCuts::MergeCones(leftNode.cone, rightNode.cone);
#endif
            m_CPUData.push_back(node);
        }

        lastLevelStartIndex += (thisLevelCount * 2);
    }

    // Reverse The Order And Add A Place Holder In The First
    LightNode node;
    node.ID = -1;
    node.boundMin = float3(std::numeric_limits<float>::infinity());
    node.boundMax = float3(-std::numeric_limits<float>::infinity());
    node.intensity = 0.0f;

    m_CPUData.push_back(node);
    std::reverse(m_CPUData.begin(), m_CPUData.end());

#ifndef USE_TEST_DATA
    bboxReadBuffer->unmap();
    colorReadBuffer->unmap();
#endif

    m_NodeBuffer = Buffer::create(sizeof(LightNode) * m_CPUData.size(), ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, m_CPUData.data());

#ifdef USE_TEST_DATA
    Buffer::SharedPtr nodeReadBuffer = Buffer::create(sizeof(Node) * m_CPUData.size(), ResourceBindFlags::None, Buffer::CpuAccess::Read);
    renderContext->copyBufferRegion(nodeReadBuffer.get(), 0, m_NodeBuffer.get(), 0, m_NodeBuffer->getSize());
    renderContext->flush(true);

    Node* nodes = (Node*)nodeReadBuffer->map(Buffer::MapType::Read);
    for (uint i = 0; i < m_CPUData.size(); i++)
    {
        Node node = nodes[i];
        std::cout << "ID: " << node.ID << std::endl;
        std::cout << "Intensity: " << node.intensity << std::endl;
        std::cout << "Min: " << node.boundMin.x << ", " << node.boundMin.y << ", " << node.boundMin.z << std::endl;
        std::cout << "Max: " << node.boundMax.x << ", " << node.boundMax.y << ", " << node.boundMax.z << std::endl;
    }
    nodeReadBuffer->unmap();
#endif
}


void LightTree::GenerateLevelIds(const std::vector<LightNode>& nodes, std::vector<int>& levelIds, int curId, int offset, int leafStartIndex, int curLevel)
{
    levelIds[offset + curId] = curLevel;
    const LightNode& node = nodes[offset + curId];
    int leftChild = node.ID;
    int rightChild = node.ID + 1;
    if (leftChild >= leafStartIndex || rightChild >= leafStartIndex) return;

    GenerateLevelIds(nodes, levelIds, leftChild, offset, leafStartIndex, curLevel + 1);
    GenerateLevelIds(nodes, levelIds, rightChild, offset, leafStartIndex, curLevel + 1);
}

void LightTree::BuildUnbalancedCPU(RenderContext* renderContext, Buffer::SharedPtr boundingBoxBuffer, Buffer::SharedPtr normalBuffer, Buffer::SharedPtr fluxBuffer, uint count, float VSLradius, uint seed)
{
    m_CPUData.clear();
    typedef std::default_random_engine RNG;
    std::uniform_real_distribution<float> distribution;

    Buffer::SharedPtr colorReadBuffer = Buffer::create(fluxBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read);
    Buffer::SharedPtr bboxReadBuffer = Buffer::create(boundingBoxBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read);
    Buffer::SharedPtr normalReadBuffer = Buffer::create(normalBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read);

    renderContext->copyBufferRegion(bboxReadBuffer.get(), 0, boundingBoxBuffer.get(), 0, boundingBoxBuffer->getSize());
    renderContext->copyBufferRegion(colorReadBuffer.get(), 0, fluxBuffer.get(), 0, fluxBuffer->getSize());
    renderContext->copyBufferRegion(normalReadBuffer.get(), 0, normalBuffer.get(), 0, normalBuffer->getSize());

    renderContext->flush(true);

    PackedBoundingBox* bboxs = (PackedBoundingBox*)bboxReadBuffer->map(Buffer::MapType::Read);
    float* flux = (float*)colorReadBuffer->map(Buffer::MapType::Read);
    uint* normal = (uint*)normalReadBuffer->map(Buffer::MapType::Read);

    int numNodes = 2 * count;

    m_LeafStartIndex = numNodes; // mark invalid child node (leaf node)

    m_CPUData.resize(numNodes);

    LightCuts cpuLightCuts;

    RNG sampler;
    sampler.seed(seed);
    //sampler.seed(frameId);

    cpuLightCuts.Build(count, [&](int i) {return float3(flux[i], 0, 0); },
        [&](int i) {return bboxs[i].getCenter(); },
#ifdef LIGHT_CONE
        [&](int i) {return float4(decodeNormal2x16(normal[i]), 0); },
#else
        [&](int i) {},
#endif
        [&](int i) {

            float3 center = bboxs[i].getCenter();
            float3 halfExtent = bboxs[i].getHalfExtent();
            return AABB(center - VSLradius * halfExtent, center + VSLradius * halfExtent);
        },
        [&]() {return distribution(sampler); });

    for (int i = 1; i < numNodes; i++)
    {
        LightCuts::Node curnode = cpuLightCuts.GetNode(i - 1);
        m_CPUData[i].boundMin = curnode.boundBox.minPoint;
        m_CPUData[i].boundMax = curnode.boundBox.maxPoint;
        m_CPUData[i].intensity = curnode.probTree;
        m_CPUData[i].ID = curnode.primaryChild;
#ifdef LIGHT_CONE
        m_CPUData[i].cone = curnode.boundingCone;
#endif
    }

    m_NodeBuffer = Buffer::create(sizeof(LightNode) * m_CPUData.size(), ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, m_CPUData.data());
}

void LightTree::BuildCPUForRichVPL(RenderContext* renderContext, Buffer::SharedPtr positionBuffer, Buffer::SharedPtr powerBuffer, uint count, float VSLradius)
{
    m_CPUData.clear();

    renderContext->flush(true);

#ifndef USE_TEST_DATA
    uint vplCount = count;
    uint treeLevelCount = int(std::ceil(std::log2(vplCount))) + 1;

    m_LeafStartIndex = pow(2, treeLevelCount - 1);

    Buffer::SharedPtr positionReadBuffer = Buffer::create(positionBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read);
    Buffer::SharedPtr powerReadBuffer = Buffer::create(powerBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::Read);

    renderContext->copyBufferRegion(positionReadBuffer.get(), 0, positionBuffer.get(), 0, positionBuffer->getSize());
    renderContext->copyBufferRegion(powerReadBuffer.get(), 0, powerBuffer.get(), 0, powerBuffer->getSize());

    renderContext->flush(true);

    float3* positions = (float3*)positionReadBuffer->map(Buffer::MapType::Read);
    float* powers = (float*)powerReadBuffer->map(Buffer::MapType::Read);
#else
    uint vplCount = 7;
    uint treeLevelCount = int(std::ceil(std::log2(vplCount))) + 1;

    // Build Fake Data
    float3* positions = new float3[7];
    float3* colors = new float3[7];

    for (uint i = 0; i < vplCount; i++)
    {
        positions[i] = float3(i, i, i);
        colors[i] = float3(i, i, i);
    }
#endif

    // Compute Total Bounding Box
    AABB bounds;
    for (uint i = 0; i < vplCount; i++)
    {
        bounds.include(positions[i]);
    }

    // Gen Morton Code
    std::vector<MortonPair> sortedVPLIDs;
    sortedVPLIDs.reserve(vplCount);

    for (uint i = 0; i < vplCount; i++)
    {
        float3 pos = positions[i];
        float3 normedRatio = (pos - bounds.minPoint) / bounds.extent();
        uint3 quantitizedPos = uint3(65535U * normedRatio.x, 65535U * normedRatio.y, 65535U * normedRatio.z);
        uint mortenCode = EncodeMorton3(quantitizedPos.x, quantitizedPos.y, quantitizedPos.z);
        sortedVPLIDs.push_back(MortonPair(i, mortenCode));
    }

    // Sort
    std::sort(sortedVPLIDs.begin(), sortedVPLIDs.end(), MortonComparer());

    /*
    Build Tree
    */
    uint totalNodeCount = pow(2, treeLevelCount) - 1;
    m_CPUData.reserve(totalNodeCount);

    auto isLeaf = [=](uint index)
    {
        return index < pow(2, treeLevelCount - 1);
    };

    // Push Leaf Nodes
    for (uint i = 0; i < pow(2, treeLevelCount - 1); i++)
    {
        LightNode node;
        if (i < vplCount)
        {
            uint vplIndex = sortedVPLIDs[i].id;
            node.ID = vplIndex;
            node.boundMin = positions[vplIndex];
            node.boundMax = positions[vplIndex];
            node.intensity = powers[vplIndex];
        }
        else
        {
            node.ID = -1;
            node.boundMin = float3(std::numeric_limits<float>::infinity());
            node.boundMax = float3(-std::numeric_limits<float>::infinity());
            node.intensity = 0.0f;
        }

        m_CPUData.push_back(node);
    }

    // Build Parent Nodes
    uint lastLevelStartIndex = 0;
    for (uint i = 2; i <= treeLevelCount; i++)
    {
        uint thisLevelCount = pow(2, treeLevelCount - i);
        for (uint j = 0; j < thisLevelCount; j++)
        {
            LightNode leftNode = m_CPUData[lastLevelStartIndex + uint(j * 2) + 0];
            LightNode rightNode = m_CPUData[lastLevelStartIndex + uint(j * 2) + 1];

            LightNode node;
            node.ID = -1;
            node.boundMin = min(leftNode.boundMin, rightNode.boundMin);
            node.boundMax = max(leftNode.boundMax, rightNode.boundMax);
            node.intensity = leftNode.intensity + rightNode.intensity;

            m_CPUData.push_back(node);
        }

        lastLevelStartIndex += (thisLevelCount * 2);
    }

    // Reverse The Order And Add A Place Holder In The First
    LightNode node;
    node.ID = -1;
    node.boundMin = float3(std::numeric_limits<float>::infinity());
    node.boundMax = float3(-std::numeric_limits<float>::infinity());
    node.intensity = 0.0f;

    m_CPUData.push_back(node);
    std::reverse(m_CPUData.begin(), m_CPUData.end());

#ifndef USE_TEST_DATA
    positionReadBuffer->unmap();
    powerReadBuffer->unmap();
#endif

    m_NodeBuffer = Buffer::create(sizeof(LightNode) * m_CPUData.size(), ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, m_CPUData.data());

#ifdef USE_TEST_DATA
    Buffer::SharedPtr nodeReadBuffer = Buffer::create(sizeof(Node) * m_CPUData.size(), ResourceBindFlags::None, Buffer::CpuAccess::Read);
    renderContext->copyBufferRegion(nodeReadBuffer.get(), 0, m_NodeBuffer.get(), 0, m_NodeBuffer->getSize());
    renderContext->flush(true);

    Node* nodes = (Node*)nodeReadBuffer->map(Buffer::MapType::Read);
    for (uint i = 0; i < m_CPUData.size(); i++)
    {
        Node node = nodes[i];
        std::cout << "ID: " << node.ID << std::endl;
        std::cout << "Intensity: " << node.intensity << std::endl;
        std::cout << "Min: " << node.boundMin.x << ", " << node.boundMin.y << ", " << node.boundMin.z << std::endl;
        std::cout << "Max: " << node.boundMax.x << ", " << node.boundMax.y << ", " << node.boundMax.z << std::endl;
    }
    nodeReadBuffer->unmap();
#endif

    renderContext->uavBarrier(m_NodeBuffer.get());
}

struct Node
{
    uint index;
    float prob;
    Node(uint index, float prob)
    {
        this->index = index;
        this->prob = prob;
    }
    Node(const Node& n)
    {
        this->index = n.index;
        this->prob = n.prob;
    }
};

void LightTree::Init()
{

}

struct AliasStruct
{
    int index;
    int alias;
    float pdf;
    float probs;

    AliasStruct(int index, int alias, float pdf, float probs)
    {
        this->index = index;
        this->alias = alias;
        this->pdf = pdf;
        this->probs = probs;
    }
    AliasStruct(const AliasStruct& n)
    {
        this->index = n.index;
        this->alias = n.alias;
        this->pdf = n.pdf;
        this->probs = n.probs;
    }
};
#define LIGHT_GROUP_COUNT 256

void LightTree::BuildAliasTable(RenderContext* renderContext)
{
    
    std::vector<uint> emissiveLightNodes;
    emissiveLightNodes.reserve(LIGHT_GROUP_COUNT);

    for (int i = LIGHT_GROUP_COUNT; i < LIGHT_GROUP_COUNT * 2; i++)
    {
        if (m_CPUData[i].intensity > 0.0f)
        {
            emissiveLightNodes.push_back(i);
        }
    }

    float fluxSum = 0.0f;
    for (int i = 0; i < emissiveLightNodes.size(); i++)
    {
        fluxSum += m_CPUData[emissiveLightNodes[i]].intensity;
    }

    uint num = (uint)emissiveLightNodes.size();
    m_AliasCount = num;
    AliasStruct initValue(-1, -1, 0.0f, 0.0f);
    std::vector<AliasStruct>  aliasStructArray(num, initValue);


    for (uint i = 0; i < num; i++)
    {
        aliasStructArray[i].pdf = m_CPUData[emissiveLightNodes[i]].intensity / fluxSum;
        aliasStructArray[i].probs = aliasStructArray[i].pdf * num;
        aliasStructArray[i].index = emissiveLightNodes[i];
    }

    std::vector<Node> Large, Small;
    for (uint i = 0; i < num; i++)
    {
        Node n(i, aliasStructArray.at(i).probs);
        if (aliasStructArray.at(i).probs >= 1.0f)
            Large.push_back(n);
        else
            Small.push_back(n);
    }

    while (Large.size() > 0 && Small.size() > 0)
    {
        Node n1 = Small.back();
        Small.pop_back();
        Node n2 = Large.back();
        Large.pop_back();
        float pg = n1.prob + n2.prob - 1.0f;
        aliasStructArray.at(n1.index).alias = n2.index;

        n2.prob -= 1.0f - n1.prob;

        if (pg < 1.0f)
        {
            aliasStructArray.at(n2.index).probs = n2.prob;
            Small.push_back(n2);
        }
        else
        {
            Large.push_back(n2);
        }
    }

    if (Large.size() > 0)
    {
        for (auto it : Large)
            aliasStructArray.at(it.index).probs = 1.0f;
    }
    if (Small.size() > 0)
    {
        for (auto it : Small)
            aliasStructArray.at(it.index).probs = 1.0f;
    }

    if (aliasStructArray.size() == 0)
    {
        m_AliasTableBuffer = Buffer::createStructured(sizeof(AliasStruct), 1, Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess, Buffer::CpuAccess::None, nullptr);
    }
    else
    {
        m_AliasTableBuffer = Buffer::createStructured(sizeof(AliasStruct), num, Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess, Buffer::CpuAccess::None, aliasStructArray.data());
    }

    renderContext->uavBarrier(m_AliasTableBuffer.get());
    std::cout << "Emissive Light Tree Count " << emissiveLightNodes.size() << std::endl;
}
