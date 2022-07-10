#pragma once
#include "Falcor.h"
#include "Utils/VirtualLight/VirtualLightContainer.h"
#include "LightTreeDefinition.slangh"

using namespace Falcor;

class LightTree : public std::enable_shared_from_this<LightTree>
{
public:
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

    using SharedPtr = std::shared_ptr<LightTree>;
    static LightTree::SharedPtr Create();

    void BuildCPU(RenderContext* renderContext, VirtualLightContainer* pVPLContainer, Buffer::SharedPtr fluxBuffer, bool useBalancedTree, float VSLradius, uint balancedTreeQuantizeLevel, uint seed=123);

    void BuildBalancedCPU(RenderContext* renderContext, Buffer::SharedPtr positionBuffer, Buffer::SharedPtr normalBuffer, Buffer::SharedPtr fluxBuffer, uint count, float VSLradius, uint balancedTreeQuantLevel);

    void BuildUnbalancedCPU(RenderContext* renderContext, Buffer::SharedPtr positionBuffer, Buffer::SharedPtr normalBuffer, Buffer::SharedPtr fluxBuffer, uint count, float VSLradius, uint seed);

    void BuildCPUForRichVPL(RenderContext* renderContext, Buffer::SharedPtr positionBuffer, Buffer::SharedPtr powerBuffer, uint count, float VSLradius);

    //void BuildAliasTableForRIS(RenderContext* renderContext);
    void BuildAliasTable(RenderContext* renderContext);

    inline Buffer::SharedPtr GetNodeBuffer() const { return m_NodeBuffer; };
    inline uint GetNodeCount() const { return m_CPUData.size(); }
    inline uint GetLeafStartIndex() const{return m_LeafStartIndex;}

    inline Buffer::SharedPtr GetAliasTableBuffer() const { return m_AliasTableBuffer; };
    inline uint GetAliasCount() const { return m_AliasCount; }

    //inline Buffer::SharedPtr GetRISAliasTableBuffer() const { return m_RISAliasTableBuffer; };
    //inline uint GetRISAliasCount() const { return m_RISAliasCount; }

private:
    void Init();
    void GenerateLevelIds(const std::vector<LightNode>& nodes, std::vector<int>& levelIds, int curId, int offset, int leafStartIndex, int curLevel);

    std::vector<LightNode> m_CPUData;
    Buffer::SharedPtr m_NodeBuffer;
    uint m_LeafStartIndex;
    Buffer::SharedPtr m_AliasTableBuffer;
    uint m_AliasCount;

    //Buffer::SharedPtr m_RISAliasTableBuffer;
    //uint m_RISAliasCount;
};

