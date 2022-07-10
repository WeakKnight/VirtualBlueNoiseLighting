/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#pragma once
#include "Falcor.h"
#include "FalcorExperimental.h"
#include "Utils/Sampling/SampleGenerator.h"
#include "Utils/VirtualLight/VirtualLightContainer.h"
#include "Timer.h"

using namespace Falcor;

class Partition
{
public:
    class Partition(float3& min, float3& max, int level, float ratio)
    {
        boundingBoxMin = min;
        boundingBoxMax = max;
        diagonalVector = max - min;
        mLevel = level;
        mRatio = ratio;
    }
    int3 putIntoPartition(float3& point)
    {
        float3 relativeTranslation = point - boundingBoxMin;
        float3 relativePos = relativeTranslation / diagonalVector * (float)(1 << mLevel);
        int3 gridIndex = (int3)relativePos;
        gridIndex = glm::clamp(gridIndex, 0, 1 << mLevel - 1);
        return gridIndex;

    }
    int linearIndexOfPartition(int3 gridIndex)
    {
        return gridIndex.x + gridIndex.y * (1 << mLevel) + gridIndex.z * (2 << mLevel);
    }

    //for non-uniform grid partition
    // Input: Point cloud, indices  ****** Output: two point cloud, two index arrays
    void NonUniformPartition(float3* pointCloud, std::vector<uint>& inputIndices, std::vector<uint>& ouputIndicesLeft, std::vector<uint>& ouputIndicesRight, bool printInfo = false)
    {
        int pointCount = inputIndices.size();
        std::vector<float3> pointCloudInCell;
        pointCloudInCell.reserve(pointCount);

        Timer tempTimer;
        if (printInfo)
            tempTimer.begin("NonUniformPartition Stage1");
        for (int i = 0; i < pointCount; i++)
        {
            pointCloudInCell.emplace_back(pointCloud[inputIndices.at(i)]);
        }

        std::vector<float> xValues(pointCount);
        std::vector<float> yValues(pointCount);
        std::vector<float> zValues(pointCount);

        for (int i = 0; i < pointCount; i++)
        {
            xValues.at(i) = pointCloudInCell[i].x;
        }
        for (int i = 0; i < pointCount; i++)
        {
            yValues.at(i) = pointCloudInCell[i].y;
        }
        for (int i = 0; i < pointCount; i++)
        {
            zValues.at(i) = pointCloudInCell[i].z;
        }

        if (printInfo)
        tempTimer.end();


        if (printInfo)
            tempTimer.begin("NonUniformPartition Stage2");
        std::sort(xValues.begin(),xValues.end());
        std::sort(yValues.begin(), yValues.end());
        std::sort(zValues.begin(), zValues.end());

        float xSize = xValues.at(xValues.size()-1) - xValues.at(0);
        float ySize = yValues.at(yValues.size()-1) - yValues.at(0);
        float zSize = zValues.at(zValues.size()-1) - zValues.at(0);

        float xMin = xValues.at(0);
        float yMin = yValues.at(0);
        float zMin = zValues.at(0);


        int medianIndex = int(pointCount / 2);

        float xPlane = xValues.at(medianIndex);
        float yPlane = yValues.at(medianIndex);
        float zPlane = zValues.at(medianIndex);

        
        float xPercent = (xPlane - xMin) / std::max(xSize, 1e-3f * diagonalVector.x);
        float yPercent = (yPlane - yMin) / std::max(ySize, 1e-3f * diagonalVector.y);
        float zPercent = (zPlane - zMin) / std::max(zSize, 1e-3f * diagonalVector.z);

        float disToXPlane = std::abs(xPercent - 0.5f);
        float disToYPlane = std::abs(yPercent - 0.5f);
        float disToZPlane = std::abs(zPercent - 0.5f);

        if (printInfo)
            tempTimer.end();


        if (printInfo)
            tempTimer.begin("NonUniformPartition Stage3");
        // saparate with xPlane
        if (disToXPlane <= disToYPlane && disToXPlane <= disToZPlane)
        {
            for (int i = 0; i < pointCount; i++)
            {
                if (pointCloudInCell.at(i).x > xPlane)
                {
                    ouputIndicesLeft.emplace_back(inputIndices[i]);
                }
                else
                {
                    ouputIndicesRight.emplace_back(inputIndices[i]);
                }

            }
            
        }
        else
        {
            //saparate with yPlane
            if (disToYPlane <= disToZPlane)
            {
                for (int i = 0; i < pointCount; i++)
                {
                    if (pointCloudInCell.at(i).y > yPlane)
                    {
                        ouputIndicesLeft.emplace_back(inputIndices[i]);
                    }
                    else
                    {
                        ouputIndicesRight.emplace_back(inputIndices[i]);
                    }

                }

            }
            //saparate with zPlane
            else
            {
                for (int i = 0; i < pointCount; i++)
                {
                    if (pointCloudInCell.at(i).z > zPlane)
                    {
                        ouputIndicesLeft.emplace_back(inputIndices[i]);
                    }
                    else
                    {
                        ouputIndicesRight.emplace_back(inputIndices[i]);
                    }

                }

            }
        }

        if (printInfo)
            tempTimer.end();
    }

    void NonUniformPartitionParallel(float3* pointCloud, std::vector<uint>& inputIndices, std::vector<uint>& ouputIndicesLeft, std::vector<uint>& ouputIndicesRight, bool printInfo = false);

    inline void setLevel(int level) { this->mLevel = level; }
    inline void setRatio(float ratio) { this->mRatio = ratio; }
    inline float getRatio() { return this->mRatio; }
    inline int getLevel() { return this->mLevel; }

private:
    float3 boundingBoxMin;
    float3 boundingBoxMax;
    float3 diagonalVector;
    int mLevel;
    float mRatio;
};

template <typename T>
struct PointCloud
{
    struct Point
    {
        T x, y, z;
    };

    using coord_t = T;  //!< The type of each coordinate

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return pts[idx].x;
        else if (dim == 1)
            return pts[idx].y;
        else
            return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard
    // bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned
    //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
    //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const
    {
        return false;
    }
};


class SampleEliminatePass : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<SampleEliminatePass>;
    static SharedPtr create(RenderContext* pRenderContext = nullptr, const Dictionary& dict = {});
    virtual std::string getDesc() override;
    virtual Dictionary getScriptingDictionary() override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pContext, const CompileData& compileData) override {}
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { return false; }
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override { return false; }

private:
    void eliminatie(RenderContext* pRenderContext, VirtualLightContainer::SharedPtr initialVirtualLights, uint targetCount, std::vector<uint>& outputIndices, std::vector<float3>& outputPositions, std::vector<float>& dmaxs);

    void eliminateWithPartition(int level, float percent, int inputCount, std::vector<float>& inputDmax, std::vector<float>& inputReverseDmax, Partition &mp, std::vector<uint>& inputIndices, float3* pointCloud, std::vector<uint>& outputIndices, std::vector<float3>* outputPositions = nullptr, std::vector<float>* dMaxArray = nullptr);

private:
    SampleEliminatePass() = default;
    float   mRatio = 0.2f;
    float   mRadiusSearchRange = 0.37f;
    uint    mRadiusSearchCount = 350;
    float   mRadius = 0.05f;
    bool    mUniformSE = false;
    float   mRadiusScalerForASBuilding = 1.5f;
    bool    mUseDMaxForASBuilding = false;
    bool    mUseParallelSE = false;
    bool    mFillTileHoles = true;
    bool    mUseSE = true;

    Scene::SharedPtr                    mpScene;
    SampleGenerator::SharedPtr          mpSampleGenerator;
    ComputePass::SharedPtr              mpComputePass;
    ComputePass::SharedPtr              mpValidationPass;
    ComputePass::SharedPtr              mpCountWeightPass;
    //ComputePass::SharedPtr              mpDilationPass;
    VirtualLightContainer::SharedPtr    mpSampleEliminatedVirtualLights;

    Buffer::SharedPtr                   mpTileVirtualLightWeights;
    std::vector<uint>                   mpSEHashMap;

    Threading mJobSystem;
};






