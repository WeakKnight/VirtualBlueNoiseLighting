#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;

struct ResamplingRuntimeParameters
{
    uint tileCount;
    uint tileSize;
    uint reservoirBlockRowPitch;
    uint reservoirArrayPitch;
};

struct Reservoir
{
    // Light index (bits 0..30) and validity bit (31)
    uint lightData;

    // Sample UV encoded in 16-bit fixed point format
    uint uvData;

    // Overloaded: represents RIS weight sum during streaming,
    // then reservoir weight (inverse PDF) after FinalizeResampling
    float weightSum;

    // Target PDF of the selected sample
    float targetPdf;

    // Number of samples considered for this reservoir
    uint M;

    // Light index helpers
    static const uint c_MaxM = 0x3fff;
    static const uint c_LightValidBit = 0x80000000;
    static const uint c_LightIndexMask = 0x7FFFFFFF;
};

uint reservoirPositionToPointer(
    ResamplingRuntimeParameters params,
    uint2 reservoirPosition,
    uint reservoirArrayIndex,
    )
{
    uint2 blockIdx = reservoirPosition / 16;
    uint2 positionInBlock = reservoirPosition % 16;

    return reservoirArrayIndex * params.reservoirArrayPitch
        + blockIdx.y * params.reservoirBlockRowPitch
        + blockIdx.x * (16 * 16)
        + positionInBlock.y * 16
        + positionInBlock.x;
}


void setReservoirWeight(
    ResamplingRuntimeParameters params,
    RWStructuredBuffer<Reservoir> LightReservoirs,
    uint2 reservoirPosition,
    uint reservoirArrayIndex,
    float weight)
{
    uint pointer = reservoirPositionToPointer(params, reservoirPosition, reservoirArrayIndex);
    LightReservoirs[pointer].weightSum = weight;
}

void storeReservoir(
    ResamplingRuntimeParameters params,
    const Reservoir reservoir,
    RWStructuredBuffer<Reservoir> LightReservoirs,
    uint2 reservoirPosition,
    uint reservoirArrayIndex)
{
    uint pointer = reservoirPositionToPointer(params, reservoirPosition, reservoirArrayIndex);
    LightReservoirs[pointer] = reservoir;
}

Reservoir loadReservoir(
    ResamplingRuntimeParameters params,
    RWStructuredBuffer<Reservoir> LightReservoirs,
    uint2 reservoirPosition,
    uint reservoirArrayIndex)
{
    uint pointer = reservoirPositionToPointer(params, reservoirPosition, reservoirArrayIndex);
    return LightReservoirs[pointer];
}

void samplePdfMipmap(
    inout SampleGenerator rng, 
    Texture2D<float> pdfTexture, // full mip chain starting from unnormalized sampling pdf in mip 0
    uint2 pdfTextureSize,        // dimensions of pdfTexture at mip 0; must be 16k or less
    out uint2 position,
    out float pdf)
{
    int lastMipLevel = max(0, int(floor(log2(max(pdfTextureSize.x, pdfTextureSize.y)))) - 1);

    position = 0;
    pdf = 1.0;
    for (int mipLevel = lastMipLevel; mipLevel >= 0; mipLevel--)
    {
        position *= 2;

        float4 samples; // there's no version of Gather that supports mipmaps, really?
        // samples.x = max(0, pdfTexture.mips[mipLevel][position + uint2(0, 0)].x);
        // samples.y = max(0, pdfTexture.mips[mipLevel][position + uint2(0, 1)].x);
        // samples.z = max(0, pdfTexture.mips[mipLevel][position + uint2(1, 0)].x);
        // samples.w = max(0, pdfTexture.mips[mipLevel][position + uint2(1, 1)].x);

        samples.x = max(0, pdfTexture.Load(int3(position + uint2(0, 0), mipLevel)).x);
        samples.y = max(0, pdfTexture.Load(int3(position + uint2(0, 1), mipLevel)).x);
        samples.z = max(0, pdfTexture.Load(int3(position + uint2(1, 0), mipLevel)).x);
        samples.w = max(0, pdfTexture.Load(int3(position + uint2(1, 1), mipLevel)).x);

        float weightSum = samples.x + samples.y + samples.z + samples.w;
        if (weightSum <= 0)
        {
            pdf = 0;
            return;
        }

        samples /= weightSum;

        float rnd = sampleNext1D(rng);
        
        int2 selectedOffset;

        if (rnd < samples.x)
        { 
            pdf *= samples.x;
        }
        else
        {
            rnd -= samples.x;
            
            if (rnd < samples.y)
            {
                position += uint2(0, 1);
                pdf *= samples.y;
            }
            else
            {
                rnd -= samples.y;

                if (rnd < samples.z)
                {
                    position += uint2(1, 0);
                    pdf *= samples.z;
                }
                else
                {
                    position += uint2(1, 1);
                    pdf *= samples.w;
                }
            }
        }
    }
}

Reservoir emptyReservoir()
{
    Reservoir s;
    s.lightData = 0;
    s.uvData = 0;
    s.targetPdf = 0;
    s.weightSum = 0;
    s.M = 0;
    return s;
}

bool isValidReservoir(const Reservoir reservoir)
{
    return reservoir.lightData != 0;
}

uint getReservoirLightIndex(const Reservoir reservoir)
{
    return reservoir.lightData & Reservoir::c_LightIndexMask;
}

uint packUV(float2 uv)
{
    return uint(saturate(uv.x) * 0xffff) | (uint(saturate(uv.y) * 0xffff) << 16);
}

float2 unpackUV(uint uvData)
{
    return float2(uvData & 0xffff, uvData >> 16) / float(0xffff);
}

float2 getReservoirSampleUV(const Reservoir reservoir)
{
    return unpackUV(reservoir.uvData);
}

float getReservoirInvPdf(const Reservoir reservoir)
{
    return reservoir.weightSum;
}

// Adds a new, non-reservoir light sample into the reservoir, returns true if this sample was selected.
// Algorithm (3) from the ReSTIR paper, Streaming RIS using weighted reservoir sampling.
bool streamSample(
    inout Reservoir reservoir,
    uint lightIndex,
    float2 uv,
    float random,
    float targetPdf,
    float invSourcePdf)
{
    // What's the current weight
    float risWeight = targetPdf * invSourcePdf;

    // Add one sample to the counter
    reservoir.M += 1;

    // Update the weight sum
    reservoir.weightSum += risWeight;

    // Decide if we will randomly pick this sample
    bool selectSample = (random * reservoir.weightSum < risWeight);

    // If we did select this sample, update the relevant data.
    // New samples don't have visibility or age information, we can skip that.
    if (selectSample) 
    {
        reservoir.lightData = lightIndex | Reservoir::c_LightValidBit;
        reservoir.uvData = packUV(uv);
        reservoir.targetPdf = targetPdf;
    }

    return selectSample;
}

// Adds `newReservoir` into `reservoir`, returns true if the new reservoir's sample was selected.
// Algorithm (4) from the ReSTIR paper, Combining the streams of multiple reservoirs.
// Normalization - Equation (6) - is postponed until all reservoirs are combined.
bool combineReservoirs(
    inout Reservoir reservoir,
    const Reservoir newReservoir,
    float random,
    float targetPdf,
    float misWeight = 1.f)
{
    // What's the current weight (times any prior-step RIS normalization factor)
    float risWeight = targetPdf * newReservoir.weightSum * newReservoir.M * misWeight;

    // Our *effective* candidate pool is the sum of our candidates plus those of our neighbors
    reservoir.M += newReservoir.M;

    // Update the weight sum
    reservoir.weightSum += risWeight;

    // Decide if we will randomly pick this sample
    bool selectSample = (random * reservoir.weightSum < risWeight);

    // If we did select this sample, update the relevant data
    if (selectSample) 
    {
        reservoir.lightData = newReservoir.lightData;
        reservoir.uvData = newReservoir.uvData;
        reservoir.targetPdf = targetPdf;
    }

    return selectSample;
}

// Performs normalization of the reservoir after streaming. Equation (6) from the ReSTIR paper.
void finalizeResampling(
    inout Reservoir reservoir,
    float normalizationNumerator,
    float normalizationDenominator)
{
    float denominator = reservoir.targetPdf * normalizationDenominator;

    reservoir.weightSum = (denominator == 0.0) ? 0.0 : (reservoir.weightSum * normalizationNumerator) / denominator;
}
