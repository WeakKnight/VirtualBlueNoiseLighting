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
import Utils.Math.MathHelpers;
import Utils.Math.FormatConversion;
import Utils.Color.ColorHelpers;

// Helper function to reflect the folds of the lower hemisphere
// over the diagonals in the octahedral map
float2 octWrap(float2 v)
{
    return (1.f - abs(v.yx)) * (v.xy >= 0.f ? 1.f : -1.f);
}

/**********************/
// Signed encodings
// Converts a normalized direction to the octahedral map (non-equal area, signed)
// n - normalized direction
// Returns a signed position in octahedral map [-1, 1] for each component
float2 ndirToOctSigned(float3 n)
{
    // Project the sphere onto the octahedron (|x|+|y|+|z| = 1) and then onto the xy-plane
    float2 p = n.xy * (1.f / (abs(n.x) + abs(n.y) + abs(n.z)));
    return (n.z < 0.f) ? octWrap(p) : p;
}

// Converts a point in the octahedral map to a normalized direction (non-equal area, signed)
// p - signed position in octahedral map [-1, 1] for each component 
// Returns normalized direction
float3 octToNdirSigned(float2 p)
{
    // https://twitter.com/Stubbesaurus/status/937994790553227264
    float3 n = float3(p.x, p.y, 1.0 - abs(p.x) - abs(p.y));
    float t = max(0, -n.z);
    n.xy += n.xy >= 0.0 ? -t : t;
    return normalize(n);
}

/**********************/
// Unorm 32 bit encodings
// Converts a normalized direction to the octahedral map (non-equal area, unsigned normalized)
// n - normalized direction
// Returns a packed 32 bit unsigned normalized position in octahedral map
// The two components of the result are stored in UNORM16 format, [0..1]
uint ndirToOctUnorm32(float3 n)
{
    float2 p = ndirToOctSigned(n);
    p = saturate(p.xy * 0.5 + 0.5);
    return uint(p.x * 0xfffe) | (uint(p.y * 0xfffe) << 16);
}

// Converts a point in the octahedral map (non-equal area, unsigned normalized) to normalized direction
// pNorm - a packed 32 bit unsigned normalized position in octahedral map
// Returns normalized direction
float3 octToNdirUnorm32(uint pUnorm)
{
    float2 p;
    p.x = saturate(float(pUnorm & 0xffff) / 0xfffe);
    p.y = saturate(float(pUnorm >> 16) / 0xfffe);
    p = p * 2.0 - 1.0;
    return octToNdirSigned(p);
}

// Pack [0.0, 1.0] float to a uint of a given bit depth
#define PACK_UFLOAT_TEMPLATE(size)                      \
uint Pack_R ## size ## _UFLOAT(float r, float d = 0.5f) \
{                                                       \
    const uint mask = (1U << size) - 1U;                \
                                                        \
    return (uint)floor(r * mask + d) & mask;            \
}                                                       \
                                                        \
float Unpack_R ## size ## _UFLOAT(uint r)               \
{                                                       \
    const uint mask = (1U << size) - 1U;                \
                                                        \
    return (float)(r & mask) / (float)mask;             \
}

PACK_UFLOAT_TEMPLATE(8)
PACK_UFLOAT_TEMPLATE(10)
PACK_UFLOAT_TEMPLATE(11)
PACK_UFLOAT_TEMPLATE(16)

uint Pack_R11G11B10_UFLOAT(float3 rgb, float3 d = float3(0.5f, 0.5f, 0.5f))
{
    uint r = Pack_R11_UFLOAT(rgb.r, d.r);
    uint g = Pack_R11_UFLOAT(rgb.g, d.g) << 11;
    uint b = Pack_R10_UFLOAT(rgb.b, d.b) << 22;
    return r | g | b;
}

float3 Unpack_R11G11B10_UFLOAT(uint rgb)
{
    float r = Unpack_R11_UFLOAT(rgb);
    float g = Unpack_R11_UFLOAT(rgb >> 11);
    float b = Unpack_R10_UFLOAT(rgb >> 22);
    return float3(r, g, b);
}

uint Pack_R8G8B8A8_Gamma_UFLOAT(float4 rgba, float gamma = 2.2, float4 d = float4(0.5f, 0.5f, 0.5f, 0.5f))
{
    rgba = pow(saturate(rgba), 1.0 / gamma);
    uint r = Pack_R8_UFLOAT(rgba.r, d.r);
    uint g = Pack_R8_UFLOAT(rgba.g, d.g) << 8;
    uint b = Pack_R8_UFLOAT(rgba.b, d.b) << 16;
    uint a = Pack_R8_UFLOAT(rgba.a, d.a) << 24;
    return r | g | b | a;
}

float4 Unpack_R8G8B8A8_Gamma_UFLOAT(uint rgba, float gamma = 2.2)
{
    float r = Unpack_R8_UFLOAT(rgba);
    float g = Unpack_R8_UFLOAT(rgba >> 8);
    float b = Unpack_R8_UFLOAT(rgba >> 16);
    float a = Unpack_R8_UFLOAT(rgba >> 24);
    float4 v = float4(r, g, b, a);
    v = pow(saturate(v), gamma);
    return v;
}

/** Encode a normal packed as 2x 8-bit snorms in the octahedral mapping. The high 16 bits are unused.
*/
uint encodeNormal2x8(float3 normal)
{
    float2 octNormal = ndir_to_oct_snorm(normal);
    return packSnorm2x8(octNormal);
}

/** Decode a normal packed as 2x 8-bit snorms in the octahedral mapping.
*/
float3 decodeNormal2x8(uint packedNormal)
{
    float2 octNormal = unpackSnorm2x8(packedNormal);
    return oct_to_ndir_snorm(octNormal);
}

/** Encode a normal packed as 2x 16-bit snorms in the octahedral mapping.
*/
uint encodeNormal2x16(float3 normal)
{
    float2 octNormal = ndir_to_oct_snorm(normal);
    return packSnorm2x16(octNormal);
}

/** Decode a normal packed as 2x 16-bit snorms in the octahedral mapping.
*/
float3 decodeNormal2x16(uint packedNormal)
{
    float2 octNormal = unpackSnorm2x16(packedNormal);
    return oct_to_ndir_snorm(octNormal);
}

/** Encode a normal packed as 3x 16-bit snorms. Note: The high 16 bits of the second dword are unused.
*/
uint2 encodeNormal3x16(float3 normal)
{
    uint2 packedNormal;
    packedNormal.x = packSnorm2x16(normal.xy);
    packedNormal.y = packSnorm16(normal.z);
    return packedNormal;
}

/** Decode a normal packed as 3x 16-bit snorms. Note: The high 16 bits of the second dword are unused.
*/
float3 decodeNormal3x16(uint2 packedNormal)
{
    float3 normal;
    normal.xy = unpackSnorm2x16(packedNormal.x);
    normal.z = unpackSnorm16(packedNormal.y);
    return normalize(normal);
}

/** Encode an RGB color into a 32-bit LogLuv HDR format.
    The supported luminance range is roughly 10^-6..10^6 in 0.17% steps.

    The log-luminance is encoded with 14 bits and chroma with 9 bits each.
    This was empirically more accurate than using 8 bit chroma.
    Black (all zeros) is handled exactly.
*/
uint encodeLogLuvHDR(float3 color)
{
    // Convert RGB to XYZ.
    float3 XYZ = RGBtoXYZ_Rec709(color);

    // Encode log2(Y) over the range [-20,20) in 14 bits (no sign bit).
    // TODO: Fast path that uses the bits from the fp32 representation directly.
    float logY = 409.6f * (log2(XYZ.y) + 20.f); // -inf if Y==0
    uint Le = (uint)clamp(logY, 0.f, 16383.f);

    // Early out if zero luminance to avoid NaN in chroma computation.
    // Note Le==0 if Y < 9.55e-7. We'll decode that as exactly zero.
    if (Le == 0) return 0;

    // Compute chroma (u,v) values by:
    //  x = X / (X + Y + Z)
    //  y = Y / (X + Y + Z)
    //  u = 4x / (-2x + 12y + 3)
    //  v = 9y / (-2x + 12y + 3)
    //
    // These expressions can be refactored to avoid a division by:
    //  u = 4X / (-2X + 12Y + 3(X + Y + Z))
    //  v = 9Y / (-2X + 12Y + 3(X + Y + Z))
    //
    float invDenom = 1.f / (-2.f * XYZ.x + 12.f * XYZ.y + 3.f * (XYZ.x + XYZ.y + XYZ.z));
    float2 uv = float2(4.f, 9.f) * XYZ.xy * invDenom;

    // Encode chroma (u,v) in 9 bits each.
    // The gamut of perceivable uv values is roughly [0,0.62], so scale by 820 to get 9-bit values.
    uint2 uve = (uint2)clamp(820.f * uv, 0.f, 511.f);

    return (Le << 18) | (uve.x << 9) | uve.y;
}

/** Decode an RGB color stored in a 32-bit LogLuv HDR format.
    See encodeLogLuvHDR() for details.
*/
float3 decodeLogLuvHDR(uint packedColor)
{
    // Decode luminance Y from encoded log-luminance.
    uint Le = packedColor >> 18;
    if (Le == 0) return float3(0.f);

    float logY = (float(Le) + 0.5f) / 409.6f - 20.f;
    float Y = pow(2.f, logY);

    // Decode normalized chromaticity xy from encoded chroma (u,v).
    //
    //  x = 9u / (6u - 16v + 12)
    //  y = 4v / (6u - 16v + 12)
    //
    uint2 uve = uint2(packedColor >> 9, packedColor) & 0x1ff;
    float2 uv = (float2(uve) + 0.5f) / 820.f;

    float invDenom = 1.f / (6.f * uv.x - 16.f * uv.y + 12.f);
    float2 xy = float2(9.f, 4.f) * uv * invDenom;

    // Convert chromaticity to XYZ and back to RGB.
    //  X = Y / y * x
    //  Z = Y / y * (1 - x - y)
    //
    float s = Y / xy.y;
    float3 XYZ = { s * xy.x, Y, s * (1.f - xy.x - xy.y) };

    // Convert back to RGB and clamp to avoid out-of-gamut colors.
    return max(XYZtoRGB_Rec709(XYZ), 0.f);
}
