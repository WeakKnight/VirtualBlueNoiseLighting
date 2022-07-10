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
#include "Utils/Math/MathConstants.slangh"
#include "Scene/Material/MaterialDefines.slangh"
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
import Utils.Sampling.SampleGenerator;
import Utils.Helpers;
import Experimental.Scene.Material.Fresnel;
import Experimental.Scene.Material.Microfacet;
import Scene.Shading;
import Scene.Scene;
import Scene.TextureSampler;

#if USE_SECOND_DIFFUSE_BRDF == 1
#define USE_SECOND_BRDF 1
#else
#define USE_SECOND_BRDF 0
#endif
/// 0 Ash, 1 Frosbite
#if IS_SPEC_GLOSS_WORKFLOW == 1
#define SECOND_DIFFUSE_BRDF 0
#else
#define SECOND_DIFFUSE_BRDF 1
#endif

#define EnableDeltaBSDF 1

enum class LOBE_TYPE
{
    DIFFUSE = 1,
    SPECULAR = 2,
    DELTA = 3,
};

// Enable GGX sampling using the distribution of visible normals (VNDF) instead of classic NDF sampling.
// This should be the default as it has lower variance, disable for testing only.
// TODO: Make default when transmission with VNDF sampling is properly validated

// We clamp the GGX width parameter to avoid numerical instability.
// In some computations, we can avoid clamps etc. if 1.0 - alpha^2 != 1.0, so the epsilon should be 1.72666361e-4 or larger in fp32.
// The the value below is sufficient to avoid visible artifacts.
// Falcor used to clamp roughness to 0.08 before the clamp was removed for allowing delta events. We continue to use the same threshold.
static const float kMinGGXAlpha = 0.0064f;

// Minimum cos(theta) for the view and light vectors.
// A few functions are not robust for cos(theta) == 0.0.
// TODO: Derive appropriate bounds
static const float kMinCosTheta = 1e-6f;

/** Interface for BxDFs.
    Conventions:
    - wo is the outgoing or scattering direction and points away from the shading location.
    - wi is the incident or light direction and points away from the shading location.
    - the local shading frame has normal N=(0,0,1), tangent T=(1,0,0) and bitangent B=(0,1,0).
    - the outgoing direction is always in the positive hemisphere.
    - evaluating the BxDF always includes the foreshortening term (dot(wi, n) = wi.z).
*/

// Workaround for isnan() not working in slang.
bool isNaN(float f)
{
    uint u = asuint(f) & ~0x80000000u; // clear out the sign bit
    return (u > 0x7F800000);           // greater than Inf is NaN
}

float3 SampleNdfGGXWalter(float3 N, float3 V, const float2 u, const float alpha, out float pdf, out float VdotH, out float NdotH)
{
    // Draw sample from D(H) * NdotH.
    float a2 = alpha * alpha;
    float cosThetaHSqr = min((1 - u.x) / ((a2 - 1) * u.x + 1), 1.0f); // Clamp to avoid 1.0+epsilon causing NaNs below.
    float cosThetaH = sqrt(cosThetaHSqr);
    float sinThetaH = sqrt(1 - cosThetaHSqr);
    float phiH = u.y * M_2PI;

    // Convert half vector to world space.
    float3 H = float3(sinThetaH * cos(phiH), sinThetaH * sin(phiH), cosThetaH);
    H = fromLocal(H, N);

    NdotH = cosThetaH;
    VdotH = dot(V, H);

    // Compute incident direction L by reflecting V about H.
    float3 L = normalize(2.f * VdotH * H - V);   // Note: L is already of unit length, but normalize to reduce the risk of round-off errors.

    // Evaluate the pdf.
    // The pdf in half vector space is pdf = D(H) * NdotH, which we multiply by the Jacobian of the half-vector transform.
    float d = (a2 - 1) * cosThetaHSqr + 1;
    pdf = (a2 * NdotH) / (d * d * VdotH * M_4PI);
    //pdf = evalNdfGGX(alpha, NdotH) * NdotH / (4.f * VdotH);   // For reference, identical to the line above

    // Reject sample if L is in the lower hemisphere. Note: We should check dot(N,V) elsewhere.
    float NdotL = dot(N, L);
    if (NdotL < kMinCosTheta)
    {
        pdf = 0.f;
    }

    return L;
}

interface IBRDF
{
    /** Evaluates the BxDF.
        \param[in] wo Outgoing direction.
        \param[in] wi Incident direction.
        \return Returns f(wo, wi) * dot(wi, n).
    */
    float3 eval(float3 wo, float3 wi);

    float3 evalFr(float3 wo, float3 wi);

    /** Samples the BxDF.
        \param[in] wo Outgoing direction.
        \param[out] wi Incident direction.
        \param[out] pdf pdf with respect to solid angle for sampling incident direction wi (0 if a delta event is sampled).
        \param[out] weight Sample weight f(wo, wi) * dot(wi, n) / pdf(wi).
        \param[out] lobe Sampled lobe.
        \param[inout] sg Sample generator.
        \return Returns true if successful.
    */
    bool sample(float3 wo, out float3 wi, out float pdf, out float3 weight, out uint lobe, inout SampleGenerator sg);

    /** Evaluates the BxDF directional pdf for sampling incident direction wi.
        \param[in] wo Outgoing direction.
        \param[in] wi Incident direction.
        \return Returns the pdf with respect to solid angle for sampling incident direction wi (0 for delta events).
    */
    float evalPdf(float3 wo, float3 wi);
}

struct DiffuseBRDF : IBRDF
{
    float3 albedo;  ///< Diffuse albedo.

    float3 evalFr(float3 wo, float3 wi)
    {
        return M_1_PI * albedo;
    }

    float3 eval(float3 wo, float3 wi)
    {
        if (min(wo.z, wi.z) < kMinCosTheta) return float3(0);

        return M_1_PI * albedo * wi.z;
    }

    bool sample(float3 wo, out float3 wi, out float pdf, out float3 weight, out uint lobe, inout SampleGenerator sg)
    {
        wi = sample_cosine_hemisphere_concentric(sampleNext2D(sg), pdf);

        if (min(wo.z, wi.z) < kMinCosTheta) return false;

        weight = albedo;
        lobe = (uint)LOBE_TYPE::DIFFUSE;
        return true;
    }

    float evalPdf(float3 wo, float3 wi)
    {
        if (min(wo.z, wi.z) < kMinCosTheta) return 0;

        return M_1_PI * wi.z;
    }
};

// Ashikhmin-Shirley Model (conserves energy, requires Fresnel term using Schlick approximation)
struct DiffuseAshBRDF : IBRDF
{
    float3 albedo;
    float3 specAlbedo;

    float3 evalFr(float3 wo, float3 wi)
    {
        return evalWeight(wo, wi) * M_1_PI;
    }

    float3 eval(const float3 wo, const float3 wi)
    {
        if (min(wo.z, wi.z) < kMinCosTheta) return float3(0.f);

        return evalWeight(wo, wi) * M_1_PI * wi.z;
    }

    bool sample(const float3 wo, out float3 wi, out float pdf, out float3 weight, out uint lobe, inout SampleGenerator sg)
    {
        wi = sample_cosine_hemisphere_concentric(sampleNext2D(sg), pdf);

        if (min(wo.z, wi.z) < kMinCosTheta) return false;

        weight = evalWeight(wo, wi);
        lobe = (uint)LOBE_TYPE::DIFFUSE;
        return true;
    }

    float evalPdf(const float3 wo, const float3 wi)
    {
        if (min(wo.z, wi.z) < kMinCosTheta) return 0.f;

        return M_1_PI * wi.z;
    }

    // Returns f(wi, wo) * pi.
    float3 evalWeight(float3 wo, float3 wi)
    {
        float3 h = normalize(wo + wi);
        float3 factor = (1 - specAlbedo) * 28.f / 23.f * (1.f - pow(1.f - wo.z / 2, 5.f)) * (1.f - pow(1.f - wi.z / 2, 5.f));
        return albedo * factor;
    }
};

/** Frostbites's diffuse reflection.
    This is Disney's diffuse BRDF with an ad-hoc normalization factor to ensure energy conservation.
    Based on https://seblagarde.files.wordpress.com/2015/07/course_notes_moving_frostbite_to_pbr_v32.pdf
*/
struct DiffuseReflectionFrostbite : IBRDF
{
    float3 albedo;          ///< Diffuse albedo.
    float linearRoughness;  ///< Roughness before remapping.

    float3 evalFr(float3 wo, float3 wi)
    {
        return eval(wo, wi) / wi.z;  
    }

    float3 eval(float3 wo, float3 wi)
    {
        if (min(wo.z, wi.z) < kMinCosTheta) return float3(0);

        return evalWeight(wo, wi) * M_1_PI * wi.z;
    }

    bool sample<S : ISampleGenerator>(float3 wo, out float3 wi, out float pdf, out float3 weight, out uint lobe, inout S sg)
    {
        wi = sample_cosine_hemisphere_concentric(sampleNext2D(sg), pdf);
        lobe = (uint)LOBE_TYPE::DIFFUSE;

        if (min(wo.z, wi.z) < kMinCosTheta)
        {
            weight = {};
            return false;
        }

        weight = evalWeight(wo, wi);
        return true;
    }

    float evalPdf(float3 wo, float3 wi)
    {
        if (min(wo.z, wi.z) < kMinCosTheta) return 0;

        return M_1_PI * wi.z;
    }

    // private

    // Returns f(wo, wi) * pi.
    float3 evalWeight(float3 wo, float3 wi)
    {
        float3 h = normalize(wo + wi);
        float wiDotH = dot(wi, h);
        float energyBias = lerp(0, 0.5, linearRoughness);
        float energyFactor = lerp(1, 1.0 / 1.51, linearRoughness);
        float fd90 = energyBias + 2 * wiDotH * wiDotH * linearRoughness;
        float fd0 = 1;
        float wiScatter = evalFresnelSchlick(fd0, fd90, wi.z).r;
        float woScatter = evalFresnelSchlick(fd0, fd90, wo.z).r;
        return albedo * wiScatter * woScatter * energyFactor;
    }
};

/** Specular reflection using microfacets.
*/
struct SpecularBRDF : IBRDF
{
    float3 albedo;  ///< Specular albedo.
    float alpha;    ///< GGX width parameter.

    float3 evalFr(float3 wo, float3 wi)
    {
        return eval(wo, wi) / wi.z;  
    }

    float3 eval(float3 wo, float3 wi)
    {
        if (min(wo.z, wi.z) < kMinCosTheta) return float3(0);

#if EnableDeltaBSDF
        // Handle delta reflection.
        if (alpha == 0) return float3(0);
#endif

        float3 h = normalize(wo + wi);
        float woDotH = dot(wo, h);

        float D = evalNdfGGX(alpha, h.z);

        float G = evalMaskingSmithGGXSeparable(alpha, wo.z, wi.z);

        float3 F = evalFresnelSchlick(albedo, 1, woDotH);
        return F * D * G * 0.25 / wo.z;
    }

    bool sample(float3 wo, out float3 wi, out float pdf, out float3 weight, out uint lobe, inout SampleGenerator sg)
    {
        if (wo.z < kMinCosTheta) return false;

        // Sample the GGX distribution to find a microfacet normal (half vector).
        float3 h = sampleGGX_VNDF(alpha, wo, sampleNext2D(sg), pdf);    // pdf = G1(wo) * D(h) * max(0,dot(wo,h)) / wo.z

        // Reflect the outgoing direction to find the incident direction.
        float woDotH = dot(wo, h);
        wi = 2 * woDotH * h - wo;
        if (wi.z < kMinCosTheta) return false;

#if EnableDeltaBSDF
        // Handle delta reflection.
        if (alpha == 0)
        {
            wi = float3(-wo.x, -wo.y, wo.z);
            pdf = 0;
            weight = evalFresnelSchlick(albedo, 1, wo.z);
            lobe = (uint)LOBE_TYPE::SPECULAR;
            return true;
        }
#endif

        float G = evalMaskingSmithGGXSeparable(alpha, wo.z, wi.z);
        float GOverG1wo = evalG1GGX(alpha * alpha, wi.z);

        float3 F = evalFresnelSchlick(albedo, 1, woDotH);

        pdf /= (4 * woDotH); // Jacobian of the reflection operator.

        weight = F * GOverG1wo;

        lobe = (uint)LOBE_TYPE::SPECULAR;
        return true;
    }

    float evalPdf(float3 wo, float3 wi)
    {
        if (min(wo.z, wi.z) < kMinCosTheta) return 0;

#if EnableDeltaBSDF
        // Handle delta reflection.
        if (alpha == 0) return 0;
#endif

        float3 h = normalize(wo + wi);
        float woDotH = dot(wo, h);

        float pdf = evalPdfGGX_VNDF(alpha, wo, h);

        return pdf / (4 * woDotH);
    }
};

/** Layered BSDF used as primary material in Falcor.

    This consists of a diffuse and specular BRDF.
*/
struct SimpleBRDF : IBRDF
{
    DiffuseBRDF diffuseReflection;
#if USE_SECOND_BRDF == 1
#if SECOND_DIFFUSE_BRDF == 0
    DiffuseAshBRDF diffuseAshReflection;
#else
    DiffuseReflectionFrostbite diffuseFrosbite;
#endif
    bool treatAsPureDiffuse;
#endif
    SpecularBRDF specularReflection;

    float pDiffuseReflection;               ///< Probability for sampling the diffuse BRDF.
    float pSpecularReflection;              ///< Probability for sampling the specular BRDF.

    /** Setup the BSDF for sampling and evaluation.
        TODO: Currently specular reflection and transmission lobes are not properly separated.
        This leads to incorrect behaviour if only the specular reflection or transmission lobe is selected.
        Things work fine as long as both or none are selected.
        \param[in] sd Shading data.
    */
    [mutating] void setup(const ShadingData sd)
    {
        // Setup lobes.
        diffuseReflection.albedo = sd.diffuse;

        // Compute GGX alpha.
        float alpha = sd.linearRoughness * sd.linearRoughness;

#if EnableDeltaBSDF
        // Alpha below min alpha value means using delta reflection/transmission.
        if (alpha < kMinGGXAlpha) alpha = 0;
#else
        alpha = max(alpha, kMinGGXAlpha);
#endif

        specularReflection.albedo = sd.specular;
        specularReflection.alpha = alpha;

#if USE_SECOND_BRDF == 1
#if SECOND_DIFFUSE_BRDF == 0
        diffuseAshReflection.albedo = sd.diffuse;
        diffuseAshReflection.specAlbedo = sd.specular;
#else
        diffuseFrosbite.albedo = sd.diffuse;
        diffuseFrosbite.linearRoughness = sd.linearRoughness;
#endif

        treatAsPureDiffuse = sd.specular.x + sd.specular.y + sd.specular.z < 0.0001f;
#endif

        // Compute sampling weights.
        float metallicBRDF = sd.metallic;
        float dielectricBRDF = (1 - sd.metallic);

        float diffuseWeight = luminance(sd.diffuse);
        float specularWeight = luminance(evalFresnelSchlick(sd.specular, 1.f, dot(sd.V, sd.N)));

        pDiffuseReflection =  diffuseWeight * dielectricBRDF;
        pSpecularReflection = specularWeight * (metallicBRDF + dielectricBRDF);

        float normFactor = pDiffuseReflection + pSpecularReflection;
        if (normFactor > 0)
        {
            normFactor = 1 / normFactor;
            pDiffuseReflection *= normFactor;
            pSpecularReflection *= normFactor;
        }
    }

    float3 eval(float3 wo, float3 wi)
    {
        float3 result = 0;

        if (pDiffuseReflection > 0)
        {
#if USE_SECOND_BRDF == 1
            if (treatAsPureDiffuse) 
            {
                result += diffuseReflection.eval(wo, wi);
            }
            else 
            {
#if SECOND_DIFFUSE_BRDF == 0
                result += diffuseAshReflection.eval(wo, wi);
#else
                result += diffuseFrosbite.eval(wo, wi);
#endif
            }
#else
            result += diffuseReflection.eval(wo, wi);
#endif
        }
        if (pSpecularReflection > 0) result += specularReflection.eval(wo, wi);

        if (isNaN(result.x) || isNaN(result.y) || isNaN(result.z))
        {
            result = 0;
        }

        return result;
    }

    float3 evalFr(float3 wo, float3 wi)
    {
        float3 result = 0;
        if (pDiffuseReflection > 0)
        {
#if USE_SECOND_BRDF == 1
            if (treatAsPureDiffuse) 
            {
                result += diffuseReflection.evalFr(wo, wi);
            }
            else 
            {
#if SECOND_DIFFUSE_BRDF == 0
                result += diffuseAshReflection.evalFr(wo, wi);
#else
                result += diffuseFrosbite.evalFr(wo, wi);
#endif
            }
#else
            result += diffuseReflection.evalFr(wo, wi);
#endif
        }
        if (pSpecularReflection > 0) result += specularReflection.evalFr(wo, wi);

        if (isNaN(result.x) || isNaN(result.y) || isNaN(result.z))
        {
            result = 0;
        }

        return result;
    }

    float3 evalDiffuse(float3 wo, float3 wi)
    {
        float3 brdfVal;
#if USE_SECOND_BRDF == 1
        if (treatAsPureDiffuse)
        {
            brdfVal = diffuseReflection.eval(wo, wi);
        }
        else
        {
#if SECOND_DIFFUSE_BRDF == 0
            brdfVal = diffuseAshReflection.eval(wo, wi);
#else
            brdfVal = diffuseFrosbite.eval(wo, wi);
#endif
        }
#else
        brdfVal = diffuseReflection.eval(wo, wi);
#endif
        return brdfVal;
    }

    float3 evalDiffuseFr(float3 wo, float3 wi)
    {
        float3 brdfVal;
#if USE_SECOND_BRDF == 1
        if (treatAsPureDiffuse)
        {
            brdfVal = diffuseReflection.evalFr(wo, wi);
        }
        else
        {
#if SECOND_DIFFUSE_BRDF == 0
            brdfVal = diffuseAshReflection.evalFr(wo, wi);
#else
            brdfVal = diffuseFrosbite.evalFr(wo, wi);
#endif
        }
#else
        brdfVal = diffuseReflection.evalFr(wo, wi);
#endif
        return brdfVal;
    }

    float3 evalSpecular(float3 wo, float3 wi)
    {
        return specularReflection.eval(wo, wi);
    }

    float3 evalFrWIthoutCheck(float3 wo, float3 wi)
    {
        float3 result = 0;
#if USE_SECOND_BRDF == 1
        if (treatAsPureDiffuse) 
        {
            result += diffuseReflection.evalFr(wo, wi);
        }
        else 
        {
#if SECOND_DIFFUSE_BRDF == 0
            result += diffuseAshReflection.evalFr(wo, wi);
#else
            result += diffuseFrosbite.evalFr(wo, wi);
#endif
        }
#else
        result += diffuseReflection.evalFr(wo, wi);
#endif
        result += specularReflection.evalFr(wo, wi);

        if (isNaN(result.x) || isNaN(result.y) || isNaN(result.z))
        {
            result = 0;
        }

        return result;
    }

    bool sample(const ShadingData sd, out float3 L, out float pdf, out uint lobeType, inout SampleGenerator sg)
    {
        float pdfSubBRDF;
        return sample(sd.N, sd.V, L, pdf, pdfSubBRDF, lobeType, sg);
    }

    bool sample(const ShadingData sd, out float3 L, out float pdf, out float pdfSubBRDF, out uint lobeType, inout SampleGenerator sg)
    {
        return sample(sd.N, sd.V, L, pdf, pdfSubBRDF, lobeType, sg);
    }

    bool sample(float3 N, float3 V, out float3 L, out float pdf, inout SampleGenerator sg)
    {
        uint lobeType;
        return sample(N, V, L, pdf, lobeType, sg);
    }

    bool sample(float3 N, float3 V, out float3 L, out float pdf, out uint lobeType, inout SampleGenerator sg)
    {
        float pdfSubBRDF;
        return sample(N, V, L, pdf, pdfSubBRDF, lobeType, sg);
    }

    bool sample(float3 N, float3 V, out float3 L, out float pdf, out float pdfSubBRDF, out uint lobeType, inout SampleGenerator sg)
    {
        // Draw uniform random numbers for lobe selection (1D) and sampling (2D).
        const float2 u = sampleNext2D(sg);
        const float uSelect = sampleNext1D(sg);

        float pmfSelectedLobe;
        float pdfOther;

        // Randomly select which lobe to sample.
        if (uSelect < pDiffuseReflection)
        {
            // Sample diffuse lobe.
            L = cosineWeightedSampling(u, N, pdf);
            // Check that L and V are in the positive hemisphere.
            float NdotL = dot(N, L);
            if (min(dot(N, V), NdotL) < kMinCosTheta)
            {
                pdf = 0.f;
                return false;
            }
            pdfSubBRDF = pdf;

            pmfSelectedLobe = pDiffuseReflection;

            // Evaluate the pdf of the other sampling strategy.

            float3 H = normalize(V + L);
            float NdotH = saturate(dot(N, H));
            float VdotH = saturate(dot(V, H));

            // We're sampling the GGX distribution with pdf = D(H) * NdotH / (4.f * VdotH).
            float alpha = specularReflection.alpha;
            pdfOther = evalNdfGGX(alpha, NdotH) * NdotH / (4.f * VdotH);
            if(alpha == 0)
            {
                pdfOther = 0.f;
            }

            lobeType = (uint)LOBE_TYPE::DIFFUSE;
        }
        else
        {
            // Sample the GGX distribution with pdf = D(H) * NdotH / (4.f * VdotH).
            float alpha = specularReflection.alpha;
            if(alpha == 0)
            {
                pdf = 1.f;
                L = reflect(-V, N);
                lobeType = (uint)LOBE_TYPE::DELTA;
            }
            else
            {
                float VdotH, NdotH;
                L = SampleNdfGGXWalter(N, V, u, alpha, pdf, VdotH, NdotH);
                lobeType = (uint)LOBE_TYPE::SPECULAR;
            }

            // Check that L and V are in the positive hemisphere.
            float NdotL = dot(N, L);
            if (min(dot(N, V), NdotL) < kMinCosTheta)
            {
                pdf = 0.f;
                return false;
            }
            pdfSubBRDF = pdf;

            pmfSelectedLobe = 1.f - pDiffuseReflection;

            // Evaluate the pdf of the other sampling strategy.
            pdfOther = NdotL < kMinCosTheta ? 0.f : NdotL * M_1_PI;

            if (lobeType == (uint)LOBE_TYPE::DELTA) pdfOther = 0.f;
        }

        // Evaluate the pdf for the sample as a linear combination of the two sampling strategies' pdfs.
        pdf = pmfSelectedLobe * pdf + (1.f - pmfSelectedLobe) * pdfOther;

        if(isNaN(pdf))
        {
            return false;
        }

        return specularReflection.alpha == 0 || pdf > 0.f;
    }

    float evalPdf(float3 wo, float3 wi)
    {
        float pdf = 0;
        if (pDiffuseReflection > 0) pdf += pDiffuseReflection * diffuseReflection.evalPdf(wo, wi);
        if (pSpecularReflection > 0) 
        {
            float alpha = specularReflection.alpha;
            float3 H = normalize(wo + wi);
            float pdfOther = evalNdfGGX(alpha, H.z) * H.z / (4.f * dot(wo,H));

            pdf += pSpecularReflection * pdfOther;
        }
        

        if (isNaN(pdf))
        {
            pdf = 0;
        }

        return pdf;
    }

    float evalDiffusePdf(float3 wo, float3 wi)
    {
        float pdf = 0;
        if (pDiffuseReflection > 0) pdf = diffuseReflection.evalPdf(wo, wi);
        if (isNaN(pdf)) pdf = 0;
        return pdf;
    }

    float evalSpecularPdf(float3 wo, float3 wi)
    {
        float pdf = 0;
        if (pSpecularReflection > 0) 
        {
            float alpha = specularReflection.alpha;
            float3 H = normalize(wo + wi);
            pdf = evalNdfGGX(alpha, H.z) * H.z / (4.f * dot(wo,H));
        }
        if (isNaN(pdf)) pdf = 0;
        return pdf;
    }

};
