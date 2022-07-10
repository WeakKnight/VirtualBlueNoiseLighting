/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// NRD v2.10

//=================================================================================================================================
// INPUT PARAMETERS
//=================================================================================================================================
/*
float3 radiance:
    - radiance != irradiance, it's pure energy coming from a particular direction
    - radiance should not include any BRDFs ( material information )
    - radiance can be premultiplied with "exposure"
    - radiance should not include PI for diffuse ( it will be canceled out later when the denoised output will be multiplied with albedo / PI )
    - for diffuse rays
        - use COS-distribution ( or custom importance sampling )
        - if radiance is the result of path tracing, pass normalized hit distance as the sum of 1-all hits (always ignore primary hit!)
    - for specular
        - use VNDF sampling ( or custom importance sampling )
        - if radiance is the result of path tracing, pass normalized hit distance as the sum of first 1-3 hits (always ignore primary hit!)

float linearRoughness:
    - linearRoughness = sqrt( roughness ), where "roughness" = "m" = "alpha" - specular or real roughness

float normal:
    - world space normal

float viewZ:
    - linear view space Z for primary rays ( linearized camera depth )

float distanceToOccluder:
    - distance to occluder, must follow the rules:
        - NoL <= 0         - 0 ( it's very important )
        - NoL > 0 ( hit )  - hit distance
        - NoL > 0 ( miss ) - NRD_FP16_MAX

float tanOfLightAngularRadius:
    - tan( lightAngularSize * 0.5 )
    - angular size is computed from the shadow receiving point
    - in other words, tanOfLightAngularRadius = distanceToLight / lightRadius

float normHitDist:
    - normalized hit distance, gotten by using "REBLUR_FrontEnd_GetNormHitDist"
    - REBLUR must be aware of the normalization function via "nrd::HitDistanceParameters"
    - by definition, normalized hit distance is AO ( ambient occlusion ) for diffuse and SO ( specular occlusion ) for specular
    - AO can be used to emulate 2nd+ diffuse bounces
    - SO can be used to adjust IBL lighting
    - ".w" channel of diffuse / specular output is AO / SO
    - if you don't know which normalization function to choose use default values of "nrd::HitDistanceParameters"

NOTE: if "roughness" is needed as an input parameter use is as "isDiffuse ? 1 : roughness"
*/

 // IMPORTANT: DO NOT MODIFY THIS FILE WITHOUT FULL RECOMPILATION OF NRD LIBRARY!

//=================================================================================================================================
// BINDINGS
//=================================================================================================================================

#if( defined COMPILER_FXC || defined COMPILER_DXC )

    #define NRD_CONSTANTS_START \
        cbuffer globalConstants : register( b0 ) {

    #define NRD_CONSTANT( constantType, constantName ) constantType constantName;

    #define NRD_CONSTANTS_END \
        };

    #define NRD_INPUT_TEXTURE( resourceType, resourceName, regName, bindingIndex ) \
        resourceType resourceName : register( regName ## bindingIndex );
    #define NRD_OUTPUT_TEXTURE( resourceType, resourceName, regName, bindingIndex ) \
        resourceType resourceName : register( regName ## bindingIndex );
    #define NRD_SAMPLER( resourceType, resourceName, regName, bindingIndex ) \
        resourceType resourceName : register( regName ## bindingIndex );

#elif( defined COMPILER_UNREAL_ENGINE )

    #define NRD_CONSTANTS_START
    #define NRD_CONSTANT( constantType, constantName ) constantType constantName;
    #define NRD_CONSTANTS_END

    #define NRD_INPUT_TEXTURE( resourceType, resourceName, regName, bindingIndex ) \
        resourceType resourceName;
    #define NRD_OUTPUT_TEXTURE( resourceType, resourceName, regName, bindingIndex ) \
        resourceType resourceName;
    #define NRD_SAMPLER( resourceType, resourceName, regName, bindingIndex ) \
        resourceType resourceName;

#elif( defined( NRD_INPUT_TEXTURE ) && defined( NRD_OUTPUT_TEXTURE ) && defined( NRD_CONSTANTS_START ) && defined( NRD_CONSTANT ) && defined( NRD_CONSTANTS_END ) )

    // Custom engine that has already defined all the macros

#else

    #error "Please, define one of COMPILER_FXC / COMPILER_DXC or COMPILER_UNREAL_ENGINE or add custom bindings"
    #error "Please, define the following HLSL keywords to match your platform"

    #define RWTexture2D
    #define numthreads
    #define SV_DispatchThreadId
    #define SV_GroupThreadId
    #define SV_GroupId
    #define cbuffer
    #define SV_GroupIndex
    #define GroupMemoryBarrierWithGroupSync
    #define groupshared
    #define GroupMemoryBarrier
    #define SampleLevel
    #define unorm
    #define InterlockedAdd
    #define InterlockedMax

#endif

#define NRD_RADIANCE_COMPRESSION_MODE_NONE                      0
#define NRD_RADIANCE_COMPRESSION_MODE_MODERATE                  1
#define NRD_RADIANCE_COMPRESSION_MODE_LESS_MID_ROUGHNESS        2
#define NRD_RADIANCE_COMPRESSION_MODE_BETTER_LOW_ROUGHNESS      3
#define NRD_RADIANCE_COMPRESSION_MODE_SIMPLER_LOW_ROUGHNESS     4

#define NRD_NORMAL_ENCODING_UNORM8                              0 // R8G8B8A8 - worst quality, best perf
#define NRD_NORMAL_ENCODING_OCT10                               1 // R10G10B10A2 - adds small overhead, good quality
#define NRD_NORMAL_ENCODING_UNORM16                             2 // R16G16B16A16 - best quality, worst perf

//=================================================================================================================================
// SETTINGS
//=================================================================================================================================

// Must match encoding used for IN_NORMAL_ROUGHNESS
#ifndef NRD_USE_SQRT_LINEAR_ROUGHNESS
    #define NRD_USE_SQRT_LINEAR_ROUGHNESS                       0
#endif

// Must match encoding used for IN_NORMAL_ROUGHNESS
#ifndef NRD_USE_MATERIAL_ID_AWARE_FILTERING
    #define NRD_USE_MATERIAL_ID_AWARE_FILTERING                 0
#endif

// Must match encoding used for IN_NORMAL_ROUGHNESS
#ifndef NRD_NORMAL_ENCODING
    #define NRD_NORMAL_ENCODING                                 NRD_NORMAL_ENCODING_UNORM8
#endif

// [Optional] Color compression for spatial passes (can be NRD_RADIANCE_COMPRESSION_MODE_NONE if the signal is relatively clean)
#ifndef NRD_RADIANCE_COMPRESSION_MODE
    #define NRD_RADIANCE_COMPRESSION_MODE                       NRD_RADIANCE_COMPRESSION_MODE_BETTER_LOW_ROUGHNESS // 0-4
#endif

// [Optional] Color compression for disocclusion fix (0.1 is enough to suppress undesired fireflies)
#ifndef NRD_MIP_COLOR_COMPRESSION_AMOUNT
    #define NRD_MIP_COLOR_COMPRESSION_AMOUNT                    0.1 //0-1
#endif

// Compatibility stubs (if NRD is integrated as a white box)
#ifndef NRD_CS_MAIN
    #define NRD_CS_MAIN                                         main
#endif

#define NRD_EXPORT                                              // can be non empty

//=================================================================================================================================
// PRIVATE
//=================================================================================================================================

#define NRD_MIN_PDF                                             0.01
#define NRD_FP16_VIEWZ_SCALE                                    0.0125
#define NRD_FP16_MAX                                            65504.0

float3 _NRD_DecodeUnitVector( float2 p, const bool bSigned = false, const bool bNormalize = true )
{
    p = bSigned ? p : ( p * 2.0 - 1.0 );

    // https://twitter.com/Stubbesaurus/status/937994790553227264
    float3 n = float3( p.xy, 1.0 - abs( p.x ) - abs( p.y ) );
    float t = saturate( -n.z );
    n.xy += n.xy >= 0.0 ? -t : t;

    return bNormalize ? normalize( n ) : n;
}

float _NRD_Luminance( float3 linearColor )
{
    return dot( linearColor, float3( 0.2990, 0.5870, 0.1140 ) );
}

float _NRD_GetColorCompressionExposureForSpatialPasses( float linearRoughness )
{
    // Prerequsites:
    // - to minimize biasing the results compression for high roughness should be avoided (diffuse signal compression can lead to darker image)
    // - the compression function must be monotonic for full roughness range

    // http://fooplot.com/#W3sidHlwZSI6MCwiZXEiOiIwLjUvKDErNTAqeCkiLCJjb2xvciI6IiNGNzBBMEEifSx7InR5cGUiOjAsImVxIjoiMC41KigxLXgpLygxKzYwKngpIiwiY29sb3IiOiIjMkJGRjAwIn0seyJ0eXBlIjowLCJlcSI6IjAuNSooMS14KS8oMSsxMDAwKngqeCkrKDEteF4wLjUpKjAuMDMiLCJjb2xvciI6IiMwMDU1RkYifSx7InR5cGUiOjAsImVxIjoiMC42KigxLXgqeCkvKDErNDAwKngqeCkiLCJjb2xvciI6IiMwMDAwMDAifSx7InR5cGUiOjEwMDAsIndpbmRvdyI6WyIwIiwiMSIsIjAiLCIxIl0sInNpemUiOlsyOTUwLDk1MF19XQ--

    // Moderate compression
    #if( NRD_RADIANCE_COMPRESSION_MODE == NRD_RADIANCE_COMPRESSION_MODE_MODERATE )
        return 0.5 / ( 1.0 + 50.0 * linearRoughness );
    // Less compression for mid-high roughness
    #elif( NRD_RADIANCE_COMPRESSION_MODE == NRD_RADIANCE_COMPRESSION_MODE_LESS_MID_ROUGHNESS )
        return 0.5 * ( 1.0 - linearRoughness ) / ( 1.0 + 60.0 * linearRoughness );
    // Close to the previous one, but offers more compression for low roughness
    #elif( NRD_RADIANCE_COMPRESSION_MODE == NRD_RADIANCE_COMPRESSION_MODE_BETTER_LOW_ROUGHNESS )
        return 0.5 * ( 1.0 - linearRoughness ) / ( 1.0 + 1000.0 * linearRoughness * linearRoughness ) + ( 1.0 - sqrt( saturate( linearRoughness ) ) ) * 0.03;
    // A modification of the preious one ( simpler )
    #elif( NRD_RADIANCE_COMPRESSION_MODE == NRD_RADIANCE_COMPRESSION_MODE_SIMPLER_LOW_ROUGHNESS )
        return 0.6 * ( 1.0 - linearRoughness * linearRoughness ) / ( 1.0 + 400.0 * linearRoughness * linearRoughness );
    // No compression
    #else
        return 0;
    #endif
}

// Hit distance normalization
float _REBLUR_GetHitDistanceNormalization( float viewZ, float4 hitDistParams, float meterToUnitsMultiplier, float linearRoughness = 1.0 )
{
    return ( hitDistParams.x * meterToUnitsMultiplier + abs( viewZ ) * hitDistParams.y ) * lerp( 1.0, hitDistParams.z, saturate( exp2( hitDistParams.w * linearRoughness * linearRoughness ) ) );
}

//=================================================================================================================================
// FRONT-END PACKING
//=================================================================================================================================

//========
// NRD
//========

// This function is used in all denoisers to decode normal, roughness and optional materialID
float4 NRD_FrontEnd_UnpackNormalAndRoughness( float4 p, out float materialID )
{
    float4 r;
    #if( NRD_NORMAL_ENCODING == NRD_NORMAL_ENCODING_OCT10 )
        const bool bSigned = false;
        const bool bNormalize = false;
        r.xyz = _NRD_DecodeUnitVector( p.xy, bSigned, bNormalize );
        r.w = p.z;
    #else
        r.xyz = p.xyz * 2.0 - 1.0;
        r.w = p.w;
    #endif

    // By default NRD offers only this decoding variant. It can be changed to match a specific encoding method
    #if( NRD_NORMAL_ENCODING == NRD_NORMAL_ENCODING_OCT10 && NRD_USE_MATERIAL_ID_AWARE_FILTERING == 1 )
        materialID = p.w;
    #else
        materialID = 0;
    #endif

    // Normalization is very important due to potential octahedron encoding and potential "best fit" usage for simple "N * 0.5 + 0.5" method
    r.xyz = normalize( r.xyz );

    #if( NRD_USE_SQRT_LINEAR_ROUGHNESS == 1 )
        r.w *= r.w;
    #endif

    return r;
}

// Helper functions to pack / unpack ray (or averaged) direction and PDF
float4 NRD_FrontEnd_PackDirectionAndPdf( float3 direction, float pdf )
{
    return float4( direction, pdf );
}

float4 NRD_FrontEnd_UnpackDirectionAndPdf( float4 directionAndPdf )
{
    directionAndPdf.w = max( directionAndPdf.w, NRD_MIN_PDF );

    return directionAndPdf;
}

//========
// REBLUR
//========

// This function returns AO / SO which REBLUR can decode back to "hit distance" internally
float REBLUR_FrontEnd_GetNormHitDist( float hitDist, float viewZ, float4 hitDistParams, float meterToUnitsMultiplier, float linearRoughness = 1.0 )
{
    float f = _REBLUR_GetHitDistanceNormalization( viewZ, hitDistParams, meterToUnitsMultiplier, linearRoughness );

    return saturate( hitDist / f );
}

float4 REBLUR_FrontEnd_PackRadianceAndHitDist( float3 radiance, float normHitDist, bool sanitize = true )
{
    if( sanitize )
    {
        radiance = any( isnan( radiance ) | isinf( radiance ) ) ? 0 : clamp( radiance, 0, NRD_FP16_MAX );
        normHitDist = ( isnan( normHitDist ) | isinf( normHitDist ) ) ? 0 : saturate( normHitDist );
    }

    return float4( radiance, normHitDist );
}

//========
// RELAX
//========

float4 RELAX_FrontEnd_PackRadianceAndHitDist( float3 radiance, float hitDist, bool sanitize = true )
{
    if( sanitize )
    {
        radiance = any( isnan( radiance ) | isinf( radiance ) ) ? 0 : clamp( radiance, 0, NRD_FP16_MAX );
        hitDist = ( isnan( hitDist ) | isinf( hitDist ) ) ? 0 : clamp( hitDist, 0, NRD_FP16_MAX );
    }

    return float4( radiance, hitDist );
}

//========
// SIGMA
//========

// Must be used to "clear" INF pixels
#define SIGMA_INF_SHADOW        float2( NRD_FP16_MAX, NRD_FP16_MAX )
#define SIGMA_MIN_DISTANCE      0.0001 // not 0, because it means "NoL < 0, stop processing"

// SIGMA ( single light )

float2 SIGMA_FrontEnd_PackShadow( float viewZ, float distanceToOccluder, float tanOfLightAngularRadius )
{
    float2 r;
    r.x = 0.0;
    r.y = clamp( viewZ * NRD_FP16_VIEWZ_SCALE, -NRD_FP16_MAX, NRD_FP16_MAX );

    [flatten]
    if( distanceToOccluder == NRD_FP16_MAX )
        r.x = NRD_FP16_MAX;
    else if( distanceToOccluder != 0.0 )
    {
        float distanceToOccluderProj = distanceToOccluder * tanOfLightAngularRadius;
        r.x = clamp( distanceToOccluderProj, SIGMA_MIN_DISTANCE, 32768.0 );
    }

    return r;
}

float2 SIGMA_FrontEnd_PackShadow( float viewZ, float distanceToOccluder, float tanOfLightAngularRadius, float3 translucency, out float4 shadowTranslucency )
{
    shadowTranslucency.x = float( distanceToOccluder == NRD_FP16_MAX );
    shadowTranslucency.yzw = saturate( translucency );

    return SIGMA_FrontEnd_PackShadow( viewZ, distanceToOccluder, tanOfLightAngularRadius );
}

// SIGMA ( multi light )

#define SIGMA_MULTILIGHT_DATATYPE float2x3

SIGMA_MULTILIGHT_DATATYPE SIGMA_FrontEnd_MultiLightStart()
{
    return ( SIGMA_MULTILIGHT_DATATYPE )0;
}

void SIGMA_FrontEnd_MultiLightUpdate( float3 L, float distanceToOccluder, float tanOfLightAngularRadius, float weight, inout SIGMA_MULTILIGHT_DATATYPE multiLightShadowData )
{
    float shadow = float( distanceToOccluder == NRD_FP16_MAX );
    float distanceToOccluderProj = SIGMA_FrontEnd_PackShadow( 0, distanceToOccluder, tanOfLightAngularRadius ).x;

    // Weighted sum for "pseudo" translucency
    multiLightShadowData[ 0 ] += L * shadow;

    // Weighted sum for distance to occluder (denoising will be driven by most important light)
    weight *= _NRD_Luminance( L );

    multiLightShadowData[ 1 ] += float3( distanceToOccluderProj * weight, weight, 0 );
}

float2 SIGMA_FrontEnd_MultiLightEnd( float viewZ, SIGMA_MULTILIGHT_DATATYPE multiLightShadowData, float3 Lsum, out float4 shadowTranslucency )
{
    shadowTranslucency.yzw = multiLightShadowData[ 0 ] / max( Lsum, 1e-6 );
    shadowTranslucency.x = _NRD_Luminance( shadowTranslucency.yzw );

    float2 r;
    r.x = multiLightShadowData[ 1 ].x / max( multiLightShadowData[ 1 ].y, 1e-6 );
    r.y = clamp( viewZ * NRD_FP16_VIEWZ_SCALE, -NRD_FP16_MAX, NRD_FP16_MAX );

    return r;
}

//=================================================================================================================================
// BACK-END UNPACKING
//=================================================================================================================================

//========
// REBLUR
//========

#define REBLUR_BackEnd_UnpackRadiance( color )  ( color ) // it's a stub for compatibility, currently unpacks internally

//========
// RELAX
//========

#define RELAX_BackEnd_UnpackRadiance( color )  ( color ) // it's a stub for compatibility, currently unpacks internally

//========
// SIGMA
//========

#define SIGMA_BackEnd_UnpackShadow( color )  ( color * color )

//=================================================================================================================================
// MISC
//=================================================================================================================================

// This can be a start, but works badly in many cases:
//      hitDist = hitDist1 + hitDist2 + ... ;
// Proper solution:
//      hitDist = NRD_GetCorrectedHitDist( hitDist1, 1, roughness0 );
//      hitDist += NRD_GetCorrectedHitDist( hitDist2, 2, roughness0, importance2 );
// where "importance" shows how much energy a new hit brings compared with the previous state (see NRD sample for more details)
// Notes:
//      0  - primary hit
//      1+ - bounces

float NRD_GetCorrectedHitDist( float hitDist, float bounceIndex, float roughness0 = 1.0, float importance = 1.0 )
{
    // TODO: currently works well only with sane distances, if a huge value is passed (like, 1e32) the effectivness of "fade" will be destroyed...
    // A "miss" must be last in the chain. How to better handle it?
    float fade = lerp( 1.0, bounceIndex, roughness0 );

    return hitDist * importance / ( fade * fade );
}

// We loose G-term if trimming is high, return it back in pre-integrated form
// A typical use case is:
/*
    float g = gIn_IntegratedBRDF.SampleLevel( gLinearSampler, float2( NoV, roughness ), 0.0 ).x; // pre-integrated G-term

    float trimmingFactor = NRD_GetTrimmingFactor( roughness, trimmingParams );
    F *= lerp( g, 1.0, trimmingFactor );
    Lsum += spec * F;
*/
float NRD_GetTrimmingFactor( float roughness, float3 trimmingParams )
{
    float trimmingFactor = trimmingParams.x * smoothstep( trimmingParams.y, trimmingParams.z, roughness );

    return trimmingFactor;
}

// Needs to be used to avoid summing up NAN/INF values in many rays per pixel scenarios
float NRD_GetSampleWeight( float3 radiance, bool sanitize = true )
{
    return ( any( isnan( radiance ) | isinf( radiance ) ) && sanitize ) ? 0.0 : 1.0;
}
