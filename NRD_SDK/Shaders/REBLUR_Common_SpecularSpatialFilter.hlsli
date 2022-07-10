/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#ifndef REBLUR_SPATIAL_MODE
    #error REBLUR_SPATIAL_MODE must be defined!
#endif

#if( REBLUR_SPATIAL_MODE == REBLUR_PRE_BLUR )
    #define POISSON_SAMPLE_NUM      REBLUR_PRE_BLUR_POISSON_SAMPLE_NUM
    #define POISSON_SAMPLES( i )    REBLUR_PRE_BLUR_POISSON_SAMPLES( i )
#else
    #define POISSON_SAMPLE_NUM      REBLUR_POISSON_SAMPLE_NUM
    #define POISSON_SAMPLES( i )    REBLUR_POISSON_SAMPLES( i )
#endif

{
    float4 center = spec;

    float exposure = _NRD_GetColorCompressionExposureForSpatialPasses( roughness );
    spec.xyz = STL::Color::Compress( spec.xyz, exposure );

    float radius = gSpecBlurRadius;
    #if( REBLUR_SPATIAL_MODE == REBLUR_PRE_BLUR )
        float2 specInternalData = REBLUR_PRE_BLUR_INTERNAL_DATA;
        radius *= REBLUR_PRE_BLUR_RADIUS_SCALE( roughness );
    #else
        float minAccumSpeed = GetMipLevel( roughness ) + 0.001;
        float boost = saturate( 1.0 - specInternalData.y / minAccumSpeed );
        radius *= ( 1.0 + 2.0 * boost ) / 3.0;
    #endif
    radius *= GetBlurRadiusScaleBasingOnTrimming( roughness, gSpecTrimmingParams.xyz );

    float radiusScale = 1.0;
    float radiusBias = 0.0; // see GetBlurRadius()
    float strictness = 1.0;

    #if( REBLUR_SPATIAL_MODE == REBLUR_POST_BLUR )
        radiusScale = REBLUR_POST_BLUR_RADIUS_SCALE;
        radiusBias = lerp( REBLUR_RADIUS_BIAS_CONFIDENCE_BASED_SCALE, 1.0, error.w ) * error.z * gSpecBlurRadiusScale + 0.00001;
        strictness = REBLUR_POST_BLUR_STRICTNESS;
    #elif( REBLUR_SPATIAL_MODE == REBLUR_BLUR )
        radiusBias = lerp( REBLUR_RADIUS_BIAS_CONFIDENCE_BASED_SCALE, 1.0, error.w ) * error.z * gSpecBlurRadiusScale + 0.00001;
        strictness = REBLUR_BLUR_NORMAL_WEIGHT_RELAXATION;
    #endif

    // Blur radius
    float hitDist = GetHitDist( center.w, viewZ, gSpecHitDistParams, roughness );
    float blurRadius = GetBlurRadius( radius, hitDist, viewZ, specInternalData.x, radiusBias, radiusScale, roughness );
    float worldBlurRadius = PixelRadiusToWorld( gUnproject, gIsOrtho, blurRadius, viewZ );

    // Denoising
    float anisoFade = lerp( curvature, 1.0, specInternalData.x );
    float3 Vv = GetViewVector( Xv, true );
    float2x3 TvBv = GetKernelBasis( Vv, Nv, worldBlurRadius, roughness, anisoFade );
    float2 geometryWeightParams = GetGeometryWeightParams( gPlaneDistSensitivity, gMeterToUnitsMultiplier, Xv, Nv, lerp( 1.0, REBLUR_PLANE_DIST_MIN_SENSITIVITY_SCALE, specInternalData.x ) );
    float normalWeightParams = GetNormalWeightParams( specInternalData.x, curvature, viewZ, roughness, gNormalWeightStrictness * strictness );
    float2 hitDistanceWeightParams = GetHitDistanceWeightParams( center.w, specInternalData.x, roughness );
    float2 roughnessWeightParams = GetRoughnessWeightParams( roughness );
    float2 sum = 1.0;

    #ifdef REBLUR_SPATIAL_REUSE
        blurRadius = REBLUR_PRE_BLUR_SPATIAL_REUSE_BASE_RADIUS_SCALE * gSpecBlurRadius * GetSpecMagicCurve( roughness ) * hitDist / ( hitDist + PixelRadiusToWorld( gUnproject, gIsOrtho, gRectSize.y, viewZ ) );

        float2 geometryWeightParamsReuse = GetGeometryWeightParams( gPlaneDistSensitivity, gMeterToUnitsMultiplier, Xv, Nv, REBLUR_PLANE_DIST_MIN_SENSITIVITY_SCALE );

        float angle = STL::ImportanceSampling::GetSpecularLobeHalfAngle( roughness );
        float normalWeightParamsReuse = rcp( max( angle, NRD_ENCODING_ERRORS.x ) );

        float3 V = STL::Geometry::RotateVectorInverse( gWorldToView, Vv );
        float NoV = abs( dot( Nv, Vv ) );
    #endif

    #if( REBLUR_SPATIAL_MODE == REBLUR_PRE_BLUR )
        float2 minwh = REBLUR_HIT_DIST_MIN_WEIGHT * 2.0;
    #elif( REBLUR_SPATIAL_MODE == REBLUR_BLUR )
        float2 minwh = float2( 0.0, REBLUR_HIT_DIST_MIN_WEIGHT );
    #else
        float2 minwh = float2( 0.0, REBLUR_HIT_DIST_MIN_WEIGHT * 0.5 );
    #endif
    minwh = lerp( 1.0, minwh, error.w );

    [unroll]
    for( uint i = 0; i < POISSON_SAMPLE_NUM; i++ )
    {
        float3 offset = POISSON_SAMPLES( i );

        // Sample coordinates
        #if( REBLUR_USE_SCREEN_SPACE_SAMPLING == 1 || defined( REBLUR_SPATIAL_REUSE ) )
            float2 uv = pixelUv + STL::Geometry::RotateVector( rotator, offset.xy ) * gInvScreenSize * blurRadius;
        #else
            float2 uv = GetKernelSampleCoordinates( gViewToClip, offset, Xv, TvBv[ 0 ], TvBv[ 1 ], rotator );
        #endif

        // Handle half res input in the checkerboard mode
        #if( REBLUR_SPATIAL_MODE == REBLUR_PRE_BLUR )
            float2 checkerboardUv = uv;
            if( gSpecCheckerboard != 2 )
                checkerboardUv = ApplyCheckerboard( uv, gSpecCheckerboard, i, gRectSize, gInvRectSize, gFrameIndex );
        #endif

        // Fetch data
        float2 uvScaled = uv * gResolutionScale;
        float4 Ns = gIn_Normal_Roughness.SampleLevel( gNearestMirror, uvScaled + gRectOffset, 0 );

        #if( REBLUR_SPATIAL_MODE == REBLUR_PRE_BLUR )
            float2 checkerboardUvScaled = checkerboardUv * gResolutionScale + gRectOffset;
            float4 s = gIn_Spec.SampleLevel( gNearestMirror, checkerboardUvScaled, 0 );
            float zs = abs( gIn_ViewZ.SampleLevel( gNearestMirror, uvScaled + gRectOffset, 0 ) );
        #else
            float4 s = gIn_Spec.SampleLevel( gNearestMirror, uvScaled, 0 );
            float zs = gIn_ScaledViewZ.SampleLevel( gNearestMirror, uvScaled, 0 ) / NRD_FP16_VIEWZ_SCALE;
        #endif

        float3 Xvs = STL::Geometry::ReconstructViewPosition( uv, gFrustum, zs, gIsOrtho );

        float materialIDs;
        Ns = NRD_FrontEnd_UnpackNormalAndRoughness( Ns, materialIDs );

        // Sample weight
        float w = IsInScreen( uv );
        w *= float( materialIDs == materialID );
        w *= GetGaussianWeight( offset.z );

        #ifdef REBLUR_SPATIAL_REUSE
            float4 dirPdf = NRD_FrontEnd_UnpackDirectionAndPdf( gIn_SpecDirectionPdf.SampleLevel( gNearestMirror, checkerboardUvScaled, 0 ) );
            float3 L = dirPdf.xyz;
            float3 H = normalize( L + V );
            float NoL = saturate( dot( N, L ) );
            float NoH = saturate( dot( N, H ) );
            float VoH = saturate( dot( V, H ) );
            float D = STL::BRDF::DistributionTerm( roughness, NoH );
            float G = STL::BRDF::GeometryTermMod_SmithUncorrelated( roughness, NoL, NoV, VoH, NoH );

            w *= GetRoughnessWeight( roughnessWeightParams, Ns.w );

            float2 ww = w;
            ww.x *= D * G * NoL / ( dirPdf.w + 0.001 );
            ww.x *= GetGeometryWeight( geometryWeightParamsReuse, Nv, Xvs );
            ww.x *= GetNormalWeight( normalWeightParamsReuse, N, Ns.xyz );

            #if( REBLUR_USE_SPATIAL_REUSE_FOR_HIT_DIST == 1 )
                ww.y = ww.x;
            #else
                ww.y *= GetGeometryWeight( geometryWeightParams, Nv, Xvs );
                ww.y *= GetNormalWeight( normalWeightParams, N, Ns.xyz );
                ww.y *= lerp( minwh.y, 1.0, GetHitDistanceWeight( hitDistanceWeightParams, s.w ) );
            #endif
        #else
            float2 ww = GetCombinedWeight( w, geometryWeightParams, Nv, Xvs, normalWeightParams, N, Ns, hitDistanceWeightParams, s.w, minwh, roughnessWeightParams );
        #endif

        s.xyz = STL::Color::Compress( s.xyz, exposure );

        spec += s * ww.xxxy;
        sum += ww;
    }

    spec *= STL::Math::PositiveRcp( sum.xxxy );

    spec.xyz = STL::Color::Decompress( spec.xyz, exposure );

    // Estimate error
    error.z = GetColorErrorForAdaptiveRadiusScale( spec, center, specInternalData.x, roughness );

    // Input mix
    spec = lerp( spec, center, REBLUR_INPUT_MIX.xxxy );

    // Output
    gOut_Spec[ pixelPos ] = spec;
}

#undef POISSON_SAMPLE_NUM
#undef POISSON_SAMPLES
