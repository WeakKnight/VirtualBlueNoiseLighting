/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "NRD.hlsli"
#include "STL.hlsli"
#include "REBLUR_DiffuseSpecular_PostBlur.resources.hlsli"

NRD_DECLARE_CONSTANTS

#include "NRD_Common.hlsli"
NRD_DECLARE_SAMPLERS
#include "REBLUR_Common.hlsli"

NRD_DECLARE_INPUT_TEXTURES
NRD_DECLARE_OUTPUT_TEXTURES

[numthreads( GROUP_X, GROUP_Y, 1 )]
NRD_EXPORT void NRD_CS_MAIN( int2 threadId : SV_GroupThreadId, int2 pixelPos : SV_DispatchThreadId, uint threadIndex : SV_GroupIndex )
{
    uint2 pixelPosUser = gRectOrigin + pixelPos;
    float2 pixelUv = float2( pixelPos + 0.5 ) * gInvRectSize;

    // Early out
    float viewZ = gIn_ScaledViewZ[ pixelPos ] / NRD_FP16_VIEWZ_SCALE;

    [branch]
    if( viewZ > gInf )
        return;

    // Normal and roughness
    float materialID;
    float4 normalAndRoughness = NRD_FrontEnd_UnpackNormalAndRoughness( gIn_Normal_Roughness[ pixelPosUser ], materialID );
    float3 N = normalAndRoughness.xyz;
    float3 Nv = STL::Geometry::RotateVector( gWorldToView, N );
    float roughness = normalAndRoughness.w;

    // Internal data
    float curvature;
    float4 internalData = UnpackDiffSpecInternalData( gIn_InternalData[ pixelPos ], curvature );
    float2 diffInternalData = internalData.xy;
    float2 specInternalData = internalData.zw;

    // Shared data
    float3 Xv = STL::Geometry::ReconstructViewPosition( pixelUv, gFrustum, viewZ, gIsOrtho );
    float4 rotator = GetBlurKernelRotation( REBLUR_POST_BLUR_ROTATOR_MODE, pixelPos, gRotator, gFrameIndex );
    float4 error = gInOut_Error[ pixelPos ];

    // Spatial filtering
    #define REBLUR_SPATIAL_MODE REBLUR_POST_BLUR

    #if( defined REBLUR_DIFFUSE )
        float4 diff = gIn_Diff[ pixelPos ];

        #include "REBLUR_Common_DiffuseSpatialFilter.hlsli"
    #endif

    #if( defined REBLUR_SPECULAR )
        float4 spec = gIn_Spec[ pixelPos ];

        #include "REBLUR_Common_SpecularSpatialFilter.hlsli"
    #endif

    // Output
    // no output to "gInOut_Error" because error is not used in next passes
}
