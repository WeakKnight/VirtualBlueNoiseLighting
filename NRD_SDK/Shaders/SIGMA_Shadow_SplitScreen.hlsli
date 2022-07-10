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
#include "SIGMA_Shadow_SplitScreen.resources.hlsli"

NRD_DECLARE_CONSTANTS

#include "NRD_Common.hlsli"
NRD_DECLARE_SAMPLERS
#include "SIGMA_Common.hlsli"

NRD_DECLARE_INPUT_TEXTURES
NRD_DECLARE_OUTPUT_TEXTURES

[numthreads( GROUP_X, GROUP_Y, 1)]
NRD_EXPORT void NRD_CS_MAIN( uint2 pixelPos : SV_DispatchThreadId)
{
    float2 pixelUv = float2( pixelPos + 0.5 ) * gInvRectSize;
    uint2 pixelPosUser = gRectOrigin + pixelPos;

    if( pixelUv.x > gSplitScreen )
        return;

    float2 data = gIn_Hit_ViewZ[ pixelPosUser ];
    float viewZ = abs( data.y ) / NRD_FP16_VIEWZ_SCALE;

    SIGMA_TYPE s;
    #ifdef SIGMA_TRANSLUCENT
        s = gIn_Shadow_Translucency[ pixelPosUser ];
    #else
        s = float( data.x == NRD_FP16_MAX );
    #endif

    gOut_Shadow_Translucency[ pixelPos ] = s * float( viewZ < gInf );
}
