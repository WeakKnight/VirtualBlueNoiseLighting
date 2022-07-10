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
#include "REBLUR_DiffuseSpecular_SplitScreen.resources.hlsli"

NRD_DECLARE_CONSTANTS

#include "NRD_Common.hlsli"
NRD_DECLARE_SAMPLERS
#include "REBLUR_Common.hlsli"

NRD_DECLARE_INPUT_TEXTURES
NRD_DECLARE_OUTPUT_TEXTURES

[numthreads( GROUP_X, GROUP_Y, 1)]
NRD_EXPORT void NRD_CS_MAIN( uint2 pixelPos : SV_DispatchThreadId)
{
    float2 pixelUv = float2( pixelPos + 0.5 ) * gInvRectSize;
    uint2 pixelPosUser = gRectOrigin + pixelPos;

    if( pixelUv.x > gSplitScreen )
        return;

    float viewZ = gIn_ViewZ[ pixelPosUser ];
    uint2 checkerboardPos = pixelPos;

    #if( defined REBLUR_DIFFUSE )
        checkerboardPos.x = pixelPos.x >> ( gDiffCheckerboard != 2 ? 1 : 0 );
        float4 diffResult = gIn_Diff[ gRectOrigin + checkerboardPos ];
        gOut_Diff[ pixelPos ] = diffResult * float( viewZ < gInf );
    #endif

    #if( defined REBLUR_SPECULAR )
        checkerboardPos.x = pixelPos.x >> ( gSpecCheckerboard != 2 ? 1 : 0 );
        float4 specResult = gIn_Spec[ gRectOrigin + checkerboardPos ];
        gOut_Spec[ pixelPos ] = specResult * float( viewZ < gInf );
    #endif
}
