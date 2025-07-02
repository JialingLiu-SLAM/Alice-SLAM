//
//  Shared.h
//  VINS_ios
//
//  Created by 张剑华 on 2023/5/8.
//  Copyright © 2023 栗大人. All rights reserved.
//

#ifndef UPANDRUNNING3D_SHARED_H
#define UPANDRUNNING3D_SHARED_H

#include <simd/simd.h>

typedef struct
{
//    Projection矩阵将其变换到屏幕空间
    simd::float4x4 modelViewProjectionMatrix;
    //用于MVP矩阵变换的模型视图矩阵 Model-View矩阵将顶点坐标变换到视图空间坐标系
    simd::float4x4 modelViewMatrix;
    simd::float3x3 normalMatrix;
} Uniforms;


#endif
