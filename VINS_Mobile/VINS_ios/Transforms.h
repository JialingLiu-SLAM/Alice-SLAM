//
//  Transforms.h
//  VINS_ios
//
//  Created by 张剑华 on 2023/5/8.
//  Copyright © 2023 栗大人. All rights reserved.
//


#ifndef UPANDRUNNING3D_TRANSFORMS_H
#define UPANDRUNNING3D_TRANSFORMS_H

#include <simd/simd.h>

// This function returns the identity matrix
simd::float4x4 Identity();

// This function constructs a symmetric perspective projection matrix.
// fovy is the vertical field of view, in radians
simd::float4x4 PerspectiveProjection(float aspect, float fovy, float near, float far);

// This function constructs a matrix that rotates points around the
// specified axis by the specified angle (in radians)
simd::float4x4 Rotation(simd::float3 axis, float angle);

simd::float4x4 matrix_uniform_scale(float s);

#endif
