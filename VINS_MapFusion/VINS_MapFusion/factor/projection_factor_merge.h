//
//  projection_factor_merge.h
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/9/10.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef projection_factor_merge_h
#define projection_factor_merge_h

#include <stdio.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utility.hpp"

class ProjectionFactor_merge : public ceres::SizedCostFunction<2, 3,3, 7 , 7>
{
public:
    ProjectionFactor_merge(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
        
    //pts_i是主地图的世界坐标，pts_j是小地图的相机平面坐标
    Eigen::Vector3d pts_i, pts_j;//相机平面坐标 两个都是相机平面坐标
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};


class MultiClientFactor_merge : public ceres::SizedCostFunction<3, 1,3,3,3,4,3,4>
{
public:
//    MultiClientFactor_merge(const Eigen::Vector3d &_relative_t, const double &_relative_yaw,const double &_euler_pitch, const double &_euler_roll,const double &_euler_pitch_j, const double &_euler_roll_j);
    MultiClientFactor_merge(const Eigen::Vector3d &_relative_t, const double &_euler_pitch, const double &_euler_roll);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
        
    //
    Eigen::Vector3d relative_t;//
    double relative_yaw,euler_pitch,euler_roll,euler_pitch_j,euler_roll_j;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

class MultiClientFactor_mergePR : public ceres::SizedCostFunction<3, 1,3,3,3,4,3,4>
{
public:
//    MultiClientFactor_merge(const Eigen::Vector3d &_relative_t, const double &_relative_yaw,const double &_euler_pitch, const double &_euler_roll,const double &_euler_pitch_j, const double &_euler_roll_j);
    MultiClientFactor_mergePR(const Eigen::Vector3d &_relative_t, const double &_euler_pitch, const double &_euler_roll);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
        
    //
    Eigen::Vector3d relative_t;//
    double relative_yaw,euler_pitch,euler_roll,euler_pitch_j,euler_roll_j;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};


#endif /* projection_factor_merge_h */
