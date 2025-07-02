//
//  initial_aligment.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/12/26.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef initial_aligment_hpp
#define initial_aligment_hpp

#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "imu_factor.h"
#include "utility.hpp"
#include <map>
#include "feature_manager.hpp"


//using namespace Eigen;
using namespace std;

class ImageFrame
{
public:
    ImageFrame(){};
    ImageFrame(const map<int, Eigen::Vector3d>& _points, double _t):points{_points},t{_t},is_key_frame{false}
    {
    };
    map<int, Eigen::Vector3d> points;
    double t;
    Eigen::Matrix3d R;//imu坐标系下的 确认过一遍 R_w_b 此时w是c0，后面才会更换到和g对齐的最终的世界坐标系
    Eigen::Vector3d T;//相机坐标系下的 R_w_c  此时w是c0，后面才会更换到和g对齐的最终的世界坐标系
    IntegrationBase *pre_integration;
    bool is_key_frame;
};

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &g, Eigen::VectorXd &x);

bool VisualOptiIMU(map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &g, Eigen::VectorXd &x);

#endif /* initial_aligment_hpp */
