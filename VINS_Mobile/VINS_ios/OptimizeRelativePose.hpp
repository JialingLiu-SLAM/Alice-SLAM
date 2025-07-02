//
//  OptimizeRelativePose.hpp
//  VINS_ios
//
//  Created by 张剑华 on 2022/7/25.
//  Copyright © 2022 栗大人. All rights reserved.
//

#ifndef OptimizeRelativePose_hpp
#define OptimizeRelativePose_hpp


#include <stdio.h>
#include <algorithm>
#include <time.h>
#include <random>
#include "opencv2/imgproc/imgproc.hpp"
#include<opencv2/core/core.hpp>
#include <eigen3/Eigen/Dense>
#include "keyframe.h"
using namespace std;


extern double compute_inlier_residual_threshold( std::vector< double > residuals, double ratio );
 extern bool optiRelativePose_forMainMap(std::vector<cv::Point2f> measurements_old_norm_real, std::vector<Eigen::Vector3d> point_3d_cur_real, KeyFrame* old_kf, KeyFrame* cur_kf, Eigen::Matrix3d &R_relative, Eigen::Vector3d &T_relative);


#endif /* OptimizeRelativePose_hpp */
