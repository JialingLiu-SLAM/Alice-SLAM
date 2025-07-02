//
//  OptimizeRelativePose.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2021/6/30.
//  Copyright © 2021 zx. All rights reserved.
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
#include "KeyFrame.hpp"
using namespace std;


extern double compute_inlier_residual_threshold( std::vector< double > residuals, double ratio );
 extern bool optiRelativePose_forMainMap(std::vector<cv::Point2f> measurements_old_norm_real, std::vector<Vector3d> point_3d_cur_real, KeyFrame* old_kf, KeyFrame* cur_kf, Matrix3d &R_relative, Vector3d &T_relative);
//第3篇论文的版本
extern bool optiRelativePose_forMainMap2(std::vector<cv::Point2f> measurements_old_norm_real, std::vector<Vector3d> point_3d_cur_real, KeyFrame* old_kf, KeyFrame* cur_kf, Matrix3d &R_relative, Vector3d &T_relative,std::vector<cv::Point2f> measurements_old);


#endif /* OptimizeRelativePose_hpp */
