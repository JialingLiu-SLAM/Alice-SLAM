//
//  feature_manager.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/20.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef feature_manager_hpp
#define feature_manager_hpp

#include <stdio.h>
#include <list>
#include <vector>
#include <stdlib.h>
#include <iostream>
//#include "feature_tracker.hpp"
#include <eigen3/Eigen/Dense>
#include "global_param.hpp"
#include "utility.hpp"
//#include "keyframe.h"

#include "DVision.h"

#define COMPENSATE_ROTATION false
//ios
//#define MIN_PARALLAX_POINT ((double)(3.0/549))
//#define MIN_PARALLAX ((double)(10.0/549))
//euroc
#define MIN_PARALLAX_POINT ((double)(3.0/460))
#define MIN_PARALLAX ((double)10.0/460)

//kitti0930
//#define MIN_PARALLAX_POINT ((double)(3.0/980))
//#define MIN_PARALLAX ((double)10.0/980)

//day1
//#define MIN_PARALLAX_POINT ((double)(3.0/466))
//#define MIN_PARALLAX ((double)10.0/466)


#define INIT_DEPTH ((double)(5.0))

//using namespace Eigen;
using namespace std;

class FeaturePerFrame//一个特征点的属性 空间特征点映射到帧上对应图像坐标、特征点跟踪速度、空间坐标
{
public:
    //保存了归一化坐标，图像坐标以及深度
    FeaturePerFrame(const Eigen::Vector3d &_point)
    {
        z = _point(2);
        point = _point / z;
    }
    Eigen::Vector3d point;//特征点的空间坐标  归一化坐标？ 相机平面坐标系的点
    double z;//好像一直为1
    bool is_used;
    double parallax;
    double dep_gradient;
    
    //ljl
    //这里存一下 没有去畸变的点 像素坐标
    Eigen::Vector3d distorted_point_uv;
    
};
class KeyFrame;
class FeaturePerId//管理一个特征点
{
public:
    //以feature_id为索引，并保存了出现该角点的第一帧id
    const int feature_id;//特征点
    int start_frame;//第一次观测到该点的帧 序号
    vector<FeaturePerFrame> feature_per_frame;//管理对应帧的属性 一个3D点被一部分帧观测到，该3D点在对应帧的像素坐标信息
    
    int used_num;//被观测的帧数
    bool is_margin;
    bool is_outlier;
    
    double estimated_depth;//深度 乘了尺度的
    
    bool fixed;
    bool in_point_cloud;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail; 该特征点的状态 是否被三角化
    
    FeaturePerId(int _feature_id, int _start_frame)
    : feature_id(_feature_id), start_frame(_start_frame),
    used_num(0), estimated_depth(-1.0),is_outlier(false),fixed(false),in_point_cloud(false)
    {
    }
    
    int endFrame();//最后一次观测到该点的帧 序号
    
    //ljl
    //存关键帧地址
    vector<KeyFrame*> per_kf;//存储所有观测到它的关键帧，上面是包括普通帧的
};

class FeatureManager//管理所有特征点
{
public:
    FeatureManager(Eigen::Matrix3d _Rs[]);
    bool addFeatureCheckParallax(int frame_count, const map<int, Eigen::Vector3d> &image_msg, int &parallax_num);
    bool addFeatureCheckParallax(int frame_count, const map<int, Eigen::Vector3d> &image_msg, int &parallax_num, const map<int, Eigen::Vector3d> &distorted_image);
    
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    void triangulate(Eigen::Vector3d Ps[], Eigen::Vector3d tic, Eigen::Matrix3d ric, bool is_nonlinear);
    Eigen::VectorXd getDepthVector();
    
    int getFeatureCount();
    void clearState();
    void tagMarginalizedPoints(bool marginalization_flag);
    void removeBack();
    void removeFront(int frame_count);
    void setDepth(const Eigen::VectorXd &x);
    void clearDepth(const Eigen::VectorXd &x);
    void shift(int n_start_frame);
    void removeFailures();
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    /*
     variables
     */
    //通过vins内的FeatureManager可以得到滑动窗口内所有的角点信息
    list<FeaturePerId> feature;//存储每一个特征点，及其对应的每帧的属性
    std::vector<std::pair<int, std::vector<int>>> outlier_info;
    int last_track_num;
    
private:
    double compensatedParallax1(FeaturePerId &it_per_id);
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Eigen::Matrix3d *Rs;
    Eigen::Matrix3d ric;
    
};

class Feature_local//管理一个3d点
{
public:
    //以feature_id为索引，并保存了出现该角点的第一帧id
    const int feature_id;//特征点
    DVision::BRIEF::bitset des;
    bool is_margin;
    bool is_outlier;
    Eigen::Vector3d point;
    
    
    cv::Point2f measurements_coarse;//用于实验 验证匹配点对是否准确
    cv::Point2f measurements_coarse_undistorted;
    double header;//记录是哪一帧看到的
    
    double estimated_depth;//深度 乘了尺度的
    
    bool fixed;
    bool in_point_cloud;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail; 该特征点的状态 是否被三角化
    
    Feature_local(int _feature_id)
    : feature_id(_feature_id),estimated_depth(-1.0),is_outlier(false),fixed(false),in_point_cloud(false)
    {
    }
    
};
#endif /* feature_manager_hpp */
