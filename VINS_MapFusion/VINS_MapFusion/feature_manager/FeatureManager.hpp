//
//  FeatureManager.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/27.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef FeatureManager_hpp
#define FeatureManager_hpp

#include <eigen3/Eigen/Dense>
#include <list>
#include <map>
#include <vector>
#include <stdlib.h>


#include "global_param.hpp"
#include "utility.hpp"


#include <stdio.h>

using namespace Eigen;
using namespace std;

class FeaturePerFrame//一个特征点的属性 空间特征点映射到帧上对应图像坐标、特征点跟踪速度、空间坐标
{
public:
    FeaturePerFrame(const Vector3d &_point)
    {
        z = _point(2);
        point = _point / z;
    }
    Vector3d point;//特征点的空间坐标  归一化坐标？
    double z;
    bool is_used;
    double parallax;
    double dep_gradient;
    
    //ljl
//    int frame_id;//按理应该是连续的帧都能看到
};

class FeaturePerId//管理一个特征点
{
public:
    const int feature_id;//特征点
    int start_frame;//第一次观测到该点的帧 序号
    vector<FeaturePerFrame> feature_per_frame;//管理对应帧的属性
    
    int used_num;//出现的次数
    bool is_margin;
    bool is_outlier;
    
    double estimated_depth;//逆深度
    
    bool fixed;
    bool in_point_cloud;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail; 该特征点的状态 是否被三角化
    
    FeaturePerId(int _feature_id, int _start_frame)
    : feature_id(_feature_id), start_frame(_start_frame),
    used_num(0), estimated_depth(-1.0),is_outlier(false),fixed(false),in_point_cloud(false)
    {
    }
    
    int endFrame();//最后一次观测到该点的帧 序号
};

class FeatureManager//管理所有特征点
{
public:
    FeatureManager(Matrix3d _Rs[]);
    bool addFeatureCheckParallax(int frame_count, const map<int, Vector3d> &image_msg, int &parallax_num);
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    void triangulate(Vector3d Ps[], Vector3d tic, Matrix3d ric, bool is_nonlinear);
    VectorXd getDepthVector();
    
    int getFeatureCount();
    void clearState();
    void tagMarginalizedPoints(bool marginalization_flag);
    void removeBack();
    void removeFront(int frame_count);
    void setDepth(const VectorXd &x);
    void clearDepth(const VectorXd &x);
    void shift(int n_start_frame);
    void removeFailures();
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    /*
     variables
     */
    list<FeaturePerId> feature;//存储每一个特征点，及其对应的每帧的属性
    std::vector<std::pair<int, std::vector<int>>> outlier_info;
    int last_track_num;
    
private:
    double compensatedParallax1(FeaturePerId &it_per_id);
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric;
    
};


#endif /* FeatureManager_hpp */
