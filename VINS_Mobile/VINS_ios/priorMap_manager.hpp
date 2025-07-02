//
//  priorMap_manager.hpp
//  VINS_ios
//
//  Created by 张剑华 on 2022/10/5.
//  Copyright © 2022 栗大人. All rights reserved.
//

#ifndef priorMap_manager_hpp
#define priorMap_manager_hpp

#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include "utility.hpp"
#include "global_param.hpp"
#include <opencv2/opencv.hpp>
#include <list>
using namespace std;

class FeaturePerFrame_localMap//一个特征点的属性 空间特征点映射到帧上对应图像坐标、特征点跟踪速度、空间坐标
{
public:
    //保存了归一化坐标，图像坐标以及深度
    FeaturePerFrame_localMap(const Eigen::Vector3d &_point)
    {
        z = _point(2);
        point = _point / z;
    }
    Eigen::Vector3d point;//特征点的空间坐标  归一化坐标？ 相机平面坐标系的点
    double z;//好像一直为1
   
};
class FeaturePerId_localMap//管理一个特征点
{
public:
    //以feature_id为索引，并保存了出现该角点的第一帧id
    const int feature_id;//特征点
    int start_frame;//第一次观测到该点的帧 序号
    vector<FeaturePerFrame_localMap> feature_per_frame;

    bool is_margin;
//    bool is_outlier;
    Eigen::Vector3d point;
    
//    double estimated_depth;//深度 乘了尺度的
    int endFrame();
    
    FeaturePerId_localMap(int _feature_id,int _start_frame, Eigen::Vector3d _point): feature_id(_feature_id),start_frame(_start_frame),point(_point)
    {
    }
    
};

class Feature_localMap//管理一个3d点
{
public:
    Feature_localMap(Eigen::Matrix3d _Rs[]);
    
    
    int getFeatureCount();
    void clearState();
    
    void addFeature(int frame_count, std::vector<cv::Point2f> measurements_cur_coarse_new, std::vector<Eigen::Vector3d> point_3d_old_new , std::vector<int> feature_id_cur_new);
    void removeBack();
    void removeFront(int frame_count);
    
   
    /*
     variables
     */
    //通过vins内的FeatureManager可以得到滑动窗口内所有的角点信息
    list<FeaturePerId_localMap> feature;//存储每一个特征点，及其对应的每帧的属性
    std::vector<std::pair<int, std::vector<int>>> outlier_info;
    int last_track_num;
    

   
    const Eigen::Matrix3d *Rs;
    Eigen::Matrix3d ric;
    
};

#endif /* priorMap_manager_hpp */
