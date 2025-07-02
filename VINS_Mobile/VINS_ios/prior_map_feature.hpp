//
//  prior_map_feature.hpp
//  VINS_ios
//
//  Created by 张剑华 on 2022/9/19.
//  Copyright © 2022 栗大人. All rights reserved.
//

#ifndef prior_map_feature_hpp
#define prior_map_feature_hpp

#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "keyframe.h"
//#include "feature_manager.hpp"
#include "global_param.hpp"
#include <vector>

#include "DVision.h"
#include "utility.hpp"

#define COL  752
#define ROW 480

using namespace std;
using namespace DVision;

class KeyFrame;

class Feature_PriorMap//管理一个3d点
{
public:
    //以feature_id为索引，并保存了出现该角点的第一帧id
     int feature_id;//特征点 const
    DVision::BRIEF::bitset des;//3d点对应的描述符 必须赋值
    bool is_margin;
    bool is_outlier;
    Eigen::Vector3d point;//这里是投影的3d点 必须赋值
    
    
    cv::Point2f measurements_coarse;//用于实验 验证匹配点对是否准确
    cv::Point2f measurements_coarse_undistorted;
    double header;//记录是哪一帧看到的
    
    double estimated_depth;//深度 乘了尺度的
    
    bool fixed;
    bool in_point_cloud;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail; 该特征点的状态 是否被三角化
    
    Feature_PriorMap(int _feature_id)
    : feature_id(_feature_id),estimated_depth(-1.0),is_outlier(false),fixed(false),in_point_cloud(false)
    {
    }
    
};



class PriorMapFeature {
public:
//    void detectPriorMapFeatures(KeyFrame* old_kf,KeyFrame* cur_kf,KeyFrame* curLoopKf,int forw_id,double loop_pose[7]);
//    void UpdateLocalKeyFrames(KeyFrame* oldKf,int curKf_id);
//    void UpdateLocalPoints();
    void detectPriorMapFeatures(KeyFrame* old_kf,KeyFrame* cur_kf,KeyFrame* curLoopKf,double* loop_pose_priorMap);
    
    void matchPriorMapFeatures();
    
    PriorMapFeature();

    //判断是否满足直线匹配的要去i
//    bool judgeMidPoint(cv::line_descriptor::KeyLine& cur_line, cv::line_descriptor::KeyLine& fowr_line);
//    bool judgeAngle(cv::line_descriptor::KeyLine& cur_line, cv::line_descriptor::KeyLine& fowr_line);
    void setParameter();
    void setLoopKf(double (&loop_pose_forFeatureTracker)[7]);
    list<Feature_PriorMap> feature_local;
    
    std::vector<cv::Point2f> measurements_cur_coarse_pixel_new;//像素坐标系
    std::vector<cv::Point2f> measurements_cur_coarse_new;//相机平面坐标
    std::vector<Eigen::Vector3d> point_3d_old_new;
    std::vector<int> feature_id_cur_new;
    double header;
    
    bool isUpdate;
private:
    Eigen::Matrix3d ric_curClient;
    Eigen::Vector3d tic_curClient;
    double fx ;
    double fy ;
    double cx ;
    double cy ;
    
//    KeyFrame* old_kf,curLoopKf;
    int curKf_id,forw_id;
    double loop_pose[7];

    
};

#endif /* prior_map_feature_hpp */
