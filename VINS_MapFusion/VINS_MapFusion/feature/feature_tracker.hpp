//
//  feature_tracker.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//
#include <opencv2/opencv.hpp>
#ifndef feature_tracker_hpp
#define feature_tracker_hpp

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <eigen3/Eigen/Dense>
#include "global_param.hpp"
#include <string>
#include <list>
#include "utility.hpp"
#include <opencv2/core/eigen.hpp>
#include "vins_pnp.hpp"

#define MAX_CNT 70
#define MIN_DIST 30
//ios
//#define COL 480
//#define ROW 640
//euroc
#define COL 752
#define ROW 480
//xiaomi
//#define COL 640
//#define ROW 480
#define F_THRESHOLD 1.0
#define EQUALIZE 1
//using namespace cv;
using namespace std;
using namespace Eigen;
/*
 image frame
 --------> x:480
 |
 |
 |
 |
 |
 | y:640
 */
struct max_min_pts{
    cv::Point2f min;
    cv::Point2f max;
};

struct IMU_MSG_LOCAL {
    double header;
    Vector3d acc;
    Vector3d gyr;
};

class FeatureTracker
{
public:
    FeatureTracker();
    bool solveVinsPnP(double header, Vector3d &P, Matrix3d &R, bool vins_normal);
    void readImage(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, vector<cv::Point2f> &good_pts, vector<double> &track_len, double header, Vector3d &P, Matrix3d &R, bool vins_normal);
    void setMask();
    void rejectWithF();
    void addPoints();
    bool updateID(unsigned int i);
    
    /*
     varialbles
     */
    int frame_cnt;
    cv::Mat mask;
//    prev_img： 上一次发布数据时对应的图像帧
//    cur_img： 光流跟踪的前一帧图像，而不是“当前帧”
//    forw_img： 光流跟踪的后一帧图像，真正意义上的“当前帧”
    cv::Mat cur_img, pre_img, forw_img;
//    prev_pts： 上一次发布的，且能够被当前帧（forw）跟踪到的特征点
//    cur_pts： 在光流跟踪的前一帧图像中，能够被当前帧（forw）跟踪到的特征点
//    forw_pts： 光流跟踪的后一帧图像，即当前帧中的特征点（除了跟踪到的特征点，可能还包含新检测的特征点）
    vector<cv::Point2f> n_pts,cur_pts,pre_pts,forw_pts;
    
//    id初始化为-1，track_cnt初始化为1 更新特征点id的步骤被特意放到了回调函数img_callback()中
    vector<int> ids,track_cnt;
    vector<max_min_pts> parallax_cnt;
    static int n_id;
    int img_cnt;
    double current_time;
    vinsPnP vins_pnp;
    bool use_pnp;
    
    /*
     interface
     */
    map<int, Vector3d> image_msg;//图像的点云
    bool update_finished;
    list<IMG_MSG_LOCAL> solved_features;//2D观测点 3D坐标 跟踪次数
    VINS_RESULT solved_vins;
    vector<IMU_MSG_LOCAL> imu_msgs;//加速度 角速度
    
    
    
    double FOCUS_LENGTH_Y_server;
    double PY_server;
    double FOCUS_LENGTH_X_server;
    double PX_server;
    void setCam_intrinsic(DeviceType device);
    
    
};
#endif /* feature_tracker_hpp */
