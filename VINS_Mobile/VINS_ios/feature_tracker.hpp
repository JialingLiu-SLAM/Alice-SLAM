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

#include "prior_map_feature.hpp"

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

#include "CameraFactory.h"
#include "PinholeCamera.h"
using namespace camodocal;


//ios
//#define MAX_CNT 70
//#define MIN_DIST 30
//#define COL 480
//#define ROW 640

//euroc
#define MAX_CNT 150
#define MIN_DIST 30
#define COL  752
#define ROW 480

//xiaomi
//#define MAX_CNT 70
//#define MIN_DIST 30
//#define COL 640
//#define ROW 480

//kitti0930
//#define MAX_CNT 200
//#define MIN_DIST 30
//#define COL 1392
//#define ROW 512

//day1
//#define MAX_CNT 150
//#define MIN_DIST 30
//#define COL 752
//#define ROW 480

#define MAX_CNT_priorMap 50



#define F_THRESHOLD 1.0
#define EQUALIZE 1
//using namespace cv;
using namespace std;
//using namespace Eigen;
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
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
};

class KeyFrame;
class PriorMapFeature;


class FeatureTracker
{
public:
    FeatureTracker();
    bool solveVinsPnP(double header, Eigen::Vector3d &P, Eigen::Matrix3d &R, bool vins_normal);
    void readImage(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, vector<cv::Point2f> &good_pts, vector<double> &track_len, double header, Eigen::Vector3d &P, Eigen::Matrix3d &R, bool vins_normal);
    void readImage2(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, vector<cv::Point2f> &good_pts, vector<double> &track_len, double header, Eigen::Vector3d &P, Eigen::Matrix3d &R, bool vins_normal);
    void setMask();
    void rejectWithF();
    void addPoints();
    void addPoints2();
    bool updateID(unsigned int i);
    void undistortedPoints();
    void readIntrinsicParameter();
    
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
    vector<cv::Point2f> n_pts,cur_pts,pre_pts,forw_pts;//像素坐标系的点
    
//    id初始化为-1，track_cnt初始化为1 更新特征点id的步骤被特意放到了回调函数img_callback()中
// 特征点id集合 ，track_cnt特征点跟踪次数集合
    vector<int> ids,track_cnt;
    vector<max_min_pts> parallax_cnt;
    static int n_id;
    int img_cnt;//图片是每3张处理一次，所以值为 0 1 2
    double current_time;
    vinsPnP vins_pnp;
    bool use_pnp;
    
    /*
     interface
     */
    //图像坐标系
    map<int, Eigen::Vector3d> image_msg;//图像的点云 //相机平面坐标系 不能完全说是归一化 是因为2维到3维只能恢复出两个维度，另一个维度设为1
    bool update_finished;
    list<IMG_MSG_LOCAL> solved_features;//2D观测点 3D坐标 跟踪次数
    VINS_RESULT solved_vins;
    vector<IMU_MSG_LOCAL> imu_msgs;//加速度 角速度
    
    //ljl
    bool start_playback_dataEuroc;
    bool start_playback_dataKitti0930;
    bool start_playback_mvsec;
    //用于去畸变
    camodocal::CameraPtr m_camera;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;//相机平面点
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    
    map<int, Eigen::Vector3d> distorted_image_msg;//像素坐标系
    
    //临时用一下
    void undistortedPoints_handlerImg(cv::Mat &img);
    
//    ---------------------------增加先验地图特征---------------------------
    PriorMapFeature* priorMapFeature;
    std::mutex priorMap_mutex;
    bool isUpdate;//主要是告诉提取新的特征点 我们更新了局部地图点

    queue<KeyFrame*> old_kf_priorMap;
    queue<KeyFrame*> curLoopKf_priorMap;
    KeyFrame* cur_kf_priorMap;
    queue<double*> loop_pose_priorMap;
    void setLoopKf(double (&_loop_pose_forFeatureTracker)[7]);
    
    vector<cv::Point2f> pre_pMap_pts, cur_pMap_pts, forw_pMap_pts;
    std::vector<cv::Point2f> measurements_cur_coarse_new;//相机平面坐标
    std::vector<Eigen::Vector3d> point_3d_old_new;
    std::vector<int> feature_id_cur_new;
    double header_forwKf;
    std::vector<cv::Point2f> measurements_cur_coarse_pixel_new;//像素坐标

    vector<int> priorMap_ids;

    vector<int> priorMap_track_cnt;

    static int n_priorMap_id;
    
    void addPoints3();

    void readImage3(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, vector<cv::Point2f> &good_pts, vector<double> &track_len, double header, Eigen::Vector3d &P, Eigen::Matrix3d &R, bool vins_normal);
    void rejectWithF_priorMap();

};
#endif /* feature_tracker_hpp */
