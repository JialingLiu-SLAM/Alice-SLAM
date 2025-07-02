//
//  VINS.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/27.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef VINS_hpp
#define VINS_hpp

#include "utility.hpp"
#include "projection_facor.hpp"
#include <opencv2/core/eigen.hpp>
#include "global_param.hpp"
#include "FeatureManager.hpp"
#include <stdio.h>
#include <queue>

//存储闭环检测结果，包块闭环帧和匹配帧的位姿关系、匹配帧的ID、闭环帧的位姿、闭环帧中良好的匹配点以及匹配正良好匹配点的ID
struct RetriveData
{
    /* data */
    int old_index;
    int cur_index;
    double header;
    Vector3d P_old;
    Quaterniond Q_old;
    Vector3d P_cur;
    Quaterniond Q_cur;
    vector<cv::Point2f> measurements;
    vector<int> features_ids;
    bool use;
    Vector3d relative_t;
    Quaterniond relative_q;
    double relative_yaw;
    double loop_pose[7];
    
    vector<vector<int> > features_ids_all;
    vector<vector<Eigen::Vector3d>> point_clouds_all;
    vector<double> header_all;
};

class VINS
{
public:
    VINS();
    
    FeatureManager f_manager;//滑窗的特征点管理器
    
    //相机和imu之间的外参
    Matrix3d ric;//已给值
    Vector3d tic;
    Vector3d Ps[10 * (WINDOW_SIZE + 1)];//里面有尺度信息，各个帧相对于b0帧的平移 世界坐标系
    Matrix3d Rs[10 * (WINDOW_SIZE + 1)];//还未给初值
    
    
    //for loop closure
    RetriveData retrive_pose_data, front_pose;
    
    Vector3d t_drift;//未完 还没给值
    Matrix3d r_drift;
    
    vector<double> Headers;//这个意思改了 存放的是窗口内的关键帧 
    Vector3d g;//重力加速度
    
    
    //for falure detection
    bool failure_hand;
    int fail_times;
    
    
    void setIMUModel(double FOCUS_LENGTH_X_server);
    void setExtrinsic();
    
    
    //把公共区域的都搬到这里了
    queue<int> kf_global_index;
    bool start_global_optimization = false;
    queue<int> start_kf_global_index;
    std::mutex globalOpti_index_mutex;
    //ljl
    bool isSendLoopData;//这个是告知客户端 有回环了，赶紧去优化出相对位姿
    bool isSendLoop_another;//这个只是告知客户端 我有回环了
    bool isSendLoop_another2;//这个只是告知客户端 我有回环了和相对位姿
    
    queue<vector<uchar>> send_status;
    queue<int> send_status_index;
    
    
};



#endif /* VINS_hpp */
