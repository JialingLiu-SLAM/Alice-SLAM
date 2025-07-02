//
//  Client.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/25.
//  Copyright © 2020 zx. All rights reserved.
//
#include "PinholeCamera.h"
#ifndef Client_hpp
#define Client_hpp

#include <stdio.h>
#include <iostream>

#include<queue>
#include <cstdio>
#include <cstdlib>
#include "global_param.hpp"

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include<opencv2/core/core.hpp>

#include "CameraFactory.h"

#include "feature_coder.hpp"

using namespace camodocal;

#define MAX_NAME_LENGHT 20

using namespace Eigen;




class Client {
  public:
    char *name;
    int id;
    cv::Mat Twclient;
    
    bool is_add;//判断是否被加入到PoseGraphGlobal
    
    bool status;//为false表示连接断开，为true表示能正常通信，默认一开始通信是刚连接上的

    //Socket stuff
    int sock;
    
    std::queue<std::pair<char *,int>> buffer_kindsAndLen;//
    std::queue<std::pair<char *,int>> buffer_all_split;//
    std::mutex* buffer_kindsAndLen_mutex;

    Client();
    void SetName(const char *name);
    void SetId(int id);
    int getId();
//    static Client* getClientById(int id);
    
    //--------------------相机内参
    double FOCUS_LENGTH_Y_server;
    double PY_server;
    double FOCUS_LENGTH_X_server;
    double PX_server;
    double TIC_X_server;
    double TIC_Y_server;
    double TIC_Z_server;
    
    double RIC_y_server;
    double RIC_p_server;
    double RIC_r_server;
    void setCam_intrinsic(DeviceType device);
    
    void setWidth_Height(int width,int height);
    int getWidth();
    int getHeight();
    
    Matrix3d ric_client;//已给值
    Vector3d tic_client;
    
//压缩器 人手一个
    LBFC2::CodingStats codingModel;
    LBFC2::FeatureCoder *encoder;
    LBFC2::FeatureCoder *decoder;
    
    
//    //记录 检测回环后 全局优化做到哪个位置了 记住最后一个有回环的KF下标
//    int kf_loop_global_index=-1;
    //去畸变用
    bool isUndistorted;//不需要畸变则为false
    double k1_global;
    double k2_global;
    double p1_global;
    double p2_global;
    double FOCAL_LENGTH;
    camodocal::CameraPtr m_camera;
    void undistortedPoints(vector<cv::KeyPoint> &keypoints);
    void undistortedPoints_2f(vector<Eigen::Vector2f> &vpoints2f);
    void readIntrinsicParameter();
    
    //---------------------为锁准备的
    ~Client(){
        delete buffer_kindsAndLen_mutex;
    }
    Client(Client const& mutex_c)
    {
        buffer_kindsAndLen_mutex=new std::mutex();
    }
    Client& operator=(Client const& mutex_c){
        return *this;
    }
    
    //1021日 暂时注释
//private:
    int m_imageWidth;
    int m_imageHeight;
};


#endif /* Client_hpp */
