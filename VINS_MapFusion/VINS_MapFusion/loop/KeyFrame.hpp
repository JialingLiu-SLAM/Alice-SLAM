//
//  KeyFrame.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/27.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef KeyFrame_hpp
#define KeyFrame_hpp

#include "Client.hpp"
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "utility.hpp"
#include "global_param.hpp"
#include "DVision.h" 
//#include "LoopClosure.hpp"
#include "DBoW2.h"

#include "BowVector.h"
#include "FeatureVector.h"

#include <fstream>
#include <stdio.h>

#include "ReadWriteLock.hpp"
using namespace Eigen;
using namespace std;
using namespace DVision;
using namespace DBoW2;



class KeyFrame{
    public:

        KeyFrame();
    
        ~KeyFrame(){cout<<"KeyFrame 释放内存"<<endl;}

        void rejectWithF(vector<cv::Point2f> &measurements_old,
                         vector<cv::Point2f> &measurements_old_norm);
        
        void searchInBoW(std::vector<cv::Point2f> &cur_pts,
                         std::vector<cv::Point2f> &old_pts,
                         std::vector<cv::Point2f> &old_measurements);
//        void buildKeyFrameFeatures(VINS &vins);
        
        void searchByDes(std::vector<cv::Point2f> &measurements_old,
                         std::vector<cv::Point2f> &measurements_old_norm,
                         const std::vector<BRIEF::bitset> &descriptors_old,
                         const std::vector<cv::KeyPoint> &keypoints_old);
        
        
        bool findConnectionWithOldFrame(const KeyFrame* old_kf,
                                        const std::vector<cv::Point2f> &cur_pts, const std::vector<cv::Point2f> &old_pts,
                                        std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm);
    //------------服务器 老帧3D 新帧2D
    
        void rejectWithF_server(vector<cv::Point2f> &measurements_old,
        vector<cv::Point2f> &measurements_old_norm,Client* c_old);
        
        void searchByDes_server(std::vector<cv::Point2f> &measurements_old,
        std::vector<cv::Point2f> &measurements_old_norm,
        const std::vector<BRIEF::bitset> &descriptors_old,
        const std::vector<cv::KeyPoint> &keypoints_old,Client* c_old);
        
        bool findConnectionWithOldFrame_server(const KeyFrame* old_kf,
        const std::vector<cv::Point2f> &cur_pts, const std::vector<cv::Point2f> &old_pts,
        std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm);
    
    //---------------服务器 老帧2D 新帧3D
    void rejectWithF_server_old(vector<cv::Point2f> &measurements_old,
    vector<cv::Point2f> &measurements_old_norm,Client* c_old);
    void rejectWithF_server_old2(vector<cv::Point2f> &measurements_old,
                                           vector<cv::Point2f> &measurements_old_norm,Client* c_old,std::vector<uchar> &status1);

    void searchByDes_server_old(std::vector<cv::Point2f> &measurements_old,
    std::vector<cv::Point2f> &measurements_old_norm,
    const std::vector<BRIEF::bitset> &descriptors_old,
                                          const std::vector<cv::KeyPoint> &keypoints_old,Client* c_old);
    void searchByDes_server_old2(std::vector<cv::Point2f> &measurements_old,
    std::vector<cv::Point2f> &measurements_old_norm,
    const std::vector<BRIEF::bitset> &descriptors_old,
                                          const std::vector<cv::KeyPoint> &keypoints_old,Client* c_old);
    
//    void rejectWithF_server_old(vector<cv::Point2f> &measurements_old,
//    vector<cv::Point2f> &measurements_old_norm,std::vector<int> features_id_old_matching,Client* c_old);
//
//    void searchByDes_server_old(std::vector<cv::Point2f> &measurements_old,
//    std::vector<cv::Point2f> &measurements_old_norm,
//    const std::vector<BRIEF::bitset> &descriptors_old,
//                                          const std::vector<cv::KeyPoint> &keypoints_old,Client* c_old);
    
    bool findConnectionWithOldFrame_server_old(const KeyFrame* old_kf, std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm);
    bool findConnectionWithOldFrame_server_old2(const KeyFrame* old_kf, std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm);
    //------------地图融合
    void rejectWithF_server_mapFuse(vector<cv::Point2f> &measurements_old, vector<cv::Point2f> &measurements_cur, vector<Eigen::Vector3d> &pointsCloud_old_3d);
    
    void rejectWithF_server_mapFuse2(vector<cv::Point2f> &measurements_old,
                                               vector<cv::Point2f> &measurements_cur, vector<Eigen::Vector3d> &pointsCloud_old_3d, vector<int> &featureId_cur);
    //------------位姿更新
    
        void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
        
        void updateOriginPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
        
        void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
        
        void getOriginPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
        
        void addConnection(int index, KeyFrame* connected_kf);
        
        void addConnection(int index, KeyFrame* connected_kf, Vector3d relative_t, Quaterniond relative_q, double relative_yaw);
        
        void addLoopConnection(int index, KeyFrame* loop_kf);
        
        void updateLoopConnection(Vector3d relative_t, Quaterniond relative_q, double relative_yaw);
        
        void detectLoop(int index);
        
        void removeLoop();
        
        int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);
    
        void setWin_keyPoint_des();
        
        // data
        double header;//
        std::vector<Eigen::Vector3d> point_clouds, point_clouds_origin;//存放win里面的3D坐标
        //feature in origin image plane 像素坐标
        std::vector<cv::Point2f> measurements, measurements_origin;//给了初值 已给完
        //feature in normalize image plane
        std::vector<cv::Point2f> pts_normalize;//measurements 3D点变平面点 相机平面点
        //feature ID
        std::vector<int> features_id, features_id_origin;//特征点id 窗口内能被2帧观测到，且观测到他的第一帧不位于窗口后方
        //feature descriptor
        std::vector<BRIEF::bitset> descriptors;
        //keypoints 像素坐标
        std::vector<cv::KeyPoint> keypoints;
        //3d 空间点 应该没有填错
        std::vector<Eigen::Vector3d> pts_3d_server,pts_3d_server_origin;
        
        int global_index;//一个地图全局索引
    
    vector<int> desBow_index;//参考下标 通讯完后，清空了。后面如果要用再改
    bool is_bowIndex_end;
    vector<uchar> bitstream;//压缩描述符
    int  bitNum;//用于通讯的时候判断 需要发送压缩描述符的总量
            
        
    
//        const char *BRIEF_PATTERN_FILE;
        // index t_w t_y t_z q_w q_x q_y q_z yaw
        
    
        //窗口点的深度
        std::vector<double> win_keyPoint_depth,win_keyPoint_depth_origin;
        std::vector<double> win_point_z,win_point_z_origin;//窗口特征点的z 初始化了
    
    //这个可能还没赋值 但是已经用了 赋值了 是客户端那边传过来的 做了一定的优化的
//    Matrix3d qic;
//    Vector3d tic;
    void cam2Imu(Eigen::Vector3d T_c_w,Eigen::Matrix3d R_c_w,Eigen::Vector3d &t_w_i,Eigen::Matrix3d &r_w_i);
    void cam2Imu(Eigen::Vector3f T_c_w, Eigen::Matrix3f R_c_w, Eigen::Vector3f &t_w_i,
        Eigen::Matrix3f &r_w_i);
        
private:
    Eigen::Vector3d T_w_i;//初始化好了 这个应该读 从imu到w坐标系的转换
    Eigen::Matrix3d R_w_i;//
    Eigen::Vector3d origin_T_w_i;//
    Eigen::Matrix3d origin_R_w_i;//
    std::mutex mMutexPose;
    std::vector<cv::KeyPoint> window_keypoints;//
    std::vector<BRIEF::bitset> window_descriptors;//
    
    std::mutex mMutex_w1Pose;
    Eigen::Vector3d T_w1_i;
    Eigen::Matrix3d R_w1_i;
    
    //ljl
public:
    
    void update_w1Pose(const Eigen::Vector3d &_T_w1_i, const Eigen::Matrix3d &_R_w1_i);
    void get_w1Pose(Eigen::Vector3d &_T_w1_i, Eigen::Matrix3d &_R_w1_i);
    
    bool IsOriginUpdate;//收到原始位姿
    bool sendLoop;//这个判断是否做过全局优化了
    bool sendRejectWithF;
    vector<uchar> send_status,send_status_new;//前一次 send_status_new还没考虑清楚要不要，暂时是放多个机器人融合 得到的新的status
    
    //存放所有地图匹配的起始位置 暂时不用
    std::pair<int, int> recordMapFusionIndex;//前面是clientId ,后面是匹配到了哪个帧
    
    //定义一个子地图     
        
    //画图用的 还没给值 初始值给了
    Eigen::Vector3d relocalize_t;//一开始的初始值是相机位姿
    Eigen::Matrix3d relocalize_r;
    void getPath(Eigen::Vector3d& path);//返回平移
    
    //记录每个关键帧回环到了哪一帧
    std::pair<int ,int> recordLoopStartIndex;//clientID 匹配到了这个位置
    
    //这里保存的是和自己地图的一个回环
    void detectFusion(int old_index);
    int fusion_index;
    bool has_fusion;
    bool is_fusioned;
    
    bool use_retrive;
    bool has_loop;//表示有没有回环，但是那个错误的回环 然后删除还没加
    bool check_loop;//记录做过回环检测了
    
    // looped by other frame
    bool is_looped;
    int loop_index;
    int resample_index;
    
    list<pair<int, Eigen::Matrix<double, 8, 1 > > > connection_list;
//    Eigen::Matrix<double, 8, 1 > loop_info;
    Eigen::Matrix<double, 4, 1 > loop_info_better;
    void updateLoopConnection(Vector3d relative_t, double relative_yaw);
    Quaterniond loop_info_better_q;
    double relative_pitch;
    double relative_roll;
    //这个是给地图内部的回环用的
    bool isRemove_loop;//默认为false 如果为true，表示发生了回环，并且本身的位姿估计已经非常准确了，所以需要降低优化边的权重，防止过拟合
    bool is_get_loop_info;//记录loop_info是否更新了
    int segment_index;//序列号


    void update_globalLoopConnection(Vector3d relative_t, Quaterniond relative_q, double relative_yaw);
    void detect_globalLoop(int index);
    Eigen::Matrix<double, 8, 1 > global_loop_info;
    double global_relative_pitch;
    double global_relative_roll;
    //用于做多个地图的全局优化
    int resample_globalIndex;
    //给多地图融合用
    bool isRemove_globalLoop;//默认为false 如果为true，表示发生了回环，并且本身的位姿估计已经非常准确了，所以需要降低优化边的权重，防止过拟合
    
    //这里到时候要改 赋值还没给
    bool has_global_loop;
    int global_loop_index;
    bool check_global_loop[10];
    
    bool is_Send;//这个是判断如果这个真和其它地图发生了回环 要发送给Client，主要是为了后期 多个地图融合优化后的位姿更新，如何分割序列边 所以后面如果发生了两次，也并不会影响

    
    bool is_global_looped;//这个是记录有没有被回环 两个地图之间
    int is_global_looped_index;//这里表示是哪个帧和它发生了回环
    void is_detected_globalLoop(int index);
    
//    bool pre_has_global_loop;
//    int pre_global_loop_index;
//    bool is_get_global_loop_info;//这里是表示有没有进行局部优化 得到两帧之间的relative
    
    
    bool is_des_end;//因为des是分开发的，记录一下是否接收完毕了
    int pointNum;//记录一下keyPoint的数量s
    
    //这个用的3d点是 其实搞错了是相机坐标系的点
    bool solveRelativePoseByPnP(std::vector<cv::Point2f> &measurements_old, Matrix3f &R_relative, Vector3f &T_relative,vector<cv::Point3f> &pts_3_vector,bool isAdd_otherClientId);//计算多个机器人相对位姿
    //这个用的3d点是世界坐标系的点
    bool solveRelativePoseByPnP_2(std::vector<cv::Point2f> &measurements_old, Matrix3f &R_relative, Vector3f &T_relative,vector<cv::Point3f> &pts_3_vector,bool isAdd_otherClientId);
    //这个和上面的一样，只是传入的参数换成double
    bool solveRelativePoseByPnP_3(std::vector<cv::Point2f> measurements_cur_norm, Matrix3d &R_relative, Vector3d &T_relative,vector<Vector3d> point_3d_cur_real,bool isAdd_otherClientId);
    //保存和其它地图的匹配上的 ClientId global_index relative_pose
    vector<int> clientId_server;
    vector<int> global_index_server;
    vector<Matrix3f> relative_r_server;
    vector<Vector3f> relative_t_server;
    
    //为了保存相机内参
    Client *c;
    //保存偏移量 多个地图融合的时候用
    Vector3d t_drift;//这里保存的是地图内部的偏移量 一直会更新
    Matrix3d r_drift;
    Eigen::Vector3d T_wOld_i;//地图内部最开始的rt 偏移量和3d点是一致的
    Eigen::Matrix3d R_wOld_i;//
    void updatePose_old(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
    void getPose_old(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
    //保存多地图融合后的偏移量
    Eigen::Vector3d t_w_w1_drift;
    Eigen::Matrix3d r_w_w1_drift;
    
    //要对keypoints去畸变
    bool isUndistorted_kf=false;//false 表示要去畸变 需要判断 这个数据本身是不是需要做去畸变
    bool isAllKeypoint =false;//当全部特征点发送过来 变成true
    
    
    bool isInImage(float x, float y);
    vector<int> GetFeaturesInArea(const float &x, const float &y, const float &r);//找的是measurements 到时候用的是世界坐标系下的点
    vector<int> GetFeaturesInArea_1(const float &x, const float &y, const float &r);//找到是keyPoints 这个到时候提供keypoints 图像坐标系的点
    
    BowVector m_bowvec;
    FeatureVector m_featvec;
    
    
    std::mutex mMutexConnections;
    //这个服务器并没有存了，因为发送这个数据量多
//    std::map<KeyFrame*,int> mConnectedKeyFrameWeights; ///< 与该关键帧连接的关键帧与权重
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames; ///< 排序后的关键帧
    std::vector<int> mvOrderedWeights; ///< 排序后的权重(从大到小)
    //添加两帧之间 新的观测关系
    void AddConnection_weight(KeyFrame *pKF, const int &weight);
    //更新两帧之间的权重
    void UpdateBestCovisibles();
    //存储mvpOrderedConnectedKeyFrames对应的id 临时存储，后面会清空
    std::vector<int> mvOrderedConnectedKeyFrames_global_index;
    //返回共视程度最高的N帧
    vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    vector<KeyFrame*> GetBestCovisibilityKeyFrames_old(const int &N);
    
    
//    int isuse;
    //存一下和其他地图 粗糙的相对位姿关系，一般都是3D点存储
    vector<int> clientId;
    vector<int> kfId;
    vector<std::pair<Matrix3d , Vector3d >  > relativePose;
    vector<Eigen::Matrix<double, 8, 1 > > relative_global_loop_info_multiClient;
    ReadWriteLock readWriteLock_loop;//写者优先
    
    double vio_length=0.0;//统计到目前为止走了多少里程计
    int edge=0;//前端滑动窗口约束总量
    int edge_single=0;//该帧在滑动窗口中的约束量
    
    int loop_edge=0;//和别人回环 初始的残差数量 （这个决定了找到更多点的可能性）
    double loop_error=0.0;//和别人回环 初始残差值 后面或许需要求个平均
    int loop_edge_final=0;//和别人回环 最终的残差数量
    double loop_error_final=0.0;//和别人回环 最终残差值
    double var_imu=0.0;//滑窗内 imu激励
    Eigen::Vector3d rela_t=Eigen::Vector3d::Zero();
    Eigen::Vector2d rela_angle=Eigen::Vector2d::Zero();//pitch roll
};

#endif /* KeyFrame_hpp */
