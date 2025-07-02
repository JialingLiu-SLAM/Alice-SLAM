//
//  HandleData.hpp
//  VINS_ios
//
//  Created by 张剑华 on 2020/5/27.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef HandleData_hpp
#define HandleData_hpp

#include <stdio.h>
//#include "initial_aligment.hpp"
#include "keyframe.h"
#include "global_param.hpp"
#include <iostream>
#include "keyfame_database.h"
#include <unistd.h>



struct PackHead//26
{
    char PackName[10];//1
    unsigned long PackIdx;//8
    size_t PackLen;//8
};
extern unsigned char saveEnd[5];

extern void produceStreamForCameraImuParam(stringstream &outputStream,DeviceType deviceType);
//发送固定参数 相机内参K、4个畸变参数（这个不知道是哪个）、相机和imu之间的外参、只发送一次
extern void  sendCameraParam(DeviceType deviceType);


extern void produceStreamForInit_aligmentParam(stringstream &outputStream,Eigen::Vector3d g);
//暂时只发送重力加速度，陀螺仪偏差 不确定回环检测要不要用？？
extern void sendInit_aligmentParam(Eigen::Vector3d g );

//发送一个帧的2D特征点、3D特征点、描述符信息，世界坐标系位姿
extern void produceStreamForKF(stringstream &outputStream,KeyFrame *imgKeyFrame);
extern std::vector<cv::KeyPoint>::iterator produceStreamForKF_1_1(stringstream &outputStream,KeyFrame *imgKeyFrame,int &can_send_keys_num);//
extern std::vector<cv::KeyPoint>::iterator produceStreamForKF_1_2(stringstream &outputStream,KeyFrame *imgKeyFrame,std::vector<cv::KeyPoint>::iterator iter,int des_num,bool is_end);
extern std::vector<BRIEF::bitset>::iterator produceStreamForKF_2(stringstream &outputStream,KeyFrame *imgKeyFrame,int des_num,bool is_end,std::vector<BRIEF::bitset>::iterator iter_start);
extern void sendKeyFrame(KeyFrame *imgKeyFrame);


//发送一个帧的2D特征点、3D特征点、描述符信息，世界坐标系位姿 压缩了描述符
extern std::vector<cv::KeyPoint>::iterator produceStreamForKF_1_1_compress(stringstream &outputStream,KeyFrame *imgKeyFrame,int &can_send_keys_num);//
extern std::vector<cv::KeyPoint>::iterator produceStreamForKF_1_2_compress(stringstream &outputStream,KeyFrame *imgKeyFrame,std::vector<cv::KeyPoint>::iterator iter,int des_num,bool is_end);
extern void produceStreamForKF_2_compress(stringstream &outputStream,int des_sub,bool is_end,int desIndex_num,KeyFrame *imgKeyFrame ,vector<uchar> &bitstream);
extern void produceStreamForKF_2_1_BowIndexcompress(stringstream &outputStream,int des_num,bool is_end,int desIndex_num,vector<int> &visualWords,KeyFrame *imgKeyFrame);
extern void sendKeyFrame_compress(KeyFrame *imgKeyFrame);


//发送一个帧的2D特征点、3D特征点、nodeId，世界坐标系位姿
extern std::vector<cv::KeyPoint>::iterator produceStreamForKF_1_1_words(stringstream &outputStream,KeyFrame *imgKeyFrame,int &can_send_keys_num);//
extern std::vector<cv::KeyPoint>::iterator produceStreamForKF_1_2_words(stringstream &outputStream,KeyFrame *imgKeyFrame,std::vector<cv::KeyPoint>::iterator iter,int des_num,bool is_end);
extern void produceStreamForKF_2_1_BowIndexcompress(stringstream &outputStream,int des_num,bool is_end,int desIndex_num,int visualWords,int global_index);
extern std::vector<BRIEF::bitset>::iterator produceStreamForKF_2_words(stringstream &outputStream,KeyFrame *imgKeyFrame,int des_num,bool is_end,std::vector<BRIEF::bitset>::iterator iter_start);
extern void sendKeyFrame_words(KeyFrame *imgKeyFrame);

//第一次发送关键帧，初始算出来的位姿
extern void produceStreamForKF_Pose(stringstream &outputStream,KeyFrame *imgKeyFrame);//
extern void sendKeyFrame_Pose(KeyFrame *imgKeyFrame);

//发送窗口优化完的位姿
extern void produceStreamForKFOriginPose(stringstream &outputStream,KeyFrame *imgKeyFrame);//
extern void sendKeyFrameOriginPose(KeyFrame *imgKeyFrame);

//发送错误的回环检测
extern void produceStreamForErrorLoop(stringstream &outputStream,int curKF_Loop_Index);//
extern void sendErrorLoop(int curKF_Loop_Index);

//发送偏差更新的下标
extern void produceStreamForCorrectLoopIndex(stringstream &outputStream,int curKF_CorrectLoop_Index);//
extern void sendCorrectLoopIndex(int curKF_CorrectLoop_Index);

//发送front_pose的相对位姿变换
extern void produceStreamForFrontPoseRelative(stringstream &outputStream,Eigen::Vector3d relative_t,double relative_yaw,int relative_cur_index);
extern void sendFrontPoseRelative(Eigen::Vector3d relative_t,double relative_yaw,int relative_cur_index);

//这里是多加了两个front_pose数据 relative_pitch relative_roll
extern void produceStreamForFrontPoseRelative_add_pitch_roll(stringstream &outputStream,Eigen::Vector3d relative_t, double relative_yaw,int relative_cur_index,double relative_pitch,double relative_roll);
extern void sendFrontPoseRelative_add_pitch_roll(Eigen::Vector3d relative_t, double relative_yaw,int relative_cur_index,double relative_pitch,double relative_roll);

//这个是需要降低权重的
extern void produceStreamForFrontPoseRelative_add_pitch_roll_forRemove(stringstream &outputStream,Eigen::Vector3d relative_t,double relative_yaw,int relative_cur_index,double relative_pitch,double relative_roll);
extern void sendFrontPoseRelative_add_pitch_roll_forRemove(Eigen::Vector3d relative_t,double relative_yaw,int relative_cur_index,double relative_pitch,double relative_roll);

//发送AR信息  暂时还没改
extern void produceStreamForAR(stringstream &outputStream,Eigen::Vector3f ori, Eigen::Vector3f cox, Eigen::Vector3f coy, Eigen::Vector3f coz, float size);
extern void sendAR(Eigen::Vector3f ori, Eigen::Vector3f cox, Eigen::Vector3f coy, Eigen::Vector3f coz, float size);

extern void produceStreamForOurAR(stringstream &outputStream,Eigen::Vector3f ori, Eigen::Vector3f cox, Eigen::Vector3f coy, Eigen::Vector3f coz, float size);
extern void sendOurAR(Eigen::Vector3f ori, Eigen::Vector3f cox, Eigen::Vector3f coy, Eigen::Vector3f coz, float size);


//接收数据
//全局优化
extern void receiveGlobalData(const char* buffer,long unsigned int offset,KeyFrameDatabase *kfbd);
//回环检测
extern void receiveLoopData(const char* buffer,long unsigned int offset,VINS *vins);
extern void receiveLoopData_another(const char* buffer,long unsigned int offset,VINS *vins);
extern void receiveLoopData_another2(const char* buffer,long unsigned int offset,VINS *vins,double (&loop_pose_forFeatureTracker)[7]);
//拒绝外点
extern void receiveRejectWithF(const char* buffer,long unsigned int offset,VINS *vins);
//接收AR
extern void receiveAr(const char* buffer,long unsigned int offset,VINS *vins);
//接收多个地图优化数据
extern void receiveGlobalData_multiClient(const char* buffer,long unsigned int offset,KeyFrameDatabase *kfbd);

    
  
#endif /* HandleData_hpp */
