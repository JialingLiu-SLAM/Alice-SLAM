//
//  HandleData.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/27.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef HandleData_hpp
#define HandleData_hpp

#include <stdio.h>
#include "KeyFrame.hpp"
#include "MyThread.hpp"
#include "PoseGraph.hpp"
#include "PoseGraphGlobal.hpp"
#include "Client.hpp"

struct PackHead
{
    char PackName[10];
    unsigned long PackIdx;
    size_t PackLen;
};
extern unsigned char saveEnd[5];


//固定参数 相机内参K、4个畸变参数、相机和imu的外参 ，目前只发送了设备名称
 extern DeviceType receiveCameraParam(const char* buffer);
//暂时只接收g
extern Vector3d receiveInit_aligmentParam(const char* buffer);
    //一个帧的2D特征点、3D特征点、描述符，位姿
extern KeyFrame* receiveKF(const char* buffer);
extern KeyFrame* receiveKF_1_1(const char* buffer);
extern void receiveKF_1_2(const char* buffer,PoseGraph *poseGraph);
//剩余描述符
extern void receiveKF_add(const char* buffer,PoseGraph *poseGraph);
//压缩剩余描述符 对应参考的下标
extern void receiveKF_add_compressIndex(const char* buffer,PoseGraph *poseGraph,int packLen);
//压缩剩余描述符
extern void receiveKF_add_compress(const char* buffer,PoseGraph *poseGraph,Client *c,int packLen);
//origin_pose
extern KeyFrame* receiveKF_OriginPose(const char* buffer,PoseGraph *poseGraph);
//错误的回环匹配
extern void receiveErrorLoop(const char* buffer,PoseGraph *poseGraph);
//偏移纠正回环到的下标
extern void receiveCorrectLoop(const char* buffer,PoseGraph *poseGraph);
//发送front_pose的relative
extern void receiveFrontPoseRelative(const char* buffer,PoseGraph *poseGraph);
//发送front_pose的relative 增加了pitch和roll
extern void receiveFrontPoseRelative_add_pitch_roll(const char* buffer,PoseGraph *poseGraph);
//这个是要降低权重的 防止过拟合
extern void receiveFrontPoseRelative_add_pitch_roll_forRemove(const char* buffer,PoseGraph *poseGraph);
//接收到了AR
extern int receiveAr(const char* buffer,PoseGraphGlobal *poseGraphGlobal, int clientId);



extern void produceStreamForGloablLoopData(stringstream &outputStream,PoseGraph *poseGraph);
extern char* sendGlobalLoopData(int &len_end,PoseGraph *poseGraph);

extern void produceStreamForLoopData(stringstream &outputStream,VINS* vins);
extern char* sendLoopData(int &len_end,VINS* vins);

extern void produceStreamForLoopData_another(stringstream &outputStream,VINS* vins);
extern char* sendLoopData_another(int &len_end,VINS* vins);

extern void produceStreamForLoopData_another2(stringstream &outputStream,VINS* vins);
extern char* sendLoopData_another2(int &len_end,VINS* vins);

//剔除外点
extern void produceStreamForRejectWithF(stringstream &outputStream,VINS* vins);
extern char* sendRejectWithF(int &len_end,VINS* vins);

//发送AR给其它client
extern void produceStreamForAr(stringstream &outputStream,PoseGraphGlobal *poseGraphGlobal, int ground_ar_idx, int clientId);
extern char* sendArData(int &len_end,PoseGraphGlobal *poseGraphGlobal, int ground_ar_idx, int clientId);


extern void produceStreamForGloablLoopData_multiClient(stringstream &outputStream,PoseGraph *poseGraph);
extern char* sendGlobalLoopData_multiClient(int &len_end,PoseGraph *poseGraph);




#endif /* HandleData_hpp */
