//
//  FeatureManager_server.h
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/9/7.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef FeatureManager_server_h
#define FeatureManager_server_h

#include "utility.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <list>
using namespace Eigen;
using namespace std;

class FeaturePerFrame_server//一个特征点的属性 空间特征点映射到帧上对应图像坐标、特征点跟踪速度、空间坐标
{
public:
    FeaturePerFrame_server(const Vector3d &_point): point(_point)
    {
    }
    Vector3d point;//世界坐标
   
};

class FeaturePerId_server//管理一个特征点
{
public:
    const int feature_id;//特征点
    
    vector<FeaturePerFrame_server> feature_per_frame_other;//管理对应帧的属性
    vector<int> frameId_inClient_per_feature;//对应是哪一个帧看到的 
    vector<int> clientId;//对应那个帧 属于哪个地图
    
    vector<FeaturePerFrame_server> feature_per_frame_main;//管理对应帧的属性 主地图的点放这里
    int frameId_inClient_per_feature_main;//这个没有用
    
    
    
    
    int used_num;//出现的次数
    
    bool fixed;
   
    
    FeaturePerId_server(int _feature_id): feature_id(_feature_id),used_num(0),fixed(false)
    {
    }
    
    
};

class FeatureManager_server//管理所有特征点
{
public:
    FeatureManager_server(){
        
    }
    
    list<FeaturePerId_server> feature;//存储每一个特征点，及其对应的每帧的属性
    
//
//private:
//    Matrix3d ric;
    
};

#endif /* FeatureManager_server_h */
