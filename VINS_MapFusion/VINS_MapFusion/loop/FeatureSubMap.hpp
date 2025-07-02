//
//  FeatureSubMap.h
//  dbow_test
//
//  Created by 张剑华 on 2020/8/15.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef FeatureSubMap_hpp
#define FeatureSubMap_hpp

#include "KeyFrame.hpp"

#include "utility.hpp"
#include "ReadWriteLock.hpp"

using namespace Eigen;


class FeatureSubMap{
public:
    FeatureSubMap();
    void perfectFuse_subMap(FeatureSubMap * curSubMap);
//    传进来的是oldSubMap
    void perfectFuse_subMap2(FeatureSubMap * curSubMap);
    bool isBigView(int threshold_bigView);
    bool isBigView_noMutex(int threshold_bigView);
    list<KeyFrame*> ml_keyframe;//一层的kf
    list<KeyFrame*> ml2_kf;//二层的kf
    vector<int> ml_kfId;//这个是加入到loop closure里面的顺序
    vector<int> ml2_kfId;//这个是加入到loop closure里面的顺序
    
    int loop_mltra_num;
    
    vector<int> noRepresent_kfId;//这个是全局下的kf globalIndex
//    临时记录 实验用
    vector<double> noRepresent_kfHeader;//这个是全局下的kf globalIndex
    
    vector<int> noRepresent_kfId_l2;//这个是全局下的kf globalIndex
//    临时记录 实验用
    vector<double> noRepresent_kfHeader_l2;//这个是全局下的kf globalIndex
    
//    对点先不做任何处理
//    vector<FeaturePerId> feature;//存储每一个特征点，及其对应的每帧的属性
//    vector<cv::KeyPoint> keyPoint;
//    vector<BRIEF::bitset> descriptor;
//    vector<int> featureID;
//    Matrix3d ric;//因为我们得到的是r_w_i 到时候放到初始化函数里面初始化
    
    
    int global_index;//这个是记录当前的gloal index
    //    加一个机器编号
        int global_agent_id;
    
    //记录多个global_index  可以用来查找空间上相邻关系
    vector<int> global_index_moreEarly;//老的按顺序存放在这个里面 有序 说明
    vector<int> global_agent_id_moreEarly;
    
    KeyFrame* center_kf;
    
    
    vector<int> relatedSubMapIndex;//记录空间上的强关联关系
    std::mutex mMutexSubMapList;
    
    vector<int> related_otherAgentSubMapIndex;
    vector<int> related_otherAgentId;
    
//    void replaceSelf(FeatureSubMap *oldSubMap);
    void operator=(const FeatureSubMap& old);
};

//class RelatedSubMap{
//public:
//    RelatedSubMap();
////    第一个vector是存的每一个区域， 第二个vector存的是每个区域 有哪些subMap
//    vector<vector<int> > relatedSubmap_id;//最后一个为基准
//};

class FeatureMap{
public:
    FeatureMap();
    int getIndex_fromAgentId(int curAgent_id);
    FeatureSubMap* getSubMapByIndex(int index,int curAgent_id);
    void allocateSubMap(KeyFrame* kf_cur,int old_index,int kfNum_tree,vector<int> old_vector);
    
    void allocateSubMap2(KeyFrame* kf_cur,int old_index,int kfNum_tree,vector<int> old_vector);
    void allocateSubMap_multi(KeyFrame* kf_cur,KeyFrame* kf_old);
    void allocateSubMap_multi2(KeyFrame* kf_cur,KeyFrame* kf_old);
//private:
//    TODO 换成智能指针https://stackoverflow.com/questions/16465633/how-can-i-use-something-like-stdvectorstdmutex
//    ReadWriteLock readWriteLock_ml_submap;//防止ml_submap在增长时遍历
//    const int agent_num=5;
    vector<shared_ptr<std::mutex> > mMutex_ml_submap;
    //TODO还应该加一个vector锁 防止ml_submap在增长
    vector< list<FeatureSubMap*> > ml_submap;//这里也许应该用vector 方便当前地图被融合时，当前地图指向老地图
//    机器编号
    ReadWriteLock readWriteLock_agent_id;
    vector<int> agent_id;

    

//    即便子地图融合了 也没有改变对应的所属子地图id
    vector<vector<int> > kfAttr_ForAMap;//表示是代表帧 或 非代表帧 1为l1 2为l2 0为非代表帧-l1层的 3为非代表帧-l2层的  后面也许要考虑非l1层和l2层的非代表帧
    vector<vector<int> > kfAttr_SubMapId;

    vector< map<int,int> > treeId_kf;//前一个是kfId, 后一个是treeId

//    vector<RelatedSubMap*> relatedSubMap_forAgent;
    vector<vector<int> > relatedSubMap_forAgent;
    
    vector<int> delteSubMap_num;

};


#endif /* FeatureSubMap_h */
