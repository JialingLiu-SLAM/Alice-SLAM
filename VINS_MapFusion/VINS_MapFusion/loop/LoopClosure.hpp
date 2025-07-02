//
//  LoopClosure.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/27.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef LoopClosure_hpp
#define LoopClosure_hpp

// DLoopDetector and DBoW2
#include "DBoW2.h" // defines BriefVocabulary
#include "DLoopDetector.h" // defines BriefLoopDetector
#include "DVision.h" // Brief
#include <stdio.h>
#include "PoseGraph.hpp"
#include "demoDetector.hpp"
#include "VINS.hpp"
#include <map>
#include <time.h>

//#include <stdlib.h>

//#include <algorithm>
//#include <iostream>
#include <random>
#include "OptimizeRelativePose.hpp"

#include "FeatureSubMap.hpp"

using namespace DLoopDetector;
using namespace DBoW2;
using namespace DVision;
using namespace std;

//#include "GCRANSAC.h"
//#include "types.h"
//#include "empty_inlier_selector.h"
//#include "preemption_sprt.h"
//#include "uniform_sampler.h"
//#include "prosac_sampler.h"
//#include "model.h"
//#include "statistics.h"
//#include "flann_neighborhood_graph.h"
//using namespace gcransac;



class LoopClosure
{
public:
    LoopClosure(const char *voc_file, int _image_w, int _image_h);

    bool startLoopClosure(std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors,
                          std::vector<cv::Point2f> &cur_pts,
                          std::vector<cv::Point2f> &old_pts,
                          int &old_index);
    bool startLoopClosure_2(std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors, std::vector<cv::Point2f> &cur_pts, std::vector<cv::Point2f> &old_pts, int &old_index, BowVector &m_bowvec, FeatureVector &m_featvec);
    bool startLoopClosure_3(KeyFrame* cur_kf, std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors,
                            std::vector<cv::Point2f> &cur_pts,
                            std::vector<cv::Point2f> &old_pts,
                              KeyFrame* *old_kf,BowVector &m_bowvec, FeatureVector &m_featvec);
    bool startLoopClosure_4(KeyFrame* cur_kf);
    bool startLoopClosure_5(KeyFrame* cur_kf);
    bool startLoopClosure_6(KeyFrame* cur_kf);
    bool startLoopClosure_7(KeyFrame* cur_kf);
    bool startLoopClosure_8(KeyFrame* cur_kf);
    bool startLoopClosure_9(KeyFrame* cur_kf);//对应loopClosureRun_6   目前正在用
    bool startLoopClosure_10(KeyFrame* cur_kf);//对应loopClosureRun_7
    bool startLoopClosure_11(KeyFrame* cur_kf);//9的基础上改，先用回原来的检测回环的方法 DBOW中
    void eraseIndex(std::vector<int> &erase_index);
    
    //ljl
    void loopClosureRun();
    void loopClosureRun_2();//论文最后版本
    void loopClosureRun_3();//时间一致性 改成 共视帧的一致性 论文    目前正在用的
    void loopClosureRun_4();//这个接力用 因为3一个人来不及算
    void loopClosureRun_5();//把3的部分 再挪一点过来
    void loopClosureRun_6();//5算了两遍，可能太多了
    void loopClosureRun_7();//6用的新帧到老帧的投影，可能新帧会发生变化，结果会不稳定
    void loopClosureRun_8();//基于6的基础 改动
    void loopClosureRun_9();//基于8的基础 加上ceres过滤 论文       目前正在用的 接力的
    void loopClosureRun_10();//基于9的基础，把下面的优化 也加一个过滤
    void setPoseGraph(PoseGraph* poseGraph);
    void setVINS(VINS* vins);
    
    bool startLoopClosure_12(KeyFrame* cur_kf);//基于9的基础 用老帧3d点
    void loopClosureRun_12();//基于9的基础 用老帧3d点
    
     
//    子地图用的 保持和startLoopClosure_9一致的格式，到时候直接在loop_closureRun_3中调用
    bool startLoopClosure_subMap(KeyFrame* cur_kf);
    void setFeatureMap(FeatureMap* global_featureMap);
    
    
    
    /* data */
    demoDetector<BriefVocabulary, BriefLoopDetector, FBrief::TDescriptor> demo;
    int IMAGE_W;
    int IMAGE_H;
    
    map<int,int> treeId_kf;//前一个是treeId , 后一个是kfId
    
    int preLoopKfId;//用于记录前一次通过增强几何一致性验证的帧Id，后面等待5帧再进行
private:
//    子地图用
    FeatureMap* global_featureMap;
    
    PoseGraph* poseGraph;
    VINS* vins;
    
    // 回环帧下标
    int old_index = -1;
    // Textview for showing vins status
    int loop_old_index = -1;
    
//    int last_kf;//上一个参与检测的id
//    int real_kf_inTree;//因为会出现跳帧的问题（没及时参与到回环检测）,所以记录缺少参与的数量
    int kfNum_tree;//树里面帧的数量
   
    
    //这个是临时构造的滑动窗口
    list<FeaturePerId> feature;//存储每一个特征点，及其对应的每帧的属性
    
    
    //两个线程接力
    list<vector<vector<cv::Point2f> > > mlvv_measurements_old_norm_all;
    list<vector<vector<Eigen::Vector3d>> > mlvv_point_clouds_all;
    
    //实验用一下---------------
//    list<vector<cv::Point2f> > mlvv_measurements_old_single;
//    list<vector<cv::Point2f> > mlvv_measurements_cur_single;
//    list<vector<int> > mlvv_featureId_old_single;
//    list<vector<int> > mlvv_featureId_cur_single;
    //----------------------
    
    
    list<vector<cv::Point2f> > mlvv_measurements_old_norm_single;
    list<vector<Eigen::Vector3d> > mlvv_point_clouds_single;
//    list<vector<KeyFrame*> > real_vpCovKFi_cur;
    list<vector<Matrix3d> > mlv_oldR_b_a;
    list<vector<Vector3d> > mlv_oldT_b_a;
    list<Matrix3d> ml_relative_r;
    list<Vector3d> ml_relative_t;
    list<vector<int> > mlv_vpkf_index;
    list<std::pair<KeyFrame* , KeyFrame*> > mlp_kf_pairs;
    std::mutex relative_mutex;
    
//    list<vector<int> > old_vector_forSubMap;
};

#endif /* LoopClosure_hpp */
