//
//  PoseGraphGlobal.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/6/16.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef PoseGraphGlobal_hpp
#define PoseGraphGlobal_hpp
#include "PoseGraph.hpp"
#include "demoDetector_server.h"
#include <stdio.h>
#include "utility.hpp"
//#include "HandleData.hpp"
#include <math.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "FeatureManager_server.h"
#include "projection_factor_merge.h"
#include "../factor/projection_facor.hpp"
#include "pose_local_parameterization.hpp"
#include <algorithm>
#include <time.h>
#include <random>
#include "OptimizeRelativePose.hpp"
#include "ReadWriteLock.hpp"
#include "UnionFind.h"
#include <iomanip>
#include "FeatureSubMap.hpp"
using namespace std;

struct GroundPoint
{
    int idx;
    Vector3f center;
    bool boxflag;
    bool moveflag;
    Vector3f ori, cox, coy, coz;
    Vector3f lix, liy, liz;
    float size;
    Vector4f initPlane;
    int clientId;//记录是哪个用户创建的ar
    vector<int> isSendAr;//记录该物体发给哪些客户端了 为1，表示发过了
    
    GroundPoint(int idx_, Vector3f center_)
    {
        idx = idx_;
        center = center_;
        boxflag = false;
        moveflag = false;
    }
    
    GroundPoint(int idx_, int clientId_)
    {
        idx = idx_;
        clientId=clientId_;
        boxflag = false;
        moveflag = false;
        
        isSendAr.resize(10);
        for(int i=0;i<10;i++){
            isSendAr[i]=0;
            
        }
    }
};

class PoseGraphGlobal
{
public:
    PoseGraphGlobal(const char *_voc_filei, int _image_w, int _image_h);
    //添加PoseGraph
    void addPoseGraph(int clientID,PoseGraph *poseGraph);
    PoseGraph* getMainPoseGraph();
    //回环线程
    bool startLoopClosure_2(std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors,
    std::vector<cv::Point2f> &cur_pts, std::vector<cv::Point2f> &old_pts, int &old_index,int &min_startDetect_id, int cur_kf_global_index, int clientId_main, demoDetector_server<BriefVocabulary, BriefLoopDetector_server, FBrief::TDescriptor> &demo_test, int clientId);
    
    bool startLoopClosure_3(int keyPoint_num_main,std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors,
                                       std::vector<cv::Point2f> &cur_pts,
                                       std::vector<cv::Point2f> &old_pts,
                                             int &old_index,int &min_startDetect_id,int cur_kf_global_index,int clientId_main, demoDetector_server<BriefVocabulary, BriefLoopDetector_server, FBrief::TDescriptor> &demo_test, int clientId,std::vector<cv::Point2f> &cur_pts_3d, std::vector<cv::Point2f> &old_pts_2d);
   
    void eraseIndex(std::vector<int> &erase_index);
    void loopClosureRun_global_11_2_2(int clientId);
    void loopClosureRun_global_11_2(int clientId);
    void loopClosureRun_global_11_3_2(int clientId);
    void loopClosureRun_global_11_3(int clientId);
    
    void loopClosureRun_global_12_1(int clientId_main);
    void loopClosureRun_global_12_1_2(int clientId_other);
    
    void loopClosureRun_global_13_1(int clientId_main);
    void loopClosureRun_global_13_1_2(int clientId_other);
   
    int MergingEnd=0;//后面融合结束了 再++
    
    void MergeLocal_14();//论文实验最后一版
    void MergeLocal_15();//换了一种新的查找匹配点的方法
    void MergeLocal_16();//增加成双向重投影误差
    
    
    void loopClosureRun_global_14_1_2(int clientId);
    void loopClosureRun_global_14_1(int clientId);
    void MergeLocal_17();//论文2实验第一版
    
    

    void loopClosureRun_global_15_all(int clientId);
    void loopClosureRun_global_16_all(int clientId);//15有大量误匹配
    void loopClosureRun_global_17_all(int clientId);//将共视帧利用上
    bool findMoreMatch(Matrix3d &R_relative,Vector3d &T_relative,KeyFrame *iter_kf_main,KeyFrame *cur_kf);
    void MergeLocal_18();//论文3实验第一版
    void realMergeLocal_18(KeyFrame* kf_3d, KeyFrame* kf_2d, PoseGraph* pg_3d, PoseGraph* pg_2d,int noMain);
    void realMergeLocal_19(KeyFrame* kf_3d, KeyFrame* kf_2d, PoseGraph* pg_3d, PoseGraph* pg_2d,int noMain);//意图提高速度 通过合并相邻回环帧的局部窗口
    void realMergeLocal_20(KeyFrame* kf_3d, KeyFrame* kf_2d, PoseGraph* pg_3d, PoseGraph* pg_2d,int noMain);//使用gcransac
    void GlobalFuse_9(int i);//没有尖锐了
    void GlobalFuse_10(int i);//9可能会有一些数据错误
    void GlobalFuse_11(int i);//告诉先验相对位姿 都优化
    void GlobalFuse_12(int i);//把相对位姿 当作一个优化变量
    void GlobalFuse_13(int i);//基于10的基础 用子地图做优化
    void GlobalFuse_14(int i);//基于9的基础 将现有位姿当先验知识
    FeatureMap* global_featureMap;
    
    /* data */
    demoDetector_server<BriefVocabulary, BriefLoopDetector_server, FBrief::TDescriptor> demo_global;
    int IMAGE_W;
    int IMAGE_H;
    //融合线程
    void optimize4DoFLoopPoseGraph(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r);
    //存储两个地图之间 点的匹配情况
  
    void Fuse_4_1( Matrix3f R_relative_best,Vector3f T_relative_best,  list<KeyFrame*> spLocalWindowKFs, list<KeyFrame*> spFixedKFs, list<KeyFrame*> spMergeConnectedKFs, float th);
    void Fuse_4_2( Matrix3f R_relative_best,Vector3f T_relative_best,  list<KeyFrame*> spLocalWindowKFs, list<KeyFrame*> spFixedKFs, list<KeyFrame*> spMergeConnectedKFs, float th);
    void Fuse_4_1_2( Matrix3f R_relative_best,Vector3f T_relative_best,  list<KeyFrame*> spLocalWindowKFs, list<KeyFrame*> spMergeConnectedKFs, float th);
    void Fuse_4_2_2( Matrix3f R_relative_best,Vector3f T_relative_best,  list<KeyFrame*> spLocalWindowKFs, list<KeyFrame*> spMergeConnectedKFs, float th);
    
    void Fuse_5_2_2( Matrix3f R_relative_best,Vector3f T_relative_best, list<KeyFrame*> spLocalWindowKFs,   KeyFrame* pMergeKF_main, float th);
    
    void Fuse_6_2_2( Matrix3f R_relative_best,Vector3f T_relative_best, list<KeyFrame*> spLocalWindowKFs,  list<KeyFrame*> spMergeConnectedKFs, KeyFrame* pKF_main_matched, float th);
    
    //全局优化
    void GlobalFuse_4();
    void GlobalFuse_6(int i);//这个是实验的最后版本
    void GlobalFuse_7(int i);
    void GlobalFuse_8(int i);//基于6的基础改的
    

    FeatureManager_server f_manager_server_main;
    void setIMUModel(double FOCUS_LENGTH_X_server);
    
    //存放了所有的地图 和所属的clientId
    map<int,PoseGraph*> PoseGraphGloabl_map;//clientId
//    std::mutex poseGraphGlobal_mutex;
    ReadWriteLock readWriteLock;//写者优先
    int pushDatabase_num=0;
    vector<int> mv_pushDatabase_num;
    
    int isFirstFusion[10]={0};//其实是一开始没给值 只有发生一次回环才会加入，0表示没有回环，1表示第一次回环，2表示多次回环，3表示待进行全局优化 数组下标表示的ClientId和主地图发生回环 4表示要进行局部窗口优化,但是在此之前还没有过回环，5表示要进行局部窗口优化，但是在此之前已经有过回环,6表示要进行局部优化，但是之前还有一个全局优化还没做
    //新里面重新定义了含义，0表示没有回环优化要做，检测到回环+1，要进行局部窗口优化>0，局部窗口优化完-1，
    //判断是否两个地图是否融合成功过
    bool isFusion[10]={0};//1表示融合成功了 最终是用来发送ar的 后面要改成，根据每个地图和哪几个地图融合成功了，而不是仅仅主地图
//    vector<demoDetector_server<BriefVocabulary, BriefLoopDetector_server, FBrief::TDescriptor> > mv_demo_global;
    
    //存放所有的ar物体
    vector<GroundPoint> grounds;
    int Ground_idx;
    
//    void setFeatureMap(FeatureMap* global_featureMap);
    
private:
    
    int TH_LOW;
    
//    static const int client_num=10;
    //这个也还没给值
    int max_frame_num_global;//最多多少帧参与 多地图的全局优化 这里表示的是两个地图，所以后面还要再分配一下 每个地图占多少
    //多个地图融合的全局优化 还是要存一下的 以防短时间内多个地图发生融合 后期可以设置 最大个数，或者如果两个地图要两次全局融合 那也可以不用记录
    
    //这个得到的是第一次融合
    //int：表示其他地图的ClientId
    //pair:先默认第一个放的是主地图 后一个放的是其它地图
    map<int,std::pair<int,int> > earliest_globalFuse_index;
    //最新的回环的帧 下标就是ClientId ,存的是主地图匹配的帧 其他地图匹配的帧
    vector<std::pair<int,int> > latest_globalFuse_index;//这里到时候 要加一下定义大小 不用push_back 直接用[]
    
    
    //下标表示ClientId 这里存储的是真正的两个地图之间的相对位姿
    vector<Matrix3d> r_w2_w1_allClient;
    vector<Vector3d> t_w2_w1_allClient;
    //这里再存储一个 当时两个地图之间的相对位姿 为了算两帧之间的相对front_pose
    vector<Matrix3d> r_w2_w1_allClient_cur;
    vector<Vector3d> t_w2_w1_allClient_cur;
    
    //多个地图之间的相对位姿信息记录
    vector<vector<int> > feature_id_cur;
    vector<vector<cv::Point2f> > measurements_norm_main;
    vector<std::pair<int,int> > cur_globalFuse_index;//这里存的是主地图 其它地图 当前发生融合 要进行局部窗口优化的帧的id，下标是ClientId
    vector<std::queue<std::pair<int,int> > > q_cur_globalFuse_index;//相对前面改一改，因为回环容易发生在一块区域，所以需要vector
    vector<bool> isOpposite;//为true,表示3D-2D的约束关系反的,即约束关系并不是存在主地图的帧里面
    vector<int> earliest_global_loop_index;//这里存的是主地图与当前地图发生融合最早的帧 存的是主地图的下标
    vector<std::pair<int,int> > latest_global_loop_index;//这里存的是最大的 发生融合的下标 先是主地图，后是其它地图
    
//    20220401
    vector<int > latest_globalLoop_index;//这里存的是最大的 发生融合的下标
    vector<int> earliest_globalLoop_index;//这里存的是最小的 发生融合的下标
    UnionFind uf;//并查集 判断这几个地图有没有融合成一个地图 在全局优化的时候用
    
    vector<vector<int> > feature_id_main;
    vector<vector<cv::Point2f> > measurements_norm_cur;//图像坐标
    
    vector<bool> has_pre_loop_index;
    vector<std::pair<int,int> > pre_loop_index;//下标是clientId 第一个int是的主地图kf id, 第二个int是当前地图的kf id
    
    
    //第2次实验 存的是w2世界坐标系 到 a是相机坐标系
    vector<Matrix3f> r_ca_w2_cur;
    vector<Vector3f> t_ca_w2_cur;
    
//    vector<Matrix3f> r_cb_w1_cur;
//    vector<Vector3f> t_cb_w1_cur;
    
    
    void realMergeLocal_17(KeyFrame* kf_3d, KeyFrame* kf_2d, PoseGraph* pg_3d, PoseGraph* pg_2d,int noMain);
    
    //    子地图用
//    FeatureMap* global_featureMap;
};
#endif /* PoseGraphGlobal_hpp */
