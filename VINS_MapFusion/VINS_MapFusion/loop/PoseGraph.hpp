//
//  PoseGraph.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/29.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef PoseGraph_hpp
#define PoseGraph_hpp

#include <stdio.h>
#include <vector>
#include "KeyFrame.hpp"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <queue>
#include "VINS.hpp"


#include "ReadWriteLock.hpp"
#include "demoDetector_server.h"

//for save keyframe result
struct KEYFRAME_DATA
{
    double header;
    Vector3d translation;
    Quaterniond rotation;
};

class PoseGraph
{
public:
//    PoseGraph();
    PoseGraph(const char *_voc_file, int _image_w, int _image_h);
    void add(KeyFrame* pKF);
    void resample(vector<int> &erase_index);
    void erase(KeyFrame* pKF);
    int size();
    void optimize4DoFLoopPoseGraph(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r);

    void optimize4DoFLoopPoseGraph5(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r);//第2篇论文
    void optimize4DoFLoopPoseGraph6(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r);

    void optimize4DoFLoopPoseGraph7(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r);

    
    KeyFrame* getKeyframe(int index);
    KeyFrame* getKeyframe2(int index);
    list<KeyFrame*>::iterator getKeyframe_iter(int index);
    KeyFrame* getLastKeyframe();
    KeyFrame* getLastKeyframe(int last_index);
    KeyFrame* getLastKeyframe_index(int last_index);
    KeyFrame* getLastUncheckKeyframe();
    void updateVisualization();
    void addLoop(int loop_index);
    //ljl
    void globalLoopRun();
    void setVINS(VINS* vins);
    void viewPointClouds();
    void viewPath();
    void viewPath_2();
    void viewPath_3();

    
//    void setPoseGraphGlobal(PoseGraphGlobal* poseGraphGlobal);
    void setClient(Client* c);
    
    //这个好像用来画图的 第一个是原始的位姿，第2个是地图融合的位姿 加了两个地图的相对位姿进去
    vector<Vector3f> refine_path,refine_path_draw;
    vector<double> path_time;//实验用
//    vector<Quaterniond> refine_r;//实验用 这个没用了
//    vector<Vector3d> refine_t;//实验用
    vector<Matrix3f> refine_path_r,refine_path_r_draw;//用这个
    
    //记录在哪里有地图融合进来
//    vector<vector<int>> mainGraph_fusionStart_globalIndex;//主地图的匹配帧id 这个目前还没有用到
    //这里改成所有人的下标 都死ClientId
//    vector<int> fusion_otherClientId;//小地图的id 这里也不用了
    vector<PoseGraph*> fusion_otherGraph;//小地图的poseGraph 只记录发生融合了的
    //这里只有主地图的更新正确，其它地图只是第一次融合 更新到0了，方便后面判断到底有没有融合成功
    vector<int> fusion_relative_isUpdate;//记录画图更新时，更新到哪个下标了 也是已经更新了的长度 -2表示还没有过值，-1表示更新了一个新的位姿 整个地图需要重新更新一下
    vector<Matrix3f>  relative_r_mainToOther;//这里存的都是当时最好的best_pose
    vector<Vector3f> relative_t_mainToOther;
    vector<double> relative_score_min;//最低分数
    
    
    //这个记录计算相对位姿的锁
    std::mutex refine_path_mutex; //应该可以不加锁 每次画之前 都把他赋值给另一个人 还是要加锁啊
    //再给一个 refine_path_draw的锁
    std::mutex refine_path_draw_mutex;
    //这个是refine_path keyframeList的锁
    std::mutex mMutexkeyFrameList;
    //这个只给fusion_otherGraph加锁  然后得到了poseGraphOther，用它下面的元素给其它vector加锁
    std::mutex fusion_poseGraph_mutex;
    
    std::mutex keyFrameList_global_fusion_mutex;//避免地图内部的优化 和 多地图之间的优化同时进行
    //refine_path的锁 只是用来防止它在清空的时候，我刷新了画面
    std::mutex drawMutex;
    
    vector<KEYFRAME_DATA> all_keyframes;
    vector<int> segment_indexs;
    int max_seg_index, cur_seg_index;//序列号 意味着是第几个地图
    
    //ljl
    vector<Vector3d> get_t_global();
    vector<double> get_yaw_global();
    vector<Matrix3d> get_r_global();
//    Vector3d get_t_drift();
//    double get_yaw_drift();
//    Matrix3d get_r_drift();
    
    
    //画图用 存放当前帧的位姿
    Vector3d curKF_P;
    Matrix3d curKF_R;

    //测试用 后续未必要发送 这个有没有可能发生一个在添加，一个在删除
    queue<vector<int> > special_kf_inOpti;
    std::mutex special_kf_mutex;
    //for 地图内部
    queue<vector<int> > special_kf_inOpti_intra;
    std::mutex special_kf_mutex_intra;
    queue<int> loopKF_index;
    queue<int> curKF_loop_index;
    queue<int> earliest_queue;
    
    bool isSendGlobalData;
    queue<int> lastKF_index;//回环偏移更新到哪个关键帧了
    
    void updateCorrectLoopPose(int endIndex);
    string getTime();
    
    list<KeyFrame*> keyFrameList;//存放了所有关键帧
    std::mutex isFirstFusion_globalMutex;//给poseGraphGlobal 中 isFirstFusion用
    
    std::mutex isAddKF2Database_mutex;
    
    
    //实验用 判断 程序结束，不再接收数据了
    bool isEnd;
    
    double total_length;
    Vector3d t_drift;//初始化为0，
    double yaw_drift;
    Matrix3d r_drift;
    
//    PoseGraphGlobal mpPoseGraphGlobal;
//    给earliest_loop_index latest_loop_index加锁
    std::mutex loop_index_mutex;
    int earliest_loop_index;
    //未赋值 这里是最后发生回环的帧
    int latest_loop_index;
    
    //未赋值 赋值了 这里是判断做哪样的全局优化
    bool is_fusion;
    ReadWriteLock readWriteLock_is_fusion_mutex;//写者优先
    
    bool start_global_fuse_opti;
    std::mutex start_global_fuse_opti_mutex;
    
    // Translation drift
    Eigen::Vector3d loop_correct_t = Eigen::Vector3d(0, 0, 0);//回环之后计算得到的漂移

    // Rotation drift
    Eigen::Matrix3d loop_correct_r = Eigen::Matrix3d::Identity();
    VINS* vins;
    
    
    demoDetector_server<BriefVocabulary, BriefLoopDetector_server, FBrief::TDescriptor> demo_global_in_poseGraph;
    
    //多个地图融合优化 要发送给客户端的数据
    vector<Vector3d> get_t_global_multiClient();
    vector<Matrix3d> get_r_global_multiClient();
    vector<int> get_kf_id_hasComPlace_withOtherMap();
    void add2_t_global_multiClient(vector<Vector3d> t);
    void add2_r_global_multiClient(vector<Matrix3d> r);
    void add2_kf_id_hasComPlace_withOtherMap(vector<int> kf_id);
    void pop_kf_id_hasComPlace_withOtherMap();
//    void clear_t_global_multiClient();
//    void clear_r_global_multiClient();
//    void clear_kf_id_hasComPlace_withOtherMap();
    queue<int> loop_index_multiClient;//最早的更新id
    queue<int> curKF_loop_index_multiClient;//最晚的回环id
//    queue<int> earliest_multiClient_queue;
    bool isSendGloablData_multiClient;
    int max_frame_num_global;
    double min_dis;
    
    
    
    int kfNum_tree;//树里面帧的数量
    map<int,int> treeId_kf;//前一个是treeId , 后一个是kfId
    
    ReadWriteLock readWriteLock_fuseGlobalIndex;//写者优先
    std::queue<std::pair<int,int> > q_cur3d_old2d_globalIndex;//匹配的关键帧
    //匹配的其它地图的 clientId
    std::queue<int> q_old_clientId;
    Client* c;
    
//    这个后面要加锁 参与优化的过程中，不允许该地图参与其它地图的优化
//    int fuse_index;//全局优化中 作为其它地图时的 id
    
//    存放地图之间相对的pitch roll
//    他们锁为  keyFrameList_global_fusion_mutex 因为每次优化时，都会独占
//    vector<double> pitch_relativeOtherMap,roll_relativeOtherMap;
////    存放相对的地图clientid 存放的是pitch roll的下标
//    map<int, int> relativeOtherMap_clientId;
    
//    实验 为全局优化 提供先验
    vector<bool> fuseClientId;//未融合为false，融合为true
    vector<Matrix3d> fromI2OtherClient_r;
    vector<Vector3d> fromI2OtherClient_t;
    
    std::mutex globalLoop_mutex;
//    保存全局优化过程中 找到的匹配关系 暂时只保存当前5帧的
    vector<vector<int> > vpkf_index;
//    vector<vector<Matrix3d> > oldR_b_a;
//    vector<vector<Vector3d> > oldT_b_a;
    
    vector<vector<vector<cv::Point2f>>> measurements_old_norm_all;
    vector<vector<vector<Eigen::Vector3d>>> point_clouds_all;
    
    vector< vector<KeyFrame*> > vpCovKFi_cur_3d;//这里是共视帧
    vector< vector<KeyFrame*> > vpCovKFi_old_2d;
    
    vector<int> curGlobalLoopKf_id;//这里是回环帧，只保存最近5帧的，在第5帧做完主动的全局回环后，删掉里面第一帧的
    vector<int> oldGlobalLoopKf_id;
    vector<int> oldAgent_id_forLoopKf;
    
    int prePoseGraphOpti;//记录前一次发生回环优化的帧id
    int waitOpti;//记录最新等待回环优化的帧id，以防后面的帧没有通过回环检测
    int hasDetectLoopKfId;//记录检测到哪一帧了
    std::mutex hasDetectLoopKfId_mutex;
    int waitNum;
    
//    实现地图之间真正的融合 都在一个坐标系下（包括3d点）
//    Matrix3d relative_r_global_local;
//    Vector3d relative_t_global_local;
private:
    
    
    
    int max_frame_num;
    
    Vector3d last_P;
    
    //存储要发送给客户端的数据 这个是地图内部发生全局优化
    queue<vector<Vector3d> > t_global;
    vector<double> yaw_global;//没用
    queue<vector<Matrix3d> > r_global;
    
    //这个是多个地图之间发生融合优化 存储要发送给客户端的数据
    std::mutex rt_global_multiClient_mutex;
    queue<vector<Vector3d> > t_global_multiClient;
    queue<vector<Matrix3d> > r_global_multiClient;
    //另外存储一下 还没有发送过的 发生了多个地图之间的公共区域 不需要发送相对位姿
    queue<vector<int> > kf_id_hasComPlace_withOtherMap;//这个存储的是新发生的 和其它地图有公共区域的 kf的id
    
    
};

template <typename T>
T NormalizeAngle(const T& angle_degrees) {
    T two_pi(2.0 * 180);
    
    if (angle_degrees > T(0))
        return angle_degrees -
        two_pi * ceres::floor((angle_degrees + T(180)) / two_pi);
    else
        return angle_degrees +
        two_pi * ceres::floor((-angle_degrees + T(180)) / two_pi);
};

class AngleLocalParameterization {
public:
    
    template <typename T>
    bool operator()(const T* theta_radians, const T* delta_theta_radians,
                    T* theta_radians_plus_delta) const {
        *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);
        
        return true;
    }
    
    static ceres::LocalParameterization* Create() {
        return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                1, 1>);
    }
};

template <typename T> inline
void QuaternionInverse(const T q[4], T q_inverse[4])
{
    q_inverse[0] = q[0];
    q_inverse[1] = -q[1];
    q_inverse[2] = -q[2];
    q_inverse[3] = -q[3];
};

struct RelativeTError
{
    RelativeTError(double t_x, double t_y, double t_z)
    :t_x(t_x), t_y(t_y), t_z(t_z){}
    
    template <typename T>
    bool operator()(const T* const w_q_i, const T* ti, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        T i_q_w[4];
        QuaternionInverse(w_q_i, i_q_w);
        
        T t_i_ij[3];
        ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);
        
        residuals[0] = t_i_ij[0] - T(t_x);
        residuals[1] = t_i_ij[1] - T(t_y);
        residuals[2] = t_i_ij[2] - T(t_z);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z)
    {
        return (new ceres::AutoDiffCostFunction<
                RelativeTError, 3, 4, 3, 3>(
                                            new RelativeTError(t_x, t_y, t_z)));
    }
    
    double t_x, t_y, t_z;
    
};


struct TError
{
    TError(double t_x, double t_y, double t_z)
    :t_x(t_x), t_y(t_y), t_z(t_z){}
    
    template <typename T>
    bool operator()(const T* tj, T* residuals) const
    {
        residuals[0] = tj[0] - T(t_x);
        residuals[1] = tj[1] - T(t_y);
        residuals[2] = tj[2] - T(t_z);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z)
    {
        return (new ceres::AutoDiffCostFunction<
                TError, 3, 3>(
                              new TError(t_x, t_y, t_z)));
    }
    
    double t_x, t_y, t_z;
    
};

struct RelativeRTError
{
    RelativeRTError(double t_x, double t_y, double t_z, double q_w, double q_x, double q_y, double q_z, double t_var,  double q_var)
    :t_x(t_x), t_y(t_y), t_z(t_z), q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z), t_var(t_var), q_var(q_var)
    {
        t_norm = sqrt(t_x * t_x + t_y * t_y + t_z * t_z);
    }
    
    template <typename T>
    bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        T i_q_w[4];
        QuaternionInverse(w_q_i, i_q_w);
        
        T t_i_ij[3];
        ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);
        
        //residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_norm);
        //residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_norm);
        //residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_norm);
        residuals[0] = (t_i_ij[0] - T(t_x)) * T(t_var);
        residuals[1] = (t_i_ij[1] - T(t_y))* T(t_var);
        residuals[2] = (t_i_ij[2] - T(t_z))* T(t_var);
        
        T relative_q[4];
        relative_q[0] = T(q_w);
        relative_q[1] = T(q_x);
        relative_q[2] = T(q_y);
        relative_q[3] = T(q_z);
        
        T q_i_j[4];
        ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);
//        std::cout<<"q_i_j"<<q_i_j[0]<<" , "<<q_i_j[1]<<" , "<<q_i_j[2]<<" , "<<q_i_j[3]<<endl;
        
        T relative_q_inv[4];
        QuaternionInverse(relative_q, relative_q_inv);
        
        T error_q[4];
        ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);
        
        residuals[3] = error_q[1]* T(q_var);
        residuals[4] = error_q[2]* T(q_var);
        residuals[5] = error_q[3]* T(q_var);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double q_w, const double q_x, const double q_y, const double q_z,const double t_var, const double q_var)
    {
        return (new ceres::AutoDiffCostFunction<
                RelativeRTError, 6, 4, 3, 4, 3>(
                                                new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z,t_var, q_var)));
    }
    
    double t_x, t_y, t_z, t_norm;
    double q_w, q_x, q_y, q_z;
    double t_var, q_var;
    
};

struct RelativeRTError_relative
{
    RelativeRTError_relative(double t_x, double t_y, double t_z, double q_w, double q_x, double q_y, double q_z, double t_var,  double q_var)
    :t_x(t_x), t_y(t_y), t_z(t_z), q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z), t_var(t_var), q_var(q_var)
    {
        t_norm = sqrt(t_x * t_x + t_y * t_y + t_z * t_z);
    }
    
    template <typename T>
    bool operator()(const T* const w_q_i, const T* ti,T* residuals) const
    {
        
        residuals[0] = (ti[0] - T(t_x)) * T(t_var);
        residuals[1] = (ti[1] - T(t_y))* T(t_var);
        residuals[2] = (ti[2] - T(t_z))* T(t_var);
        
        T relative_q[4];
        relative_q[0] = T(q_w);
        relative_q[1] = T(q_x);
        relative_q[2] = T(q_y);
        relative_q[3] = T(q_z);
        
      
        T relative_q_inv[4];
        QuaternionInverse(relative_q, relative_q_inv);
        
        T error_q[4];
        ceres::QuaternionProduct(w_q_i, relative_q_inv, error_q);
        
        residuals[3] = error_q[1]* T(q_var);
        residuals[4] = error_q[2]* T(q_var);
        residuals[5] = error_q[3]* T(q_var);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double q_w, const double q_x, const double q_y, const double q_z,const double t_var, const double q_var)
    {
        return (new ceres::AutoDiffCostFunction<
                RelativeRTError_relative, 6, 4, 3>(
                                                new RelativeRTError_relative(t_x, t_y, t_z, q_w, q_x, q_y, q_z,t_var, q_var)));
    }
    
    double t_x, t_y, t_z, t_norm;
    double q_w, q_x, q_y, q_z;
    double t_var, q_var;
    
};

struct RelativeRTError_3opti_main
{
    RelativeRTError_3opti_main(double t_x, double t_y, double t_z, double q_w, double q_x, double q_y, double q_z, double t_var,  double q_var)
    :t_x(t_x), t_y(t_y), t_z(t_z), q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z), t_var(t_var), q_var(q_var)
    {
        t_norm = sqrt(t_x * t_x + t_y * t_y + t_z * t_z);
    }
    
    template <typename T>
    bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, const T* w_q_w1_w2, const T* tj_w1_w2, T* residuals) const
    {
//        补充 位姿变换
        T t_i_w1[3];
        ceres::QuaternionRotatePoint(w_q_w1_w2, ti, t_i_w1);
        T q_i_w1[4];
        ceres::QuaternionProduct(w_q_w1_w2, w_q_i, q_i_w1);
        
        
        T t_w_ij[3];
        t_w_ij[0] =tj[0] -t_i_w1[0]-tj_w1_w2[0] ;
        t_w_ij[1] =tj[1] -t_i_w1[1]-tj_w1_w2[1] ;
        t_w_ij[2] =tj[2] -t_i_w1[2]-tj_w1_w2[2] ;
        
        T i_q_w[4];
        QuaternionInverse(q_i_w1, i_q_w);
        
        T t_i_ij[3];
        ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);
        
        //residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_norm);
        //residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_norm);
        //residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_norm);
        residuals[0] = (t_i_ij[0] - T(t_x)) * T(t_var);
        residuals[1] = (t_i_ij[1] - T(t_y))* T(t_var);
        residuals[2] = (t_i_ij[2] - T(t_z))* T(t_var);
        
        T relative_q[4];
        relative_q[0] = T(q_w);
        relative_q[1] = T(q_x);
        relative_q[2] = T(q_y);
        relative_q[3] = T(q_z);
        
        T q_i_j[4];
        ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);
//        std::cout<<"q_i_j"<<q_i_j[0]<<" , "<<q_i_j[1]<<" , "<<q_i_j[2]<<" , "<<q_i_j[3]<<endl;
        
        T relative_q_inv[4];
        QuaternionInverse(relative_q, relative_q_inv);
        
        T error_q[4];
        ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);
        
        residuals[3] = error_q[1]* T(q_var);
        residuals[4] = error_q[2]* T(q_var);
        residuals[5] = error_q[3]* T(q_var);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double q_w, const double q_x, const double q_y, const double q_z,const double t_var, const double q_var)
    {
        return (new ceres::AutoDiffCostFunction<
                RelativeRTError_3opti_main, 6, 4, 3, 4, 3, 4, 3>(
                                                new RelativeRTError_3opti_main(t_x, t_y, t_z, q_w, q_x, q_y, q_z,t_var, q_var)));
    }
    
    double t_x, t_y, t_z, t_norm;
    double q_w, q_x, q_y, q_z;
    double t_var, q_var;
    
};

struct RelativeRTError_3opti
{
    RelativeRTError_3opti(double t_x, double t_y, double t_z, double q_w, double q_x, double q_y, double q_z, double t_var,  double q_var)
    :t_x(t_x), t_y(t_y), t_z(t_z), q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z), t_var(t_var), q_var(q_var)
    {
        t_norm = sqrt(t_x * t_x + t_y * t_y + t_z * t_z);
    }
    
    template <typename T>
    bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, const T* w_q_w1_w2, const T* tj_w1_w2, T* residuals) const
    {
//        补充 位姿变换
        T t_j_w1[3];
        ceres::QuaternionRotatePoint(w_q_w1_w2, tj, t_j_w1);
        T q_j_w1[4];
        ceres::QuaternionProduct(w_q_w1_w2, w_q_j, q_j_w1);
        
        
        T t_w_ij[3];
        t_w_ij[0] = t_j_w1[0]+tj_w1_w2[0] - ti[0];
        t_w_ij[1] = t_j_w1[1]+tj_w1_w2[1]  - ti[1];
        t_w_ij[2] = t_j_w1[2]+tj_w1_w2[2]  - ti[2];
        
        T i_q_w[4];
        QuaternionInverse(w_q_i, i_q_w);
        
        T t_i_ij[3];
        ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);
        
        //residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_norm);
        //residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_norm);
        //residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_norm);
        residuals[0] = (t_i_ij[0] - T(t_x)) * T(t_var);
        residuals[1] = (t_i_ij[1] - T(t_y))* T(t_var);
        residuals[2] = (t_i_ij[2] - T(t_z))* T(t_var);
        
        T relative_q[4];
        relative_q[0] = T(q_w);
        relative_q[1] = T(q_x);
        relative_q[2] = T(q_y);
        relative_q[3] = T(q_z);
        
        T q_i_j[4];
        ceres::QuaternionProduct(i_q_w, q_j_w1, q_i_j);
//        std::cout<<"q_i_j"<<q_i_j[0]<<" , "<<q_i_j[1]<<" , "<<q_i_j[2]<<" , "<<q_i_j[3]<<endl;
        
        T relative_q_inv[4];
        QuaternionInverse(relative_q, relative_q_inv);
        
        T error_q[4];
        ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);
        
        residuals[3] = error_q[1]* T(q_var);
        residuals[4] = error_q[2]* T(q_var);
        residuals[5] = error_q[3]* T(q_var);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double q_w, const double q_x, const double q_y, const double q_z,const double t_var, const double q_var)
    {
        return (new ceres::AutoDiffCostFunction<
                RelativeRTError_3opti, 6, 4, 3, 4, 3, 4, 3>(
                                                new RelativeRTError_3opti(t_x, t_y, t_z, q_w, q_x, q_y, q_z,t_var, q_var)));
    }
    
    double t_x, t_y, t_z, t_norm;
    double q_w, q_x, q_y, q_z;
    double t_var, q_var;
    
};

template <typename T>
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
{
    
    T y = yaw / T(180.0) * T(M_PI);
    T p = pitch / T(180.0) * T(M_PI);
    T r = roll / T(180.0) * T(M_PI);
    
    
    R[0] = cos(y) * cos(p);
    R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
    R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
    R[3] = sin(y) * cos(p);
    R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
    R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
    R[6] = -sin(p);
    R[7] = cos(p) * sin(r);
    R[8] = cos(p) * cos(r);
};

template <typename T>
void quaternionToRotationMatrix(const T w, const T yaw, const T pitch, const T roll, T R[9])
{
   
    R[0] =  T(1) -  T(2) * (pitch * pitch) - T(2)* (roll * roll);
    R[1] = T(2) * yaw * pitch- T(2) * w* roll;
    R[2] = T(2) *yaw * roll+ T(2) * w *pitch;
    R[3] = T(2) * yaw * pitch+ T(2) * w * roll;
    R[4] =  T(1) - T(2) * (yaw * yaw) - T(2)* (roll * roll);
    R[5] = T(2) * pitch* roll - T(2) * w * yaw;
    R[6] = T(2) * yaw * roll-T(2) * w * pitch;
    R[7] = T(2) * pitch *roll +T(2) * w *yaw;
    R[8] =  T(1) - T(2) * (yaw * yaw) - T(2) * (pitch * pitch);
};


template <typename T>
void RToYPR_y(const T R[9], T yaw[1])
{
    T y = atan2(R[3], R[0]);
    yaw[0]= y / T(M_PI) * T(180.0);
};




template <typename T>
void RotationMatrixToYawPitchRoll ( const T R[9], T yaw,  T pitch,  T roll )
{
    T n[3];
    n[0]=R[0];
    n[1]=R[3];
    n[2]=R[6];

    
    T o[3];
    o[0]= R[1];
    o[1]= R[4];
    o[2]= R[7];
    T a[3];
    a[0]= R[2];
    a[ 1]= R[5];
    a[2]= R[8];
    
    T y = atan2(n[1], n[0]);
    T p = atan2(-n[2], n[0] * cos(y) + n[1] * sin(y));
    T r = atan2(a[0] * sin(y) - a[1] * cos(y), -o[0] * sin(y) + o[1] * cos(y));
    
    yaw=y/T(M_PI)* T(180.0);
    pitch=p/T(M_PI)* T(180.0);
    roll=r/T(M_PI)* T(180.0);

};

template <typename T>
void RotationMatrixTranspose(const T R[9], T inv_R[9])
{
    inv_R[0] = R[0];
    inv_R[1] = R[3];
    inv_R[2] = R[6];
    inv_R[3] = R[1];
    inv_R[4] = R[4];
    inv_R[5] = R[7];
    inv_R[6] = R[2];
    inv_R[7] = R[5];
    inv_R[8] = R[8];
};

template <typename T>
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
{
    r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
    r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
    r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
};

template <typename T>
void RotationMatrixRotateT(const T R[9], const T t[3], T r_t[3])
{
    r_t[0] = -R[0] * t[0] - R[1] * t[1] - R[2] * t[2];
    r_t[1] = -R[3] * t[0] - R[4] * t[1] - R[5] * t[2];
    r_t[2] = -R[6] * t[0] - R[7] * t[1] - R[8] * t[2];
};

template <typename T>
void RotationMatrixRotateMatrix(const T R[9], const T r_2[9], T r_3[9])
{
    r_3[0] = R[0] * r_2[0] + R[1] * r_2[3] + R[2] * r_2[6];
    r_3[1] = R[0] * r_2[1] + R[1] * r_2[4] + R[2] * r_2[7];
    r_3[2] = R[0] * r_2[2] + R[1] * r_2[5] + R[2] * r_2[8];

    r_3[3] = R[3] * r_2[0] + R[4] * r_2[3] + R[5] * r_2[6];
    r_3[4] = R[3] * r_2[1] + R[4] * r_2[4] + R[5] * r_2[7];
    r_3[5] = R[3] * r_2[2] + R[4] * r_2[5] + R[5] * r_2[8];
    
    r_3[6] = R[6] * r_2[0] + R[7] * r_2[3] + R[8] * r_2[6];
    r_3[7] = R[6] * r_2[1] + R[7] * r_2[4] + R[8] * r_2[7];
    r_3[8] = R[6] * r_2[2] + R[7] * r_2[5] + R[8] * r_2[8];
};

/**
//估计1个相机位姿用的
struct SixDOFError
{
    SixDOFError(float px , float py , float pz , float u, float v )
    :px(px), py(py), pz(pz), u(u), v(v){}
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const ri, const T* ti, T* residuals) const
    {
        T t_w[3];
        t_w[0] = T(px) ;
        t_w[1] = T(py) ;
        t_w[2] = T(pz) ;
        
        T t_i_w[3];
        t_i_w[0] = ti[0] ;
        t_i_w[1] = ti[1] ;
        t_i_w[2] = ti[2] ;
        
        // euler to rotation
        T r_i_w[9];
        YawPitchRollToRotationMatrix(ri[0] , ri[1] , ri[2] , r_i_w);
        T t_i[3];
        RotationMatrixRotatePoint(r_i_w, t_w, t_i);
        t_i[0]+=t_i_w[0] ;
        t_i[1]+=t_i_w[1] ;
        t_i[2]+=t_i_w[2] ;
        //转成相机平面坐标 也就是上面传进来的uv并不是像素坐标，而是 相机平面坐标
//        //计算残差
        
//        if(t_i[2]>-0.000001 && t_i[2]<0.000001){
//            residuals[0] = (t_i[0] - T(u));
//            residuals[1] = (t_i[1] - T(v));
//        }else{
            residuals[0] = (t_i[0]/t_i[2] - T(u));
            residuals[1] = (t_i[1]/t_i[2] - T(v));
//        }
        
//        cout<<residuals[0]<<" ,"<<residuals[1]<<endl;
        
        return true;
    }
    
    static ceres::CostFunction* Create(const float px , const float py ,const float pz ,const float u,const float v )
    {
        return (new ceres::AutoDiffCostFunction<
                SixDOFError, 2, 3, 3>(
                                             new SixDOFError(px, py, pz, u, v)));
    }
    
    float u,v,  px, py, pz;
    
};
 */

struct FourDOFError
{
    FourDOFError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){}
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        // rotation transpose
        T i_R_w[9];
        //求出旋转矩阵的转置
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
//        //计算残差
        residuals[0] = (t_i_ij[0] - T(t_x));
        residuals[1] = (t_i_ij[1] - T(t_y));
        residuals[2] = (t_i_ij[2] - T(t_z));
        residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double pitch_i, const double roll_i)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFError, 4, 1, 3, 1, 3>(
                                             new FourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
    
};

struct FourDOFError_another
{
    FourDOFError_another(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i,double weight)
    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i),weight(weight){
//        weight = 1.0;//2 4 默认一直用的是1
    }
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        // rotation transpose
        T i_R_w[9];
        //求出旋转矩阵的转置
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
//        //计算残差
        residuals[0] = (t_i_ij[0] - T(t_x))* T(weight);
        residuals[1] = (t_i_ij[1] - T(t_y))* T(weight);
        residuals[2] = (t_i_ij[2] - T(t_z))* T(weight);
        residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw)) * T(weight) ;/// T(1.0)
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double pitch_i, const double roll_i,double weight=1)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFError_another, 4, 1, 3, 1, 3>(
                                             new FourDOFError_another(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i,weight)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
    double weight;
};

struct FourDOFError_another_pitchRoll
{
    FourDOFError_another_pitchRoll(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){
        weight = 1.0;//2 4
    }
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, const T* pitch, const T* roll, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i)+pitch[0], T(roll_i)+roll[0], w_R_i);
        // rotation transpose
        T i_R_w[9];
        //求出旋转矩阵的转置
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
//        //计算残差
        residuals[0] = (t_i_ij[0] - T(t_x))* T(weight);
        residuals[1] = (t_i_ij[1] - T(t_y))* T(weight);
        residuals[2] = (t_i_ij[2] - T(t_z))* T(weight);
        residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw)) * T(weight) ;/// T(1.0)
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double pitch_i, const double roll_i)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFError_another_pitchRoll, 4, 1, 3, 1, 3, 1, 1>(
                                             new FourDOFError_another_pitchRoll(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
    double weight;
};
//这上面三个 其实只是权重不一样，后面改成权重 从参数传入
struct FourDOFError_posegraph
{
    FourDOFError_posegraph(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){
        weight = 2.0;//2 4
    }
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        // rotation transpose
        T i_R_w[9];
        //求出旋转矩阵的转置
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
//        //计算残差
        residuals[0] = (t_i_ij[0] - T(t_x))* T(weight);
        residuals[1] = (t_i_ij[1] - T(t_y))* T(weight);
        residuals[2] = (t_i_ij[2] - T(t_z))* T(weight);
        residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw)) * T(weight) ;/// T(1.0)
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double pitch_i, const double roll_i)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFError_posegraph, 4, 1, 3, 1, 3>(
                                             new FourDOFError_posegraph(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
    double weight;
};

/**
struct FourDOFError_another_reprojection
{
    FourDOFError_another_reprojection(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pts_j_z, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pts_j_z(pts_j_z), pose_Tic_cur_t_x(pose_Tic_cur_t_x), pose_Tic_cur_t_y(pose_Tic_cur_t_y), pose_Tic_cur_t_z(pose_Tic_cur_t_z), pose_Tic_cur_r_yaw(pose_Tic_cur_r_yaw), pose_Tic_cur_r_pitch(pose_Tic_cur_r_pitch), pose_Tic_cur_r_roll(pose_Tic_cur_r_roll){
        weight = 100.0;
    }
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* yaw_j, const T* tj, T* residuals) const
    {
        
        
        // euler to rotation
        T w2_R_i2[9];
        YawPitchRollToRotationMatrix(yaw_j[0],yaw_j[1], yaw_j[2], w2_R_i2);
        T w2_t_i2[3];
        w2_t_i2[0]=tj[0];
        w2_t_i2[1]=tj[1];
        w2_t_i2[2]=tj[2];
        
        T R_imu_c[9];
        YawPitchRollToRotationMatrix(T(pose_Tic_cur_r_yaw), T(pose_Tic_cur_r_pitch), T(pose_Tic_cur_r_roll), R_imu_c);
        T t_imu_c[3];
        t_imu_c[0]=T(pose_Tic_cur_t_x);
        t_imu_c[1]=T(pose_Tic_cur_t_y);
        t_imu_c[2]=T(pose_Tic_cur_t_z);
        
        T r_w_c[9];
        RotationMatrixRotateMatrix(w2_R_i2,R_imu_c,r_w_c);
        T t_w_c[3];
        RotationMatrixRotatePoint(w2_R_i2, t_imu_c, t_w_c);
        t_w_c[0]+=w2_t_i2[0];
        t_w_c[1]+=w2_t_i2[1];
        t_w_c[2]+=w2_t_i2[2];
        
        // rotation transpose
        T c_R_w[9];
        //求出旋转矩阵的转置
        RotationMatrixTranspose(r_w_c, c_R_w);
        T c_t_w[3];
        RotationMatrixRotateT(c_R_w, t_w_c, c_t_w);
        
        T t_i[3];
        t_i[0]=T(pts_i_x);
        t_i[1]=T(pts_i_y);
        t_i[2]=T(pts_i_z);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(c_R_w, t_i, t_i_ij);
        t_i_ij[0]+=c_t_w[0];
        t_i_ij[1]+=c_t_w[1];
        t_i_ij[2]+=c_t_w[2];
         
        
        
//        //计算残差
        residuals[0] = (t_i_ij[0]/t_i_ij[2] - T(pts_j_x))* T(weight);
        residuals[1] = (t_i_ij[1]/t_i_ij[2] - T(pts_j_y))* T(weight);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pts_j_z, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFError_another_reprojection, 2, 3, 3>(
                                             new FourDOFError_another_reprojection(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pts_j_z, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pts_j_z;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    double weight;
};
 */
/**
struct real_FourDOFError_another_reprojection
{
    real_FourDOFError_another_reprojection(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pts_j_z, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll, double loop_pitch,double loop_roll)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pts_j_z(pts_j_z), pose_Tic_cur_t_x(pose_Tic_cur_t_x), pose_Tic_cur_t_y(pose_Tic_cur_t_y), pose_Tic_cur_t_z(pose_Tic_cur_t_z), pose_Tic_cur_r_yaw(pose_Tic_cur_r_yaw), loop_pitch(loop_pitch), loop_roll(loop_roll){
        weight = 100.0;
    }
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* yaw_j, const T* tj, T* residuals) const
    {
        
        
        // euler to rotation
        T w2_R_i2[9];
        YawPitchRollToRotationMatrix(yaw_j[0],T(loop_pitch), T(loop_roll), w2_R_i2);
        T w2_t_i2[3];
        w2_t_i2[0]=tj[0];
        w2_t_i2[1]=tj[1];
        w2_t_i2[2]=tj[2];
        
        T R_imu_c[9];
        YawPitchRollToRotationMatrix(T(pose_Tic_cur_r_yaw), T(pose_Tic_cur_r_pitch), T(pose_Tic_cur_r_roll), R_imu_c);
        T t_imu_c[3];
        t_imu_c[0]=T(pose_Tic_cur_t_x);
        t_imu_c[1]=T(pose_Tic_cur_t_y);
        t_imu_c[2]=T(pose_Tic_cur_t_z);
        
        T r_w_c[9];
        RotationMatrixRotateMatrix(w2_R_i2,R_imu_c,r_w_c);
        T t_w_c[3];
        RotationMatrixRotatePoint(w2_R_i2, t_imu_c, t_w_c);
        t_w_c[0]+=w2_t_i2[0];
        t_w_c[1]+=w2_t_i2[1];
        t_w_c[2]+=w2_t_i2[2];
        
        // rotation transpose
        T c_R_w[9];
        //求出旋转矩阵的转置
        RotationMatrixTranspose(r_w_c, c_R_w);
        T c_t_w[3];
        RotationMatrixRotateT(c_R_w, t_w_c, c_t_w);
        
        T t_i[3];
        t_i[0]=T(pts_i_x);
        t_i[1]=T(pts_i_y);
        t_i[2]=T(pts_i_z);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(c_R_w, t_i, t_i_ij);
        t_i_ij[0]+=c_t_w[0];
        t_i_ij[1]+=c_t_w[1];
        t_i_ij[2]+=c_t_w[2];
         
        
        
//        //计算残差
        residuals[0] = (t_i_ij[0]/t_i_ij[2] - T(pts_j_x))* T(weight);
        residuals[1] = (t_i_ij[1]/t_i_ij[2] - T(pts_j_y))* T(weight);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pts_j_z, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll, const double loop_pitch, const double loop_roll)
    {
        return (new ceres::AutoDiffCostFunction<
                real_FourDOFError_another_reprojection, 2, 1, 3>(
                                             new real_FourDOFError_another_reprojection(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pts_j_z, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll, loop_pitch, loop_roll)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pts_j_z;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    double loop_pitch, loop_roll;
    double weight;
};
 */
/**
struct FourDOFError_my_t
{
    FourDOFError_my_t(double t_x, double t_y, double t_z) :t_x(t_x), t_y(t_y), t_z(t_z){}
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], yaw_i[1], yaw_i[2], w_R_i);
        // rotation transpose
        T i_R_w[9];
        //求出旋转矩阵的转置
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
//        //计算残差
        residuals[0] = (t_i_ij[0] - T(t_x));
        residuals[1] = (t_i_ij[1] - T(t_y));
        residuals[2] = (t_i_ij[2] - T(t_z));
       
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFError_my_t, 3, 3, 3, 3>(
                                             new FourDOFError_my_t(t_x, t_y, t_z)));
    }
    
    double t_x, t_y, t_z;
    
};
 */
/**
struct FourDOFError_my_r
{
    FourDOFError_my_r(double relative_yaw, double relative_pitch, double relative_roll)
    : relative_yaw(relative_yaw), relative_pitch(relative_pitch), relative_roll(relative_roll){}
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const yaw_i,  const T* yaw_j, T* residuals) const
    {
        
        residuals[0] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));
        residuals[1] = NormalizeAngle(yaw_j[1] - yaw_i[1] - T(relative_pitch));
        residuals[2] = NormalizeAngle(yaw_j[2] - yaw_i[2] - T(relative_roll));
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double relative_yaw, const double relative_pitch, const double relative_roll)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFError_my_r, 3, 3, 3>(
                                             new FourDOFError_my_r(relative_yaw, relative_pitch, relative_roll)));
    }
    
    double relative_yaw, relative_pitch, relative_roll;
    
};
 */

struct EightDOFWeightError_yaw
{
    EightDOFWeightError_yaw( double relative_yaw, double pitch_i, double roll_i, double pitch_j, double roll_j)
    :relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i), pitch_j(pitch_j), roll_j(roll_j){
//        weight = 10;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const yaw_i, const T* yaw_j, const T* ypr_i,  const T* ypr_j, T* residuals) const
    {
       
        // euler to rotation
        T R_wm_w2[9];
//        YawPitchRollToRotationMatrix(ypr_j[0], ypr_j[1],ypr_j[2], w2_R_wm);
        quaternionToRotationMatrix(ypr_j[3],ypr_j[0], ypr_j[1],ypr_j[2],R_wm_w2);
        // euler to rotation
        T R_wm_w1[9];
//        YawPitchRollToRotationMatrix(ypr_i[0], ypr_i[1],ypr_i[2], w1_R_wm);
        quaternionToRotationMatrix(ypr_i[3],ypr_i[0], ypr_i[1],ypr_i[2],R_wm_w1);
        // rotation transpose
        T R_w2_wm[9];
        RotationMatrixTranspose(R_wm_w2, R_w2_wm);
        // euler to rotation
        T R_w1_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i),T(roll_i), R_w1_i);
        // rotation matrix rotate matrix
        T r_wm_i[9];
        RotationMatrixRotateMatrix(R_wm_w1, R_w1_i,  r_wm_i);
        
        // rotation matrix rotate matrix
        T r_w2_i[9];
        RotationMatrixRotateMatrix(R_w2_wm, r_wm_i, r_w2_i);
        
        
        
        T yaw_i_w2J[1];
        RToYPR_y(r_w2_i,yaw_i_w2J);
        
        //计算残差
        
//        residuals[0] = NormalizeAngle(( yaw_j_w1I- yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);
        residuals[0] = NormalizeAngle(( yaw_j[0]- yaw_i_w2J[0] - T(relative_yaw))) ;//* T(weight) / T(10.0);
        
        return true;
    }
    
    static ceres::CostFunction* Create( const double relative_yaw, const double pitch_i, const double roll_i, const double pitch_j, const double roll_j)
    {
        return (new ceres::AutoDiffCostFunction<
                EightDOFWeightError_yaw, 1, 1, 1, 4, 4>(
                                                   new EightDOFWeightError_yaw(relative_yaw, pitch_i, roll_i,pitch_j , roll_j)));
    }
    
    double  pitch_j, roll_j;
    double relative_yaw, pitch_i, roll_i;
    double weight;
    
};

struct SixDOFWeightError_relativeR
{
    SixDOFWeightError_relativeR( ){
        weight = 10;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* ypr_i,  const T* ypr_j, T* residuals) const
    {
        
        // euler to rotation
        T w2_R_wm[9];
//        YawPitchRollToRotationMatrix(ypr_j[0], ypr_j[1],ypr_j[2], w2_R_wm);
        quaternionToRotationMatrix(ypr_j[0], ypr_j[1],ypr_j[2],ypr_j[3],w2_R_wm);
        // euler to rotation
        T w1_R_wm[9];
//        YawPitchRollToRotationMatrix(ypr_i[0], ypr_i[1],ypr_i[2], w1_R_wm);
        quaternionToRotationMatrix(ypr_i[0], ypr_i[1],ypr_i[2],ypr_i[3],w1_R_wm);
       
        
        T yaw_i_w1, pitch_i_w1, roll_i_w1;
        RotationMatrixToYawPitchRoll(w1_R_wm,yaw_i_w1, pitch_i_w1, roll_i_w1);
        T yaw_j_w1, pitch_j_w1, roll_j_w1;
        RotationMatrixToYawPitchRoll(w2_R_wm,yaw_j_w1, pitch_j_w1, roll_j_w1);
        //计算残差
        
        residuals[0] = NormalizeAngle(( yaw_j_w1- yaw_i_w1 )) * T(weight) / T(10.0);
        residuals[1] = NormalizeAngle(( pitch_j_w1- pitch_i_w1 )) * T(weight) / T(10.0);
        residuals[2] = NormalizeAngle(( roll_j_w1- roll_i_w1)) * T(weight) / T(10.0);
        return true;
    }
    
    static ceres::CostFunction* Create()
    {
        return (new ceres::AutoDiffCostFunction<
                SixDOFWeightError_relativeR, 3,  4, 4>( new SixDOFWeightError_relativeR()));
    }
    
    double weight;
    
};

struct SixDOFWeightError_relativet
{
    SixDOFWeightError_relativet( ){
        weight = 10;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* t_i,  const T* t_j, T* residuals) const
    {
       
        //计算残差
        
        residuals[0] = (-t_i[0]+t_j[0]) * T(weight) / T(10.0);
        residuals[1] = (-t_i[1]+t_j[1]) * T(weight) / T(10.0);
        residuals[2] = (-t_i[2]+t_j[2]) * T(weight) / T(10.0);
        return true;
    }
    
    static ceres::CostFunction* Create()
    {
        return (new ceres::AutoDiffCostFunction<
                SixDOFWeightError_relativet, 3,  3, 3>( new SixDOFWeightError_relativet()));
    }
    
    double weight;
    
};

struct FourDOFWeightError
{
    FourDOFWeightError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i, double weight)
    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i), weight(weight){
//        weight = 10;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        // rotation transpose
        T i_R_w[9];
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
        //计算残差
        residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
        residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
        residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
        residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double pitch_i, const double roll_i,const double weight)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError, 4, 1, 3, 1, 3>(
                                                   new FourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i, weight)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
    double weight;
    
};

struct FourDOFWeightError_forRemove
{
    FourDOFWeightError_forRemove(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i, double weight)
    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i), weight(weight){
//        weight = 3;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        // rotation transpose
        T i_R_w[9];
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
        //计算残差
        residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
        residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
        residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
        residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double pitch_i, const double roll_i, const double weight)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError_forRemove, 4, 1, 3, 1, 3>(
                                                   new FourDOFWeightError_forRemove(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i,weight)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
    double weight;
    
};

/**
struct SixDOFWeightError_relativePose
{
    SixDOFWeightError_relativePose(double t_x, double t_y, double t_z, double yaw_i, double pitch_i, double roll_i)
    :t_x(t_x), t_y(t_y), t_z(t_z), yaw_i(yaw_i), pitch_i(pitch_i), roll_i(roll_i){
        weight = 10;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const yaw_i_i, const T* ti, T* residuals) const
    {
        //计算残差
        residuals[0] = (ti[0] - T(t_x)) * T(weight);
        residuals[1] = (ti[1] - T(t_y)) * T(weight);
        residuals[2] = (ti[2] - T(t_z)) * T(weight);
        residuals[3] = NormalizeAngle((yaw_i_i[0] - T(yaw_i))) * T(weight) / T(10.0);
        residuals[4] = NormalizeAngle((yaw_i_i[1] - T(pitch_i))) * T(weight) / T(10.0);
        residuals[5] = NormalizeAngle((yaw_i_i[2] - T(roll_i))) * T(weight) / T(10.0);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double yaw_i, const double pitch_i, const double roll_i)
    {
        return (new ceres::AutoDiffCostFunction<
                SixDOFWeightError_relativePose, 6, 3, 3>(
                                                   new SixDOFWeightError_relativePose(t_x, t_y, t_z, yaw_i, pitch_i, roll_i)));
    }
    
    double t_x, t_y, t_z;
    double yaw_i, pitch_i, roll_i;
    double weight;
    
};
 */

struct FourDOFWeightError_cur2main
{
    FourDOFWeightError_cur2main(double t_x, double t_y, double t_z, double relative_yaw,  double relative_pitch, double relative_roll, double yaw_i, double pitch_i, double roll_i ,double t0_i, double t1_i, double t2_i, double pitch_j, double roll_j, double weight)
    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), relative_pitch(relative_pitch), relative_roll(relative_roll), yaw_i(yaw_i) ,pitch_i(pitch_i), roll_i(roll_i) ,t0_i(t0_i) ,t1_i(t1_i) ,t2_i(t2_i) ,pitch_j(pitch_j) ,roll_j(roll_j),weight(weight){
//        weight = 10;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const yaw_i_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        T t_real_i[3];
        t_real_i[0]=T(t0_i);
        t_real_i[1]=T(t1_i);
        t_real_i[2]=T(t2_i);
        T w_R_i_real[9];
        YawPitchRollToRotationMatrix(T(yaw_i), T(pitch_i), T(roll_i), w_R_i_real);
        
        
        T r_w2_w1[9];
        YawPitchRollToRotationMatrix(yaw_i_i[0], yaw_i_i[1],yaw_i_i[2], r_w2_w1);
        T r_w1_w2[9];
        RotationMatrixTranspose(r_w2_w1, r_w1_w2);
        T t_w1_w2[3];
        RotationMatrixRotateT(r_w1_w2, ti, t_w1_w2);
        
        
        T r_w2_j[9];
        YawPitchRollToRotationMatrix(yaw_j[0], T(pitch_j), T(roll_j), r_w2_j);
        T r_w1_j[9];
        RotationMatrixRotateMatrix(r_w1_w2,r_w2_j,r_w1_j);
        T t_w1_j[3];
        RotationMatrixRotatePoint(r_w1_w2, tj, t_w1_j);
        t_w1_j[0]+=t_w1_w2[0];
        t_w1_j[1]+=t_w1_w2[1];
        t_w1_j[2]+=t_w1_w2[2];
        
 
        T t_w_ij[3];
        t_w_ij[0] = t_w1_j[0] - t_real_i[0];
        t_w_ij[1] = t_w1_j[1] - t_real_i[1];
        t_w_ij[2] = t_w1_j[2] - t_real_i[2];
        
        
        // rotation transpose
        T i_R_w[9];
        RotationMatrixTranspose(w_R_i_real, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
        //计算残差
        residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
        residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
        residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
        
        
        T yaw_w1_j;
        T pitch_w1_j;
        T roll_w1_j;
        RotationMatrixToYawPitchRoll(r_w1_j, yaw_w1_j, pitch_w1_j, roll_w1_j);
        
        //下面写错了
        residuals[3] = NormalizeAngle((yaw_w1_j - T(yaw_i) - T(relative_yaw))) * T(weight) / T(10.0);
//        residuals[4] = NormalizeAngle((pitch_w1_j - T(pitch_i) - T(relative_pitch))) * T(weight) / T(10.0);
//        residuals[5] = NormalizeAngle((roll_w1_j - T(roll_i) - T(relative_roll))) * T(weight) / T(10.0);
//       std::cout<<"test residuals[4]:"<<residuals[4]<<std::endl;
//        std::cout<<"test residuals[5]:"<<residuals[5]<<std::endl;
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double relative_pitch, const double relative_roll, const double yaw_i,const double pitch_i, const double roll_i, const double t0_i, const double t1_i, const double t2_i, const double pitch_j, const double roll_j,const double weight )
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError_cur2main, 4, 1, 3, 1, 3>(
                                                   new FourDOFWeightError_cur2main(t_x, t_y, t_z, relative_yaw, relative_pitch, relative_roll, yaw_i, pitch_i, roll_i, t0_i, t1_i, t2_i, pitch_j, roll_j,weight)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, relative_pitch, relative_roll;
    double yaw_i, pitch_i, roll_i;
    double t0_i, t1_i, t2_i;
    double pitch_j, roll_j;
    double weight;
    
};

/**
struct FourDOFWeightError_another
{
    FourDOFWeightError_another(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){
        weight = 10;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        // rotation transpose
        T i_R_w[9];
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
        //计算残差
        residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
        residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
        residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
        residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double pitch_i, const double roll_i)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError_another, 4, 1, 3, 1, 3>(
                                                   new FourDOFWeightError_another(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
    double weight;
    
};
 */
/**
struct FourDOFWeightError_global
{
    FourDOFWeightError_global(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){
        weight = 10;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        // rotation transpose
        T i_R_w[9];
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
        //计算残差
        residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
        residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
        residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
        residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double pitch_i, const double roll_i)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError_global, 4, 1, 3, 1, 3>(
                                                   new FourDOFWeightError_global(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
    double weight;
    
};
 */
struct FourDOFWeightError_reprojection
{
    FourDOFWeightError_reprojection(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pts_j_z, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll,  double kf_pitch, double kf_roll, double weight)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pts_j_z(pts_j_z), pose_Tic_cur_t_x(pose_Tic_cur_t_x), pose_Tic_cur_t_y(pose_Tic_cur_t_y), pose_Tic_cur_t_z(pose_Tic_cur_t_z), pose_Tic_cur_r_yaw(pose_Tic_cur_r_yaw), pose_Tic_cur_r_pitch(pose_Tic_cur_r_pitch), pose_Tic_cur_r_roll(pose_Tic_cur_r_roll),  kf_pitch(kf_pitch), kf_roll(kf_roll), weight(weight){
//        weight = 20;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const yaw_kf, const T* t_kf, const T* r_w2_w1, const T* t_w2_w1, T* residuals) const
    {
        T t_w1_i[3];
        t_w1_i[0] = T(pts_i_x);
        t_w1_i[1] = T(pts_i_y);
        t_w1_i[2] = T(pts_i_z);
        
        // euler to rotation
        T R_w2_w1[9];
        YawPitchRollToRotationMatrix(r_w2_w1[0], r_w2_w1[1], r_w2_w1[2], R_w2_w1);
        
        T t_w2_i[3];
        RotationMatrixRotatePoint(R_w2_w1, t_w1_i, t_w2_i);
        
        t_w2_i[0]+=t_w2_w1[0];
        t_w2_i[1]+=t_w2_w1[1];
        t_w2_i[2]+=t_w2_w1[2];
        
        
        T R_w2_imu[9];
        YawPitchRollToRotationMatrix(yaw_kf[0], T(kf_pitch), T(kf_roll), R_w2_imu);
        // rotation transpose
        T R_imu_w2[9];
        RotationMatrixTranspose(R_w2_imu, R_imu_w2);
        T t_imu_w2[3];
        RotationMatrixRotateT(R_imu_w2, t_kf, t_imu_w2);
        // rotation matrix rotate point
        T t_imu_i[3];
        RotationMatrixRotatePoint(R_imu_w2, t_w2_i, t_imu_i);
        t_imu_i[0]+=t_imu_w2[0];
        t_imu_i[1]+=t_imu_w2[1];
        t_imu_i[2]+=t_imu_w2[2];
        
        T R_imu_c[9];
        YawPitchRollToRotationMatrix(T(pose_Tic_cur_r_yaw), T(pose_Tic_cur_r_pitch), T(pose_Tic_cur_r_roll), R_imu_c);
        T t_imu_c[3];
        t_imu_c[0]=T(pose_Tic_cur_t_x);
        t_imu_c[1]=T(pose_Tic_cur_t_y);
        t_imu_c[2]=T(pose_Tic_cur_t_z);
        
        T R_c_imu[9];
        RotationMatrixTranspose(R_imu_c, R_c_imu);
        T t_c_imu[3];
        RotationMatrixRotateT(R_c_imu, t_imu_c, t_c_imu);
        // rotation matrix rotate point
        T t_cam_i[3];
        RotationMatrixRotatePoint(R_c_imu, t_imu_i, t_cam_i);
        t_cam_i[0]+=t_c_imu[0];
        t_cam_i[1]+=t_c_imu[1];
        t_cam_i[2]+=t_c_imu[2];
        
        
        //计算残差
        residuals[0] = (t_cam_i[0]/t_cam_i[2] - T(pts_j_x)) * T(weight);
        residuals[1] = (t_cam_i[1]/t_cam_i[2] - T(pts_j_y)) * T(weight);
//        residuals[2] = (t_cam_i[2] - T(pts_j_z)) * T(weight);
        
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pts_j_z, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll,const double kf_pitch,const double kf_roll,const double weight)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError_reprojection, 2, 1, 3, 3, 3>(
                                                   new FourDOFWeightError_reprojection(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pts_j_z, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll, kf_pitch, kf_roll,weight)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pts_j_z;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    double kf_pitch, kf_roll;
    double weight;
    
};

//这里是 新的版本 新帧的3D点  投影到老帧的2D点
struct FourDOFWeightError_reprojection2
{
    FourDOFWeightError_reprojection2(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pts_j_z, double weight)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pts_j_z(pts_j_z), weight(weight){
//        weight = 100;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* r_c1_w2, const T* t_c1_w2, T* residuals) const
    {
        T t_w2_i[3];
        t_w2_i[0] = T(pts_i_x);
        t_w2_i[1] = T(pts_i_y);
        t_w2_i[2] = T(pts_i_z);
        
        // euler to rotation
        T R_c1_w2[9];
        YawPitchRollToRotationMatrix(r_c1_w2[0], r_c1_w2[1], r_c1_w2[2], R_c1_w2);
        
        T t_c1_i[3];
        RotationMatrixRotatePoint(R_c1_w2, t_w2_i, t_c1_i);
        
        t_c1_i[0]+=t_c1_w2[0];
        t_c1_i[1]+=t_c1_w2[1];
        t_c1_i[2]+=t_c1_w2[2];
        
        
      
        
        //计算残差
        residuals[0] = (t_c1_i[0]/t_c1_i[2] - T(pts_j_x)) * T(weight);
        residuals[1] = (t_c1_i[1]/t_c1_i[2] - T(pts_j_y)) * T(weight);
//        residuals[2] = (t_cam_i[2] - T(pts_j_z)) * T(weight);
        
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pts_j_z, const double weight)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError_reprojection2, 2, 3, 3>(
                                                   new FourDOFWeightError_reprojection2(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pts_j_z,weight)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pts_j_z;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    double kf_pitch, kf_roll;
    double weight;
    
};

//增加更多的投影帧 双向重投影误差
struct FourDOFWeightError_reprojection3
{
    FourDOFWeightError_reprojection3(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pts_j_z, double r_cn_c1_x, double r_cn_c1_y, double r_cn_c1_z, double t_cn_c1_x, double t_cn_c1_y,double t_cn_c1_z,double weight)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pts_j_z(pts_j_z), r_cn_c1_x(r_cn_c1_x), r_cn_c1_y(r_cn_c1_y), r_cn_c1_z(r_cn_c1_z), t_cn_c1_x(t_cn_c1_x), t_cn_c1_y(t_cn_c1_y), t_cn_c1_z(t_cn_c1_z), weight(weight){
//        weight = 20;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* r_c1_w2, const T* t_c1_w2, T* residuals) const
    {
        T t_w2_i[3];
        t_w2_i[0] = T(pts_i_x);
        t_w2_i[1] = T(pts_i_y);
        t_w2_i[2] = T(pts_i_z);
        
        // euler to rotation
        T R_c1_w2[9];
        YawPitchRollToRotationMatrix(r_c1_w2[0], r_c1_w2[1], r_c1_w2[2], R_c1_w2);
        
        T R_cn_c1[9];
        YawPitchRollToRotationMatrix(T(r_cn_c1_x), T(r_cn_c1_y), T(r_cn_c1_z), R_cn_c1);
        
        T t_cn_c1[3];
        t_cn_c1[0] = T(t_cn_c1_x);
        t_cn_c1[1] = T(t_cn_c1_y);
        t_cn_c1[2] = T(t_cn_c1_z);
        
        T r_cn_w2[9];
        RotationMatrixRotateMatrix(R_cn_c1, R_c1_w2, r_cn_w2);
        T t_cn_w2[3];
        RotationMatrixRotatePoint(R_cn_c1, t_c1_w2, t_cn_w2);
        t_cn_w2[0]+=t_cn_c1[0];
        t_cn_w2[1]+=t_cn_c1[1];
        t_cn_w2[2]+=t_cn_c1[2];
        
        
        T t_cn_i[3];
        RotationMatrixRotatePoint(r_cn_w2, t_w2_i, t_cn_i);
        t_cn_i[0]+=t_cn_w2[0];
        t_cn_i[1]+=t_cn_w2[1];
        t_cn_i[2]+=t_cn_w2[2];
        
        
      
        
        
        //计算残差
        residuals[0] = (t_cn_i[0]/t_cn_i[2] - T(pts_j_x)) * T(weight);
        residuals[1] = (t_cn_i[1]/t_cn_i[2] - T(pts_j_y)) * T(weight);
//        residuals[2] = (t_cam_i[2] - T(pts_j_z)) * T(weight);
        
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x,const double pts_i_y,const double pts_i_z,const double pts_j_x,const double pts_j_y,const double pts_j_z,const double r_cn_c1_x,const double r_cn_c1_y,const double r_cn_c1_z,const double t_cn_c1_x,const double t_cn_c1_y,const double t_cn_c1_z,const double weight)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError_reprojection3, 2,  3, 3>(
                                                   new FourDOFWeightError_reprojection3(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pts_j_z, r_cn_c1_x, r_cn_c1_y, r_cn_c1_z, t_cn_c1_x, t_cn_c1_y, t_cn_c1_z,weight)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pts_j_z;
    double r_cn_c1_x, r_cn_c1_y, r_cn_c1_z, t_cn_c1_x, t_cn_c1_y, t_cn_c1_z ;
    double weight;
    
};

struct FourDOFWeightError_reprojection_inverse_Wmain_Wcur
{
    FourDOFWeightError_reprojection_inverse_Wmain_Wcur(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pts_j_z, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll,  double kf_pitch, double kf_roll, double weight)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pts_j_z(pts_j_z), pose_Tic_cur_t_x(pose_Tic_cur_t_x), pose_Tic_cur_t_y(pose_Tic_cur_t_y), pose_Tic_cur_t_z(pose_Tic_cur_t_z), pose_Tic_cur_r_yaw(pose_Tic_cur_r_yaw), pose_Tic_cur_r_pitch(pose_Tic_cur_r_pitch), pose_Tic_cur_r_roll(pose_Tic_cur_r_roll),  kf_pitch(kf_pitch), kf_roll(kf_roll), weight(weight){
//        weight = 20;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const yaw_kf, const T* t_kf, const T* r_w2_w1, const T* t_w2_w1, T* residuals) const
    {
        T t_w2_i[3];
        t_w2_i[0] = T(pts_i_x);
        t_w2_i[1] = T(pts_i_y);
        t_w2_i[2] = T(pts_i_z);
        
        // euler to rotation
        T R_w2_w1[9];
        YawPitchRollToRotationMatrix(r_w2_w1[0], r_w2_w1[1], r_w2_w1[2], R_w2_w1);
        T R_w1_w2[9];
        RotationMatrixTranspose(R_w2_w1, R_w1_w2);
        T t_w1_w2[3];
        RotationMatrixRotateT(R_w1_w2, t_w2_w1, t_w1_w2);
        
        
        T t_w1_i[3];
        RotationMatrixRotatePoint(R_w1_w2, t_w2_i, t_w1_i);
        t_w1_i[0]+=t_w1_w2[0];
        t_w1_i[1]+=t_w1_w2[1];
        t_w1_i[2]+=t_w1_w2[2];
        
        
        T R_w1_imu[9];
        YawPitchRollToRotationMatrix(yaw_kf[0], T(kf_pitch), T(kf_roll), R_w1_imu);
        // rotation transpose
        T R_imu_w1[9];
        RotationMatrixTranspose(R_w1_imu, R_imu_w1);
        T t_imu_w1[3];
        RotationMatrixRotateT(R_imu_w1, t_kf, t_imu_w1);
        // rotation matrix rotate point
        T t_imu_i[3];
        RotationMatrixRotatePoint(R_imu_w1, t_w1_i, t_imu_i);
        t_imu_i[0]+=t_imu_w1[0];
        t_imu_i[1]+=t_imu_w1[1];
        t_imu_i[2]+=t_imu_w1[2];
        
        T R_imu_c[9];
        YawPitchRollToRotationMatrix(T(pose_Tic_cur_r_yaw), T(pose_Tic_cur_r_pitch), T(pose_Tic_cur_r_roll), R_imu_c);
        T t_imu_c[3];
        t_imu_c[0]=T(pose_Tic_cur_t_x);
        t_imu_c[1]=T(pose_Tic_cur_t_y);
        t_imu_c[2]=T(pose_Tic_cur_t_z);
        
        T R_c_imu[9];
        RotationMatrixTranspose(R_imu_c, R_c_imu);
        T t_c_imu[3];
        RotationMatrixRotateT(R_c_imu, t_imu_c, t_c_imu);
        // rotation matrix rotate point
        T t_cam_i[3];
        RotationMatrixRotatePoint(R_c_imu, t_imu_i, t_cam_i);
        t_cam_i[0]+=t_c_imu[0];
        t_cam_i[1]+=t_c_imu[1];
        t_cam_i[2]+=t_c_imu[2];
        
        
        //计算残差
        residuals[0] = (t_cam_i[0]/t_cam_i[2] - T(pts_j_x)) * T(weight);
        residuals[1] = (t_cam_i[1]/t_cam_i[2] - T(pts_j_y)) * T(weight);
//        residuals[2] = (t_cam_i[2] - T(pts_j_z)) * T(weight);
        
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pts_j_z, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll, const double kf_pitch, const double kf_roll,const double weight)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError_reprojection_inverse_Wmain_Wcur, 2, 1, 3, 3, 3>(
                                                   new FourDOFWeightError_reprojection_inverse_Wmain_Wcur(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pts_j_z, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll, kf_pitch, kf_roll,weight)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pts_j_z;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    double kf_pitch, kf_roll;
    double weight;
    
};



//计算相对位姿 一个4自由度，一个6自由度
struct FourSixDOFWeightError_reprojection
{
    FourSixDOFWeightError_reprojection(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll, double t_b_a_x , double t_b_a_y , double t_b_a_z , double r_b_a_yaw , double r_b_a_pitch , double r_b_a_roll, double weight)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pose_Tic_cur_t_x(pose_Tic_cur_t_x), pose_Tic_cur_t_y(pose_Tic_cur_t_y), pose_Tic_cur_t_z(pose_Tic_cur_t_z), pose_Tic_cur_r_yaw(pose_Tic_cur_r_yaw), pose_Tic_cur_r_pitch(pose_Tic_cur_r_pitch), pose_Tic_cur_r_roll(pose_Tic_cur_r_roll), t_b_a_x(t_b_a_x), t_b_a_y(t_b_a_y), t_b_a_z(t_b_a_z), r_b_a_yaw(r_b_a_yaw), r_b_a_pitch(r_b_a_pitch), r_b_a_roll(r_b_a_roll), weight(weight){
//        weight = 100;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* r_w_j, const T* t_w_j, T* residuals) const
    {
        T pts_w_i[3];
        pts_w_i[0] = T(pts_i_x);
        pts_w_i[1] = T(pts_i_y);
        pts_w_i[2] = T(pts_i_z);
        
        // euler to rotation
        T R_w_j[9];
        YawPitchRollToRotationMatrix(r_w_j[0], r_w_j[1], r_w_j[2], R_w_j);
        T T_w_j[3];
        T_w_j[0] = t_w_j[0];
        T_w_j[1] = t_w_j[1];
        T_w_j[2] = t_w_j[2];
        
        T r_a_b[9];
        YawPitchRollToRotationMatrix(T(r_b_a_yaw), T(r_b_a_pitch), T(r_b_a_roll), r_a_b);
        T T_a_b[3];
        T_a_b[0] = T(t_b_a_x);
        T_a_b[1] = T(t_b_a_y);
        T_a_b[2] = T(t_b_a_z);
        
        T R_w_camA[9];
        RotationMatrixRotateMatrix(R_w_j, r_a_b,R_w_camA);
        T t_w_camA[3];
        RotationMatrixRotatePoint(R_w_j, T_a_b, t_w_camA);
        t_w_camA[0]+=T_w_j[0];
        t_w_camA[1]+=T_w_j[1];
        t_w_camA[2]+=T_w_j[2];
        
        T R_imu_c[9];
        YawPitchRollToRotationMatrix(T(pose_Tic_cur_r_yaw), T(pose_Tic_cur_r_pitch), T(pose_Tic_cur_r_roll), R_imu_c);
        T t_imu_c[3];
        t_imu_c[0]=T(pose_Tic_cur_t_x);
        t_imu_c[1]=T(pose_Tic_cur_t_y);
        t_imu_c[2]=T(pose_Tic_cur_t_z);
        
        
        T R_w_camJ[9];
        RotationMatrixRotateMatrix(R_w_camA, R_imu_c,R_w_camJ);
        T t_w_camJ[3];
        RotationMatrixRotatePoint(R_w_camA, t_imu_c, t_w_camJ);
        t_w_camJ[0]+=t_w_camA[0];
        t_w_camJ[1]+=t_w_camA[1];
        t_w_camJ[2]+=t_w_camA[2];
        
        T R_camJ_w[9];
        RotationMatrixTranspose(R_w_camJ, R_camJ_w);
        T t_camJ_w[3];
        RotationMatrixRotateT(R_camJ_w, t_w_camJ, t_camJ_w);
        
        T pt_camJ_ptsI[3];
        RotationMatrixRotatePoint(R_camJ_w, pts_w_i, pt_camJ_ptsI);
        pt_camJ_ptsI[0]+=t_camJ_w[0];
        pt_camJ_ptsI[1]+=t_camJ_w[1];
        pt_camJ_ptsI[2]+=t_camJ_w[2];
             
        
        //计算残差
        residuals[0] = (pt_camJ_ptsI[0]/pt_camJ_ptsI[2] - T(pts_j_x)) * T(weight);
        residuals[1] = (pt_camJ_ptsI[1]/pt_camJ_ptsI[2] - T(pts_j_y)) * T(weight);

        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll,const double t_b_a_x ,const double t_b_a_y ,const double t_b_a_z ,const double r_b_a_yaw ,const double r_b_a_pitch ,const double r_b_a_roll,const double weight)
    {
        return (new ceres::AutoDiffCostFunction<
                FourSixDOFWeightError_reprojection, 2, 3, 3>(
                new FourSixDOFWeightError_reprojection(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll,t_b_a_x, t_b_a_y, t_b_a_z, r_b_a_yaw, r_b_a_pitch, r_b_a_roll,weight)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    double r_b_a_yaw, r_b_a_pitch, r_b_a_roll, t_b_a_x, t_b_a_y, t_b_a_z;
    
    double weight;
    
};

//内部回环 粗糙的相对位姿
struct FourSixDOFWeightError_reprojection2
{
    FourSixDOFWeightError_reprojection2(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll,double weight)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pose_Tic_cur_t_x(pose_Tic_cur_t_x), pose_Tic_cur_t_y(pose_Tic_cur_t_y), pose_Tic_cur_t_z(pose_Tic_cur_t_z), pose_Tic_cur_r_yaw(pose_Tic_cur_r_yaw), pose_Tic_cur_r_pitch(pose_Tic_cur_r_pitch), pose_Tic_cur_r_roll(pose_Tic_cur_r_roll), weight(weight){
//        weight = 100;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* r_w_j, const T* t_w_j, T* residuals) const
    {
        T pts_w_i[3];
        pts_w_i[0] = T(pts_i_x);
        pts_w_i[1] = T(pts_i_y);
        pts_w_i[2] = T(pts_i_z);
        
        // euler to rotation
        T R_w_j[9];
        YawPitchRollToRotationMatrix(r_w_j[0], r_w_j[1], r_w_j[2], R_w_j);
        T T_w_j[3];
        T_w_j[0] = t_w_j[0];
        T_w_j[1] = t_w_j[1];
        T_w_j[2] = t_w_j[2];
        
     
        
        T R_imu_c[9];
        YawPitchRollToRotationMatrix(T(pose_Tic_cur_r_yaw), T(pose_Tic_cur_r_pitch), T(pose_Tic_cur_r_roll), R_imu_c);
        T t_imu_c[3];
        t_imu_c[0]=T(pose_Tic_cur_t_x);
        t_imu_c[1]=T(pose_Tic_cur_t_y);
        t_imu_c[2]=T(pose_Tic_cur_t_z);
        
        
        T R_w_camJ[9];
        RotationMatrixRotateMatrix(R_w_j, R_imu_c,R_w_camJ);
        T t_w_camJ[3];
        RotationMatrixRotatePoint(R_w_j, t_imu_c, t_w_camJ);
        t_w_camJ[0]+=T_w_j[0];
        t_w_camJ[1]+=T_w_j[1];
        t_w_camJ[2]+=T_w_j[2];
        
        T R_camJ_w[9];
        RotationMatrixTranspose(R_w_camJ, R_camJ_w);
        T t_camJ_w[3];
        RotationMatrixRotateT(R_camJ_w, t_w_camJ, t_camJ_w);
        
        T pt_camJ_ptsI[3];
        RotationMatrixRotatePoint(R_camJ_w, pts_w_i, pt_camJ_ptsI);
        pt_camJ_ptsI[0]+=t_camJ_w[0];
        pt_camJ_ptsI[1]+=t_camJ_w[1];
        pt_camJ_ptsI[2]+=t_camJ_w[2];
             
        
        //计算残差
        residuals[0] = (pt_camJ_ptsI[0]/pt_camJ_ptsI[2] - T(pts_j_x)) * T(weight);
        residuals[1] = (pt_camJ_ptsI[1]/pt_camJ_ptsI[2] - T(pts_j_y)) * T(weight);

//        cout<<"误差："<<residuals[0]<<" ,"<<residuals[1]<<endl;
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll,const double weight)
    {
        return (new ceres::AutoDiffCostFunction<
                FourSixDOFWeightError_reprojection2, 2, 3, 3>(
                new FourSixDOFWeightError_reprojection2(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll,weight)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    
    double weight;
    
};
//全局 地图融合，第一步算粗糙相对位姿
struct FourSixDOFWeightError_reprojection_server
{
    FourSixDOFWeightError_reprojection_server(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll,double weight)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pose_Tic_cur_t_x(pose_Tic_cur_t_x), pose_Tic_cur_t_y(pose_Tic_cur_t_y), pose_Tic_cur_t_z(pose_Tic_cur_t_z), pose_Tic_cur_r_yaw(pose_Tic_cur_r_yaw), pose_Tic_cur_r_pitch(pose_Tic_cur_r_pitch), pose_Tic_cur_r_roll(pose_Tic_cur_r_roll), weight(weight){
//        weight = 100;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* r_w_j, const T* t_w_j, T* residuals) const
    {
        T pts_w_i[3];
        pts_w_i[0] = T(pts_i_x);
        pts_w_i[1] = T(pts_i_y);
        pts_w_i[2] = T(pts_i_z);
        
      
        
        // euler to rotation
        T R_w_j[9];
        YawPitchRollToRotationMatrix(r_w_j[0], r_w_j[1], r_w_j[2], R_w_j);
        T T_w_j[3];
        T_w_j[0] = t_w_j[0];
        T_w_j[1] = t_w_j[1];
        T_w_j[2] = t_w_j[2];
        
     
        
        T R_imu_c[9];
        YawPitchRollToRotationMatrix(T(pose_Tic_cur_r_yaw), T(pose_Tic_cur_r_pitch), T(pose_Tic_cur_r_roll), R_imu_c);
        T t_imu_c[3];
        t_imu_c[0]=T(pose_Tic_cur_t_x);
        t_imu_c[1]=T(pose_Tic_cur_t_y);
        t_imu_c[2]=T(pose_Tic_cur_t_z);
        
        
        T R_w_camJ[9];
        RotationMatrixRotateMatrix(R_w_j, R_imu_c,R_w_camJ);
        T t_w_camJ[3];
        RotationMatrixRotatePoint(R_w_j, t_imu_c, t_w_camJ);
        t_w_camJ[0]+=T_w_j[0];
        t_w_camJ[1]+=T_w_j[1];
        t_w_camJ[2]+=T_w_j[2];
        
        T R_camJ_w[9];
        RotationMatrixTranspose(R_w_camJ, R_camJ_w);
        T t_camJ_w[3];
        RotationMatrixRotateT(R_camJ_w, t_w_camJ, t_camJ_w);
        
        T pt_camJ_ptsI[3];
        RotationMatrixRotatePoint(R_camJ_w, pts_w_i, pt_camJ_ptsI);
        pt_camJ_ptsI[0]+=t_camJ_w[0];
        pt_camJ_ptsI[1]+=t_camJ_w[1];
        pt_camJ_ptsI[2]+=t_camJ_w[2];
             
        
        //计算残差
        residuals[0] = (pt_camJ_ptsI[0]/pt_camJ_ptsI[2] - T(pts_j_x)) * T(weight);
        residuals[1] = (pt_camJ_ptsI[1]/pt_camJ_ptsI[2] - T(pts_j_y)) * T(weight);

//        cout<<"误差："<<residuals[0]<<" ,"<<residuals[1]<<endl;
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll,  const double weight)
    {
        return (new ceres::AutoDiffCostFunction<
                FourSixDOFWeightError_reprojection_server, 2, 3, 3>(
                new FourSixDOFWeightError_reprojection_server(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll,weight)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    
    double weight;
    
};

struct FourSixDOFWeightError_reprojection_3
{
    FourSixDOFWeightError_reprojection_3(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll, double r_b_a_pitch , double r_b_a_roll, double weight)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pose_Tic_cur_t_x(pose_Tic_cur_t_x), pose_Tic_cur_t_y(pose_Tic_cur_t_y), pose_Tic_cur_t_z(pose_Tic_cur_t_z), pose_Tic_cur_r_yaw(pose_Tic_cur_r_yaw), pose_Tic_cur_r_pitch(pose_Tic_cur_r_pitch), pose_Tic_cur_r_roll(pose_Tic_cur_r_roll),  r_b_a_pitch(r_b_a_pitch), r_b_a_roll(r_b_a_roll), weight(weight){
//        weight = 100;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* r_w_j, const T* t_w_j,const T* r_b_a_xx, const T* t_b_a_xx, T* residuals) const
    {
        T pts_w_i[3];
        pts_w_i[0] = T(pts_i_x);
        pts_w_i[1] = T(pts_i_y);
        pts_w_i[2] = T(pts_i_z);
        
        // euler to rotation
        T R_w_j[9];
        YawPitchRollToRotationMatrix(r_w_j[0], r_w_j[1], r_w_j[2], R_w_j);
        T T_w_j[3];
        T_w_j[0] = t_w_j[0];
        T_w_j[1] = t_w_j[1];
        T_w_j[2] = t_w_j[2];
        
        T r_a_b[9];
        YawPitchRollToRotationMatrix(r_b_a_xx[0], T(r_b_a_pitch), T(r_b_a_roll), r_a_b);
        T T_a_b[3];
        T_a_b[0] = t_b_a_xx[0];
        T_a_b[1] = t_b_a_xx[1];
        T_a_b[2] = t_b_a_xx[2];
        
        T R_w_camA[9];
        RotationMatrixRotateMatrix(R_w_j, r_a_b,R_w_camA);
        T t_w_camA[3];
        RotationMatrixRotatePoint(R_w_j, T_a_b, t_w_camA);
        t_w_camA[0]+=T_w_j[0];
        t_w_camA[1]+=T_w_j[1];
        t_w_camA[2]+=T_w_j[2];
        
        T R_imu_c[9];
        YawPitchRollToRotationMatrix(T(pose_Tic_cur_r_yaw), T(pose_Tic_cur_r_pitch), T(pose_Tic_cur_r_roll), R_imu_c);
        T t_imu_c[3];
        t_imu_c[0]=T(pose_Tic_cur_t_x);
        t_imu_c[1]=T(pose_Tic_cur_t_y);
        t_imu_c[2]=T(pose_Tic_cur_t_z);
        
        
        T R_w_camJ[9];
        RotationMatrixRotateMatrix(R_w_camA, R_imu_c,R_w_camJ);
        T t_w_camJ[3];
        RotationMatrixRotatePoint(R_w_camA, t_imu_c, t_w_camJ);
        t_w_camJ[0]+=t_w_camA[0];
        t_w_camJ[1]+=t_w_camA[1];
        t_w_camJ[2]+=t_w_camA[2];
        
        T R_camJ_w[9];
        RotationMatrixTranspose(R_w_camJ, R_camJ_w);
        T t_camJ_w[3];
        RotationMatrixRotateT(R_camJ_w, t_w_camJ, t_camJ_w);
        
        T pt_camJ_ptsI[3];
        RotationMatrixRotatePoint(R_camJ_w, pts_w_i, pt_camJ_ptsI);
        pt_camJ_ptsI[0]+=t_camJ_w[0];
        pt_camJ_ptsI[1]+=t_camJ_w[1];
        pt_camJ_ptsI[2]+=t_camJ_w[2];
             
        
        //计算残差
        residuals[0] = (pt_camJ_ptsI[0]/pt_camJ_ptsI[2] - T(pts_j_x)) * T(weight);
        residuals[1] = (pt_camJ_ptsI[1]/pt_camJ_ptsI[2] - T(pts_j_y)) * T(weight);

        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll ,const double r_b_a_pitch ,const double r_b_a_roll,const double weight)
    {
        return (new ceres::AutoDiffCostFunction<
                FourSixDOFWeightError_reprojection_3, 2, 3, 3,1,3>(
                new FourSixDOFWeightError_reprojection_3(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll, r_b_a_pitch, r_b_a_roll,weight)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    double r_b_a_pitch, r_b_a_roll;
    
    double weight;
    
};

/**
//FourSixDOFWeightError_reprojection2 和这个一样 只是权重不一样
struct FourSixDOFWeightError_reprojection4
{
    FourSixDOFWeightError_reprojection4(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pose_Tic_cur_t_x(pose_Tic_cur_t_x), pose_Tic_cur_t_y(pose_Tic_cur_t_y), pose_Tic_cur_t_z(pose_Tic_cur_t_z), pose_Tic_cur_r_yaw(pose_Tic_cur_r_yaw), pose_Tic_cur_r_pitch(pose_Tic_cur_r_pitch), pose_Tic_cur_r_roll(pose_Tic_cur_r_roll){
        weight = 100;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* r_w_j, const T* t_w_j, T* residuals) const
    {
        T pts_w_i[3];
        pts_w_i[0] = T(pts_i_x);
        pts_w_i[1] = T(pts_i_y);
        pts_w_i[2] = T(pts_i_z);
        
        // euler to rotation
        T R_w_j[9];
        YawPitchRollToRotationMatrix(r_w_j[0], r_w_j[1], r_w_j[2], R_w_j);
        T T_w_j[3];
        T_w_j[0] = t_w_j[0];
        T_w_j[1] = t_w_j[1];
        T_w_j[2] = t_w_j[2];
        
     
        
        T R_imu_c[9];
        YawPitchRollToRotationMatrix(T(pose_Tic_cur_r_yaw), T(pose_Tic_cur_r_pitch), T(pose_Tic_cur_r_roll), R_imu_c);
        T t_imu_c[3];
        t_imu_c[0]=T(pose_Tic_cur_t_x);
        t_imu_c[1]=T(pose_Tic_cur_t_y);
        t_imu_c[2]=T(pose_Tic_cur_t_z);
        
        
        T R_w_camJ[9];
        RotationMatrixRotateMatrix(R_w_j, R_imu_c,R_w_camJ);
        T t_w_camJ[3];
        RotationMatrixRotatePoint(R_w_j, t_imu_c, t_w_camJ);
        t_w_camJ[0]+=T_w_j[0];
        t_w_camJ[1]+=T_w_j[1];
        t_w_camJ[2]+=T_w_j[2];
        
        T R_camJ_w[9];
        RotationMatrixTranspose(R_w_camJ, R_camJ_w);
        T t_camJ_w[3];
        RotationMatrixRotateT(R_camJ_w, t_w_camJ, t_camJ_w);
        
        T pt_camJ_ptsI[3];
        RotationMatrixRotatePoint(R_camJ_w, pts_w_i, pt_camJ_ptsI);
        pt_camJ_ptsI[0]+=t_camJ_w[0];
        pt_camJ_ptsI[1]+=t_camJ_w[1];
        pt_camJ_ptsI[2]+=t_camJ_w[2];
             
        
        //计算残差
        residuals[0] = (pt_camJ_ptsI[0]/pt_camJ_ptsI[2] - T(pts_j_x)) * T(weight);
        residuals[1] = (pt_camJ_ptsI[1]/pt_camJ_ptsI[2] - T(pts_j_y)) * T(weight);

        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll)
    {
        return (new ceres::AutoDiffCostFunction<
                FourSixDOFWeightError_reprojection4, 2, 3, 3>(
                new FourSixDOFWeightError_reprojection4(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    
    double weight;
    
};
 */

struct FourSixDOFError
{
    FourSixDOFError(double t_b_a_x , double t_b_a_y , double t_b_a_z , double r_b_a_yaw ,double weight)
    :t_b_a_x(t_b_a_x), t_b_a_y(t_b_a_y), t_b_a_z(t_b_a_z), r_b_a_yaw(r_b_a_yaw), weight(weight){}
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, T* residuals) const
    {
        
//        //计算残差
        residuals[0] = (ti[0] - T(t_b_a_x))*weight;
        residuals[1] = (ti[1] - T(t_b_a_y))*weight;
        residuals[2] = (ti[2] - T(t_b_a_z))*weight;
        residuals[3] = NormalizeAngle(yaw_i[0] - T(r_b_a_yaw))*weight;
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_b_a_x ,const double t_b_a_y ,const double t_b_a_z ,const double r_b_a_yaw ,double weight)
    {
        return (new ceres::AutoDiffCostFunction<
                FourSixDOFError, 4, 1, 3>(
                                             new FourSixDOFError(t_b_a_x, t_b_a_y, t_b_a_z, r_b_a_yaw,weight)));
    }
    
    double r_b_a_yaw,  t_b_a_x, t_b_a_y, t_b_a_z;
    double weight;
    
};


struct FourSixDOFWeightError_reprojection_another
{
    FourSixDOFWeightError_reprojection_another(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll, double weight)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pose_Tic_cur_t_x(pose_Tic_cur_t_x), pose_Tic_cur_t_y(pose_Tic_cur_t_y), pose_Tic_cur_t_z(pose_Tic_cur_t_z), pose_Tic_cur_r_yaw(pose_Tic_cur_r_yaw), pose_Tic_cur_r_pitch(pose_Tic_cur_r_pitch), pose_Tic_cur_r_roll(pose_Tic_cur_r_roll), weight(weight){
//        weight = 100;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* r_w_j, const T* t_w_j, T* residuals) const
    {
        T pts_w_i[3];
        pts_w_i[0] = T(pts_i_x);
        pts_w_i[1] = T(pts_i_y);
        pts_w_i[2] = T(pts_i_z);
        
        // euler to rotation
        T R_w_j[9];
        YawPitchRollToRotationMatrix(r_w_j[0], r_w_j[1], r_w_j[2], R_w_j);
        T T_w_j[3];
        T_w_j[0] = t_w_j[0];
        T_w_j[1] = t_w_j[1];
        T_w_j[2] = t_w_j[2];
        
      
      
        
        T R_imu_c[9];
        YawPitchRollToRotationMatrix(T(pose_Tic_cur_r_yaw), T(pose_Tic_cur_r_pitch), T(pose_Tic_cur_r_roll), R_imu_c);
        T t_imu_c[3];
        t_imu_c[0]=T(pose_Tic_cur_t_x);
        t_imu_c[1]=T(pose_Tic_cur_t_y);
        t_imu_c[2]=T(pose_Tic_cur_t_z);
        
        
        T R_w_camJ[9];
        RotationMatrixRotateMatrix(R_w_j, R_imu_c,R_w_camJ);
        T t_w_camJ[3];
        RotationMatrixRotatePoint(R_w_j, t_imu_c, t_w_camJ);
        t_w_camJ[0]+=T_w_j[0];
        t_w_camJ[1]+=T_w_j[1];
        t_w_camJ[2]+=T_w_j[2];
        
        T R_camJ_w[9];
        RotationMatrixTranspose(R_w_camJ, R_camJ_w);
        T t_camJ_w[3];
        RotationMatrixRotateT(R_camJ_w, t_w_camJ, t_camJ_w);
        
        T pt_camJ_ptsI[3];
        RotationMatrixRotatePoint(R_camJ_w, pts_w_i, pt_camJ_ptsI);
        pt_camJ_ptsI[0]+=t_camJ_w[0];
        pt_camJ_ptsI[1]+=t_camJ_w[1];
        pt_camJ_ptsI[2]+=t_camJ_w[2];
             
        
        //计算残差
        residuals[0] = (pt_camJ_ptsI[0]/pt_camJ_ptsI[2] - T(pts_j_x)) * T(weight);
        residuals[1] = (pt_camJ_ptsI[1]/pt_camJ_ptsI[2] - T(pts_j_y)) * T(weight);

        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll, const double weight)
    {
        return (new ceres::AutoDiffCostFunction<
                FourSixDOFWeightError_reprojection_another, 2, 3, 3>(
                new FourSixDOFWeightError_reprojection_another(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll,weight)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
//    double r_b_a_yaw, r_b_a_pitch, r_b_a_roll, t_b_a_x, t_b_a_y, t_b_a_z;
    
    double weight;
    
};

/**
struct FourSixDOFWeightError_reprojection_another2
{
    FourSixDOFWeightError_reprojection_another2(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll,  double r_w_j_pitch,  double r_w_j_roll,  double r_w_i_pitch,  double r_w_i_roll)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pose_Tic_cur_t_x(pose_Tic_cur_t_x), pose_Tic_cur_t_y(pose_Tic_cur_t_y), pose_Tic_cur_t_z(pose_Tic_cur_t_z), pose_Tic_cur_r_yaw(pose_Tic_cur_r_yaw), pose_Tic_cur_r_pitch(pose_Tic_cur_r_pitch), pose_Tic_cur_r_roll(pose_Tic_cur_r_roll), r_w_j_pitch(r_w_j_pitch), r_w_j_roll(r_w_j_roll), r_w_i_pitch(r_w_i_pitch), r_w_i_roll(r_w_i_roll){
        weight = 20;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* r_w_j, const T* t_w_j, const T* r_w_i, const T* t_w_i, T* residuals) const
    {
        T pts_w_i[3];
        pts_w_i[0] = T(pts_i_x);
        pts_w_i[1] = T(pts_i_y);
        pts_w_i[2] = T(pts_i_z);
        
        // euler to rotation
        T R_w_i[9];
        YawPitchRollToRotationMatrix(r_w_i[0], T(r_w_i_pitch), T(r_w_i_roll), R_w_i);
        T T_w_i[3];
        T_w_i[0] = t_w_i[0];
        T_w_i[1] = t_w_i[1];
        T_w_i[2] = t_w_i[2];
        
        T R_imu_c[9];
        YawPitchRollToRotationMatrix(T(pose_Tic_cur_r_yaw), T(pose_Tic_cur_r_pitch), T(pose_Tic_cur_r_roll), R_imu_c);
        T t_imu_c[3];
        t_imu_c[0]=T(pose_Tic_cur_t_x);
        t_imu_c[1]=T(pose_Tic_cur_t_y);
        t_imu_c[2]=T(pose_Tic_cur_t_z);
        
        T R_w_camI[9];
        RotationMatrixRotateMatrix(R_w_i, R_imu_c,R_w_camI);
        T t_w_camI[3];
        RotationMatrixRotatePoint(R_w_i, t_imu_c, t_w_camI);
        t_w_camI[0]+=T_w_i[0];
        t_w_camI[1]+=T_w_i[1];
        t_w_camI[2]+=T_w_i[2];
        
        T pt_camI_ptsI[3];
        RotationMatrixRotatePoint(R_w_camI, pts_w_i, pt_camI_ptsI);
        pt_camI_ptsI[0]+=t_w_camI[0];
        pt_camI_ptsI[1]+=t_w_camI[1];
        pt_camI_ptsI[2]+=t_w_camI[2];
      
        
        
        
        // euler to rotation
        T R_w_j[9];
        YawPitchRollToRotationMatrix(r_w_j[0], T(r_w_j_pitch), T(r_w_j_roll), R_w_j);
        T T_w_j[3];
        T_w_j[0] = t_w_j[0];
        T_w_j[1] = t_w_j[1];
        T_w_j[2] = t_w_j[2];
        
        T R_w_camJ[9];
        RotationMatrixRotateMatrix(R_w_j, R_imu_c,R_w_camJ);
        T t_w_camJ[3];
        RotationMatrixRotatePoint(R_w_j, t_imu_c, t_w_camJ);
        t_w_camJ[0]+=T_w_j[0];
        t_w_camJ[1]+=T_w_j[1];
        t_w_camJ[2]+=T_w_j[2];
        
        T R_camJ_w[9];
        RotationMatrixTranspose(R_w_camJ, R_camJ_w);
        T t_camJ_w[3];
        RotationMatrixRotateT(R_camJ_w, t_w_camJ, t_camJ_w);
        
        T pt_camJ_ptsI[3];
        RotationMatrixRotatePoint(R_camJ_w, pt_camI_ptsI, pt_camJ_ptsI);
        pt_camJ_ptsI[0]+=t_camJ_w[0];
        pt_camJ_ptsI[1]+=t_camJ_w[1];
        pt_camJ_ptsI[2]+=t_camJ_w[2];
             
        
        //计算残差
        residuals[0] = (pt_camJ_ptsI[0]/pt_camJ_ptsI[2] - T(pts_j_x)) * T(weight);
        residuals[1] = (pt_camJ_ptsI[1]/pt_camJ_ptsI[2] - T(pts_j_y)) * T(weight);

        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll, const double r_w_j_pitch, const double r_w_j_roll, const double r_w_i_pitch, const double r_w_i_roll)
    {
        return (new ceres::AutoDiffCostFunction<
                FourSixDOFWeightError_reprojection_another2, 2, 1, 3, 1, 3>(
                new FourSixDOFWeightError_reprojection_another2(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll, r_w_j_pitch, r_w_j_roll, r_w_i_pitch, r_w_i_roll)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
//    double r_b_a_yaw, r_b_a_pitch, r_b_a_roll, t_b_a_x, t_b_a_y, t_b_a_z;
    
    double r_w_j_pitch, r_w_j_roll, r_w_i_pitch, r_w_i_roll;
    
    double weight;
    
};
 */
/**
struct FourDOFWeightError_my_t
{
    FourDOFWeightError_my_t(double t_x, double t_y, double t_z)
    :t_x(t_x), t_y(t_y), t_z(t_z){
        weight = 20;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], yaw_i[1], yaw_i[2], w_R_i);
        // rotation transpose
        T i_R_w[9];
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
        //计算残差
        residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
        residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
        residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError_my_t, 3, 3, 3, 3>(
                                                   new FourDOFWeightError_my_t(t_x, t_y, t_z)));
    }
    
    double t_x, t_y, t_z;
    double weight;
    
};
 */
 /**
struct FourDOFWeightError_my_r
{
    FourDOFWeightError_my_r(double relative_yaw, double relative_pitch, double relative_roll)
    :relative_yaw(relative_yaw), relative_pitch(relative_pitch), relative_roll(relative_roll){
        weight = 10;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const yaw_i, const T* yaw_j, T* residuals) const
    {
        
        residuals[0] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);
        residuals[1] = NormalizeAngle((yaw_j[1] - yaw_i[1] - T(relative_pitch))) * T(weight) / T(10.0);
        residuals[2] = NormalizeAngle((yaw_j[2] - yaw_i[2] - T(relative_roll))) * T(weight) / T(10.0);
        return true;
    }
    
    static ceres::CostFunction* Create(const double relative_yaw, const double relative_pitch, const double relative_roll)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError_my_r, 3, 3, 3>(
                                                   new FourDOFWeightError_my_r(relative_yaw, relative_pitch, relative_roll)));
    }
    
    double relative_yaw, relative_pitch, relative_roll;
    double weight;
    
};
  */
/**
struct FourDOFError_my
{
    FourDOFError_my(double t_x, double t_y, double t_z, double relative_yaw, double relative_pitch, double relative_roll)
    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), relative_pitch(relative_pitch), relative_roll(relative_roll){}
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], yaw_i[1], yaw_i[2], w_R_i);
        // rotation transpose
        T i_R_w[9];
        //求出旋转矩阵的转置
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
//        //计算残差
        residuals[0] = (t_i_ij[0] - T(t_x));
        residuals[1] = (t_i_ij[1] - T(t_y));
        residuals[2] = (t_i_ij[2] - T(t_z));
        residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));
        residuals[4] = NormalizeAngle(yaw_j[1] - yaw_i[1] - T(relative_pitch));
        residuals[5] = NormalizeAngle(yaw_j[2] - yaw_i[2] - T(relative_roll));
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double relative_pitch, const double relative_roll)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFError_my, 4, 3, 3, 3, 3>(
                                             new FourDOFError_my(t_x, t_y, t_z, relative_yaw, relative_pitch, relative_roll)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, relative_pitch, relative_roll;
    
};
 
 struct FourDOFWeightError_my
 {
     FourDOFWeightError_my(double t_x, double t_y, double t_z, double relative_yaw, double relative_pitch, double relative_roll)
     :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), relative_pitch(relative_pitch), relative_roll(relative_roll){
         weight = 10;
     }
     
     template <typename T>
 //    闭环边的残差计算
     bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
     {
         T t_w_ij[3];
         t_w_ij[0] = tj[0] - ti[0];
         t_w_ij[1] = tj[1] - ti[1];
         t_w_ij[2] = tj[2] - ti[2];
         
         // euler to rotation
         T w_R_i[9];
         YawPitchRollToRotationMatrix(yaw_i[0], yaw_i[1], yaw_i[2], w_R_i);
         // rotation transpose
         T i_R_w[9];
         RotationMatrixTranspose(w_R_i, i_R_w);
         // rotation matrix rotate point
         T t_i_ij[3];
         RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
         //计算残差
         residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
         residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
         residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
         residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);
         residuals[4] = NormalizeAngle((yaw_j[1] - yaw_i[1] - T(relative_pitch))) * T(weight) / T(10.0);
         residuals[5] = NormalizeAngle((yaw_j[2] - yaw_i[2] - T(relative_roll))) * T(weight) / T(10.0);
         return true;
     }
     
     static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                        const double relative_yaw, const double relative_pitch, const double relative_roll)
     {
         return (new ceres::AutoDiffCostFunction<
                 FourDOFWeightError_my, 4, 3, 3, 3, 3>(
                                                    new FourDOFWeightError_my(t_x, t_y, t_z, relative_yaw, relative_pitch, relative_roll)));
     }
     
     double t_x, t_y, t_z;
     double relative_yaw, relative_pitch, relative_roll;
     double weight;
     
 };
 */
#endif /* PoseGraph_hpp */
