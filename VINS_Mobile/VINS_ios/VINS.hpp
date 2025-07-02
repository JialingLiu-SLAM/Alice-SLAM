//
//  VINS.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef VINS_hpp
#define VINS_hpp

#include <stdio.h>
#include "feature_manager.hpp"
#include "utility.hpp"
#include "projection_facor.hpp"
#include "pose_local_parameterization.hpp"
#include "global_param.hpp"
#include <ceres/ceres.h>
#include "marginalization_factor.hpp"
#include "imu_factor.h"
#include "draw_result.hpp"
#include <opencv2/core/eigen.hpp>
#include "inital_sfm.hpp"
#include "initial_aligment.hpp"
#include "motion_estimator.hpp"

#include <ceres/rotation.h>
#include "feature_coder.h"
#include "priorMap_manager.hpp"

//#include "keyframe.h"

class FeatureTracker;

extern bool LOOP_CLOSURE;
extern bool LOOP_CLOSURE_SERVER;
extern bool LOOP_CLOSURE_SERVER_noLoop;//意思是回环不需要在前端触发 都是服务器通知

//存储闭环检测结果，包块闭环帧和匹配帧的位姿关系、匹配帧的ID、闭环帧的位姿、闭环帧中良好的匹配点以及匹配正良好匹配点的ID
struct RetriveData
{
    /* data */
    int old_index;
    int cur_index;
    double header;
    Eigen::Vector3d P_old;
    Eigen::Quaterniond Q_old;
    Eigen::Vector3d P_cur;
    Eigen::Quaterniond Q_cur;
    vector<cv::Point2f> measurements;
    vector<int> features_ids;
    bool use;
    Eigen::Vector3d relative_t;
    Eigen::Quaterniond relative_q;
    double relative_yaw;
    double loop_pose[7];
    
    
    
    //ljl
//    bool sendRelativeData_server;
    double relative_pitch;
    double relative_roll;
    int isRemove;//0为被移除掉 1为降低权重 2正常用
    
    vector<double* > loop_pose_all;
    vector<double> header_all;
    vector<vector<cv::Point2f> > measurements_all;
    vector<vector<int> > features_ids_all;
    vector<vector<Eigen::Vector3d>> point_clouds_all;
    vector<Eigen::Vector3d> P_old_all;
    vector<Eigen::Quaterniond> Q_old_all;
};


struct RetriveData_localMapping
{
    /* data */
    double header_cur;
    vector<cv::Point2f> measurements_cur_norm;
    vector<int> features_ids_cur;
    vector<Eigen::Vector3d> point_3d_old;
    double loop_pose_forSlideWindow[7];
};
//class FeatureManager;
//class FeaturePerId;
//class FeaturePerFrame;
//class ImageFrame;
class VINS
{
public:
    
    typedef IMUFactor IMUFactor_t;
    
    VINS();
    
    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };
    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };
    
    FeatureManager f_manager;//滑窗的特征点管理器
    MotionEstimator m_estimator;
    int frame_count;//表示滑动窗口中图像数据的个数
    
    Feature_localMap priorMap_f_manager;
    
    //相机到imu的外参
    Eigen::Matrix3d ric;
    Eigen::Vector3d tic;
    MarginalizationFlag  marginalization_flag;
    //old 矩阵形式 优化后赋值给他，即使外面其它线程在用算位姿 也不会耽误时间 同时访问
    Eigen::Vector3d Ps[10 * (WINDOW_SIZE + 1)];//里面有尺度信息，各个帧相对于b0帧的平移 世界坐标系
    Eigen::Vector3d Vs[10 * (WINDOW_SIZE + 1)];
    Eigen::Matrix3d Rs[10 * (WINDOW_SIZE + 1)];
    Eigen::Vector3d Bas[10 * (WINDOW_SIZE + 1)];
    Eigen::Vector3d Bgs[10 * (WINDOW_SIZE + 1)];//陀螺仪的偏置
    //new 向量形式 优化是用它
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];//存放的是滑动窗口第i帧的位姿 应该是相机坐标系下
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];//满足滑动窗口条件的特征点的深度
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];//相机到imu的外参
    
    //for loop closure
    RetriveData  retrive_pose_data,front_pose;//
    bool loop_enable;
    queue<RetriveData> retrive_pose_data_server;
    
    //画图用到
    vector<Eigen::Vector3f> correct_point_cloud;
    Eigen::Vector3f correct_Ps[WINDOW_SIZE];//纠正过的位姿
    Eigen::Matrix3f correct_Rs[WINDOW_SIZE];
    
    
    Eigen::Vector3d t_drift;//
    Eigen::Matrix3d r_drift;
    
    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;//它指的是先验矩阵对应的状态量X,被边缘化留下的先验信息
    vector<Eigen::Vector3f> point_cloud;
    
    int edge=0;
    int edge_single=0;
    double var_imu;//imu激励
    //计算滑窗内被track过的特征点的数量
    int feature_num;
    
    IntegrationBase *pre_integrations[10 * (WINDOW_SIZE + 1)];
    bool first_imu;
    Eigen::Vector3d acc_0, gyr_0;
    vector<double> dt_buf[10 * (WINDOW_SIZE + 1)];
    vector<Eigen::Vector3d> linear_acceleration_buf[10 * (WINDOW_SIZE + 1)];
    vector<Eigen::Vector3d> angular_velocity_buf[10 * (WINDOW_SIZE + 1)];
    Eigen::Matrix<double, 7, 1> IMU_linear[10 * (WINDOW_SIZE + 1)];
    Eigen::Matrix3d IMU_angular[10 * (WINDOW_SIZE + 1)];
    double Headers[10 * (WINDOW_SIZE + 1)];
    Eigen::Vector3d g;//重力加速度
    
    vector<Eigen::Vector3d> init_poses;//初始化位姿的平移部分
    double initial_timestamp;
    Eigen::Vector3d init_P;
    Eigen::Vector3d init_V;
    Eigen::Matrix3d init_R;
    
    SolverFlag solver_flag;//判断初始化是否完成, INITIAL未初始化
    Eigen::Matrix3d Rc[10 * (WINDOW_SIZE + 1)];
    
    //for initialization
    //存储所有的ImageFrame对象（每读取一帧图像就会构建ImageFrame对象）
    //ImageFrame对象中保存了图像帧的位姿，相应的预积分和图像特征点信息
    map<double, ImageFrame> all_image_frame;//所有的图像帧
    IntegrationBase *tmp_pre_integration;
    Eigen::Matrix3d back_R0;
    Eigen::Vector3d back_P0;
    //for falure detection
    bool failure_hand;
    bool failure_occur;//vins失败了
    Eigen::Matrix3d last_R, last_R_old;
    Eigen::Vector3d last_P, last_P_old;
    
    //for visulization
    DrawResult drawresult;
    cv::Mat image_show;
    cv::Mat imageAI;
    enum InitStatus
    {
        FAIL_IMU,
        FAIL_PARALLAX,
        FAIL_RELATIVE,
        FAIL_SFM,
        FAIL_PNP,
        FAIL_ALIGN,
        FAIL_CHECK,
        SUCC
    };
    InitStatus init_status;
    int parallax_num_view;
    int fail_times;
    int initProgress;
    double final_cost;
    double visual_cost;
    int visual_factor_num;
    
    void solve_ceres(int buf_num);
    
    void solveCalibration();
    void old2new();
    void new2old();
    void clearState();
    void setIMUModel();
    void setExtrinsic();
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    void processImage(map<int, Eigen::Vector3d> &image_msg, double header, int buf_num);
    void processImage(map<int, Eigen::Vector3d> &image_msg, double header, int buf_num, map<int, Eigen::Vector3d> &distorted_image);
    void processIMU(double t, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
    void changeState();
    bool solveInitial();
    bool relativePose(int camera_id, Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l);
    bool visualInitialAlign();
    bool failureDetection();
    void failureRecover();
    void reInit();
    void update_loop_correction();
    
    //ljl
    queue<vector<uchar>> send_status;
    queue<int> send_status_index;
    bool sendServer_relative;
    Eigen::Vector3d relative_t_sendServer;
    Eigen::Quaterniond relative_q_sendServer;
    double relative_yaw_sendServer;
    int relative_cur_index_sendServer=0;
    double relative_pitch_sendServer;
    double relative_roll_sendServer;
    int isRemove_sendServer;
    
    void solve_ceres2(int buf_num);
    void new2old_2();
    void setFeature_tracker(FeatureTracker* feature_tracker);
    FeatureTracker* feature_tracker;
    
//    存储找到的老帧的3d到当前关键帧的匹配关系
//    queue<double> q_header_cur;
//    queue<std::vector<cv::Point2f> > q_measurements_cur_norm_all;
//    queue<std::vector<Eigen::Vector3d>> q_point_3d_old_all;
//    queue<std::vector<int>> q_feature_id_cur_all;//如果为-1，表明这些点没有被跟踪上的
//    queue<double*> loop_pose_forSlideWindow;//这里只记录最小的那个，因为偏移最小
    std::mutex q_old_3d_mutex;
    
    queue<RetriveData_localMapping>  retrive_pose_data_localMapping;
    double loop_pose_forSlideWindow_update[7];
    
    
//    跟踪回环地图
    std::vector<cv::Point2f> measurements_cur_coarse_new;//像素坐标
    std::vector<Eigen::Vector3d> point_3d_old_new;
    std::vector<int> feature_id_cur_new;
    double header_forwkf;
    double para_Pose_priorMap[WINDOW_SIZE + 1][SIZE_POSE];
    Eigen::Vector3d t_drift_priorMap;//
    Eigen::Matrix3d r_drift_priorMap;
    bool isUpdate_rt_drift;
    int kf_id_inLocal;
    
//    void UpdateLocalKeyFrames(KeyFrame* oldKf,int curKf_id);
//    void UpdateLocalPoints();
//    vector<KeyFrame*> mvpLocalKeyFrames;
//    list<Feature_local> feature_local;
//    int SearchByProjection(KeyFrame* cur_kf);
    
    //ljl imu初始化用
//    void InertialOptimization(map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &g, Eigen::VectorXd &x);
    
};

template <typename T>
void toRotationMatrix(const T x, const T y, const T z, const T w, T R[9])
{
 

  const T tx  = T(2)*x;
  const T ty  = T(2)*y;
  const T tz  = T(2)*z;
  const T twx = tx*w;
  const T twy = ty*w;
  const T twz = tz*w;
  const T txx = tx*x;
  const T txy = ty*x;
  const T txz = tz*x;
  const T tyy = ty*y;
  const T tyz = tz*y;
  const T tzz = tz*z;

    R[0] = T(1)-(tyy+tzz);
    R[1] = txy-twz;
    R[2] = txz+twy;
    R[3] = txy+twz;
    R[4]= T(1)-(txx+tzz);
    R[5] = tyz-twx;
    R[6]= txz-twy;
    R[7]= tyz+twx;
    R[8] = T(1)-(txx+tyy);

};
template <typename T>
void RotationMatrixRotatePoint_vins(const T R[9], const T t[3], T r_t[3])
{
    r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
    r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
    r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
};
struct ProjectionFactor_replace
{
    ProjectionFactor_replace(double pts_i_x , double pts_i_y , double pts_i_z ,double pts_j_x , double pts_j_y , double pts_j_z )
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pts_j_z(pts_j_z){}
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const rt_i, const T* rt_ic, const T* inv_dep_i, T* residuals) const
    {
        T t_i[3];
        t_i[0] = rt_i[0];
        t_i[1] = rt_i[1];
        t_i[2] = rt_i[2];
        // euler to rotation
        T R_i[9];
        toRotationMatrix(rt_i[3],rt_i[4],rt_i[5],rt_i[6],R_i);
        
        T t_ic[3];
        t_ic[0] = rt_ic[0];
        t_ic[1] = rt_ic[1];
        t_ic[2] = rt_ic[2];
        T R_ic[9];
        toRotationMatrix(rt_ic[3],rt_ic[4],rt_ic[5],rt_ic[6],R_ic);
        
        T inv_dep_i_opti[1];
        inv_dep_i_opti[0]=inv_dep_i[0];
        
        T pts_w_i[3];
        pts_w_i[0] = T(pts_i_x);
        pts_w_i[1] = T(pts_i_y);
        pts_w_i[2] = T(pts_i_z);
        
      
        
        T pts_camera_i[3];
        pts_camera_i[0]= pts_w_i[0] / inv_dep_i_opti[0];
        pts_camera_i[1]= pts_w_i[1] / inv_dep_i_opti[0];
        pts_camera_i[2]= pts_w_i[2] / inv_dep_i_opti[0];
        
        T pts_imu_i[3];
        RotationMatrixRotatePoint_vins(R_ic, pts_camera_i, pts_imu_i);
        pts_imu_i[0]+=t_ic[0];
        pts_imu_i[1]+=t_ic[1];
        pts_imu_i[2]+=t_ic[2];
        
        T pts_w[3];
        RotationMatrixRotatePoint_vins(R_i, pts_imu_i, pts_w);
        pts_w[0]+=t_i[0];
        pts_w[1]+=t_i[1];
        pts_w[2]+=t_i[2];

//        //计算残差
        residuals[0] = (pts_w[0] - T(pts_j_x))*T(20);
        residuals[1] = (pts_w[1] - T(pts_j_y))*T(20);
        residuals[2] = (pts_w[2] - T(pts_j_z))*T(20);
        
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x ,const double pts_i_y ,const double pts_i_z ,const double pts_j_x ,const double pts_j_y ,const double pts_j_z )
    {
        return (new ceres::AutoDiffCostFunction<
                ProjectionFactor_replace, 3, 7, 7,1>(
                                             new ProjectionFactor_replace(pts_i_x ,  pts_i_y ,  pts_i_z , pts_j_x ,  pts_j_y ,  pts_j_z)));
    }
    
    double pts_i_x ,  pts_i_y ,  pts_i_z , pts_j_x ,  pts_j_y ,  pts_j_z;
    
};
template <typename T>
void RotationMatrixRotateMatrix_vins(const T R[9], const T r_2[9], T r_3[9])
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

template <typename T>
void RotationMatrixTranspose_vins(const T R[9], T inv_R[9])
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
struct ProjectionFactor_inMap
{
    ProjectionFactor_inMap(double pts_i_x , double pts_i_y , double pts_i_z ,double pts_j_x , double pts_j_y , double pts_j_z )
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pts_j_z(pts_j_z){}
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const rt_i, const T* rt_ic, T* residuals) const
    {
        T t_i[3];
        t_i[0] = rt_i[0];
        t_i[1] = rt_i[1];
        t_i[2] = rt_i[2];
        // euler to rotation
        T R_i[9];
        toRotationMatrix(rt_i[3],rt_i[4],rt_i[5],rt_i[6],R_i);
        
        T t_ic[3];
        t_ic[0] = rt_ic[0];
        t_ic[1] = rt_ic[1];
        t_ic[2] = rt_ic[2];
        T R_ic[9];
        toRotationMatrix(rt_ic[3],rt_ic[4],rt_ic[5],rt_ic[6],R_ic);
        
        T t_w_c[3];
        T R_w_c[9];
        RotationMatrixRotateMatrix_vins(R_i, R_ic, R_w_c);
        RotationMatrixRotatePoint_vins(R_i,t_ic,t_w_c);
        t_w_c[0]+=t_i[0];
        t_w_c[1]+=t_i[1];
        t_w_c[2]+=t_i[2];
        
        T t_c_w[3];
        T R_c_w[9];
        RotationMatrixTranspose_vins(R_w_c,R_c_w);
        RotationMatrixRotatePoint_vins(R_c_w,t_w_c,t_c_w);
        t_c_w[0] = -t_c_w[0];
        t_c_w[1] = -t_c_w[1];
        t_c_w[2] = -t_c_w[2];
        
        T pts_w_i[3];
        pts_w_i[0] = T(pts_i_x);
        pts_w_i[1] = T(pts_i_y);
        pts_w_i[2] = T(pts_i_z);
        
      
        
        
        T pts_c_i[3];
        RotationMatrixRotatePoint_vins(R_c_w, pts_w_i, pts_c_i);
        pts_c_i[0]+=t_c_w[0];
        pts_c_i[1]+=t_c_w[1];
        pts_c_i[2]+=t_c_w[2];
        
        if(pts_c_i[2]>-T(0.0000001) && pts_c_i[2]<T(0.0000001)){
            residuals[0] =T( 0);
            residuals[1] = T(0);
        }else{
            T pts_camera_i[3];
            pts_camera_i[0]= pts_c_i[0] / pts_c_i[2];
            pts_camera_i[1]= pts_c_i[1] / pts_c_i[2];
    //        //计算残差
            residuals[0] = (pts_camera_i[0] - T(pts_j_x))*T(20);
            residuals[1] = (pts_camera_i[1] - T(pts_j_y))*T(20);
        } 
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x ,const double pts_i_y ,const double pts_i_z ,const double pts_j_x ,const double pts_j_y ,const double pts_j_z )
    {
        return (new ceres::AutoDiffCostFunction<
                ProjectionFactor_inMap, 2, 7, 7>(
                                             new ProjectionFactor_inMap(pts_i_x ,  pts_i_y ,  pts_i_z , pts_j_x ,  pts_j_y ,  pts_j_z)));
    }
    
    double pts_i_x ,  pts_i_y ,  pts_i_z , pts_j_x ,  pts_j_y ,  pts_j_z;
    
};

template <typename T>
T NormalizeAngle_vins(const T& angle_degrees) {
    T two_pi(2.0 * 180);
    
    if (angle_degrees > T(0))
        return angle_degrees -
        two_pi * ceres::floor((angle_degrees + T(180)) / two_pi);
    else
        return angle_degrees +
        two_pi * ceres::floor((-angle_degrees + T(180)) / two_pi);
};
struct ProjectionFactor_inMap_pose
{
    ProjectionFactor_inMap_pose(double pts_i_x , double pts_i_y , double pts_i_z ,double pts_j_x , double pts_j_y , double pts_j_z,double pts_j_w )
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pts_j_z(pts_j_z), pts_j_w(pts_j_w){}
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const rt_i, T* residuals) const
    {
        T t_i[3];
        t_i[0] = rt_i[0];
        t_i[1] = rt_i[1];
        t_i[2] = rt_i[2];
        // euler to rotation
//        T R_i[9];
//        toRotationMatrix(rt_i[3],rt_i[4],rt_i[5],rt_i[6],R_i);
        
        T r_i[4];
        r_i[0] = rt_i[3];
        r_i[1] = rt_i[4];
        r_i[2] = rt_i[5];
        r_i[3] = rt_i[6];
        
        residuals[0] = (t_i[0] - T(pts_i_x))*T(20);
        residuals[1] = (t_i[1] - T(pts_i_y))*T(20);
        residuals[2] = (t_i[2] - T(pts_i_z))*T(20);
        residuals[3] = NormalizeAngle_vins(r_i[0]  - T(pts_j_x))*T(20);
        residuals[4] = NormalizeAngle_vins(r_i[1]  - T(pts_j_y))*T(20);
        residuals[5] = NormalizeAngle_vins(r_i[2]  - T(pts_j_z))*T(20);
        
        
//        T t_i_gt[3];
//        t_i_gt[0] = T(pts_i_x);
//        t_i_gt[1] = T(pts_i_y);
//        t_i_gt[2] = T(pts_i_z);
//        // euler to rotation
//        T R_i_gt[9];
//        toRotationMatrix(pts_j_x,pts_j_y,pts_j_z,pts_j_w,R_i_gt);
//
//
//
//        T t_i_w[3];
//        T R_i_w[9];
//        RotationMatrixTranspose_vins(R_i_gt,R_i_w);
//        RotationMatrixRotatePoint_vins(R_i_w,t_i_gt,t_i_w);
//        t_i_w[0] = -t_i_w[0];
//        t_i_w[1] = -t_i_w[1];
//        t_i_w[2] = -t_i_w[2];
//
//        T t_w_c[3];
//        T R_w_c[9];
//        RotationMatrixRotateMatrix_vins(R_i, R_i_w, R_w_c);
//        RotationMatrixRotatePoint_vins(R_i,t_i_w,t_w_c);
//        t_w_c[0]+=t_i[0];
//        t_w_c[1]+=t_i[1];
//        t_w_c[2]+=t_i[2];
//
//
//
//    //        //计算残差
//        residuals[0] = (t_w_c[0])*T(20);
//        residuals[1] = (t_w_c[1] )*T(20);
//        residuals[2] = (t_w_c[2] )*T(20);
//        residuals[3] = (pts_camera_i[1] - T(pts_j_y))*T(20);
//        residuals[4] = (pts_camera_i[1] - T(pts_j_y))*T(20);
//        residuals[5] = (pts_camera_i[1] - T(pts_j_y))*T(20);
        
       
        
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x ,const double pts_i_y ,const double pts_i_z ,const double pts_j_x ,const double pts_j_y ,const double pts_j_z,const double pts_j_w )
    {
        return (new ceres::AutoDiffCostFunction<
                ProjectionFactor_inMap_pose, 6, 7>(
                                             new ProjectionFactor_inMap_pose(pts_i_x ,  pts_i_y ,  pts_i_z , pts_j_x ,  pts_j_y ,  pts_j_z,pts_j_w)));
    }
    
    double pts_i_x ,  pts_i_y ,  pts_i_z , pts_j_x ,  pts_j_y ,  pts_j_z, pts_j_w;
    
};

template <typename T>
void RotationMatrixToYawPitchRoll_vins ( const T R[9], T yaw,  T pitch,  T roll )
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

struct ProjectionFactor_inMap_relativePose
{
    ProjectionFactor_inMap_relativePose(double pts_i_x , double pts_i_y , double pts_i_z ,double pts_j_x , double pts_j_y , double pts_j_z)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pts_j_z(pts_j_z){}
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const rt_i, const T* const rt_j,T* residuals) const
    {
        T t_i[3];
        t_i[0] = rt_i[0];
        t_i[1] = rt_i[1];
        t_i[2] = rt_i[2];
       
        T r_i[4];
        r_i[0] = rt_i[3];
        r_i[1] = rt_i[4];
        r_i[2] = rt_i[5];
        r_i[3] = rt_i[6];
        T R_i_gt[9];
        toRotationMatrix(r_i[0],r_i[1],r_i[2],r_i[3],R_i_gt);
        
        T t_j[3];
        t_j[0] = rt_j[0];
        t_j[1] = rt_j[1];
        t_j[2] = rt_j[2];
       
        T r_j[4];
        r_j[0] = rt_j[3];
        r_j[1] = rt_j[4];
        r_j[2] = rt_j[5];
        r_j[3] = rt_j[6];
        T R_j_gt[9];
        toRotationMatrix(r_j[0],r_j[1],r_j[2],r_j[3],R_j_gt);
        T R_j_gt_T[9];
        RotationMatrixTranspose_vins(R_j_gt,R_j_gt_T);
        
        T t_w_ij[3];
        t_w_ij[0] = t_i[0] - t_j[0];
        t_w_ij[1] = t_i[1] - t_j[1];
        t_w_ij[2] = t_i[2] - t_j[2];
        T t_i_w[3];
        RotationMatrixRotatePoint_vins(R_j_gt_T,t_w_ij,t_i_w);
        T R_w_c[9];
        RotationMatrixRotateMatrix_vins(R_j_gt_T, R_i_gt, R_w_c);
        T yaw_i_w1, pitch_i_w1, roll_i_w1;
        RotationMatrixToYawPitchRoll_vins(R_w_c,yaw_i_w1, pitch_i_w1, roll_i_w1);
        cout<<"yaw_i_w1"<<yaw_i_w1<<endl;
        
        residuals[0] = (t_i_w[0] - T(pts_i_x))*T(20);
        residuals[1] = (t_i_w[1] - T(pts_i_y))*T(20);
        residuals[2] = (t_i_w[2] - T(pts_i_z))*T(20);
        residuals[3] = NormalizeAngle_vins(yaw_i_w1  - T(pts_j_x))*T(20);
        residuals[4] = NormalizeAngle_vins(pitch_i_w1  - T(pts_j_y))*T(20);
        residuals[5] = NormalizeAngle_vins(roll_i_w1 - T(pts_j_z))*T(20);
        
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x ,const double pts_i_y ,const double pts_i_z ,const double pts_j_x ,const double pts_j_y ,const double pts_j_z)
    {
        return (new ceres::AutoDiffCostFunction<
                ProjectionFactor_inMap_relativePose, 6, 7,7>(
                                             new ProjectionFactor_inMap_relativePose(pts_i_x ,  pts_i_y ,  pts_i_z , pts_j_x ,  pts_j_y ,pts_j_z)));
    }
    
    double pts_i_x ,  pts_i_y ,  pts_i_z , pts_j_x ,  pts_j_y ,  pts_j_z;
    
};
#endif /* VINS_hpp */
