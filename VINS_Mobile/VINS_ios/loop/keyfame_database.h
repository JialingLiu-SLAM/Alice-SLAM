//
//  keyfame_database.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2017/5/2.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//
#include "keyframe.h"
#ifndef keyfame_database_hpp
#define keyfame_database_hpp

#include <stdio.h>
#include <vector>
#include <list>

#include <assert.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "utility.hpp"

//for save keyframe result
struct KEYFRAME_DATA
{
    double header;
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
};

class KeyFrameDatabase
{
public:
    KeyFrameDatabase();
    void add(KeyFrame* pKF);
    void resample(vector<int> &erase_index);
    void erase(KeyFrame* pKF);
    int size();
    void optimize4DoFLoopPoseGraph(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r);
    void optimize4DoFLoopPoseGraph_my1(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r);
    KeyFrame* getKeyframe(int index);
    KeyFrame* getLastKeyframe();
    KeyFrame* getLastKeyframe(int last_index);
    KeyFrame*  getLastKeyframe_index(int last_index);
    KeyFrame* getLastUncheckKeyframe();
    void updateVisualization();
    void addLoop(int loop_index);
    
    //画图用的
    vector<Eigen::Vector3f> refine_path;//位姿
    vector<double> path_time;//实验用
    vector<Eigen::Quaterniond> refine_r;//实验用
    vector<KEYFRAME_DATA> all_keyframes;//没用上
    vector<int> segment_indexs;
    int max_seg_index, cur_seg_index;//最大序列号（只能5个图像序列，mobile没做这种设置），  当前的图像序列
    
    //ljl
    //存储从服务端接收的数据
    queue<vector<Eigen::Vector3d>> t_global;
    queue<vector<Eigen::Matrix3d>> r_global;
    queue<int> loopKF_index;
    queue<int> curKF_loop_index;
    bool start_global_optimization;
    queue<int> lastKF_index;//检测回环更新位姿偏移的 最后一个帧
    
    
    //存储从服务器接收的多个地图融合优化的数据
    queue<vector<Eigen::Vector3d>> t_global_multiClient;
    queue<vector<Eigen::Matrix3d>> r_global_multiClient;
    queue<vector<int>> kf_id_hasComPlace_withOtherMap;
    queue<int> loopKF_index_multiClient;
    queue<int> curKF_loop_index_multiClient;
    int max_frame_num_global;
    double min_dis;
    bool start_global_optimization_multiClient;
//    queue<int> lastKF_index_multiClient;//检测回环更新位姿偏移的 最后一个帧 这个可以和上面那个共用
    void optimize4DoFLoopPoseGraph_server_multiClient(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r);
    //测试用的
    queue<vector<int>> special_kf_inOpti;
    std::mutex special_kf_mutex;
    //for 地图内部
    queue<vector<int> > special_kf_inOpti_intra;
    std::mutex special_kf_intra_mutex;
    
    
    void optimize4DoFLoopPoseGraph_server(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r);
    void optimize4DoFLoopPoseGraph_server2(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r);
    int getKFListSize();
    int earliest_loop_index;
    queue<int> earliest_queue;
    
private:
    list<KeyFrame*> keyFrameList;//存放了所有关键帧
    std::mutex mMutexkeyFrameList;
    
    Eigen::Vector3d t_drift;
    double yaw_drift;
    Eigen::Matrix3d r_drift;
    int max_frame_num;
    double total_length;
    Eigen::Vector3d last_P;
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
    RelativeRTError(double t_x, double t_y, double t_z, double q_w, double q_x, double q_y, double q_z)
    :t_x(t_x), t_y(t_y), t_z(t_z), q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z)
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
        residuals[0] = (t_i_ij[0] - T(t_x));
        residuals[1] = (t_i_ij[1] - T(t_y));
        residuals[2] = (t_i_ij[2] - T(t_z));
        
        T relative_q[4];
        relative_q[0] = T(q_w);
        relative_q[1] = T(q_x);
        relative_q[2] = T(q_y);
        relative_q[3] = T(q_z);
        
        T q_i_j[4];
        ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);
        
        T relative_q_inv[4];
        QuaternionInverse(relative_q, relative_q_inv);
        
        T error_q[4];
        ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);
        
        residuals[3] = T(2) * error_q[1];
        residuals[4] = T(2) * error_q[2];
        residuals[5] = T(2) * error_q[3];
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double q_w, const double q_x, const double q_y, const double q_z)
    {
        return (new ceres::AutoDiffCostFunction<
                RelativeRTError, 6, 4, 3, 4, 3>(
                                                new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z)));
    }
    
    double t_x, t_y, t_z, t_norm;
    double q_w, q_x, q_y, q_z;
    
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

struct FourDOFWeightError
{
    FourDOFWeightError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
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
                FourDOFWeightError, 4, 1, 3, 1, 3>(
                                                   new FourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
    double weight;
    
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
template <typename T>
void RotationMatrixRotateT(const T R[9], const T t[3], T r_t[3])
{
    r_t[0] = -R[0] * t[0] - R[1] * t[1] - R[2] * t[2];
    r_t[1] = -R[3] * t[0] - R[4] * t[1] - R[5] * t[2];
    r_t[2] = -R[6] * t[0] - R[7] * t[1] - R[8] * t[2];
};

//内部回环 粗糙的相对位姿
struct FourSixDOFWeightError_reprojection2
{
    FourSixDOFWeightError_reprojection2(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll)
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

//        cout<<"误差："<<residuals[0]<<" ,"<<residuals[1]<<endl;
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll)
    {
        return (new ceres::AutoDiffCostFunction<
                FourSixDOFWeightError_reprojection2, 2, 3, 3>(
                new FourSixDOFWeightError_reprojection2(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    
    double weight;
    
};
struct FourSixDOFError
{
    FourSixDOFError(double t_b_a_x , double t_b_a_y , double t_b_a_z , double r_b_a_yaw )
    :t_b_a_x(t_b_a_x), t_b_a_y(t_b_a_y), t_b_a_z(t_b_a_z), r_b_a_yaw(r_b_a_yaw){}
    
    template <typename T>
//    序列边的残差计算
    bool operator()(const T* const yaw_i, const T* ti, T* residuals) const
    {
        
//        //计算残差
        residuals[0] = (ti[0] - T(t_b_a_x));
        residuals[1] = (ti[1] - T(t_b_a_y));
        residuals[2] = (ti[2] - T(t_b_a_z));
        residuals[3] = NormalizeAngle(yaw_i[0] - T(r_b_a_yaw));
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_b_a_x ,const double t_b_a_y ,const double t_b_a_z ,const double r_b_a_yaw )
    {
        return (new ceres::AutoDiffCostFunction<
                FourSixDOFError, 4, 1, 3>(
                                             new FourSixDOFError(t_b_a_x, t_b_a_y, t_b_a_z, r_b_a_yaw)));
    }
    
    double r_b_a_yaw,  t_b_a_x, t_b_a_y, t_b_a_z;
    
};

struct FourSixDOFWeightError_reprojection_3
{
    FourSixDOFWeightError_reprojection_3(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll, double r_b_a_pitch , double r_b_a_roll)
    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pose_Tic_cur_t_x(pose_Tic_cur_t_x), pose_Tic_cur_t_y(pose_Tic_cur_t_y), pose_Tic_cur_t_z(pose_Tic_cur_t_z), pose_Tic_cur_r_yaw(pose_Tic_cur_r_yaw), pose_Tic_cur_r_pitch(pose_Tic_cur_r_pitch), pose_Tic_cur_r_roll(pose_Tic_cur_r_roll),  r_b_a_pitch(r_b_a_pitch), r_b_a_roll(r_b_a_roll){
        weight = 100;
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
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll ,const double r_b_a_pitch ,const double r_b_a_roll)
    {
        return (new ceres::AutoDiffCostFunction<
                FourSixDOFWeightError_reprojection_3, 2, 3, 3,1,3>(
                new FourSixDOFWeightError_reprojection_3(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll, r_b_a_pitch, r_b_a_roll)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    double r_b_a_pitch, r_b_a_roll;
    
    double weight;
    
};

//全局 地图融合，第一步算粗糙相对位姿
struct FourSixDOFWeightError_reprojection_server
{
    FourSixDOFWeightError_reprojection_server(double pts_i_x, double pts_i_y, double pts_i_z, double pts_j_x, double pts_j_y, double pose_Tic_cur_t_x, double pose_Tic_cur_t_y, double pose_Tic_cur_t_z, double pose_Tic_cur_r_yaw, double pose_Tic_cur_r_pitch,double pose_Tic_cur_r_roll)
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

//        cout<<"误差："<<residuals[0]<<" ,"<<residuals[1]<<endl;
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double pts_i_x, const double pts_i_y, const double pts_i_z, const double pts_j_x, const double pts_j_y, const double pose_Tic_cur_t_x, const double pose_Tic_cur_t_y, const double pose_Tic_cur_t_z, const double pose_Tic_cur_r_yaw, const double pose_Tic_cur_r_pitch, const double pose_Tic_cur_r_roll)
    {
        return (new ceres::AutoDiffCostFunction<
                FourSixDOFWeightError_reprojection_server, 2, 3, 3>(
                new FourSixDOFWeightError_reprojection_server(pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y, pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll)));
    }
    
    double pts_i_x, pts_i_y, pts_i_z, pts_j_x, pts_j_y;
    double pose_Tic_cur_t_x, pose_Tic_cur_t_y, pose_Tic_cur_t_z, pose_Tic_cur_r_yaw, pose_Tic_cur_r_pitch, pose_Tic_cur_r_roll ;
    
    double weight;
    
};
//template <typename T>
//void toRotationMatrix(const T x, const T y, const T z, const T w, T R[9])
//{
// 
//
//  const T tx  = T(2)*x;
//  const T ty  = T(2)*y;
//  const T tz  = T(2)*z;
//  const T twx = tx*w;
//  const T twy = ty*w;
//  const T twz = tz*w;
//  const T txx = tx*x;
//  const T txy = ty*x;
//  const T txz = tz*x;
//  const T tyy = ty*y;
//  const T tyz = tz*y;
//  const T tzz = tz*z;
//
//    R[0] = T(1)-(tyy+tzz);
//    R[1] = txy-twz;
//    R[2] = txz+twy;
//    R[3] = txy+twz;
//    R[4]= T(1)-(txx+tzz);
//    R[5] = tyz-twx;
//    R[6]= txz-twy;
//    R[7]= tyz+twx;
//    R[8] = T(1)-(txx+tyy);
//
//}
//struct ProjectionFactor_replace
//{
//    ProjectionFactor_replace(double pts_i_x , double pts_i_y , double pts_i_z ,double pts_j_x , double pts_j_y , double pts_j_z )
//    :pts_i_x(pts_i_x), pts_i_y(pts_i_y), pts_i_z(pts_i_z), pts_j_x(pts_j_x), pts_j_y(pts_j_y), pts_j_z(pts_j_z){}
//    
//    template <typename T>
////    序列边的残差计算
//    bool operator()(const T* const rt_i, const T* rt_ic, const T* inv_dep_i, T* residuals) const
//    {
//        T t_i[3];
//        t_i[0] = rt_i[0];
//        t_i[1] = rt_i[1];
//        t_i[2] = rt_i[2];
//        // euler to rotation        
//        T R_i[9];
//        toRotationMatrix(rt_i[3],rt_i[4],rt_i[5],rt_i[6],R_i);
//        
//        T t_ic[3];
//        t_ic[0] = rt_ic[0];
//        t_ic[1] = rt_ic[1];
//        t_ic[2] = rt_ic[2];
//        T R_ic[9];
//        toRotationMatrix(rt_ic[3],rt_ic[4],rt_ic[5],rt_ic[6],R_ic);
//        
//        T inv_dep_i_opti[1];
//        inv_dep_i_opti[0]=inv_dep_i[0];
//        
//        T pts_w_i[3];
//        pts_w_i[0] = T(pts_i_x);
//        pts_w_i[1] = T(pts_i_y);
//        pts_w_i[2] = T(pts_i_z);
//        
//      
//        
//        T pts_camera_i[3];
//        pts_camera_i[0]= pts_w_i[0] / inv_dep_i_opti[0];
//        pts_camera_i[1]= pts_w_i[1] / inv_dep_i_opti[0];
//        pts_camera_i[2]= pts_w_i[2] / inv_dep_i_opti[0];
//        
//        T pts_imu_i[3];
//        RotationMatrixRotatePoint(R_ic, pts_camera_i, pts_imu_i);
//        pts_imu_i[0]+=t_ic[0];
//        pts_imu_i[1]+=t_ic[1];
//        pts_imu_i[2]+=t_ic[2];
//        
//        T pts_w[3];
//        RotationMatrixRotatePoint(R_i, pts_imu_i, pts_w);
//        pts_w[0]+=t_i[0];
//        pts_w[1]+=t_i[1];
//        pts_w[2]+=t_i[2];
//
////        //计算残差
//        residuals[0] = (pts_w[0] - T(pts_j_x));
//        residuals[1] = (pts_w[1] - T(pts_j_y));
//        residuals[2] = (pts_w[2] - T(pts_j_z));
//        
//        
//        return true;
//    }
//    
//    static ceres::CostFunction* Create(const double pts_i_x ,const double pts_i_y ,const double pts_i_z ,const double pts_j_x ,const double pts_j_y ,const double pts_j_z )
//    {
//        return (new ceres::AutoDiffCostFunction<
//                ProjectionFactor_replace, 3, 7, 7,1>(
//                                             new ProjectionFactor_replace(pts_i_x ,  pts_i_y ,  pts_i_z , pts_j_x ,  pts_j_y ,  pts_j_z)));
//    }
//    
//    double pts_i_x ,  pts_i_y ,  pts_i_z , pts_j_x ,  pts_j_y ,  pts_j_z;
//    
//};
#endif /* keyfame_database_hpp */
