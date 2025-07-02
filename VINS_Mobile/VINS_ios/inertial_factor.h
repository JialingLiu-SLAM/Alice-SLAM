//
//  inertial_factor.h
//  VINS_ios
//
//  Created by 张剑华 on 2021/4/8.
//  Copyright © 2021 栗大人. All rights reserved.
//

#ifndef inertial_factor_h
#define inertial_factor_h



#include <stdio.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utility.hpp"
#include "initial_aligment.hpp"
#include <math.h>

//acc 低于10维，可以自动求导
struct WeightAccFactor
{
    WeightAccFactor(double acc_x, double acc_y, double acc_z)
    :acc_x(acc_x), acc_y(acc_y), acc_z(acc_z){
        weight = 1;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const acc_cal, T* residuals) const
    {
        //计算残差
        residuals[0] = (acc_cal[0] - T(acc_x)) * T(weight);
        residuals[1] = (acc_cal[1] - T(acc_y)) * T(weight);
        residuals[2] = (acc_cal[2] - T(acc_z)) * T(weight);
        return true;
    }
    
    static ceres::CostFunction* Create(const double acc_x, const double acc_y, const double acc_z)
    {
        return (new ceres::AutoDiffCostFunction<
                WeightAccFactor, 3, 3>(new WeightAccFactor(acc_x, acc_y, acc_z)));
    }
    
    double acc_x, acc_y, acc_z;
    double weight;
};



//gyro
struct WeightGyroFactor
{
    WeightGyroFactor(double gyro_x, double gyro_y, double gyro_z)
    :gyro_x(gyro_x), gyro_y(gyro_y), gyro_z(gyro_z){
        weight = 1;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const gyro_cal, T* residuals) const
    {
        //计算残差
        residuals[0] = (gyro_cal[0] - T(gyro_x)) * T(weight);
        residuals[1] = (gyro_cal[1] - T(gyro_y)) * T(weight);
        residuals[2] = (gyro_cal[2] - T(gyro_z)) * T(weight);
        return true;
    }
    
    static ceres::CostFunction* Create(const double gyro_x, const double gyro_y, const double gyro_z)
    {
        return (new ceres::AutoDiffCostFunction<
                WeightGyroFactor, 3, 3>(new WeightGyroFactor(gyro_x, gyro_y, gyro_z)));
    }
    
    double gyro_x, gyro_y, gyro_z;
    double weight;
    
};

//g
struct WeightGFactor
{
    WeightGFactor(double g_C0)
    :g_C0(g_C0){
        weight = 0.9;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const g_cal, T* residuals) const
    {
        //计算残差
        residuals[0] = (g_cal[0] *g_cal[0] +g_cal[1] *g_cal[1] +g_cal[2] *g_cal[2] - T(g_C0)) * T(weight);
      
        return true;
    }
    
    static ceres::CostFunction* Create(const double g_C0)
    {
        return (new ceres::AutoDiffCostFunction<
                WeightGFactor, 1, 3>(new WeightGFactor (g_C0)));
    }
    
    //g=R_w_c0*g_C0
    double g_C0;
    double weight;
    
};

//scale
struct WeightScaleFactor
{
    WeightScaleFactor(double scale_x)
    :scale_x(scale_x){
        weight = 1;
    }
    
    template <typename T>
//    闭环边的残差计算
    bool operator()(const T* const scale_cal, T* residuals) const
    {
        //计算残差
        residuals[0] = (exp(scale_cal[0]) - T(scale_x)) * T(weight);
//        cout<<"测试 scale有没有优化："<<residuals[0]<<" ,"<<exp(scale_cal[0])<<" ,"<<T(scale_x)<<endl;
        return true;
    }
    
    static ceres::CostFunction* Create(const double scale_x)
    {
        return (new ceres::AutoDiffCostFunction<
                WeightScaleFactor, 1, 1>(new WeightScaleFactor (scale_x)));
    }
    
    //scale=exp(scale_x)
    double scale_x;
    double weight;
    
};

//
class EdgeInertialGS : public ceres::SizedCostFunction<9, 3, 3, 3, 3, 1>
//class EdgeInertialGS : public ceres::SizedCostFunction<9, 3, 3, 3, 3, 1,3>
{
public:
    
  
    
    //不需要优化的：视觉T
    //需要优化的：视觉速度，重力，尺度，bias
    EdgeInertialGS(const ImageFrame &_frame_i, const ImageFrame &_frame_j, const Eigen::Vector3d &_t_imu_cam);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    
    

    Eigen::Matrix3d JRg, JVg, JPg;
    Eigen::Matrix3d JVa, JPa;
   
    
    Eigen::Vector3d g, gI;
    
    double dt;
//    dq,dp,dv;
    ImageFrame frame_i,frame_j;
    IntegrationBase *pre_integration_j;
    Eigen::Quaterniond q_ij,delta_q;
    Eigen::Vector3d delta_v,delta_p;
    Eigen::Vector3d t_imu_cam;
   
    Eigen::Matrix3d dp_dba,dp_dbg,dq_dbg,dv_dba,dv_dbg;
    
    Eigen::Vector3d bias_gyro,bias_acc;
    
    Eigen::Vector3d acc;
    
    
    
    static Eigen::Matrix<double,9,9> sqrt_info;
//    static Eigen::Matrix<double,12,12> sqrt_info;
    static double sum_t;

   


};


#endif /* inertial_factor_h */
