//
//  projection_factor_merge.m
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/9/10.
//  Copyright © 2020 zx. All rights reserved.
//

#include "projection_factor_merge.h"

Eigen::Matrix2d ProjectionFactor_merge::sqrt_info;
double ProjectionFactor_merge::sum_t;

ProjectionFactor_merge::ProjectionFactor_merge(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j) : pts_i(_pts_i), pts_j(_pts_j){};

//这里是两个imu-cam的外参数
/**
bool ProjectionFactor_merge::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    
    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    
    Eigen::Vector3d tic_i(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic_i(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    
    Eigen::Vector3d tic_j(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond qic_j(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);
    
    double inv_dep_i = parameters[4][0];
    
    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic_i * pts_camera_i + tic_i;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    
    
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Eigen::Vector3d pts_camera_j = qic_j.inverse() * (pts_imu_j - tic_j);
    
    double dep_j = pts_camera_j.z();
    
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
    residual = sqrt_info * residual;
    
    if (jacobians)
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric_j = qic_j.toRotationMatrix();
        Eigen::Matrix3d ric_i = qic_i.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);
        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
        0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
        
        reduce = sqrt_info * reduce;
        
        if (jacobians[0])//右扰动模型
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            
            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric_j.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() = ric_j.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);
            
            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }
        
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
            
            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric_j.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric_j.transpose() * Utility::skewSymmetric(pts_imu_j);
            
            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_posei(jacobians[2]);
            Eigen::Matrix<double, 3, 6> jaco_ex_i;
            jaco_ex_i.leftCols<3>() = ric_j.transpose() * Rj.transpose() * Ri ;
            Eigen::Matrix3d tmp_r = ric_j.transpose() * Rj.transpose() * Ri * ric_i;
            jaco_ex_i.rightCols<3>() =  -tmp_r * Utility::skewSymmetric(pts_camera_i);
            jacobian_ex_posei.leftCols<6>() = reduce * jaco_ex_i;
            jacobian_ex_posei.rightCols<1>().setZero();
        }
        
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_posej(jacobians[3]);
            Eigen::Matrix<double, 3, 6> jaco_ex_j;
            jaco_ex_j.leftCols<3>() = -ric_j.transpose() ;
            Eigen::Matrix3d tmp_r = ric_j.transpose() * Rj.transpose() * Ri * ric_i;
            jaco_ex_j.rightCols<3>() = Utility::skewSymmetric(tmp_r * pts_camera_i) +
            Utility::skewSymmetric(ric_j.transpose() * (Rj.transpose() * (Ri * tic_i + Pi - Pj) - tic_j)) ;
            jacobian_ex_posej.leftCols<6>() = reduce * jaco_ex_j;
            jacobian_ex_posej.rightCols<1>().setZero();
        }
        if (jacobians[4])
        {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[4]);
#if 1
            jacobian_feature = reduce * ric_j.transpose() * Rj.transpose() * Ri * ric_i * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
#else
            jacobian_feature = reduce * ric_j.transpose() * Rj.transpose() * Ri * ric_i * pts_i;
#endif
        }
    }
    
    return true;
}
*/

//这里是假设 3D点是准确的 也就是说，主地图的点是准确的
bool ProjectionFactor_merge::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pj(parameters[0][0], parameters[0][1], parameters[0][2]);     
    Eigen::Matrix3d matrix_qj=Utility::ypr2R(Eigen::Vector3d(parameters[1][0],parameters[1][1],parameters[1][2]));
    Eigen::Quaterniond Qj;
    Qj=matrix_qj;

    Eigen::Vector3d twj_wi(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qwj_wi(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
   
    Eigen::Vector3d tic_j(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond qic_j(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);
    
 
    
 
    
    Eigen::Vector3d pts_w =qwj_wi*pts_i+twj_wi;//把i点从主地图世界坐标系转到小地图的世界坐标系
    
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Eigen::Vector3d pts_camera_j = qic_j.inverse() * (pts_imu_j - tic_j);
    
    double dep_j = pts_camera_j.z();
    
//    if(dep_j>30 || dep_j<0){
//        std::cout<<" 测试 dep_j:"<<dep_j;
//        std::cout<<"测试 两个世界坐标系的转换 "<<twj_wi.x()<<" "<<twj_wi.y()<<" "<<twj_wi.z()<<" "<<qwj_wi.x()<<" "<<qwj_wi.y()<<" "<<qwj_wi.z()<<" "<<qwj_wi.w()<<std::endl;
//        std::cout<<"测试 Pj Qj "<<Pj.x()<<" "<<Pj.y()<<" "<<Pj.z()<<" "<<Qj.x()<<" "<<Qj.y()<<" "<<Qj.z()<<" "<<Qj.w()<<std::endl;
//        std::cout<<"测试 tic_j qic_j "<<tic_j.x()<<" "<<tic_j.y()<<" "<<tic_j.z()<<" "<<qic_j.x()<<" "<<qic_j.y()<<" "<<qic_j.z()<<" "<<qic_j.w()<<std::endl;
//    }
 
    
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
//    std::cout<<"测试 实际误差："<<residual[0]<<" "<<residual[1]<<std::endl;
    
    residual = sqrt_info * residual;
//    std::cout<<"测试 变化后 实际误差："<<residual[0]<<" "<<residual[1]<<std::endl;
    
    if (jacobians)
    {
        Eigen::Matrix3d Rj = matrix_qj;
        Eigen::Matrix3d Rwj_wi = qwj_wi.toRotationMatrix();
        Eigen::Matrix3d ric_j = qic_j.toRotationMatrix();
//        Eigen::Matrix3d ric_i = qic_i.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);
        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
        0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
        
        reduce = sqrt_info * reduce;
        
        
         if (jacobians[0])//右扰动模型
         {
             Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_pose_j(jacobians[0]);
             
             Eigen::Matrix<double, 3, 3> jaco_j;
             jaco_j = -ric_j.transpose()*Rj.transpose() ;
             
             
             jacobian_pose_j = reduce * jaco_j;
             
         }
        if (jacobians[1])//右扰动模型
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
            
            Eigen::Matrix<double, 3, 3> jaco_j;
            jaco_j = ric_j.transpose() * Utility::skewSymmetric(pts_imu_j);
            
            
            jacobian_pose_j= reduce * jaco_j;
            
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_wj_wi(jacobians[2]);
            Eigen::Matrix<double, 3, 6> jaco_wj_wi;
            jaco_wj_wi.leftCols<3>() = ric_j.transpose() * Rj.transpose() ;
            Eigen::Matrix3d tmp_r = ric_j.transpose() * Rj.transpose();
            jaco_wj_wi.rightCols<3>() =  -tmp_r * Utility::skewSymmetric(Rwj_wi*pts_i);
            jacobian_pose_wj_wi.leftCols<6>() = reduce * jaco_wj_wi;
            jacobian_pose_wj_wi.rightCols<1>().setZero();
        }
        
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_posej(jacobians[3]);
            Eigen::Matrix<double, 3, 6> jaco_ex_j;
            jaco_ex_j.leftCols<3>() = -ric_j.transpose() ;
            jaco_ex_j.rightCols<3>() = Utility::skewSymmetric(pts_camera_j) ;
            jacobian_ex_posej.leftCols<6>() = reduce * jaco_ex_j;
            jacobian_ex_posej.rightCols<1>().setZero();
        }
        
    }
    
    return true;
}


//---------------------分割线-----------------
Eigen::Matrix2d MultiClientFactor_merge::sqrt_info;
double MultiClientFactor_merge::sum_t;

//MultiClientFactor_merge::MultiClientFactor_merge(const Eigen::Vector3d &_relative_t, const double &_relative_yaw,const double &_euler_pitch, const double &_euler_roll,const double &_euler_pitch_j, const double &_euler_roll_j) : relative_t(_relative_t), relative_yaw(_relative_yaw),  euler_pitch(_euler_pitch), euler_roll(_euler_roll),  euler_pitch_j(_euler_pitch_j), euler_roll_j(_euler_roll_j){};

MultiClientFactor_merge::MultiClientFactor_merge(const Eigen::Vector3d &_relative_t, const double &_euler_pitch, const double &_euler_roll) : relative_t(_relative_t), euler_pitch(_euler_pitch), euler_roll(_euler_roll){};

//这里是
bool MultiClientFactor_merge::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    
//    Eigen::Vector3d Pi_yaw(parameters[0][0]);
    double Pi_yaw_theta=parameters[0][0];
//    Eigen::Matrix3d matrix_Pi_c_w=Utility::ypr2R(Eigen::Vector3d(Pi_yaw,euler_pitch,euler_roll));
    
//    Eigen::Quaterniond Pi_yaw(cos(Pi_yaw_theta/2.0/180.0*M_PI), 0, 0, sin(Pi_yaw_theta/2.0/180.0*M_PI));
    Eigen::Quaterniond Pi_pitch(cos(euler_pitch/2.0/180.0*M_PI), 0, sin(euler_pitch/2.0/180.0*M_PI), 0);
    Eigen::Quaterniond Pi_roll(cos(euler_roll/2.0/180.0*M_PI), sin(euler_roll/2.0/180.0*M_PI) ,0,  0);
    
    Eigen::Vector3d Pi_t_w(parameters[1][0],parameters[1][1],parameters[1][2]);
    
    Eigen::Vector3d Pj_t_w(parameters[2][0],parameters[2][1],parameters[2][2]);
    Eigen::Vector3d twm_wi(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond qwm_wi(parameters[4][3], parameters[4][0], parameters[4][1], parameters[4][2]);

    Eigen::Vector3d twm_wj(parameters[5][0], parameters[5][1], parameters[5][2]);
    Eigen::Quaterniond qwm_wj(parameters[6][3], parameters[6][0], parameters[6][1], parameters[6][2]);
    
    
 
    Eigen::Vector3d ptsI_wm_i1 =qwm_wi*Pi_t_w+twm_wi;
    Eigen::Vector3d ptsI_w2_I1 =qwm_wj.inverse()*(ptsI_wm_i1-twm_wj);
    
    Eigen::Vector3d pts_relative = Pj_t_w-ptsI_w2_I1;
    
    Eigen::Quaterniond ypr_pr_i=Pi_pitch*Pi_roll;
    
    
    Eigen::Matrix3d ypr_y_i=Utility::y2R(Eigen::Vector2d(Pi_yaw_theta,0));
    Eigen::Matrix3d ypr_y_daoshu=Utility::y2R_daoshu(Eigen::Vector2d(Pi_yaw_theta,0));
    
    Eigen::Quaterniond qwm_w1t=qwm_wj.inverse()*qwm_wi;
    Eigen::Vector3d pr_relative_t=ypr_pr_i*relative_t;
    
    Eigen::Vector3d pts_relative_real = qwm_w1t*ypr_y_i * pr_relative_t;
    
    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual= (pts_relative-pts_relative_real);
    if (jacobians)
    {
        Eigen::Matrix<double, 3, 3> reduce(3, 3);
        
        reduce = Eigen::Matrix3d::Identity();
        
         if (jacobians[0])//右扰动模型
         {
             Eigen::Map<Eigen::Vector3d> jacobian_i_yaw(jacobians[0]);
             jacobian_i_yaw = -reduce * qwm_w1t*ypr_y_daoshu*pr_relative_t / 180.0 * M_PI;
//             jacobian_i_yaw = reduce * qwm_w1t*Utility::skewSymmetric(Pi_yaw * pr_relative_t);
//             std::cout<<"期待变换ypr_y_T="<<ypr_y_T.w()<<" , "<<ypr_y_T.x()<<" , "<<ypr_y_T.y()<<" , "<<ypr_y_T.z()<<std::endl;
         }
        if (jacobians[1])//右扰动模型
        {
            Eigen::Map<Eigen::Matrix<double,3, 3, Eigen::RowMajor>> jacobian_i_t(jacobians[1]);
            jacobian_i_t = -reduce * qwm_wj.inverse()*qwm_wi;
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double,3, 3, Eigen::RowMajor>> jacobian_j_t(jacobians[2]);
            jacobian_j_t = -reduce * Eigen::Matrix3d::Identity();
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double,3, 3, Eigen::RowMajor>> jacobian_ex_posei_t(jacobians[3]);
            jacobian_ex_posei_t = -reduce * qwm_wj.inverse();
        }
        if (jacobians[4])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_ex_posei_r(jacobians[4]);
            Eigen::Matrix<double, 3, 3> jaco_ex_i;
            jaco_ex_i =qwm_wj.inverse()*Utility::skewSymmetric(qwm_wi*Pi_t_w)+ qwm_wj.inverse()*Utility::skewSymmetric(qwm_wi*ypr_y_i * pr_relative_t) ;
            jacobian_ex_posei_r.leftCols<3>() = reduce * jaco_ex_i;
            jacobian_ex_posei_r.rightCols<1>().setZero();
        }
        if (jacobians[5])
        {
            Eigen::Map<Eigen::Matrix<double,3, 3, Eigen::RowMajor>> jacobian_ex_posej_t(jacobians[5]);
            jacobian_ex_posej_t =reduce * qwm_wj.inverse();
        }
        if (jacobians[6])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_ex_posej_r(jacobians[6]);
            Eigen::Matrix<double, 3, 3> jaco_ex_j;
            jaco_ex_j = -Utility::skewSymmetric(ptsI_w2_I1)- Utility::skewSymmetric(pts_relative_real);
            jacobian_ex_posej_r.leftCols<3>() = reduce * jaco_ex_j;
            jacobian_ex_posej_r.rightCols<1>().setZero();
        }
    }
    
    
    return true;
}


//---------------------分割线22-----------------
Eigen::Matrix2d MultiClientFactor_mergePR::sqrt_info;
double MultiClientFactor_mergePR::sum_t;

//MultiClientFactor_merge::MultiClientFactor_merge(const Eigen::Vector3d &_relative_t, const double &_relative_yaw,const double &_euler_pitch, const double &_euler_roll,const double &_euler_pitch_j, const double &_euler_roll_j) : relative_t(_relative_t), relative_yaw(_relative_yaw),  euler_pitch(_euler_pitch), euler_roll(_euler_roll),  euler_pitch_j(_euler_pitch_j), euler_roll_j(_euler_roll_j){};

MultiClientFactor_mergePR::MultiClientFactor_mergePR(const Eigen::Vector3d &_relative_t, const double &_euler_pitch, const double &_euler_roll) : relative_t(_relative_t), euler_pitch(_euler_pitch), euler_roll(_euler_roll){};

//这里是
bool MultiClientFactor_mergePR::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    
//    Eigen::Vector3d Pi_yaw(parameters[0][0]);
    double Pi_yaw_theta=parameters[0][0];
//    Eigen::Matrix3d matrix_Pi_c_w=Utility::ypr2R(Eigen::Vector3d(Pi_yaw,euler_pitch,euler_roll));
    
//    Eigen::Quaterniond Pi_yaw(cos(Pi_yaw_theta/2.0/180.0*M_PI), 0, 0, sin(Pi_yaw_theta/2.0/180.0*M_PI));
    Eigen::Quaterniond Pi_pitch(cos(euler_pitch/2.0/180.0*M_PI), 0, sin(euler_pitch/2.0/180.0*M_PI), 0);
    Eigen::Quaterniond Pi_roll(cos(euler_roll/2.0/180.0*M_PI), sin(euler_roll/2.0/180.0*M_PI) ,0,  0);
    
    Eigen::Vector3d Pi_t_w(parameters[1][0],parameters[1][1],parameters[1][2]);
    
    Eigen::Vector3d Pj_t_w(parameters[2][0],parameters[2][1],parameters[2][2]);
    Eigen::Vector3d twm_wi(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond qwm_wi(parameters[4][3], parameters[4][0], parameters[4][1], parameters[4][2]);

    Eigen::Vector3d twm_wj(parameters[5][0], parameters[5][1], parameters[5][2]);
    Eigen::Quaterniond qwm_wj(parameters[6][3], parameters[6][0], parameters[6][1], parameters[6][2]);
    
    
 
    Eigen::Vector3d ptsI_wm_i1 =qwm_wi*Pi_t_w+twm_wi;
    Eigen::Vector3d ptsI_w2_I1 =qwm_wj.inverse()*(ptsI_wm_i1-twm_wj);
    
    Eigen::Vector3d pts_relative = Pj_t_w-ptsI_w2_I1;
    
    Eigen::Quaterniond ypr_pr_i=Pi_pitch*Pi_roll;
    
    
    Eigen::Matrix3d ypr_y_i=Utility::y2R(Eigen::Vector2d(Pi_yaw_theta,0));
    Eigen::Matrix3d ypr_y_daoshu=Utility::y2R_daoshu(Eigen::Vector2d(Pi_yaw_theta,0));
    
    Eigen::Quaterniond qwm_w1t=qwm_wj.inverse()*qwm_wi;
    Eigen::Vector3d pr_relative_t=ypr_pr_i*relative_t;
    
    Eigen::Vector3d pts_relative_real = qwm_w1t*ypr_y_i * pr_relative_t;
    
    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual= (pts_relative-pts_relative_real);
    if (jacobians)
    {
        Eigen::Matrix<double, 3, 3> reduce(3, 3);
        
        reduce = Eigen::Matrix3d::Identity();
        
         if (jacobians[0])//右扰动模型
         {
             Eigen::Map<Eigen::Vector3d> jacobian_i_yaw(jacobians[0]);
             jacobian_i_yaw = -reduce * qwm_w1t*ypr_y_daoshu*pr_relative_t / 180.0 * M_PI;
//             jacobian_i_yaw = reduce * qwm_w1t*Utility::skewSymmetric(Pi_yaw * pr_relative_t);
//             std::cout<<"期待变换ypr_y_T="<<ypr_y_T.w()<<" , "<<ypr_y_T.x()<<" , "<<ypr_y_T.y()<<" , "<<ypr_y_T.z()<<std::endl;
         }
        if (jacobians[1])//右扰动模型
        {
            Eigen::Map<Eigen::Matrix<double,3, 3, Eigen::RowMajor>> jacobian_i_t(jacobians[1]);
            jacobian_i_t = -reduce * qwm_wj.inverse()*qwm_wi;
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double,3, 3, Eigen::RowMajor>> jacobian_j_t(jacobians[2]);
            jacobian_j_t = -reduce * Eigen::Matrix3d::Identity();
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double,3, 3, Eigen::RowMajor>> jacobian_ex_posei_t(jacobians[3]);
            jacobian_ex_posei_t = -reduce * qwm_wj.inverse();
        }
        if (jacobians[4])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_ex_posei_r(jacobians[4]);
            Eigen::Matrix<double, 3, 3> jaco_ex_i;
            jaco_ex_i =qwm_wj.inverse()*Utility::skewSymmetric(qwm_wi*Pi_t_w)+ qwm_wj.inverse()*Utility::skewSymmetric(qwm_wi*ypr_y_i * pr_relative_t) ;
            jacobian_ex_posei_r.leftCols<3>() = reduce * jaco_ex_i;
            jacobian_ex_posei_r.rightCols<1>().setZero();
        }
        if (jacobians[5])
        {
            Eigen::Map<Eigen::Matrix<double,3, 3, Eigen::RowMajor>> jacobian_ex_posej_t(jacobians[5]);
            jacobian_ex_posej_t =reduce * qwm_wj.inverse();
        }
        if (jacobians[6])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_ex_posej_r(jacobians[6]);
            Eigen::Matrix<double, 3, 3> jaco_ex_j;
            jaco_ex_j = -Utility::skewSymmetric(ptsI_w2_I1)- Utility::skewSymmetric(pts_relative_real);
            jacobian_ex_posej_r.leftCols<3>() = reduce * jaco_ex_j;
            jacobian_ex_posej_r.rightCols<1>().setZero();
        }
    }
    
    
    return true;
}

