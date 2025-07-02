//
//  VINS.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "VINS.hpp"

bool LOOP_CLOSURE =  false;
bool LOOP_CLOSURE_SERVER=  true ;
bool LOOP_CLOSURE_SERVER_noLoop=  false;


VINS::VINS()
:f_manager{Rs},fail_times{0},
failure_hand{false},
drawresult{0.0, 0.0, 0.0, 0.0, 0.0, 7.0},priorMap_f_manager{Rs}
{
    printf("init VINS begins\n");
    t_drift.setZero();
    r_drift.setIdentity();
    clearState();
    failure_occur = 0;
    last_P.setZero();
    last_R.setIdentity();
    last_P_old.setZero();
    last_R_old.setIdentity();
    
    //ljl
//    front_pose.sendRelativeData_server=false;
    front_pose.isRemove=2;
    sendServer_relative=false;
    
    header_forwkf=0;
    
    isUpdate_rt_drift=false;
    kf_id_inLocal=-1;
}

void VINS::setIMUModel()
{
    ProjectionFactor::sqrt_info = FOCUS_LENGTH_X / 1.5 * Eigen::Matrix2d::Identity();
}

void VINS::clearState()
{
    printf("clear state\n");
    for (int i = 0; i < 10 * (WINDOW_SIZE + 1); i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        IMU_linear[i].setZero();
        IMU_angular[i].setIdentity();
        pre_integrations[i] = nullptr;
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();
        
        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }
    tic << TIC_X,
    TIC_Y,
    TIC_Z;
    ric = Utility::ypr2R(Eigen::Vector3d(RIC_y,RIC_p,RIC_r));
    
    frame_count = 0;
    first_imu = false;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    initProgress = 0;
    
    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;
    
    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();
    
    f_manager.clearState();
    priorMap_f_manager.clearState();
    
    front_pose.cur_index=0;
//    printf("clearstate PS %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
//    printf("clearstate Vs %lf %lf %lf\n", Vs[0].x(),Vs[0].y(), Vs[0].z());
}

void VINS::setExtrinsic()
{
    tic << TIC_X,
    TIC_Y,
    TIC_Z;
    ric = Utility::ypr2R(Eigen::Vector3d(RIC_y,RIC_p,RIC_r));
}
void VINS::old2new()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Eigen::Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();
        
        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();
        
        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();
        
        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic.x();
        para_Ex_Pose[i][1] = tic.y();
        para_Ex_Pose[i][2] = tic.z();
        Eigen::Quaterniond q{ric};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }
    //triangulate
    Eigen::VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
}
//备份
void VINS::new2old()
{
//    printf("before new2old PS %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    
    Eigen::Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    
    Eigen::Vector3d origin_P0 = Ps[0];
    
    if (failure_occur)
    {
        printf("failure recover %lf %lf %lf\n", last_P.x(), last_P.y(), last_P.z());
        origin_R0 = Utility::R2ypr(last_R_old);
        origin_P0 = last_P_old;
    }
    
    Eigen::Vector3d origin_R00 = Utility::R2ypr(Eigen::Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5]).toRotationMatrix());
    
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Eigen::Matrix3d rot_diff = Utility::ypr2R(Eigen::Vector3d(y_diff, 0, 0));
    
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        
        Rs[i] = rot_diff * Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        Ps[i] = rot_diff * Eigen::Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
//        printf("new2old PS %lf %lf %lf\n", Ps[i].x(),Ps[i].y(), Ps[i].z());
        Vs[i] = rot_diff * Eigen::Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);
//        printf("new2old Vs %lf %lf %lf\n", Vs[i].x(),Vs[i].y(), Vs[i].z());
        Bas[i] = Eigen::Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);
        
        Bgs[i] = Eigen::Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }
    Eigen::Vector3d cur_P0 = Ps[0];
    
    if(LOOP_CLOSURE && loop_enable)
    {
        loop_enable = false;
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
//            if(front_pose.header == Headers[i])
            if(fabs(front_pose.header - Headers[i])<0.00001)
            {
                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
                
                Rs_loop = rot_diff * Rs_loop;
                Ps_loop = rot_diff * (Ps_loop - Eigen::Vector3d(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2])) + origin_P0;
                
                double drift_yaw = Utility::R2ypr(front_pose.Q_old.toRotationMatrix()).x() - Utility::R2ypr(Rs_loop).x();
                r_drift = Utility::ypr2R(Eigen::Vector3d(drift_yaw, 0, 0));
                //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
                t_drift = front_pose.P_old - r_drift * Ps_loop;
            }
        }
    }
    
   else if(LOOP_CLOSURE_SERVER && loop_enable)
    {
        cout<<"这里应该进不来"<<endl;
        loop_enable = false;
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
            if(fabs(front_pose.header - Headers[i])<0.00001)
//            if(front_pose.header == Headers[i])
            {
                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
                
                Rs_loop = rot_diff * Rs_loop;
                Ps_loop = rot_diff * (Ps_loop - Eigen::Vector3d(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2])) + origin_P0;
                
                double drift_yaw = Utility::R2ypr(front_pose.Q_old.toRotationMatrix()).x() - Utility::R2ypr(Rs_loop).x();
                r_drift = Utility::ypr2R(Eigen::Vector3d(drift_yaw, 0, 0));
                //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
                t_drift = front_pose.P_old - r_drift * Ps_loop;
            }
        }
    }
    
   else if(LOOP_CLOSURE_SERVER_noLoop && loop_enable)
    {
        cout<<"这里应该进不来"<<endl;
        loop_enable = false;
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
            if(fabs(front_pose.header - Headers[i])<0.00001)
//            if(front_pose.header == Headers[i])
            {
                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
                
                Rs_loop = rot_diff * Rs_loop;
                Ps_loop = rot_diff * (Ps_loop - Eigen::Vector3d(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2])) + origin_P0;
                
                double drift_yaw = Utility::R2ypr(front_pose.Q_old.toRotationMatrix()).x() - Utility::R2ypr(Rs_loop).x();
                r_drift = Utility::ypr2R(Eigen::Vector3d(drift_yaw, 0, 0));
                //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
                t_drift = front_pose.P_old - r_drift * Ps_loop;
            }
        }
    }
    
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic = Eigen::Vector3d(para_Ex_Pose[i][0],
                       para_Ex_Pose[i][1],
                       para_Ex_Pose[i][2]);
        ric = Eigen::Quaterniond(para_Ex_Pose[i][6],
                          para_Ex_Pose[i][3],
                          para_Ex_Pose[i][4],
                          para_Ex_Pose[i][5]).toRotationMatrix();
    }
    
    Eigen::VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
    {
        dep(i) = para_Feature[i][0];
    }
    f_manager.setDepth(dep);
}

void VINS::new2old_2()
{
//    printf("before new2old PS %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    
    Eigen::Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    
    Eigen::Vector3d origin_P0 = Ps[0];
    
    if (failure_occur)
    {
        printf("failure recover %lf %lf %lf\n", last_P.x(), last_P.y(), last_P.z());
        origin_R0 = Utility::R2ypr(last_R_old);
        origin_P0 = last_P_old;
    }
    
    Eigen::Vector3d origin_R00 = Utility::R2ypr(Eigen::Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5]).toRotationMatrix());
    
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Eigen::Matrix3d rot_diff = Utility::ypr2R(Eigen::Vector3d(y_diff, 0, 0));
    
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        
        Rs[i] = rot_diff * Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        Ps[i] = rot_diff * Eigen::Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
//        printf("new2old PS %lf %lf %lf\n", Ps[i].x(),Ps[i].y(), Ps[i].z());
        Vs[i] = rot_diff * Eigen::Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);
//        printf("new2old Vs %lf %lf %lf\n", Vs[i].x(),Vs[i].y(), Vs[i].z());
        Bas[i] = Eigen::Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);
        
        Bgs[i] = Eigen::Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }
    Eigen::Vector3d cur_P0 = Ps[0];
    
    if(LOOP_CLOSURE && loop_enable)
    {
        loop_enable = false;
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
//            if(front_pose.header == Headers[i])
            if(fabs(front_pose.header - Headers[i])<0.00001)
            {
                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
                
//                新帧3d点 老帧2d
//                Rs_loop = rot_diff * Rs_loop;
//                Ps_loop = rot_diff * (Ps_loop - Eigen::Vector3d(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2])) + origin_P0;
//
//                double drift_yaw = Utility::R2ypr(front_pose.Q_old.toRotationMatrix()).x() - Utility::R2ypr(Rs_loop).x();
//                r_drift = Utility::ypr2R(Eigen::Vector3d(drift_yaw, 0, 0));
//                //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
//                t_drift = front_pose.P_old - r_drift * Ps_loop;
                
//               老帧3d 新帧2d
                Eigen::Matrix3d Rs_i = Rs[i];
                Eigen::Vector3d Ps_i = Ps[i];


                double drift_yaw = Utility::R2ypr(Rs_loop).x() - Utility::R2ypr(Rs_i).x();
                r_drift = Utility::ypr2R(Eigen::Vector3d(drift_yaw, 0, 0));
                //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
                t_drift = Ps_loop - r_drift * Ps_i;
            }
        }
    }
    
   else if(LOOP_CLOSURE_SERVER && loop_enable)
    {
        loop_enable = false;
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
            if(fabs(front_pose.header - Headers[i])<0.00001)
//            if(front_pose.header == Headers[i])
            {
//                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
//                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
//
//                Rs_loop = rot_diff * Rs_loop;
//                Ps_loop = rot_diff * (Ps_loop - Eigen::Vector3d(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2])) + origin_P0;
//
//                double drift_yaw = Utility::R2ypr(front_pose.Q_old.toRotationMatrix()).x() - Utility::R2ypr(Rs_loop).x();
//                r_drift = Utility::ypr2R(Eigen::Vector3d(drift_yaw, 0, 0));
//                //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
//                t_drift = front_pose.P_old - r_drift * Ps_loop;
                
//                服务器发送过来 老帧3d点
//                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
//                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
//
//                Eigen::Matrix3d Rs_i = Rs[i];
//                Eigen::Vector3d Ps_i = Ps[i];
//
//                double drift_yaw = Utility::R2ypr(Rs_loop).x() - Utility::R2ypr(Rs_i).x();
//                r_drift = Utility::ypr2R(Eigen::Vector3d(drift_yaw, 0, 0));
//                //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
//                t_drift = Ps_loop - r_drift * Ps_i;
                cout<<"进行了"<<endl;
                
//                局部地图跟踪
//                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(loop_pose_forSlideWindow_update[6],  loop_pose_forSlideWindow_update[3],  loop_pose_forSlideWindow_update[4],  loop_pose_forSlideWindow_update[5]).normalized().toRotationMatrix();
//                Eigen::Vector3d Ps_loop = Eigen::Vector3d( loop_pose_forSlideWindow_update[0], loop_pose_forSlideWindow_update[1],  loop_pose_forSlideWindow_update[2]);
//
//                Eigen::Matrix3d Rs_i = Rs[i];
//                Eigen::Vector3d Ps_i = Ps[i];
//
//                double drift_yaw = Utility::R2ypr(Rs_loop).x() - Utility::R2ypr(Rs_i).x();
//                r_drift = Utility::ypr2R(Eigen::Vector3d(drift_yaw, 0, 0));
//                //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
//                t_drift = Ps_loop - r_drift * Ps_i;
            }
        }
    }
//   else if(LOOP_CLOSURE_SERVER && isUpdate_rt_drift)
//    {
//        isUpdate_rt_drift = false;
//        Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(para_Pose_priorMap[kf_id_inLocal][6],  para_Pose_priorMap[kf_id_inLocal][3],  para_Pose_priorMap[kf_id_inLocal][4],  para_Pose_priorMap[kf_id_inLocal][5]).normalized().toRotationMatrix();
//        Eigen::Vector3d Ps_loop = Eigen::Vector3d( para_Pose_priorMap[kf_id_inLocal][0],  para_Pose_priorMap[kf_id_inLocal][1],  para_Pose_priorMap[kf_id_inLocal][2]);
//
//        Rs_loop = rot_diff * Rs_loop;
//        Ps_loop = rot_diff * (Ps_loop - Eigen::Vector3d(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2])) + origin_P0;
//
//        double drift_yaw =Utility::R2ypr(Rs_loop).x()- Utility::R2ypr(Rs[kf_id_inLocal]).x() ;
//        r_drift = Utility::ypr2R(Eigen::Vector3d(drift_yaw, 0, 0));
//        //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
//        t_drift = Ps_loop  - r_drift *Ps[kf_id_inLocal] ;
//
//    }
   else if(LOOP_CLOSURE_SERVER_noLoop && loop_enable)
    {
        loop_enable = false;
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
            if(fabs(front_pose.header - Headers[i])<0.00001)
//            if(front_pose.header == Headers[i])
            {
//                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
//                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
//
//                Rs_loop = rot_diff * Rs_loop;
//                Ps_loop = rot_diff * (Ps_loop - Eigen::Vector3d(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2])) + origin_P0;
//
//                double drift_yaw = Utility::R2ypr(front_pose.Q_old.toRotationMatrix()).x() - Utility::R2ypr(Rs_loop).x();
//                r_drift = Utility::ypr2R(Eigen::Vector3d(drift_yaw, 0, 0));
//                //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
//                t_drift = front_pose.P_old - r_drift * Ps_loop;
                
                
//                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
//                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
//
//                Eigen::Matrix3d Rs_i = Rs[i];
//                Eigen::Vector3d Ps_i = Ps[i];
//
//                double drift_yaw = Utility::R2ypr(Rs_loop).x() - Utility::R2ypr(Rs_i).x();
//                r_drift = Utility::ypr2R(Eigen::Vector3d(drift_yaw, 0, 0));
//                //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
//                t_drift = Ps_loop - r_drift * Ps_i;
            }
        }
    }
    
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic = Eigen::Vector3d(para_Ex_Pose[i][0],
                       para_Ex_Pose[i][1],
                       para_Ex_Pose[i][2]);
        ric = Eigen::Quaterniond(para_Ex_Pose[i][6],
                          para_Ex_Pose[i][3],
                          para_Ex_Pose[i][4],
                          para_Ex_Pose[i][5]).toRotationMatrix();
    }
    
    Eigen::VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
    {
        dep(i) = para_Feature[i][0];
    }
    f_manager.setDepth(dep);
}

bool VINS::failureDetection()
{
    bool is_failure = false;
    
    if (f_manager.last_track_num < 4)
    {
        printf("failure little feature %d\n", f_manager.last_track_num);
        is_failure = true;
    }
    /*
     if (Bas[WINDOW_SIZE].norm() > 1)
     {
     printf("failure  big IMU acc bias estimation %f\n", Bas[WINDOW_SIZE].norm());
     is_failure = true;
     }
     */
    if (Bgs[WINDOW_SIZE].norm() > 1)
    {
        printf("failure  big IMU gyr bias estimation %f\n", Bgs[WINDOW_SIZE].norm());
        is_failure = true;
    }
    Eigen::Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 1)
    {
        printf("failure big translation\n");
        is_failure = true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 0.5)
    {
        printf("failure  big z translation\n");
        is_failure = true;
    }
    Eigen::Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Eigen::Matrix3d delta_R = tmp_R.transpose() * last_R;
    Eigen::Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 40)
    {
        printf("failure  big delta_angle \n");
        is_failure = true;
    }
    
    if(failure_hand)
    {
        failure_hand = false;
        is_failure = true;
        printf("failure by hand!\n");
    }
    
    return is_failure;
}

/*
 void VINS::failureRecover()
 {
 int his_index = 0;
 for(int i = 0; i < WINDOW_SIZE; i++)
 {
 if(Headers_his[i] == Headers[0])
 {
 his_index = i;
 break;
 }
 if(i == WINDOW_SIZE -1)
 his_index = i;
 }
 Eigen::Vector3d his_R0 = Utility::R2ypr(Rs_his[his_index]);
 
 Eigen::Vector3d his_P0 = Ps_his[his_index];
 
 Eigen::Vector3d cur_R0 = Utility::R2ypr(Rs[0]);
 Eigen::Vector3d cur_P0 = Ps[0];
 
 double y_diff = his_R0.x() - cur_R0.x();
 
 Eigen::Matrix3d rot_diff = Utility::ypr2R(Eigen::Vector3d(y_diff, 0, 0));
 
 for (int i = 0; i <= WINDOW_SIZE; i++)
 {
 Rs[i] = rot_diff * Rs[i];
 Ps[i] = rot_diff * (Ps[i] - cur_P0) + his_P0;
 Vs[i] = rot_diff * Vs[i];
 }
 }
 */

void VINS::reInit()
{
    failure_hand = true;
    failureDetection();
}

void VINS::update_loop_correction()
{
    //update loop correct pointcloud
    correct_point_cloud.clear();
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 4 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (/*it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 ||*/ it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Eigen::Vector3d tmp = r_drift * Rs[imu_i] * (ric * pts_i + tic) + r_drift * Ps[imu_i] + t_drift;
        correct_point_cloud.push_back(tmp.cast<float>());
    }
    //update correct pose
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        Eigen::Vector3d correct_p = r_drift * Ps[i] + t_drift;
        correct_Ps[i] = correct_p.cast<float>();
        Eigen::Matrix3d correct_r = r_drift * Rs[i];
        correct_Rs[i] = correct_r.cast<float>();
    }
}

void VINS::processIMU(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity)
{
//    cout<<"frame_count="<<frame_count<<endl;
    //判断是不是第一个imu消息，如果是第一个imu消息，则将当前传入的线加速度和角速度作为初始的加速度和角速度
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    //创建预积分对象
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    //当滑动窗口内有图像帧数据时，进行预积分 frame_count表示滑动窗口中图像数据的个数
    if (frame_count != 0)
    {
        //covariance propagate协方差传播
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        
        if(solver_flag != NON_LINEAR) //comments because of recovering
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);
        
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
        
        //更新Rs旋转 Ps位置 Vs速度三个向量数组
        //midpoint integration
        {
            
            Eigen::Vector3d g{0,0,GRAVITY};
            int j = frame_count;
//            printf("before processIMU PS%d %lf %lf %lf\n", j,Ps[j].x(),Ps[j].y(), Ps[j].z());
//            printf("dt=%lf ",dt);
//            printf("before processIMU Vs%d %lf %lf %lf\n", j,Vs[j].x(),Vs[j].y(), Vs[j].z());
            
            Eigen::Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
            Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
            Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            Eigen::Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
            Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
//            printf("before processIMU un_acc%d %lf %lf %lf\n", j,un_acc.x(),un_acc.y(), un_acc.z());
            Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
            Vs[j] += dt * un_acc;
            
//            printf("processIMU PS0 %lf %lf %lf\n",Ps[0].x(),Ps[0].y(), Ps[0].z());
//            printf("processIMU Vs%d %lf %lf %lf\n", j,Vs[j].x(),Vs[j].y(), Vs[j].z());
        }
        
    }
    //当frame_count==0的时候 表示滑动窗口中还没有图像帧数据，不需要进行预积分，只进行线加速度和角速度的更新
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}
//image_msg 第一个元素是 相机编号，代表第几帧图像 从0开始
void VINS::processImage(map<int, Eigen::Vector3d> &image_msg, double header, int buf_num)
{
//    printf("processIMU PS0 %lf %lf %lf\n",Ps[0].x(),Ps[0].y(), Ps[0].z());
    int track_num;
//    printf("adding feature points %lu\n", image_msg.size());
    //根据视差来决定marg掉哪一帧，如果次新帧是关键帧，marg掉最老帧，如果次新帧不是关键帧，marg掉次新帧
    if (f_manager.addFeatureCheckParallax(frame_count, image_msg, track_num))//添加特征并检测
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;
    
//        printf("marginalization_flag %d\n", int(marginalization_flag));
    //    printf("this frame is-------------------------------%s\n", marginalization_flag ? "reject" : "accept");
//        printf("Solving %d\n", frame_count);
//    //计算滑窗内被track过的特征点的数量
//    printf("number of feature: %d %d\n", feature_num = f_manager.getFeatureCount(), track_num);
    
    Headers[frame_count] = header;
    
    //判断是初始化还是非线性优化
    if(solver_flag == INITIAL)
    {
        //将图像数据 时间 临时预积分值存储到图像帧类中，ImageFrame这个类的定义在initial_alignment.h中
        ImageFrame imageframe(image_msg, header);
        imageframe.pre_integration = tmp_pre_integration;
        all_image_frame.insert(make_pair(header, imageframe));
        tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0)};
        
        if (frame_count == WINDOW_SIZE)
        {
            if(track_num < 20)
            {
                clearState();
                return;
            }
            bool result = false;
            //当前帧时间戳大于上一次初始化时间戳0.3秒，就进行初始化操作*1e9 *1e9-0.00001 *1e6-0.00001
            if(header - initial_timestamp > (0.3*1e9-0.00001))//euroc
//            if(header - initial_timestamp > 0.3)
            {
                result = solveInitial();//计算位姿 陀螺仪校正 重力加速度方向 尺度
                initial_timestamp = header;//初始化时间戳更新
            }
//            初始化成功则进行一次非线性优化，不成功则进行滑窗操作
            if(result)
            {
                //先进行一次滑动窗口非线性优化，得到当前帧与第I帧的位姿
//                solve_ceres(buf_num);
                solve_ceres2(buf_num);
                if(final_cost > 200)  //initialization failed, need reinitialize200
                {
                    printf("final cost %lf faild!\n",final_cost);
                    delete last_marginalization_info;
                    last_marginalization_info = nullptr;
                    solver_flag = INITIAL;
                    init_status = FAIL_CHECK;
                    fail_times++;
                    slideWindow();
                }
                else
                {
                    printf("final cost %lf succ!\n",final_cost);
                    failure_occur = 0;
                    //update init progress
                    initProgress = 100;
                    init_status = SUCC;
                    fail_times = 0;
                    printf("Initialization finish---------------------------------------------------!\n");
//                    printf("init header: %lf\n",header);
                    solver_flag = NON_LINEAR;
                    slideWindow();//滑动窗口
                    f_manager.removeFailures();//剔除失败点
                    update_loop_correction();
                    last_R = Rs[WINDOW_SIZE];
                    last_P = Ps[WINDOW_SIZE];
                    last_R_old = Rs[0];
                    last_P_old = Ps[0];
                    

                }
            }
            else
            {
                slideWindow();
            }
        }
        else
        {
            frame_count++;
            initProgress +=2;
        }
    }
    else//非线性优化
    {
        bool is_nonlinear = true;
        f_manager.triangulate(Ps, tic, ric, is_nonlinear);//计算特征点深度estimated_depth
//        solve_ceres(buf_num);//后端位姿优化
        solve_ceres2(buf_num);
        failure_occur = 0;
        
        if (failureDetection())
        {
            failure_occur = 1;
            clearState();
            return;
        }
        slideWindow();//滑动窗口
        f_manager.removeFailures();//剔除失败点
        update_loop_correction();
        
        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R_old = Rs[0];
        last_P_old = Ps[0];
    }
}
//image_msg 第一个元素是 相机编号，代表第几帧图像 从0开始
void VINS::processImage(map<int, Eigen::Vector3d> &image_msg, double header, int buf_num, map<int, Eigen::Vector3d> &distorted_image)
{
    if(fabs(header_forwkf)>0.000001){
        int frame_inLocalWindow=-1;
        for(int i=WINDOW_SIZE-2;i>=0;i--){
            if(fabs(Headers[i]-header_forwkf)<0.000001){
                frame_inLocalWindow=i;
                break;
            }
        }
        if(frame_inLocalWindow>0){
            cout<<"添加新的数据="<<frame_inLocalWindow<<endl;
            TS(addFeature);
            priorMap_f_manager.addFeature(frame_inLocalWindow, measurements_cur_coarse_new, point_3d_old_new,feature_id_cur_new );
            measurements_cur_coarse_new.clear();
            point_3d_old_new.clear();
            feature_id_cur_new.clear();
            header_forwkf=0.0;
            TE(addFeature);
        }
    }
    
    
//    printf("processIMU PS0 %lf %lf %lf\n",Ps[0].x(),Ps[0].y(), Ps[0].z());
    int track_num;
//    printf("adding feature points %lu\n", image_msg.size());
    //根据视差来决定marg掉哪一帧，如果次新帧是关键帧，marg掉最老帧，如果次新帧不是关键帧，marg掉次新帧
    if (f_manager.addFeatureCheckParallax(frame_count, image_msg, track_num, distorted_image))//添加特征并检测
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;
    
//        printf("marginalization_flag %d\n", int(marginalization_flag));
    //    printf("this frame is-------------------------------%s\n", marginalization_flag ? "reject" : "accept");
//        printf("Solving %d\n", frame_count);
//    //计算滑窗内被track过的特征点的数量
//    printf("number of feature: %d %d\n", feature_num = f_manager.getFeatureCount(), track_num);
    
    Headers[frame_count] = header;
    
    //判断是初始化还是非线性优化
    if(solver_flag == INITIAL)
    {
        //将图像数据 时间 临时预积分值存储到图像帧类中，ImageFrame这个类的定义在initial_alignment.h中
        ImageFrame imageframe(image_msg, header);
        imageframe.pre_integration = tmp_pre_integration;
        all_image_frame.insert(make_pair(header, imageframe));
        tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0)};
        
        if (frame_count == WINDOW_SIZE)
        {
            if(track_num < 20)
            {
                clearState();
                return;
            }
            bool result = false;
            //当前帧时间戳大于上一次初始化时间戳0.3秒，就进行初始化操作*1e9 *1e9-0.00001 *1e6-0.00001
            if(header - initial_timestamp > (0.3*1e9-0.00001))
//            if(header - initial_timestamp > 0.3)
            {
                result = solveInitial();//计算位姿 陀螺仪校正 重力加速度方向 尺度
                initial_timestamp = header;//初始化时间戳更新
            }
//            初始化成功则进行一次非线性优化，不成功则进行滑窗操作
            if(result)
            {
                //先进行一次滑动窗口非线性优化，得到当前帧与第I帧的位姿
//                solve_ceres(buf_num);
                solve_ceres2(buf_num);
                if(final_cost > 200)  //initialization failed, need reinitialize200
                {
                    printf("final cost %lf faild!\n",final_cost);
                    delete last_marginalization_info;
                    last_marginalization_info = nullptr;
                    solver_flag = INITIAL;
                    init_status = FAIL_CHECK;
                    fail_times++;
                    slideWindow();
                }
                else
                {
                    printf("final cost %lf succ!\n",final_cost);
                    failure_occur = 0;
                    //update init progress
                    initProgress = 100;
                    init_status = SUCC;
                    fail_times = 0;
                    printf("Initialization finish---------------------------------------------------!\n");
//                    printf("init header: %lf\n",header);
                    solver_flag = NON_LINEAR;
                    slideWindow();//滑动窗口
                    f_manager.removeFailures();//剔除失败点
                    update_loop_correction();
                    last_R = Rs[WINDOW_SIZE];
                    last_P = Ps[WINDOW_SIZE];
                    last_R_old = Rs[0];
                    last_P_old = Ps[0];
                    

                }
            }
            else
            {
                slideWindow();
            }
        }
        else
        {
            frame_count++;
            initProgress +=2;
        }
    }
    else//非线性优化
    {
        bool is_nonlinear = true;
        f_manager.triangulate(Ps, tic, ric, is_nonlinear);//计算特征点深度estimated_depth
//        solve_ceres(buf_num);//后端位姿优化
        solve_ceres2(buf_num);
        failure_occur = 0;
        
        if (failureDetection())
        {
            failure_occur = 1;
            clearState();
            return;
        }
        slideWindow();//滑动窗口
        f_manager.removeFailures();//剔除失败点
        update_loop_correction();
        
        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R_old = Rs[0];
        last_P_old = Ps[0];
    }
}
//非线性优化
//优化小滑窗内的PVQ，不优化路标点，涉及两个约束：IMU帧间约束，每一帧的PnP的视觉重投影误差约束
//备份一份
void VINS::solve_ceres(int buf_num)
{
    //1.构建最小二乘问题
    ceres::Problem problem;
    //2.创建LossFunction对象，lossfunction用来减小Outlier的影响
    ceres::LossFunction *loss_function;
    
    loss_function = new ceres::CauchyLoss(1.0);
    //3.添加优化变量参数块
    //添加要优化的变量：相机位姿、速度、加速度偏差、陀螺仪偏差
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        //para_Pose[i]中存放的是滑动窗口中第i帧的位姿，para_Pose[i]的大小为SIZE_POSE(值为7)
        //local_parameterization 参数更新方式需要另外定义 eg:四元数的广义加法
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        //SIZE_SPEEDBIAS值为9
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    //添加要优化的变量：相机到IMU的外参
    //但是这里设置了常量 不优化
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    
    for (int i = 0; i < NUM_OF_F; i++)
    {
        problem.AddParameterBlock(para_Feature[i], SIZE_FEATURE);
    }
    //将要优化的变量转为数组形式
    old2new();
    
    //marginalization factor
    //4.添加残差块
    //添加边缘化的先验残差信息，第一次进行优化的时候last_marginalization_info还为空值
    if (last_marginalization_info != nullptr)
    {
//        AddResidualBlock函数中第一个参数是costfunction，第二个参数是lossfunction，第三个参数是参数块
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        //TODO 不满足parameter_blocks.size() == cost_function->parameter_block_sizes().size()
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    
    //IMU factor
    //添加IMU测量值残差
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
    
    //projection factor
    int f_m_cnt = 0;
    double f_sum = 0.0;
    double r_f_sum = 0.0;
    int feature_index = -1;
    //遍历特征构建视觉重投影残差
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        ++feature_index;
        
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        //第一个观测到该特征的帧对应的特征点坐标
        Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        //遍历能观测到该特征的每个帧
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            
            if (imu_i == imu_j)
            {
                continue;
            }
            Eigen::Vector3d pts_j = it_per_frame.point;
            
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            
            f_m_cnt++;
            
            double **para = new double *[4];
            para[0] = para_Pose[imu_i];
            para[1] = para_Pose[imu_j];
            para[2] = para_Ex_Pose[0];
            para[3] = para_Feature[feature_index];
            double *res = new double[2];
            f->Evaluate(para, res, NULL);
            f_sum += sqrt(res[0] * res[0] + res[1] * res[1]);
            
            double rho[3];
            loss_function->Evaluate(res[0] * res[0] + res[1] * res[1], rho);
            r_f_sum += rho[0];
        }
    }
    visual_cost = r_f_sum;
    visual_factor_num = f_m_cnt;
    //重定位残差
    //是单目VIO维持的当前滑动窗口与过去的位姿图对齐
    //将所有回环帧的位姿作为常量，利用所有IMU测量值，局部视觉测量和从回环中提取特征对应值，共同优化滑动窗口
    //消除错误的回环
    if(LOOP_CLOSURE)
    {
        //loop close factor
        //front_pose.measurements.clear();
        if(!(fabs(front_pose.header - retrive_pose_data.header)<0.00001))
//        if(front_pose.header != retrive_pose_data.header)
        {
            front_pose = retrive_pose_data;  //need lock
            printf("use loop\n");
        }
        if(!front_pose.measurements.empty())
        {
            //the retrive pose is in the current window
            if((front_pose.header - Headers[0])>-0.00001)
//            if(front_pose.header >= Headers[0])
            {
                //tmp_retrive_pose_buf.push(front_pose);
                printf("loop front pose  in window\n");
                for(int i = 0; i < WINDOW_SIZE; i++)
                {
                    if(fabs(front_pose.header - Headers[i])<0.00001)
//                    if(front_pose.header == Headers[i])//找到匹配帧 在窗口中的位置i
                    {
//                        cout<<"youhua qian front_pose.loop_pose："<<endl;
//                        for (int k = 0; k < 7; k++){
//                            front_pose.loop_pose[k] = para_Pose[i][k];//把回环优化后的位姿 告诉回环记录的
//                        }
                        
                        Eigen::Quaterniond loop_q=front_pose.Q_old;
                        Eigen::Vector3d loop_t=front_pose.P_old;
                        for (int k = 0; k < 3; k++){
                            front_pose.loop_pose[k] =loop_t[k];//把回环优化后的位姿 告诉回环记录的
                        }
                        front_pose.loop_pose[3]=loop_q.x();
                        front_pose.loop_pose[4]=loop_q.y();
                        front_pose.loop_pose[5]=loop_q.z();
                        front_pose.loop_pose[6]=loop_q.w();
                        
                        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
                        problem.AddParameterBlock(front_pose.loop_pose, SIZE_POSE, local_parameterization);//要优化的变量
                        
                        int retrive_feature_index = 0;
                        int feature_index = -1;
                        int loop_factor_cnt = 0;
                        //遍历特征
                        for (auto &it_per_id : f_manager.feature)
                        {
                            it_per_id.used_num = it_per_id.feature_per_frame.size();
                            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                                continue;
                            ++feature_index;
                            //获取观测到该特征的起始帧
                            int start = it_per_id.start_frame;
                            //feature has been obeserved in ith frame
                            int end = (int)(start + it_per_id.feature_per_frame.size() - i - 1);//能被当前帧看到
                            if(start <= i && end >=0)//第i帧能看到这个特征点
                            {
                                while(front_pose.features_ids[retrive_feature_index] < it_per_id.feature_id)
                                {
                                    retrive_feature_index++;
                                }
                                
                                if(front_pose.features_ids[retrive_feature_index] == it_per_id.feature_id)
                                {
                                    Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements[retrive_feature_index].x, front_pose.measurements[retrive_feature_index].y, 1.0);//回环帧上的点的坐标
                                    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                                    //double ratio = 1.0;
                                    
//                                    cout<<"测试一下 pts_j pts_i:"<<pts_j[0]<<" "<<pts_j[1]<<" "<<pts_j[2]<<" "<<pts_i[0]<<" "<<pts_i[1]<<" "<<pts_i[2]<<endl;
//                                    cout<<"测试一下 para_Pose[start]:"<<para_Pose[start][0]<<" "<<para_Pose[start][1]<<" "<<para_Pose[start][2]<<" "<<para_Pose[start][3]<<" "<<para_Pose[start][4]<<" "<<para_Pose[start][5]<<" "<<para_Pose[start][6]<<endl;
//                                    cout<<"测试一下 para_Feature[feature_index]:"<<para_Feature[feature_index][0]<<endl;
                                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);//匹配点和被观测到的第一帧之间的重投影残差
                                    problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);
                                    
                                    //printf("loop add factor %d %d %lf %lf %d\n",retrive_feature_index,feature_index,
                                    //                                         pts_j.x(), pts_i.x(),front_pose.features_ids.size());
                                    retrive_feature_index++;
                                    loop_factor_cnt++;
                                    loop_enable = true;
                                }
                                
                            }
                        }
//                        printf("add %d loop factor\n", loop_factor_cnt);
                    }
                }
            }
        }
    }
    
   else if(LOOP_CLOSURE_SERVER)
    {
        //loop close factor
        //front_pose.measurements.clear();
        if(!retrive_pose_data_server.empty()){
//            cout<<"retrive_pose_data_server 不空"<<endl;
            if(!(fabs(front_pose.header - retrive_pose_data_server.front().header)<0.00001))
//            if(front_pose.header != retrive_pose_data_server.front().header)
            {
                if((front_pose.header - Headers[0])>-0.00001 && front_pose.isRemove!=0){
                    sendServer_relative=true;
                    relative_q_sendServer=front_pose.relative_q;
                    relative_t_sendServer=front_pose.relative_t;
                    relative_yaw_sendServer=front_pose.relative_yaw;
                    relative_cur_index_sendServer=front_pose.cur_index;
                    relative_pitch_sendServer=front_pose.relative_pitch;
                    relative_roll_sendServer=front_pose.relative_roll;
                    isRemove_sendServer=front_pose.isRemove;
                }
                
                Eigen::Matrix<double, 8, 1> connected_info_test;
                connected_info_test <<relative_t_sendServer.x(), relative_t_sendServer.y(), relative_t_sendServer.z(),
                relative_q_sendServer.w(), relative_q_sendServer.x(), relative_q_sendServer.y(), relative_q_sendServer.z(),relative_yaw_sendServer;
//                cout<<"relative_t_sendServer:"<<connected_info_test(0)<<" "<<connected_info_test(1)<<" "<<connected_info_test(2)<<" "<<connected_info_test(7)<<" header"<<front_pose.header<<endl;
                
                
                front_pose = retrive_pose_data_server.front();  //need lock
                retrive_pose_data_server.pop();
                
                if(!((front_pose.header - Headers[0])>-0.00001)){
                    int num_loop=retrive_pose_data_server.size();
                    for(int i=0;i<num_loop;i++){
                        front_pose=retrive_pose_data_server.front();
                        retrive_pose_data_server.pop();
                        if((front_pose.header - Headers[0])>-0.00001)
                            break;
                    }
                }
                front_pose.isRemove=2;
//                front_pose.sendRelativeData_server=true;
                
//                cout<<"front_pose test cur "<<front_pose.cur_index<<" old_index "<<front_pose.old_index<<" "<<front_pose.measurements.size()<<endl;
                printf("use loop\n");
            }
        }
        if(!front_pose.measurements.empty() && front_pose.isRemove!=0)
        {
//            cout<<fixed<<setprecision(1)<<"测试 ："<<front_pose.header<<" "<<Headers[0]<<endl;
            //the retrive pose is in the current window
            if((front_pose.header - Headers[0])>-0.00001)
//            if(front_pose.header >= Headers[0])
            {
                //tmp_retrive_pose_buf.push(front_pose);
                printf("loop front pose  in window\n");
                for(int i = 0; i < WINDOW_SIZE; i++)
                {
                    if(fabs(front_pose.header - Headers[i])<0.00001)
//                    if(front_pose.header == Headers[i])
                    {
//                        for (int k = 0; k < 7; k++)
//                            front_pose.loop_pose[k] = para_Pose[i][k];
                        
                        Eigen::Quaterniond loop_q=front_pose.Q_old;
                        Eigen::Vector3d loop_t=front_pose.P_old;
                        for (int k = 0; k < 3; k++){
                            front_pose.loop_pose[k] =loop_t[k];//把回环优化后的位姿 告诉回环记录的
                        }
                        front_pose.loop_pose[3]=loop_q.x();
                        front_pose.loop_pose[4]=loop_q.y();
                        front_pose.loop_pose[5]=loop_q.z();
                        front_pose.loop_pose[6]=loop_q.w();
                        
                        
                        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
                        problem.AddParameterBlock(front_pose.loop_pose, SIZE_POSE, local_parameterization);
                        
                        int retrive_feature_index = 0;
                        int feature_index = -1;
                        int loop_factor_cnt = 0;
                        //遍历特征
                        for (auto &it_per_id : f_manager.feature)
                        {
                            it_per_id.used_num = it_per_id.feature_per_frame.size();
                            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                                continue;
                            ++feature_index;
                            //获取观测到该特征的起始帧
                            int start = it_per_id.start_frame;
                            //feature has been obeserved in ith frame
                            int end = (int)(start + it_per_id.feature_per_frame.size() - i - 1);
                            if(start <= i && end >=0)
                            {
                                while(front_pose.features_ids[retrive_feature_index] < it_per_id.feature_id)
                                {
                                    retrive_feature_index++;
                                }
                                
                                if(front_pose.features_ids[retrive_feature_index] == it_per_id.feature_id)
                                {
                                    Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements[retrive_feature_index].x, front_pose.measurements[retrive_feature_index].y, 1.0);
                                    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                                    //double ratio = 1.0;
                                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                                    problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);
                                    
                                    //printf("loop add factor %d %d %lf %lf %d\n",retrive_feature_index,feature_index,
                                    //                                         pts_j.x(), pts_i.x(),front_pose.features_ids.size());
                                    retrive_feature_index++;
                                    loop_factor_cnt++;
                                    loop_enable = true;
                                }
                                
                            }
                        }
//                        printf("add %d loop factor\n", loop_factor_cnt);
                    }
                }
            }
        }
    }
   else if(LOOP_CLOSURE_SERVER_noLoop)
    {
        //loop close factor
        //front_pose.measurements.clear();
        if(!retrive_pose_data_server.empty()){
//            cout<<"retrive_pose_data_server 不空"<<endl;
            if(!(fabs(front_pose.header - retrive_pose_data_server.front().header)<0.00001))
//            if(front_pose.header != retrive_pose_data_server.front().header)
            {
                if((front_pose.header - Headers[0])>-0.00001 && front_pose.isRemove!=0){
                    sendServer_relative=true;
                    relative_q_sendServer=front_pose.relative_q;
                    relative_t_sendServer=front_pose.relative_t;
                    relative_yaw_sendServer=front_pose.relative_yaw;
                    relative_cur_index_sendServer=front_pose.cur_index;
                    relative_pitch_sendServer=front_pose.relative_pitch;
                    relative_roll_sendServer=front_pose.relative_roll;
                    isRemove_sendServer=front_pose.isRemove;
                }
                
                Eigen::Matrix<double, 8, 1> connected_info_test;
                connected_info_test <<relative_t_sendServer.x(), relative_t_sendServer.y(), relative_t_sendServer.z(),
                relative_q_sendServer.w(), relative_q_sendServer.x(), relative_q_sendServer.y(), relative_q_sendServer.z(),relative_yaw_sendServer;
//                cout<<"relative_t_sendServer:"<<connected_info_test(0)<<" "<<connected_info_test(1)<<" "<<connected_info_test(2)<<" "<<connected_info_test(7)<<" header"<<front_pose.header<<endl;
                
                
                front_pose = retrive_pose_data_server.front();  //need lock
                retrive_pose_data_server.pop();
                
                if(!((front_pose.header - Headers[0])>-0.00001)){
                    int num_loop=retrive_pose_data_server.size();
                    for(int i=0;i<num_loop;i++){
                        front_pose=retrive_pose_data_server.front();
                        retrive_pose_data_server.pop();
                        if((front_pose.header - Headers[0])>-0.00001)
                            break;
                    }
                }
                front_pose.isRemove=2;
//                front_pose.sendRelativeData_server=true;
                
//                cout<<"front_pose test cur "<<front_pose.cur_index<<" old_index "<<front_pose.old_index<<" "<<front_pose.measurements.size()<<endl;
                printf("use loop\n");
            }
        }
        if(!front_pose.measurements.empty() && front_pose.isRemove!=0)
        {
//            cout<<fixed<<setprecision(1)<<"测试 ："<<front_pose.header<<" "<<Headers[0]<<endl;
            //the retrive pose is in the current window
            if((front_pose.header - Headers[0])>-0.00001)
//            if(front_pose.header >= Headers[0])
            {
                //tmp_retrive_pose_buf.push(front_pose);
                printf("loop front pose  in window\n");
                for(int i = 0; i < WINDOW_SIZE; i++)
                {
                    if(fabs(front_pose.header - Headers[i])<0.00001)
//                    if(front_pose.header == Headers[i])
                    {
//                        for (int k = 0; k < 7; k++)
//                            front_pose.loop_pose[k] = para_Pose[i][k];
                        
                        Eigen::Quaterniond loop_q=front_pose.Q_old;
                        Eigen::Vector3d loop_t=front_pose.P_old;
                        for (int k = 0; k < 3; k++){
                            front_pose.loop_pose[k] =loop_t[k];//把回环优化后的位姿 告诉回环记录的
                        }
                        front_pose.loop_pose[3]=loop_q.x();
                        front_pose.loop_pose[4]=loop_q.y();
                        front_pose.loop_pose[5]=loop_q.z();
                        front_pose.loop_pose[6]=loop_q.w();
                        
                        
                        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
                        problem.AddParameterBlock(front_pose.loop_pose, SIZE_POSE, local_parameterization);
                        
                        int retrive_feature_index = 0;
                        int feature_index = -1;
                        int loop_factor_cnt = 0;
                        //遍历特征
                        for (auto &it_per_id : f_manager.feature)
                        {
                            it_per_id.used_num = it_per_id.feature_per_frame.size();
                            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                                continue;
                            ++feature_index;
                            //获取观测到该特征的起始帧
                            int start = it_per_id.start_frame;
                            //feature has been obeserved in ith frame
                            int end = (int)(start + it_per_id.feature_per_frame.size() - i - 1);
                            if(start <= i && end >=0)
                            {
                                while(front_pose.features_ids[retrive_feature_index] < it_per_id.feature_id)
                                {
                                    retrive_feature_index++;
                                }
                                
                                if(front_pose.features_ids[retrive_feature_index] == it_per_id.feature_id)
                                {
                                    Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements[retrive_feature_index].x, front_pose.measurements[retrive_feature_index].y, 1.0);
                                    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                                    //double ratio = 1.0;
                                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                                    problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);
                                    
                                    //printf("loop add factor %d %d %lf %lf %d\n",retrive_feature_index,feature_index,
                                    //                                         pts_j.x(), pts_i.x(),front_pose.features_ids.size());
                                    retrive_feature_index++;
                                    loop_factor_cnt++;
                                    loop_enable = true;
                                }
                                
                            }
                        }
//                        printf("add %d loop factor\n", loop_factor_cnt);
                    }
                }
            }
        }
    }
    //5.创建优化求解器
    ceres::Solver::Options options;
    //6.设定优化器的solver_type类型
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 1;
    //7.设定优化使用的算法。这里使用置信域类优化算法中的dogleg方法
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = true;
    options.logging_type=ceres::SILENT;
    //8.设置迭代求解的次数
    options.max_num_iterations = 10;//10
    //options.use_nonmonotonic_steps = true;
    if(buf_num<2)
        options.max_solver_time_in_seconds = SOLVER_TIME;
    else if(buf_num<4)
        options.max_solver_time_in_seconds = SOLVER_TIME * 2.0 / 3.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME / 2.0;
    
    ceres::Solver::Summary summary;//优化信息
    //TE(prepare_solver);
//    TS(ceres);
//    printf("solve\n");
    
//    for(int i = 0; i< WINDOW_SIZE; i++)
//    {
//        if(front_pose.header == Headers[i])
//        {
//            cout<<"vins.front_pose: before1"<<endl;
//            for (int k = 0; k < 7; k++){
//                cout<<para_Pose[0][k]<<" "<<front_pose.loop_pose[k]<<endl;
//            }
//        }
//    }
    
    
    //9.开始求解
    ceres::Solve(options, &problem, &summary);
//    cout<<"solve end:"<<summary.termination_type<<endl;
    final_cost = summary.final_cost;
//    cout << summary.FullReport() << endl;
//    TE(ceres);
    
//    if(options.IsValid(&)){
//        printf("ceres solve is valid\n");
//    }else{
//        printf("ceres solve is not valid\n");
//    }
    
    
    
//    cout<<"vins.front_pose: before2"<<endl;
//    for (int k = 0; k < 7; k++){
//        cout<<para_Pose[0][k]<<" "<<front_pose.loop_pose[k]<<endl;
//    }
    

    
    if(LOOP_CLOSURE)
    {
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
            if(fabs(front_pose.header - Headers[i])<0.00001)
//            if(front_pose.header == Headers[i])
            {
                Eigen::Matrix3d Rs_i = Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
                Eigen::Vector3d Ps_i = Eigen::Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);//当前帧
                

                
                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4], front_pose.loop_pose[5]).normalized().toRotationMatrix();
                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
                
                front_pose.relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
                front_pose.relative_q = Rs_loop.transpose() * Rs_i;
                front_pose.relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                
              //ljl
                front_pose.relative_pitch=Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                front_pose.relative_roll=Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
//
                
//                cout<<"relative_t:"<<front_pose.relative_t.norm()<<endl;
//                cout<<"真正的relative_yaw:"<<front_pose.relative_yaw<<" "<<front_pose.cur_index<<" "<< front_pose.old_index<<endl;
                
            }
        }
    }
    
    else if(LOOP_CLOSURE_SERVER)
    {
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
            if(fabs(front_pose.header - Headers[i])<0.00001)
//            if(front_pose.header == Headers[i])
            {
                Eigen::Matrix3d Rs_i = Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
                Eigen::Vector3d Ps_i = Eigen::Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
                

                
                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
                
                front_pose.relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
                front_pose.relative_q = Rs_loop.transpose() * Rs_i;
                front_pose.relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                
                
                //ljl
                front_pose.relative_pitch=Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                front_pose.relative_roll=Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
//                                cout<<"relative_t:"<<front_pose.relative_t.norm()<<endl;
                //                cout<<"relative_q:"<<front_pose.relative_q<<endl;
//                                cout<<"relative_yaw:"<<front_pose.relative_yaw<<endl;
            }
        }
    }
    else if(LOOP_CLOSURE_SERVER_noLoop)
    {
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
            if(fabs(front_pose.header - Headers[i])<0.00001)
//            if(front_pose.header == Headers[i])
            {
                Eigen::Matrix3d Rs_i = Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
                Eigen::Vector3d Ps_i = Eigen::Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
                

                
                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
                
                front_pose.relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
                front_pose.relative_q = Rs_loop.transpose() * Rs_i;
                front_pose.relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                
                
                //ljl
                front_pose.relative_pitch=Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                front_pose.relative_roll=Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
//                                cout<<"relative_t:"<<front_pose.relative_t.norm()<<endl;
                //                cout<<"relative_q:"<<front_pose.relative_q<<endl;
//                                cout<<"relative_yaw:"<<front_pose.relative_yaw<<endl;
            }
        }
    }
    //求解完成后，将数组转化为向量
    new2old();
    
    vector<ceres::ResidualBlockId> residual_set;
    problem.GetResidualBlocks(&residual_set);
    for (auto it : residual_set)
        problem.RemoveResidualBlock(it);
    
    //for marginalization back//判断边缘化标志
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        old2new();
        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        
        {
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        
        {
            int feature_index = -1;
            //遍历特征
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;
                
                ++feature_index;
                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;
                //当前特征所在的第一个观测帧中观测的点坐标
                Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                //遍历当前特征的观测帧
                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    
                    Eigen::Vector3d pts_j = it_per_frame.point;
                    if (imu_i == imu_j)
                        continue;
                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                   vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                   vector<int>{0, 3});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                    
                }
            }
        }
        
//        TS(per_marginalization);
        marginalization_info->preMarginalize(); //??
//        TE(per_marginalization);
//        TS(marginalization);
        marginalization_info->marginalize();   //??
//        TE(marginalization);
        
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    //marginalize front
    else
    {
        if (last_marginalization_info&&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            old2new();
//            当次新帧不是关键帧时，直接剪切掉次新帧和它的视觉观测边（该帧和路标点之间的关联），而不对次新帧进行marginalize处理
//            但是要保留次新帧的IMU数据，从而保证IMU预积分的连续性，这样才能积分计算出下一帧的测量值
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    assert(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);
                
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
            //计算每次IMU和视觉观测(cost_function)对应的参数块(parameter_blocks),雅可比矩阵(jacobians),残差值(residuals)
            marginalization_info->preMarginalize();
            //多线程计算整个先验项的参数块,雅可比矩阵和残差值
            marginalization_info->marginalize();
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
}

//先测试 当前帧的3d点 和 老帧的2d点
void VINS::solve_ceres2(int buf_num)
{
    //1.构建最小二乘问题
    ceres::Problem problem;
    //2.创建LossFunction对象，lossfunction用来减小Outlier的影响
    ceres::LossFunction *loss_function;
    
    loss_function = new ceres::CauchyLoss(1.0);
    //3.添加优化变量参数块
    //添加要优化的变量：相机位姿、速度、加速度偏差、陀螺仪偏差
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        //para_Pose[i]中存放的是滑动窗口中第i帧的位姿，para_Pose[i]的大小为SIZE_POSE(值为7)
        //local_parameterization 参数更新方式需要另外定义 eg:四元数的广义加法
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        //SIZE_SPEEDBIAS值为9
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }

    //添加要优化的变量：相机到IMU的外参
    //但是这里设置了常量 不优化
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    
    for (int i = 0; i < NUM_OF_F; i++)
    {
        problem.AddParameterBlock(para_Feature[i], SIZE_FEATURE);
    }
    //将要优化的变量转为数组形式
    old2new();
    
    //marginalization factor
    //4.添加残差块
    //添加边缘化的先验残差信息，第一次进行优化的时候last_marginalization_info还为空值
    if (last_marginalization_info != nullptr)
    {
//        AddResidualBlock函数中第一个参数是costfunction，第二个参数是lossfunction，第三个参数是参数块
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        //TODO 不满足parameter_blocks.size() == cost_function->parameter_block_sizes().size()
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    
    //IMU factor
    //添加IMU测量值残差
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
    
    //projection factor
    int f_m_cnt = 0;
    double f_sum = 0.0;
    double r_f_sum = 0.0;
    int feature_index = -1;
    //ljl
    edge=last_marginalization_parameter_blocks.size();
    edge_single=0;
    //遍历特征构建视觉重投影残差
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        ++feature_index;
        
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        //第一个观测到该特征的帧对应的特征点坐标
        Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        //遍历能观测到该特征的每个帧
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            
            if (imu_i == imu_j)
            {
                continue;
            }
            Eigen::Vector3d pts_j = it_per_frame.point;
            
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            
            f_m_cnt++;
            
            double **para = new double *[4];
            para[0] = para_Pose[imu_i];
            para[1] = para_Pose[imu_j];
            para[2] = para_Ex_Pose[0];
            para[3] = para_Feature[feature_index];
            double *res = new double[2];
            f->Evaluate(para, res, NULL);
            f_sum += sqrt(res[0] * res[0] + res[1] * res[1]);
            
            double rho[3];
            loss_function->Evaluate(res[0] * res[0] + res[1] * res[1], rho);
            r_f_sum += rho[0];
            
            edge++;
            if(imu_j==(WINDOW_SIZE - 2)){
                edge_single++;
            }
        }
    }
    visual_cost = r_f_sum;
    visual_factor_num = f_m_cnt;
    //重定位残差
    //是单目VIO维持的当前滑动窗口与过去的位姿图对齐
    //将所有回环帧的位姿作为常量，利用所有IMU测量值，局部视觉测量和从回环中提取特征对应值，共同优化滑动窗口
    //消除错误的回环
    
//    if(LOOP_CLOSURE)
//    {
//        //loop close factor
//        //front_pose.measurements.clear();
//        if(!(fabs(front_pose.header - retrive_pose_data.header)<0.00001))
////        if(front_pose.header != retrive_pose_data.header)
//        {
//            for(int aa=0,bb=front_pose.loop_pose_all.size();aa<<bb;aa++){
//                delete [](front_pose.loop_pose_all[aa]);
//            }
//            front_pose = retrive_pose_data;  //need lock
//            printf("use loop\n");
//        }
////        if(!front_pose.measurements.empty())
//        if(!front_pose.measurements_all.empty())
//        {
//            //the retrive pose is in the current window
////            if((front_pose.header - Headers[0])>-0.00001)
////            if(front_pose.header >= Headers[0])
////            临时测试一下
//                for(int a=0,b=front_pose.header_all.size();a<b;a++){
//                    double front_pose_header=front_pose.header_all[a];
//
//                    Eigen::Quaterniond loop_q=front_pose.Q_old_all[a];
//                    Eigen::Vector3d loop_t=front_pose.P_old_all[a];
//                    double* loop_pose_flag=new double[7];
//                    for (int k = 0; k < 3; k++){
//                        loop_pose_flag[k]=loop_t[k];//把回环优化后的位姿 告诉回环记录的
//                    }
//                    loop_pose_flag[3]=loop_q.x();
//                    loop_pose_flag[4]=loop_q.y();
//                    loop_pose_flag[5]=loop_q.z();
//                    loop_pose_flag[6]=loop_q.w();
//                    front_pose.loop_pose_all.push_back(loop_pose_flag);
//
//                    /**
//                    if((front_pose_header - Headers[0])>-0.00001){
//                        //tmp_retrive_pose_buf.push(front_pose);
//                        printf("loop front pose  in window\n");
//                        for(int i = 0; i < WINDOW_SIZE; i++)
//                        {
////                            if(fabs(front_pose.header - Headers[i])<0.00001)
//        //                    if(front_pose.header == Headers[i])//找到匹配帧 在窗口中的位置i
//                                if(fabs(front_pose_header - Headers[i])<0.00001)
//                            {
//
////                                for (int k = 0; k < 7; k++){
////                                    front_pose.loop_pose[k] = para_Pose[i][k];//把回环优化后的位姿 告诉回环记录的
////                                }
//
////                                Eigen::Quaterniond loop_q=front_pose.Q_old;
////                                Eigen::Vector3d loop_t=front_pose.P_old;
////
////                                for (int k = 0; k < 3; k++){
////                                    front_pose.loop_pose[k] =loop_t[k];//把回环优化后的位姿 告诉回环记录的
////                                }
////                                front_pose.loop_pose[3]=loop_q.x();
////                                front_pose.loop_pose[4]=loop_q.y();
////                                front_pose.loop_pose[5]=loop_q.z();
////                                front_pose.loop_pose[6]=loop_q.w();
//
//
//                                ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
////                                problem.AddParameterBlock(front_pose.loop_pose, SIZE_POSE, local_parameterization);//要优化的变量
//
//                                problem.AddParameterBlock(front_pose.loop_pose_all[a], SIZE_POSE, local_parameterization);//要优化的变量
//        //                        ljl
//                                problem.SetParameterBlockConstant(front_pose.loop_pose_all[a]);
//
//                                int retrive_feature_index = 0;
//                                int feature_index = -1;
//                                int loop_factor_cnt = 0;
////                                遍历特征
//                                for (auto &it_per_id : f_manager.feature)
//                                {
//                                    it_per_id.used_num = it_per_id.feature_per_frame.size();
//                                    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
//                                        continue;
//                                    ++feature_index;
//                                    //获取观测到该特征的起始帧
//                                    int start = it_per_id.start_frame;
//                                    //feature has been obeserved in ith frame
//                                    int end = (int)(start + it_per_id.feature_per_frame.size() - i - 1);//能被当前帧看到
//                                    if(start <= i && end >=0)//第i帧能看到这个特征点
//                                    {
//                                        while(front_pose.features_ids_all[a][retrive_feature_index]==-1){
////                                            没有跟踪上的点 地图中有
//
//
//                                            Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements_all[a][retrive_feature_index].x, front_pose.measurements_all[a][retrive_feature_index].y, 1.0);//回环帧上的点的坐标
//                                            Eigen::Vector3d pts_i = front_pose.point_clouds_all[a][retrive_feature_index];
//
//
//                                            ceres::CostFunction* cost_function = ProjectionFactor_inMap::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), pts_j.z());
//                                            problem.AddResidualBlock(cost_function, NULL, para_Pose[i],para_Ex_Pose[0]);
//
//                                            retrive_feature_index++;
//                                        }
////                                        while(front_pose.features_ids[retrive_feature_index] < it_per_id.feature_id)
//                                            while(front_pose.features_ids_all[a][retrive_feature_index] < it_per_id.feature_id)
//                                        {
////                                            没有跟踪上的点 地图中有
//                                            Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements_all[a][retrive_feature_index].x, front_pose.measurements_all[a][retrive_feature_index].y, 1.0);//回环帧上的点的坐标
//                                            Eigen::Vector3d pts_i = front_pose.point_clouds_all[a][retrive_feature_index];
//
//
//                                            ceres::CostFunction* cost_function = ProjectionFactor_inMap::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), pts_j.z());
//                                            problem.AddResidualBlock(cost_function, NULL, para_Pose[i],para_Ex_Pose[0]);
//                                            retrive_feature_index++;
//                                        }
//
////                                        if(front_pose.features_ids[retrive_feature_index] == it_per_id.feature_id)
//                                            if(front_pose.features_ids_all[a][retrive_feature_index] == it_per_id.feature_id)
//                                        {
////                                            定义一个误差 3d点位置相同
//                                            Eigen::Vector3d pts_gt=front_pose.point_clouds_all[a][retrive_feature_index];
//                                            Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
//                                            ceres::CostFunction* cost_function = ProjectionFactor_replace::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_gt.x(), pts_gt.y(), pts_gt.z());
//                                            problem.AddResidualBlock(cost_function, NULL, para_Pose[start],para_Ex_Pose[0],
//                                                                     para_Feature[feature_index]);
//
////                                            Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements[retrive_feature_index].x, front_pose.measurements[retrive_feature_index].y, 1.0);//回环帧上的点的坐标
//
////                                            Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements_all[a][retrive_feature_index].x, front_pose.measurements_all[a][retrive_feature_index].y, 1.0);//回环帧上的点的坐标
////                                            Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
////
////
////
////                                            //double ratio = 1.0;
////
////                                            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);//匹配点和被观测到的第一帧之间的重投影残差
//////                                            problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);
////                                            problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose_all[a], para_Ex_Pose[0], para_Feature[feature_index]);
//
//
//                                            retrive_feature_index++;
//                                            loop_factor_cnt++;//没什么用 只是记录
//                                            loop_enable = true;
//                                        }
//
//                                    }
//                                }
//                                printf("add %d loop factor\n", loop_factor_cnt);
//
////                                loop_enable = true;
//
////                                遍历特征
////                                for (int c=0,d=front_pose.features_ids_all[a].size();c<d;c++)
////                                {
////                                    if(front_pose.features_ids_all[a][c]==-1){
////    //                                            没有跟踪上的点 地图中有 所以需要添加到滑动窗口 还需要提取是哪个帧观测到它了 并且记录这个点的3d不需要优化
////    //                                            feature_tracker->addPoints();
////    //                                            for (unsigned int i = 0;; i++)
////    //                                            {
////    //                                                bool completed = false;
////    //                                                completed |= feature_tracker->updateID(i);
////    //                                                if (!completed)
////    //                                                    break;
////    //                                            }
////
////                                        FeaturePerFrame f_per_fra(front_pose.point_clouds_all[a][c]);//存3d点
////                                        f_manager.feature.push_back(FeaturePerId(feature_id, frame_count)); //give id and start frame
////                                        f_manager.feature.back().feature_per_frame.push_back(f_per_fra);
////    //                                            设法让feature_tracker跟踪上
////
////                                    }
////                                }
//                            }
//                        }
//
//                    }
//                     */
//                }
//
//        }
//    }
//
    vector<int> kf_id_inLocal;
     if(LOOP_CLOSURE_SERVER)
    {
        //loop close factor
        //front_pose.measurements.clear();
//        确认是哪个front_pose
        if(!retrive_pose_data_server.empty()){
//            cout<<"retrive_pose_data_server 不空"<<endl;
            if(!(fabs(front_pose.header - retrive_pose_data_server.front().header)<0.00001))
//            if(front_pose.header != retrive_pose_data_server.front().header)
            {
                if((front_pose.header - Headers[0])>-0.00001 && front_pose.isRemove!=0){
                    sendServer_relative=true;
                    relative_q_sendServer=front_pose.relative_q;
                    relative_t_sendServer=front_pose.relative_t;
                    relative_yaw_sendServer=front_pose.relative_yaw;
                    relative_cur_index_sendServer=front_pose.cur_index;
                    relative_pitch_sendServer=front_pose.relative_pitch;
                    relative_roll_sendServer=front_pose.relative_roll;
                    isRemove_sendServer=front_pose.isRemove;
                }

                Eigen::Matrix<double, 8, 1> connected_info_test;
                connected_info_test <<relative_t_sendServer.x(), relative_t_sendServer.y(), relative_t_sendServer.z(),
                relative_q_sendServer.w(), relative_q_sendServer.x(), relative_q_sendServer.y(), relative_q_sendServer.z(),relative_yaw_sendServer;
//                cout<<"relative_t_sendServer:"<<connected_info_test(0)<<" "<<connected_info_test(1)<<" "<<connected_info_test(2)<<" "<<connected_info_test(7)<<" header"<<front_pose.header<<endl;


                front_pose = retrive_pose_data_server.front();  //need lock
                retrive_pose_data_server.pop();

                if(!((front_pose.header - Headers[0])>-0.00001)){
                    int num_loop=retrive_pose_data_server.size();
                    for(int i=0;i<num_loop;i++){
                        front_pose=retrive_pose_data_server.front();
                        retrive_pose_data_server.pop();
                        if((front_pose.header - Headers[0])>-0.00001)
                            break;
                    }
                }
                front_pose.isRemove=2;
//                front_pose.sendRelativeData_server=true;

//                cout<<"front_pose test cur "<<front_pose.cur_index<<" old_index "<<front_pose.old_index<<" "<<front_pose.measurements.size()<<endl;
                printf("use loop\n");
//                cout<<"数据不应该为空"<<front_pose.point_clouds_all.empty()<<endl;
            }
        }
        if(!front_pose.measurements.empty() && front_pose.isRemove!=0)
        {
//            cout<<fixed<<setprecision(1)<<"测试 ："<<front_pose.header<<" "<<Headers[0]<<endl;
            //the retrive pose is in the current window
            if((front_pose.header - Headers[0])>-0.00001)
//            if(front_pose.header >= Headers[0])
            {
                //tmp_retrive_pose_buf.push(front_pose);
                printf("loop front pose  in window\n");
                cout<<front_pose.loop_pose[3]<<" , "<<front_pose.loop_pose[0]<<endl;
                for(int i = 0; i < WINDOW_SIZE; i++)
                {
                    if(fabs(front_pose.header - Headers[i])<0.00001)
//                    if(front_pose.header == Headers[i])
                    {
//                        for (int k = 0; k < 7; k++)
//                            front_pose.loop_pose[k] = para_Pose[i][k];

                        Eigen::Quaterniond loop_q=front_pose.Q_old;
                        Eigen::Vector3d loop_t=front_pose.P_old;
                        for (int k = 0; k < 3; k++){
                            front_pose.loop_pose[k] =loop_t[k];//把回环优化后的位姿 告诉回环记录的
                        }
                        front_pose.loop_pose[3]=loop_q.x();
                        front_pose.loop_pose[4]=loop_q.y();
                        front_pose.loop_pose[5]=loop_q.z();
                        front_pose.loop_pose[6]=loop_q.w();


                        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
                        problem.AddParameterBlock(front_pose.loop_pose, SIZE_POSE, local_parameterization);

                        int retrive_feature_index = 0;
                        int feature_index = -1;
                        int loop_factor_cnt = 0;
                        //遍历特征
                        for (auto &it_per_id : f_manager.feature)
                        {
                            it_per_id.used_num = it_per_id.feature_per_frame.size();
                            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                                continue;
                            ++feature_index;
                            //获取观测到该特征的起始帧
                            int start = it_per_id.start_frame;
                            //feature has been obeserved in ith frame
                            int end = (int)(start + it_per_id.feature_per_frame.size() - i - 1);
                            if(start <= i && end >=0)
                            {
                                while(front_pose.features_ids[retrive_feature_index] < it_per_id.feature_id)
                                {
                                    retrive_feature_index++;
                                }

                                if(front_pose.features_ids[retrive_feature_index] == it_per_id.feature_id)
                                {
                                    Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements[retrive_feature_index].x, front_pose.measurements[retrive_feature_index].y, 1.0);
                                    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                                    //double ratio = 1.0;
                                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                                    problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);

                                    //printf("loop add factor %d %d %lf %lf %d\n",retrive_feature_index,feature_index,
                                    //                                         pts_j.x(), pts_i.x(),front_pose.features_ids.size());
                                    retrive_feature_index++;
                                    loop_factor_cnt++;
                                    loop_enable = true;
                                }

                            }
                        }
//                        printf("add %d loop factor\n", loop_factor_cnt);
                    }
                }
            }
        }

        if(priorMap_f_manager.feature.size()>0){
//            for (int i = 0; i <= WINDOW_SIZE; i++)
//            {
//                para_Pose_priorMap[i][0] = Ps[i].x();
//                para_Pose_priorMap[i][1] = Ps[i].y();
//                para_Pose_priorMap[i][2] = Ps[i].z();
//                Eigen::Quaterniond q{Rs[i]};
//                para_Pose_priorMap[i][3] = q.x();
//                para_Pose_priorMap[i][4] = q.y();
//                para_Pose_priorMap[i][5] = q.z();
//                para_Pose_priorMap[i][6] = q.w();
//
//            }
            //添加线段视觉残差
//            int priorMap_feature_index = -1;
           
            for (auto &it_per_id : priorMap_f_manager.feature)//遍历滑窗内所有的空间点
            {
                Eigen::Vector3d pts_w = it_per_id.point;

//                ++priorMap_feature_index;
                int imu_j = it_per_id.start_frame - 1;
                for (auto &it_per_frame : it_per_id.feature_per_frame) {
                    imu_j++;
                    Eigen::Vector3d pts_s = it_per_frame.point;

//                    if(std::find(kf_id_inLocal.begin(), kf_id_inLocal.end(), imu_j) == kf_id_inLocal.end()){
//                        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
//                        //para_Pose[i]中存放的是滑动窗口中第i帧的位姿，para_Pose[i]的大小为SIZE_POSE(值为7)
//                        //local_parameterization 参数更新方式需要另外定义 eg:四元数的广义加法
//                        problem.AddParameterBlock(para_Pose_priorMap[imu_j], SIZE_POSE, local_parameterization);
//                        kf_id_inLocal.push_back(imu_j);
//                    }

                    ceres::CostFunction* priorMap_f = ProjectionFactor_inMap::Create( pts_w[0], pts_w[1],pts_w[2],pts_s[0],pts_s[1],pts_s[2]);
                    problem.AddResidualBlock(priorMap_f, NULL, para_Pose[imu_j], para_Ex_Pose[0]);

                }
            }
//            isUpdate_rt_drift=true;
            //        对位姿的约束
//            if(kf_id_inLocal.size()>1){
//                //tmp_retrive_pose_buf.push(front_pose);
//                printf("位姿约束数量为%d\n",kf_id_inLocal.size());
//                sort(kf_id_inLocal.begin(), kf_id_inLocal.end());
//                for(int i = 0; i < kf_id_inLocal.size()-1; i++)
//                {
//                    int kf_id1=kf_id_inLocal[i],kf_id2=kf_id_inLocal[i+1];
//                    Eigen::Vector3d kf1_t=Ps[kf_id1],kf2_t=Ps[kf_id2];
//                    Eigen::Matrix3d kf1_r=Rs[kf_id1],kf2_r=Rs[kf_id2];
//                    Eigen::Vector3d relative_t = kf2_r.transpose() * (kf1_t - kf2_t);
//                    Eigen::Matrix3d relative_q = kf2_r.transpose() * kf1_r;
////                    Eigen::Quaterniond q{relative_q};
//                    Eigen::Vector3d euler_conncected = Utility::R2ypr(relative_q);
//
//                        ceres::CostFunction* cost_function = ProjectionFactor_inMap_relativePose::Create(  relative_t[0], relative_t[1],relative_t[2], euler_conncected[0], euler_conncected[1], euler_conncected[2]);
//                        problem.AddResidualBlock(cost_function, NULL, para_Pose_priorMap[kf_id1], para_Pose_priorMap[kf_id2]);
//
//                }
//            }

        }
        //将先验地图的地图点 加载到f_manager中
//        3d-2d投影约束

//        if(!front_pose.point_clouds_all.empty())
//        {
//            //the retrive pose is in the current window
////            if(front_pose.header >= Headers[0])
////            临时测试一下
//
//            double front_pose_header=front_pose.header;
//            if((front_pose_header - Headers[0])>-0.00001){
//
//                cout<<"front_pose_header="<<front_pose_header<<" , "<<Headers[0]<<endl;
//                Eigen::Vector3d t_cur(front_pose.loop_pose[0],front_pose.loop_pose[1],front_pose.loop_pose[2]);
//                Eigen::Quaterniond q_cur(front_pose.loop_pose[3],front_pose.loop_pose[4],front_pose.loop_pose[5],front_pose.loop_pose[6]);
//                Eigen::Matrix3d r_cur=q_cur.toRotationMatrix();
//                Eigen::Matrix3d r_wc=r_cur*ric;
//                Eigen::Vector3d t_wc=r_cur*tic+t_cur;
//                r_cur=r_wc.transpose();
//                t_cur=-r_cur*t_cur;
//
//                for(int a=0,b=front_pose.point_clouds_all.size();a<b;a++){
////                    double front_pose_header=front_pose.header_all[a];
////
////                    Eigen::Quaterniond loop_q=front_pose.Q_old_all[a];
////                    Eigen::Vector3d loop_t=front_pose.P_old_all[a];
////                    double* loop_pose_flag=new double[7];
////                    for (int k = 0; k < 3; k++){
////                        loop_pose_flag[k]=loop_t[k];//把回环优化后的位姿 告诉回环记录的
////                    }
////                    loop_pose_flag[3]=loop_q.x();
////                    loop_pose_flag[4]=loop_q.y();
////                    loop_pose_flag[5]=loop_q.z();
////                    loop_pose_flag[6]=loop_q.w();
////                    front_pose.loop_pose_all.push_back(loop_pose_flag);
//
//
//
//                        //tmp_retrive_pose_buf.push(front_pose);
//
//                        for(int i = 0; i < WINDOW_SIZE; i++)
//                        {
////                            if(fabs(front_pose.header - Headers[i])<0.00001)
//        //                    if(front_pose.header == Headers[i])//找到匹配帧 在窗口中的位置i
//                                if(fabs(front_pose_header - Headers[i])<0.00001)
//                            {
////                                优化位姿
//                                printf("loop front pose  in window--test\n");
////                                ceres::CostFunction* cost_function = ProjectionFactor_inMap_pose::Create(  front_pose.loop_pose[0], front_pose.loop_pose[1],front_pose.loop_pose[2], front_pose.loop_pose[3], front_pose.loop_pose[4],front_pose.loop_pose[5],front_pose.loop_pose[6]);
////                                problem.AddResidualBlock(cost_function, NULL, para_Pose[i]);
//
//
//
//
////                                for (int k = 0; k < 7; k++){
////                                    front_pose.loop_pose[k] = para_Pose[i][k];//把回环优化后的位姿 告诉回环记录的
////                                }
//
////                                Eigen::Quaterniond loop_q=front_pose.Q_old;
////                                Eigen::Vector3d loop_t=front_pose.P_old;
////
////                                for (int k = 0; k < 3; k++){
////                                    front_pose.loop_pose[k] =loop_t[k];//把回环优化后的位姿 告诉回环记录的
////                                }
////                                front_pose.loop_pose[3]=loop_q.x();
////                                front_pose.loop_pose[4]=loop_q.y();
////                                front_pose.loop_pose[5]=loop_q.z();
////                                front_pose.loop_pose[6]=loop_q.w();
//
//
////                                ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
//////                                problem.AddParameterBlock(front_pose.loop_pose, SIZE_POSE, local_parameterization);//要优化的变量
////
////                                problem.AddParameterBlock(front_pose.loop_pose_all[a], SIZE_POSE, local_parameterization);//要优化的变量
////        //                        ljl
////                                problem.SetParameterBlockConstant(front_pose.loop_pose_all[a]);
//
//                                int retrive_feature_index = 0;
//                                int feature_index = -1;
//                                int loop_factor_cnt = 0;
//
//
//
////                                遍历特征
////                                for (auto &it_per_id : f_manager.feature)
////                                {
////                                    it_per_id.used_num = it_per_id.feature_per_frame.size();
////                                    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
////                                        continue;
////                                    ++feature_index;
////                                    //获取观测到该特征的起始帧
////                                    int start = it_per_id.start_frame;
////                                    //feature has been obeserved in ith frame
////                                    int end = (int)(start + it_per_id.feature_per_frame.size() - i - 1);//能被当前帧看到
////                                    if(start <= i && end >=0)//第i帧能看到这个特征点
////                                    {
//                                cout<<"front_pose.features_ids_all[a][retrive_feature_index]=";
//                                        while(front_pose.features_ids_all[a][retrive_feature_index]!=-1){
////                                            没有跟踪上的点 地图中有
//                                            //没有找到该feature的id，则把特征点放入feature的list容器中
//                                            Eigen::Vector3d pts_i = front_pose.point_clouds_all[a][retrive_feature_index];
//                                            pts_i=r_cur*pts_i+t_cur;
//                                            FeaturePerFrame f_per_fra(pts_i);
//                                            f_manager.feature.push_back(FeaturePerId(front_pose.features_ids_all[a][retrive_feature_index], i)); //give id and start frame
//                                            f_manager.feature.back().feature_per_frame.push_back(f_per_fra);    //give point
//
//                                            cout<<front_pose.features_ids_all[a][retrive_feature_index]<<" , ";
////                                            Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements_all[a][retrive_feature_index].x, front_pose.measurements_all[a][retrive_feature_index].y, 1.0);//回环帧上的点的坐标
////                                            Eigen::Vector3d pts_i = front_pose.point_clouds_all[a][retrive_feature_index];
////
////
////                                            ceres::CostFunction* cost_function = ProjectionFactor_inMap::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), pts_j.z());
////                                            problem.AddResidualBlock(cost_function, NULL, para_Pose[i],para_Ex_Pose[0]);
//
//                                            retrive_feature_index++;
//                                        }
//                                cout<<endl<<"real_feature=";
//                                for (auto &it_per_id : f_manager.feature)
//                                {
//                                    cout<<it_per_id.feature_id<<" , ";
//                                }
//                                cout<<endl;
//                                cout<<"retrive_feature_index="<<retrive_feature_index<<endl;
////                                        while(front_pose.features_ids[retrive_feature_index] < it_per_id.feature_id)
//                                        /**
//                                            while(front_pose.features_ids_all[a][retrive_feature_index] < it_per_id.feature_id)
//                                        {
////                                            没有跟踪上的点 地图中有
////                                            Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements_all[a][retrive_feature_index].x, front_pose.measurements_all[a][retrive_feature_index].y, 1.0);//回环帧上的点的坐标
////                                            Eigen::Vector3d pts_i = front_pose.point_clouds_all[a][retrive_feature_index];
////
////
////                                            ceres::CostFunction* cost_function = ProjectionFactor_inMap::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), pts_j.z());
////                                            problem.AddResidualBlock(cost_function, NULL, para_Pose[i],para_Ex_Pose[0]);
//                                            retrive_feature_index++;
//                                        }
//
////                                        if(front_pose.features_ids[retrive_feature_index] == it_per_id.feature_id)
//                                            if(front_pose.features_ids_all[a][retrive_feature_index] == it_per_id.feature_id)
//                                        {
////                                            定义一个误差 3d点位置相同
//                                            Eigen::Vector3d pts_gt=front_pose.point_clouds_all[a][retrive_feature_index];
//                                            Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
//                                            ceres::CostFunction* cost_function = ProjectionFactor_replace::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_gt.x(), pts_gt.y(), pts_gt.z());
//                                            problem.AddResidualBlock(cost_function, NULL, para_Pose[start],para_Ex_Pose[0],
//                                                                     para_Feature[feature_index]);
//
////                                            Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements[retrive_feature_index].x, front_pose.measurements[retrive_feature_index].y, 1.0);//回环帧上的点的坐标
//
////                                            Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements_all[a][retrive_feature_index].x, front_pose.measurements_all[a][retrive_feature_index].y, 1.0);//回环帧上的点的坐标
////                                            Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
////
////
////
////                                            //double ratio = 1.0;
////
////                                            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);//匹配点和被观测到的第一帧之间的重投影残差
//////                                            problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);
////                                            problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose_all[a], para_Ex_Pose[0], para_Feature[feature_index]);
//
//
//                                            retrive_feature_index++;
//                                            loop_factor_cnt++;//没什么用 只是记录
//                                            loop_enable = true;
//                                            cout<<"利用老帧的点了"<<endl;
//                                        }
//                                         */
////                                    }
////                                }
//                                printf("add %d loop factor\n", loop_factor_cnt);
//
////                                loop_enable = true;
//
////                                遍历特征
////                                for (int c=0,d=front_pose.features_ids_all[a].size();c<d;c++)
////                                {
////                                    if(front_pose.features_ids_all[a][c]==-1){
////    //                                            没有跟踪上的点 地图中有 所以需要添加到滑动窗口 还需要提取是哪个帧观测到它了 并且记录这个点的3d不需要优化
////    //                                            feature_tracker->addPoints();
////    //                                            for (unsigned int i = 0;; i++)
////    //                                            {
////    //                                                bool completed = false;
////    //                                                completed |= feature_tracker->updateID(i);
////    //                                                if (!completed)
////    //                                                    break;
////    //                                            }
////
////                                        FeaturePerFrame f_per_fra(front_pose.point_clouds_all[a][c]);//存3d点
////                                        f_manager.feature.push_back(FeaturePerId(feature_id, frame_count)); //give id and start frame
////                                        f_manager.feature.back().feature_per_frame.push_back(f_per_fra);
////    //                                            设法让feature_tracker跟踪上
////
////                                    }
////                                }
//                            }
//                        }
//
//                    }
//
//                }
//
//        }
//
//
////        对位姿的约束
////        double front_pose_header=front_pose.header;
////        if((front_pose_header - Headers[0])>-0.00001){
////            //tmp_retrive_pose_buf.push(front_pose);
////            printf("loop front pose  in window\n");
////            for(int i = 0; i < WINDOW_SIZE; i++)
////            {
//////                            if(fabs(front_pose.header - Headers[i])<0.00001)
//////                    if(front_pose.header == Headers[i])//找到匹配帧 在窗口中的位置i
////                    if(fabs(front_pose_header - Headers[i])<0.00001)
////                {
////
////                    ceres::CostFunction* cost_function = ProjectionFactor_inMap_pose::Create(  front_pose.loop_pose[0], front_pose.loop_pose[1],front_pose.loop_pose[2], front_pose.loop_pose[3], front_pose.loop_pose[4],front_pose.loop_pose[5],front_pose.loop_pose[6]);
////                    problem.AddResidualBlock(cost_function, NULL, para_Pose[i]);
////                }
////            }
////        }
    }
//
//    else if(LOOP_CLOSURE_SERVER_noLoop)
//    {
//        //loop close factor
//        //front_pose.measurements.clear();
//        if(!retrive_pose_data_server.empty()){
////            cout<<"retrive_pose_data_server 不空"<<endl;
//            if(!(fabs(front_pose.header - retrive_pose_data_server.front().header)<0.00001))
////            if(front_pose.header != retrive_pose_data_server.front().header)
//            {
//                if((front_pose.header - Headers[0])>-0.00001 && front_pose.isRemove!=0){
//                    sendServer_relative=true;
//                    relative_q_sendServer=front_pose.relative_q;
//                    relative_t_sendServer=front_pose.relative_t;
//                    relative_yaw_sendServer=front_pose.relative_yaw;
//                    relative_cur_index_sendServer=front_pose.cur_index;
//                    relative_pitch_sendServer=front_pose.relative_pitch;
//                    relative_roll_sendServer=front_pose.relative_roll;
//                    isRemove_sendServer=front_pose.isRemove;
//                }
//
//                Eigen::Matrix<double, 8, 1> connected_info_test;
//                connected_info_test <<relative_t_sendServer.x(), relative_t_sendServer.y(), relative_t_sendServer.z(),
//                relative_q_sendServer.w(), relative_q_sendServer.x(), relative_q_sendServer.y(), relative_q_sendServer.z(),relative_yaw_sendServer;
////                cout<<"relative_t_sendServer:"<<connected_info_test(0)<<" "<<connected_info_test(1)<<" "<<connected_info_test(2)<<" "<<connected_info_test(7)<<" header"<<front_pose.header<<endl;
//
//
//                front_pose = retrive_pose_data_server.front();  //need lock
//                retrive_pose_data_server.pop();
//
//                if(!((front_pose.header - Headers[0])>-0.00001)){
//                    int num_loop=retrive_pose_data_server.size();
//                    for(int i=0;i<num_loop;i++){
//                        front_pose=retrive_pose_data_server.front();
//                        retrive_pose_data_server.pop();
//                        if((front_pose.header - Headers[0])>-0.00001)
//                            break;
//                    }
//                }
//                front_pose.isRemove=2;
////                front_pose.sendRelativeData_server=true;
//
////                cout<<"front_pose test cur "<<front_pose.cur_index<<" old_index "<<front_pose.old_index<<" "<<front_pose.measurements.size()<<endl;
//                printf("use loop\n");
//            }
//        }
//        if(!front_pose.measurements.empty() && front_pose.isRemove!=0)
//        {
////            cout<<fixed<<setprecision(1)<<"测试 ："<<front_pose.header<<" "<<Headers[0]<<endl;
//            //the retrive pose is in the current window
//            if((front_pose.header - Headers[0])>-0.00001)
////            if(front_pose.header >= Headers[0])
//            {
//                //tmp_retrive_pose_buf.push(front_pose);
//                printf("loop front pose  in window\n");
//                cout<<front_pose.loop_pose[3]<<" , "<<front_pose.loop_pose[0]<<endl;
//                for(int i = 0; i < WINDOW_SIZE; i++)
//                {
//                    if(fabs(front_pose.header - Headers[i])<0.00001)
////                    if(front_pose.header == Headers[i])
//                    {
////                        for (int k = 0; k < 7; k++)
////                            front_pose.loop_pose[k] = para_Pose[i][k];
//
//                        Eigen::Quaterniond loop_q=front_pose.Q_old;
//                        Eigen::Vector3d loop_t=front_pose.P_old;
//                        for (int k = 0; k < 3; k++){
//                            front_pose.loop_pose[k] =loop_t[k];//把回环优化后的位姿 告诉回环记录的
//                        }
//                        front_pose.loop_pose[3]=loop_q.x();
//                        front_pose.loop_pose[4]=loop_q.y();
//                        front_pose.loop_pose[5]=loop_q.z();
//                        front_pose.loop_pose[6]=loop_q.w();
//
//
//                        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
//                        problem.AddParameterBlock(front_pose.loop_pose, SIZE_POSE, local_parameterization);
//
//                        int retrive_feature_index = 0;
//                        int feature_index = -1;
//                        int loop_factor_cnt = 0;
//                        //遍历特征
//                        for (auto &it_per_id : f_manager.feature)
//                        {
//                            it_per_id.used_num = it_per_id.feature_per_frame.size();
//                            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
//                                continue;
//                            ++feature_index;
//                            //获取观测到该特征的起始帧
//                            int start = it_per_id.start_frame;
//                            //feature has been obeserved in ith frame
//                            int end = (int)(start + it_per_id.feature_per_frame.size() - i - 1);
//                            if(start <= i && end >=0)
//                            {
//                                while(front_pose.features_ids[retrive_feature_index] < it_per_id.feature_id)
//                                {
//                                    retrive_feature_index++;
//                                }
//
//                                if(front_pose.features_ids[retrive_feature_index] == it_per_id.feature_id)
//                                {
//                                    Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements[retrive_feature_index].x, front_pose.measurements[retrive_feature_index].y, 1.0);
//                                    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
//                                    //double ratio = 1.0;
//                                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
//                                    problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);
//
//                                    //printf("loop add factor %d %d %lf %lf %d\n",retrive_feature_index,feature_index,
//                                    //                                         pts_j.x(), pts_i.x(),front_pose.features_ids.size());
//                                    retrive_feature_index++;
//                                    loop_factor_cnt++;
//                                    loop_enable = true;
//                                }
//
//                            }
//                        }
////                        printf("add %d loop factor\n", loop_factor_cnt);
//                    }
//                }
//            }
//        }
//    }
    
//    if(LOOP_CLOSURE_SERVER)
//    {
////        queue<std::vector<cv::Point2f>> v_measurements_cur_coarse_all;
////        queue<std::vector<Eigen::Vector3d>> v_point_3d_old_all;
////        queue<std::vector<int>> v_feature_id_cur_all;
////        queue<double> v_cur_header_all;
////        queue<double*> v_rt_double_all;
//        q_old_3d_mutex.lock();
//        while(!retrive_pose_data_localMapping.empty()){
//            cout<<"不应该进行"<<endl;
//            double cur_header=retrive_pose_data_localMapping.front().header_cur;
//            if((cur_header - Headers[0])>-0.00001){
////                v_measurements_cur_coarse_all=q_measurements_cur_norm_all;
////                v_point_3d_old_all=q_point_3d_old_all;
////                v_feature_id_cur_all=q_feature_id_cur_all;
////                v_cur_header_all=q_header_cur;
////                v_rt_double_all=loop_pose_forSlideWindow;
//
//                double* loop_pose_ljl=retrive_pose_data_localMapping.front().loop_pose_forSlideWindow;
//                for(int i=0;i<7;i++){
//                    loop_pose_forSlideWindow_update[i]=loop_pose_ljl[i];
//                }
//                break;
//            }else{
////                q_header_cur.pop();
////                q_measurements_cur_norm_all.pop();
////                q_point_3d_old_all.pop();
////                q_feature_id_cur_all.pop();
////                delete[] loop_pose_forSlideWindow.front();
////                loop_pose_forSlideWindow.pop();
//
//                retrive_pose_data_localMapping.pop();
//            }
//
//        }
//
//        q_old_3d_mutex.unlock();
//
//    //        3d-2d投影约束
//        if(!retrive_pose_data_localMapping.empty())
//        {
//            int queue_size=retrive_pose_data_localMapping.size();
//            for(int a=0,b=queue_size;a<1;a++){//这里暂时只放最老的那一个约束在里面
//                RetriveData_localMapping rl_single=retrive_pose_data_localMapping.front();
//                double v_cur_header=rl_single.header_cur;
//                std::vector<cv::Point2f> v_measurements_cur_coarse=rl_single.measurements_cur_norm;
//                std::vector<Eigen::Vector3d> v_point_3d_old=rl_single.point_3d_old;
//                std::vector<int> v_feature_id_cur=rl_single.features_ids_cur;
////                v_cur_header_all.pop();
////                v_measurements_cur_coarse_all.pop();
////                v_point_3d_old_all.pop();
////                v_feature_id_cur_all.pop();
//                for(int i = 0; i < WINDOW_SIZE; i++)
//                {
//                    //找到匹配帧 在窗口中的位置i
//                    if(fabs(v_cur_header - Headers[i])<0.00001)
//                    {
//
//                        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
//                        problem.AddParameterBlock(loop_pose_forSlideWindow_update, SIZE_POSE, local_parameterization);//要优化的变量
//
//                        for(int d=0,e=v_point_3d_old.size();d<e;d++){
//                            Eigen::Vector3d pts_j = Eigen::Vector3d((double)(v_measurements_cur_coarse[d].x),(double)(v_measurements_cur_coarse[d].y) , 1.0);//回环帧上的点的坐标
//                            Eigen::Vector3d pts_i = v_point_3d_old[d];
//                            ceres::CostFunction* cost_function = ProjectionFactor_inMap::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), pts_j.z());
//                            problem.AddResidualBlock(cost_function, NULL, loop_pose_forSlideWindow_update,para_Ex_Pose[0]);
//                        }
//
//
////                                            定义一个误差 3d点位置相同 适用于跟踪上的点
////                        Eigen::Vector3d pts_gt=front_pose.point_clouds_all[a][retrive_feature_index];
////                        Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
////                        ceres::CostFunction* cost_function = ProjectionFactor_replace::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_gt.x(), pts_gt.y(), pts_gt.z());
////                        problem.AddResidualBlock(cost_function, NULL, para_Pose[start],para_Ex_Pose[0],para_Feature[feature_index]);
//
//
////                        Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements[retrive_feature_index].x, front_pose.measurements[retrive_feature_index].y, 1.0);//回环帧上的点的坐标
////
////                        Eigen::Vector3d pts_j = Eigen::Vector3d(front_pose.measurements_all[a][retrive_feature_index].x, front_pose.measurements_all[a][retrive_feature_index].y, 1.0);//回环帧上的点的坐标
////                        Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
////
////                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);//匹配点和被观测到的第一帧之间的重投影残差
//////                                            problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);
////                        problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose_all[a], para_Ex_Pose[0], para_Feature[feature_index]);
//                        cout<<"约束加入成功"<<endl;
//                        loop_enable=true;
//                        break;
//
//                }
//
//            }
//
//            }
//        }
//    }
    
    //5.创建优化求解器
    ceres::Solver::Options options;
    //6.设定优化器的solver_type类型
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 1;
    //7.设定优化使用的算法。这里使用置信域类优化算法中的dogleg方法
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = false;
//    options.logging_type=ceres::SILENT;
    //8.设置迭代求解的次数
    options.max_num_iterations = 10;//10
    //options.use_nonmonotonic_steps = true;
    if(buf_num<2)
        options.max_solver_time_in_seconds = SOLVER_TIME;
    else if(buf_num<4)
        options.max_solver_time_in_seconds = SOLVER_TIME * 2.0 / 3.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME / 2.0;
    
    ceres::Solver::Summary summary;//优化信息
    //TE(prepare_solver);
   
//    TS(ceres);
   

    
    //9.开始求解
    ceres::Solve(options, &problem, &summary);
//    cout<<"solve end:"<<summary.termination_type<<endl;
    final_cost = summary.final_cost;
//    cout << summary.FullReport() << endl;
//    TE(ceres);
    
//    if(options.IsValid(&)){
//        printf("ceres solve is valid\n");
//    }else{
//        printf("ceres solve is not valid\n");
//    }
    
   
/**
    
    if(LOOP_CLOSURE)
    {
        
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
            if(fabs(front_pose.header - Headers[i])<0.00001)
//            if(front_pose.header == Headers[i])
            {
                front_pose.loop_pose[0]=front_pose.loop_pose_all[0][0];
                front_pose.loop_pose[1]=front_pose.loop_pose_all[0][1];
                front_pose.loop_pose[2]=front_pose.loop_pose_all[0][2];
                front_pose.loop_pose[3]=front_pose.loop_pose_all[0][3];
                front_pose.loop_pose[4]=front_pose.loop_pose_all[0][4];
                front_pose.loop_pose[5]=front_pose.loop_pose_all[0][5];
                front_pose.loop_pose[6]=front_pose.loop_pose_all[0][6];
                
                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4], front_pose.loop_pose[5]).normalized().toRotationMatrix();
                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
                

                
//                当前帧3d
//                Eigen::Matrix3d Rs_i = Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
//                Eigen::Vector3d Ps_i = Eigen::Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
////当前帧
//                front_pose.relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                front_pose.relative_q = Rs_loop.transpose() * Rs_i;
//                front_pose.relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
//
//              //ljl
//                front_pose.relative_pitch=Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
//                front_pose.relative_roll=Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());


//                老帧3d
                Eigen::Matrix3d Rs_i = front_pose.Q_old.toRotationMatrix();
                Eigen::Vector3d Ps_i = front_pose.P_old;//老帧
//                老帧3d
                front_pose.relative_t = Rs_i.transpose() * (-Ps_i +Ps_loop);
                front_pose.relative_q = Rs_i.transpose() * Rs_loop;
                front_pose.relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).x() - Utility::R2ypr(Rs_i).x());

              //ljl
                front_pose.relative_pitch=Utility::normalizeAngle(Utility::R2ypr(Rs_loop).y() - Utility::R2ypr(Rs_i).y());
                front_pose.relative_roll=Utility::normalizeAngle(Utility::R2ypr(Rs_loop).z() - Utility::R2ypr(Rs_i).z());

                
            }
        }
    }
    
    else if(LOOP_CLOSURE_SERVER)
    {
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
            if(fabs(front_pose.header - Headers[i])<0.00001)
//            if(front_pose.header == Headers[i])
            {
//                Eigen::Matrix3d Rs_i = Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
//                Eigen::Vector3d Ps_i = Eigen::Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
//
//
//
//                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
//                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
//
//                front_pose.relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                front_pose.relative_q = Rs_loop.transpose() * Rs_i;
//                front_pose.relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                
                cout<<"LOOP_CLOSURE_SERVER relative_yaw "<<endl;


                //ljl
//                front_pose.relative_pitch=Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
//                front_pose.relative_roll=Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                
                
              

//                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4], front_pose.loop_pose[5]).normalized().toRotationMatrix();
//                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
//
////                老帧3d
//                Eigen::Matrix3d Rs_i = front_pose.Q_old.toRotationMatrix();
//                Eigen::Vector3d Ps_i = front_pose.P_old;//老帧
////                老帧3d
//                front_pose.relative_t = Rs_i.transpose() * (-Ps_i +Ps_loop);
//                front_pose.relative_q = Rs_i.transpose() * Rs_loop;
//                front_pose.relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).x() - Utility::R2ypr(Rs_i).x());
//
//              //ljl
//                front_pose.relative_pitch=Utility::normalizeAngle(Utility::R2ypr(Rs_loop).y() - Utility::R2ypr(Rs_i).y());
//                front_pose.relative_roll=Utility::normalizeAngle(Utility::R2ypr(Rs_loop).z() - Utility::R2ypr(Rs_i).z());
                
                
//                                cout<<"relative_t:"<<front_pose.relative_t.norm()<<endl;
                //                cout<<"relative_q:"<<front_pose.relative_q<<endl;
//                                cout<<"relative_yaw:"<<front_pose.relative_yaw<<endl;
            }
        }
    }
    
    else if(LOOP_CLOSURE_SERVER_noLoop)
    {
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
            if(fabs(front_pose.header - Headers[i])<0.00001)
//            if(front_pose.header == Headers[i])
            {
//                Eigen::Matrix3d Rs_i = Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
//                Eigen::Vector3d Ps_i = Eigen::Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
//
//
//
//                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
//                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
//
//                front_pose.relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                front_pose.relative_q = Rs_loop.transpose() * Rs_i;
//                front_pose.relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                
                cout<<"LOOP_CLOSURE_SERVER_noLoop relative_yaw "<<endl;


                //ljl
//                front_pose.relative_pitch=Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
//                front_pose.relative_roll=Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                
                
              

//                Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4], front_pose.loop_pose[5]).normalized().toRotationMatrix();
//                Eigen::Vector3d Ps_loop = Eigen::Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
//
////                老帧3d
//                Eigen::Matrix3d Rs_i = front_pose.Q_old.toRotationMatrix();
//                Eigen::Vector3d Ps_i = front_pose.P_old;//老帧
////                老帧3d
//                front_pose.relative_t = Rs_i.transpose() * (-Ps_i +Ps_loop);
//                front_pose.relative_q = Rs_i.transpose() * Rs_loop;
//                front_pose.relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).x() - Utility::R2ypr(Rs_i).x());
//
//              //ljl
//                front_pose.relative_pitch=Utility::normalizeAngle(Utility::R2ypr(Rs_loop).y() - Utility::R2ypr(Rs_i).y());
//                front_pose.relative_roll=Utility::normalizeAngle(Utility::R2ypr(Rs_loop).z() - Utility::R2ypr(Rs_i).z());
                
                
//                                cout<<"relative_t:"<<front_pose.relative_t.norm()<<endl;
                //                cout<<"relative_q:"<<front_pose.relative_q<<endl;
//                                cout<<"relative_yaw:"<<front_pose.relative_yaw<<endl;
            }
        }
    }
 */
    //求解完成后，将数组转化为向量
    
    new2old_2();
    
    vector<ceres::ResidualBlockId> residual_set;
    problem.GetResidualBlocks(&residual_set);
    for (auto it : residual_set)
        problem.RemoveResidualBlock(it);
    
    //for marginalization back//判断边缘化标志
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        old2new();
        //! 先验误差会一直保存，而不是只使用一次
        //! 如果上一次边缘化的信息存在
        //! 要边缘化的参数块是 para_Pose[0] para_SpeedBias[0] 以及 para_Feature[feature_index](滑窗内的第feature_index个点的逆深度)
        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {//查询last_marginalization_parameter_blocks中是首帧状态量的序号
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor构造边缘化的的Factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);//添加上一次边缘化的参数块
            
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
//        把这次要marg的IMU信息加进来
        {
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
//        把这次要marg的视觉信息加进来
        {//添加视觉的先验，只添加起始帧是旧帧且观测次数大于2的Features
            int feature_index = -1;
            //遍历特征
            for (auto &it_per_id : f_manager.feature)//遍历滑窗内所有的Features
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();//该特征点被观测到的次数
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;//Feature的观测次数不小于2次，且起始帧不属于最后两帧
                
                ++feature_index;
                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)//只选择被边缘化的帧的Features
                    continue;
                //当前特征所在的第一个观测帧中观测的点坐标
                Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                //遍历当前特征的观测帧
                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    
                    Eigen::Vector3d pts_j = it_per_frame.point;
                    if (imu_i == imu_j)//不需要起始观测帧
                        continue;
                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                   vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                   vector<int>{0, 3});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                    
                }
            }
        }
        
//        TS(per_marginalization);
        marginalization_info->preMarginalize(); //??
//        TE(per_marginalization);
//        TS(marginalization);
        marginalization_info->marginalize();   //??
//        TE(marginalization);
        
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    //marginalize front
    else
    {
        if (last_marginalization_info&&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            old2new();
//            当次新帧不是关键帧时，直接剪切掉次新帧和它的视觉观测边（该帧和路标点之间的关联），而不对次新帧进行marginalize处理
//            但是要保留次新帧的IMU数据，从而保证IMU预积分的连续性，这样才能积分计算出下一帧的测量值
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    assert(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);
                
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
            //计算每次IMU和视觉观测(cost_function)对应的参数块(parameter_blocks),雅可比矩阵(jacobians),残差值(residuals)
            marginalization_info->preMarginalize();
            //多线程计算整个先验项的参数块,雅可比矩阵和残差值
            marginalization_info->marginalize();
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);//last_marginalization_parameter_blocks的赋值
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
}







/**
bool VINS::solveInitial()
{
    printf("solve initial------------------------------------------\n");
    printf("PS %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    printf("PS %lf %lf %lf\n", Ps[10].x(),Ps[10].y(), Ps[10].z());
    //check imu observibility

     
    // global sfm
    Eigen::Quaterniond *Q = new Eigen::Quaterniond[frame_count + 1];//旋转量，四元数数组
    Eigen::Vector3d *T = new Eigen::Vector3d[frame_count + 1];//平移量数组
    map<int, Eigen::Vector3d> sfm_tracked_points;//用于存储sfm重建出来的特征点的坐标
    vector<SFMFeature> sfm_f;
//    将f_manager.feature中的feature存储到sfm_f中
    {
        for (auto &it_per_id : f_manager.feature)
        {
            int imu_j = it_per_id.start_frame - 1;
            
            SFMFeature tmp_feature;
            tmp_feature.state = false;
            tmp_feature.id = it_per_id.feature_id;
            //遍历每一个能观察到该feature的frame
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                Eigen::Vector3d pts_j = it_per_frame.point;
                //每个特征点能够被哪些帧观测到以及特征点在这些帧中的坐标
                tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
            }
            sfm_f.push_back(tmp_feature);//将特征信息保存在sdm_f中
        }
        
//        位姿求解
        Eigen::Matrix3d relative_R;
        Eigen::Vector3d relative_T;
        int l;
//        这里就已经得到图像的特征点2d坐标的提取，相机第l帧和最后一帧之间的旋转和平移（注意暂时还没有得到特征的3d点坐标）
//        就可以构建全局的SFM类GlobalSFM sfm，得到的Rt为当前帧到第l帧的坐标系变换Rt，存储在relative_R和relative_T当中
        //通过对极约束中的F矩阵恢复R、t
        if (!relativePose(0, relative_R, relative_T, l))//求得位姿
        {
            printf("init solve 5pts between first frame and last frame failed\n");
            return false;
        }
        //update init progress
        initProgress = 30;
        
        GlobalSFM sfm;
//        以第l帧作为参考帧，三角化特征点，对滑窗每一帧求解sfm问题
        if(!sfm.construct(frame_count + 1, Q, T, l,
                          relative_R, relative_T,
                          sfm_f, sfm_tracked_points))
        {
            printf("global SFM failed!");
            init_status = FAIL_SFM;
            marginalization_flag = MARGIN_OLD;
            fail_times++;
            return false;
        }
        //update init progress
        initProgress = 50;
    }
//    对于所有的图像帧，包括不在滑动窗口中的，提供初始的RT估计，然后solvePnP进行优化
    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Eigen::Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    //遍历所有的图像帧
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        
        
        if(fabs((frame_it->first) - Headers[i])<0.00001)
//        if((frame_it->first) == Headers[i])
        {
//            cout << "key frame " << i << endl;
            frame_it->second.is_key_frame = true;
            //各帧相对于参考帧的旋转和相机-IMU之间的旋转做乘积，就将坐标系转换到了IMU坐标系下
            frame_it->second.R = Q[i].toRotationMatrix() * ric.transpose();
            //各帧相对于参考帧的平移
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if(fabs(frame_it->first-Headers[i])>0.00001 && (frame_it->first) > (Headers[i]-0.00001))
//        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        //注意这里的 Q和 T是图像帧的位姿，而不是求解PNP时所用的坐标系变换矩阵，两者具有对称关系
        Eigen::Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Eigen::Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        //使用罗德里格斯公式将旋转矩阵转换成旋转向量
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);
        
        frame_it->second.is_key_frame = false;
        //获取 pnp需要用到的存储每个特征点三维点和图像坐标的 vector
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            //cout << "feature id " << feature_id;
            //cout << " pts image_frame " << (i_p.second.head<2>() * 460 ).transpose() << endl;
            it = sfm_tracked_points.find(feature_id);
            if(it != sfm_tracked_points.end())
            {
                Eigen::Vector3d world_pts = it->second;
                cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                pts_3_vector.push_back(pts_3);
                
                Eigen::Vector2d img_pts = id_pts.second.head<2>();
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);
            }
        }
//        构造相机内参，因为已经是归一化后坐标了，这里的内参就是单位矩阵了
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1);
        //保证特征点数大于5，这样就可以用五点法求解
        if(pts_3_vector.size() < 6 )
        {
            printf("init Not enough points for solve pnp !\n");
            return false;
        }
        
        
        
//        世界坐标系中的特征点的坐标，图像中的像素点的坐标，相机内参，去畸变参数，要求的旋转量，要求的平移量
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))//该API输入是世界坐标系的点，求解出的是世界坐标系到相机坐标系的变换，所以一般需要将结果转置
        {
            printf("init solve pnp fail!\n");
            init_status = FAIL_PNP;
            fail_times++;
            return false;
        }
        //旋转向量到旋转矩阵的转换
        cv::Rodrigues(rvec, r);
        //cout << "r " << endl << r << endl;
        Eigen::MatrixXd R_pnp,tmp_R_pnp;
        //将矩阵从cv中的cv::Mat类型转换为eigen中的Eigen::MatrixXd
        cv::cv2eigen(r, tmp_R_pnp);
        //cout << "R_pnp " << endl << R_pnp << endl;
//        将坐标变换矩阵转变成图像帧位姿，并转换为IMU坐标系的位姿
        R_pnp = tmp_R_pnp.transpose();
        Eigen::MatrixXd T_pnp;
        //将位移矩阵转换为Eigen::MatrixXd形式
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        //将求得的旋转和平移存储起来，这里RIC[0].transpose()是相机到IMU的旋转，这样就把相机坐标系中求出的位姿旋转到IMU坐标系中去了
        frame_it->second.R = R_pnp * ric.transpose();//求出位姿
        frame_it->second.T = T_pnp;
    }
    delete[] Q;
    delete[] T;
    
    //update init progress
    initProgress = 75;
    
    printf("init PS after pnp %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    //ljl 在视觉惯性联合初始化之前，加一个仅imu的参数粗略估计
    
    
    
//    真正的视觉惯性联合初始化 视觉惯性对齐求解
    //ljl 这里希望把尺度的约束（为正）加进去，重力加速度不再是包括偏置的（偏置也不断优化）
    //先看一下 orb的预积分，陀螺仪的偏置和加速度计的偏置在联合优化里会优化，那么预积分会改变值吗?
    if (visualInitialAlign())
    {
        //update init progress
        initProgress = 85;
        return true;
    }
    else
    {
        init_status = FAIL_ALIGN;
        fail_times++;
        return false;
    }
    
}

*/
bool VINS::solveInitial()
{
    printf("solve initial------------------------------------------\n");
    printf("PS %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    printf("PS %lf %lf %lf\n", Ps[10].x(),Ps[10].y(), Ps[10].z());
    //check imu observibility原版注释 ljl添加
    
   
     
    // global sfm
    Eigen::Quaterniond *Q = new Eigen::Quaterniond[frame_count + 1];//旋转量，四元数数组
    Eigen::Vector3d *T = new Eigen::Vector3d[frame_count + 1];//平移量数组
    map<int, Eigen::Vector3d> sfm_tracked_points;//用于存储sfm重建出来的特征点的坐标
    vector<SFMFeature> sfm_f;
//    将f_manager.feature中的feature存储到sfm_f中
    {
        for (auto &it_per_id : f_manager.feature)
        {
            int imu_j = it_per_id.start_frame - 1;
            
            SFMFeature tmp_feature;
            tmp_feature.state = false;
            tmp_feature.id = it_per_id.feature_id;
            //遍历每一个能观察到该feature的frame
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                Eigen::Vector3d pts_j = it_per_frame.point;
                //每个特征点能够被哪些帧观测到以及特征点在这些帧中的坐标
                tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
            }
            sfm_f.push_back(tmp_feature);//将特征信息保存在sdm_f中
        }
        
//        位姿求解
        Eigen::Matrix3d relative_R;
        Eigen::Vector3d relative_T;
        int l;
//        这里就已经得到图像的特征点2d坐标的提取，相机第l帧和最后一帧之间的旋转和平移（注意暂时还没有得到特征的3d点坐标）
//        就可以构建全局的SFM类GlobalSFM sfm，得到的Rt为当前帧到第l帧的坐标系变换Rt，存储在relative_R和relative_T当中
        //通过对极约束中的F矩阵恢复R、t
        if (!relativePose(0, relative_R, relative_T, l))//求得位姿
        {
            printf("init solve 5pts between first frame and last frame failed\n");
            return false;
        }
        //update init progress
        initProgress = 30;
        
        GlobalSFM sfm;
//        以第l帧作为参考帧，三角化特征点，对滑窗每一帧求解sfm问题
        if(!sfm.construct(frame_count + 1, Q, T, l,
                          relative_R, relative_T,
                          sfm_f, sfm_tracked_points))
        {
            printf("global SFM failed!");
            init_status = FAIL_SFM;
            marginalization_flag = MARGIN_OLD;
            fail_times++;
            return false;
        }
        //update init progress
        initProgress = 50;
    }
//    对于所有的图像帧，包括不在滑动窗口中的，提供初始的RT估计，然后solvePnP进行优化
    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Eigen::Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    //遍历所有的图像帧
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        
        
        if(fabs((frame_it->first) - Headers[i])<0.00001)
//        if((frame_it->first) == Headers[i])
        {
//            cout << "key frame " << i << endl;
            frame_it->second.is_key_frame = true;
            //各帧相对于参考帧的旋转和相机-IMU之间的旋转做乘积，就将坐标系转换到了IMU坐标系下
            frame_it->second.R = Q[i].toRotationMatrix() * ric.transpose();
            //各帧相对于参考帧的平移
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if(fabs(frame_it->first-Headers[i])>0.00001 && (frame_it->first) > (Headers[i]-0.00001))
//        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        //注意这里的 Q和 T是图像帧的位姿，而不是求解PNP时所用的坐标系变换矩阵，两者具有对称关系
        Eigen::Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Eigen::Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        //使用罗德里格斯公式将旋转矩阵转换成旋转向量
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);
        
        frame_it->second.is_key_frame = false;
        //获取 pnp需要用到的存储每个特征点三维点和图像坐标的 vector
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            //cout << "feature id " << feature_id;
            //cout << " pts image_frame " << (i_p.second.head<2>() * 460 ).transpose() << endl;
            it = sfm_tracked_points.find(feature_id);
            if(it != sfm_tracked_points.end())
            {
                Eigen::Vector3d world_pts = it->second;
                cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                pts_3_vector.push_back(pts_3);
                
                Eigen::Vector2d img_pts = id_pts.second.head<2>();
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);
            }
        }
//        构造相机内参，因为已经是归一化后坐标了，这里的内参就是单位矩阵了
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1);
        //保证特征点数大于5，这样就可以用五点法求解
        if(pts_3_vector.size() < 6 )
        {
            printf("init Not enough points for solve pnp !\n");
            return false;
        }
        
        
        
//        世界坐标系中的特征点的坐标，图像中的像素点的坐标，相机内参，去畸变参数，要求的旋转量，要求的平移量
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))//该API输入是世界坐标系的点，求解出的是世界坐标系到相机坐标系的变换，所以一般需要将结果转置
        {
            printf("init solve pnp fail!\n");
            init_status = FAIL_PNP;
            fail_times++;
            return false;
        }
        //旋转向量到旋转矩阵的转换
        cv::Rodrigues(rvec, r);
        //cout << "r " << endl << r << endl;
        Eigen::MatrixXd R_pnp,tmp_R_pnp;
        //将矩阵从cv中的cv::Mat类型转换为eigen中的Eigen::MatrixXd
        cv::cv2eigen(r, tmp_R_pnp);
        //cout << "R_pnp " << endl << R_pnp << endl;
//        将坐标变换矩阵转变成图像帧位姿，并转换为IMU坐标系的位姿
        R_pnp = tmp_R_pnp.transpose();
        Eigen::MatrixXd T_pnp;
        //将位移矩阵转换为Eigen::MatrixXd形式
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        //将求得的旋转和平移存储起来，这里RIC[0].transpose()是相机到IMU的旋转，这样就把相机坐标系中求出的位姿旋转到IMU坐标系中去了
        frame_it->second.R = R_pnp * ric.transpose();//求出位姿
        frame_it->second.T = T_pnp;
    }
    delete[] Q;
    delete[] T;
    
    //update init progress
    initProgress = 75;
    
    printf("init PS after pnp %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    //ljl 在视觉惯性联合初始化之前，加一个仅imu的参数粗略估计
    
    
    
//    真正的视觉惯性联合初始化 视觉惯性对齐求解
    //ljl 这里希望把尺度的约束（为正）加进去，重力加速度不再是包括偏置的（偏置也不断优化）
    //先看一下 orb的预积分，陀螺仪的偏置和加速度计的偏置在联合优化里会优化，那么预积分会改变值吗?
    //这里直接换成优化的方式求解
    if (visualInitialAlign())
    {
        //update init progress
        initProgress = 85;
        return true;
    }
    else
    {
        init_status = FAIL_ALIGN;
        fail_times++;
        return false;
    }
    
}






/**
//视觉惯导对齐 备份
bool VINS::visualInitialAlign()
{
    TS(solve_g);
    Eigen::VectorXd x;
    //solve scale 视觉惯性联合初始化，计算陀螺仪的偏置，尺度，重力加速度和速度
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        printf("solve g failed!");
        printf("init PS alignment failed %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
        return false;
    }
    TE(solve_g);
    printf("init PS algnment succ %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    // change state获取滑动窗口内所有图像帧相对于第l帧的位姿信息，并设置为关键帧
    for (int i = 0; i <= frame_count; i++)
    {
        Eigen::Matrix3d Ri = all_image_frame[Headers[i]].R;
        Eigen::Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }
    //3.获取特征点深度
    Eigen::VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);
    
    //triangulat on cam pose , no tic
    Eigen::Vector3d TIC_TMP;
    TIC_TMP.setZero();
    //三角化计算地图点的深度，Ps中存放的是各个帧相对于参考帧之间的平移，ric为相机-IMU之间的旋转
    f_manager.triangulate(Ps, TIC_TMP, ric, true);
    
    double s = (x.tail<1>())(0);
    //4.这里陀螺仪的偏差Bgs改变了，需遍历滑动窗口中的帧，重新预积分
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Eigen::Vector3d::Zero(), Bgs[i]);
    }
//    printf("s=%lf ,tic=%lf %lf %lf\n",s,tic[0],tic[1],tic[2]);
    //5.计算各帧相对于b0的位姿信息，前边计算的都是相对于第l帧的位姿
    for (int i = frame_count; i >= 0; i--){
//        printf("before ps= %lf , %lf, %lf \n",Ps[i][0],Ps[i][1],Ps[i][2]);
        Ps[i] = s * Ps[i] - Rs[i] * tic - (s * Ps[0] - Rs[0] * tic);
//        printf("after ps= %lf , %lf, %lf \n",Ps[i][0],Ps[i][1],Ps[i][2]);
    }
    
//    printf("PS after scale %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            //存储速度
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    printf("init finish--------------------\n");
    //更新每个地图点被观测到的帧数(used_num)和预测的深度(estimated_depth)
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }
//    refine之后就获得了C_0坐标系下的重力g^{c_0}，此时通过将g^{c_0}旋转至z轴方向，
//    这样就可以计算相机系到世界坐标系的旋转矩阵q_{c_0}^w，这里求得的是rot_diff,这样就可以将所有变量调整至世界系中。
    Eigen::Matrix3d R0 = Utility::g2R(g);
    double yaw0 = Utility::R2ypr(R0).x();
    Eigen::Matrix3d yaw_refine = Utility::ypr2R(Eigen::Vector3d{-yaw0,0,0});
    R0 = yaw_refine * R0;
    g = R0 * g;
    //Eigen::Matrix3d rot_diff = R0 * Rs[0].transpose();
    Eigen::Matrix3d rot_diff = R0;
    //所有变量从参考坐标系c_0旋转到世界坐标系w
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
        init_poses.push_back(Ps[i]);
    }
    
    return true;
}
*/

bool VINS::visualInitialAlign()
{
    TS(solve_g);
    Eigen::VectorXd x;
    //solve scale 视觉惯性联合初始化，计算陀螺仪的偏置，尺度，重力加速度和速度
//    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    bool result =VisualOptiIMU(all_image_frame, Bgs, g, x);
    
    
    
    if(!result)
    {
        printf("solve g failed!");
        printf("init PS alignment failed %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
        return false;
    }
    TE(solve_g);
    printf("init PS algnment succ %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    // change state获取滑动窗口内所有图像帧相对于第l帧的位姿信息，并设置为关键帧
    for (int i = 0; i <= frame_count; i++)
    {
        Eigen::Matrix3d Ri = all_image_frame[Headers[i]].R;
        Eigen::Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }
    //3.获取特征点深度
    Eigen::VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);
    
    //triangulat on cam pose , no tic
    Eigen::Vector3d TIC_TMP;
    TIC_TMP.setZero();
    //三角化计算地图点的深度，Ps中存放的是各个帧相对于参考帧之间的平移，ric为相机-IMU之间的旋转
    f_manager.triangulate(Ps, TIC_TMP, ric, true);
    
    double s = (x.tail<1>())(0);
    //4.这里陀螺仪的偏差Bgs改变了，需遍历滑动窗口中的帧，重新预积分
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Eigen::Vector3d::Zero(), Bgs[i]);
    }
//    printf("s=%lf ,tic=%lf %lf %lf\n",s,tic[0],tic[1],tic[2]);
    //5.计算各帧相对于b0的位姿信息，前边计算的都是相对于第l帧的位姿
    for (int i = frame_count; i >= 0; i--){
//        printf("before ps= %lf , %lf, %lf \n",Ps[i][0],Ps[i][1],Ps[i][2]);
        Ps[i] = s * Ps[i] - Rs[i] * tic - (s * Ps[0] - Rs[0] * tic);
//        printf("after ps= %lf , %lf, %lf \n",Ps[i][0],Ps[i][1],Ps[i][2]);
    }
    
//    printf("PS after scale %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            //存储速度
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    printf("init finish--------------------\n");
    //更新每个地图点被观测到的帧数(used_num)和预测的深度(estimated_depth)
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }
//    refine之后就获得了C_0坐标系下的重力g^{c_0}，此时通过将g^{c_0}旋转至z轴方向，
//    这样就可以计算相机系到世界坐标系的旋转矩阵q_{c_0}^w，这里求得的是rot_diff,这样就可以将所有变量调整至世界系中。
    Eigen::Matrix3d R0 = Utility::g2R(g);
    double yaw0 = Utility::R2ypr(R0).x();
    Eigen::Matrix3d yaw_refine = Utility::ypr2R(Eigen::Vector3d{-yaw0,0,0});
    R0 = yaw_refine * R0;
    g = R0 * g;
    //Eigen::Matrix3d rot_diff = R0 * Rs[0].transpose();
    Eigen::Matrix3d rot_diff = R0;
    //所有变量从参考坐标系c_0旋转到世界坐标系w
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
        init_poses.push_back(Ps[i]);
    }
    
    return true;
}

//只有初始化时用
bool VINS::relativePose(int camera_id, Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l)
{
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
        //通过f_manager获取（滑动窗口中）第i帧和最后一帧的特征匹配corres
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Eigen::Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Eigen::Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
                
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            parallax_num_view = average_parallax * 520;
//            最新的keyFrame和sliding window中某个keyFrame之间有足够feature匹配和足够大的视差
            if(average_parallax * 520 < 30)
            {
                init_status = FAIL_PARALLAX;
                fail_times++;
                return false;
            }
//            这两帧之间通过五点法恢复出R，t并且三角化出3D的特征点feature point
//            这种relativePose得到的位姿是第I帧的，第I帧的筛选是从第一帧开始到滑动窗口所有帧中一开始满足平均视差足够大的帧，这里的第I帧会作为参考帧到下面的全局SFM使用
            if(m_estimator.solveRelativeRT(corres, relative_R, relative_T))//算位姿
            {
                l = i;
                printf("average_parallax %f choose l %d and newest frame to triangulate the whole structure\n", average_parallax * 520, l);
                return true;
            }
            else
            {
                init_status = FAIL_RELATIVE;
                fail_times++;
                return false;
            }
        }
    }
    return false;
}
/*
 marginalize the state from the sliding window and change feature start frame
 */
//上一次非线性优化结束，最后的H矩阵就是本轮非线性优化的先验矩阵的前身
void VINS::slideWindow()
{
    //marginalize old keyframe 删除的是滑窗第一帧
    if (marginalization_flag == MARGIN_OLD)
    {
        //ljl
        {
            map<double, ImageFrame>::iterator frame_it;
            Eigen::Vector3d sum_g;
            int num=0;
            for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
            {
                double dt = frame_it->second.pre_integration->sum_dt;
                if(dt>0.000000){
                    Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
                    sum_g += tmp_g;
                    num++;
                }
            }
        
        Eigen::Vector3d aver_g;
        aver_g = sum_g * 1.0 / num;
        cout << "aver_g " << aver_g.transpose() << endl;
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
        double dt = frame_it->second.pre_integration->sum_dt;
        Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
        //cout << "frame g " << tmp_g.transpose() << endl;
        }
            var_imu = sqrt(var / ((int)all_image_frame.size() - 1));
        printf("IMU variation %f!\n", var_imu);
   //     if(var < 0.25)
   //     {
   //     printf("init IMU variation not enouth!\n");
   //     init_status = FAIL_IMU;
   //     fail_times++;
   //     return false;
   //     }
        }
        
        
//        保存最老帧信息 旋转和位置
        back_R0 = Rs[0];
        back_P0 = Ps[0];
//        依次把滑窗内信息前移
        if (frame_count == WINDOW_SIZE)
        {
            //交换滑动窗口中的位姿、速度、陀螺仪偏差、加速度buf、角速度buf等的位置
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);
                std::swap(pre_integrations[i], pre_integrations[i + 1]);
                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
            }
//            把滑窗末尾(10帧)信息给最新一帧(11帧)
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
            
            if(pre_integrations[WINDOW_SIZE] != NULL)
            {
                delete pre_integrations[WINDOW_SIZE];
            }
//            新实例化一个IMU预积分对象给下一个最新一帧
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
//            清空第11帧的buf
            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();
            
            if (solver_flag == INITIAL)
            {
                double t_0 = Headers[0];
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else  //non keyframe 删除的是滑窗第10帧。
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Eigen::Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Eigen::Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];
//                当前帧和前一帧之间的 IMU 预积分转换为当前帧和前二帧之间的 IMU 预积分
                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);
                
                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }
//            用最新一帧的信息覆盖上一帧信息
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];
//            因为已经把第11帧的信息覆盖了第10帧，所以现在把第11帧清除
            if(pre_integrations[WINDOW_SIZE]!=NULL)
            {
                delete pre_integrations[WINDOW_SIZE];
            }
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
            
            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();
//            滑窗
//            为什么这里可以不对前一帧进行边缘化而是直接丢弃，原因就是当前帧和前一帧很相似。
            slideWindowNew();
        }
    }
}

void VINS::slideWindowOld()
{
//    printf("marginalize back\n");
    point_cloud.clear();
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        
        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2
            &&it_per_id.solve_flag == 1 )
        {
            int imu_i = it_per_id.start_frame;
            Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Eigen::Vector3d tmp = Rs[imu_i] * (ric * pts_i + tic) + Ps[imu_i];
            point_cloud.push_back(tmp.cast<float>());
        }
    }
    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Eigen::Matrix3d R0, R1;
        Eigen::Vector3d P0, P1;
        R0 = back_R0 * ric;//滑窗原先的最老帧(被merge掉)的旋转(c->w)
        R1 = Rs[0] * ric;//滑窗原先第二老的帧(现在是最老帧)的旋转(c->w)
        P0 = back_P0 + back_R0 * tic;//滑窗原先的最老帧(被merge掉)的平移(c->w)
        P1 = Ps[0] + Rs[0] * tic;//滑窗原先第二老的帧(现在是最老帧)的平移(c->w)
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);//把首次在原先最老帧出现的特征点转移到原先第二老帧的相机坐标里(仅在slideWindowOld()出现过)
    }
    else
        f_manager.removeBack();//当最新一帧是关键帧时，用于merge滑窗内最老帧(仅在slideWindowOld()出现过)
    
    
    priorMap_f_manager.removeBack();
    
}
void VINS::slideWindowNew()
{
//    因为已经把第11帧的信息覆盖了第10帧，所以现在把第11帧清除
//    printf("marginalize front\n");
    //唯一用法：当最新一帧(11)不是关键帧时，用于merge滑窗内最新帧(10)(仅在slideWindowNew()出现过)
    f_manager.removeFront(frame_count);
    
    priorMap_f_manager.removeFront(frame_count);
}


void VINS::setFeature_tracker(FeatureTracker* _feature_tracker){
    feature_tracker=_feature_tracker;
}
