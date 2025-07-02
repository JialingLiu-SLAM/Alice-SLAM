//
//  inertial_factor.cpp
//  VINS_ios
//
//  Created by 张剑华 on 2021/4/8.
//  Copyright © 2021 栗大人. All rights reserved.
//


#include "inertial_factor.h"


Eigen::Matrix<double,9,9> EdgeInertialGS::sqrt_info;
double EdgeInertialGS::sum_t;

/**
EdgeInertialGS::EdgeInertialGS(const ImageFrame &_frame_i, const ImageFrame &_frame_j, const Eigen::Vector3d &_t_imu_cam) : frame_i(_frame_i), frame_j(_frame_j), t_imu_cam(_t_imu_cam){
    pre_integration_j=frame_j.pre_integration;
    q_ij=(frame_i.R.transpose() * frame_j.R);//确认一下 这个是不是imu坐标系的，是的话，不能写这，得更新，但是猜测 应该是相机坐标系的
    
    
    
    dt=pre_integration_j->sum_dt;
    
    
    
    //
    dp_dba=pre_integration_j->jacobian.block<3,3>(O_P, O_BA);
    dp_dbg = pre_integration_j->jacobian.block<3, 3>(O_P, O_BG);
    
    dq_dbg = pre_integration_j->jacobian.block<3, 3>(O_R, O_BG);
    
    dv_dba = pre_integration_j->jacobian.block<3, 3>(O_V, O_BA);
    dv_dbg = pre_integration_j->jacobian.block<3, 3>(O_V, O_BG);
    
    
    
    delta_q=pre_integration_j->delta_q;
    delta_p=pre_integration_j->delta_p;
    delta_v=pre_integration_j->delta_v;
    
    //这里本应该用第一帧的i,但是经常遇到它为空，这里就是上一时刻的偏置，所以可以用j
    bias_gyro=frame_j.pre_integration->linearized_bg;//原始值
//    bias_acc=frame_i.pre_integration->linearized_ba;
    
   
    sqrt_info = Eigen::LLT<Eigen::Matrix<double, 9, 9>>((pre_integration_j->covariance.block<9,9>(0,6)).inverse()).matrixL().transpose();
    
//    cout<<"测试sqrt_info 空不空：";
//    for(int i=0;i<9;i++){
//        for(int j=0;j<9;j++){
//            cout<<sqrt_info(i,j)<<" ";
//        }
//        cout<<endl;
//    }
//    cout<<endl<<endl;
//    sqrt_info =Eigen::Matrix<double,9,9>::Identity();
}

bool EdgeInertialGS::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
   
    
    //假设第一个参数是 陀螺仪bias
    Eigen::Vector3d b_gyro(parameters[0][0], parameters[0][1], parameters[0][2]);
    //第二个参数是 前一帧的速度
    Eigen::Vector3d v_i(parameters[1][0], parameters[1][1], parameters[1][2]);
    //第三个参数是 后一帧的速度
    Eigen::Vector3d v_j(parameters[2][0], parameters[2][1], parameters[2][2]);
    //第四个参数是 重力（什么坐标系的 还没弄清楚） C0坐标系的
    Eigen::Vector3d g(parameters[3][0], parameters[3][1], parameters[3][2]);
    //第五个参数是 尺度（什么坐标系的 还没弄清楚）传的是exp(scale_x)中的scale_x,下面的式子要改成对应的真实尺度
    double scale=parameters[4][0];
   
    
    
    //先进行一次更新 一阶线性更新
    //陀螺仪偏置变化量
    Eigen::Vector3d dbg = b_gyro - bias_gyro;
//    Eigen::Vector3d dba = b_acc - bias_acc;
    
    //得到相应变化的预计分量
    Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);//是不是要归一化？归一化了
    Eigen::Vector3d corrected_delta_v = delta_v+ dv_dbg * dbg;// + dv_dba * dba
    Eigen::Vector3d corrected_delta_p = delta_p  + dp_dbg * dbg;//+ dp_dba * dba
    
    
    //这里为什么会把陀螺仪bias放进来，是因为这个值的变化，会影响预积分的值
//    Eigen::Matrix3d tmp_A(3,3);
//    tmp_A.setZero();
//    tmp_A= pre_integration_j->jacobian.template block<3,3>(O_R, O_BG);
    Eigen::Vector3d tmp_b(3);
    tmp_b.setZero();
    tmp_b=2*((corrected_delta_q.inverse()* q_ij).vec());
    //这里矩阵可能得设的大一点，然后通过块进行操作
    Eigen::Map<Eigen::Matrix<double, 9, 1> > residual(residuals);
    residual.block<3,1>(3,0) =tmp_b;//q
    
//    cout<<"测试为什么出错 imu:";
//    cout<<b_gyro.transpose() <<","<<bias_gyro.transpose()<<endl;
//    cout<<delta_q.coeffs()<<" ,"<<dbg.transpose()<<endl;
//    for(int i=0;i<3;i++){
//        for(int j=0;j<3;j++){
//            cout<<dq_dbg(i,j)<<" ";
//
//        }
//        cout<<endl;
//
//    }
//    cout<<endl<<endl;
//
//    cout<<corrected_delta_q.coeffs()<<" ,"<<corrected_delta_q.inverse().coeffs()<<endl;
//
//    cout<<tmp_b.transpose()<<endl<<endl;
//
//    cout<<"q_ij:"<<q_ij.coeffs()<<endl;
    
    cout<<"测试residual q空不空";
    for(int i=0;i<3;i++){
        cout<<residual(3+i,0)<<" ";
        
    }
    cout<<endl<<endl;
    
       
    Eigen::Matrix3d tmp_A_2_00=-dt*Eigen::Matrix3d::Identity();
    Eigen::Matrix3d tmp_A_2_06=frame_i.R.transpose() * dt * dt /2 * Eigen::Matrix3d::Identity();
    Eigen::Vector3d tmp_A_2_09=frame_i.R.transpose()*(frame_j.T-frame_i.T)/100;//思考这个除100，是不是转换尺度单位
    Eigen::Vector3d tmp_b_2_00=corrected_delta_p+ frame_i.R.transpose() * frame_j.R*t_imu_cam-t_imu_cam;
    //3*1误差项
    residual.block<3,1>(0,0) =tmp_A_2_00*v_i+tmp_A_2_06*g+tmp_A_2_09*exp(scale)-tmp_b_2_00;//p
    
    cout<<"测试residual p空不空";
    for(int i=0;i<3;i++){
        cout<<residual[i,0]<<" ";
        
    }
    cout<<endl<<endl;
    
    
    Eigen::Matrix3d tmp_A_3_00=-Eigen::Matrix3d::Identity();
    Eigen::Matrix3d tmp_A_3_03=frame_i.R.transpose() * frame_j.R;
    Eigen::Matrix3d tmp_A_3_06=frame_i.R.transpose() * dt * Eigen::Matrix3d::Identity();
    Eigen::Vector3d tmp_b_3_00=corrected_delta_v;
    //3*1误差项
    residual.block<3,1>(6,0) =tmp_A_3_00*v_i+tmp_A_3_03*v_j+tmp_A_3_06*g-tmp_b_3_00;//v
    
    cout<<"测试residual v空不空";
    for(int i=0;i<3;i++){
        cout<<residual[6+i,0]<<" ";
        
    }
    cout<<endl<<endl;
    
  
   
//    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    
    if (jacobians)
    {
//陀螺仪bias
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_pose_bg(jacobians[0]);

            
            jacobian_pose_bg.block<3,3>(3,0)=-Utility::Qleft(q_ij.inverse() * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
            jacobian_pose_bg.block<3,3>(0,0)=-dp_dbg;
            jacobian_pose_bg.block<3,3>(6,0)=-dv_dbg;
            
            jacobian_pose_bg = sqrt_info * jacobian_pose_bg;
            
//            cout<<"测试 jacobian_pose_bg：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_pose_bg(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
        }
//前一帧的速度 只和bias_acc有关，和bias_gyro无关
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_pose_Vi(jacobians[1]);
            jacobian_pose_Vi.setZero();
            jacobian_pose_Vi.block<3,3>(0,0)=tmp_A_2_00;
            jacobian_pose_Vi.block<3,3>(6,0)=tmp_A_3_00;
            
            jacobian_pose_Vi = sqrt_info * jacobian_pose_Vi;
            
//            cout<<"测试 jacobian_pose_Vi：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_pose_Vi(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
        }
//后一帧的速度
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_pose_Vj(jacobians[2]);
            jacobian_pose_Vj.setZero();
            jacobian_pose_Vj.block<3,3>(6,0)=tmp_A_3_03;
            
            jacobian_pose_Vj = sqrt_info * jacobian_pose_Vj;
            
//            cout<<"测试 jacobian_pose_Vj：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_pose_Vj(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
          
        }
        //重力，这里并没有做对齐，只是求解出来C0坐标系下，重力这个向量的表示
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_gC0(jacobians[3]);
            jacobian_gC0.setZero();
            jacobian_gC0.block<3,3>(0,0)=tmp_A_2_06;
            jacobian_gC0.block<3,3>(6,0)=tmp_A_3_06;
            
            jacobian_gC0 = sqrt_info * jacobian_gC0;
            
//            cout<<"测试 jacobian_gC0：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_gC0(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
        }
        //尺度 是exp(scale_x)，这里的变量是scale_x
        if (jacobians[4])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 1>> jacobian_scaleX(jacobians[4]);//不理解 为什么是9，3,案例应该是9，1
            jacobian_scaleX.setZero();
//            jacobian_scaleX.block<3,1>(0,0)=tmp_A_2_09*exp(scale);
            jacobian_scaleX(0,0)=(tmp_A_2_09*exp(scale))[0];
            jacobian_scaleX(1,0)=(tmp_A_2_09*exp(scale))[1];
            jacobian_scaleX(2,0)=(tmp_A_2_09*exp(scale))[2];
            
            
            jacobian_scaleX = sqrt_info * jacobian_scaleX;
//            cout<<"测试 jacobian_scaleX：";
//            for(int i=0;i<9;i++){
//                cout<<jacobian_scaleX(i,0)<<" ";
//
//            }
//            cout<<endl<<endl;
        }
      
    }
    
    return true;
    
}
*/


EdgeInertialGS::EdgeInertialGS(const ImageFrame &_frame_i, const ImageFrame &_frame_j, const Eigen::Vector3d &_t_imu_cam) : frame_i(_frame_i), frame_j(_frame_j), t_imu_cam(_t_imu_cam){
    
    pre_integration_j=frame_j.pre_integration;
    q_ij=(frame_i.R.transpose() * frame_j.R);//确认一下 这个是不是imu坐标系的，是的话，不能写这，得更新，但是猜测 应该是相机坐标系的 这个是imu坐标系
    
    //这里是想要前后两帧时间差
    dt=pre_integration_j->sum_dt;
    
    
    
    //这里再思考一下，应该写错了 应该是这样的
    dp_dba=pre_integration_j->jacobian.block<3,3>(O_P, O_BA);
    dp_dbg = pre_integration_j->jacobian.block<3, 3>(O_P, O_BG);
    dq_dbg = pre_integration_j->jacobian.block<3, 3>(O_R, O_BG);
    dv_dba = pre_integration_j->jacobian.block<3, 3>(O_V, O_BA);
    dv_dbg = pre_integration_j->jacobian.block<3, 3>(O_V, O_BG);
    
    //这个表示两帧之间 预积分出来的相对值
    delta_q=pre_integration_j->delta_q;
    delta_p=pre_integration_j->delta_p;
    delta_v=pre_integration_j->delta_v;
    
    
    //这里要思考一下 是当前帧的acc,还是前一帧的acc
    acc=pre_integration_j->acc_1;
       
    
    
    //这里本应该用第一帧的i,但是经常遇到它为空，这里就是上一时刻的偏置，所以可以用j
    bias_gyro=frame_j.pre_integration->linearized_bg;//原始值
    bias_acc=frame_j.pre_integration->linearized_ba;
    

    sqrt_info = Eigen::LLT<Eigen::Matrix<double, 9, 9>>(pre_integration_j->covariance_my.inverse()).matrixL().transpose();


//    sqrt_info =Eigen::Matrix<double,9,9>::Identity();
}

//这里是没评判acc_bias
bool EdgeInertialGS::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    //假设第一个参数是 陀螺仪bias
    Eigen::Vector3d b_gyro(parameters[0][0], parameters[0][1], parameters[0][2]);
    //第二个参数是 前一帧的速度
    Eigen::Vector3d v_i(parameters[1][0], parameters[1][1], parameters[1][2]);
    //第三个参数是 后一帧的速度
    Eigen::Vector3d v_j(parameters[2][0], parameters[2][1], parameters[2][2]);
    //第四个参数是 重力（什么坐标系的 还没弄清楚） C0坐标系的
    Eigen::Vector3d g(parameters[3][0], parameters[3][1], parameters[3][2]);
    //第五个参数是 尺度（什么坐标系的 还没弄清楚）传的是exp(scale_x)中的scale_x,下面的式子要改成对应的真实尺度
    double scale=parameters[4][0];
    
   
    
    
    //先进行一次更新 一阶线性更新
    //陀螺仪偏置变化量
    Eigen::Vector3d dbg = b_gyro - bias_gyro;
    
//    //得到相应变化的预计分量
//    //看一下 如果dbg为0，会不会影响到最终的corrected_delta_q。因为第一次进来，肯定是希望等于delta_q 是等于的
    Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);//是不是要归一化？归一化了
    Eigen::Vector3d corrected_delta_v = delta_v+ dv_dbg * dbg;//
    Eigen::Vector3d corrected_delta_p = delta_p  + dp_dbg * dbg;//
    
    //减去偏置后的加速度
//    Eigen::Vector3d acc_real= acc-b_acc;
//    //雅克比也要做更新
//    Eigen::Matrix3d Wacc ;
//    Wacc<<0, -acc_real(2), acc_real(1),acc_real(2), 0, -acc_real(0),-acc_real(1), acc_real(0), 0;
//    Eigen::Matrix3d dR=delta_q.toRotationMatrix();
//    Eigen::Matrix3d JPa = dp_dba + dv_dba*dt -0.5f*dR*dt*dt;
//    Eigen::Matrix3d JPg = dp_dbg + dv_dbg*dt -0.5f*dR*dt*dt*Wacc*JRg;
//    Eigen::Matrix3d JVa = dv_dba - dR*dt;
//    Eigen::Matrix3d JVg = dv_dbg - dR*dt*Wacc*dq_dbg;
    
    //这里为什么会把陀螺仪bias放进来，是因为这个值的变化，会影响预积分的值
//    Eigen::Matrix3d tmp_A(3,3);
//    tmp_A.setZero();
//    tmp_A= pre_integration_j->jacobian.template block<3,3>(O_R, O_BG);
    Eigen::Vector3d tmp_b(3);
    tmp_b.setZero();
    tmp_b=2.0*(corrected_delta_q.inverse()* q_ij).vec();
    //这里矩阵可能得设的大一点，然后通过块进行操作
    Eigen::Map<Eigen::Matrix<double, 9, 1> > residual(residuals);
    residual.block<3,1>(3,0) =tmp_b;//q
        
       
    Eigen::Matrix3d tmp_A_2_00=-dt*Eigen::Matrix3d::Identity();
    Eigen::Matrix3d tmp_A_2_06=frame_i.R.transpose() * dt * dt /2.0 * Eigen::Matrix3d::Identity();
    Eigen::Vector3d tmp_A_2_09= frame_i.R.transpose()*(frame_j.T-frame_i.T)/100.0; //思考这个除100，是不是转换尺度单位
    Eigen::Vector3d tmp_b_2_00=corrected_delta_p+ frame_i.R.transpose() * frame_j.R*t_imu_cam-t_imu_cam;
    //3*1误差项
    residual.block<3,1>(0,0) =tmp_A_2_00*v_i+tmp_A_2_06*g+tmp_A_2_09*exp(scale)-tmp_b_2_00;//p
    
//    residual.block<3,1>(0,0)=frame_i.R.transpose()*(0.5*g*dt*dt+(frame_j.T-frame_i.T)/100.0*exp(scale)-v_i*dt)-corrected_delta_p;
    
    
    
    Eigen::Matrix3d tmp_A_3_00=-Eigen::Matrix3d::Identity();
    Eigen::Matrix3d tmp_A_3_03=frame_i.R.transpose() * frame_j.R;
    Eigen::Matrix3d tmp_A_3_06=frame_i.R.transpose() * dt * Eigen::Matrix3d::Identity();
    Eigen::Vector3d tmp_b_3_00=corrected_delta_v;
    //3*1误差项
    residual.block<3,1>(6,0) =tmp_A_3_00*v_i+tmp_A_3_03*v_j+tmp_A_3_06*g-tmp_b_3_00;//v
    
//    residual.block<3,1>(6,0)=frame_i.R.transpose()*(g*dt+v_j-v_i)-corrected_delta_v;
 
   
    residual = sqrt_info * residual;
    
    if (jacobians)
    {
//陀螺仪bias
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_pose_bg(jacobians[0]);

            
            jacobian_pose_bg.block<3,3>(3,0)=-Utility::Qleft(q_ij.inverse() * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
            jacobian_pose_bg.block<3,3>(0,0)=-dp_dbg;
            jacobian_pose_bg.block<3,3>(6,0)=-dv_dbg;
            
            jacobian_pose_bg = sqrt_info * jacobian_pose_bg;
            
//            cout<<"测试 jacobian_pose_bg：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_pose_bg(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
        }
//前一帧的速度 只和bias_acc有关，和bias_gyro无关
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_pose_Vi(jacobians[1]);
            jacobian_pose_Vi.setZero();
            jacobian_pose_Vi.block<3,3>(0,0)=tmp_A_2_00;
            jacobian_pose_Vi.block<3,3>(6,0)=tmp_A_3_00;
            
//            jacobian_pose_Vi.block<3,3>(0,0)=-frame_i.R.transpose()*dt;
//            jacobian_pose_Vi.block<3,3>(6,0)=-frame_i.R.transpose();
            
            jacobian_pose_Vi = sqrt_info * jacobian_pose_Vi;
            
//            cout<<"测试 jacobian_pose_Vi：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_pose_Vi(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
        }
//后一帧的速度
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_pose_Vj(jacobians[2]);
            jacobian_pose_Vj.setZero();
            jacobian_pose_Vj.block<3,3>(6,0)=tmp_A_3_03;
            
//            jacobian_pose_Vj.block<3,3>(6,0)=frame_i.R.transpose();
            
            jacobian_pose_Vj = sqrt_info * jacobian_pose_Vj;
            
//            cout<<"测试 jacobian_pose_Vj：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_pose_Vj(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
          
        }
        //重力，这里并没有做对齐，只是求解出来C0坐标系下，重力这个向量的表示
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_gC0(jacobians[3]);
            jacobian_gC0.setZero();
            
            jacobian_gC0.block<3,3>(0,0)=tmp_A_2_06;
            jacobian_gC0.block<3,3>(6,0)=tmp_A_3_06;
            
//            jacobian_gC0.block<3,3>(0,0)=frame_i.R.transpose()*0.5*dt*dt;
//            jacobian_gC0.block<3,3>(6,0)=frame_i.R.transpose()*dt;
            
            jacobian_gC0 = sqrt_info * jacobian_gC0;
            
//            cout<<"测试 jacobian_gC0：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_gC0(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
        }
        //尺度 是exp(scale_x)，这里的变量是scale_x
        if (jacobians[4])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 1>> jacobian_scaleX(jacobians[4]);//不理解 为什么是9，3,案例应该是9，1
            jacobian_scaleX.setZero();
            Eigen::Vector3d b=tmp_A_2_09*exp(scale);
            
//            Eigen::Vector3d b=frame_i.R.transpose()*(frame_j.T-frame_i.T)/100.0*exp(scale);
            
            jacobian_scaleX(0,0)=b[0];
            jacobian_scaleX(1,0)=b[1];
            jacobian_scaleX(2,0)=b[2];
            
            
            jacobian_scaleX = sqrt_info * jacobian_scaleX;
//            cout<<"测试 jacobian_scaleX：";
//            for(int i=0;i<9;i++){
//                cout<<jacobian_scaleX(i,0)<<" ";
//
//            }
//            cout<<endl<<endl;
        }
      
      
    }
    
    return true;
    
}
/**
//这里评判了acc_bias
bool EdgeInertialGS::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    //假设第一个参数是 陀螺仪bias
    Eigen::Vector3d b_gyro(parameters[0][0], parameters[0][1], parameters[0][2]);
    //第二个参数是 前一帧的速度
    Eigen::Vector3d v_i(parameters[1][0], parameters[1][1], parameters[1][2]);
    //第三个参数是 后一帧的速度
    Eigen::Vector3d v_j(parameters[2][0], parameters[2][1], parameters[2][2]);
    //第四个参数是 重力（什么坐标系的 还没弄清楚） C0坐标系的
    Eigen::Vector3d g(parameters[3][0], parameters[3][1], parameters[3][2]);
    //第五个参数是 尺度（什么坐标系的 还没弄清楚）传的是exp(scale_x)中的scale_x,下面的式子要改成对应的真实尺度
    double scale=parameters[4][0];
    Eigen::Vector3d b_acc(parameters[5][0], parameters[5][1], parameters[5][2]);
   
    
    
    //先进行一次更新 一阶线性更新
    //陀螺仪偏置变化量
    Eigen::Vector3d dbg = b_gyro - bias_gyro;
    Eigen::Vector3d dba = b_acc - bias_acc;
//
//    //得到相应变化的预计分量
//    //看一下 如果dbg为0，会不会影响到最终的corrected_delta_q。因为第一次进来，肯定是希望等于delta_q 是等于的
    Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);//是不是要归一化？归一化了
    Eigen::Vector3d corrected_delta_v = delta_v+ dv_dbg * dbg+ dv_dba * dba;//
    Eigen::Vector3d corrected_delta_p = delta_p  + dp_dbg * dbg+ dp_dba * dba;//
    
    //减去偏置后的加速度
//    Eigen::Vector3d acc_real= acc-b_acc;
//    //雅克比也要做更新
//    Eigen::Matrix3d Wacc ;
//    Wacc<<0, -acc_real(2), acc_real(1),acc_real(2), 0, -acc_real(0),-acc_real(1), acc_real(0), 0;
//    Eigen::Matrix3d dR=delta_q.toRotationMatrix();
//    Eigen::Matrix3d JPa = dp_dba + dv_dba*dt -0.5f*dR*dt*dt;
//    Eigen::Matrix3d JPg = dp_dbg + dv_dbg*dt -0.5f*dR*dt*dt*Wacc*JRg;
//    Eigen::Matrix3d JVa = dv_dba - dR*dt;
//    Eigen::Matrix3d JVg = dv_dbg - dR*dt*Wacc*dq_dbg;
    
    //这里为什么会把陀螺仪bias放进来，是因为这个值的变化，会影响预积分的值
//    Eigen::Matrix3d tmp_A(3,3);
//    tmp_A.setZero();
//    tmp_A= pre_integration_j->jacobian.template block<3,3>(O_R, O_BG);
    Eigen::Vector3d tmp_b(3);
    tmp_b.setZero();
    tmp_b=2.0*((corrected_delta_q.inverse()* q_ij).vec());
    //这里矩阵可能得设的大一点，然后通过块进行操作
    Eigen::Map<Eigen::Matrix<double, 9, 1> > residual(residuals);
    residual.block<3,1>(3,0) =tmp_b;//q
        
       
    Eigen::Matrix3d tmp_A_2_00=-dt*Eigen::Matrix3d::Identity();
    Eigen::Matrix3d tmp_A_2_06=frame_i.R.transpose() * dt * dt /2.0 * Eigen::Matrix3d::Identity();
    Eigen::Vector3d tmp_A_2_09=frame_i.R.transpose()*(frame_j.T-frame_i.T)/100.0;//思考这个除100，是不是转换尺度单位
    Eigen::Vector3d tmp_b_2_00=corrected_delta_p+ frame_i.R.transpose() * frame_j.R*t_imu_cam-t_imu_cam;
    //3*1误差项
    residual.block<3,1>(0,0) =tmp_A_2_00*v_i+tmp_A_2_06*g+tmp_A_2_09*exp(scale)-tmp_b_2_00;//p
    
//    residual.block<3,1>(0,0)=frame_i.R.transpose()*(0.5*g*dt*dt+(frame_j.T-frame_i.T)/100.0*exp(scale)-v_i*dt)-corrected_delta_p;
    
    
    
    Eigen::Matrix3d tmp_A_3_00=-Eigen::Matrix3d::Identity();
    Eigen::Matrix3d tmp_A_3_03=frame_i.R.transpose() * frame_j.R;
    Eigen::Matrix3d tmp_A_3_06=frame_i.R.transpose() * dt * Eigen::Matrix3d::Identity();
    Eigen::Vector3d tmp_b_3_00=corrected_delta_v;
    //3*1误差项
    residual.block<3,1>(6,0) =tmp_A_3_00*v_i+tmp_A_3_03*v_j+tmp_A_3_06*g-tmp_b_3_00;//v
    
//    residual.block<3,1>(6,0)=frame_i.R.transpose()*(g*dt+v_j-v_i)-corrected_delta_v;
 
   
    residual = sqrt_info * residual;
    
    if (jacobians)
    {
//陀螺仪bias
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_pose_bg(jacobians[0]);

            
            jacobian_pose_bg.block<3,3>(3,0)=-Utility::Qleft(q_ij.inverse() * delta_q).bottomRightCorner<3, 3>() * dq_dbg;
            jacobian_pose_bg.block<3,3>(0,0)=-dp_dbg;
            jacobian_pose_bg.block<3,3>(6,0)=-dv_dbg;
            
            jacobian_pose_bg = sqrt_info * jacobian_pose_bg;
            
//            cout<<"测试 jacobian_pose_bg：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_pose_bg(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
        }
//前一帧的速度 只和bias_acc有关，和bias_gyro无关
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_pose_Vi(jacobians[1]);
            jacobian_pose_Vi.setZero();
            jacobian_pose_Vi.block<3,3>(0,0)=tmp_A_2_00;
            jacobian_pose_Vi.block<3,3>(6,0)=tmp_A_3_00;
            
//            jacobian_pose_Vi.block<3,3>(0,0)=-frame_i.R.transpose()*dt;
//            jacobian_pose_Vi.block<3,3>(6,0)=-frame_i.R.transpose();
            
            jacobian_pose_Vi = sqrt_info * jacobian_pose_Vi;
            
//            cout<<"测试 jacobian_pose_Vi：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_pose_Vi(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
        }
//后一帧的速度
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_pose_Vj(jacobians[2]);
            jacobian_pose_Vj.setZero();
            jacobian_pose_Vj.block<3,3>(6,0)=tmp_A_3_03;
            
//            jacobian_pose_Vj.block<3,3>(6,0)=frame_i.R.transpose();
            
            jacobian_pose_Vj = sqrt_info * jacobian_pose_Vj;
            
//            cout<<"测试 jacobian_pose_Vj：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_pose_Vj(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
          
        }
        //重力，这里并没有做对齐，只是求解出来C0坐标系下，重力这个向量的表示
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_gC0(jacobians[3]);
            jacobian_gC0.setZero();
            
            jacobian_gC0.block<3,3>(0,0)=tmp_A_2_06;
            jacobian_gC0.block<3,3>(6,0)=tmp_A_3_06;
            
//            jacobian_gC0.block<3,3>(0,0)=frame_i.R.transpose()*0.5*dt*dt;
//            jacobian_gC0.block<3,3>(6,0)=frame_i.R.transpose()*dt;
            
            jacobian_gC0 = sqrt_info * jacobian_gC0;
            
//            cout<<"测试 jacobian_gC0：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_gC0(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
        }
        //尺度 是exp(scale_x)，这里的变量是scale_x
        if (jacobians[4])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 1>> jacobian_scaleX(jacobians[4]);//不理解 为什么是9，3,案例应该是9，1
            jacobian_scaleX.setZero();
            Eigen::Vector3d b=tmp_A_2_09*exp(scale);
            
//            Eigen::Vector3d b=frame_i.R.transpose()*(frame_j.T-frame_i.T)/100.0*exp(scale);
            
            jacobian_scaleX(0,0)=b[0];
            jacobian_scaleX(1,0)=b[1];
            jacobian_scaleX(2,0)=b[2];
            
            
            jacobian_scaleX = sqrt_info * jacobian_scaleX;
//            cout<<"测试 jacobian_scaleX：";
//            for(int i=0;i<9;i++){
//                cout<<jacobian_scaleX(i,0)<<" ";
//
//            }
//            cout<<endl<<endl;
        }
        if (jacobians[5])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_pose_ba(jacobians[5]);

            
            jacobian_pose_ba.setZero();
            jacobian_pose_ba.block<3,3>(0,0)=-dp_dba;
            jacobian_pose_ba.block<3,3>(6,0)=-dv_dba;
            
            jacobian_pose_ba = sqrt_info * jacobian_pose_ba;
            
//            cout<<"测试 jacobian_pose_ba：";
//            for(int i=0;i<9;i++){
//                for(int j=0;j<3;j++){
//                    cout<<jacobian_pose_ba(i,j)<<" ";
//
//                }
//                cout<<endl;
//            }
//            cout<<endl<<endl;
        }
      
    }
    
    return true;
    
}
*/
/**
 //这个如果要用，还没改完
Eigen::Matrix<double,12,12> EdgeInertialGS::sqrt_info;
double EdgeInertialGS::sum_t;

EdgeInertialGS::EdgeInertialGS(const ImageFrame &_frame_i, const ImageFrame &_frame_j, const Eigen::Vector3d &_t_imu_cam) : frame_i(_frame_i), frame_j(_frame_j), t_imu_cam(_t_imu_cam){
    pre_integration_j=frame_j.pre_integration;
    q_ij=(frame_i.R.transpose() * frame_j.R);//确认一下 这个是不是imu坐标系的，是的话，不能写这，得更新，但是猜测 应该是相机坐标系的
  
    dt=pre_integration_j->sum_dt;
    
    
    
    //
    dp_dba=pre_integration_j->jacobian.block<3,3>(O_P, O_BA);
    dp_dbg = pre_integration_j->jacobian.block<3, 3>(O_P, O_BG);
    
    dq_dbg = pre_integration_j->jacobian.block<3, 3>(O_R, O_BG);
    
    dv_dba = pre_integration_j->jacobian.block<3, 3>(O_V, O_BA);
    dv_dbg = pre_integration_j->jacobian.block<3, 3>(O_V, O_BG);
    
    
    
    delta_q=pre_integration_j->delta_q;
    bias_gyro=pre_integration_j->linearized_bg;//原始值
    
   
    sqrt_info = Eigen::LLT<Eigen::Matrix<double, 9, 9>>((pre_integration_j->covariance.block<9,9>(0,6)).inverse()).matrixL().transpose();
    
//    cout<<"测试sqrt_info 空不空：";
//    for(int i=0;i<9;i++){
//        for(int j=0;j<9;j++){
//            cout<<sqrt_info(i,j)<<" ";
//        }
//        cout<<endl;
//    }
//    cout<<endl<<endl;
//    sqrt_info =Eigen::Matrix<double,9,9>::Identity();
}

bool EdgeInertialGS::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
   
    
    //假设第一个参数是 陀螺仪bias 这个是前一帧的
    Eigen::Vector3d b_gyro_i(parameters[0][0], parameters[0][1], parameters[0][2]);
    //第二个参数是 前一帧的速度
    Eigen::Vector3d v_i(parameters[1][0], parameters[1][1], parameters[1][2]);
    //第三个参数是 后一帧的速度
    Eigen::Vector3d v_j(parameters[2][0], parameters[2][1], parameters[2][2]);
    //第四个参数是 重力（什么坐标系的 还没弄清楚） C0坐标系的
    Eigen::Vector3d g(parameters[3][0], parameters[3][1], parameters[3][2]);
    //第五个参数是 尺度（什么坐标系的 还没弄清楚）传的是exp(scale_x)中的scale_x,下面的式子要改成对应的真实尺度
    double scale=parameters[4][0];
    //这个后一帧的
    Eigen::Vector3d b_gyro_j(parameters[5][0], parameters[5][1], parameters[5][2]);
    
    
    
    //先进行一次更新 一阶线性更新
//    Eigen::Vector3d dba = Bai - linearized_ba;
    //陀螺仪偏置变化量
    Eigen::Vector3d dbg = b_gyro_i - bias_gyro;
    //得到相应变化的预计分量
    Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);//是不是要归一化？
//    Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
//    Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;
    
    
    //这里为什么会把陀螺仪bias放进来，是因为这个值的变化，会影响预积分的值
//    Eigen::Matrix3d tmp_A(3,3);
//    tmp_A.setZero();
//    tmp_A= pre_integration_j->jacobian.template block<3,3>(O_R, O_BG);
    Eigen::Vector3d tmp_b(3);
    tmp_b.setZero();
    tmp_b=2*(corrected_delta_q.inverse()* q_ij).vec();
    //这里矩阵可能得设的大一点，然后通过块进行操作
    Eigen::Map<Eigen::Matrix<double, 12, 1> > residual(residuals);
    residual.block<3,1>(3,0) =tmp_b;//q
    
    cout<<"测试为什么出错 imu:";
    cout<<delta_q.coeffs()<<" ,"<<dbg.transpose()<<endl;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            cout<<dq_dbg(i,j)<<" ";
            
        }
        cout<<endl;
        
    }
    cout<<endl<<endl;
    
    cout<<corrected_delta_q.coeffs()<<" ,"<<corrected_delta_q.inverse().coeffs()<<endl;
    
    cout<<"测试residual q空不空";
    for(int i=0;i<3;i++){
        cout<<residual[3+i,0]<<" ";
        
    }
    cout<<endl<<endl;
    
       
    Eigen::Matrix3d tmp_A_2_00=-dt*Eigen::Matrix3d::Identity();
    Eigen::Matrix3d tmp_A_2_06=frame_i.R.transpose() * dt * dt /2 * Eigen::Matrix3d::Identity();
    Eigen::Vector3d tmp_A_2_09=frame_i.R.transpose()*(frame_j.T-frame_i.T)/100;//思考这个除100，是不是转换尺度单位
    Eigen::Vector3d tmp_b_2_00=pre_integration_j->delta_p+ frame_i.R.transpose() * frame_j.R*t_imu_cam-t_imu_cam;
    //3*1误差项
    residual.block<3,1>(0,0) =tmp_A_2_00*v_i+tmp_A_2_06*g+tmp_A_2_09*exp(scale)-tmp_b_2_00;//p
    
    cout<<"测试residual p空不空";
    for(int i=0;i<3;i++){
        cout<<residual[i,0]<<" ";
        
    }
    cout<<endl<<endl;
    
    
    Eigen::Matrix3d tmp_A_3_00=-Eigen::Matrix3d::Identity();
    Eigen::Matrix3d tmp_A_3_03=frame_i.R.transpose() * frame_j.R;
    Eigen::Matrix3d tmp_A_3_06=frame_i.R.transpose() * dt * Eigen::Matrix3d::Identity();
    Eigen::Vector3d tmp_b_3_00=pre_integration_j->delta_v;
    //3*1误差项
    residual.block<3,1>(6,0) =tmp_A_3_00*v_i+tmp_A_3_03*v_j+tmp_A_3_06*g-tmp_b_3_00;//v
    
    cout<<"测试residual v空不空";
    for(int i=0;i<3;i++){
        cout<<residual[6+i,0]<<" ";
        
    }
    cout<<endl<<endl;
    
    
    residual.block<3,1>(9,0)=b_gyro_j-b_gyro_i;
    
    //旋转 只和bias_gyro有关
    //速度 只和bias_acc有关
    //平移 一定和bias_acc bias_gyro有关
    
   
    //sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    
    if (jacobians)
    {
//        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
//        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
//        Eigen::Matrix3d ric = qic.toRotationMatrix();
//        Eigen::Matrix<double, 2, 3> reduce(2, 3);
//        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
//        0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

//        reduce = sqrt_info * reduce;
//陀螺仪bias
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 12, 3, Eigen::RowMajor>> jacobian_pose_bg(jacobians[0]);

            
            jacobian_pose_bg.block<3,3>(3,0)=-Utility::Qleft(q_ij.inverse() * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
            jacobian_pose_bg.block<3,3>(0,0)=-dp_dbg;
            jacobian_pose_bg.block<3,3>(6,0)=-dv_dbg;
            jacobian_pose_bg.block<3,3>(9,0)=-Eigen::Matrix3d::Identity();
            
            jacobian_pose_bg = sqrt_info * jacobian_pose_bg;
        }
//前一帧的速度 只和bias_acc有关，和bias_gyro无关
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 12, 3, Eigen::RowMajor>> jacobian_pose_Vi(jacobians[1]);
            jacobian_pose_Vi.setZero();
            jacobian_pose_Vi.block<3,3>(0,0)=tmp_A_2_00;
            jacobian_pose_Vi.block<3,3>(6,0)=tmp_A_3_00;
            
            jacobian_pose_Vi = sqrt_info * jacobian_pose_Vi;
        }
//后一帧的速度
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 12, 3, Eigen::RowMajor>> jacobian_pose_Vj(jacobians[2]);
            jacobian_pose_Vj.setZero();
            jacobian_pose_Vj.block<3,3>(6,0)=tmp_A_3_03;
            
            jacobian_pose_Vj = sqrt_info * jacobian_pose_Vj;
          
        }
        //重力，这里并没有做对齐，只是求解出来C0坐标系下，重力这个向量的表示
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 12, 3, Eigen::RowMajor>> jacobian_gC0(jacobians[3]);
            jacobian_gC0.setZero();
            jacobian_gC0.block<3,3>(0,0)=tmp_A_2_06;
            jacobian_gC0.block<3,3>(6,0)=tmp_A_3_06;
            
            jacobian_gC0 = sqrt_info * jacobian_gC0;
        }
        //尺度 是exp(scale_x)，这里的变量是scale_x
        if (jacobians[4])
        {
            Eigen::Map<Eigen::Matrix<double, 12, 1>> jacobian_scaleX(jacobians[4]);//不理解 为什么是9，3,案例应该是9，1
            jacobian_scaleX.setZero();
            jacobian_scaleX.block<3,1>(0,0)=tmp_A_2_09*exp(scale);
            
            jacobian_scaleX = sqrt_info * jacobian_scaleX;
            
        }
        //bias_gyro 下一时刻的 
        if (jacobians[5])
        {
            Eigen::Map<Eigen::Matrix<double, 12, 3>> jacobian_bias_j(jacobians[5]);
            jacobian_bias_j.setZero();
            jacobian_bias_j.block<3,3>(9,0)=Eigen::Matrix3d::Identity();
            
            jacobian_bias_j = sqrt_info * jacobian_bias_j;
            
        }
    }
    
    return true;
    
}

*/
