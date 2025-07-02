//
//  initial_aligment.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/12/26.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "initial_aligment.hpp"
#include "inertial_factor.h"
//陀螺仪bias校正
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs)
{
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    Eigen::Vector3d delta_bg;
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
//    cout<<"all_image_frame.size="<<all_image_frame.size()<<endl;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        Eigen::MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        Eigen::VectorXd tmp_b(3);
        tmp_b.setZero();
        //对应公式中的四元数乘积运算：q_ij = (q^c0_bk)^-1 * (q^c0_bk+1)
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
        //tmp_A = J_j_bw
        //O_R的值为3 O_BG的值为12
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);//3 12
        //tmp_b = 2 * ((r^bk_bk+1)^-1 * (q^c0_bk)^-1 * (q^c0_bk+1))_vec
        //      = 2 * ((r^bk_bk+1)^-1 * q_ij)_vec
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        //tmp_A * delta_bg = tmp_b
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
        
//        cout<<"测试 测试：tmp_b="<<(frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec()<<endl;
    }
    //进行ldlt分解
    delta_bg = A.ldlt().solve(b);
//    cout << " delta_bg ! " << delta_bg.transpose() << endl;
    
    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += delta_bg;
    //计算出陀螺仪偏差后再利用新的陀螺仪偏差进行预积分求解
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Eigen::Vector3d::Zero(), Bgs[0]);
    }
}

//在半径为g=9.81的半球上找到切面的一对正交基，存放在bc当中
Eigen::MatrixXd TangentBasis(Eigen::Vector3d &g0)
{
    Eigen::Vector3d b, c;
    Eigen::Vector3d a = g0.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    Eigen::MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

void RefineGravity(map<double, ImageFrame> &all_image_frame, Eigen::Vector3d &g, Eigen::VectorXd &x)
{
    Eigen::Vector3d g0 = g.normalized() * G_NORM;
    Eigen::Vector3d lx, ly;
    //Eigen::VectorXd x;
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 2 + 1;
    
    Eigen::MatrixXd A{n_state, n_state};
    A.setZero();
    Eigen::VectorXd b{n_state};
    b.setZero();
    
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    //迭代求解4次
    for(int k = 0; k < 4; k++)
    {
        Eigen::MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);
        int i = 0;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);
            
            Eigen::MatrixXd tmp_A(6, 9);
            tmp_A.setZero();
            Eigen::VectorXd tmp_b(6);
            tmp_b.setZero();
            
            double dt = frame_j->second.pre_integration->sum_dt;
            
            
            tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
            tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity() * lxly;
            tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;
            Eigen::Vector3d TIC;
            TIC<<TIC_X,TIC_Y,TIC_Z;
            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC - TIC - frame_i->second.R.transpose() * dt * dt / 2 * g0;
            //tmp_b.block<3, 1>(0, 0) = imu_factors[i + 1]->pre_integration.delta_p;
            
            tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
            tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity() * lxly;
            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity() * g0;
            
            
            Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
            //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
            //Eigen::MatrixXd cov_inv = cov.inverse();
            cov_inv.setIdentity();
            
            Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
            Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;
            
            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();
            
            A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
            b.tail<3>() += r_b.tail<3>();
            
            A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
            A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
        A = A * 1000.0;
        b = b * 1000.0;
        x = A.ldlt().solve(b);
        Eigen::VectorXd dg = x.segment<2>(n_state - 3);
        g0 = (g0 + lxly * dg).normalized() * G_NORM;
        //double s = x(n_state - 1);
    }
    g = g0;
}


void RefineGravity_2(map<double, ImageFrame> &all_image_frame, Eigen::Vector3d &g, Eigen::VectorXd &x)
{
    Eigen::Vector3d g0 = g.normalized() * G_NORM;
    Eigen::Vector3d lx, ly;
    //Eigen::VectorXd x;
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 2 + 1;
    
    Eigen::MatrixXd A{n_state, n_state};
    A.setZero();
    Eigen::VectorXd b{n_state};
    b.setZero();
    
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    //迭代求解4次
    for(int k = 0; k < 4; k++)
    {
        Eigen::MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);
        int i = 0;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);
            
            Eigen::MatrixXd tmp_A(6, 9);
            tmp_A.setZero();
            Eigen::VectorXd tmp_b(6);
            tmp_b.setZero();
            
            double dt = frame_j->second.pre_integration->sum_dt;
            
            
            tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
            tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity() * lxly;
            tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;
            Eigen::Vector3d TIC;
            TIC<<TIC_X,TIC_Y,TIC_Z;
            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC - TIC - frame_i->second.R.transpose() * dt * dt / 2 * g0;
            //tmp_b.block<3, 1>(0, 0) = imu_factors[i + 1]->pre_integration.delta_p;
            
            tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
            tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity() * lxly;
            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity() * g0;
            
            
            Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
            //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
            //Eigen::MatrixXd cov_inv = cov.inverse();
            cov_inv.setIdentity();
            
            Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
            Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;
            
            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();
            
            A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
            b.tail<3>() += r_b.tail<3>();
            
            A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
            A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
        A = A * 1000.0;
        b = b * 1000.0;
        x = A.ldlt().solve(b);
        Eigen::VectorXd dg = x.segment<2>(n_state - 3);
        g0 = (g0 + lxly * dg).normalized() * G_NORM;
        //double s = x(n_state - 1);
    }
    g = g0;
}
bool SolveScale(map<double, ImageFrame> &all_image_frame, Eigen::Vector3d &g, Eigen::VectorXd &x)
{
    printf("SolveScale\n");
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 3 + 1;
    
    Eigen::MatrixXd A{n_state, n_state};
    A.setZero();
    Eigen::VectorXd b{n_state};
    b.setZero();
    
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);
        //tmp_A为6*10的矩阵，就是H
        Eigen::MatrixXd tmp_A(6, 10);
        tmp_A.setZero();
        //tmp_b对应b
        Eigen::VectorXd tmp_b(6);
        tmp_b.setZero();
        
        double dt = frame_j->second.pre_integration->sum_dt;
        //-I*delta_tk
        tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
        //1/2*R_c0^bk*deltat_k^2*
        tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity();
        //R_c0^bk*(bar{p}_{c_{k+1}}^{c_0}-bar{p}_{c_{k}}^{c_0})
        tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;
        Eigen::Vector3d TIC;
        TIC<<TIC_X,TIC_Y,TIC_Z;
        //alpha_{b_{k+1}}^{b_k}+R_{c_0}^{b_k}*R_{b_{k+1}}^c_0*p_c^b-p_c^b
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC - TIC;
        //tmp_b.block<3, 1>(0, 0) = imu_factors[i + 1]->pre_integration.delta_p;
        //cout << "delta_p   " << frame_j->second.imu_factor->pre_integration.delta_p.transpose() << endl;
        //-I
        tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
        //R_c0^{b_k}*R_{b_{k+1}}^c0
        tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
        //R_{c_0}^{b_k}*delta t
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity();
        //beta_{b_{k+1}}^{b_k}
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        //cout << "delta_v   " << frame_j->second.pre_integration.delta_v.transpose() << endl;
        
        Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //Eigen::MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity();//变为单位矩阵
        //10*6  6*6  6*10 = 10*10
        Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        //10*6  6*6  6*1 = 10*1
        Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;
        //构造A和b
        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();
        
        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
        b.tail<4>() += r_b.tail<4>();
        
        A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
        A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    //对Ax=b进行分解求出x
    x = A.ldlt().solve(b);
    //从求解出的x向量里边取出最后边的尺度s
    double s = x(n_state - 1) / 100.0;
    printf("estimated scale: %f\n", s);
    //取出对重力向量g的计算值
    g = x.segment<3>(n_state - 4);
//    cout << " result g     " << g.norm() << " " << g.transpose() << endl;
    if(fabs(g.norm() - G_NORM) > G_THRESHOLD ||  s < 0)
    {
        return false;
    }
    
    RefineGravity(all_image_frame, g, x);
    s = (x.tail<1>())(0) / 100.0;
    (x.tail<1>())(0) = s;
    printf("refine estimated scale: %f", s);
//    cout << " refine     " << g.norm() << " " << g.transpose() << endl;
    if(s > 0.0 )
    {
        printf("initial succ!\n");
        printf("测试 s的尺度到底应该是多少：%lf\n",s);
//        cout<<"测试 s的尺度到底应该是多少："<<s<<endl;
    }
    else
    {
        printf("initial fail\n");
        return false;
    }
    return true;
}
//计算陀螺仪偏置bg 尺度s 重力加速度g 速度v
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &g, Eigen::VectorXd &x)
{
//    得到陀螺仪偏置之后将其值保存到前面定义的Bgs[]中，最后在重新计算一次预积分。
    solveGyroscopeBias(all_image_frame, Bgs);
    
    //尺度 计算完成，重力的计算完成
    if(SolveScale(all_image_frame, g, x))
        return true;
    else 
        return false;
}



//思考 舒尔补 那些东西弄哪里去了
bool InertialOptimization(map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &g, Eigen::VectorXd &x){
    
    //牢记：只是要确定bias,scale,重力方向（重力方向和最终的世界坐标系z轴对齐，最终要求得视觉坐标系到世界坐标系的转换关系）
    //提供的测量值：gyro角速度，acc加速度
    
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 3 + 1;//要优化的维度v： 3*帧的数量， g:3,   scale_x:1,    bias_gyro:3
    x.resize(n_state);
    x.setZero();
    
    ceres::Problem problem;
    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::DENSE_SCHUR;
//    options.trust_region_strategy_type = ceres::DOGLEG;
//    options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 50;
    ceres::Solver::Summary summary;
    ceres::LossFunction *loss_function;
//    loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    
    //待改 这里应该是第一帧的加速度偏差
//    double acc_b[3];
    double gyro_b[3]={0,0,0},scale_x=2.3,g_c0[3]={0,-9.8,0};//优化的变量
    
    //acc bias因为没有任何先验，所以初始化时，假设bias为0
//    Eigen::Vector3d acc_bias_prior(0,0,0);
//    ceres::CostFunction* acc_cost_function = WeightAccFactor::Create( acc_bias_prior[0], acc_bias_prior[1], acc_bias_prior[2]);
//    problem.AddResidualBlock(acc_cost_function, NULL, acc_b);
    //gyro bias
    Eigen::Vector3d gyro_bias_prior(0,0,0);//期望值
    ceres::CostFunction* gyro_cost_function = WeightGyroFactor::Create( gyro_bias_prior[0], gyro_bias_prior[1], gyro_bias_prior[2]);
    problem.AddResidualBlock(gyro_cost_function, NULL, gyro_b);
    
    //G scale则是都统一放在一个问题里面
    //其实这个也应该给个先验，帮助更好的定位准确的G scale
    //这个g为C0坐标系下，先验设置为模长9.81
    //后续根据施密特正交化，要和（0，0，-1）正交的两组基，获得最终的三个正交基，最终定义这个为最终的世界坐标系
//    ceres::CostFunction* g_cost_function = WeightGFactor::Create(96.24);
//    problem.AddResidualBlock(g_cost_function, NULL,g_c0 );//期望值
    //scale先验为1，能不能变量是x exp(x)=scale
//    double scale_prior=100.0;//期望值 因为计算过程放大了
//    ceres::CostFunction* scale_cost_function = WeightScaleFactor::Create(scale_prior);
//    problem.AddResidualBlock(scale_cost_function, NULL, &scale_x);
    

    
    //TODO先遍历一遍把值放到数组里，因为不然旋转矩阵转欧拉角要重复转一次
    
    //因为帧的位姿这些都不优化，所以不需要添加变量，直接以参数的形式传递进去就好
//    只优化帧的速度（因为没有真正的尺度）
    //遍历所有的帧
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    Eigen::Vector3d t_imu_cam;
    t_imu_cam<<TIC_X, TIC_Y, TIC_Z;
    double v[all_frame_count][3];
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);
        
//        cout<<"测试是第几项："<<i<<endl;
        EdgeInertialGS *f = new EdgeInertialGS(frame_i->second, frame_j->second, t_imu_cam);
        problem.AddResidualBlock(f, loss_function, gyro_b, v[i], v[i+1], g_c0,&scale_x);
        
//        problem.AddResidualBlock(f, loss_function,gyro_b, v[i], v[i+1], g_c0,&scale_x,acc_b);
        
    }
    //开始求解
    ceres::Solve(options, &problem,  &summary);
    
    std::cout << "imu fuse: "<<summary.BriefReport() << "\n";
    
    
    
    double final_cost=summary.final_cost;
    g<<g_c0[0],g_c0[1],g_c0[2];
    
    scale_x=exp(scale_x);
    x[n_state-1]=scale_x;
    
    cout<<"测试重力的变化: "<<g.norm()<<", "<<g_c0[0]<<g_c0[1]<<g_c0[2]<<" , "<<scale_x <<", "<<x[n_state-1]<<endl;
    //final_cost>200 ||
    if( fabs(g.norm() - G_NORM) > G_THRESHOLD || scale_x<1e-3)
       {
        printf("imu imu final cost %lf faild!!!\n",final_cost);
        return false;
    }
    
    
    
    for(int i=0;i<all_frame_count;i++){
        x[i*3]=v[i][0];
        x[i*3+1]=v[i][1];
        x[i*3+2]=v[i][2];
        
    }
    x[n_state-4]=g_c0[0];
    x[n_state-3]=g_c0[1];
    x[n_state-2]=g_c0[2];
    
    
    
    
    Eigen::Vector3d bg;
    bg<<gyro_b[0],gyro_b[1],gyro_b[2];
    //更新Bgs[i]
    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += bg;
    //计算出陀螺仪偏差后再利用新的陀螺仪偏差进行预积分求解
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Eigen::Vector3d::Zero(), Bgs[0]);
    }
    
  
    
    RefineGravity(all_image_frame, g, x);
   
    
    scale_x=(x.tail<1>())(0)/100.0;
    (x.tail<1>())(0)=scale_x;
    if(scale_x > 0.0 )
    {
        printf("initial succ!\n");
        printf("测试 s的尺度到底应该是多少：%lf\n",scale_x);
//        cout<<"测试 s的尺度到底应该是多少："<<scale_x<<endl;
    }
    else
    {
        printf("initial fail， %lf\n",scale_x);
        return false;
    }
    return true;
}


bool VisualOptiIMU(map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &g, Eigen::VectorXd &x){
    
    solveGyroscopeBias(all_image_frame, Bgs);

    //尺度 计算完成，重力的计算完成
    if(SolveScale(all_image_frame, g, x))
        return true;
    else
        return false;
    
    
//    cout<<"开始纯惯性的初始化 参数估计"<<endl;
//    //尺度 计算完成，重力的计算完成
//    if(InertialOptimization(all_image_frame,Bgs, g, x))
//        return true;
//    else
//        return false;
}
