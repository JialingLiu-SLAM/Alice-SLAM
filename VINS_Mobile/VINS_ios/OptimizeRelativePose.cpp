//
//  OptimizeRelativePose.cpp
//  VINS_ios
//
//  Created by 张剑华 on 2022/7/25.
//  Copyright © 2022 栗大人. All rights reserved.
//

#include "OptimizeRelativePose.hpp"
#include "keyfame_database.h"
double compute_inlier_residual_threshold( std::vector< double > residuals, double ratio )
{
    std::set< double > dis_vec;
    for ( size_t i = 0; i < ( size_t )( residuals.size() / 2 ); i++ )
    {
        dis_vec.insert( fabs( residuals[ 2 * i + 0 ] ) + fabs( residuals[ 2 * i + 1 ] )  );
//        cout<<"测试误差是不是事先拍序的："<<fabs( residuals[ 2 * i + 0 ] ) + fabs( residuals[ 2 * i + 1 ] ) <<endl;
    }
//    cout<<"最终0.8的误差："<<*( std::next( dis_vec.begin(), ( int ) ( ( ratio ) * dis_vec.size() ) ) )<<endl;
    //下面这个会自动从小到大排序 找出第0.8个
    return *( std::next( dis_vec.begin(), ( int ) ( ( ratio ) * dis_vec.size() ) ) );
}

bool optiRelativePose_forMainMap(std::vector<cv::Point2f> measurements_old_norm_real, std::vector<Eigen::Vector3d> point_3d_cur_real, KeyFrame* old_kf, KeyFrame* cur_kf, Eigen::Matrix3d &R_relative, Eigen::Vector3d &T_relative){
    
    
    Eigen::Matrix3d ric_curClient;
    Eigen::Vector3d tic_curClient;
    tic_curClient<<TIC_X,
    TIC_Y,
    TIC_Z;
    ric_curClient = Utility::ypr2R(Eigen::Vector3d(RIC_y,RIC_p,RIC_r));
    Eigen::Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
    const float &fx = FOCUS_LENGTH_X;
    const float &fy = FOCUS_LENGTH_Y;
    const float &cx = PX;
    const float &cy = PY;
    
    
    Eigen::Matrix3d oldKF_r;
    Eigen::Vector3d oldKF_t;
    cur_kf->getOriginPose(oldKF_t, oldKF_r);
    {
        
        ceres::Problem problem;
        ceres::Solver::Options options;
        //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::DOGLEG;
        ceres::Solver::Summary summary;
        ceres::LossFunction *loss_function;
        loss_function = new ceres::HuberLoss(1.0);
        
        std::vector<ceres::ResidualBlockId> residual_block_ids;
        ceres::ResidualBlockId              block_id;

        double t_array[3];//平移数组，其中存放每个关键帧的平移向量
        double euler_array[3];
        t_array[0] = oldKF_t(0);
        t_array[1] = oldKF_t(1);
        t_array[2] = oldKF_t(2);
        //将矩阵转换为向量
        Eigen::Vector3d euler_angle_old = Utility::R2ypr(oldKF_r);
        euler_array[0] = euler_angle_old.x();
        euler_array[1] = euler_angle_old.y();
        euler_array[2] = euler_angle_old.z();
        problem.AddParameterBlock(euler_array, 3);
        problem.AddParameterBlock(t_array, 3);

        for(int a=0,b=point_3d_cur_real.size();a<b;a++){
            //找到主地图那个点 所在帧的位姿
            Eigen::Vector3d pts_i = point_3d_cur_real[a];

            //相机平面坐标
            cv::Point2f pt=measurements_old_norm_real[a];
            float xx=pt.x;
            float yy=pt.y;

             Eigen::Vector2d pts_j ;//要求是归一化图像坐标
             pts_j<<xx,yy;

            ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_server::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z());
            block_id =  problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
            residual_block_ids.push_back( block_id );
         }

        
        std::vector< ceres::ResidualBlockId > residual_block_ids_temp;
        residual_block_ids_temp.reserve( residual_block_ids.size() );
        
        for ( size_t ii = 0; ii < 1; ii++ )
        {
            options.max_num_iterations = 2;


//                    set_ceres_solver_bound( problem, t_array );
            ceres::Solve( options, &problem, &summary );
//                    std::cout <<"算一个粗糙的相对位姿1："<< summary.BriefReport() << "\n";

            residual_block_ids_temp.clear();
            ceres::Problem::EvaluateOptions eval_options;
            eval_options.residual_blocks = residual_block_ids;
            double              total_cost = 0.0;
            std::vector<double> residuals;
            problem.Evaluate( eval_options, &total_cost, &residuals, nullptr, nullptr );

            double m_inliner_ratio_threshold = compute_inlier_residual_threshold( residuals, 0.80  );
            double m_inlier_threshold = std::max( 2.0, m_inliner_ratio_threshold );
//                    cout<<"m_inlier_threshold="<<m_inlier_threshold<<endl;
            //screen_out << "Inlier threshold is: " << m_inlier_final_threshold << endl;
            for ( unsigned int i = 0; i < residual_block_ids.size(); i++ )
            {
//                        cout<<"测试误差："<<fabs( residuals[ 2 * i + 0 ] ) + fabs( residuals[ 2 * i + 1 ] )<<endl;
                if ( ( fabs( residuals[ 2 * i + 0 ] ) + fabs( residuals[ 2 * i + 1 ] ) ) > m_inlier_threshold ) // std::min( 1.0, 10 * avr_cost )
                {
                    //screen_out << "Remove outliers, drop id = " << (void *)residual_block_ids[ i ] <<endl;
                    problem.RemoveResidualBlock( residual_block_ids[ i ] );
                }
                else
                {
                    residual_block_ids_temp.push_back( residual_block_ids[ i ] );
                }
            }
            residual_block_ids = residual_block_ids_temp;
        }
        
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.max_num_iterations = 20;

        ceres::Solve( options, &problem, &summary );
        std::cout <<"全局优化 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";
        

        if(summary.termination_type!=ceres::
           CONVERGENCE){
            return false;
        }

        T_relative[0]=t_array[0];
        T_relative[1]=t_array[1];
        T_relative[2]=t_array[2];

        R_relative=Utility::ypr2R(Eigen::Vector3d(euler_array[0],euler_array[1],euler_array[2]));
    }
    return true;
}
