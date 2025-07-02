//
//  OptimizeRelativePose.cpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2021/6/30.
//  Copyright © 2021 zx. All rights reserved.
//

#include "OptimizeRelativePose.hpp"
#include "PoseGraph.hpp"

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

bool optiRelativePose_forMainMap(std::vector<cv::Point2f> measurements_old_norm_real, std::vector<Vector3d> point_3d_cur_real, KeyFrame* old_kf, KeyFrame* cur_kf, Matrix3d &R_relative, Vector3d &T_relative){
    
    
    Client* client_cur=old_kf->c;
    Matrix3d ric_curClient=client_cur->ric_client;
    Vector3d tic_curClient=client_cur->tic_client;
    Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
//    const float &fx = client_cur->FOCUS_LENGTH_X_server;
//    const float &fy = client_cur->FOCUS_LENGTH_Y_server;
//    const float &cx = client_cur->PX_server;
//    const float &cy = client_cur->PY_server;

    
    Matrix3d oldKF_r;
    Vector3d oldKF_t;
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
        Vector3d euler_angle_old = Utility::R2ypr(oldKF_r);
        euler_array[0] = euler_angle_old.x();
        euler_array[1] = euler_angle_old.y();
        euler_array[2] = euler_angle_old.z();
        problem.AddParameterBlock(euler_array, 3);
        problem.AddParameterBlock(t_array, 3);

        for(int a=0,b=point_3d_cur_real.size();a<b;a++){
            //找到主地图那个点 所在帧的位姿
            Vector3d pts_i = point_3d_cur_real[a];

            //相机平面坐标
            cv::Point2f pt=measurements_old_norm_real[a];
            float xx=pt.x;
            float yy=pt.y;

             Vector2d pts_j ;//要求是归一化图像坐标
             pts_j<<xx,yy;

            ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_server::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),100.0);
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

        R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));
    }
    return true;
}


bool optiRelativePose_forMainMap2(std::vector<cv::Point2f> measurements_old_norm_real, std::vector<Vector3d> point_3d_cur_real, KeyFrame* old_kf, KeyFrame* cur_kf, Matrix3d &R_relative, Vector3d &T_relative,std::vector<cv::Point2f> measurements_old){
    std::vector<cv::Point2f> measurements_cur=cur_kf->measurements;
    
    Client* client_cur=old_kf->c;
    Matrix3d ric_curClient=client_cur->ric_client;
    Vector3d tic_curClient=client_cur->tic_client;
    Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
//    const float &fx = client_cur->FOCUS_LENGTH_X_server;
//    const float &fy = client_cur->FOCUS_LENGTH_Y_server;
//    const float &cx = client_cur->PX_server;
//    const float &cy = client_cur->PY_server;

//    cout<<cur_kf->global_index<<" , "<<old_kf->global_index<<endl;
    
    Matrix3d oldKF_r;
    Vector3d oldKF_t;
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
        
        std::vector<ceres::ResidualBlockId> residual_block_ids_all;

        double t_array[3];//平移数组，其中存放每个关键帧的平移向量
        double euler_array[3];
        t_array[0] = oldKF_t(0);
        t_array[1] = oldKF_t(1);
        t_array[2] = oldKF_t(2);
        //将矩阵转换为向量
        Vector3d euler_angle_old = Utility::R2ypr(oldKF_r);
        euler_array[0] = euler_angle_old.x();
        euler_array[1] = euler_angle_old.y();
        euler_array[2] = euler_angle_old.z();
        problem.AddParameterBlock(euler_array, 3);
        problem.AddParameterBlock(t_array, 3);
        std::map<ceres::ResidualBlockId ,int> residual_para;
        std::map<ceres::ResidualBlockId ,bool> residual_exit;//true为存在
        vector<ceres::CostFunction* > cost_function_all;
        
        

        for(int a=0,b=point_3d_cur_real.size();a<b;a++){
            //找到主地图那个点 所在帧的位姿
            Vector3d pts_i = point_3d_cur_real[a];

            //相机平面坐标
            cv::Point2f pt=measurements_old_norm_real[a];
            float xx=pt.x;
            float yy=pt.y;

             Vector2d pts_j ;//要求是归一化图像坐标
             pts_j<<xx,yy;

            ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_server::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),100.0);
            block_id =  problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
            residual_block_ids.push_back( block_id );
//            residual_para.insert(std::make_pair(block_id, a));
//            residual_exit.insert(std::make_pair(block_id,true));
//
//            cost_function_all.push_back(cost_function);
         }
//        residual_block_ids_all=residual_block_ids;
        
        std::vector< ceres::ResidualBlockId > residual_block_ids_temp;
        residual_block_ids_temp.reserve( residual_block_ids.size() );
        
//        const float chi2Mono[4]={5.991,5.991,5.991,5.991};
        const int its[4]={2,6,10,10};
        const double rate[4]={0.8,0.8,0.8,0.8};
        const double out_max[4]={2,2,2,2};
        for ( size_t ii = 0; ii < 4; ii++ )
        {
            options.max_num_iterations = its[ii];//2


//                    set_ceres_solver_bound( problem, t_array );
            ceres::Solve( options, &problem, &summary );
//                    std::cout <<"算一个粗糙的相对位姿1："<< summary.BriefReport() << "\n";
//            cout<<"problem.NumResidualBlocks()="<< problem.NumResidualBlocks()<<endl;
            
//            double **para_all=new double*[2];
//            for(int i=0;i<2;i++){
//                para_all[i]=new double[3];
//            }
//            para_all[0][0]=euler_array[0];
//            para_all[0][1]=euler_array[1];
//            para_all[0][2]=euler_array[2];
//            para_all[1][0]=t_array[0];
//            para_all[1][1]=t_array[1];
//            para_all[1][2]=t_array[2];
//            cout<<"para_all="<<para_all[0][0]<<" ，"<< para_all[1][0]<<" ，"<< para_all[1][1]<<" ，"<< para_all[1][2]<<endl;
            
            residual_block_ids_temp.clear();
            ceres::Problem::EvaluateOptions eval_options;
            eval_options.residual_blocks = residual_block_ids;
            double              total_cost = 0.0;
            std::vector<double> residuals;
            
//            double const* const* parameters
            double* residuals_costFunction;
            vector<double> residuals_all;
//            for(int a=0,b=cost_function_all.size();a<b;a++){
////                residuals_costFunction.clear();
//                ceres::CostFunction* cost_function =cost_function_all[a];
////                cost_function->Evaluate(para_all,residuals_costFunction, nullptr );
////                residuals_all.push_back(residuals_costFunction[0]);
////                residuals_all.push_back(residuals_costFunction[1]);
//
//                residuals_all.push_back(0);
//                residuals_all.push_back(0);
//
////                cout<<"自己算的误差"<<residuals_costFunction[0]<<" , "<<residuals_costFunction[1]<<endl;
//            }
            
//            cout<<"residual_block_ids="<<residual_block_ids.size()<< endl;
//            for(int a=0,b=residual_block_ids.size();a<b;a++){
////                residuals_costFunction.clear();
//                int index=residual_para[residual_block_ids[a]];
//                double** hhh;
//                ceres::CostFunction* cost_function =cost_function_all[index];
//                cost_function->Evaluate(para_all,residuals_costFunction, hhh );
//                residuals_all.push_back(residuals_costFunction[0]);
//                residuals_all.push_back(residuals_costFunction[1]);
//
////                residuals_all.push_back(0);
////                residuals_all.push_back(0);
//
//                cout<<"自己算的误差"<<residuals_costFunction[0]<<" , "<<residuals_costFunction[1]<<endl;
//            }
            
            problem.Evaluate( eval_options, &total_cost, &residuals, nullptr, nullptr );
//            cout<<"自动误差"<<residuals[0]<<" , "<<residuals[1]<<endl;

            double m_inliner_ratio_threshold = compute_inlier_residual_threshold( residuals, rate[ii] );
            double m_inlier_threshold = std::max( out_max[ii], m_inliner_ratio_threshold );
//                    cout<<"m_inlier_threshold="<<m_inlier_threshold<<endl;
            //screen_out << "Inlier threshold is: " << m_inlier_final_threshold << endl;
            
//            for ( unsigned int i = 0; i < residual_block_ids.size(); i++ )
//            {
////                        cout<<"测试误差："<<fabs( residuals[ 2 * i + 0 ] ) + fabs( residuals[ 2 * i + 1 ] )<<endl;
//                if ( ( fabs( residuals[ 2 * i + 0 ] ) + fabs( residuals[ 2 * i + 1 ] ) ) > m_inlier_threshold ) // std::min( 1.0, 10 * avr_cost )
//                {
////                    判断之前是否已经移除过了
//                    if(residual_exit[residual_block_ids[i]]){
//                        problem.RemoveResidualBlock( residual_block_ids[ i ] );
//                        residual_exit[residual_block_ids[i]]=false;
//                    }
//
//                }
//                else
//                {
////                    判断之前被移除过 但是此次要重新加回去
//                    if(residual_exit[residual_block_ids[i]]==false){
////                        Vector3d pts_i = point_3d_cur_real[i];
////
////                        //相机平面坐标
////                        cv::Point2f pt=measurements_old_norm_real[i];
////                        float xx=pt.x;
////                        float yy=pt.y;
////
////                         Vector2d pts_j ;//要求是归一化图像坐标
////                         pts_j<<xx,yy;
////                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_server::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),100.0);
////                        block_id =  problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
////                        residual_block_ids_all[i]=block_id;
////                        residual_exit.insert(std::make_pair(block_id, true));
//
//                    }else{
//                        residual_block_ids_temp.push_back( residual_block_ids[ i ] );
//                    }
//
//
//
//                }
//            }
            
            
            
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
            
//            delete [](para_all[0]);
//            delete [](para_all[1]);
//            delete [](para_all);
           
          
            
            
//        {
//             std::ofstream outFile;
//                outFile.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/pt/"+
//                                       to_string(cur_kf->global_index)+"&"+to_string(cur_kf->header)+"&"+
//                                       to_string(old_kf->global_index)+"&"+to_string(old_kf->header)+"&"+to_string(ii)+"_allPoint.txt");
//                outFile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
//                outFile.precision(19);
//                outFile << measurements_old.size()<<"\n";
//                for(int i=0,j=measurements_old.size(); i<j; i++)
//             {
//                 cv::Point2f pc=measurements_cur[i];
//                 cv::Point2f po=measurements_old[i];
//                 //写入数据
//                 outFile << pc.x<<" "<<pc.y<<" "<<po.x<<" "<<po.y<<"\n";
//             }
//
//             //关闭文件
//             outFile.close();
//        }
                
           
            
            if(problem.NumResidualBlocks()<10){
                break;
            }
            
            
        }
        
//        options.linear_solver_type = ceres::DENSE_SCHUR;
//        options.max_num_iterations = 20;
//
//        ceres::Solve( options, &problem, &summary );
//        std::cout <<"全局优化 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";
        
        if(summary.termination_type!=ceres::
           CONVERGENCE){
            return false;
        }

        T_relative[0]=t_array[0];
        T_relative[1]=t_array[1];
        T_relative[2]=t_array[2];

        R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));
        
    }
    
    
    return true;
}
