//
//  LoopClosure.cpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/27.
//  Copyright © 2020 zx. All rights reserved.
//

#include "LoopClosure.hpp"
#include <time.h>
#include "gc_ransac.h"

LoopClosure::LoopClosure(const char *_voc_file, int _image_w, int _image_h)
:demo(_voc_file,_image_w, _image_h), IMAGE_W(_image_w), IMAGE_H(_image_h)
{
    old_index = -1;
    loop_old_index = -1;
//    last_kf=-1;
//    real_kf_inTree=0;
    treeId_kf.clear();
    kfNum_tree=0;
    preLoopKfId=-1;
    printf("loop vocfile %s\n",_voc_file);
    printf(" loop closure init finish\n");
}


bool LoopClosure::startLoopClosure(std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors,
                                   std::vector<cv::Point2f> &cur_pts,
                                   std::vector<cv::Point2f> &old_pts,
                                   int &old_index)
{
  try
  {
    bool loop_succ = false;
    loop_succ = demo.run("BRIEF", keys, descriptors, cur_pts, old_pts, old_index);
    return loop_succ;
  }
  catch(const std::string &ex)
  {
    cout << "Error loop: " << ex << endl;
    return false;
  }
}

bool LoopClosure::startLoopClosure_2(std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors,
                                   std::vector<cv::Point2f> &cur_pts,
                                   std::vector<cv::Point2f> &old_pts,
                                   int &old_index,BowVector &m_bowvec, FeatureVector &m_featvec)
{
  try
  {
    bool loop_succ = false;
    loop_succ = demo.run_2("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, m_bowvec, m_featvec);
    return loop_succ;
  }
  catch(const std::string &ex)
  {
    cout << "Error loop: " << ex << endl;
    return false;
  }
}

//这里是两帧之间算出相对位姿，然后根据这个位姿，在当前帧的共视帧和 匹配帧的共视帧之间投影找点，算出相对位姿
//投影找到的点 少，并且可能不太好 因为不能收敛
bool LoopClosure::startLoopClosure_3(KeyFrame* cur_kf, std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors,
                                   std::vector<cv::Point2f> &cur_pts,
                                   std::vector<cv::Point2f> &old_pts,
                                     KeyFrame* *old_kf,BowVector &m_bowvec, FeatureVector &m_featvec)
{
  try
  {
    bool loop_succ = false;
      //可以先只做一个简单的判断，不要检测时间一致性
      //这里返回的是分数超过阈值的
      int best_id=demo.run_3("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, m_bowvec, m_featvec);
      //检测成功后，检测共视帧的一致性
      //选出分数最高的3帧作为侯选帧，并计算一级共视帧的分数（最多选5个），求和，根据这个找到匹配性最高的侯选帧
    if(best_id!=-1){
        //找到分数最高的3个侯选帧 验证一下 是不是从高到低排序 分数 QueryResults是从高到低, tIsland不是
        //可能存在一个缺陷，最适合的不在前面3个候选里面

        
        *old_kf= poseGraph->getKeyframe(best_id);
        
        cout<<"测试 可能的匹配："<<(*old_kf)->global_index<<" "<<cur_kf->global_index<<endl;
            
        
 
        
        //这个是不是可以用两帧 计算匹配点，计算基础矩阵 最终的特征点数量判断
        std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
        std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
        std::vector<Vector3d> point_3d_cur;
        cur_kf->findConnectionWithOldFrame_server_old(*old_kf, measurements_old_coarse, measurements_old_norm_coarse);
        point_3d_cur=cur_kf->point_clouds;
        std::vector<int> features_id=cur_kf->features_id;
        //这里最低设成15
        if(measurements_old_coarse.size()<20){
            cout<<"几何匹配失败，点数不够20"<<endl;
            return false;
        }
        Matrix3f R_relative;
        Vector3f T_relative;
        vector<cv::Point3f> pts_3_vector;
        bool is_fusion=cur_kf->solveRelativePoseByPnP_2(measurements_old_norm_coarse ,R_relative, T_relative,pts_3_vector,false);
        
        if(is_fusion){
            //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
            
        
            //找5帧，加上自己，总共6帧，有3帧以上就行
            std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
            //记录当前帧的共视帧 能够和候选帧的共视帧匹配上的数量
            int similarNum=0;
            vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
            //如果这里报错，说明没有共视帧，那么记得加上共视帧阈值设大了的，补丁 加了加了
            vpCovKFi_cur[0] = cur_kf;
            
            //这里检索当前帧的共视帧 与 侯选帧的匹配关系
            vector<vector<cv::Point2f>> measurements_old_norm_all;
            vector<vector<Eigen::Vector3d>> point_clouds_all;
            vector<KeyFrame*> real_vpCovKFi_cur;//最终匹配点数符合要求的
            //这个最终用来固定的 优化中
            int min_global_index=cur_kf->global_index,min_id_index=0;
            
                
            vector<KeyFrame*> vpCovKFi_old=(*old_kf)->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_old.push_back(vpCovKFi_old[0]);
            vpCovKFi_old[0]=(*old_kf);
            
            vector<Matrix3f> old_r;
            vector<Vector3f> old_t;
            //这个存储候选帧的共视帧 之间的位姿关系, 是从候选帧到共视帧的位姿
            vector<Matrix3d> oldR_b_a;
            vector<Vector3d> oldT_b_a;
            
            Matrix3d oldKF_r;
            Vector3d oldKF_t;
            (*old_kf)->getOriginPose(oldKF_t, oldKF_r);
            int temp_index=0;
            //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
            for(KeyFrame* pkFi_old: vpCovKFi_old){
                if(temp_index!=0){
                    Matrix3d rwi_old;
                    Vector3d twi_old;
                    
                    pkFi_old->getOriginPose(twi_old, rwi_old);
                    
                    Matrix3d r_b_a;
                    Vector3d t_b_a;
                    r_b_a=rwi_old.transpose()*oldKF_r;
                    t_b_a=rwi_old.transpose()*(oldKF_t-twi_old);
                    oldR_b_a.push_back(r_b_a);
                    oldT_b_a.push_back(t_b_a);
                    
                    
                    old_r.push_back(r_b_a.cast<float>()*R_relative);
                    old_t.push_back(r_b_a.cast<float>()*T_relative+t_b_a.cast<float>());
                    
                }else{
                    old_r.push_back(R_relative);
                    old_t.push_back(T_relative);
                    
                    oldR_b_a.push_back(Matrix3d::Identity());
                    oldT_b_a.push_back(Vector3d::Zero());
                }
                
                temp_index++;
            }
            
            Client* client_cur=(*old_kf)->c;
            
            Matrix3d ric_curClient=client_cur->ric_client;
            Vector3d tic_curClient=client_cur->tic_client;
            
            const float &fx = client_cur->FOCUS_LENGTH_X_server;
            const float &fy = client_cur->FOCUS_LENGTH_Y_server;
            const float &cx = client_cur->PX_server;
            const float &cy = client_cur->PY_server;
            
            
            vector<cv::Point2f> measurements_old_2d;
            vector<Eigen::Vector3d> pointsCloud_cur_3d;
            vector<cv::Point2f> measurements_cur_2d;
            
            vector<vector<Eigen::Vector3d>> pointsCloud_cur_3d_all;
            vector<vector<cv::Point2f>> measurements_old_2d_all;//最终要存 转为图像的
            vector<KeyFrame* > old_kf_all;//存储所有匹配上kf
            
            
            for(KeyFrame* pKFi : vpCovKFi_cur){
                //因为第一帧是当前帧 所以跳过
                if(pKFi==cur_kf){
                    pointsCloud_cur_3d_all.push_back(point_3d_cur);
                    measurements_old_2d_all.push_back(measurements_old_norm_coarse);
                    old_kf_all.push_back(*old_kf);
                    continue;
                }
                //描述符得全有
                if(!pKFi->is_des_end){
                    continue;
                }
                
                vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
//                vector<int> feature_id_origin_cur=pKFi->features_id_origin;
                vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                int nPoints = point_clouds_origin_cur.size();
                int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                
                measurements_old_2d.clear();
                pointsCloud_cur_3d.clear();
                measurements_cur_2d.clear();
                
                int min_global_index=cur_kf->global_index;//最小的id，这个要固定
                int num=0;//记录匹配点的数量
                int temp_kf_index=0;//记录遍历到哪个帧了
                //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                for(KeyFrame* pkFi_old: vpCovKFi_old){
                    //描述符得全有
                    if(!pkFi_old->is_des_end){
                        temp_kf_index++;
                        continue;
                    }

                    vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                    Matrix3f r_camOld_w=old_r[temp_kf_index];
                    Vector3f t_camOld_w=old_t[temp_kf_index];
                    
                    
                    for(int i=0;i<nPoints;i++){
                        Vector3d point_main=point_clouds_origin_cur[i];
                        //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                        Vector3f p3D_c2=r_camOld_w*point_main.cast<float>()+t_camOld_w;
                        //深度必须为正
                        if(p3D_c2[2]<0.0f || p3D_c2[2]>30){
        //                    cout<<"深度不为正"<<endl;
                            continue;
                        }
                        // 投影到图像上
                        float x = p3D_c2[0];
                        float y = p3D_c2[1];
                        float z = p3D_c2[2];
                        //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                        const float u=fx*x/z+cx;
                        const float v=fy*y/z+cy;
                        
                        
                        if(!pkFi_old->isInImage(u, v)){
        //                    cout<<"投影点不在图像内"<<endl;
                            continue;
                        }
                        
                        const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1(u, v, 50);
                        //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                        if(vIndices.empty()){
        //                    cout<<"半径为50个像素 找不到点"<<endl;
                            continue;
                        }
                        //des和keypoints长度不一样
                        //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                        BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                        int bestDist = 256;
                        int bestIndex = -1;
                        for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                        {
        //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                            int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                            if(dis < bestDist)
                            {
                                bestDist = dis;
                                bestIndex = *vit;
                            }
                        }
                        
                        if(bestDist<=50){
                            pointsCloud_cur_3d.push_back(point_main);
                            measurements_old_2d.push_back(keypoints_old[bestIndex].pt);
                            measurements_cur_2d.push_back(measurements_origin_cur[i]);
                            
                        }
                                      
                    }
                    
                    
//                    cout<<"地图内部 50个像素内,找到的点数："<<pointsCloud_cur_3d.size()<<endl;
                    if(measurements_cur_2d.size()>=15){
                        pKFi->rejectWithF_server_mapFuse(measurements_cur_2d, measurements_old_2d,pointsCloud_cur_3d);
//                        cout<<"地图内部 f矩阵拒绝后的点数："<<measurements_cur_2d.size()<<endl;
                        if(measurements_old_2d.size()>=15){
                        
                            cv::Point2f norm_pt;
                            int pCount=measurements_old_2d.size();
                            for(int aa=0;aa<pCount;aa++){
                                norm_pt.x = (measurements_old_2d[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                norm_pt.y = (measurements_old_2d[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                measurements_old_2d[aa]=norm_pt;
                                
                            }
                            
                            pointsCloud_cur_3d_all.push_back(pointsCloud_cur_3d);
                            measurements_old_2d_all.push_back(measurements_old_2d);
                            num+=pointsCloud_cur_3d.size();
                            
                            if(min_global_index>pKFi->global_index){
                                min_global_index=pKFi->global_index;
//                                min_id_index=similarNum;
                            }
                            
                            old_kf_all.push_back(pkFi_old);
                            similarNum++;
                            break;
                        }
                    }
                    
                    temp_kf_index++;
                }
                
                
            }
            
           
            Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
            
            if(similarNum>=2){
                //构造优化问题
                double t_array[similarNum][3];//平移数组，其中存放每个关键帧的平移向量
                Quaterniond q_array[similarNum];
                double euler_array[similarNum][3];
                
                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                //options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 50;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
                //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
                ceres::LocalParameterization* angle_local_parameterization =
                AngleLocalParameterization::Create();
                
                int i=0;
                //这里应该可以只一个优化项，在当前帧位姿误差下，世界坐标系到老帧的位姿
                //把第一项给老帧 匹配帧
                Quaterniond tmp_q_old;
                Matrix3d tmp_r_old;
                Vector3d tmp_t_old;
                tmp_q_old = R_relative.cast<double>();
                t_array[i][0] = (T_relative.cast<double>())(0);
                t_array[i][1] = (T_relative.cast<double>())(1);
                t_array[i][2] = (T_relative.cast<double>())(2);
//                q_array[i] = tmp_q_old;
                //将矩阵转换为向量
                Vector3d euler_angle_old = Utility::R2ypr(tmp_q_old.toRotationMatrix());
                euler_array[i][0] = euler_angle_old.x();
                euler_array[i][1] = euler_angle_old.y();
                euler_array[i][2] = euler_angle_old.z();
                problem.AddParameterBlock(euler_array[i], 3);
                problem.AddParameterBlock(t_array[i], 3);
                
                i++;
                
                int old_kf_index=0;
                
                for(KeyFrame* pKFi : old_kf_all){
                    {
//                    Quaterniond tmp_q;
//                    Matrix3d tmp_r;
//                    Vector3d tmp_t;
//                    pKFi->getOriginPose(tmp_t, tmp_r);
//                    tmp_q = tmp_r;
//                    t_array[i][0] = tmp_t(0);
//                    t_array[i][1] = tmp_t(1);
//                    t_array[i][2] = tmp_t(2);
//                    q_array[i] = tmp_q;
//                    //将矩阵转换为向量
//                    Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
//                    euler_array[i][0] = euler_angle.x();
//                    euler_array[i][1] = euler_angle.y();
//                    euler_array[i][2] = euler_angle.z();
//                    problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
//                    problem.AddParameterBlock(t_array[i], 3);
//                    if((i-1)==min_id_index){
//                        problem.SetParameterBlockConstant(euler_array[i]);
//                        problem.SetParameterBlockConstant(t_array[i]);
//                    }
//
//                    //和前面的有一个约束关系
//                    int j=i;
//                    while(j-1>0){
//                        j--;
//                        Vector3d euler_conncected = Utility::R2ypr(q_array[j].toRotationMatrix());
//                        //p̂_j^w - p̂_i^w 计算平移量的偏差
//                        Vector3d relative_t(t_array[i][0] - t_array[j][0], t_array[i][1] - t_array[j][1], t_array[i][2] - t_array[j][2]);
//                        relative_t = q_array[j].inverse() * relative_t;
//
//                        double relative_yaw = euler_array[i][0] - euler_array[j][0];
//                        ceres::CostFunction* cost_function = FourDOFError_posegraph::Create( relative_t.x(), relative_t.y(), relative_t.z(),
//                            relative_yaw, euler_conncected.y(), euler_conncected.z());
//                        problem.AddResidualBlock(cost_function, loss_function, euler_array[j],
//                                                 t_array[j],
//                                                 euler_array[i],
//                                                 t_array[i]);
//                    }
                    }
                    Matrix3d relative_r_b_a;
                    Vector3d relative_t_b_a;
                    
                    
                    //找一下 对应的应该是哪个相对位姿
                    int relative_index=0;
                    for(KeyFrame* pkFi_old: vpCovKFi_old){
                        if(pkFi_old==pKFi){
                            relative_r_b_a=oldR_b_a[relative_index];
                            relative_t_b_a=oldT_b_a[relative_index];
                            break;
                        }
                        relative_index++;
                    }
                    Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
         
                    vector<Vector3d> point_single=pointsCloud_cur_3d_all[old_kf_index];
                    vector<cv::Point2f> measure_single=measurements_old_2d_all[old_kf_index];
                    for(int a=0,b=point_single.size();a<b;a++){
                        
                            //找到主地图那个点 所在帧的位姿
                            Vector3d pts_i = point_single[a];

                            //相机平面坐标
                            cv::Point2f pt=measure_single[a];
                            float xx=pt.x;
                            float yy=pt.y;

                             Vector2d pts_j ;//要求是归一化图像坐标
                             pts_j<<xx,yy;

                            ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                            problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0]);
                         }

                    
                    
                    old_kf_index++;
                    
                    i++;
                    
                }
                
                ceres::Solve(options, &problem, &summary);
                std::cout << summary.BriefReport() << "\n";
                
                if(summary.termination_type==ceres::
                   CONVERGENCE){
//                    cout<<"地图内部 检测到回环： old_index="<<(*old_kf)->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
//                    Vector3d q_cur;
//                    q_cur<<euler_array[1][0],euler_array[1][1],euler_array[1][2];
                    Matrix3d Rs_i ;
                    Vector3d Ps_i ;//当前帧
                    cur_kf->getOriginPose(Ps_i, Rs_i);
                    
//                    Vector3d q;
//                    q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
//                    Vector3d relative_t = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
//                    Matrix3d relative_q = Utility::ypr2R(q);
//                    double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
//                    cur_kf->updateLoopConnection(relative_t, relative_yaw);

                    Vector3d q;
                    q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                    Matrix3d Rs_loop = Utility::ypr2R(q);
                    Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);

                    Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
                    Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                    double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                    cur_kf->updateLoopConnection(relative_t, relative_yaw);
                    
                    //先临时搬过来
                    old_index=(*old_kf)->global_index;
                    Vector3d T_w_i_old;
                    Matrix3d R_w_i_old;
                    (*old_kf)->getPose(T_w_i_old, R_w_i_old);
                    
                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm_coarse;
                    retrive_data.features_ids = features_id;
                    retrive_data.old_index=(*old_kf)->global_index;
                    vins->retrive_pose_data = (retrive_data);
    //                        vins->isSendLoopData=true;
                    
                    cur_kf->detectLoop((*old_kf)->global_index);
                    poseGraph->addLoop((*old_kf)->global_index);//todo 这里面 改一下
                    (*old_kf)->is_looped = 1;
                    loop_old_index = (*old_kf)->global_index;
                    return true;
                    
                }else{
//                    cout<<"地图内部 检测失败： old_index="<<(*old_kf)->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
                    return false;
                }
            }else{
                cout<<"地图内部 匹配帧数不够多="<<similarNum<<endl;
            }
            
            
               
            }
            //这个侯选帧， 和当前帧 以及当前帧的共视帧做匹配
            
            //可能是bow分数 ，或者几何一致性匹配？？？ 后面考虑
            
        }
      
      return false;
            
      //当前帧是否能找到三个共视帧 和 匹配帧
      
      //几何一致性检测
      
//    loop_succ = demo.run_2("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, m_bowvec, m_featvec);
//    return loop_succ;
  }
  catch(const std::string &ex)
  {
    cout << "Error loop: " << ex << endl;
    return false;
  }
}


//相对于上面， 换成不再是只是用点云和keypoint做匹配，因为这样点太少
//用keypoint和keypoint匹配，测试点数量够不够
//直接使用描述符匹配 是不是匹配点数量可能会多一些，而不是重投影？
//计算方式 思考能不能换成 只通过两帧计算？
//检查一下候选怎么给的不一样了？因为并不是每帧都能参与到优化，所以那棵树里面 下标并不是就是对应的那帧
//当属于共视帧匹配数不够时，能不能通过前后帧计数 使其达到共视帧的要求（因为前后帧并不一定共视程度最高）
//似乎是计算的时间太长了？检查一下是不是每帧都参与检测？ 的确会有跳帧出现 ,但是直接用两帧算出来的，很粗糙 投影找不到匹配点
bool LoopClosure::startLoopClosure_4(KeyFrame* cur_kf)
{
  try
  {
    bool loop_succ = false;
      int old_index;
//      cout<<"测试是不是每一帧都参与回环检测？"<<cur_kf->global_index<<endl;
      
      
      //可以先只做一个简单的判断，不要检测时间一致性
      //这里返回的是分数超过阈值的
      vector<cv::Point2f> cur_pts;
      vector<cv::Point2f> old_pts;
      std::vector<cv::KeyPoint> keys=cur_kf->keypoints;
      std::vector<BRIEF::bitset> descriptors=cur_kf->descriptors;
      int best_id=demo.run_3("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, cur_kf->m_bowvec, cur_kf->m_featvec);
      
      treeId_kf[kfNum_tree]= cur_kf->global_index;
      kfNum_tree++;
      //描述符得全有
      if(!cur_kf->is_des_end){
          return false;
      }
      
//      int kf_num=cur_kf->global_index-last_kf;
//      if(kf_num!=1){
//          real_kf_inTree+=(kf_num-1);
//      }
//      last_kf=cur_kf->global_index;
      
      //检测成功后，检测共视帧的一致性
      //选出分数最高的3帧作为侯选帧，并计算一级共视帧的分数（最多选5个），求和，根据这个找到匹配性最高的侯选帧
    if(best_id!=-1){
        //这里直接加这么多也是有问题的 我想想
//        int real_old_best_id=best_id+real_kf_inTree;
        int real_old_best_id=treeId_kf[best_id];
        KeyFrame* old_kf;
        old_kf= poseGraph->getKeyframe(real_old_best_id);
        
        //当前帧与候选帧的匹配点数量还是应该满足几何一致性
//        bool isTemporal_cur_old= demo.isGeometricallyConsistent_DI(best_id, cur_kf->keypoints,cur_kf->descriptors,cur_kf->m_featvec,cur_pts,old_pts);
//        if(!isTemporal_cur_old){
//            return false;
//        }
        
        
        
        Matrix3d R_relative;
        Vector3d T_relative;

        
        Client* client_cur=old_kf->c;
        Matrix3d ric_curClient=client_cur->ric_client;
        Vector3d tic_curClient=client_cur->tic_client;
        Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
        const float &fx = client_cur->FOCUS_LENGTH_X_server;
        const float &fy = client_cur->FOCUS_LENGTH_Y_server;
        const float &cx = client_cur->PX_server;
        const float &cy = client_cur->PY_server;
        
        //这个是不是可以用两帧 计算匹配点，计算基础矩阵 最终的特征点数量判断
        std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
        std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
        std::vector<Vector3d> point_3d_cur;
        std::vector<int> features_id;
        std::vector<cv::Point2f> measurements_cur;//像素坐标
        
        vector<vector<cv::Point2f>> measurements_old_norm_all;
        vector<vector<Eigen::Vector3d>> point_clouds_all;
        vector<KeyFrame*> real_vpCovKFi_cur;//最终匹配点数符合要求的
        //存储一下当前帧和候选帧的匹配关系
        std::vector<cv::Point2f> measurements_old_norm_real;//图像坐标
        std::vector<Vector3d> point_3d_cur_real;
        
        //有没有可能上面算的相对位姿不够准，通过bow的方式，找当前帧到 侯选帧的共视帧的匹配关系
        //优化得到当前帧位姿误差下 到侯选帧的位姿关系
        vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
        vpCovKFi_old.push_back(vpCovKFi_old[0]);
        vpCovKFi_old[0]=old_kf;
        //得到当前帧位姿误差下 到侯选帧与其共视帧之间的相对位姿关系
        vector<Matrix3d> old_r;
        vector<Vector3d> old_t;
        //这个存储候选帧的共视帧 之间的位姿关系, 是从候选帧到共视帧的位姿
        vector<Matrix3d> oldR_b_a;
        vector<Vector3d> oldT_b_a;
        Matrix3d oldKF_r;
        Vector3d oldKF_t;
        old_kf->getOriginPose(oldKF_t, oldKF_r);
        int temp_index=0;
        //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
        for(KeyFrame* pkFi_old: vpCovKFi_old){
            if(temp_index!=0){
                Matrix3d rwi_old;
                Vector3d twi_old;
                pkFi_old->getOriginPose(twi_old, rwi_old);
                
                Matrix3d r_b_a;
                Vector3d t_b_a;
                r_b_a=oldKF_r.transpose()* rwi_old;
                t_b_a=oldKF_r.transpose()*(twi_old-oldKF_t);
                oldR_b_a.push_back(r_b_a);
                oldT_b_a.push_back(t_b_a);
            }else{
                oldR_b_a.push_back(Matrix3d::Identity());
                oldT_b_a.push_back(Vector3d::Zero());
            }
            temp_index++;
        }
        //得到当前帧 与 侯选帧和共视帧 之间匹配关系
//        int num=0;
        point_clouds_all.clear();
        measurements_old_norm_all.clear();
        real_vpCovKFi_cur.clear();
        measurements_old_norm_real.clear();
        point_3d_cur_real.clear();
        
        clock_t start,ends;
        
        for(KeyFrame* pkf_old: vpCovKFi_old){
            start=clock();
            measurements_old_coarse.clear();
            measurements_old_norm_coarse.clear();
            point_3d_cur.clear();
            cur_kf->findConnectionWithOldFrame_server_old(pkf_old, measurements_old_coarse, measurements_old_norm_coarse);
            point_3d_cur=cur_kf->point_clouds;
            if(pkf_old==old_kf){
                if(measurements_old_coarse.size()<22){
                    
                    cout<<"当前帧与侯选帧的匹配点数少于22，"<<measurements_old_coarse.size()<<endl;
                    return  false;
                }
                measurements_old_norm_real=measurements_old_norm_coarse;
                point_3d_cur_real=point_3d_cur;
            }

            if(measurements_old_coarse.size()<15){
                ends=clock();
                cout<<"几何匹配失败，点数不够15，是" <<measurements_old_coarse.size()<<" , 花费的时间："<<ends-start<<endl;
                continue;
            }

//            num++;

            point_clouds_all.push_back(point_3d_cur);
            measurements_old_norm_all.push_back(measurements_old_norm_coarse);
            real_vpCovKFi_cur.push_back(pkf_old);
            ends=clock();
            cout<<"当前帧与候选帧 bow几何匹配点数超过15，花费的时间"<<ends-start<<endl;

        }
        cout<<"测试 可能的匹配："<<setprecision(19)<<old_kf->global_index<<" "<<old_kf->header<<" "<<cur_kf->global_index<<" "<<cur_kf->header<<endl;
//        cout<<"粗糙位姿优化，用的帧数："<<num<<endl;
//        if(num<1)
//            return  false;
        
//        measurements_old_coarse.clear();
//        measurements_old_norm_coarse.clear();
//        cur_kf->findConnectionWithOldFrame_server_old(old_kf, measurements_old_coarse, measurements_old_norm_coarse);
//        if(measurements_old_coarse.size()<22){
//            cout<<"当前帧与侯选帧的匹配点数少于22，"<<measurements_old_coarse.size()<<endl;
//            return  false;
//        }
//        measurements_old_norm_real=measurements_old_norm_coarse;
//        point_3d_cur_real=cur_kf->point_clouds;
        cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false);
        //优化 得到当前帧位姿误差下 到侯选帧的位姿关系
        start=clock();
        {
            ceres::Problem problem;
            ceres::Solver::Options options;
            //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            //options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 20;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
//            loss_function = new ceres::CauchyLoss(1.0);

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

            int i=0;
            for(KeyFrame* pkf_old: real_vpCovKFi_cur){
                Matrix3d relative_r_b_a=oldR_b_a[i];
                Vector3d relative_t_b_a=oldT_b_a[i];
                Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                vector<Vector3d> point_single=point_clouds_all[i];
                vector<cv::Point2f> measure_single=measurements_old_norm_all[i];
                for(int a=0,b=point_single.size();a<b;a++){
                    //找到主地图那个点 所在帧的位姿
                    Vector3d pts_i = point_single[a];

                    //相机平面坐标
                    cv::Point2f pt=measure_single[a];
                    float xx=pt.x;
                    float yy=pt.y;

                     Vector2d pts_j ;//要求是归一化图像坐标
                     pts_j<<xx,yy;

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(), relative_t_b_a(0), relative_t_b_a(1), relative_t_b_a(2), relative_r_b_a_euler(0), relative_r_b_a_euler(1), relative_r_b_a_euler(2),100.0);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
                 }
                i++;
            }

            ceres::Solve(options, &problem, &summary);
//            std::cout <<"地图内部 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";

            if(summary.termination_type!=ceres::
               CONVERGENCE){
                return false;
            }

            T_relative[0]=t_array[0];
            T_relative[1]=t_array[1];
            T_relative[2]=t_array[2];

            R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));

//            cout<<"优化结束后的粗糙位姿："<<T_relative[0]<<", "<<T_relative[1]<<", "<<T_relative[2]<<", "<<euler_array[0]<<", "<<euler_array[1]<<", "<<euler_array[2]<<endl;
        }
        ends=clock();
        cout<<"计算粗糙位姿花费的时间:"<<ends-start<<endl;
        //记录当前帧的共视帧 能够和候选帧的共视帧匹配上的数量
        int similarNum=0;
        {
            
            int i=0;
            for(KeyFrame* pkf_old:vpCovKFi_old){
                Matrix3d r_w_imuOld=R_relative*oldR_b_a[i];
                Vector3d t_w_imuOld=R_relative*oldT_b_a[i] +T_relative;
                Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                Matrix3d r_camOld_w=r_w_camOld.transpose();
                old_r.push_back(r_camOld_w);
                old_t.push_back(-r_camOld_w*t_w_camOld);
                i++;
                
            }
            //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
            
            //找5帧，加上自己，总共6帧，有3帧以上就行
            std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
            //如果这里报错，说明没有共视帧，那么记得加上共视帧阈值设大了的，补丁 加了加了
            vpCovKFi_cur[0] = cur_kf;
            
            
            
            point_clouds_all.clear();
            measurements_old_norm_all.clear();
            real_vpCovKFi_cur.clear();
            for(KeyFrame* pKFi : vpCovKFi_cur){
                //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
                if(pKFi==cur_kf){
                    point_clouds_all.push_back(point_3d_cur_real);
                    measurements_old_norm_all.push_back(measurements_old_norm_real);
                    real_vpCovKFi_cur.push_back(old_kf);
                    
                    continue;
                }
                //描述符得全有
                if(!pKFi->is_des_end){
                    continue;
                }
                
                vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                int nPoints = point_clouds_origin_cur.size();
                int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                
                
                int num=0;//记录匹配点的数量
                int temp_kf_index=0;//记录遍历到哪个帧了
                //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                for(KeyFrame* pkFi_old: vpCovKFi_old){
                    start=clock();
                
                    //描述符得全有
                    if(!pkFi_old->is_des_end){
                        temp_kf_index++;
                        continue;
                    }
                    
                    measurements_old_coarse.clear();
                    measurements_old_norm_coarse.clear();
                    point_3d_cur.clear();
                    measurements_cur.clear();

                    vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                    //这个得到的是到imu坐标系的位姿
                    Matrix3d r_camOld_w=old_r[temp_kf_index];
                    Vector3d t_camOld_w=old_t[temp_kf_index];
                    
                    
                    
                    for(int i=0;i<nPoints;i++){
                        Vector3d point_main=point_clouds_origin_cur[i];
                        //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                        Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                        //深度必须为正
                        if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
//                            cout<<"深度不为正"<<endl;
                            continue;
                        }
                        // 投影到图像上
                        double x = p3D_c2[0];
                        double y = p3D_c2[1];
                        double z = p3D_c2[2];
                        //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                        double u=fx*x/z+cx;
                        double v=fy*y/z+cy;
                      
                        
                        if(!pkFi_old->isInImage((float)u, (float)v)){
//                            cout<<"投影点不在图像内"<<endl;
                            continue;
                        }
                        
                        const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 60);
                        //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                        if(vIndices.empty()){
//                            cout<<"半径为50个像素 找不到点"<<endl;
                            continue;
                        }
                        //des和keypoints长度不一样
                        //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                        BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                        int bestDist = 256;
                        int bestIndex = -1;
                        for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                        {
        //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                            int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                            if(dis < bestDist)
                            {
                                bestDist = dis;
                                bestIndex = *vit;
                            }
                        }
                        
                        if(bestDist<=60){
                            point_3d_cur.push_back(point_main);
                            measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                            measurements_cur.push_back(measurements_origin_cur[i]);
                            
                        }
                                      
                    }
                    
                    
//                    cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                    if(measurements_cur.size()>=15){
                        pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
                        cout<<"f矩阵拒绝后的点数："<<measurements_cur.size()<<", "<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<endl;
                        if(measurements_old_coarse.size()>=15){
                        
                            cv::Point2f norm_pt;
                            int pCount=measurements_old_coarse.size();
                            for(int aa=0;aa<pCount;aa++){
                                norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                measurements_old_coarse[aa]=norm_pt;
                                
                            }
                            
                            point_clouds_all.push_back(point_3d_cur);
                            measurements_old_norm_all.push_back(measurements_old_coarse);
                            num+=point_3d_cur.size();
                            real_vpCovKFi_cur.push_back(pkFi_old);
                            similarNum++;
                            break;
                        }
                        
                        
                    }
                    
                    temp_kf_index++;
                    ends=clock();
                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                }
                
                
            }
                
                
        }
            
           
            
            
        if(similarNum>=2){
            //构造优化问题
            double t_array[similarNum][3];//平移数组，其中存放每个关键帧的平移向量
            Quaterniond q_array[similarNum];
            double euler_array[similarNum][3];
            
            ceres::Problem problem;
            ceres::Solver::Options options;
            //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            //options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 20;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
            ceres::LocalParameterization* angle_local_parameterization =
            AngleLocalParameterization::Create();
            
            int i=0;
            //这里应该可以只一个优化项，在当前帧位姿误差下，世界坐标系到老帧的位姿
            //把第一项给老帧 匹配帧
           
            Matrix3d tmp_r_old;
            Vector3d tmp_t_old;
            t_array[i][0] = T_relative(0);
            t_array[i][1] = T_relative(1);
            t_array[i][2] = T_relative(2);
            //将矩阵转换为向量
            Vector3d euler_angle_old = Utility::R2ypr(R_relative);
            euler_array[i][0] = euler_angle_old.x();
            euler_array[i][1] = euler_angle_old.y();
            euler_array[i][2] = euler_angle_old.z();
            problem.AddParameterBlock(euler_array[i], 3);
            problem.AddParameterBlock(t_array[i], 3);
            
            i++;
            
            int old_kf_index=0;
            
            for(KeyFrame* pKFi : real_vpCovKFi_cur){
               
                Matrix3d relative_r_b_a;
                Vector3d relative_t_b_a;
                
                
                //找一下 对应的应该是哪个相对位姿
                int relative_index=0;
                for(KeyFrame* pkFi_old: vpCovKFi_old){
                    if(pkFi_old==pKFi){
                        relative_r_b_a=oldR_b_a[relative_index];
                        relative_t_b_a=oldT_b_a[relative_index];
                        break;
                    }
                    relative_index++;
                }
                Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
     
                vector<Vector3d> point_single=point_clouds_all[old_kf_index];
                vector<cv::Point2f> measure_single=measurements_old_norm_all[old_kf_index];
                for(int a=0,b=point_single.size();a<b;a++){
                    
                    //找到主地图那个点 所在帧的位姿
                    Vector3d pts_i = point_single[a];

                    //相机平面坐标
                    cv::Point2f pt=measure_single[a];
                    float xx=pt.x;
                    float yy=pt.y;

                     Vector2d pts_j ;//要求是归一化图像坐标
                     pts_j<<xx,yy;

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0]);
                 }

                
                
                old_kf_index++;
                
                i++;
                
            }
            
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << "\n";
            
            if(summary.termination_type==ceres::
               CONVERGENCE){
                cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
                
                Matrix3d Rs_i ;
                Vector3d Ps_i ;//当前帧
                cur_kf->getOriginPose(Ps_i, Rs_i);


                Vector3d q;
                q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                Matrix3d Rs_loop = Utility::ypr2R(q);
                Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);

                Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                cur_kf->relative_pitch=relative_pitch;
                cur_kf->relative_roll=relative_roll;
                cur_kf->updateLoopConnection(relative_t, relative_yaw);
                
                
                //先临时搬过来
                old_index=old_kf->global_index;
                Vector3d T_w_i_old;
                Matrix3d R_w_i_old;
                old_kf->getPose(T_w_i_old, R_w_i_old);
                
                Quaterniond Q_loop_old(R_w_i_old);
                RetriveData retrive_data;
                retrive_data.cur_index = cur_kf->global_index;
                retrive_data.header = cur_kf->header;
                retrive_data.P_old = T_w_i_old;
                retrive_data.Q_old = Q_loop_old;
                retrive_data.use = true;
                retrive_data.measurements = measurements_old_norm_real;
                retrive_data.features_ids = features_id;//这个并没有赋值
                retrive_data.old_index=old_kf->global_index;
                vins->retrive_pose_data = (retrive_data);
//                        vins->isSendLoopData=true;
                
                cur_kf->detectLoop(old_kf->global_index);
                poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                old_kf->is_looped = 1;
                loop_old_index = old_kf->global_index;
                vins->isSendLoop_another=true;
                cur_kf->is_get_loop_info=true;
                return true;
                
            }else{
                cout<<"检测失败： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
                return false;
            }
            }else{
                cout<<"匹配帧数不够多="<<similarNum<<endl;
            }
            
            
               
    }
            //这个侯选帧， 和当前帧 以及当前帧的共视帧做匹配
            
            //可能是bow分数 ，或者几何一致性匹配？？？ 后面考虑
            
//        }
      
      return false;
  }
  catch(const std::string &ex)
  {
    cout << "Error loop: " << ex << endl;
    return false;
  }
}

//因为bow很耗时，所以只在当前帧和侯选帧之间bow选一次
//计算的相对位姿，比较一下直接计算和ceres优化谁的更好一点, ceres好非常非常多
//然后通过投影找匹配点
//这里的版本最终改成是发回客户端 计算相对位姿，自己计算的相对位姿不准
bool LoopClosure::startLoopClosure_5(KeyFrame* cur_kf)
{
  try
  {
    bool loop_succ = false;
      int old_index;
//      cout<<"测试是不是每一帧都参与回环检测？"<<cur_kf->global_index<<endl;
      
      
      //可以先只做一个简单的判断，不要检测时间一致性
      //这里返回的是分数超过阈值的
      vector<cv::Point2f> cur_pts;
      vector<cv::Point2f> old_pts;
      std::vector<cv::KeyPoint> keys=cur_kf->keypoints;
      std::vector<BRIEF::bitset> descriptors=cur_kf->descriptors;
      int best_id=demo.run_3("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, cur_kf->m_bowvec, cur_kf->m_featvec);
      
      treeId_kf[kfNum_tree]= cur_kf->global_index;
      kfNum_tree++;
      //描述符得全有
      if(!cur_kf->is_des_end){
          return false;
      }

      
      //检测成功后，检测共视帧的一致性
      //选出分数最高的3帧作为侯选帧，并计算一级共视帧的分数（最多选5个），求和，根据这个找到匹配性最高的侯选帧
    if(best_id!=-1){
        //这里直接加这么多也是有问题的 我想想 改好了
        int real_old_best_id=treeId_kf[best_id];
        KeyFrame* old_kf;
        old_kf= poseGraph->getKeyframe(real_old_best_id);
        
 
        
        Matrix3d R_relative;
        Vector3d T_relative;

        
        Client* client_cur=old_kf->c;
        Matrix3d ric_curClient=client_cur->ric_client;
        Vector3d tic_curClient=client_cur->tic_client;
        Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
        const float &fx = client_cur->FOCUS_LENGTH_X_server;
        const float &fy = client_cur->FOCUS_LENGTH_Y_server;
        const float &cx = client_cur->PX_server;
        const float &cy = client_cur->PY_server;
        
        //这个是不是可以用两帧 计算匹配点，计算基础矩阵 最终的特征点数量判断
        std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
        std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
        std::vector<Vector3d> point_3d_cur;
        std::vector<int> features_id;
        std::vector<cv::Point2f> measurements_cur;//像素坐标
        
        vector<vector<cv::Point2f>> measurements_old_norm_all;
        vector<vector<Eigen::Vector3d>> point_clouds_all;
        vector<KeyFrame*> real_vpCovKFi_cur;//最终匹配点数符合要求的
        //存储一下当前帧和候选帧的匹配关系
        std::vector<cv::Point2f> measurements_old_norm_real;//图像坐标
        std::vector<Vector3d> point_3d_cur_real;
        
        //有没有可能上面算的相对位姿不够准，通过bow的方式，找当前帧到 侯选帧的共视帧的匹配关系
        //优化得到当前帧位姿误差下 到侯选帧的位姿关系
        vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
        vpCovKFi_old.push_back(vpCovKFi_old[0]);
        vpCovKFi_old[0]=old_kf;
        //得到当前帧位姿误差下 到侯选帧与其共视帧之间的相对位姿关系
        vector<Matrix3d> old_r;
        vector<Vector3d> old_t;
        //这个存储候选帧的共视帧 之间的位姿关系, 是从候选帧到共视帧的位姿
        vector<Matrix3d> oldR_b_a;
        vector<Vector3d> oldT_b_a;
        Matrix3d oldKF_r;
        Vector3d oldKF_t;
        old_kf->getOriginPose(oldKF_t, oldKF_r);
        int temp_index=0;
        //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
        for(KeyFrame* pkFi_old: vpCovKFi_old){
            if(temp_index!=0){
                Matrix3d rwi_old;
                Vector3d twi_old;
                pkFi_old->getOriginPose(twi_old, rwi_old);
                
                Matrix3d r_b_a;
                Vector3d t_b_a;
                r_b_a=oldKF_r.transpose()* rwi_old;
                t_b_a=oldKF_r.transpose()*(twi_old-oldKF_t);
                oldR_b_a.push_back(r_b_a);
                oldT_b_a.push_back(t_b_a);
            }else{
                oldR_b_a.push_back(Matrix3d::Identity());
                oldT_b_a.push_back(Vector3d::Zero());
            }
            temp_index++;
        }
        //得到当前帧 与 侯选帧和共视帧 之间匹配关系
//        int num=0;
        point_clouds_all.clear();
        measurements_old_norm_all.clear();
        real_vpCovKFi_cur.clear();
        measurements_old_norm_real.clear();
        point_3d_cur_real.clear();
        
//        clock_t start,ends;

        measurements_old_coarse.clear();
        measurements_old_norm_coarse.clear();
        cur_kf->findConnectionWithOldFrame_server_old(old_kf, measurements_old_coarse, measurements_old_norm_coarse);
        if(measurements_old_coarse.size()<22){
            cout<<"当前帧与侯选帧的匹配点数少于22，"<<measurements_old_coarse.size()<<endl;
            return  false;
        }
        cout<<"测试 可能的匹配："<<setprecision(19)<<old_kf->global_index<<" "<<old_kf->header<<" "<<cur_kf->global_index<<" "<<cur_kf->header<<endl;
        measurements_old_norm_real=measurements_old_norm_coarse;
        point_3d_cur_real=cur_kf->point_clouds;
        features_id=cur_kf->features_id;
        point_clouds_all.push_back(point_3d_cur_real);
        measurements_old_norm_all.push_back(measurements_old_norm_real);
        real_vpCovKFi_cur.push_back(old_kf);
        cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false);
        //优化 得到当前帧位姿误差下 到侯选帧的位姿关系
//        start=clock();
        {
            ceres::Problem problem;
            ceres::Solver::Options options;
            //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            //options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 20;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
//            loss_function = new ceres::CauchyLoss(1.0);

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

            int i=0;
//            for(KeyFrame* pkf_old: real_vpCovKFi_cur){
                Matrix3d relative_r_b_a=oldR_b_a[i];
                Vector3d relative_t_b_a=oldT_b_a[i];
                Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                vector<Vector3d> point_single=point_clouds_all[i];
                vector<cv::Point2f> measure_single=measurements_old_norm_all[i];
                for(int a=0,b=point_single.size();a<b;a++){
                    //找到主地图那个点 所在帧的位姿
                    Vector3d pts_i = point_single[a];

                    //相机平面坐标
                    cv::Point2f pt=measure_single[a];
                    float xx=pt.x;
                    float yy=pt.y;

                     Vector2d pts_j ;//要求是归一化图像坐标
                     pts_j<<xx,yy;

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(), relative_t_b_a(0), relative_t_b_a(1), relative_t_b_a(2), relative_r_b_a_euler(0), relative_r_b_a_euler(1), relative_r_b_a_euler(2),100.0);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
                 }
//                i++;
//            }

            ceres::Solve(options, &problem, &summary);
//            std::cout <<"地图内部 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";

            if(summary.termination_type!=ceres::
               CONVERGENCE){
                return false;
            }

            T_relative[0]=t_array[0];
            T_relative[1]=t_array[1];
            T_relative[2]=t_array[2];

            R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));

        }
//        ends=clock();
//        cout<<"计算粗糙位姿花费的时间:"<<ends-start<<endl;
        //记录当前帧的共视帧 能够和候选帧的共视帧匹配上的数量
        int similarNum=0;
        {
            
            int i=0;
            for(KeyFrame* pkf_old:vpCovKFi_old){
                Matrix3d r_w_imuOld=R_relative*oldR_b_a[i];
                Vector3d t_w_imuOld=R_relative*oldT_b_a[i] +T_relative;
                Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                Matrix3d r_camOld_w=r_w_camOld.transpose();
                old_r.push_back(r_camOld_w);
                old_t.push_back(-r_camOld_w*t_w_camOld);
                i++;
                
            }
            //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
            
            //找5帧，加上自己，总共6帧，有3帧以上就行
            std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
            //如果这里报错，说明没有共视帧，那么记得加上共视帧阈值设大了的，补丁 加了加了
            vpCovKFi_cur[0] = cur_kf;
            
            
            
            point_clouds_all.clear();
            measurements_old_norm_all.clear();
            real_vpCovKFi_cur.clear();
            for(KeyFrame* pKFi : vpCovKFi_cur){
                //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
                if(pKFi==cur_kf){
                    point_clouds_all.push_back(point_3d_cur_real);
                    measurements_old_norm_all.push_back(measurements_old_norm_real);
                    real_vpCovKFi_cur.push_back(old_kf);
                    
                    continue;
                }
                //描述符得全有
                if(!pKFi->is_des_end){
                    continue;
                }
                
                vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                int nPoints = point_clouds_origin_cur.size();
                int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                
                
                int num=0;//记录匹配点的数量
                int temp_kf_index=0;//记录遍历到哪个帧了
                //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                for(KeyFrame* pkFi_old: vpCovKFi_old){
//                    start=clock();
                
                    //描述符得全有
                    if(!pkFi_old->is_des_end){
                        temp_kf_index++;
                        continue;
                    }
                    
                    measurements_old_coarse.clear();
                    measurements_old_norm_coarse.clear();
                    point_3d_cur.clear();
                    measurements_cur.clear();

                    vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                    //这个得到的是到imu坐标系的位姿
                    Matrix3d r_camOld_w=old_r[temp_kf_index];
                    Vector3d t_camOld_w=old_t[temp_kf_index];
                    
                    
                    
                    for(int i=0;i<nPoints;i++){
                        Vector3d point_main=point_clouds_origin_cur[i];
                        //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                        Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                        //深度必须为正
                        if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
//                            cout<<"深度不为正"<<endl;
                            continue;
                        }
                        // 投影到图像上
                        double x = p3D_c2[0];
                        double y = p3D_c2[1];
                        double z = p3D_c2[2];
                        //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                        double u=fx*x/z+cx;
                        double v=fy*y/z+cy;
                      
                        
                        if(!pkFi_old->isInImage((float)u, (float)v)){
//                            cout<<"投影点不在图像内"<<endl;
                            continue;
                        }
                        
                        const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 60);
                        //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                        if(vIndices.empty()){
//                            cout<<"半径为50个像素 找不到点"<<endl;
                            continue;
                        }
                        //des和keypoints长度不一样
                        //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                        BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                        int bestDist = 256;
                        int bestIndex = -1;
                        for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                        {
        //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                            int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                            if(dis < bestDist)
                            {
                                bestDist = dis;
                                bestIndex = *vit;
                            }
                        }
                        
                        if(bestDist<=60){
                            point_3d_cur.push_back(point_main);
                            measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                            measurements_cur.push_back(measurements_origin_cur[i]);
                            
                        }
                                      
                    }
                    
                    
//                    cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                    if(measurements_cur.size()>=15){
                        pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
                        cout<<"f矩阵拒绝后的点数："<<measurements_cur.size()<<", "<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<endl;
                        if(measurements_old_coarse.size()>=15){
                        
                            cv::Point2f norm_pt;
                            int pCount=measurements_old_coarse.size();
                            for(int aa=0;aa<pCount;aa++){
                                norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                measurements_old_coarse[aa]=norm_pt;
                                
                            }
                            
                            point_clouds_all.push_back(point_3d_cur);
                            measurements_old_norm_all.push_back(measurements_old_coarse);
                            num+=point_3d_cur.size();
                            real_vpCovKFi_cur.push_back(pkFi_old);
                            similarNum++;
                            break;
                        }
                        
                        
                    }
                    
                    temp_kf_index++;
//                    ends=clock();
//                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                }
                
                
            }
                
                
        }
            
           
            if(similarNum>=2){
                //先临时搬过来
                old_index=old_kf->global_index;
                Vector3d T_w_i_old;
                Matrix3d R_w_i_old;
                old_kf->getPose(T_w_i_old, R_w_i_old);

                Quaterniond Q_loop_old(R_w_i_old);
                RetriveData retrive_data;
                retrive_data.cur_index = cur_kf->global_index;
                retrive_data.header = cur_kf->header;
                retrive_data.P_old = T_w_i_old;
                retrive_data.Q_old = Q_loop_old;
                retrive_data.use = true;
                retrive_data.measurements = measurements_old_norm_real;
                retrive_data.features_ids = features_id;//这个并没有赋值
                retrive_data.old_index=old_kf->global_index;
                vins->retrive_pose_data = (retrive_data);
//                        vins->isSendLoopData=true;

                cur_kf->detectLoop(old_kf->global_index);
                poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                old_kf->is_looped = 1;
                loop_old_index = old_kf->global_index;
                vins->isSendLoopData=true;
            cout<<"检测到回环，old_index="<<old_kf->global_index<<" ,"<<cur_kf->global_index<<endl;
                return true;
            }
            
//        if(similarNum>=2){
//            //构造优化问题
//            double t_array[similarNum][3];//平移数组，其中存放每个关键帧的平移向量
//            Quaterniond q_array[similarNum];
//            double euler_array[similarNum][3];
//
//            ceres::Problem problem;
//            ceres::Solver::Options options;
//            //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
//            options.linear_solver_type = ceres::DENSE_SCHUR;
//            //options.minimizer_progress_to_stdout = true;
//            options.max_num_iterations = 20;
//            ceres::Solver::Summary summary;
//            ceres::LossFunction *loss_function;
//            loss_function = new ceres::HuberLoss(1.0);
//            //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
//            ceres::LocalParameterization* angle_local_parameterization =
//            AngleLocalParameterization::Create();
//
//            int i=0;
//            //这里应该可以只一个优化项，在当前帧位姿误差下，世界坐标系到老帧的位姿
//            //把第一项给老帧 匹配帧
//
//            Matrix3d tmp_r_old;
//            Vector3d tmp_t_old;
//            t_array[i][0] = T_relative(0);
//            t_array[i][1] = T_relative(1);
//            t_array[i][2] = T_relative(2);
//            //将矩阵转换为向量
//            Vector3d euler_angle_old = Utility::R2ypr(R_relative);
//            euler_array[i][0] = euler_angle_old.x();
//            euler_array[i][1] = euler_angle_old.y();
//            euler_array[i][2] = euler_angle_old.z();
//            problem.AddParameterBlock(euler_array[i], 3);
//            problem.AddParameterBlock(t_array[i], 3);
//
//            i++;
//
//            int old_kf_index=0;
//
//            for(KeyFrame* pKFi : real_vpCovKFi_cur){
//
//                Matrix3d relative_r_b_a;
//                Vector3d relative_t_b_a;
//
//
//                //找一下 对应的应该是哪个相对位姿
//                int relative_index=0;
//                for(KeyFrame* pkFi_old: vpCovKFi_old){
//                    if(pkFi_old==pKFi){
//                        relative_r_b_a=oldR_b_a[relative_index];
//                        relative_t_b_a=oldT_b_a[relative_index];
//                        break;
//                    }
//                    relative_index++;
//                }
//                Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
//
//                vector<Vector3d> point_single=point_clouds_all[old_kf_index];
//                vector<cv::Point2f> measure_single=measurements_old_norm_all[old_kf_index];
//                for(int a=0,b=point_single.size();a<b;a++){
//
//                    //找到主地图那个点 所在帧的位姿
//                    Vector3d pts_i = point_single[a];
//
//                    //相机平面坐标
//                    cv::Point2f pt=measure_single[a];
//                    float xx=pt.x;
//                    float yy=pt.y;
//
//                     Vector2d pts_j ;//要求是归一化图像坐标
//                     pts_j<<xx,yy;
//
//                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),relative_r_b_a_euler(1),relative_r_b_a_euler(2));
//                    problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0]);
//                 }
//
//
//
//                old_kf_index++;
//
//                i++;
//
//            }
//
//            ceres::Solve(options, &problem, &summary);
//            std::cout << summary.BriefReport() << "\n";
//
//            if(summary.termination_type==ceres::
//               CONVERGENCE){
//                cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
//
//                Matrix3d Rs_i ;
//                Vector3d Ps_i ;//当前帧
//                cur_kf->getOriginPose(Ps_i, Rs_i);
//
//
//                Vector3d q;
//                q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
//                Matrix3d Rs_loop = Utility::ypr2R(q);
//                Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
//
//                Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
////                Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
//                double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
//                double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
//                double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
//                cur_kf->relative_pitch=relative_pitch;
//                cur_kf->relative_roll=relative_roll;
//                cur_kf->updateLoopConnection(relative_t, relative_yaw);
//
//
//                //先临时搬过来
//                old_index=old_kf->global_index;
//                Vector3d T_w_i_old;
//                Matrix3d R_w_i_old;
//                old_kf->getPose(T_w_i_old, R_w_i_old);
//
//                Quaterniond Q_loop_old(R_w_i_old);
//                RetriveData retrive_data;
//                retrive_data.cur_index = cur_kf->global_index;
//                retrive_data.header = cur_kf->header;
//                retrive_data.P_old = T_w_i_old;
//                retrive_data.Q_old = Q_loop_old;
//                retrive_data.use = true;
//                retrive_data.measurements = measurements_old_norm_real;
//                retrive_data.features_ids = features_id;//这个并没有赋值
//                retrive_data.old_index=old_kf->global_index;
//                vins->retrive_pose_data = (retrive_data);
////                        vins->isSendLoopData=true;
//
//                cur_kf->detectLoop(old_kf->global_index);
//                poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
//                old_kf->is_looped = 1;
//                loop_old_index = old_kf->global_index;
//                vins->isSendLoopData=true;
////                cur_kf->is_get_loop_info=true;
//                return true;
//
//            }
//            else{
//                cout<<"检测失败： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
//                return false;
//            }
//            }
        else{
                cout<<"匹配帧数不够多="<<similarNum<<endl;
            }
            
            
               
    }
            //这个侯选帧， 和当前帧 以及当前帧的共视帧做匹配
            
            //可能是bow分数 ，或者几何一致性匹配？？？ 后面考虑
            
//        }
      
      return false;
  }
  catch(const std::string &ex)
  {
    cout << "Error loop: " << ex << endl;
    return false;
  }
}

//这里试图在服务器端计算相对位姿
//这里改好后，把多个客户端相对位姿计算改一下
//这里改成了 构建一个滑动窗口，计算相对位姿，但是算出来的位姿很差劲 这个暂时先放着
bool LoopClosure::startLoopClosure_6(KeyFrame* cur_kf)
{
  try
  {
    bool loop_succ = false;
      int old_index;
      cout<<"测试是不是每一帧都参与回环检测？"<<cur_kf->global_index<<endl;
      
      
      //可以先只做一个简单的判断，不要检测时间一致性
      //这里返回的是分数超过阈值的
      vector<cv::Point2f> cur_pts;
      vector<cv::Point2f> old_pts;
      std::vector<cv::KeyPoint> keys=cur_kf->keypoints;
      std::vector<BRIEF::bitset> descriptors=cur_kf->descriptors;
      int best_id=demo.run_3("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, cur_kf->m_bowvec, cur_kf->m_featvec);
      cur_kf->check_loop=true;
      treeId_kf[kfNum_tree]= cur_kf->global_index;
      kfNum_tree++;
      //描述符得全有
      if(!cur_kf->is_des_end){
          return false;
      }

      
      //检测成功后，检测共视帧的一致性
      //选出分数最高的3帧作为侯选帧，并计算一级共视帧的分数（最多选5个），求和，根据这个找到匹配性最高的侯选帧
    if(best_id!=-1){
        //这里直接加这么多也是有问题的 我想想 改好了
        int real_old_best_id=treeId_kf[best_id];
        KeyFrame* old_kf;
        old_kf= poseGraph->getKeyframe(real_old_best_id);
        
 
        
        Matrix3d R_relative;
        Vector3d T_relative;

        
        Client* client_cur=old_kf->c;
        Matrix3d ric_curClient=client_cur->ric_client;
        Vector3d tic_curClient=client_cur->tic_client;
        Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
        const float &fx = client_cur->FOCUS_LENGTH_X_server;
        const float &fy = client_cur->FOCUS_LENGTH_Y_server;
        const float &cx = client_cur->PX_server;
        const float &cy = client_cur->PY_server;
        
        //这个是不是可以用两帧 计算匹配点，计算基础矩阵 最终的特征点数量判断
        std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
        std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
        std::vector<Vector3d> point_3d_cur;
        std::vector<int> features_id;
        std::vector<cv::Point2f> measurements_cur;//像素坐标
        
        vector<vector<cv::Point2f>> measurements_old_norm_all;
        vector<vector<Eigen::Vector3d>> point_clouds_all;
        vector<KeyFrame*> real_vpCovKFi_cur;//最终匹配点数符合要求的
        //存储一下当前帧和候选帧的匹配关系
        std::vector<cv::Point2f> measurements_old_norm_real;//图像坐标
        std::vector<Vector3d> point_3d_cur_real;
        
        //有没有可能上面算的相对位姿不够准，通过bow的方式，找当前帧到 侯选帧的共视帧的匹配关系
        //优化得到当前帧位姿误差下 到侯选帧的位姿关系
        vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
        vpCovKFi_old.push_back(vpCovKFi_old[0]);
        vpCovKFi_old[0]=old_kf;
        //得到当前帧位姿误差下 到侯选帧与其共视帧之间的相对位姿关系
        vector<Matrix3d> old_r;
        vector<Vector3d> old_t;
        //这个存储候选帧的共视帧 之间的位姿关系, 是从候选帧到共视帧的位姿
        vector<Matrix3d> oldR_b_a;
        vector<Vector3d> oldT_b_a;
        Matrix3d oldKF_r;
        Vector3d oldKF_t;
        old_kf->getOriginPose(oldKF_t, oldKF_r);
        int temp_index=0;
        //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
        for(KeyFrame* pkFi_old: vpCovKFi_old){
            if(temp_index!=0){
                Matrix3d rwi_old;
                Vector3d twi_old;
                pkFi_old->getOriginPose(twi_old, rwi_old);
                
                Matrix3d r_b_a;
                Vector3d t_b_a;
                r_b_a=oldKF_r.transpose()* rwi_old;
                t_b_a=oldKF_r.transpose()*(twi_old-oldKF_t);
                oldR_b_a.push_back(r_b_a);
                oldT_b_a.push_back(t_b_a);
            }else{
                oldR_b_a.push_back(Matrix3d::Identity());
                oldT_b_a.push_back(Vector3d::Zero());
            }
            temp_index++;
        }
        //得到当前帧 与 侯选帧和共视帧 之间匹配关系
//        int num=0;
        point_clouds_all.clear();
        measurements_old_norm_all.clear();
        real_vpCovKFi_cur.clear();
        measurements_old_norm_real.clear();
        point_3d_cur_real.clear();
        
//        clock_t start,ends;

        measurements_old_coarse.clear();
        measurements_old_norm_coarse.clear();
        cur_kf->findConnectionWithOldFrame_server_old(old_kf, measurements_old_coarse, measurements_old_norm_coarse);
        if(measurements_old_coarse.size()<22){
//            cout<<"当前帧与侯选帧的匹配点数少于22，"<<measurements_old_coarse.size()<<endl;
            return  false;
        }
        cout<<"当前帧与侯选帧匹配点数："<<measurements_old_norm_coarse.size()<<endl;
        cout<<"测试 可能的匹配："<<setprecision(19)<<old_kf->global_index<<" "<<old_kf->header<<" "<<cur_kf->global_index<<" "<<cur_kf->header<<endl;
        measurements_old_norm_real=measurements_old_norm_coarse;
        point_3d_cur_real=cur_kf->point_clouds;
        features_id=cur_kf->features_id;
        vector<cv::Point2f> measurements_cur_real=cur_kf->measurements;
        assert(measurements_old_norm_real.size()==measurements_cur_real.size());
        
        point_clouds_all.push_back(point_3d_cur_real);
        measurements_old_norm_all.push_back(measurements_old_norm_real);
        real_vpCovKFi_cur.push_back(old_kf);
//        cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false); //这里其实没有用到了
        //优化 得到当前帧位姿误差下 到侯选帧的位姿关系
//        start=clock();
        {
            ceres::Problem problem;
            ceres::Solver::Options options;
            //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            //options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 20;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
//            loss_function = new ceres::CauchyLoss(1.0);

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

            int i=0;
//            for(KeyFrame* pkf_old: real_vpCovKFi_cur){
                Matrix3d relative_r_b_a=oldR_b_a[i];
                Vector3d relative_t_b_a=oldT_b_a[i];
                Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                vector<Vector3d> point_single=point_clouds_all[i];
                vector<cv::Point2f> measure_single=measurements_old_norm_all[i];
                for(int a=0,b=point_single.size();a<b;a++){
                    //找到主地图那个点 所在帧的位姿
                    Vector3d pts_i = point_single[a];

                    //相机平面坐标
                    cv::Point2f pt=measure_single[a];
                    float xx=pt.x;
                    float yy=pt.y;

                     Vector2d pts_j ;//要求是归一化图像坐标
                     pts_j<<xx,yy;

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(), relative_t_b_a(0), relative_t_b_a(1), relative_t_b_a(2), relative_r_b_a_euler(0), relative_r_b_a_euler(1), relative_r_b_a_euler(2),100.0);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
                 }
//                i++;
//            }

            ceres::Solve(options, &problem, &summary);
//            std::cout <<"地图内部 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";

            if(summary.termination_type!=ceres::
               CONVERGENCE){
                return false;
            }

            T_relative[0]=t_array[0];
            T_relative[1]=t_array[1];
            T_relative[2]=t_array[2];

            R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));

        }
//        ends=clock();
//        cout<<"计算粗糙位姿花费的时间:"<<ends-start<<endl;
        //记录当前帧的共视帧 能够和候选帧的共视帧匹配上的数量
        int similarNum=0;
        //找5帧，加上自己，总共6帧，有3帧以上就行
        std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
        vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
        //如果这里报错，说明没有共视帧，那么记得加上共视帧阈值设大了的，补丁 加了加了
        vpCovKFi_cur[0] = cur_kf;
        {
            
            int i=0;
            for(KeyFrame* pkf_old:vpCovKFi_old){
                Matrix3d r_w_imuOld=R_relative*oldR_b_a[i];
                Vector3d t_w_imuOld=R_relative*oldT_b_a[i] +T_relative;
                Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                Matrix3d r_camOld_w=r_w_camOld.transpose();
                old_r.push_back(r_camOld_w);
                old_t.push_back(-r_camOld_w*t_w_camOld);
                i++;
                
            }
            //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
            
            point_clouds_all.clear();
            measurements_old_norm_all.clear();
            real_vpCovKFi_cur.clear();
            for(KeyFrame* pKFi : vpCovKFi_cur){
                //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
                if(pKFi==cur_kf){
                    point_clouds_all.push_back(point_3d_cur_real);
                    measurements_old_norm_all.push_back(measurements_old_norm_real);
                    real_vpCovKFi_cur.push_back(old_kf);
                    
                    continue;
                }
                //描述符得全有
                if(!pKFi->is_des_end){
                    continue;
                }
                
                vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                int nPoints = point_clouds_origin_cur.size();
                int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                
//                vector<int> featureId_cur=pKFi->features_id_origin;
                
                int num=0;//记录匹配点的数量
                int temp_kf_index=0;//记录遍历到哪个帧了
                //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                for(KeyFrame* pkFi_old: vpCovKFi_old){
//                    start=clock();
                
                    //描述符得全有
                    if(!pkFi_old->is_des_end){
                        temp_kf_index++;
                        continue;
                    }
                    
                    measurements_old_coarse.clear();
                    measurements_old_norm_coarse.clear();
                    point_3d_cur.clear();
                    measurements_cur.clear();
                    
//                    featureId_single.clear();

                    vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                    //这个得到的是到imu坐标系的位姿
                    Matrix3d r_camOld_w=old_r[temp_kf_index];
                    Vector3d t_camOld_w=old_t[temp_kf_index];
                    
                    
                    
                    for(int i=0;i<nPoints;i++){
                        Vector3d point_main=point_clouds_origin_cur[i];
                        //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                        Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                        //深度必须为正
                        if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
//                            cout<<"深度不为正"<<endl;
                            continue;
                        }
                        // 投影到图像上
                        double x = p3D_c2[0];
                        double y = p3D_c2[1];
                        double z = p3D_c2[2];
                        //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                        double u=fx*x/z+cx;
                        double v=fy*y/z+cy;
                      
                        
                        if(!pkFi_old->isInImage((float)u, (float)v)){
//                            cout<<"投影点不在图像内"<<endl;
                            continue;
                        }
                        
                        const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 60);
                        //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                        if(vIndices.empty()){
//                            cout<<"半径为50个像素 找不到点"<<endl;
                            continue;
                        }
                        //des和keypoints长度不一样
                        //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                        BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                        int bestDist = 256;
                        int bestIndex = -1;
                        for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                        {
        //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                            int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                            if(dis < bestDist)
                            {
                                bestDist = dis;
                                bestIndex = *vit;
                            }
                        }
                        
                        if(bestDist<=60){
                            point_3d_cur.push_back(point_main);
                            measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                            measurements_cur.push_back(measurements_origin_cur[i]);
                            
//                            featureId_single.push_back(featureId_cur[i]);
                        }
                                      
                    }
                    
                    
//                    cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                    if(measurements_cur.size()>=15){
                        pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
                        cout<<"f矩阵拒绝后的点数："<<measurements_cur.size()<<", "<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<endl;
                        if(measurements_old_coarse.size()>=15){
                        
                            cv::Point2f norm_pt;
                            int pCount=measurements_old_coarse.size();
                            for(int aa=0;aa<pCount;aa++){
                                norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                measurements_old_coarse[aa]=norm_pt;
                                
                            }
                            
                            point_clouds_all.push_back(point_3d_cur);
                            measurements_old_norm_all.push_back(measurements_old_coarse);
                            num+=point_3d_cur.size();
                            real_vpCovKFi_cur.push_back(pkFi_old);
                            similarNum++;
                            
                            
                            
                           
                            break;
                        }
                        
                        
                    }
                    
                    temp_kf_index++;
//                    ends=clock();
//                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                }
                
                if(similarNum>=2){
                    break;
                }
            }
                
                
        }
        
        
//        std::vector<KeyFrame*> vpCovKFi_cur_calPose = cur_kf->GetBestCovisibilityKeyFrames(9);
//        vpCovKFi_cur_calPose.push_back(vpCovKFi_cur_calPose[0]);
//        vpCovKFi_cur_calPose[0] = cur_kf;
        feature.clear();
        int vpCovKFi_cur_len=vpCovKFi_cur.size();
        vector<pair<int,KeyFrame*> > vPairs;
        vPairs.reserve(vpCovKFi_cur_len);
        // 取出所有连接的关键帧，mConnectedKeyFrameWeights的类型为std::map<KeyFrame*,int>，而vPairs变量将共视的3D点数放在前面，利于排序
        for(int i=0;i<vpCovKFi_cur_len;i++){
           vPairs.push_back(make_pair(vpCovKFi_cur[i]->global_index,vpCovKFi_cur[i]));
        }
        // 按照权重进行排序 验证一下就是从小到大排序 是的
        sort(vPairs.begin(),vPairs.end());
        list<KeyFrame*> lKFs; // keyframe
        for(size_t i=0; i<vpCovKFi_cur_len;i++)
        {
            lKFs.push_back(vPairs[i].second);
        }
        int kf_num=0;
        for(KeyFrame* pKFi : lKFs){
            kf_num++;
            vector<Vector3d> point_3d=pKFi->point_clouds_origin;
            vector<int> featureId_single=pKFi->features_id_origin;
            vector<cv::Point2f> measurement_2d=pKFi->measurements_origin;
            
            int temp_featureId=0;
            //添加到临时滑动窗口
            for (auto &id_pts : point_3d)
            {
                
//                f_per_fra.frame_id=kf_num;
                //获取特征点的id
                int feature_id = featureId_single[temp_featureId];
                //在feature中查找该feature_id的feature是否存在
                auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                                  {
                                      return it.feature_id == feature_id;
                                  });
                //new feature
                if (it == feature.end())
                {
                    FeaturePerFrame f_per_fra(id_pts);
                    //没有找到该feature的id，则把特征点放入feature的list容器中
                    feature.push_back(FeaturePerId(feature_id, kf_num)); //give id and start frame
                    feature.back().feature_per_frame.push_back(f_per_fra);    //give point
                }
                //find match with previous feature
                else if (it->feature_id == feature_id)
                {
                    FeaturePerFrame f_per_fra(Vector3d(measurement_2d[temp_featureId].x, measurement_2d[temp_featureId].y, 1));
                    it->feature_per_frame.push_back(f_per_fra);
//                    it->used_num=it->feature_per_frame.size();
    //                                    last_track_num ++;//特征点被跟踪的次数+1
                }
                temp_featureId++;
            }
        }
         
        
            
          /**
            if(similarNum>=2){
                //先临时搬过来
                Matrix3d Rs_i ;
                Vector3d Ps_i ;//当前帧
                cur_kf->getOriginPose(Ps_i, Rs_i);


                
                Matrix3d Rs_loop = R_relative;
                Vector3d Ps_loop = T_relative;

                Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                cur_kf->relative_pitch=relative_pitch;
                cur_kf->relative_roll=relative_roll;
                cur_kf->updateLoopConnection(relative_t, relative_yaw);

                
                old_index=old_kf->global_index;
                Vector3d T_w_i_old;
                Matrix3d R_w_i_old;
                old_kf->getPose(T_w_i_old, R_w_i_old);

                Quaterniond Q_loop_old(R_w_i_old);
                RetriveData retrive_data;
                retrive_data.cur_index = cur_kf->global_index;
                retrive_data.header = cur_kf->header;
                retrive_data.P_old = T_w_i_old;
                retrive_data.Q_old = Q_loop_old;
                retrive_data.use = true;
                retrive_data.measurements = measurements_old_norm_real;
                retrive_data.features_ids = features_id;//这个并没有赋值
                retrive_data.old_index=old_kf->global_index;
                vins->retrive_pose_data = (retrive_data);
//                        vins->isSendLoopData=true;

                cur_kf->detectLoop(old_kf->global_index);
                poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                old_kf->is_looped = 1;
                loop_old_index = old_kf->global_index;
                vins->isSendLoop_another=true;
                cur_kf->is_get_loop_info=true;
            cout<<"检测到回环，old_index="<<old_kf->global_index<<" ,"<<cur_kf->global_index<<endl;
                return true;
            }
           */
            
        if(similarNum>=2){
            int optiKf_num=2;
            //构造优化问题
            double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
            Quaterniond q_array[optiKf_num];
            double euler_array[optiKf_num][3];

            ceres::Problem problem;
            ceres::Solver::Options options;
            //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            //options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 20;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
//            loss_function = new ceres::HuberLoss(1.0);
                        loss_function = new ceres::CauchyLoss(1.0);
            //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
//            ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

            int i=0;
            //这里应该可以只一个优化项，在当前帧位姿误差下，世界坐标系到老帧的位姿
            //把第一项给老帧 匹配帧

            Matrix3d tmp_r_old;
            Vector3d tmp_t_old;
            t_array[i][0] = oldKF_t(0);
            t_array[i][1] = oldKF_t(1);
            t_array[i][2] = oldKF_t(2);
            //将矩阵转换为向量
            Vector3d euler_angle_old = Utility::R2ypr(oldKF_r);
            euler_array[i][0] = euler_angle_old.x();
            euler_array[i][1] = euler_angle_old.y();
            euler_array[i][2] = euler_angle_old.z();
            problem.AddParameterBlock(euler_array[i], 3);
            problem.AddParameterBlock(t_array[i], 3);

//            cout<<"计算得到的相对位姿："<<t_array[i][0]<<" ,"<<t_array[i][1]<<" ,"<<t_array[i][2]<<" ,"<<euler_array[i][0]<<" ,"<<euler_array[i][1]<<" ,"<<euler_array[i][2]<<endl;

            
            i++;
            
            
//            int fixKf_index=cur_kf->global_index;
//            int fixKf_num=0;
            int curKf_num=0;
            //把当前帧也放进去优化，临时构造一个小型滑动窗口
            for(KeyFrame* pKFi : lKFs){
                
//                if(fixKf_index>pKFi->global_index){
//                    fixKf_index=pKFi->global_index;
//                    fixKf_num=i;
//                }
                if(pKFi==cur_kf){
                    Matrix3d tmp_r_old;
                    Vector3d tmp_t_old;
                    pKFi->getOriginPose(tmp_t_old, tmp_r_old);
                    t_array[1][0] = tmp_t_old(0);
                    t_array[1][1] = tmp_t_old(1);
                    t_array[1][2] = tmp_t_old(2);
                    //将矩阵转换为向量
                    Vector3d euler_angle_old = Utility::R2ypr(tmp_r_old);
                    euler_array[1][0] = euler_angle_old.x();
                    euler_array[1][1] = euler_angle_old.y();
                    euler_array[1][2] = euler_angle_old.z();
                    problem.AddParameterBlock(euler_array[1], 3);
                    problem.AddParameterBlock(t_array[1], 3);
                    curKf_num=i;
                }
//                else{
//                    problem.SetParameterBlockConstant(euler_array[i]);
//                    problem.SetParameterBlockConstant(t_array[i]);
//                }
                i++;
            }
//            assert(fixKf_num==1);
//            problem.SetParameterBlockConstant(euler_array[fixKf_num]);
//            problem.SetParameterBlockConstant(t_array[fixKf_num]);
            
            
            int retrive_feature_index = 0;
            
            /**
            
            for (auto &it_per_id : feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 2 )
                    continue;
//                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                
                //判断是否能够投影到当前帧，其它的约束不加
                if(imu_i+it_per_id.used_num-1>=curKf_num && imu_i<curKf_num){
                
                    //第一个观测到该特征的帧对应的特征点坐标
                    Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.feature_per_frame[0].z;
                    
                    while(features_id[retrive_feature_index] < it_per_id.feature_id)
                    {
                        retrive_feature_index++;
                    }
                    
                    if(features_id[retrive_feature_index] == it_per_id.feature_id)
                    {
                        Vector3d pts_j = Vector3d(measurements_old_norm_real[retrive_feature_index].x, measurements_old_norm_real[retrive_feature_index].y, 1.0);
//                        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.feature_per_frame[0].z;
                        
                        
                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_another::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z());
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0]);
                        
                        retrive_feature_index++;
//                        cur_num++;
                    }
                    //遍历能观测到该特征的每个帧
                    for (auto &it_per_frame : it_per_id.feature_per_frame)
                    {
                        imu_j++;
    //                    cout<<"测试这里应该相等吗？"<<imu_j<<" , "<<it_per_frame.frame_id<<endl;
                        if (imu_j != curKf_num || imu_j== imu_i)
                        {
                            continue;
                        }
                        
                        Vector3d pts_j = it_per_frame.point;
                        double xx=pts_j[0];
                        double yy=pts_j[1];
                        xx = (xx -  cx)/ fx;
                        yy = (yy -  cy)/ fy;
                        


                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_another::Create( pts_i.x(), pts_i.y(), pts_i.z(),xx , yy, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z());
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[1],t_array[1]);
//                        cur_num++;
                        break;
                    }
                }
            }
             */
            
            
            int cur_num=0;
            for (auto &it_per_id : feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 2 )
                    continue;
//                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

                //判断是否能够投影到当前帧，其它的约束不加
                if(imu_i+it_per_id.used_num-1>=curKf_num){

                    //第一个观测到该特征的帧对应的特征点坐标
                    Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.feature_per_frame[0].z;
                    //遍历能观测到该特征的每个帧
                    for (auto &it_per_frame : it_per_id.feature_per_frame)
                    {
                        imu_j++;
    //                    cout<<"测试这里应该相等吗？"<<imu_j<<" , "<<it_per_frame.frame_id<<endl;
                        if (imu_j != curKf_num || imu_j== imu_i)
                        {
                            continue;
                        }

                        Vector3d pts_j = it_per_frame.point;
                        double xx=pts_j[0];
                        double yy=pts_j[1];
                        xx = (xx -  cx)/ fx;
                        yy = (yy -  cy)/ fy;



                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_another::Create( pts_i.x(), pts_i.y(), pts_i.z(),xx , yy, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),100.0);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[1],t_array[1]);
                        cur_num++;
                        break;
                    }
                }
            }
            
//            cout<<"当前帧的约束点数："<<cur_num<<endl;
            //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
             
            int old_kf_index=0;

            cur_num=0;
            //遍历特征
            for (auto &it_per_id : feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 2 )
                    continue;
               
               
                //获取观测到该特征的起始帧
                int start = it_per_id.start_frame;
                //feature has been obeserved in ith frame
                int end = (int)(start + it_per_id.feature_per_frame.size() - curKf_num - 1);
                if(start <= curKf_num && end >=0)
                {
                    while(features_id[retrive_feature_index] < it_per_id.feature_id)
                    {
                        retrive_feature_index++;
                    }
                    
                    if(features_id[retrive_feature_index] == it_per_id.feature_id)
                    {
                        Vector3d pts_j = Vector3d(measurements_old_norm_real[retrive_feature_index].x, measurements_old_norm_real[retrive_feature_index].y, 1.0);
                        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.feature_per_frame[0].z;
                        
                        
                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_another::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),100.0);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0]);
                        
//                        double x=measurements_cur_real[retrive_feature_index].x, y=measurements_cur_real[retrive_feature_index].y;
//                        x=(x-cx)/fx;
//                        y=(y-cy)/fy;
//                        ceres::CostFunction* cost_function2 = FourSixDOFWeightError_reprojection_another::Create( pts_i.x(), pts_i.y(), pts_i.z(),x , y, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z());
//                        problem.AddResidualBlock(cost_function2, loss_function, euler_array[1],t_array[1]);
                        
                        
                        retrive_feature_index++;
                        cur_num++;
                    }
                    
                }
            }
            cout<<"老帧约束点数："<<cur_num<<endl;
             
            /**
            for(KeyFrame* pKFi : real_vpCovKFi_cur){

                Matrix3d relative_r_b_a;
                Vector3d relative_t_b_a;


                //找一下 对应的应该是哪个相对位姿
                int relative_index=0;
                for(KeyFrame* pkFi_old: vpCovKFi_old){
                    if(pkFi_old==pKFi){
                        relative_r_b_a=oldR_b_a[relative_index];
                        relative_t_b_a=oldT_b_a[relative_index];
                        break;
                    }
                    relative_index++;
                }
                Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);

                vector<Vector3d> point_single=point_clouds_all[old_kf_index];
                vector<cv::Point2f> measure_single=measurements_old_norm_all[old_kf_index];
                for(int a=0,b=point_single.size();a<b;a++){

                    //找到主地图那个点 所在帧的位姿
                    Vector3d pts_i = point_single[a];

                    //相机平面坐标
                    cv::Point2f pt=measure_single[a];
                    float xx=pt.x;
                    float yy=pt.y;

                     Vector2d pts_j ;//要求是归一化图像坐标
                     pts_j<<xx,yy;

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),relative_r_b_a_euler(1),relative_r_b_a_euler(2));
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0]);
                 }



                old_kf_index++;

//                i++;

            }
             */
            
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << "\n";

            if(summary.termination_type==ceres::CONVERGENCE){
//            {
                cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;

//                Matrix3d Rs_i ;
//                Vector3d Ps_i ;//当前帧
//                cur_kf->getOriginPose(Ps_i, Rs_i);
                Vector3d q_i;
                q_i<<euler_array[1][0],euler_array[1][1],euler_array[1][2];
                Matrix3d Rs_i = Utility::ypr2R(q_i);
                Vector3d Ps_i = Vector3d( t_array[1][0],  t_array[1][1],  t_array[1][2]);


                Vector3d q;
                q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                Matrix3d Rs_loop = Utility::ypr2R(q);
                Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
                
                cout<<"优化计算得到的相对位姿："<<Ps_loop[0]<<" ,"<<Ps_loop[1]<<" ,"<<Ps_loop[2]<<" ,"<<euler_array[0][0]<<" ,"<<euler_array[0][1]<<" ,"<<euler_array[0][2]<<endl;

                Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                cur_kf->relative_pitch=relative_pitch;
                cur_kf->relative_roll=relative_roll;
                cur_kf->updateLoopConnection(relative_t, relative_yaw);


                //先临时搬过来
                old_index=old_kf->global_index;
                Vector3d T_w_i_old;
                Matrix3d R_w_i_old;
                old_kf->getPose(T_w_i_old, R_w_i_old);

                Quaterniond Q_loop_old(R_w_i_old);
                RetriveData retrive_data;
                retrive_data.cur_index = cur_kf->global_index;
                retrive_data.header = cur_kf->header;
                retrive_data.P_old = T_w_i_old;
                retrive_data.Q_old = Q_loop_old;
                retrive_data.use = true;
                retrive_data.measurements = measurements_old_norm_real;
                retrive_data.features_ids = features_id;//这个并没有赋值 赋值了
                retrive_data.old_index=old_kf->global_index;
                vins->retrive_pose_data = (retrive_data);
//                        vins->isSendLoopData=true;

                cur_kf->detectLoop(old_kf->global_index);
                poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                old_kf->is_looped = 1;
                loop_old_index = old_kf->global_index;
                vins->isSendLoop_another=true;
                if(cur_kf->IsOriginUpdate==true){
                   if (cur_kf->sendLoop==false)
                   {

                       cur_kf->sendLoop=true;
                       vins->globalOpti_index_mutex.lock();
                       vins->kf_global_index.push(cur_kf->global_index);
                       vins->start_kf_global_index.push(cur_kf->loop_index);
                       poseGraph->latest_loop_index=cur_kf->global_index;
                       vins->start_global_optimization = true;//先暂停回环 不启动

                       vins->globalOpti_index_mutex.unlock();
                       cout<<"慢了 慢了检测到回环 并通过错误回环的检测："<<cur_kf->global_index<<endl;
                   }
               }
                cur_kf->is_get_loop_info=true;
                return true;

            }
//            else{
//                cout<<"检测失败： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
//                return false;
//            }
        }
        else{
            cout<<"匹配帧数不够多="<<similarNum<<endl;
        }
            
            
               
    }
            //这个侯选帧， 和当前帧 以及当前帧的共视帧做匹配
            
            //可能是bow分数 ，或者几何一致性匹配？？？ 后面考虑
            
//        }
      
      return false;
  }
  catch(const std::string &ex)
  {
    cout << "Error loop: " << ex << endl;
    return false;
  }
}

//这个最好效果达到过0.058 但是似乎不稳定 后面7cm 7.9cm
bool LoopClosure::startLoopClosure_7(KeyFrame* cur_kf)
{
  try
  {
//    bool loop_succ = false;
      int old_index;
      cout<<"测试是不是每一帧都参与回环检测？"<<cur_kf->global_index<<endl;
     
      //可以先只做一个简单的判断，不要检测时间一致性
      //这里返回的是分数超过阈值的
      vector<cv::Point2f> cur_pts;
      vector<cv::Point2f> old_pts;
      std::vector<cv::KeyPoint> keys=cur_kf->keypoints;
      std::vector<BRIEF::bitset> descriptors=cur_kf->descriptors;
      int best_id=demo.run_3("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, cur_kf->m_bowvec, cur_kf->m_featvec);
      cur_kf->check_loop=true;
      treeId_kf[kfNum_tree]= cur_kf->global_index;
      kfNum_tree++;
      //描述符得全有
      if(!cur_kf->is_des_end){
          cout<<"描述符不齐全："<<cur_kf->global_index<<endl;
          return false;
      }

      
      //检测成功后，检测共视帧的一致性
      //选出分数最高的3帧作为侯选帧，并计算一级共视帧的分数（最多选5个），求和，根据这个找到匹配性最高的侯选帧
    if(best_id!=-1){
        //这里直接加这么多也是有问题的 我想想 改好了
        int real_old_best_id=treeId_kf[best_id];
        KeyFrame* old_kf;
        old_kf= poseGraph->getKeyframe(real_old_best_id);
        
        Matrix3d R_relative;
        Vector3d T_relative;

        Client* client_cur=old_kf->c;
        Matrix3d ric_curClient=client_cur->ric_client;
        Vector3d tic_curClient=client_cur->tic_client;
        Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
        const float &fx = client_cur->FOCUS_LENGTH_X_server;
        const float &fy = client_cur->FOCUS_LENGTH_Y_server;
        const float &cx = client_cur->PX_server;
        const float &cy = client_cur->PY_server;
        
        //这个是不是可以用两帧 计算匹配点，计算基础矩阵 最终的特征点数量判断
        std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
        std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
        std::vector<Vector3d> point_3d_cur;
        std::vector<cv::Point2f> measurements_cur;//像素坐标
        
        vector<vector<cv::Point2f>> measurements_old_norm_all;
        vector<vector<Eigen::Vector3d>> point_clouds_all;
//        vector<KeyFrame*> real_vpCovKFi_cur;//最终匹配点数符合要求的
        
        //存储一下当前帧和候选帧的匹配关系
        std::vector<cv::Point2f> measurements_old_norm_real;//图像坐标
        std::vector<Vector3d> point_3d_cur_real;
        std::vector<int> features_id_cur_real;
        
        //有没有可能上面算的相对位姿不够准，通过bow的方式，找当前帧到 侯选帧的共视帧的匹配关系
        //优化得到当前帧位姿误差下 到侯选帧的位姿关系
        vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
        vpCovKFi_old.push_back(vpCovKFi_old[0]);
        vpCovKFi_old[0]=old_kf;
        //得到当前帧位姿误差下 到侯选帧与其共视帧之间的相对位姿关系
        vector<Matrix3d> old_r;
        vector<Vector3d> old_t;
        //这个存储候选帧的共视帧 之间的位姿关系, 是从候选帧到共视帧的位姿
        vector<Matrix3d> oldR_b_a;
        vector<Vector3d> oldT_b_a;
        Matrix3d oldKF_r;
        Vector3d oldKF_t;
        old_kf->getOriginPose(oldKF_t, oldKF_r);
        int temp_index=0;
        //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
        for(KeyFrame* pkFi_old: vpCovKFi_old){
            if(temp_index!=0){
                Matrix3d rwi_old;
                Vector3d twi_old;
                pkFi_old->getOriginPose(twi_old, rwi_old);
                
                Matrix3d r_b_a;
                Vector3d t_b_a;
                r_b_a=oldKF_r.transpose()* rwi_old;
                t_b_a=oldKF_r.transpose()*(twi_old-oldKF_t);
                oldR_b_a.push_back(r_b_a);
                oldT_b_a.push_back(t_b_a);
            }else{
                oldR_b_a.push_back(Matrix3d::Identity());
                oldT_b_a.push_back(Vector3d::Zero());
                temp_index++;//放这里就只要执行一次
            }
        }
        //得到当前帧 与 侯选帧和共视帧 之间匹配关系

        point_clouds_all.clear();
        measurements_old_norm_all.clear();
//        real_vpCovKFi_cur.clear();
        measurements_old_norm_real.clear();
        point_3d_cur_real.clear();

        measurements_old_coarse.clear();
        measurements_old_norm_coarse.clear();
        cur_kf->findConnectionWithOldFrame_server_old(old_kf, measurements_old_coarse, measurements_old_norm_coarse);
        if(measurements_old_coarse.size()<22){
//            cout<<"当前帧与侯选帧的匹配点数少于22，"<<measurements_old_coarse.size()<<", "<<cur_kf->global_index<<endl;
            return  false;
        }
//        cout<<"当前帧与侯选帧匹配点数："<<measurements_old_norm_coarse.size()<<endl;
        cout<<endl<<"测试 可能的匹配："<<setprecision(19)<<old_kf->global_index<<" "<<old_kf->header<<" "<<cur_kf->global_index<<" "<<cur_kf->header<<endl;
        measurements_old_norm_real=measurements_old_norm_coarse;
        point_3d_cur_real=cur_kf->point_clouds;
        features_id_cur_real=cur_kf->features_id;
        point_clouds_all.push_back(point_3d_cur_real);
        measurements_old_norm_all.push_back(measurements_old_norm_real);
//        real_vpCovKFi_cur.push_back(old_kf);
//        cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false); //这里其实没有用到了
        //优化 得到当前帧位姿误差下 到侯选帧的位姿关系
//        start=clock();
        {
            //先验换一下
            Matrix3d curKF_r;
            Vector3d curKF_t;
            old_kf->getPose(curKF_t, curKF_r);
            
            
            ceres::Problem problem;
            ceres::Solver::Options options;
            //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            //options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 20;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);

            double t_array[3];//平移数组，其中存放每个关键帧的平移向量
            double euler_array[3];
            t_array[0] = curKF_t(0);
            t_array[1] = curKF_t(1);
            t_array[2] = curKF_t(2);
            //将矩阵转换为向量
            Vector3d euler_angle_old = Utility::R2ypr(curKF_r);
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
//                float xx=pt.x;
//                float yy=pt.y;

//                 Vector2d pts_j ;//要求是归一化图像坐标
//                 pts_j<<xx,yy;

                ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection2::Create( pts_i.x(), pts_i.y(), pts_i.z(),pt.x, pt.y, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),100.0);
                problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
             }


            ceres::Solve(options, &problem, &summary);
//            std::cout <<"地图内部 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";

            if(summary.termination_type!=ceres::
               CONVERGENCE){
                return false;
            }

            T_relative[0]=t_array[0];
            T_relative[1]=t_array[1];
            T_relative[2]=t_array[2];

            R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));

        }
//        ends=clock();
//        cout<<"计算粗糙位姿花费的时间:"<<ends-start<<endl;
        //记录当前帧的共视帧 能够和候选帧的共视帧匹配上的数量
        int similarNum=0;
        //找5帧，加上自己，总共6帧，有3帧以上就行
        std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
        vpCovKFi_cur.push_back(vpCovKFi_cur[0]);//这里可能存在没有共视帧的情况
        //如果这里报错，说明没有共视帧，那么记得加上共视帧阈值设大了的，补丁 加了加了
        vpCovKFi_cur[0] = cur_kf;
        
        vector<int> vpkf_index;
        {
            
            temp_index=0;
            for(KeyFrame* pkf_old:vpCovKFi_old){
                Matrix3d r_w_imuOld=R_relative*oldR_b_a[temp_index];
                Vector3d t_w_imuOld=R_relative*oldT_b_a[temp_index] +T_relative;
                Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                Matrix3d r_camOld_w=r_w_camOld.transpose();
                old_r.push_back(r_camOld_w);
                old_t.push_back(-r_camOld_w*t_w_camOld);
                temp_index++;
                
            }
            //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
            
            point_clouds_all.clear();
            measurements_old_norm_all.clear();
//            real_vpCovKFi_cur.clear();
            vpkf_index.clear();
            for(KeyFrame* pKFi : vpCovKFi_cur){
                //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
                if(pKFi==cur_kf){
                    point_clouds_all.push_back(point_3d_cur_real);
                    measurements_old_norm_all.push_back(measurements_old_norm_real);
//                    real_vpCovKFi_cur.push_back(old_kf);
                    vpkf_index.push_back(0);
                    temp_index++;
                    continue;
                }
                //描述符得全有
                if(!pKFi->is_des_end){
                    continue;
                }
                
                vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                int nPoints = point_clouds_origin_cur.size();
                int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
               
//                int num=0;//记录匹配点的数量
                temp_index=0;//记录遍历到哪个帧了
                //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                for(KeyFrame* pkFi_old: vpCovKFi_old){
//                    start=clock();
                    
                    //描述符得全有
                    if(!pkFi_old->is_des_end){
                        temp_index++;
                        continue;
                    }
                    
                    measurements_old_coarse.clear();
                    measurements_old_norm_coarse.clear();
                    point_3d_cur.clear();
                    measurements_cur.clear();
                    

                    vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                    //这个得到的是到imu坐标系的位姿
                    Matrix3d r_camOld_w=old_r[temp_index];
                    Vector3d t_camOld_w=old_t[temp_index];
                    
                    for(int i=0;i<nPoints;i++){
                        Vector3d point_main=point_clouds_origin_cur[i];
                        //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                        Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                        //深度必须为正
                        if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
//                            cout<<"深度不为正"<<endl;
                            continue;
                        }
                        // 投影到图像上
                        double x = p3D_c2[0];
                        double y = p3D_c2[1];
                        double z = p3D_c2[2];
                        //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                        double u=fx*x/z+cx;
                        double v=fy*y/z+cy;
                      
                        
                        if(!pkFi_old->isInImage((float)u, (float)v)){
//                            cout<<"投影点不在图像内"<<endl;
                            continue;
                        }
                        
                        const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 60);
                        //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                        if(vIndices.empty()){
//                            cout<<"半径为50个像素 找不到点"<<endl;
                            continue;
                        }
                        //des和keypoints长度不一样
                        //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                        BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                        int bestDist = 256;
                        int bestIndex = -1;
                        for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                        {
        //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                            int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                            if(dis < bestDist)
                            {
                                bestDist = dis;
                                bestIndex = *vit;
                            }
                        }
                        
                        if(bestDist<=60){
                            point_3d_cur.push_back(point_main);
                            measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                            measurements_cur.push_back(measurements_origin_cur[i]);
                            
                        }
                                    
                    }
                    
                    
//                    cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                    if(measurements_cur.size()>=15){
                        pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
//                        cout<<"f矩阵拒绝后的点数："<<measurements_cur.size()<<", "<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<endl;
                        if(measurements_old_coarse.size()>=15){
                        
                            cv::Point2f norm_pt;
                            int pCount=measurements_old_coarse.size();
                            for(int aa=0;aa<pCount;aa++){
                                norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                measurements_old_coarse[aa]=norm_pt;
                                
                            }
                            
                            point_clouds_all.push_back(point_3d_cur);
                            measurements_old_norm_all.push_back(measurements_old_coarse);
//                            num+=point_3d_cur.size();
//                            real_vpCovKFi_cur.push_back(pkFi_old);
                            vpkf_index.push_back(temp_index);
                            similarNum++;
                            
                            break;
                        }
                        
                    }
                    
                    temp_index++;
//                    ends=clock();
//                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                }
            }
        }
        
        if(similarNum>=2){
            int optiKf_num=vpkf_index.size()+1;
            //构造优化问题
            double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
            Quaterniond q_array[optiKf_num];
            double euler_array[optiKf_num][3];

            ceres::Problem problem;
            ceres::Solver::Options options;
            //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            //options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 20;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
//            ceres::LossFunction *loss_function_feature;
//            loss_function_feature = new ceres::CauchyLoss(1.0);
            //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
            ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

//            int i=0;
            //这里应该可以只一个优化项，在当前帧位姿误差下，世界坐标系到老帧的位姿
            //把第一项给老帧 匹配帧
            
            Matrix3d curKF_r;
            Vector3d curKF_t;
            old_kf->getPose(curKF_t, curKF_r);

            Matrix3d tmp_r_old;
            Vector3d tmp_t_old;
            t_array[0][0] = curKF_t(0);
            t_array[0][1] = curKF_t(1);
            t_array[0][2] = curKF_t(2);
            //将矩阵转换为向量
            Vector3d euler_angle_old = Utility::R2ypr(curKF_r);
            euler_array[0][0] = euler_angle_old.x();
            euler_array[0][1] = euler_angle_old.y();
            euler_array[0][2] = euler_angle_old.z();
            problem.AddParameterBlock(euler_array[0], 3);
            problem.AddParameterBlock(t_array[0], 3);

//            cout<<"计算得到的相对位姿："<<t_array[i][0]<<" ,"<<t_array[i][1]<<" ,"<<t_array[i][2]<<" ,"<<euler_array[i][0]<<" ,"<<euler_array[i][1]<<" ,"<<euler_array[i][2]<<endl;

            
//            i++;
            
            //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
            map<int,int> resample;
            
            temp_index=0;
            for(int len=vpkf_index.size(); temp_index<len; temp_index++ ){
                
                int kf_index=temp_index+1;
                Matrix3d relative_r_b_a;
                Vector3d relative_t_b_a;
                relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
                relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
                Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                
                map<int,int>::iterator iter;
                iter = resample.find(vpkf_index[temp_index]);
                if(iter != resample.end())
                {
                    kf_index=resample[vpkf_index[temp_index]];
                }
                else
                {
                       
                    resample[vpkf_index[temp_index]]=kf_index;

                    //找一下 对应的应该是哪个相对位姿
    //                int relative_index=0;
    //                for(KeyFrame* pkFi_old: vpCovKFi_old){
    //                    if(pkFi_old==pKFi){
    //                        relative_r_b_a=oldR_b_a[relative_index];
    //                        relative_t_b_a=oldT_b_a[relative_index];
    //                        break;
    //                    }
    //                    relative_index++;
    //                }
                    
                    t_array[kf_index][0] = relative_t_b_a(0);
                    t_array[kf_index][1] = relative_t_b_a(1);
                    t_array[kf_index][2] = relative_t_b_a(2);
                    euler_array[kf_index][0] = relative_r_b_a_euler.x();
                    euler_array[kf_index][1] = relative_r_b_a_euler.y();
                    euler_array[kf_index][2] = relative_r_b_a_euler.z();
                    problem.AddParameterBlock(euler_array[kf_index], 1, angle_local_parameterization);
                    problem.AddParameterBlock(t_array[kf_index], 3);
                    ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),1);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                }

                vector<Vector3d> point_single=point_clouds_all[temp_index];
                vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
                for(int a=0,b=point_single.size();a<b;a++){

                    //找到主地图那个点 所在帧的位姿
                    Vector3d pts_i = point_single[a];

                    //相机平面坐标
                    cv::Point2f pt=measure_single[a];
                    float xx=pt.x;
                    float yy=pt.y;

                     Vector2d pts_j ;//要求是归一化图像坐标
                     pts_j<<xx,yy;

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                    
                    
                 }

//                temp_index++;

//                i++;

            }
             
            
            ceres::Solve(options, &problem, &summary);
            std::cout <<"精确相对位姿："<< summary.BriefReport() << "\n";

            if(summary.termination_type==ceres::CONVERGENCE){
//            {
                cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;

                Matrix3d Rs_i ;
                Vector3d Ps_i ;//当前帧
                cur_kf->getOriginPose(Ps_i, Rs_i);
//                Vector3d q_i;
//                q_i<<euler_array[1][0],euler_array[1][1],euler_array[1][2];
//                Matrix3d Rs_i = Utility::ypr2R(q_i);
//                Vector3d Ps_i = Vector3d( t_array[1][0],  t_array[1][1],  t_array[1][2]);


                Vector3d q;
                q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                Matrix3d Rs_loop = Utility::ypr2R(q);
                Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
                
//                cout<<"优化计算得到的相对位姿："<<Ps_loop[0]<<" ,"<<Ps_loop[1]<<" ,"<<Ps_loop[2]<<" ,"<<euler_array[0][0]<<" ,"<<euler_array[0][1]<<" ,"<<euler_array[0][2]<<endl;

                Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                cur_kf->relative_pitch=relative_pitch;
                cur_kf->relative_roll=relative_roll;
                cur_kf->updateLoopConnection(relative_t, relative_yaw);


                //先临时搬过来
                old_index=old_kf->global_index;
                Vector3d T_w_i_old;
                Matrix3d R_w_i_old;
                old_kf->getPose(T_w_i_old, R_w_i_old);

                Quaterniond Q_loop_old(R_w_i_old);
                RetriveData retrive_data;
                retrive_data.cur_index = cur_kf->global_index;
                retrive_data.header = cur_kf->header;
                retrive_data.P_old = T_w_i_old;
                retrive_data.Q_old = Q_loop_old;
                retrive_data.use = true;
                retrive_data.measurements = measurements_old_norm_real;
                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
                retrive_data.old_index=old_kf->global_index;
                vins->retrive_pose_data = (retrive_data);
//                        vins->isSendLoopData=true;

                cur_kf->detectLoop(old_kf->global_index);
                poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                old_kf->is_looped = 1;
                loop_old_index = old_kf->global_index;
                vins->isSendLoop_another=true;
                
                
                if(cur_kf->IsOriginUpdate==true){
                   if (cur_kf->sendLoop==false)
                   {

                       cur_kf->sendLoop=true;
                       vins->globalOpti_index_mutex.lock();
                       vins->kf_global_index.push(cur_kf->global_index);
                       vins->start_kf_global_index.push(cur_kf->loop_index);
                       poseGraph->latest_loop_index=cur_kf->global_index;
                       vins->start_global_optimization = true;//先暂停回环 不启动

                       vins->globalOpti_index_mutex.unlock();
                       cout<<"慢了 慢了检测到回环 并通过错误回环的检测："<<cur_kf->global_index<<endl;
                   }
               }
                cur_kf->is_get_loop_info=true;
                
                return true;

            }
            else{
                cout<<"检测失败 未收敛： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
                return false;
            }
        }
        else{
            cout<<"匹配帧数不够多="<<similarNum<<endl;
        }
            
            
               
    }
            //这个侯选帧， 和当前帧 以及当前帧的共视帧做匹配
            
            //可能是bow分数 ，或者几何一致性匹配？？？ 后面考虑
            
//        }
      
      return false;
  }
  catch(const std::string &ex)
  {
    cout << "Error loop: " << ex << endl;
    return false;
  }
}

//这里先换回原始的回环检测方法 然后用现在的方法计算相对位姿，因为怀疑可能回环检测出问题，可能少了一些
bool LoopClosure::startLoopClosure_8(KeyFrame* cur_kf)
{
  try
  {
    
      int old_index;
      cout<<"测试是不是每一帧都参与回环检测？"<<cur_kf->global_index<<endl;
     
      //可以先只做一个简单的判断，不要检测时间一致性
      //这里返回的是分数超过阈值的
      vector<cv::Point2f> cur_pts;
      vector<cv::Point2f> old_pts;
      std::vector<cv::KeyPoint> keys=cur_kf->keypoints;
      std::vector<BRIEF::bitset> descriptors=cur_kf->descriptors;
      
     
      
      
      bool loop_succ = false;
      loop_succ = demo.run_2("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, cur_kf->m_bowvec, cur_kf->m_featvec);

      treeId_kf[kfNum_tree]= cur_kf->global_index;
      kfNum_tree++;
      //描述符得全有
      if(!cur_kf->is_des_end){
          return false;
      }
      
      //检测成功后，检测共视帧的一致性
      //选出分数最高的3帧作为侯选帧，并计算一级共视帧的分数（最多选5个），求和，根据这个找到匹配性最高的侯选帧
    if(loop_succ){
        //这里直接加这么多也是有问题的 我想想 改好了
        int real_old_best_id=treeId_kf[old_index];
        KeyFrame* old_kf;
        old_kf= poseGraph->getKeyframe(real_old_best_id);
        
        Matrix3d R_relative;
        Vector3d T_relative;

        Client* client_cur=old_kf->c;
        Matrix3d ric_curClient=client_cur->ric_client;
        Vector3d tic_curClient=client_cur->tic_client;
        Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
        const float &fx = client_cur->FOCUS_LENGTH_X_server;
        const float &fy = client_cur->FOCUS_LENGTH_Y_server;
        const float &cx = client_cur->PX_server;
        const float &cy = client_cur->PY_server;
        
        //这个是不是可以用两帧 计算匹配点，计算基础矩阵 最终的特征点数量判断
        std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
        std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
        std::vector<Vector3d> point_3d_cur;
        std::vector<cv::Point2f> measurements_cur;//像素坐标
        
        vector<vector<cv::Point2f>> measurements_old_norm_all;
        vector<vector<Eigen::Vector3d>> point_clouds_all;
        vector<KeyFrame*> real_vpCovKFi_cur;//最终匹配点数符合要求的
        //存储一下当前帧和候选帧的匹配关系
        std::vector<cv::Point2f> measurements_old_norm_real;//图像坐标
        std::vector<Vector3d> point_3d_cur_real;
        std::vector<int> features_id_cur_real;
        
        //有没有可能上面算的相对位姿不够准，通过bow的方式，找当前帧到 侯选帧的共视帧的匹配关系
        //优化得到当前帧位姿误差下 到侯选帧的位姿关系
        vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
        vpCovKFi_old.push_back(vpCovKFi_old[0]);
        vpCovKFi_old[0]=old_kf;
        //得到当前帧位姿误差下 到侯选帧与其共视帧之间的相对位姿关系
        vector<Matrix3d> old_r;
        vector<Vector3d> old_t;
        //这个存储候选帧的共视帧 之间的位姿关系, 是从候选帧到共视帧的位姿
        vector<Matrix3d> oldR_b_a;
        vector<Vector3d> oldT_b_a;
        Matrix3d oldKF_r;
        Vector3d oldKF_t;
        old_kf->getOriginPose(oldKF_t, oldKF_r);
        int temp_index=0;
        //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
        for(KeyFrame* pkFi_old: vpCovKFi_old){
            if(temp_index!=0){
                Matrix3d rwi_old;
                Vector3d twi_old;
                pkFi_old->getOriginPose(twi_old, rwi_old);
                
                Matrix3d r_b_a;
                Vector3d t_b_a;
                r_b_a=oldKF_r.transpose()* rwi_old;
                t_b_a=oldKF_r.transpose()*(twi_old-oldKF_t);
                oldR_b_a.push_back(r_b_a);
                oldT_b_a.push_back(t_b_a);
            }else{
                oldR_b_a.push_back(Matrix3d::Identity());
                oldT_b_a.push_back(Vector3d::Zero());
            }
            temp_index++;
        }
        //得到当前帧 与 侯选帧和共视帧 之间匹配关系

        point_clouds_all.clear();
        measurements_old_norm_all.clear();
        real_vpCovKFi_cur.clear();
        measurements_old_norm_real.clear();
        point_3d_cur_real.clear();

        measurements_old_coarse.clear();
        measurements_old_norm_coarse.clear();
        cur_kf->findConnectionWithOldFrame_server_old(old_kf, measurements_old_coarse, measurements_old_norm_coarse);
        if(measurements_old_coarse.size()<22){
//            cout<<"当前帧与侯选帧的匹配点数少于22，"<<measurements_old_coarse.size()<<endl;
            return  false;
        }
//        cout<<"当前帧与侯选帧匹配点数："<<measurements_old_norm_coarse.size()<<endl;
        cout<<endl<<"测试 可能的匹配："<<setprecision(19)<<old_kf->global_index<<" "<<old_kf->header<<" "<<cur_kf->global_index<<" "<<cur_kf->header<<endl;
        measurements_old_norm_real=measurements_old_norm_coarse;
        point_3d_cur_real=cur_kf->point_clouds;
        features_id_cur_real=cur_kf->features_id;
        point_clouds_all.push_back(point_3d_cur_real);
        measurements_old_norm_all.push_back(measurements_old_norm_real);
        real_vpCovKFi_cur.push_back(old_kf);
//        cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false); //这里其实没有用到了
        //优化 得到当前帧位姿误差下 到侯选帧的位姿关系
//        start=clock();
        {
            ceres::Problem problem;
            ceres::Solver::Options options;
            //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            //options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 20;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
//            loss_function = new ceres::CauchyLoss(1.0);

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

            int i=0;
//            for(KeyFrame* pkf_old: real_vpCovKFi_cur){
                Matrix3d relative_r_b_a=oldR_b_a[i];
                Vector3d relative_t_b_a=oldT_b_a[i];
                Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                vector<Vector3d> point_single=point_clouds_all[i];
                vector<cv::Point2f> measure_single=measurements_old_norm_all[i];
                for(int a=0,b=point_single.size();a<b;a++){
                    //找到主地图那个点 所在帧的位姿
                    Vector3d pts_i = point_single[a];

                    //相机平面坐标
                    cv::Point2f pt=measure_single[a];
                    float xx=pt.x;
                    float yy=pt.y;

                     Vector2d pts_j ;//要求是归一化图像坐标
                     pts_j<<xx,yy;

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(), relative_t_b_a(0), relative_t_b_a(1), relative_t_b_a(2), relative_r_b_a_euler(0), relative_r_b_a_euler(1), relative_r_b_a_euler(2),100.0);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
                 }
//                i++;
//            }

            ceres::Solve(options, &problem, &summary);
//            std::cout <<"地图内部 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";

            if(summary.termination_type!=ceres::
               CONVERGENCE){
                return false;
            }

            T_relative[0]=t_array[0];
            T_relative[1]=t_array[1];
            T_relative[2]=t_array[2];

            R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));

        }
//        ends=clock();
//        cout<<"计算粗糙位姿花费的时间:"<<ends-start<<endl;
        //记录当前帧的共视帧 能够和候选帧的共视帧匹配上的数量
        int similarNum=0;
        //找5帧，加上自己，总共6帧，有3帧以上就行
        std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
        vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
        //如果这里报错，说明没有共视帧，那么记得加上共视帧阈值设大了的，补丁 加了加了
        vpCovKFi_cur[0] = cur_kf;
        {
            
            int i=0;
            for(KeyFrame* pkf_old:vpCovKFi_old){
                Matrix3d r_w_imuOld=R_relative*oldR_b_a[i];
                Vector3d t_w_imuOld=R_relative*oldT_b_a[i] +T_relative;
                Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                Matrix3d r_camOld_w=r_w_camOld.transpose();
                old_r.push_back(r_camOld_w);
                old_t.push_back(-r_camOld_w*t_w_camOld);
                i++;
                
            }
            //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
            
            point_clouds_all.clear();
            measurements_old_norm_all.clear();
            real_vpCovKFi_cur.clear();
            for(KeyFrame* pKFi : vpCovKFi_cur){
                //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
                if(pKFi==cur_kf){
                    point_clouds_all.push_back(point_3d_cur_real);
                    measurements_old_norm_all.push_back(measurements_old_norm_real);
                    real_vpCovKFi_cur.push_back(old_kf);
                    
                    continue;
                }
                //描述符得全有
                if(!pKFi->is_des_end){
                    continue;
                }
                
                vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                int nPoints = point_clouds_origin_cur.size();
                int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
               
                int num=0;//记录匹配点的数量
                int temp_kf_index=0;//记录遍历到哪个帧了
                //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                for(KeyFrame* pkFi_old: vpCovKFi_old){
//                    start=clock();
                
                    //描述符得全有
                    if(!pkFi_old->is_des_end){
                        temp_kf_index++;
                        continue;
                    }
                    
                    measurements_old_coarse.clear();
                    measurements_old_norm_coarse.clear();
                    point_3d_cur.clear();
                    measurements_cur.clear();
                    
//                    featureId_single.clear();

                    vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                    //这个得到的是到imu坐标系的位姿
                    Matrix3d r_camOld_w=old_r[temp_kf_index];
                    Vector3d t_camOld_w=old_t[temp_kf_index];
                    
                    
                    
                    for(int i=0;i<nPoints;i++){
                        Vector3d point_main=point_clouds_origin_cur[i];
                        //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                        Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                        //深度必须为正
                        if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
//                            cout<<"深度不为正"<<endl;
                            continue;
                        }
                        // 投影到图像上
                        double x = p3D_c2[0];
                        double y = p3D_c2[1];
                        double z = p3D_c2[2];
                        //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                        double u=fx*x/z+cx;
                        double v=fy*y/z+cy;
                      
                        
                        if(!pkFi_old->isInImage((float)u, (float)v)){
//                            cout<<"投影点不在图像内"<<endl;
                            continue;
                        }
                        
                        const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 60);
                        //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                        if(vIndices.empty()){
//                            cout<<"半径为50个像素 找不到点"<<endl;
                            continue;
                        }
                        //des和keypoints长度不一样
                        //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                        BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                        int bestDist = 256;
                        int bestIndex = -1;
                        for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                        {
        //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                            int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                            if(dis < bestDist)
                            {
                                bestDist = dis;
                                bestIndex = *vit;
                            }
                        }
                        
                        if(bestDist<=60){
                            point_3d_cur.push_back(point_main);
                            measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                            measurements_cur.push_back(measurements_origin_cur[i]);
                            
//                            featureId_single.push_back(featureId_cur[i]);
                        }
                                      
                    }
                    
                    
//                    cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                    if(measurements_cur.size()>=15){
                        pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
//                        cout<<"f矩阵拒绝后的点数："<<measurements_cur.size()<<", "<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<endl;
                        if(measurements_old_coarse.size()>=15){
                        
                            cv::Point2f norm_pt;
                            int pCount=measurements_old_coarse.size();
                            for(int aa=0;aa<pCount;aa++){
                                norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                measurements_old_coarse[aa]=norm_pt;
                                
                            }
                            
                            point_clouds_all.push_back(point_3d_cur);
                            measurements_old_norm_all.push_back(measurements_old_coarse);
                            num+=point_3d_cur.size();
                            real_vpCovKFi_cur.push_back(pkFi_old);
                            similarNum++;
                            
                            
                            
                           
                            break;
                        }
                        
                        
                    }
                    
                    temp_kf_index++;
//                    ends=clock();
//                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                }
                
                
            }
                
                
        }
        
      
            
        if(similarNum>=2){
            int optiKf_num=real_vpCovKFi_cur.size()+1;
            //构造优化问题
            double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
            Quaterniond q_array[optiKf_num];
            double euler_array[optiKf_num][3];

            ceres::Problem problem;
            ceres::Solver::Options options;
            //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            //options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 20;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
//            ceres::LossFunction *loss_function_feature;
//            loss_function_feature = new ceres::CauchyLoss(1.0);
            //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
            ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

            int i=0;
            //这里应该可以只一个优化项，在当前帧位姿误差下，世界坐标系到老帧的位姿
            //把第一项给老帧 匹配帧

            Matrix3d tmp_r_old;
            Vector3d tmp_t_old;
            t_array[i][0] = T_relative(0);
            t_array[i][1] = T_relative(1);
            t_array[i][2] = T_relative(2);
            //将矩阵转换为向量
            Vector3d euler_angle_old = Utility::R2ypr(R_relative);
            euler_array[i][0] = euler_angle_old.x();
            euler_array[i][1] = euler_angle_old.y();
            euler_array[i][2] = euler_angle_old.z();
            problem.AddParameterBlock(euler_array[i], 3);
            problem.AddParameterBlock(t_array[i], 3);

//            cout<<"计算得到的相对位姿："<<t_array[i][0]<<" ,"<<t_array[i][1]<<" ,"<<t_array[i][2]<<" ,"<<euler_array[i][0]<<" ,"<<euler_array[i][1]<<" ,"<<euler_array[i][2]<<endl;

            
            i++;
            
            //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
             
            int old_kf_index=0;
            for(KeyFrame* pKFi : real_vpCovKFi_cur){

                Matrix3d relative_r_b_a;
                Vector3d relative_t_b_a;


                //找一下 对应的应该是哪个相对位姿
                int relative_index=0;
                for(KeyFrame* pkFi_old: vpCovKFi_old){
                    if(pkFi_old==pKFi){
                        relative_r_b_a=oldR_b_a[relative_index];
                        relative_t_b_a=oldT_b_a[relative_index];
                        break;
                    }
                    relative_index++;
                }
                Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                
                
                t_array[i][0] = relative_t_b_a(0);
                t_array[i][1] = relative_t_b_a(1);
                t_array[i][2] = relative_t_b_a(2);
                euler_array[i][0] = relative_r_b_a_euler.x();
                euler_array[i][1] = relative_r_b_a_euler.y();
                euler_array[i][2] = relative_r_b_a_euler.z();
                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
                ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),1);
                problem.AddResidualBlock(cost_function, loss_function, euler_array[i],t_array[i]);
                

                vector<Vector3d> point_single=point_clouds_all[old_kf_index];
                vector<cv::Point2f> measure_single=measurements_old_norm_all[old_kf_index];
                for(int a=0,b=point_single.size();a<b;a++){

                    //找到主地图那个点 所在帧的位姿
                    Vector3d pts_i = point_single[a];

                    //相机平面坐标
                    cv::Point2f pt=measure_single[a];
                    float xx=pt.x;
                    float yy=pt.y;

                     Vector2d pts_j ;//要求是归一化图像坐标
                     pts_j<<xx,yy;

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[i],t_array[i]);
                    
                    
                 }



                old_kf_index++;

                i++;

            }
             
            
            ceres::Solve(options, &problem, &summary);
            std::cout <<"精确相对位姿："<< summary.BriefReport() << "\n";

            if(summary.termination_type==ceres::CONVERGENCE){
//            {
                cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;

                Matrix3d Rs_i ;
                Vector3d Ps_i ;//当前帧
                cur_kf->getOriginPose(Ps_i, Rs_i);
//                Vector3d q_i;
//                q_i<<euler_array[1][0],euler_array[1][1],euler_array[1][2];
//                Matrix3d Rs_i = Utility::ypr2R(q_i);
//                Vector3d Ps_i = Vector3d( t_array[1][0],  t_array[1][1],  t_array[1][2]);


                Vector3d q;
                q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                Matrix3d Rs_loop = Utility::ypr2R(q);
                Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
                
//                cout<<"优化计算得到的相对位姿："<<Ps_loop[0]<<" ,"<<Ps_loop[1]<<" ,"<<Ps_loop[2]<<" ,"<<euler_array[0][0]<<" ,"<<euler_array[0][1]<<" ,"<<euler_array[0][2]<<endl;

                Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                cur_kf->relative_pitch=relative_pitch;
                cur_kf->relative_roll=relative_roll;
                cur_kf->updateLoopConnection(relative_t, relative_yaw);


                //先临时搬过来
                old_index=old_kf->global_index;
                Vector3d T_w_i_old;
                Matrix3d R_w_i_old;
                old_kf->getPose(T_w_i_old, R_w_i_old);

                Quaterniond Q_loop_old(R_w_i_old);
                RetriveData retrive_data;
                retrive_data.cur_index = cur_kf->global_index;
                retrive_data.header = cur_kf->header;
                retrive_data.P_old = T_w_i_old;
                retrive_data.Q_old = Q_loop_old;
                retrive_data.use = true;
                retrive_data.measurements = measurements_old_norm_real;
                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
                retrive_data.old_index=old_kf->global_index;
                vins->retrive_pose_data = (retrive_data);
//                        vins->isSendLoopData=true;

                cur_kf->detectLoop(old_kf->global_index);
                poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                old_kf->is_looped = 1;
                loop_old_index = old_kf->global_index;
                vins->isSendLoop_another=true;
                cur_kf->is_get_loop_info=true;
                return true;

            }
//            else{
//                cout<<"检测失败： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
//                return false;
//            }
        }
        else{
            cout<<"匹配帧数不够多="<<similarNum<<endl;
        }
            
            
               
    }
            //这个侯选帧， 和当前帧 以及当前帧的共视帧做匹配
            
            //可能是bow分数 ，或者几何一致性匹配？？？ 后面考虑
            
//        }
      
      return false;
  }
  catch(const std::string &ex)
  {
    cout << "Error loop: " << ex << endl;
    return false;
  }
}

//这里是将7拆成9和loop_closurerun_4
bool LoopClosure::startLoopClosure_9(KeyFrame* cur_kf)
{
  try
  {

      int old_index;
//      cout<<"测试是不是每一帧都参与回环检测？"<<cur_kf->global_index<<endl;
     
      //可以先只做一个简单的判断，不要检测时间一致性
      //这里返回的是分数超过阈值的
      vector<cv::Point2f> cur_pts;
      vector<cv::Point2f> old_pts;
      std::vector<cv::KeyPoint> keys=cur_kf->keypoints;
      std::vector<BRIEF::bitset> descriptors=cur_kf->descriptors;
      int best_id=demo.run_3("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, cur_kf->m_bowvec, cur_kf->m_featvec);
      cur_kf->check_loop=true;
      treeId_kf[kfNum_tree]= cur_kf->global_index;
      kfNum_tree++;
      //描述符得全有
      if(!cur_kf->is_des_end){
//          cout<<"描述符不齐全："<<cur_kf->global_index<<endl;
          return false;
      }

      
      //检测成功后，检测共视帧的一致性
      //选出分数最高的3帧作为侯选帧，并计算一级共视帧的分数（最多选5个），求和，根据这个找到匹配性最高的侯选帧
    if(best_id!=-1){
        //这里直接加这么多也是有问题的 我想想 改好了
        int real_old_best_id=treeId_kf[best_id];
        KeyFrame* old_kf;
        old_kf= poseGraph->getKeyframe(real_old_best_id);
       

        
        //这个是不是可以用两帧 计算匹配点，计算基础矩阵 最终的特征点数量判断
        std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
        std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标

        
        //存储一下当前帧和候选帧的匹配关系
        std::vector<cv::Point2f> measurements_old_norm_real;//图像坐标
        std::vector<Vector3d> point_3d_cur_real;
      
        measurements_old_norm_real.clear();
        point_3d_cur_real.clear();

        measurements_old_coarse.clear();
        measurements_old_norm_coarse.clear();
        cur_kf->findConnectionWithOldFrame_server_old(old_kf, measurements_old_coarse, measurements_old_norm_coarse);
        if(measurements_old_coarse.size()<16){
//            cout<<"地图内部 当前帧与侯选帧的匹配点数少于22，"<<measurements_old_coarse.size()<<", "<<cur_kf->global_index<<endl;
            return  false;
        }
//        cout<<"当前帧与侯选帧匹配点数："<<measurements_old_norm_coarse.size()<<endl;
//        cout<<endl<<"测试 可能的匹配："<<setprecision(19)<<old_kf->global_index<<" "<<old_kf->header<<" "<<cur_kf->global_index<<" "<<cur_kf->header<<" "<<measurements_old_norm_coarse.size()<<endl;
        
        measurements_old_norm_real=measurements_old_norm_coarse;
        point_3d_cur_real=cur_kf->point_clouds;

//        cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false); //这里其实没有用到了
        //优化 得到当前帧位姿误差下 到侯选帧的位姿关系
//        start=clock();
        
//        ends=clock();
//        cout<<"计算粗糙位姿花费的时间:"<<ends-start<<endl;
        //记录当前帧的共视帧 能够和候选帧的共视帧匹配上的数量
        
//        cout<<"地图内部 第一次粗糙找到的匹配点数:"<<point_3d_cur_real.size()<<endl;
        relative_mutex.lock();
        mlvv_point_clouds_single.push_back(point_3d_cur_real);
        mlvv_measurements_old_norm_single.push_back(measurements_old_norm_real);
        mlp_kf_pairs.push_back(std::make_pair(cur_kf, old_kf));
//        //shiyan
//        list<vector<cv::Point2f> > mlvv_measurements_old_single;
//        list<vector<cv::Point2f> > mlvv_measurements_cur_single;
//        list<vector<int> > mlvv_featureId_old_single;
//        list<vector<int> > mlvv_featureId_cur_single;
        relative_mutex.unlock();
        
      
    }      
      return false;
  }
  catch(const std::string &ex)
  {
    cout << "Error loop: " << ex << endl;
    return false;
  }
}

bool LoopClosure::startLoopClosure_subMap(KeyFrame* cur_kf)
{
    try
    {

        int old_index;
  //      cout<<"测试是不是每一帧都参与回环检测？"<<cur_kf->global_index<<endl;
       
        //可以先只做一个简单的判断，不要检测时间一致性
        //这里返回的是分数超过阈值的
        vector<cv::Point2f> cur_pts;
        vector<cv::Point2f> old_pts;
        std::vector<cv::KeyPoint> keys=cur_kf->keypoints;
        std::vector<BRIEF::bitset> descriptors=cur_kf->descriptors;
        vector<int> old_vector;
        bool best_id=demo.run_4("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, old_vector, cur_kf->m_bowvec, cur_kf->m_featvec);
      
//        cout<<"当前帧："<<cur_kf->global_index<<" , "<<cur_kf<<endl;
        
        
        cur_kf->check_loop=true;
        treeId_kf[kfNum_tree]= cur_kf->global_index;
//记录
        kfNum_tree++;
        bool recordIsLoop=true;//记录是否变成false 最终返回recordIsLoop
        int old_kf_index=-1;
        //描述符得全有
//        if(!cur_kf->is_des_end){
//  //          cout<<"描述符不齐全："<<cur_kf->global_index<<endl;
////            这里不可能进来
//            cout<<"这里不可能进来"<<endl;
//            recordIsLoop= false;
//        }else{
            if(best_id){
                //这里直接加这么多也是有问题的 我想想 改好了
                int real_old_best_id=treeId_kf[old_index];
                KeyFrame* old_kf;
                old_kf= poseGraph->getKeyframe(real_old_best_id);
               

                
                //这个是不是可以用两帧 计算匹配点，计算基础矩阵 最终的特征点数量判断
                std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
                std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标

                
                //存储一下当前帧和候选帧的匹配关系
                std::vector<cv::Point2f> measurements_old_norm_real;//图像坐标
                std::vector<Vector3d> point_3d_cur_real;
              
                measurements_old_norm_real.clear();
                point_3d_cur_real.clear();

                measurements_old_coarse.clear();
                measurements_old_norm_coarse.clear();
                cur_kf->findConnectionWithOldFrame_server_old(old_kf, measurements_old_coarse, measurements_old_norm_coarse);
                if(measurements_old_coarse.size()<16){
        //            cout<<"地图内部 当前帧与侯选帧的匹配点数少于22，"<<measurements_old_coarse.size()<<", "<<cur_kf->global_index<<endl;
                    recordIsLoop=  false;
                }else{
            //        cout<<"当前帧与侯选帧匹配点数："<<measurements_old_norm_coarse.size()<<endl;
            //        cout<<endl<<"测试 可能的匹配："<<setprecision(19)<<old_kf->global_index<<" "<<old_kf->header<<" "<<cur_kf->global_index<<" "<<cur_kf->header<<" "<<measurements_old_norm_coarse.size()<<endl;
                    
                    measurements_old_norm_real=measurements_old_norm_coarse;
                    point_3d_cur_real=cur_kf->point_clouds;

            //        cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false); //这里其实没有用到了
                    //优化 得到当前帧位姿误差下 到侯选帧的位姿关系
            //        start=clock();
                    
            //        ends=clock();
            //        cout<<"计算粗糙位姿花费的时间:"<<ends-start<<endl;
                    //记录当前帧的共视帧 能够和候选帧的共视帧匹配上的数量
                    
            //        cout<<"地图内部 第一次粗糙找到的匹配点数:"<<point_3d_cur_real.size()<<endl;
                    relative_mutex.lock();
                    mlvv_point_clouds_single.push_back(point_3d_cur_real);
                    mlvv_measurements_old_norm_single.push_back(measurements_old_norm_real);
                    mlp_kf_pairs.push_back(std::make_pair(cur_kf, old_kf));
            //        //shiyan
            //        list<vector<cv::Point2f> > mlvv_measurements_old_single;
            //        list<vector<cv::Point2f> > mlvv_measurements_cur_single;
            //        list<vector<int> > mlvv_featureId_old_single;
            //        list<vector<int> > mlvv_featureId_cur_single;
                    old_kf_index=old_kf->global_index;
                    relative_mutex.unlock();
                }
                
               
            }
            //                    子地图用
//            vector<int> old_vector_realKdId;
//            for(int i=0,j=old_vector.size();i<j;i++){
//                int real_oldKf_id=treeId_kf[old_vector[i]];
//                old_vector_realKdId.push_back(real_oldKf_id);
//            }
            
//            old_vector_forSubMap.push_back(old_vector);
            
            
            //        ----------子地图分配 调用allocateSubMap----------
//            relative_mutex.lock();
//            vector<int> old_vector=old_vector_forSubMap.front();
//            old_vector_forSubMap.pop_front();
//            relative_mutex.unlock();
//            global_featureMap->allocateSubMap(cur_kf, old_kf_index,kfNum_tree-1,old_vector);
            
            
            
            //        ----------
            
//        }
        
//        if(best_id){
//            cout<<"allocateSubMap="<<treeId_kf[old_index]<<" , "<<old_index<<" - ";
//        }
//        for(int i=0,j=old_vector.size();i<j;i++){
//            cout<<treeId_kf[old_vector[i]]<<" , "<<old_vector[i]<<" , ";//这样old_vector变成帧的id了
//        }
//        cout<<endl;
//        global_featureMap->allocateSubMap(cur_kf, old_index,kfNum_tree-1,old_vector);
        
        //检测成功后，检测共视帧的一致性
        //选出分数最高的3帧作为侯选帧，并计算一级共视帧的分数（最多选5个），求和，根据这个找到匹配性最高的侯选帧
      
        

        return recordIsLoop;
    }
    catch(const std::string &ex)
    {
      cout << "Error loop: " << ex << endl;
      return false;
    }
}

bool LoopClosure::startLoopClosure_11(KeyFrame* cur_kf)
{
  try
  {
      bool loop_succ = false;
      vector<cv::Point2f> cur_pts;
      vector<cv::Point2f> old_pts;
      loop_succ = demo.run_2("BRIEF", cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index, cur_kf->m_bowvec, cur_kf->m_featvec);
      cur_kf->check_loop=true;
      if(loop_succ){
          KeyFrame* old_kf = poseGraph->getKeyframe(old_index);//拿到回环帧
          if (old_kf == NULL)
          {
              printf("NO such %dth frame in keyframe_database\n", old_index);
              assert(false);
          }

//      int old_index;
////      cout<<"测试是不是每一帧都参与回环检测？"<<cur_kf->global_index<<endl;
//
//      //可以先只做一个简单的判断，不要检测时间一致性
//      //这里返回的是分数超过阈值的
//      vector<cv::Point2f> cur_pts;
//      vector<cv::Point2f> old_pts;
//      std::vector<cv::KeyPoint> keys=cur_kf->keypoints;
//      std::vector<BRIEF::bitset> descriptors=cur_kf->descriptors;
//      int best_id=demo.run_3("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, cur_kf->m_bowvec, cur_kf->m_featvec);
//      cur_kf->check_loop=true;
//      treeId_kf[kfNum_tree]= cur_kf->global_index;
//      kfNum_tree++;
//      //描述符得全有
//      if(!cur_kf->is_des_end){
//          cout<<"描述符不齐全："<<cur_kf->global_index<<endl;
//          return false;
//      }
//
//
//      //检测成功后，检测共视帧的一致性
//      //选出分数最高的3帧作为侯选帧，并计算一级共视帧的分数（最多选5个），求和，根据这个找到匹配性最高的侯选帧
//    if(best_id!=-1){
//        //这里直接加这么多也是有问题的 我想想 改好了
//        int real_old_best_id=treeId_kf[best_id];
//        KeyFrame* old_kf;
//        old_kf= poseGraph->getKeyframe(real_old_best_id);
       

        
        //这个是不是可以用两帧 计算匹配点，计算基础矩阵 最终的特征点数量判断
        std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
        std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标

        
        //存储一下当前帧和候选帧的匹配关系
        std::vector<cv::Point2f> measurements_old_norm_real;//图像坐标
        std::vector<Vector3d> point_3d_cur_real;
      
        measurements_old_norm_real.clear();
        point_3d_cur_real.clear();

        measurements_old_coarse.clear();
        measurements_old_norm_coarse.clear();
        cur_kf->findConnectionWithOldFrame_server_old(old_kf, measurements_old_coarse, measurements_old_norm_coarse);
        if(measurements_old_coarse.size()<22){

            return  false;
        }
        cout<<endl<<"测试 可能的匹配："<<setprecision(19)<<old_kf->global_index<<" "<<old_kf->header<<" "<<cur_kf->global_index<<" "<<cur_kf->header<<" "<<measurements_old_norm_coarse.size()<<endl;
        
        measurements_old_norm_real=measurements_old_norm_coarse;
        point_3d_cur_real=cur_kf->point_clouds;

        
        relative_mutex.lock();
        mlvv_point_clouds_single.push_back(point_3d_cur_real);
        mlvv_measurements_old_norm_single.push_back(measurements_old_norm_real);
        mlp_kf_pairs.push_back(std::make_pair(cur_kf, old_kf));
        relative_mutex.unlock();
        
      
    }
      return false;
  }
  catch(const std::string &ex)
  {
    cout << "Error loop: " << ex << endl;
    return false;
  }
}

//9的基础上，把新帧和老帧角色调换一下，因为老帧是稳定的
bool LoopClosure::startLoopClosure_10(KeyFrame* cur_kf)
{
  try
  {

      int old_index;
      cout<<"测试是不是每一帧都参与回环检测？"<<cur_kf->global_index<<endl;
     
      //可以先只做一个简单的判断，不要检测时间一致性
      //这里返回的是分数超过阈值的
      vector<cv::Point2f> cur_pts;
      vector<cv::Point2f> old_pts;
      std::vector<cv::KeyPoint> keys=cur_kf->keypoints;
      std::vector<BRIEF::bitset> descriptors=cur_kf->descriptors;
      int best_id=demo.run_3("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, cur_kf->m_bowvec, cur_kf->m_featvec);
      cur_kf->check_loop=true;
      treeId_kf[kfNum_tree]= cur_kf->global_index;
      kfNum_tree++;
      //描述符得全有
      if(!cur_kf->is_des_end){
          cout<<"描述符不齐全："<<cur_kf->global_index<<endl;
          return false;
      }

      
      //检测成功后，检测共视帧的一致性
      //选出分数最高的3帧作为侯选帧，并计算一级共视帧的分数（最多选5个），求和，根据这个找到匹配性最高的侯选帧
    if(best_id!=-1){
        //这里直接加这么多也是有问题的 我想想 改好了
        int real_old_best_id=treeId_kf[best_id];
        KeyFrame* old_kf;
        old_kf= poseGraph->getKeyframe(real_old_best_id);
        
        
        //这个是不是可以用两帧 计算匹配点，计算基础矩阵 最终的特征点数量判断
        std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
        std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标

        
        //存储一下当前帧和候选帧的匹配关系
        std::vector<cv::Point2f> measurements_old_norm_real;//图像坐标
        std::vector<Vector3d> point_3d_cur_real;
      
        measurements_old_norm_real.clear();
        point_3d_cur_real.clear();

        measurements_old_coarse.clear();
        measurements_old_norm_coarse.clear();
        //这里会得到老帧的3d点， 新帧的2d点
        old_kf->findConnectionWithOldFrame_server_old(cur_kf, measurements_old_coarse, measurements_old_norm_coarse);
        if(measurements_old_coarse.size()<22){
//            cout<<"当前帧与侯选帧的匹配点数少于22，"<<measurements_old_coarse.size()<<", "<<cur_kf->global_index<<endl;
            return  false;
        }
//        cout<<"当前帧与侯选帧匹配点数："<<measurements_old_norm_coarse.size()<<endl;
//        cout<<endl<<"测试 可能的匹配："<<setprecision(19)<<old_kf->global_index<<" "<<old_kf->header<<" "<<cur_kf->global_index<<" "<<cur_kf->header<<endl;
        
        measurements_old_norm_real=measurements_old_norm_coarse;
        point_3d_cur_real=old_kf->point_clouds;

        //优化 得到当前帧位姿误差下 到侯选帧的位姿关系
//        start=clock();
        
//        ends=clock();
//        cout<<"计算粗糙位姿花费的时间:"<<ends-start<<endl;
        //记录当前帧的共视帧 能够和候选帧的共视帧匹配上的数量
        
        
        relative_mutex.lock();
        mlvv_point_clouds_single.push_back(point_3d_cur_real);
        mlvv_measurements_old_norm_single.push_back(measurements_old_norm_real);
        mlp_kf_pairs.push_back(std::make_pair(cur_kf, old_kf));
        relative_mutex.unlock();
        
      
    }
      return false;
  }
  catch(const std::string &ex)
  {
    cout << "Error loop: " << ex << endl;
    return false;
  }
}

void LoopClosure::eraseIndex(std::vector<int> &erase_index)
{
    demo.eraseIndex(erase_index);
}



string getTime()
{
    time_t timep;
    time (&timep); //获取time_t类型的当前时间
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );//对日期和时间进行格式化
    return tmp;
}


void LoopClosure::loopClosureRun(){
    while(true){
        //已加 再加个判断 当前帧是否已经做过回环了
        bool loop_succ=false;
        KeyFrame* cur_kf = poseGraph->getLastKeyframe();
        //检查一下 kf的keypoints和des是不是相等
        
        
        if(cur_kf!=nullptr&&cur_kf->is_des_end){
            assert(cur_kf->keypoints.size()==cur_kf->descriptors.size());
            if(cur_kf->check_loop==false){//不确定合理不，如果一个帧 发生了几次回环呢？ 合理
                

                vector<cv::Point2f> cur_pts;
                vector<cv::Point2f> old_pts;
                
                loop_succ = startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);//每一次进来 都保存过之前的帧
                cur_kf->check_loop=true;
                if(loop_succ)
                {
                    KeyFrame* old_kf = poseGraph->getKeyframe(old_index);//拿到回环帧
                    if (old_kf == NULL)
                    {
                        printf("NO such %dth frame in keyframe_database\n", old_index);
                        assert(false);
                    }
//                    printf("loop succ with %drd image\n", old_index);
                    assert(old_index!=-1);
                    
                    Vector3d T_w_i_old;
                    Matrix3d R_w_i_old;
                    
                    
                    std::vector<cv::Point2f> measurements_old;
                    std::vector<cv::Point2f> measurements_old_norm;
                    std::vector<cv::Point2f> measurements_cur;
                    std::vector<int> features_id;
                    
                    old_kf->getPose(T_w_i_old, R_w_i_old);
                    cur_kf->findConnectionWithOldFrame(old_kf, cur_pts, old_pts,
                                                       measurements_old, measurements_old_norm);//找到匹配的特征点
                    
                    //发送给客户端
//                    if(cur_kf->sendRejectWithF){
//                        cur_kf->sendRejectWithF=false;
//                        vins->send_status.push(cur_kf->send_status);
//                        vins->send_status_index.push(cur_kf->global_index);
//
//                        cur_kf->send_status.clear();
//                     }
                    
                    
                    measurements_cur = cur_kf->measurements;//还没值
                    features_id = cur_kf->features_id;
                    
                    
                    if(measurements_old_norm.size()>MIN_LOOP_NUM)
                    {
                        
                        Quaterniond Q_loop_old(R_w_i_old);
                        RetriveData retrive_data;
                        retrive_data.cur_index = cur_kf->global_index;
                        retrive_data.header = cur_kf->header;
                        retrive_data.P_old = T_w_i_old;
                        retrive_data.Q_old = Q_loop_old;
                        retrive_data.use = true;
                        retrive_data.measurements = measurements_old_norm;
                        retrive_data.features_ids = features_id;
                        retrive_data.old_index=old_index;
                        vins->retrive_pose_data = (retrive_data);
                        vins->isSendLoopData=true;
                        
                        cur_kf->detectLoop(old_index);
                        poseGraph->addLoop(old_index);
                        old_kf->is_looped = 1;
                        loop_old_index = old_index;
                        
                        cout<<"检测到回环："<<getTime()<<endl;
                    }
                }
            }
        }
        if(loop_succ){
            usleep(2000);
        }
        usleep(30);
    }

}

void LoopClosure::loopClosureRun_2(){
    while(true){
        //已加 再加个判断 当前帧是否已经做过回环了
        bool loop_succ=false;
        KeyFrame* cur_kf = poseGraph->getLastKeyframe();
        //检查一下 kf的keypoints和des是不是相等
        
        
        if(cur_kf!=nullptr&&cur_kf->is_des_end){
            assert(cur_kf->keypoints.size()==cur_kf->descriptors.size());
            if(cur_kf->check_loop==false){//不确定合理不，如果一个帧 发生了几次回环呢？ 合理
                

                vector<cv::Point2f> cur_pts;
                vector<cv::Point2f> old_pts;
                
//                loop_succ = startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);//每一次进来 都保存过之前的帧
                
                loop_succ = startLoopClosure_2(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index,cur_kf->m_bowvec, cur_kf->m_featvec);//每一次进来 都保存过之前的帧
                cout<<"测试是不是每一帧都参与回环检测？"<<cur_kf->global_index<<endl;
                
                cur_kf->check_loop=true;
                if(loop_succ)
                {
                    KeyFrame* old_kf = poseGraph->getKeyframe(old_index);//拿到回环帧
                    if (old_kf == NULL)
                    {
                        printf("NO such %dth frame in keyframe_database\n", old_index);
                        assert(false);
                    }
//                    printf("loop succ with %drd image\n", old_index);
                    assert(old_index!=-1);
                    
                    Vector3d T_w_i_old;
                    Matrix3d R_w_i_old;
                    
                    
                    std::vector<cv::Point2f> measurements_old;
                    std::vector<cv::Point2f> measurements_old_norm;
                    std::vector<cv::Point2f> measurements_cur;
                    std::vector<int> features_id;
                    
                    old_kf->getPose(T_w_i_old, R_w_i_old);
                    cur_kf->findConnectionWithOldFrame(old_kf, cur_pts, old_pts,
                                                       measurements_old, measurements_old_norm);//找到匹配的特征点
                    
                    //发送给客户端
//                    if(cur_kf->sendRejectWithF){
//                        cur_kf->sendRejectWithF=false;
//                        vins->send_status.push(cur_kf->send_status);
//                        vins->send_status_index.push(cur_kf->global_index);
//
//                        cur_kf->send_status.clear();
//                     }
                    
                    
                    measurements_cur = cur_kf->measurements;//还没值
                    features_id = cur_kf->features_id;
                    
                    
                    if(measurements_old_norm.size()>MIN_LOOP_NUM)
                    {
                        
                        Quaterniond Q_loop_old(R_w_i_old);
                        RetriveData retrive_data;
                        retrive_data.cur_index = cur_kf->global_index;
                        retrive_data.header = cur_kf->header;
                        retrive_data.P_old = T_w_i_old;
                        retrive_data.Q_old = Q_loop_old;
                        retrive_data.use = true;
                        retrive_data.measurements = measurements_old_norm;
                        retrive_data.features_ids = features_id;
                        retrive_data.old_index=old_index;
                        vins->retrive_pose_data = (retrive_data);
                        vins->isSendLoopData=true;
                        
                        cur_kf->detectLoop(old_index);
                        poseGraph->addLoop(old_index);
                        old_kf->is_looped = 1;
                        loop_old_index = old_index;
                        
                        cout<<"检测到回环："<<getTime()<<" ,old_index="<<old_index<<" ,cur_index="<<cur_kf->global_index<<endl;
                    }else{
                        cout<<"回环检测失败：measurements_old_norm.size()>MIN_LOOP_NUM ?"<<measurements_old_norm.size()<<" "<<MIN_LOOP_NUM<<endl;
                    }
                }
            }
        }
        if(loop_succ){
            usleep(2000);
        }
        usleep(30);
    }

}

//这里暂时还没改
void LoopClosure::loopClosureRun_3(){
    while(true){
        //已加 再加个判断 当前帧是否已经做过回环了
//        bool loop_succ=false;
        KeyFrame* cur_kf = poseGraph->getLastKeyframe();
        //检查一下 kf的keypoints和des是不是相等
        
        
        if(cur_kf!=nullptr&&cur_kf->is_des_end){
            assert(cur_kf->keypoints.size()==cur_kf->descriptors.size());
            if(cur_kf->check_loop==false){//不确定合理不，如果一个帧 发生了几次回环呢？ 合理
                
//                loop_succ = startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);//每一次进来 都保存过之前的帧
//                KeyFrame* old_kf;
//              _________第三篇论文  不用子地图时的回环 start——————————————
//                startLoopClosure_9(cur_kf);//每一次进来 都保存过之前的帧
                bool isLoop=startLoopClosure_12(cur_kf);
                if(!isLoop){
                    poseGraph->hasDetectLoopKfId_mutex.lock();
                    poseGraph->hasDetectLoopKfId=cur_kf->global_index;
//                    cout<<"记录当前检测过的帧id="<<poseGraph->hasDetectLoopKfId<<endl;
                    poseGraph->hasDetectLoopKfId_mutex.unlock();
                }
                
//              _________第三篇论文  不用子地图时的回环 end——————————————
//                loop_succ = startLoopClosure_4(cur_kf,cur_kf->keypoints, cur_kf->descriptors,  &old_kf,cur_kf->m_bowvec, cur_kf->m_featvec);//每一次进来 都保存过之前的帧
//                cur_kf->check_loop=true;
                
//                -------------子地图实验-------start------
//                startLoopClosure_subMap(cur_kf);
                
//                记录哪些帧参与了回环检测
//                {
//                    string kfNum_tree_dir="/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/1/";
//                    std::ofstream outFile;
//                    outFile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
//                    outFile.precision(16);
//                    //ios::binary|
//                    int agentId=cur_kf->c->getId();
//                    kfNum_tree_dir+=to_string(agentId);
//                    outFile.open(kfNum_tree_dir+"_allKF.txt",ios::out|ios::app);
//    //                    cout<<"kfNum_tree_dir="<<kfNum_tree_dir+"intra_loop.txt" <<endl;
//                    outFile<< cur_kf->global_index<<"\n";
//
//                    //关闭文件
//                    outFile.close();
//                }

            }
        }else{
            usleep(10);//论文3实验一是休眠10，但应该更久一点 ,后面尝试的20
        }

        usleep(10);
    }

}
void LoopClosure::setFeatureMap(FeatureMap* _global_featureMap){
    global_featureMap=_global_featureMap;
}


void LoopClosure::loopClosureRun_4(){
    while(true){
       
        if(!mlp_kf_pairs.empty()){
            relative_mutex.lock();
            vector<vector<cv::Point2f>> measurements_old_norm_all=mlvv_measurements_old_norm_all.front();
            mlvv_measurements_old_norm_all.pop_front();
            vector<vector<Eigen::Vector3d>> point_clouds_all=mlvv_point_clouds_all.front();
            mlvv_point_clouds_all.pop_front();
            vector<Matrix3d> oldR_b_a=mlv_oldR_b_a.front();
            mlv_oldR_b_a.pop_front();
            vector<Vector3d> oldT_b_a=mlv_oldT_b_a.front();
            mlv_oldT_b_a.pop_front();
            Matrix3d R_relative=ml_relative_r.front();
            ml_relative_r.pop_front();
            Vector3d T_relative=ml_relative_t.front();
            ml_relative_t.pop_front();
            vector<int> vpkf_index=mlv_vpkf_index.front();
            mlv_vpkf_index.pop_front();
            std::pair<KeyFrame*, KeyFrame*> kf_pair=mlp_kf_pairs.front();
            mlp_kf_pairs.pop_front();
            KeyFrame* cur_kf=kf_pair.first;
            KeyFrame* old_kf=kf_pair.second;
            relative_mutex.unlock();
            
            Client* client_cur=old_kf->c;
            Matrix3d ric_curClient=client_cur->ric_client;
            Vector3d tic_curClient=client_cur->tic_client;
            Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
            const float &fx = client_cur->FOCUS_LENGTH_X_server;
            const float &fy = client_cur->FOCUS_LENGTH_Y_server;
            const float &cx = client_cur->PX_server;
            const float &cy = client_cur->PY_server;
            
        
            int optiKf_num=vpkf_index.size()+1;
            //构造优化问题
            double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
            Quaterniond q_array[optiKf_num];
            double euler_array[optiKf_num][3];

            ceres::Problem problem;
            ceres::Solver::Options options;
            //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            //options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 20;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
    //            ceres::LossFunction *loss_function_feature;
    //            loss_function_feature = new ceres::CauchyLoss(1.0);
            //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
            ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

    //            int i=0;
            //这里应该可以只一个优化项，在当前帧位姿误差下，世界坐标系到老帧的位姿
            //把第一项给老帧 匹配帧

            Matrix3d tmp_r_old;
            Vector3d tmp_t_old;
            t_array[0][0] = T_relative(0);
            t_array[0][1] = T_relative(1);
            t_array[0][2] = T_relative(2);
            //将矩阵转换为向量
            Vector3d euler_angle_old = Utility::R2ypr(R_relative);
            euler_array[0][0] = euler_angle_old.x();
            euler_array[0][1] = euler_angle_old.y();
            euler_array[0][2] = euler_angle_old.z();
            problem.AddParameterBlock(euler_array[0], 3);
            problem.AddParameterBlock(t_array[0], 3);

            
            //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
            map<int,int> resample;
            
//            temp_index=0;
            for(int temp_index=0, len=vpkf_index.size(); temp_index<len; temp_index++ ){
                
                int kf_index=temp_index+1;
                Matrix3d relative_r_b_a;
                Vector3d relative_t_b_a;
                relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
                relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
                Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                
                map<int,int>::iterator iter;
                iter = resample.find(vpkf_index[temp_index]);
                if(iter != resample.end())
                {
                    kf_index=resample[vpkf_index[temp_index]];
                }
                else
                {
                       
                    resample[vpkf_index[temp_index]]=kf_index;

                   
                    
                    t_array[kf_index][0] = relative_t_b_a(0);
                    t_array[kf_index][1] = relative_t_b_a(1);
                    t_array[kf_index][2] = relative_t_b_a(2);
                    euler_array[kf_index][0] = relative_r_b_a_euler.x();
                    euler_array[kf_index][1] = relative_r_b_a_euler.y();
                    euler_array[kf_index][2] = relative_r_b_a_euler.z();
                    problem.AddParameterBlock(euler_array[kf_index], 1, angle_local_parameterization);
                    problem.AddParameterBlock(t_array[kf_index], 3);
                    ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),1);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                }

                vector<Vector3d> point_single=point_clouds_all[temp_index];
                vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
                for(int a=0,b=point_single.size();a<b;a++){

                    //找到主地图那个点 所在帧的位姿
                    Vector3d pts_i = point_single[a];

                    //相机平面坐标
                    cv::Point2f pt=measure_single[a];
                    float xx=pt.x;
                    float yy=pt.y;

                     Vector2d pts_j ;//要求是归一化图像坐标
                     pts_j<<xx,yy;

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                    
                    
                 }
            }
             
            
            ceres::Solve(options, &problem, &summary);
            std::cout <<"精确相对位姿："<< summary.BriefReport() << "\n";

            if(summary.termination_type==ceres::CONVERGENCE){
                //重新计算一遍
                //投影
                T_relative<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                R_relative= Utility::ypr2R(T_relative);
                T_relative = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
    
                vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
                vpCovKFi_old.push_back(vpCovKFi_old[0]);
                vpCovKFi_old[0]=old_kf;
                
                std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
                vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
                vpCovKFi_cur[0] = cur_kf;
                
                /**
                vector<Matrix3d> oldR_b_a_opti=oldR_b_a;
                vector<Vector3d> oldT_b_a_opti=oldT_b_a;
                for(int i=0,len=vpkf_index.size(); i<len; i++){
                    int j=i+1;
                    Vector3d q;
                    q<<euler_array[j][0],euler_array[j][1],euler_array[j][2];
                    Matrix3d Rs_loop = Utility::ypr2R(q);
                    Vector3d Ps_loop = Vector3d( t_array[j][0],  t_array[j][1],  t_array[j][2]);
                    oldR_b_a_opti[vpkf_index[i]]=Rs_loop;
                    oldT_b_a_opti[vpkf_index[i]]=Ps_loop;
                    
                }*/
                
                
                int similarNum=0;
                int temp_index=0;
                {
                    vector<Matrix3d> old_r;
                    vector<Vector3d> old_t;
                    //这里oldR_b_a可以更新一下
                    for(KeyFrame* pkf_old:vpCovKFi_old){
                        Matrix3d r_w_imuOld=R_relative*oldR_b_a[temp_index];
                        Vector3d t_w_imuOld=R_relative*oldT_b_a[temp_index] +T_relative;
                        Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                        Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                        Matrix3d r_camOld_w=r_w_camOld.transpose();
                        old_r.push_back(r_camOld_w);
                        old_t.push_back(-r_camOld_w*t_w_camOld);
                        temp_index++;
                    }
                    //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
                    
                    std::vector<Vector3d> point_3d_cur_real=point_clouds_all[0];
                    std::vector<cv::Point2f> measurements_old_norm_real=measurements_old_norm_all[0];
                    std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
                    std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
                    std::vector<Vector3d> point_3d_cur;
                    std::vector<cv::Point2f> measurements_cur;//像素坐标
                    
                    point_clouds_all.clear();
                    measurements_old_norm_all.clear();
                    vpkf_index.clear();
                    for(KeyFrame* pKFi : vpCovKFi_cur){
                        //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
                        if(pKFi==cur_kf){
                            point_clouds_all.push_back(point_3d_cur_real);
                            measurements_old_norm_all.push_back(measurements_old_norm_real);
//                            real_vpCovKFi_cur.push_back(old_kf);
                            vpkf_index.push_back(0);
//                            temp_index++;
                            continue;
                        }
                        //描述符得全有
                        if(!pKFi->is_des_end){
                            continue;
                        }
                        
                        vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                        std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                        vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                        int nPoints = point_clouds_origin_cur.size();
                        int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                       
        //                int num=0;//记录匹配点的数量
                        temp_index=0;//记录遍历到哪个帧了
                        //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                        for(KeyFrame* pkFi_old: vpCovKFi_old){
        //                    start=clock();
                            
                            //描述符得全有
                            if(!pkFi_old->is_des_end){
                                temp_index++;
                                continue;
                            }
                            
                            measurements_old_coarse.clear();
                            measurements_old_norm_coarse.clear();
                            point_3d_cur.clear();
                            measurements_cur.clear();
                            

                            vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                            //这个得到的是到imu坐标系的位姿
                            Matrix3d r_camOld_w=old_r[temp_index];
                            Vector3d t_camOld_w=old_t[temp_index];
                            
                            for(int i=0;i<nPoints;i++){
                                Vector3d point_main=point_clouds_origin_cur[i];
                                //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                                Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                                //深度必须为正
                                if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
        //                            cout<<"深度不为正"<<endl;
                                    continue;
                                }
                                // 投影到图像上
                                double x = p3D_c2[0];
                                double y = p3D_c2[1];
                                double z = p3D_c2[2];
                                //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                                double u=fx*x/z+cx;
                                double v=fy*y/z+cy;
                              
                                
                                if(!pkFi_old->isInImage((float)u, (float)v)){
        //                            cout<<"投影点不在图像内"<<endl;
                                    continue;
                                }
                                
                                const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 30);
                                //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                                if(vIndices.empty()){
        //                            cout<<"半径为50个像素 找不到点"<<endl;
                                    continue;
                                }
                                //des和keypoints长度不一样
                                //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                                BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                                int bestDist = 256;
                                int bestIndex = -1;
                                for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                                {
                //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                                    int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                                    if(dis < bestDist)
                                    {
                                        bestDist = dis;
                                        bestIndex = *vit;
                                    }
                                }
                                
                                if(bestDist<=60){
                                    point_3d_cur.push_back(point_main);
                                    measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                                    measurements_cur.push_back(measurements_origin_cur[i]);
                                    
                                }
                                            
                            }
                            
                            
        //                    cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                            if(measurements_cur.size()>=15){
                                pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
        //                        cout<<"f矩阵拒绝后的点数："<<measurements_cur.size()<<", "<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<endl;
                                if(measurements_old_coarse.size()>=15){
                                
                                    cv::Point2f norm_pt;
                                    int pCount=measurements_old_coarse.size();
                                    for(int aa=0;aa<pCount;aa++){
                                        norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                        norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                        measurements_old_coarse[aa]=norm_pt;
                                        
                                    }
                                    
                                    point_clouds_all.push_back(point_3d_cur);
                                    measurements_old_norm_all.push_back(measurements_old_coarse);
        //                            num+=point_3d_cur.size();
//                                    real_vpCovKFi_cur.push_back(pkFi_old);
                                    vpkf_index.push_back(temp_index);
                                    similarNum++;
                                    
                                    break;
                                }
                                
                            }
                            
                            temp_index++;
        //                    ends=clock();
        //                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                        }
                    }
                    
                    
                }
                
                if(similarNum>=2){
                    int optiKf_num=vpkf_index.size()+1;
                    //构造优化问题
                    double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
                    Quaterniond q_array[optiKf_num];
                    double euler_array[optiKf_num][3];

                    ceres::Problem problem;
                    ceres::Solver::Options options;
                    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                    options.linear_solver_type = ceres::DENSE_SCHUR;
                    //options.minimizer_progress_to_stdout = true;
                    options.max_num_iterations = 20;
                    ceres::Solver::Summary summary;
                    ceres::LossFunction *loss_function;
                    loss_function = new ceres::HuberLoss(1.0);
            //            ceres::LossFunction *loss_function_feature;
            //            loss_function_feature = new ceres::CauchyLoss(1.0);
                    //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
                    ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

            //            int i=0;
                    //这里应该可以只一个优化项，在当前帧位姿误差下，世界坐标系到老帧的位姿
                    //把第一项给老帧 匹配帧

                    Matrix3d tmp_r_old;
                    Vector3d tmp_t_old;
                    t_array[0][0] = T_relative(0);
                    t_array[0][1] = T_relative(1);
                    t_array[0][2] = T_relative(2);
                    //将矩阵转换为向量
                    Vector3d euler_angle_old = Utility::R2ypr(R_relative);
                    euler_array[0][0] = euler_angle_old.x();
                    euler_array[0][1] = euler_angle_old.y();
                    euler_array[0][2] = euler_angle_old.z();
                    problem.AddParameterBlock(euler_array[0], 3);
                    problem.AddParameterBlock(t_array[0], 3);

                    
                    //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
                    map<int,int> resample;
                    
        //            temp_index=0;
                    for(int temp_index=0, len=vpkf_index.size(); temp_index<len; temp_index++ ){
                        
                        int kf_index=temp_index+1;
                        Matrix3d relative_r_b_a;
                        Vector3d relative_t_b_a;
                        relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
                        relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
                        Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                        
                        map<int,int>::iterator iter;
                        iter = resample.find(vpkf_index[temp_index]);
                        if(iter != resample.end())
                        {
                            kf_index=resample[vpkf_index[temp_index]];
                        }
                        else
                        {
                               
                            resample[vpkf_index[temp_index]]=kf_index;

                           
                            
                            t_array[kf_index][0] = relative_t_b_a(0);
                            t_array[kf_index][1] = relative_t_b_a(1);
                            t_array[kf_index][2] = relative_t_b_a(2);
                            euler_array[kf_index][0] = relative_r_b_a_euler.x();
                            euler_array[kf_index][1] = relative_r_b_a_euler.y();
                            euler_array[kf_index][2] = relative_r_b_a_euler.z();
                            problem.AddParameterBlock(euler_array[kf_index], 1, angle_local_parameterization);
                            problem.AddParameterBlock(t_array[kf_index], 3);
                            ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),1);
                            problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                        }

                        vector<Vector3d> point_single=point_clouds_all[temp_index];
                        vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
                        for(int a=0,b=point_single.size();a<b;a++){

                            //找到主地图那个点 所在帧的位姿
                            Vector3d pts_i = point_single[a];

                            //相机平面坐标
                            cv::Point2f pt=measure_single[a];
                            float xx=pt.x;
                            float yy=pt.y;

                             Vector2d pts_j ;//要求是归一化图像坐标
                             pts_j<<xx,yy;

                            ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                            problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                            
                            
                         }
                    }
                     
                    
                    ceres::Solve(options, &problem, &summary);
                    std::cout <<"精确相对位姿："<< summary.BriefReport() << "\n";

                    if(summary.termination_type==ceres::CONVERGENCE){
                    
                        cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;

                        Matrix3d Rs_i ;
                        Vector3d Ps_i ;//当前帧
                        cur_kf->getOriginPose(Ps_i, Rs_i);
            //                Vector3d q_i;
            //                q_i<<euler_array[1][0],euler_array[1][1],euler_array[1][2];
            //                Matrix3d Rs_i = Utility::ypr2R(q_i);
            //                Vector3d Ps_i = Vector3d( t_array[1][0],  t_array[1][1],  t_array[1][2]);


                        Vector3d q;
                        q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                        Matrix3d Rs_loop = Utility::ypr2R(q);
                        Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
                        
            //                cout<<"优化计算得到的相对位姿："<<Ps_loop[0]<<" ,"<<Ps_loop[1]<<" ,"<<Ps_loop[2]<<" ,"<<euler_array[0][0]<<" ,"<<euler_array[0][1]<<" ,"<<euler_array[0][2]<<endl;

                        Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
                        Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                        double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                        double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                        double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                        cur_kf->relative_pitch=relative_pitch;
                        cur_kf->relative_roll=relative_roll;
                        cur_kf->updateLoopConnection(relative_t, relative_yaw);
                        cur_kf->loop_info_better_q=relative_q;

                        //先临时搬过来
                        old_index=old_kf->global_index;
                        Vector3d T_w_i_old;
                        Matrix3d R_w_i_old;
                        old_kf->getPose(T_w_i_old, R_w_i_old);

                        Quaterniond Q_loop_old(R_w_i_old);
                        RetriveData retrive_data;
                        retrive_data.cur_index = cur_kf->global_index;
                        retrive_data.header = cur_kf->header;
                        retrive_data.P_old = T_w_i_old;
                        retrive_data.Q_old = Q_loop_old;
                        retrive_data.use = true;
                        retrive_data.measurements = measurements_old_norm_all[0];
                        //这里暂时不给值  因为暂时不发送到客户端
        //                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
                        retrive_data.old_index=old_kf->global_index;
                        vins->retrive_pose_data = (retrive_data);
            //                        vins->isSendLoopData=true;

                        cur_kf->detectLoop(old_kf->global_index);
                        poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                        old_kf->is_looped = 1;
                        loop_old_index = old_kf->global_index;
                        vins->isSendLoop_another=true;
                        cur_kf->is_get_loop_info=true;
                        
                    }
                    else{
                        cout<<"检测失败 未收敛： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
                        
                    }
            }
            }
        }
        usleep(10);
    }

}

void LoopClosure::loopClosureRun_6(){
    while(true){
        if(!mlp_kf_pairs.empty()){
            relative_mutex.lock();
            vector<cv::Point2f> measurements_old_norm_real=mlvv_measurements_old_norm_single.front();
            mlvv_measurements_old_norm_single.pop_front();
            vector<Eigen::Vector3d> point_3d_cur_real=mlvv_point_clouds_single.front();
            mlvv_point_clouds_single.pop_front();
            std::pair<KeyFrame*, KeyFrame*> kf_pair=mlp_kf_pairs.front();
            mlp_kf_pairs.pop_front();
            KeyFrame* cur_kf=kf_pair.first;
            KeyFrame* old_kf=kf_pair.second;
            relative_mutex.unlock();
            
//            usleep(2000);
//            Matrix3d Rs_i ;
//            Vector3d Ps_i ;//当前帧
//            cur_kf->getOriginPose(Ps_i, Rs_i);
//            if(cur_kf->IsOriginUpdate){
//                cout<<"此时当前帧更新了原始位姿11"<<endl;
//            }
            
            Client* client_cur=old_kf->c;
            Matrix3d ric_curClient=client_cur->ric_client;
            Vector3d tic_curClient=client_cur->tic_client;
            Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
            const float &fx = client_cur->FOCUS_LENGTH_X_server;
            const float &fy = client_cur->FOCUS_LENGTH_Y_server;
            const float &cx = client_cur->PX_server;
            const float &cy = client_cur->PY_server;
            
            Matrix3d R_relative;
            Vector3d T_relative;
            Matrix3d oldKF_r;
            Vector3d oldKF_t;
            old_kf->getOriginPose(oldKF_t, oldKF_r);
            
            
          
            
            {
                //先验换一下 因为当前的位姿，是乘了偏移的
//                Matrix3d curKF_r;
//                Vector3d curKF_t;
//                curKF_r=poseGraph->r_drift*oldKF_r;
//                curKF_t=poseGraph->r_drift*oldKF_t+poseGraph->t_drift;
//                old_kf->getPose(curKF_t, curKF_r);
                
                
                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                //options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
    //            loss_function = new ceres::CauchyLoss(1.0);

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

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection2::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),100.0);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
                 }


                ceres::Solve(options, &problem, &summary);
//                std::cout <<"地图内部 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";

                if(summary.termination_type!=ceres::
                   CONVERGENCE){
                    continue;
                }

                T_relative[0]=t_array[0];
                T_relative[1]=t_array[1];
                T_relative[2]=t_array[2];

                R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));
               
            }
            
            vector<vector<cv::Point2f>> measurements_old_norm_all;
            vector<vector<Eigen::Vector3d>> point_clouds_all;
            std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
            std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
            std::vector<Vector3d> point_3d_cur;
            std::vector<cv::Point2f> measurements_cur;//像素坐标
            
            
            vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_old.push_back(vpCovKFi_old[0]);
            vpCovKFi_old[0]=old_kf;
            
//            list<KeyFrame*> lpCovKFi_old;
//            for(KeyFrame* pkf:vpCovKFi_old){
//                lpCovKFi_old.push_back(pkf);
//            }
            
            vector<Matrix3d> oldR_b_a;
            vector<Vector3d> oldT_b_a;
            
            int temp_index=0;
            //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
            for(KeyFrame* pkFi_old: vpCovKFi_old){
                if(temp_index!=0){
                    Matrix3d rwi_old;
                    Vector3d twi_old;
                    pkFi_old->getOriginPose(twi_old, rwi_old);
                    
                    Matrix3d r_b_a;
                    Vector3d t_b_a;
                    r_b_a=oldKF_r.transpose()* rwi_old;
                    t_b_a=oldKF_r.transpose()*(twi_old-oldKF_t);
                    oldR_b_a.push_back(r_b_a);
                    oldT_b_a.push_back(t_b_a);
                }else{
                    oldR_b_a.push_back(Matrix3d::Identity());
                    oldT_b_a.push_back(Vector3d::Zero());
                    temp_index++;//放这里就只要执行一次
                }
            }
            
            int similarNum=0;
            temp_index=0;
            vector<Matrix3d> old_r;
            vector<Vector3d> old_t;
            vector<int> vpkf_index;
            
            for(KeyFrame* pkf_old:vpCovKFi_old){
                Matrix3d r_w_imuOld=R_relative*oldR_b_a[temp_index];
                Vector3d t_w_imuOld=R_relative*oldT_b_a[temp_index] +T_relative;
                Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                Matrix3d r_camOld_w=r_w_camOld.transpose();
                old_r.push_back(r_camOld_w);
                old_t.push_back(-r_camOld_w*t_w_camOld);
                temp_index++;
                
            }
            
            while(!cur_kf->IsOriginUpdate){
                usleep(500);
            }
            std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
            vpCovKFi_cur[0] = cur_kf;
            
//            list<KeyFrame*> lpCovKFi_cur ;
//            for(KeyFrame* pkf:vpCovKFi_cur){
//                lpCovKFi_cur.push_back(pkf);
//            }
            
            {
                //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
                
                point_clouds_all.clear();
                measurements_old_norm_all.clear();
                vpkf_index.clear();
//                cout<<"当前帧的共视帧：";
                for(KeyFrame* pKFi : vpCovKFi_cur){
//                    cout<<pKFi->global_index<<" ,";
                    //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
                    if(pKFi==cur_kf ){
                        point_clouds_all.push_back(point_3d_cur_real);
                        measurements_old_norm_all.push_back(measurements_old_norm_real);
                        vpkf_index.push_back(0);
//                        old_kf->isuse=1;
//                        temp_index++;
                        continue;
                    }
                    //描述符得全有
                    if(!pKFi->is_des_end){
                        continue;
                    }
                    
                    vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                    std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                    vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                    int nPoints = point_clouds_origin_cur.size();
                    int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                   
    //                int num=0;//记录匹配点的数量
                    temp_index=0;//记录遍历到哪个帧了
                    //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                    for(KeyFrame* pkFi_old: vpCovKFi_old)
                    {
                        
//                        if(pKFi==cur_kf && pkFi_old==old_kf){
//                            point_clouds_all.push_back(point_3d_cur_real);
//                            measurements_old_norm_all.push_back(measurements_old_norm_real);
//                            vpkf_index.push_back(0);
//                            temp_index++;
//                            continue;
//                        }
                        //描述符得全有
                        if(!pkFi_old->is_des_end){
                            temp_index++;
                            continue;
                        }
                        
                        measurements_old_coarse.clear();
                        measurements_old_norm_coarse.clear();
                        point_3d_cur.clear();
                        measurements_cur.clear();
                    
                        vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                        //这个得到的是到imu坐标系的位姿
                        Matrix3d r_camOld_w=old_r[temp_index];
                        Vector3d t_camOld_w=old_t[temp_index];
                        
                        for(int i=0;i<nPoints;i++){
                            Vector3d point_main=point_clouds_origin_cur[i];
                            //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                            Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                            //深度必须为正
                            if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
    //                            cout<<"深度不为正"<<endl;
                                continue;
                            }
                            // 投影到图像上
                            double x = p3D_c2[0];
                            double y = p3D_c2[1];
                            double z = p3D_c2[2];
                            //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                            double u=fx*x/z+cx;
                            double v=fy*y/z+cy;
                          
                            
                            if(!pkFi_old->isInImage((float)u, (float)v)){
    //                            cout<<"投影点不在图像内"<<endl;
                                continue;
                            }
                            
                            const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 60);
                            //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                            if(vIndices.empty()){
    //                            cout<<"半径为50个像素 找不到点"<<endl;
                                continue;
                            }
                            //des和keypoints长度不一样
                            //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                            BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                            int bestDist = 256;
                            int bestIndex = -1;
                            for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                            {
            //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                                int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                                if(dis < bestDist)
                                {
                                    bestDist = dis;
                                    bestIndex = *vit;
                                }
                            }
                            
                            if(bestDist<=60){
//                                cout<<"最终选择的半径范围："<<u-keypoints_old[bestIndex].pt.x<<" ,"<<v-keypoints_old[bestIndex].pt.y<<endl;
                                point_3d_cur.push_back(point_main);
                                measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                                measurements_cur.push_back(measurements_origin_cur[i]);
                                
                            }
                                        
                        }
                        
                        cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                        if(measurements_cur.size()>=22){
                            pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
                            cout<<"f矩阵拒绝后的点数："<<measurements_cur.size()<<", "<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<endl;
                            if(measurements_old_coarse.size()>=22){
                            
                                cv::Point2f norm_pt;
                                int pCount=measurements_old_coarse.size();
                                for(int aa=0;aa<pCount;aa++){
                                    norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                    norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                    measurements_old_coarse[aa]=norm_pt;
                                    
                                }
                                
                                point_clouds_all.push_back(point_3d_cur);
                                measurements_old_norm_all.push_back(measurements_old_coarse);
    //                            num+=point_3d_cur.size();
    //                            real_vpCovKFi_cur.push_back(pkFi_old);
                                vpkf_index.push_back(temp_index);
                                similarNum++;
//                                pkFi_old->isuse=1;
//                                cout<<"两帧匹配上："<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<", "<<measurements_old_coarse.size()<<". "<<endl;
//                                break;
                            }
                            
                        }
                        
                        temp_index++;
    //                    ends=clock();
    //                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                    }
                }
            }
            
//            for(KeyFrame* pkFi_old: vpCovKFi_old)
//            {
//                pkFi_old->isuse=0;
//            }
        
            if(similarNum>=2){
                int optiKf_num=vpkf_index.size()+2;
                //构造优化问题
                double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
                Quaterniond q_array[optiKf_num];
                double euler_array[optiKf_num][3];

                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                //options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
        //            ceres::LossFunction *loss_function_feature;
        //            loss_function_feature = new ceres::CauchyLoss(1.0);
                //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
                ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

                
                Matrix3d curKF_r;
                Vector3d curKF_t;
                old_kf->getOriginPose(curKF_t, curKF_r);
                
                
                Matrix3d tmp_r_old;
                Vector3d tmp_t_old;
                t_array[0][0] = curKF_t(0);
                t_array[0][1] = curKF_t(1);
                t_array[0][2] = curKF_t(2);
                //将矩阵转换为向量
                Vector3d euler_angle_old = Utility::R2ypr(curKF_r);
                euler_array[0][0] = euler_angle_old.x();
                euler_array[0][1] = euler_angle_old.y();
                euler_array[0][2] = euler_angle_old.z();
                problem.AddParameterBlock(euler_array[0], 3);
                problem.AddParameterBlock(t_array[0], 3);

                
                //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
                map<int,int> resample;
                
                temp_index=0;
                for(int len=vpkf_index.size(); temp_index<len; temp_index++ ){
                    
                    int kf_index=temp_index+1;
                    Matrix3d relative_r_b_a;
                    Vector3d relative_t_b_a;
                    relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
                    relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
                    Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                    
                    map<int,int>::iterator iter;
                    iter = resample.find(vpkf_index[temp_index]);
                    if(iter != resample.end())
                    {
                        kf_index=resample[vpkf_index[temp_index]];
                    }
                    else
                    {
                        resample[vpkf_index[temp_index]]=kf_index;

                        t_array[kf_index][0] = relative_t_b_a(0);
                        t_array[kf_index][1] = relative_t_b_a(1);
                        t_array[kf_index][2] = relative_t_b_a(2);
                        euler_array[kf_index][0] = relative_r_b_a_euler.x();
                        euler_array[kf_index][1] = relative_r_b_a_euler.y();
                        euler_array[kf_index][2] = relative_r_b_a_euler.z();
                        problem.AddParameterBlock(euler_array[kf_index], 1, angle_local_parameterization);
                        problem.AddParameterBlock(t_array[kf_index], 3);
                       
                        ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),1);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                        
                    }

                    vector<Vector3d> point_single=point_clouds_all[temp_index];
                    vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
                    for(int a=0,b=point_single.size();a<b;a++){

                        //找到主地图那个点 所在帧的位姿
                        Vector3d pts_i = point_single[a];

                        //相机平面坐标
                        cv::Point2f pt=measure_single[a];

                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pt.x, pt.y, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                        
                     }
                }
                
                
                ceres::Solve(options, &problem, &summary);
                std::cout <<"精确相对位姿："<< summary.BriefReport() << "\n";

                if(summary.termination_type==ceres::CONVERGENCE){
                   
                        
                    cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;

                    Matrix3d Rs_i ;
                    Vector3d Ps_i ;//当前帧
                    cur_kf->getOriginPose(Ps_i, Rs_i);
                    
      


                    Vector3d q;
                    q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                    Matrix3d Rs_loop = Utility::ypr2R(q);
                    Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
                    
//                    Ps_loop = poseGraph->r_drift * Ps_loop + poseGraph->t_drift;
//                    Rs_loop = poseGraph->r_drift * Rs_loop;
                    
        //                cout<<"优化计算得到的相对位姿："<<Ps_loop[0]<<" ,"<<Ps_loop[1]<<" ,"<<Ps_loop[2]<<" ,"<<euler_array[0][0]<<" ,"<<euler_array[0][1]<<" ,"<<euler_array[0][2]<<endl;

                    Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
        //                Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                    double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                    double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                    double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                    cur_kf->relative_pitch=relative_pitch;
                    cur_kf->relative_roll=relative_roll;
                    cur_kf->updateLoopConnection(relative_t, relative_yaw);


                    //先临时搬过来
                    old_index=old_kf->global_index;
                    Vector3d T_w_i_old;
                    Matrix3d R_w_i_old;
                    old_kf->getPose(T_w_i_old, R_w_i_old);

                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm_all[0];
                    //这里暂时不给值  因为暂时不发送到客户端
    //                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
                    retrive_data.old_index=old_kf->global_index;
                    vins->retrive_pose_data = (retrive_data);
        //                        vins->isSendLoopData=true;

                    cur_kf->detectLoop(old_kf->global_index);
                    poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                    old_kf->is_looped = 1;
                    loop_old_index = old_kf->global_index;
                    vins->isSendLoop_another=true;
                    if(cur_kf->IsOriginUpdate==true){
                       if (cur_kf->sendLoop==false)
                       {

                           cur_kf->sendLoop=true;
                           vins->globalOpti_index_mutex.lock();
                           vins->kf_global_index.push(cur_kf->global_index);
                           vins->start_kf_global_index.push(cur_kf->loop_index);
                           poseGraph->latest_loop_index=cur_kf->global_index;
                           vins->start_global_optimization = true;//先暂停回环 不启动

                           vins->globalOpti_index_mutex.unlock();
//                           cout<<"慢了 慢了检测到回环 并通过错误回环的检测："<<cur_kf->global_index<<endl;
                       }
                   }
                    cur_kf->is_get_loop_info=true;
                    
                }
                else{
                    cout<<"检测失败 未收敛： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
                    
                }
            }
        }
        usleep(10);
    }

}

void LoopClosure::loopClosureRun_8(){
    while(true){
        if(!mlp_kf_pairs.empty()){
            relative_mutex.lock();
            vector<cv::Point2f> measurements_old_norm_real=mlvv_measurements_old_norm_single.front();
            mlvv_measurements_old_norm_single.pop_front();
            vector<Eigen::Vector3d> point_3d_cur_real=mlvv_point_clouds_single.front();
            mlvv_point_clouds_single.pop_front();
            std::pair<KeyFrame*, KeyFrame*> kf_pair=mlp_kf_pairs.front();
            mlp_kf_pairs.pop_front();
            KeyFrame* cur_kf=kf_pair.first;
            KeyFrame* old_kf=kf_pair.second;
            relative_mutex.unlock();
            
            
            Client* client_cur=old_kf->c;
            Matrix3d ric_curClient=client_cur->ric_client;
            Vector3d tic_curClient=client_cur->tic_client;
            Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
            const float &fx = client_cur->FOCUS_LENGTH_X_server;
            const float &fy = client_cur->FOCUS_LENGTH_Y_server;
            const float &cx = client_cur->PX_server;
            const float &cy = client_cur->PY_server;
            
            Matrix3d R_relative;
            Vector3d T_relative;
            Matrix3d oldKF_r;
            Vector3d oldKF_t;
            old_kf->getOriginPose(oldKF_t, oldKF_r);
            
            
          
            
            {
                //先验换一下 因为当前的位姿，是乘了偏移的
//                Matrix3d curKF_r;
//                Vector3d curKF_t;
//                old_kf->getPose(curKF_t, curKF_r);
//                curKF_r=poseGraph->r_drift*oldKF_r;
//                curKF_t=poseGraph->r_drift*oldKF_t+poseGraph->t_drift;
                
                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                options.trust_region_strategy_type = ceres::DOGLEG;
                //options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
//                loss_function = new ceres::CauchyLoss(1.0);

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

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection2::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),100.0);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
                 }


                ceres::Solve(options, &problem, &summary);
//                std::cout <<"地图内部 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";

//                if(summary.termination_type!=ceres::
//                   CONVERGENCE){
//                    continue;
//                }
//                    cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false);
//                }
//                else{
                    T_relative[0]=t_array[0];
                    T_relative[1]=t_array[1];
                    T_relative[2]=t_array[2];

                    R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));
//                }
            }
            
            vector<vector<cv::Point2f>> measurements_old_norm_all;
            vector<vector<Eigen::Vector3d>> point_clouds_all;
            std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
            std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
            std::vector<Vector3d> point_3d_cur;
            std::vector<cv::Point2f> measurements_cur;//像素坐标
            
            
            vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_old.push_back(vpCovKFi_old[0]);
            vpCovKFi_old[0]=old_kf;
            
//            list<KeyFrame*> lpCovKFi_old;
//            for(KeyFrame* pkf:vpCovKFi_old){
//                lpCovKFi_old.push_back(pkf);
//            }
            
            vector<Matrix3d> oldR_b_a;
            vector<Vector3d> oldT_b_a;
            
            int temp_index=0;
            //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
            for(KeyFrame* pkFi_old: vpCovKFi_old){
                if(temp_index!=0){
                    Matrix3d rwi_old;
                    Vector3d twi_old;
                    pkFi_old->getOriginPose(twi_old, rwi_old);
                    
                    Matrix3d r_b_a;
                    Vector3d t_b_a;
                    r_b_a=oldKF_r.transpose()* rwi_old;
                    t_b_a=oldKF_r.transpose()*(twi_old-oldKF_t);
                    oldR_b_a.push_back(r_b_a);
                    oldT_b_a.push_back(t_b_a);
                }else{
                    oldR_b_a.push_back(Matrix3d::Identity());
                    oldT_b_a.push_back(Vector3d::Zero());
                    temp_index++;//放这里就只要执行一次
                }
            }
            
            int similarNum=0;
            temp_index=0;
            vector<Matrix3d> old_r;
            vector<Vector3d> old_t;
            vector<int> vpkf_index;
            
            for(KeyFrame* pkf_old:vpCovKFi_old){
                Matrix3d r_w_imuOld=R_relative*oldR_b_a[temp_index];
                Vector3d t_w_imuOld=R_relative*oldT_b_a[temp_index] +T_relative;
                Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                Matrix3d r_camOld_w=r_w_camOld.transpose();
                old_r.push_back(r_camOld_w);
                old_t.push_back(-r_camOld_w*t_w_camOld);
                temp_index++;
                
            }
            
            while(!cur_kf->IsOriginUpdate){
                usleep(500);
            }
            std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
            vpCovKFi_cur[0] = cur_kf;
            
            
            {
                //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
                
                point_clouds_all.clear();
                measurements_old_norm_all.clear();
                vpkf_index.clear();
//                cout<<"当前帧的共视帧：";
                
//                cout<<endl<<endl;
                for(KeyFrame* pKFi : vpCovKFi_cur){
//                    cout<<pKFi->global_index<<" ,";
                    //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
//                    if(pKFi==cur_kf ){
//                        point_clouds_all.push_back(point_3d_cur_real);
//                        measurements_old_norm_all.push_back(measurements_old_norm_real);
//                        vpkf_index.push_back(0);
////                        old_kf->isuse=1;
////                        temp_index++;
//                        continue;
//                    }
                    //描述符得全有
                    if(!pKFi->is_des_end){
                        continue;
                    }
                    
                    vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                    std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                    vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                    int nPoints = point_clouds_origin_cur.size();
                    int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                   
    //                int num=0;//记录匹配点的数量
                    temp_index=0;//记录遍历到哪个帧了
                    //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                    for(KeyFrame* pkFi_old: vpCovKFi_old)
                    {
                        
                        if(pKFi==cur_kf && pkFi_old==old_kf){
                            point_clouds_all.push_back(point_3d_cur_real);
                            measurements_old_norm_all.push_back(measurements_old_norm_real);
                            vpkf_index.push_back(0);
                            temp_index++;
                            continue;
                        }
                        //描述符得全有
                        if(!pkFi_old->is_des_end){
                            temp_index++;
                            continue;
                        }
                        
                        measurements_old_coarse.clear();
                        measurements_old_norm_coarse.clear();
                        point_3d_cur.clear();
                        measurements_cur.clear();
                    
                        vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                        //这个得到的是到imu坐标系的位姿
                        Matrix3d r_camOld_w=old_r[temp_index];
                        Vector3d t_camOld_w=old_t[temp_index];
                        
                        for(int i=0;i<nPoints;i++){
                            Vector3d point_main=point_clouds_origin_cur[i];
                            //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                            Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                            //深度必须为正
                            if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
    //                            cout<<"深度不为正"<<endl;
                                continue;
                            }
                            // 投影到图像上
                            double x = p3D_c2[0];
                            double y = p3D_c2[1];
                            double z = p3D_c2[2];
                            //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                            double u=fx*x/z+cx;
                            double v=fy*y/z+cy;
                          
                            
                            if(!pkFi_old->isInImage((float)u, (float)v)){
    //                            cout<<"投影点不在图像内"<<endl;
                                continue;
                            }
                            
                            const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 60);
                            //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                            if(vIndices.empty()){
    //                            cout<<"半径为50个像素 找不到点"<<endl;
                                continue;
                            }
                            //des和keypoints长度不一样
                            //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                            BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                            int bestDist = 256;
                            int bestIndex = -1;
                            for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                            {
            //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                                int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                                if(dis < bestDist)
                                {
                                    bestDist = dis;
                                    bestIndex = *vit;
                                }
                            }
                            
                            if(bestDist<70){
//                                cout<<"最终选择的半径范围："<<u-keypoints_old[bestIndex].pt.x<<" ,"<<v-keypoints_old[bestIndex].pt.y<<endl;
                                point_3d_cur.push_back(point_main);
                                measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                                measurements_cur.push_back(measurements_origin_cur[i]);
                                
                            }
                                        
                        }
                        
//                        cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                        if(measurements_cur.size()>=22){
                            pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
//                            cout<<"f矩阵拒绝后的点数："<<measurements_cur.size()<<", "<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<endl;
                            if(measurements_old_coarse.size()>=22){
                            
                                cv::Point2f norm_pt;
                                int pCount=measurements_old_coarse.size();
                                for(int aa=0;aa<pCount;aa++){
                                    norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                    norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                    measurements_old_coarse[aa]=norm_pt;
                                    
                                }
                                
                                point_clouds_all.push_back(point_3d_cur);
                                measurements_old_norm_all.push_back(measurements_old_coarse);
    //                            num+=point_3d_cur.size();
    //                            real_vpCovKFi_cur.push_back(pkFi_old);
                                vpkf_index.push_back(temp_index);
                                similarNum++;
//                                pkFi_old->isuse=1;
//                                cout<<"两帧匹配上："<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<", "<<measurements_old_coarse.size()<<". "<<endl;
//                                break;
                            }
                            
                        }
                        
                        temp_index++;
    //                    ends=clock();
    //                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                    }
                }
            }
            
//            for(KeyFrame* pkFi_old: vpCovKFi_old)
//            {
//                pkFi_old->isuse=0;
//            }
        
            if(similarNum>=2){
                int optiKf_num=vpkf_index.size()+2;
                //构造优化问题
                double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
                Quaterniond q_array[optiKf_num];
                double euler_array[optiKf_num][3];

                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                //options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
        //            ceres::LossFunction *loss_function_feature;
        //            loss_function_feature = new ceres::CauchyLoss(1.0);
                //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
                ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

                
                Matrix3d curKF_r;
                Vector3d curKF_t;
                old_kf->getOriginPose(curKF_t, curKF_r);
                
                
                Matrix3d tmp_r_old;
                Vector3d tmp_t_old;
                t_array[0][0] = curKF_t(0);
                t_array[0][1] = curKF_t(1);
                t_array[0][2] = curKF_t(2);
                //将矩阵转换为向量
                Vector3d euler_angle_old = Utility::R2ypr(curKF_r);
                euler_array[0][0] = euler_angle_old.x();
                euler_array[0][1] = euler_angle_old.y();
                euler_array[0][2] = euler_angle_old.z();
                problem.AddParameterBlock(euler_array[0], 3);
                problem.AddParameterBlock(t_array[0], 3);

                
                //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
                map<int,int> resample;
                
                temp_index=0;
                for(int len=vpkf_index.size(); temp_index<len; temp_index++ ){
                    
                    int kf_index=temp_index+1;
                    Matrix3d relative_r_b_a;
                    Vector3d relative_t_b_a;
                    relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
                    relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
                    Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                    
                    map<int,int>::iterator iter;
                    iter = resample.find(vpkf_index[temp_index]);
                    if(iter != resample.end())
                    {
                        kf_index=resample[vpkf_index[temp_index]];
                    }
                    else
                    {
                        resample[vpkf_index[temp_index]]=kf_index;

                        t_array[kf_index][0] = relative_t_b_a(0);
                        t_array[kf_index][1] = relative_t_b_a(1);
                        t_array[kf_index][2] = relative_t_b_a(2);
                        euler_array[kf_index][0] = relative_r_b_a_euler.x();
                        euler_array[kf_index][1] = relative_r_b_a_euler.y();
                        euler_array[kf_index][2] = relative_r_b_a_euler.z();
                        problem.AddParameterBlock(euler_array[kf_index], 1, angle_local_parameterization);
                        problem.AddParameterBlock(t_array[kf_index], 3);
                       
                        ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),1);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                        
                    }

                    vector<Vector3d> point_single=point_clouds_all[temp_index];
                    vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
                    for(int a=0,b=point_single.size();a<b;a++){

                        //找到主地图那个点 所在帧的位姿
                        Vector3d pts_i = point_single[a];

                        //相机平面坐标
                        cv::Point2f pt=measure_single[a];

                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pt.x, pt.y, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                        
                     }
                }
                
                
                ceres::Solve(options, &problem, &summary);
                std::cout <<"精确相对位姿："<< summary.BriefReport() << "\n";

                if(summary.termination_type==ceres::CONVERGENCE){
                   
                        
                    cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;

                    Matrix3d Rs_i ;
                    Vector3d Ps_i ;//当前帧
                    cur_kf->getOriginPose(Ps_i, Rs_i);
                    
      


                    Vector3d q;
                    q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                    Matrix3d Rs_loop = Utility::ypr2R(q);
                    Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
                    
//                    Ps_loop = poseGraph->r_drift * Ps_loop + poseGraph->t_drift;
//                    Rs_loop = poseGraph->r_drift * Rs_loop;
                    
        //                cout<<"优化计算得到的相对位姿："<<Ps_loop[0]<<" ,"<<Ps_loop[1]<<" ,"<<Ps_loop[2]<<" ,"<<euler_array[0][0]<<" ,"<<euler_array[0][1]<<" ,"<<euler_array[0][2]<<endl;

                    Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
        //                Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                    double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                    double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                    double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                    cur_kf->relative_pitch=relative_pitch;
                    cur_kf->relative_roll=relative_roll;
                    cur_kf->updateLoopConnection(relative_t, relative_yaw);


                    //先临时搬过来
                    old_index=old_kf->global_index;
                    Vector3d T_w_i_old;
                    Matrix3d R_w_i_old;
                    old_kf->getPose(T_w_i_old, R_w_i_old);

                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm_all[0];
                    //这里暂时不给值  因为暂时不发送到客户端
    //                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
                    retrive_data.old_index=old_kf->global_index;
                    vins->retrive_pose_data = (retrive_data);
        //                        vins->isSendLoopData=true;

                    cur_kf->detectLoop(old_kf->global_index);
                    poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                    old_kf->is_looped = 1;
                    loop_old_index = old_kf->global_index;
                    vins->isSendLoop_another=true;
                    if(cur_kf->IsOriginUpdate==true){
                       if (cur_kf->sendLoop==false)
                       {

                           cur_kf->sendLoop=true;
                           vins->globalOpti_index_mutex.lock();
                           vins->kf_global_index.push(cur_kf->global_index);
                           vins->start_kf_global_index.push(cur_kf->loop_index);
                           poseGraph->latest_loop_index=cur_kf->global_index;
                           vins->start_global_optimization = true;//先暂停回环 不启动

                           vins->globalOpti_index_mutex.unlock();
//                           cout<<"慢了 慢了检测到回环 并通过错误回环的检测："<<cur_kf->global_index<<endl;
                       }
                   }
                    cur_kf->is_get_loop_info=true;
                    
                }
                else{
                    cout<<"检测失败 未收敛： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
                    
                }
            }else{
                cout<<"回环检测的数量不够"<<endl;
            }
        }
        usleep(10);
    }

}

void set_ceres_solver_bound( ceres::Problem &problem ,double * para_buffer_RT )
{
    for ( unsigned int i = 0; i < 3; i++ )
    {

        problem.SetParameterLowerBound( para_buffer_RT, i, -2 );
        problem.SetParameterUpperBound( para_buffer_RT, i, +2 );
    }
};


float *rand_array_uniform( float low = 0.0, float hight = 1.0, size_t numbers= 100 )
{
    std::mt19937 m_random_engine = std::mt19937( std::random_device{}());
    
    float *res = new float[ numbers ];
    std::uniform_real_distribution< float > m_dist = std::uniform_real_distribution< float >( low, hight );
    for ( size_t i = 0; i < numbers; i++ )
    {
        res[ i ] = m_dist( m_random_engine );
    }
    return res;
}

//double compute_inlier_residual_threshold( std::vector< double > residuals, double ratio )
//{
//    std::set< double > dis_vec;
//    for ( size_t i = 0; i < ( size_t )( residuals.size() / 2 ); i++ )
//    {
//        dis_vec.insert( fabs( residuals[ 2 * i + 0 ] ) + fabs( residuals[ 2 * i + 1 ] )  );
////        cout<<"测试误差："<<fabs( residuals[ 2 * i + 0 ] ) + fabs( residuals[ 2 * i + 1 ] ) <<endl;
//    }
//    return *( std::next( dis_vec.begin(), ( int ) ( ( ratio ) * dis_vec.size() ) ) );
//}


//double compute_inlier_residual_threshold_2( std::vector< double > residuals, double ratio )
//{
//    std::set< double > dis_vec;
//    for ( size_t i = 0; i < ( size_t )( residuals.size() / 2 ); i++ )
//    {
//        dis_vec.insert( fabs( residuals[ 2 * i + 0 ] ) + fabs( residuals[ 2 * i + 1 ] )  );
//    }
//    return *( std::next( dis_vec.begin(), ( int ) ( ( ratio ) * dis_vec.size() ) ) );
//}



//不再通过ceres过滤， 自己加一层过滤
void LoopClosure::loopClosureRun_9(){
    while(true){
        if(!mlp_kf_pairs.empty()){
            relative_mutex.lock();
            vector<cv::Point2f> measurements_old_norm_real=mlvv_measurements_old_norm_single.front();
            mlvv_measurements_old_norm_single.pop_front();
            vector<Eigen::Vector3d> point_3d_cur_real=mlvv_point_clouds_single.front();
            mlvv_point_clouds_single.pop_front();
            std::pair<KeyFrame*, KeyFrame*> kf_pair=mlp_kf_pairs.front();
            mlp_kf_pairs.pop_front();
            KeyFrame* cur_kf=kf_pair.first;
            KeyFrame* old_kf=kf_pair.second;
            relative_mutex.unlock();
            
            
            Client* client_cur=old_kf->c;
            Matrix3d ric_curClient=client_cur->ric_client;
            Vector3d tic_curClient=client_cur->tic_client;
            Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
            const float &fx = client_cur->FOCUS_LENGTH_X_server;
            const float &fy = client_cur->FOCUS_LENGTH_Y_server;
            const float &cx = client_cur->PX_server;
            const float &cy = client_cur->PY_server;
            
            Matrix3d R_relative;
            Vector3d T_relative;
            Matrix3d oldKF_r;
            Vector3d oldKF_t;
            old_kf->getOriginPose(oldKF_t, oldKF_r);
            
            {
                //先验换一下 因为当前的位姿，是乘了偏移的
//                Matrix3d curKF_r;
//                Vector3d curKF_t;
//                old_kf->getPose(curKF_t, curKF_r);
//                oldKF_r=poseGraph->r_drift*oldKF_r;
//                oldKF_t=poseGraph->r_drift*oldKF_t+poseGraph->t_drift;
                
                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                options.trust_region_strategy_type = ceres::DOGLEG;
                //options.minimizer_progress_to_stdout = true;
//                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
//                loss_function = new ceres::CauchyLoss(1.0);
                
                std::vector<ceres::ResidualBlockId> residual_block_ids;
                ceres::ResidualBlockId              block_id;

                //实验用 记录过滤了哪些
                vector<int> point_all;
                
                
                
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

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection2::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),100.0);
                    block_id =  problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
                    residual_block_ids.push_back( block_id );
                    
                 }

                //-------------临时 实验用------------------
//                options.linear_solver_type = ceres::DENSE_SCHUR;
//                options.max_num_iterations = 20;
//                ceres::Solve( options, &problem, &summary );
//                if(summary.termination_type!=ceres::
//                   CONVERGENCE){
//                    //这里记录是哪一帧，哪些特征点，featureId
//
//                    //这里通过ceres过滤一层，记录 特征点，featureId
//                }
                
                //----------------
                
                std::vector< ceres::ResidualBlockId > residual_block_ids_temp;
                residual_block_ids_temp.reserve( residual_block_ids.size() );

                // Drop some of the residual to guaruntee the real time performance.
//                if ( residual_block_ids.size() > (size_t) 1e5 )
//                {
//                    residual_block_ids_temp.clear();
//
//                    float  threshold_to_reserve = ( float ) 1e5 / ( float ) residual_block_ids.size();
//                    float *probability_to_drop = rand_array_uniform( 0, 1.0, residual_block_ids.size() );
////                    screen_out << "Number of residual blocks too Large, drop them to " << m_maximum_allow_residual_block << endl;
//                    for ( size_t i = 0; i < residual_block_ids.size(); i++ )
//                    {
//                        if ( probability_to_drop[ i ] > threshold_to_reserve )
//                        {
//                            problem.RemoveResidualBlock( residual_block_ids[ i ] );
//                        }
//                        else
//                        {
//                            residual_block_ids_temp.push_back( residual_block_ids[ i ] );
//                        }
//                    }
//                    residual_block_ids = residual_block_ids_temp;
//                    delete probability_to_drop;
//                }

//                cout<<"测试：residual_block_ids.size11= "<<residual_block_ids.size()<<endl;
                for ( size_t ii = 0; ii < 1; ii++ )
                {
                    options.max_num_iterations = 2;
//                    options.minimizer_progress_to_stdout = false;
//                    options.check_gradients = false;
//                    options.gradient_check_relative_precision = 1e-10;
                    //options.function_tolerance = 1e-100; // default 1e-6


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
                            point_all.push_back(i);
                        }
                        else
                        {
                            residual_block_ids_temp.push_back( residual_block_ids[ i ] );
                        }
                    }
                    residual_block_ids = residual_block_ids_temp;
                }
//                cout<<"过滤的点数"<<point_all.size()<<endl;
                //实验记录
//                std::ofstream outFile;
//                //打开文件
//                outFile.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/"+
//                             to_string(cur_kf->global_index)+"&"+to_string(cur_kf->header)+"&"+to_string(old_kf->global_index)+"&"+to_string(old_kf->header)+".txt");
//
//                for(auto iter=point_all.begin(), iter_end=point_all.end(); iter!=iter_end; iter++)
//                {
//                    //写入数据
//                    outFile << (*iter)<<" ";
//                }
//                outFile<<"\n";
//                //关闭文件
//                outFile.close();
                
//                cout<<"测试：residual_block_ids.size22= "<<residual_block_ids.size()<<endl;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                options.max_num_iterations = 20;
//                options.minimizer_progress_to_stdout = false;
//                options.check_gradients = false;
//                options.gradient_check_relative_precision = 1e-10;

                ceres::Solve( options, &problem, &summary );
//                std::cout <<"地图内部 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";
                

                if(summary.termination_type!=ceres::
                   CONVERGENCE){
                    continue;
                }
//                    cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false);
//                }
//                else{
                //这里代表是 在当前帧的误差基础上，老帧的世界位姿
                    T_relative[0]=t_array[0];
                    T_relative[1]=t_array[1];
                    T_relative[2]=t_array[2];

                    R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));
//                }
            }
            
            vector<vector<cv::Point2f>> measurements_old_norm_all;
            vector<vector<Eigen::Vector3d>> point_clouds_all;
            std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
            std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
            std::vector<Vector3d> point_3d_cur;
            std::vector<cv::Point2f> measurements_cur;//像素坐标
            
//            cout<<"cur_kf id:"<<cur_kf->global_index<<" , " << old_kf->global_index<<endl;
            vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
//            可能存在和第0帧共视了 第0帧没有共视帧
            if(vpCovKFi_old.size()==0){
                vpCovKFi_old.push_back(old_kf);
            }else{
                vpCovKFi_old.push_back(vpCovKFi_old[0]);//为空
                vpCovKFi_old[0]=old_kf;
            }
            
            
//            list<KeyFrame*> lpCovKFi_old;
//            for(KeyFrame* pkf:vpCovKFi_old){
//                lpCovKFi_old.push_back(pkf);
//            }
            
            vector<Matrix3d> oldR_b_a;
            vector<Vector3d> oldT_b_a;
            
            int temp_index=0;
            //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
            for(KeyFrame* pkFi_old: vpCovKFi_old){
                if(temp_index!=0){
                    Matrix3d rwi_old;
                    Vector3d twi_old;
                    pkFi_old->getOriginPose(twi_old, rwi_old);
                    
                    Matrix3d r_b_a;
                    Vector3d t_b_a;
                    r_b_a=oldKF_r.transpose()* rwi_old;
                    t_b_a=oldKF_r.transpose()*(twi_old-oldKF_t);
                    oldR_b_a.push_back(r_b_a);
                    oldT_b_a.push_back(t_b_a);
                }else{
                    oldR_b_a.push_back(Matrix3d::Identity());
                    oldT_b_a.push_back(Vector3d::Zero());
                    temp_index++;//放这里就只要执行一次
                }
            }
            
            int similarNum=0;
            temp_index=0;
            vector<Matrix3d> old_r;
            vector<Vector3d> old_t;
            vector<int> vpkf_index;
            
            for(KeyFrame* pkf_old:vpCovKFi_old){
                Matrix3d r_w_imuOld=R_relative*oldR_b_a[temp_index];
                Vector3d t_w_imuOld=R_relative*oldT_b_a[temp_index] +T_relative;
                Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                Matrix3d r_camOld_w=r_w_camOld.transpose();
                old_r.push_back(r_camOld_w);
                old_t.push_back(-r_camOld_w*t_w_camOld);
                temp_index++;
                
            }
            
            while(!cur_kf->IsOriginUpdate){
                usleep(50);
            }
            std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
            vpCovKFi_cur[0] = cur_kf;
            
            
            {
                //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
                
                point_clouds_all.clear();
                measurements_old_norm_all.clear();
                vpkf_index.clear();
//                cout<<"当前帧的共视帧：";
                
//                cout<<endl<<endl;
                for(KeyFrame* pKFi : vpCovKFi_cur){
//                    cout<<pKFi->global_index<<" ,";
                    //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
//                    if(pKFi==cur_kf ){
//                        point_clouds_all.push_back(point_3d_cur_real);
//                        measurements_old_norm_all.push_back(measurements_old_norm_real);
//                        vpkf_index.push_back(0);
////                        old_kf->isuse=1;
////                        temp_index++;
//                        continue;
//                    }
                    //描述符得全有
                    if(!pKFi->is_des_end){
                        continue;
                    }
                    
                    vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                    std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                    vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                    int nPoints = point_clouds_origin_cur.size();
                    int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                   
    //                int num=0;//记录匹配点的数量
                    temp_index=0;//记录遍历到哪个帧了
                    //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                    for(KeyFrame* pkFi_old: vpCovKFi_old)
                    {
                        
//                        if(pKFi==cur_kf && pkFi_old==old_kf){
//                            point_clouds_all.push_back(point_3d_cur_real);
//                            measurements_old_norm_all.push_back(measurements_old_norm_real);
//                            vpkf_index.push_back(0);
//                            temp_index++;
//                            continue;
//                        }
                        //描述符得全有
                        if(!pkFi_old->is_des_end){
                            temp_index++;
                            continue;
                        }
                        
                        measurements_old_coarse.clear();
                        measurements_old_norm_coarse.clear();
                        point_3d_cur.clear();
                        measurements_cur.clear();
                    
                        vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                        //这个得到的是到imu坐标系的位姿
                        Matrix3d r_camOld_w=old_r[temp_index];
                        Vector3d t_camOld_w=old_t[temp_index];
                        
                        for(int i=0;i<nPoints;i++){
                            Vector3d point_main=point_clouds_origin_cur[i];
                            //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                            Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                            //深度必须为正
                            if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
    //                            cout<<"深度不为正"<<endl;
                                continue;
                            }
                            // 投影到图像上
                            double x = p3D_c2[0];
                            double y = p3D_c2[1];
                            double z = p3D_c2[2];
                            //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                            double u=fx*x/z+cx;
                            double v=fy*y/z+cy;
                          
                            
                            if(!pkFi_old->isInImage((float)u, (float)v)){
    //                            cout<<"投影点不在图像内"<<endl;
                                continue;
                            }
                            
                            const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 20);
                            //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                            if(vIndices.empty()){
    //                            cout<<"半径为50个像素 找不到点"<<endl;
                                continue;
                            }
                            //des和keypoints长度不一样
                            //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                            BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                            int bestDist = 256;
                            int bestIndex = -1;
                            for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                            {
            //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                                int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                                if(dis < bestDist)
                                {
                                    bestDist = dis;
                                    bestIndex = *vit;
                                }
                            }
                            
                            if(bestDist<70){
//                                cout<<"最终选择的半径范围："<<u-keypoints_old[bestIndex].pt.x<<" ,"<<v-keypoints_old[bestIndex].pt.y<<endl;
                                point_3d_cur.push_back(point_main);
                                measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                                measurements_cur.push_back(measurements_origin_cur[i]);
                                
                            }
                                        
                        }
                        
//                        cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                        if(measurements_cur.size()>=22){
                            pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
//                            cout<<"f矩阵拒绝后的点数："<<measurements_old_coarse.size()<<endl;
                            if(measurements_old_coarse.size()>=16){
                            
                                cv::Point2f norm_pt;
                                int pCount=measurements_old_coarse.size();
                                for(int aa=0;aa<pCount;aa++){
                                    norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                    norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                    measurements_old_coarse[aa]=norm_pt;
                                    
                                }
                                
                                point_clouds_all.push_back(point_3d_cur);
                                measurements_old_norm_all.push_back(measurements_old_coarse);
    //                            num+=point_3d_cur.size();
    //                            real_vpCovKFi_cur.push_back(pkFi_old);
                                vpkf_index.push_back(temp_index);
                                similarNum++;
//                                pkFi_old->isuse=1;
//                                cout<<"两帧匹配上："<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<", "<<measurements_old_coarse.size()<<". "<<endl;
//                                break;
                            }
                            
                        }
                        
                        temp_index++;
    //                    ends=clock();
    //                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                    }
                }
            }
            
//            for(KeyFrame* pkFi_old: vpCovKFi_old)
//            {
//                pkFi_old->isuse=0;
//            }
        
            if(similarNum>=3){
                int optiKf_num=vpkf_index.size()+2;
                //构造优化问题
                double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
                Quaterniond q_array[optiKf_num];
                double euler_array[optiKf_num][3];

                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                //options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
        //            ceres::LossFunction *loss_function_feature;
        //            loss_function_feature = new ceres::CauchyLoss(1.0);
                //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
                ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

                
                Matrix3d curKF_r;
                Vector3d curKF_t;
                old_kf->getOriginPose(curKF_t, curKF_r);
                
                
                Matrix3d tmp_r_old;
                Vector3d tmp_t_old;
                t_array[0][0] = curKF_t(0);
                t_array[0][1] = curKF_t(1);
                t_array[0][2] = curKF_t(2);
                //将矩阵转换为向量
                Vector3d euler_angle_old = Utility::R2ypr(curKF_r);
                euler_array[0][0] = euler_angle_old.x();
                euler_array[0][1] = euler_angle_old.y();
                euler_array[0][2] = euler_angle_old.z();
                problem.AddParameterBlock(euler_array[0], 3);
                problem.AddParameterBlock(t_array[0], 3);

                
                //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
                map<int,int> resample;
                
                temp_index=0;
                for(int len=vpkf_index.size(); temp_index<len; temp_index++ ){
                    
                    int kf_index=temp_index+1;
                    Matrix3d relative_r_b_a;
                    Vector3d relative_t_b_a;
                    relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
                    relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
                    Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                    
                    map<int,int>::iterator iter;
                    iter = resample.find(vpkf_index[temp_index]);
                    if(iter != resample.end())
                    {
                        kf_index=resample[vpkf_index[temp_index]];
                    }
                    else
                    {
                        resample[vpkf_index[temp_index]]=kf_index;

                        t_array[kf_index][0] = relative_t_b_a(0);
                        t_array[kf_index][1] = relative_t_b_a(1);
                        t_array[kf_index][2] = relative_t_b_a(2);
                        euler_array[kf_index][0] = relative_r_b_a_euler.x();
                        euler_array[kf_index][1] = relative_r_b_a_euler.y();
                        euler_array[kf_index][2] = relative_r_b_a_euler.z();
                        problem.AddParameterBlock(euler_array[kf_index], 1, angle_local_parameterization);
                        problem.AddParameterBlock(t_array[kf_index], 3);
                       
                        ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),1);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                        
                    }

                    vector<Vector3d> point_single=point_clouds_all[temp_index];
                    vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
                    for(int a=0,b=point_single.size();a<b;a++){

                        //找到主地图那个点 所在帧的位姿
                        Vector3d pts_i = point_single[a];

                        //相机平面坐标
                        cv::Point2f pt=measure_single[a];

                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pt.x, pt.y, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                        
                     }
                }
                
                
                ceres::Solve(options, &problem, &summary);
//                std::cout <<"地图内部 精确相对位姿："<< summary.BriefReport() << "\n";

                if(summary.termination_type==ceres::CONVERGENCE){
                   
                        
//                    cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;

                    Matrix3d Rs_i ;
                    Vector3d Ps_i ;//当前帧
                    cur_kf->getOriginPose(Ps_i, Rs_i);
                    
      


                    Vector3d q;
                    q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                    Matrix3d Rs_loop = Utility::ypr2R(q);
                    Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);


                    Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                    Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                    double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                    double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                    double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                    
                    Quaterniond rs_loop_q=Quaterniond(Rs_loop),rs_i_q=Quaterniond(Rs_i);
                    Quaterniond rela_q=rs_loop_q.inverse()*rs_i_q;
                    
                    cout<<"当前帧与老帧"<<cur_kf->global_index<<" , "<<old_kf->global_index<<endl;
                    cout<<relative_t<<endl;
                    cout<<rela_q.w()<<" , "<<rela_q.x()<<" , "<<rela_q.y()<<" , "<<rela_q.z()<<endl;
                    
                    
                    cur_kf->relative_pitch=relative_pitch;
                    cur_kf->relative_roll=relative_roll;
                    cur_kf->updateLoopConnection(relative_t, relative_yaw);
                    cur_kf->loop_info_better_q=rela_q;
//                    cur_kf->loop_info_better_q=relative_q;
                    
                    //测试
                    Quaterniond tmp_q,tmp_i;
                    tmp_q=Rs_loop;
                    tmp_i=Rs_i;
                    double w_q_i[4],i_q_w[4],t_w_ij[3],t_i_ij[3];
                    w_q_i[0]=tmp_q.w();
                    w_q_i[1]=tmp_q.x();
                    w_q_i[2]=tmp_q.y();
                    w_q_i[3]=tmp_q.z();
                    QuaternionInverse(w_q_i, i_q_w);
                    t_w_ij[0]=Ps_i[0] - Ps_loop[0];
                    t_w_ij[1]=Ps_i[1] - Ps_loop[1];
                    t_w_ij[2]=Ps_i[2] - Ps_loop[2];
                    ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);
        //            cout<<"relative_t"<<relative_t.x()<<" , "<<relative_t.y()<<" , "<<relative_t.z()<<endl;
        //            cout<<"t_i_ij="<<t_i_ij[0]<<" , "<<t_i_ij[1]<<" , "<<t_i_ij[2]<<endl;
                    
                    
                    double w_q_j[4],q_i_j[4];
                    w_q_j[0]=tmp_i.w();
                    w_q_j[1]=tmp_i.x();
                    w_q_j[2]=tmp_i.y();
                    w_q_j[3]=tmp_i.z();
                    ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);
                    
                    
                    if(fabs(cur_kf->loop_info_better_q.x()-q_i_j[1])>0.000001 || fabs(cur_kf->loop_info_better_q.y()-q_i_j[2])>0.000001 || fabs(cur_kf->loop_info_better_q.z()-q_i_j[3])>0.000001 ){
                        cout<<"rela_q"<<cur_kf->loop_info_better_q.w()<<" , "<<cur_kf->loop_info_better_q.x()<<" , "<<cur_kf->loop_info_better_q.y()<<" , "<<cur_kf->loop_info_better_q.z()<<endl;
                        cout<<"q_i_j"<<q_i_j[0]<<" , "<<q_i_j[1]<<" , "<<q_i_j[2]<<" , "<<q_i_j[3]<<endl;
                        cout<<"数据不规范回环"<<endl;
                        
                        Quaterniond Rs_loop_inv;
                        Rs_loop_inv=Rs_loop.transpose();
                        cout<<"Rs_loop_inv="<<Rs_loop_inv.w()<<" , "<<Rs_loop_inv.x()<<" , "<<Rs_loop_inv.y()<<" , "<<Rs_loop_inv.z()<<endl;
                        cout<<"i_q_w="<<i_q_w[0]<<" , "<<i_q_w[1]<<" , "<<i_q_w[2]<<" , "<<i_q_w[3]<<endl;
                        
                        
                        continue;
                    }
                    
                    

                    //先临时搬过来
                    old_index=old_kf->global_index;
                    Vector3d T_w_i_old;
                    Matrix3d R_w_i_old;
                    old_kf->getPose(T_w_i_old, R_w_i_old);

                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm_all[0];
                    //这里暂时不给值  因为暂时不发送到客户端
    //                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
                    retrive_data.old_index=old_kf->global_index;
                    vins->retrive_pose_data = (retrive_data);
        //                        vins->isSendLoopData=true;

                    cur_kf->detectLoop(old_kf->global_index);
                    poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                    old_kf->is_looped = 1;
                    loop_old_index = old_kf->global_index;
                    vins->isSendLoop_another=true;
                    if(cur_kf->IsOriginUpdate==true){
                       if (cur_kf->sendLoop==false)
                       {

                           cur_kf->sendLoop=true;
                           vins->globalOpti_index_mutex.lock();
                           vins->kf_global_index.push(cur_kf->global_index);
                           vins->start_kf_global_index.push(cur_kf->loop_index);
                           poseGraph->latest_loop_index=cur_kf->global_index;
                           vins->start_global_optimization = true;//先暂停回环 不启动

                           vins->globalOpti_index_mutex.unlock();
//                           cout<<"慢了 慢了检测到回环 并通过错误回环的检测："<<cur_kf->global_index<<endl;
                       }
                   }
                    cur_kf->is_get_loop_info=true;
                    
//                    //        ----------子地图分配 调用allocateSubMap----------
//                    relative_mutex.lock();
//                    vector<int> old_vector=old_vector_forSubMap.front();
//                    old_vector_forSubMap.pop_front();
//                    relative_mutex.unlock();
//                    global_featureMap->allocateSubMap(cur_kf, old_kf->global_index,old_vector);
//                    //        ----保存地图内部的回环------
                    string kfNum_tree_dir="/Users/zhangjianhua/Desktop/2022paper/mh/gt/";
                    std::ofstream outFile;
                    outFile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
                    outFile.precision(16);
                    //ios::binary|
                    int agentId=cur_kf->c->getId();
                    kfNum_tree_dir+=to_string(agentId);
                    outFile.open(kfNum_tree_dir+"_intra_loop.txt",ios::out|ios::app);
//                    cout<<"kfNum_tree_dir="<<kfNum_tree_dir+"intra_loop.txt" <<endl;
                    outFile<<cur_kf->global_index<<" , "<<cur_kf->header<<" , "<<old_kf->global_index<<" , "<<old_kf->header<<"\n";
                    
                    //关闭文件
                    outFile.close();
//                    cout<<"保存保存了"<<endl;
                    
                }
                else{
//                    cout<<"地图内部 检测失败 未收敛： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
                    
                }
            }else{
//                cout<<"地图内部 回环检测的数量不够:"<<similarNum<<endl;
            }
        }
        usleep(10);
    }

}


//和startLoopClosure_9实现细节一样 但是用老帧的3d点
bool LoopClosure::startLoopClosure_12(KeyFrame* cur_kf)
{
  try
  {

      int old_index;
//      cout<<"测试是不是每一帧都参与回环检测？"<<cur_kf->global_index<<endl;
     
      //可以先只做一个简单的判断，不要检测时间一致性
      //这里返回的是分数超过阈值的
      vector<cv::Point2f> cur_pts;
      vector<cv::Point2f> old_pts;
      std::vector<cv::KeyPoint> keys=cur_kf->keypoints;
      std::vector<BRIEF::bitset> descriptors=cur_kf->descriptors;
      int best_id=demo.run_3("BRIEF", keys, descriptors, cur_pts, old_pts, old_index, cur_kf->m_bowvec, cur_kf->m_featvec);
      cur_kf->check_loop=true;
      treeId_kf[kfNum_tree]= cur_kf->global_index;
      kfNum_tree++;

      //检测成功后，检测共视帧的一致性
      //选出分数最高的3帧作为侯选帧，并计算一级共视帧的分数（最多选5个），求和，根据这个找到匹配性最高的侯选帧
    if(best_id!=-1){

        
        //这里直接加这么多也是有问题的 我想想 改好了
        int real_old_best_id=treeId_kf[best_id];
        KeyFrame* old_kf;
        old_kf= poseGraph->getKeyframe(real_old_best_id);
       
        //        记录dbow每次检测到的回环
//        {
//            string kfNum_tree_dir="/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/1/";
//            std::ofstream outFile;
//            outFile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
//            outFile.precision(16);
//            //ios::binary|
//            int agentId=cur_kf->c->getId();
//            kfNum_tree_dir+=to_string(agentId);
//            outFile.open(kfNum_tree_dir+"_dbow.txt",ios::out|ios::app);
////                    cout<<"kfNum_tree_dir="<<kfNum_tree_dir+"intra_loop.txt" <<endl;
//            outFile<< real_old_best_id<<" , "<<cur_kf->global_index<<"\n";
//
//            //关闭文件
//            outFile.close();
//        }

        
        //这个是不是可以用两帧 计算匹配点，计算基础矩阵 最终的特征点数量判断
        std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
        std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标

        
        //存储一下当前帧和候选帧的匹配关系
        std::vector<cv::Point2f> measurements_old_norm_real;//图像坐标
        std::vector<Vector3d> point_3d_cur_real;
      
        measurements_old_norm_real.clear();
        point_3d_cur_real.clear();

        measurements_old_coarse.clear();
        measurements_old_norm_coarse.clear();
//        cout<<"dbow检测到回环帧id"<<cur_kf->global_index<<" , "<<real_old_best_id<<endl;
//        新帧3d点
        cur_kf->findConnectionWithOldFrame_server_old(old_kf, measurements_old_coarse, measurements_old_norm_coarse);
        if(measurements_old_coarse.size()<16){
//            cout<<"地图内部 当前帧与侯选帧的匹配点数少于22，"<<measurements_old_coarse.size()<<", "<<cur_kf->global_index<<endl;
            return  false;
        }
//实验记录
        vector<cv::Point2f> measurements_cur;
        measurements_cur=old_kf->measurements;
        point_3d_cur_real=cur_kf->point_clouds;
        measurements_old_norm_real=measurements_old_norm_coarse;
        
        
        
//        用老帧的3d点
//        old_kf->findConnectionWithOldFrame_server_old(cur_kf, measurements_old_coarse, measurements_old_norm_coarse);
//        if(measurements_old_coarse.size()<16){
////            cout<<"地图内部 当前帧与侯选帧的匹配点数少于22，"<<measurements_old_coarse.size()<<", "<<cur_kf->global_index<<endl;
//            return  false;
//        }
//        point_3d_cur_real=old_kf->point_clouds;
//        measurements_old_norm_real=measurements_old_norm_coarse;
        

        //记录当前帧的共视帧 能够和候选帧的共视帧匹配上的数量
        
//        cout<<"地图内部 第一次粗糙找到的匹配点数:"<<point_3d_cur_real.size()<<endl;
        relative_mutex.lock();
        mlvv_point_clouds_single.push_back(point_3d_cur_real);
        mlvv_measurements_old_norm_single.push_back(measurements_old_norm_real );//measurements_old_norm_real
        mlp_kf_pairs.push_back(std::make_pair(cur_kf, old_kf));
//        mlp_kf_pairs.push_back(std::make_pair(old_kf ,cur_kf ));
        relative_mutex.unlock();
//        cout<<"检测到回环"<<cur_kf->global_index<<" , "<<old_kf->global_index<<" , "<<point_3d_cur_real.size()<<endl;
        return true;
        
        //        记录最终的dbow回环
//        {
//            string kfNum_tree_dir="/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/1/";
//            std::ofstream outFile;
//            outFile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
//            outFile.precision(16);
//            //ios::binary|
//            int agentId=cur_kf->c->getId();
//            kfNum_tree_dir+=to_string(agentId);
//            outFile.open(kfNum_tree_dir+"_geoloop.txt",ios::out|ios::app);
////                    cout<<"kfNum_tree_dir="<<kfNum_tree_dir+"intra_loop.txt" <<endl;
//            outFile<< real_old_best_id<<" , "<<cur_kf->global_index<<"\n";
//
//            //关闭文件
//            outFile.close();
//        }
        
//        {
//         std::ofstream outFile;
//            outFile.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/1/1/"+
//                                   to_string(cur_kf->global_index)+"&"+to_string(cur_kf->header)+"&"+
//                                   to_string(old_kf->global_index)+"&"+to_string(old_kf->header)+"_allPoint.txt");
//            outFile << measurements_old_coarse.size()<<"\n";
//            for(int i=0,j=measurements_old_coarse.size(); i<j; i++)
//         {
//             cv::Point2f p=measurements_old_coarse[i];
//             Vector3d p3d=point_3d_cur_real[i];
//             //写入数据
//             outFile << p.x<<" "<<p.y<<" "<<p3d[0]<<" "<<p3d[1]<<" "<<p3d[2]<<"\n";
//         }
//
//         //关闭文件
//         outFile.close();
//        }
       
        
//       {
//        std::ofstream outFile;
//           outFile.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/1/1/"+
//                        to_string(old_kf->global_index)+"&"+to_string(old_kf->header)+"&"+to_string(cur_kf->global_index)+"_allPoint.txt");
//        for(auto iter=measurements_cur.begin(), iter_end=measurements_cur.end(); iter!=iter_end; iter++)
//        {
//            cv::Point2f p=(*iter);
//            //写入数据
//            outFile << p.x<<","<<p.y<<"\n";
//        }
//
//        //关闭文件
//        outFile.close();
//
//
//        std::ofstream outFile2;
//
//           outFile2.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/1/1/"+
//                        to_string(cur_kf->global_index)+"&"+to_string(cur_kf->header)+"&"+
//                        to_string(old_kf->global_index)+"_allPoint.txt");
//        for(auto iter=measurements_old_coarse.begin(), iter_end=measurements_old_coarse.end(); iter!=iter_end; iter++)
//        {
//            cv::Point2f p=(*iter);
//            //写入数据
//            outFile2 << p.x<<","<<p.y<<"\n";
//        }
//
//        //关闭文件
//        outFile2.close();
//        }
      
    }else{
//        usleep(30);//论文3 实验1 是没有的，后面尝试加了这个
        return false;
    }
      
  }
  catch(const std::string &ex)
  {
    cout << "Error loop: " << ex << endl;
    return false;
  }
}



//和loopClosureRun_9实现细节一样 但是用老帧的3d点
void LoopClosure::loopClosureRun_12(){
    while(true){
        if(!mlp_kf_pairs.empty()){
            
            relative_mutex.lock();
            vector<cv::Point2f> measurements_old_norm_real=mlvv_measurements_old_norm_single.front();
            mlvv_measurements_old_norm_single.pop_front();
            vector<Eigen::Vector3d> point_3d_cur_real=mlvv_point_clouds_single.front();
            mlvv_point_clouds_single.pop_front();
            std::pair<KeyFrame*, KeyFrame*> kf_pair=mlp_kf_pairs.front();
            mlp_kf_pairs.pop_front();
            relative_mutex.unlock();

            KeyFrame* old_kf=kf_pair.first; //真正的当前帧
            KeyFrame* cur_kf =kf_pair.second;
            
            
//            int toGlobalOptiNum=mlp_kf_pairs.size();
//            relative_mutex.lock();
//            vector<cv::Point2f> measurements_old_norm_real=mlvv_measurements_old_norm_single.front();
//            mlvv_measurements_old_norm_single.pop_front();
//            vector<Eigen::Vector3d> point_3d_cur_real=mlvv_point_clouds_single.front();
//            mlvv_point_clouds_single.pop_front();
//            std::pair<KeyFrame*, KeyFrame*> kf_pair=mlp_kf_pairs.front();
//            mlp_kf_pairs.pop_front();
//
//            for(int i=1;i<toGlobalOptiNum;i++){
//                kf_pair=mlp_kf_pairs.front();
//                KeyFrame* old_kf_toOpti=kf_pair.first; //真正的当前帧
//                int x_cur=old_kf_toOpti->global_index;
//                if(x_cur-preLoopKfId<6){
//
//                    measurements_old_norm_real=mlvv_measurements_old_norm_single.front();
//                    mlvv_measurements_old_norm_single.pop_front();
//                    point_3d_cur_real=mlvv_point_clouds_single.front();
//                    mlvv_point_clouds_single.pop_front();
//                    mlp_kf_pairs.pop_front();
//
//                }else{
//                    break;
//                }
//
//            }
//            relative_mutex.unlock();
//            KeyFrame* old_kf=kf_pair.first; //真正的当前帧
//            KeyFrame* cur_kf =kf_pair.second;

            
//            cout<<endl<<"开始计算相对位姿"<<old_kf->global_index <<","<<getTime()<<endl;
            
            Client* client_cur=old_kf->c;
            Matrix3d ric_curClient=client_cur->ric_client;
            Vector3d tic_curClient=client_cur->tic_client;
            Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
            const float &fx = client_cur->FOCUS_LENGTH_X_server;
            const float &fy = client_cur->FOCUS_LENGTH_Y_server;
            const float &cx = client_cur->PX_server;
            const float &cy = client_cur->PY_server;
            
            Matrix3d R_relative;
            Vector3d T_relative;
//            Matrix3d oldKF_r;
//            Vector3d oldKF_t;
//            old_kf->getOriginPose(oldKF_t, oldKF_r);
            
            Matrix3d oldKF_r_flag;
            Vector3d oldKF_t_flag;
//            while(!old_kf->IsOriginUpdate){
//                usleep(50);
//            }
           
            cur_kf->getOriginPose(oldKF_t_flag, oldKF_r_flag);
//            cout<<"位姿稳定："<<old_kf->IsOriginUpdate<<endl;
            
//            ------------计算相对位姿法一
         /**
            Eigen::Matrix3d intrinsics;
            intrinsics<<fx,0,cx,0,fy,cy,0,0,1;
            string ground_truth_pose_path;//暂时不做处理
            int point_num=measurements_old_norm_real.size();
            cv::Mat points(point_num, 5, CV_64F);//u v x y z
            for(int i=0;i<point_num;i++){
                points.at<double>(i,0)=measurements_old_norm_real[i].x;
                points.at<double>(i,1)=measurements_old_norm_real[i].y;
                points.at<double>(i,2)=point_3d_cur_real[i][0];
                points.at<double>(i,3)=point_3d_cur_real[i][1];
                points.at<double>(i,4)=point_3d_cur_real[i][2];
            }
            string inlier_points_path_save="";
            const double confidence = 0.99;
            const double inlier_outlier_threshold_pnp = 5.50;
            const double spatial_coherence_weight = 0.975;
            const int fps = -1;
            Matrix3d r_cw;
            Vector3d t_cw;
            // Estimating the fundamental matrix by the Graph-Cut RANSAC algorithm
            std::vector<size_t> inlier=test6DPoseFitting2(
                              intrinsics,
                ground_truth_pose_path, // Path where the ground truth pose is found
                              points, // The path where the image and world points can be found
            inlier_points_path_save, // The path where the inlier points should be saved
                confidence, // The RANSAC confidence value
                inlier_outlier_threshold_pnp, // The used inlier-outlier threshold in GC-RANSAC.
                spatial_coherence_weight, // The weight of the spatial coherence term in the graph-cut energy minimization.
                20.0, // The radius of the neighborhood ball for determining the neighborhoods.
                fps,r_cw,t_cw); // The required FPS limit. If it is set to -1, the algorithm will not be interrupted before finishing.
            Matrix3d r_wc=r_cw.inverse();
            Vector3d t_wc=-r_wc*t_cw;
            Matrix3d r_ci=ric_curClient.inverse();
            Vector3d t_ci=-r_ci*tic_curClient;
//            R_relative=r_wc*r_ci;
//            T_relative=r_wc*t_ci+t_wc;
          */
//            ------------计算相对位姿法二
        /**
            if(inlier.size()>=3){
                {
                    //先验换一下 因为当前的位姿，是乘了偏移的
    //                Matrix3d curKF_r;
    //                Vector3d curKF_t;
    //                old_kf->getPose(curKF_t, curKF_r);
    //                oldKF_r=poseGraph->r_drift*oldKF_r;
    //                oldKF_t=poseGraph->r_drift*oldKF_t+poseGraph->t_drift;

                    ceres::Problem problem;
                    ceres::Solver::Options options;
                    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                    options.linear_solver_type = ceres::DENSE_QR;//DENSE_QR DENSE_SCHUR
                    options.trust_region_strategy_type = ceres::DOGLEG;
                    //options.minimizer_progress_to_stdout = true;
    //                options.max_num_iterations = 20;
                    ceres::Solver::Summary summary;
                    ceres::LossFunction *loss_function;
                    loss_function = new ceres::HuberLoss(1.0);
    //                loss_function = new ceres::CauchyLoss(1.0);

                    std::vector<ceres::ResidualBlockId> residual_block_ids;
                    ceres::ResidualBlockId              block_id;

                    //实验用 记录过滤了哪些
                    vector<int> point_all;



                    double t_array[3];//平移数组，其中存放每个关键帧的平移向量
                    double euler_array[3];


    //                t_array[0] = oldKF_t(0);
    //                t_array[1] = oldKF_t(1);
    //                t_array[2] = oldKF_t(2);
    //                //将矩阵转换为向量
    //                Vector3d euler_angle_old = Utility::R2ypr(oldKF_r);

                    t_array[0] = oldKF_t_flag(0);
                    t_array[1] = oldKF_t_flag(1);
                    t_array[2] = oldKF_t_flag(2);
                    //将矩阵转换为向量
                    Vector3d euler_angle_old = Utility::R2ypr(oldKF_r_flag);


                    euler_array[0] = euler_angle_old.x();
                    euler_array[1] = euler_angle_old.y();
                    euler_array[2] = euler_angle_old.z();
                    problem.AddParameterBlock(euler_array, 3);
                    problem.AddParameterBlock(t_array, 3);

//                    for(int a=0,b=point_3d_cur_real.size();a<b;a++){
                    for(int a=0,b=inlier.size();a<b;a++){
                        int  index=(int)(inlier[a]);
                        //找到主地图那个点 所在帧的位姿
                        Vector3d pts_i = point_3d_cur_real[index];

                        //相机平面坐标
                        cv::Point2f pt=measurements_old_norm_real[index];
                        float xx=pt.x;
                        float yy=pt.y;

//                        从像素坐标转到 归一化图像坐标
                        xx = (xx - cx)/fx;
                        yy = (yy - cy)/fy;

                         Vector2d pts_j ;//要求是归一化图像坐标
                         pts_j<<xx,yy;

                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection2::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z());
                        block_id =  problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
                        residual_block_ids.push_back( block_id );

                     }

                    //-------------临时 实验用------------------
    //                options.linear_solver_type = ceres::DENSE_SCHUR;
    //                options.max_num_iterations = 20;
    //                ceres::Solve( options, &problem, &summary );
    //                if(summary.termination_type!=ceres::
    //                   CONVERGENCE){
    //                    //这里记录是哪一帧，哪些特征点，featureId
    //
    //                    //这里通过ceres过滤一层，记录 特征点，featureId
    //                }

                    //----------------

                    std::vector< ceres::ResidualBlockId > residual_block_ids_temp;
                    residual_block_ids_temp.reserve( residual_block_ids.size() );

                 

                   
    //                cout<<"检测到回环 测试"<<cur_kf->global_index<<" , "<<old_kf->global_index<<endl;
    //                cout<<"过滤的点数"<<hh.size()<<" , "<< point_3d_cur_real.size()<<endl;
    //                assert(hh.size()==point_3d_cur_real.size());
//                    {
//                    //实验记录
//                    std::ofstream outFile;
//                    //打开文件
//    //                outFile.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/"+
//    //                             to_string(cur_kf->global_index)+"&"+to_string(cur_kf->header)+"&"+to_string(old_kf->global_index)+"&"+to_string(old_kf->header)+".txt");
//                    outFile.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/1/"+
//                                 to_string(cur_kf->global_index)+"&"+to_string(old_kf->global_index)+".txt");
//
//                    for(auto iter=point_all.begin(), iter_end=point_all.end(); iter!=iter_end; iter++)
//                    {
//                        //写入数据
//                        outFile << (*iter)<<" ";
//                    }
//                    outFile<<"\n";
//                        for(auto iter=hh.begin(), iter_end=hh.end(); iter!=iter_end; iter++)
//                        {
//                            //写入数据
//                            outFile << (*iter)<<",";
//                        }
//                    //关闭文件
//                    outFile.close();
//                    }

//                    {
//                    //实验记录
//                    std::ofstream outFile;
//                    //打开文件
//                    outFile.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/1/"+
//                                 to_string(cur_kf->global_index)+"&"+to_string(old_kf->global_index)+".txt");
//
//                    for(auto iter=inlier.begin(), iter_end=inlier.end(); iter!=iter_end; iter++)
//                    {
//                        //写入数据
//                        outFile << (*iter)<<" ";
//                    }
//                    outFile<<"\n";
////                        for(auto iter=hh.begin(), iter_end=hh.end(); iter!=iter_end; iter++)
////                        {
////                            //写入数据
////                            outFile << (*iter)<<",";
////                        }
//                    //关闭文件
//                    outFile.close();
//                    }

    //                cout<<"测试：residual_block_ids.size22= "<<residual_block_ids.size()<<endl;
                    options.linear_solver_type = ceres::DENSE_QR;//DENSE_QR DENSE_SCHUR
                    options.max_num_iterations = 20;
    //                options.minimizer_progress_to_stdout = false;
    //                options.check_gradients = false;
    //                options.gradient_check_relative_precision = 1e-10;

                    ceres::Solve( options, &problem, &summary );
    //                std::cout <<"地图内部 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";


                    if(summary.termination_type!=ceres::
                       CONVERGENCE){
                        continue;
                    }
    //                    cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false);
    //                }
    //                else{
                    //这里代表是 在当前帧的误差基础上，老帧的世界位姿
                        T_relative[0]=t_array[0];
                        T_relative[1]=t_array[1];
                        T_relative[2]=t_array[2];

                        R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));
    //                }
                }
            }else{
                cout<<"内点数量不足"<<inlier.size()<<endl;
                continue;
            }
         */
            
//            if(!old_kf->IsOriginUpdate){
//                string kfNum_tree_dir="/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/1/";
//                std::ofstream outFile;
//                outFile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
//                outFile.precision(16);
//                //ios::binary|
//                int agentId=cur_kf->c->getId();
//                kfNum_tree_dir+=to_string(agentId);
//                outFile.open(kfNum_tree_dir+"_noGood.txt",ios::out|ios::app);
////                    cout<<"kfNum_tree_dir="<<kfNum_tree_dir+"intra_loop.txt" <<endl;
//                outFile<< old_kf->global_index<<" , "<<cur_kf->global_index<<"\n";
//
//                //关闭文件
//                outFile.close();
//            }
//            ------------计算相对位姿法三 靠谱一点
          
            
            {
                //先验换一下 因为当前的位姿，是乘了偏移的
//                Matrix3d curKF_r;
//                Vector3d curKF_t;
//                old_kf->getPose(curKF_t, curKF_r);
//                oldKF_r=poseGraph->r_drift*oldKF_r;
//                oldKF_t=poseGraph->r_drift*oldKF_t+poseGraph->t_drift;
                
                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;//DENSE_QR DENSE_SCHUR
                options.trust_region_strategy_type = ceres::DOGLEG;
                //options.minimizer_progress_to_stdout = true;
//                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
//                loss_function = new ceres::CauchyLoss(1.0);
                
                std::vector<ceres::ResidualBlockId> residual_block_ids;
                ceres::ResidualBlockId              block_id;

                //实验用 记录过滤了哪些
                vector<int> point_all;
                
                
                
                double t_array[3];//平移数组，其中存放每个关键帧的平移向量
                double euler_array[3];
                
                
//                t_array[0] = oldKF_t(0);
//                t_array[1] = oldKF_t(1);
//                t_array[2] = oldKF_t(2);
//                //将矩阵转换为向量
//                Vector3d euler_angle_old = Utility::R2ypr(oldKF_r);
                
                t_array[0] = oldKF_t_flag(0);
                t_array[1] = oldKF_t_flag(1);
                t_array[2] = oldKF_t_flag(2);
                //将矩阵转换为向量
                Vector3d euler_angle_old = Utility::R2ypr(oldKF_r_flag);
                
                
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

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection2::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),100.0);
                    block_id =  problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
                    residual_block_ids.push_back( block_id );
                    
                 }

                //-------------临时 实验用------------------
//                options.linear_solver_type = ceres::DENSE_SCHUR;
//                options.max_num_iterations = 20;
//                ceres::Solve( options, &problem, &summary );
//                if(summary.termination_type!=ceres::
//                   CONVERGENCE){
//                    //这里记录是哪一帧，哪些特征点，featureId
//
//                    //这里通过ceres过滤一层，记录 特征点，featureId
//                }
                
                //----------------
                
                std::vector< ceres::ResidualBlockId > residual_block_ids_temp;
                residual_block_ids_temp.reserve( residual_block_ids.size() );

                
                vector<double> hh;
                for ( size_t ii = 0; ii < 1; ii++ )
                {
                    options.max_num_iterations = 2;
//                    options.minimizer_progress_to_stdout = false;
//                    options.check_gradients = false;
//                    options.gradient_check_relative_precision = 1e-10;
                    //options.function_tolerance = 1e-100; // default 1e-6


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
//                    cout << "Inlier threshold is: " << m_inlier_final_threshold << endl;
                    for ( unsigned int i = 0; i < residual_block_ids.size(); i++ )
                    {
                        hh.push_back(fabs( residuals[ 2 * i + 0 ] ) + fabs( residuals[ 2 * i + 1 ] ));
//                        cout<<"测试误差："<<fabs( residuals[ 2 * i + 0 ] ) + fabs( residuals[ 2 * i + 1 ] )<<endl;
                        if ( ( fabs( residuals[ 2 * i + 0 ] ) + fabs( residuals[ 2 * i + 1 ] ) ) > m_inlier_threshold ) // std::min( 1.0, 10 * avr_cost )
                        {
                            //screen_out << "Remove outliers, drop id = " << (void *)residual_block_ids[ i ] <<endl;
                            problem.RemoveResidualBlock( residual_block_ids[ i ] );
                            point_all.push_back(i);
                        }
                        else
                        {
                            residual_block_ids_temp.push_back( residual_block_ids[ i ] );
                        }
                    }
                    residual_block_ids = residual_block_ids_temp;
                }
//                cout<<"检测到回环 测试"<<cur_kf->global_index<<" , "<<old_kf->global_index<<endl;
//                cout<<"过滤的点数"<<hh.size()<<" , "<< point_3d_cur_real.size()<<endl;
//                assert(hh.size()==point_3d_cur_real.size());
//                {
//                //实验记录
//                std::ofstream outFile;
//                //打开文件
////                outFile.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/"+
////                             to_string(cur_kf->global_index)+"&"+to_string(cur_kf->header)+"&"+to_string(old_kf->global_index)+"&"+to_string(old_kf->header)+".txt");
//                outFile.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/1/"+
//                             to_string(cur_kf->global_index)+"&"+to_string(old_kf->global_index)+".txt");
//
//                for(auto iter=point_all.begin(), iter_end=point_all.end(); iter!=iter_end; iter++)
//                {
//                    //写入数据
//                    outFile << (*iter)<<" ";
//                }
//                outFile<<"\n";
//                    for(auto iter=hh.begin(), iter_end=hh.end(); iter!=iter_end; iter++)
//                    {
//                        //写入数据
//                        outFile << (*iter)<<",";
//                    }
//                //关闭文件
//                outFile.close();
//                }
                
//                cout<<"测试：residual_block_ids.size22= "<<residual_block_ids.size()<<endl;
                options.linear_solver_type = ceres::DENSE_SCHUR;//DENSE_QR DENSE_SCHUR
                options.max_num_iterations = 20;
//                options.minimizer_progress_to_stdout = false;
//                options.check_gradients = false;
//                options.gradient_check_relative_precision = 1e-10;

                ceres::Solve( options, &problem, &summary );
//                std::cout <<"地图内部 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";
                

                if(summary.termination_type!=ceres::
                   CONVERGENCE){
//                    std::cout <<"地图内部 算一个粗糙的相对位姿不收敛\n";
                    poseGraph->hasDetectLoopKfId_mutex.lock();
                    poseGraph->hasDetectLoopKfId=old_kf->global_index;
//                    cout<<"记录当前检测过的帧id="<<poseGraph->hasDetectLoopKfId<<endl;
                    poseGraph->hasDetectLoopKfId_mutex.unlock();
                    
                    continue;
                }
//                    cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false);
//                }
//                else{
                old_kf->loop_edge=residual_block_ids.size();
                old_kf->loop_error= summary.final_cost;
                //这里代表是 在当前帧的误差基础上，老帧的世界位姿
                T_relative[0]=t_array[0];
                T_relative[1]=t_array[1];
                T_relative[2]=t_array[2];

                R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));
                
                Eigen::Matrix3d curKF_r_flag;
                Eigen::Vector3d curKF_t_flag;
                old_kf->getOriginPose(curKF_t_flag, curKF_r_flag);
                
                R_relative=curKF_r_flag.transpose()*R_relative;
                T_relative=curKF_r_flag.transpose()*(T_relative-curKF_t_flag);
                R_relative=oldKF_r_flag*R_relative.transpose();
                T_relative=-R_relative*T_relative+oldKF_t_flag;
//                }
            }
             
            vector<vector<cv::Point2f>> measurements_old_norm_all;
            vector<vector<Eigen::Vector3d>> point_clouds_all;
            std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
            std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
            std::vector<Vector3d> point_3d_cur;
            std::vector<cv::Point2f> measurements_cur;//像素坐标
            
            
            std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
            vpCovKFi_cur[0] = cur_kf;
            
            
//            while(!old_kf->IsOriginUpdate){
//                usleep(50);
//            }
            
//            cout<<"cur_kf id:"<<cur_kf->global_index<<" , " << old_kf->global_index<<endl;
            vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
//            可能存在和第0帧共视了 第0帧没有共视帧
            if(vpCovKFi_old.size()==0){
                vpCovKFi_old.push_back(old_kf);
            }else{
                vpCovKFi_old.push_back(vpCovKFi_old[0]);//为空
                vpCovKFi_old[0]=old_kf;
            }
            
            
//            list<KeyFrame*> lpCovKFi_old;
//            for(KeyFrame* pkf:vpCovKFi_old){
//                lpCovKFi_old.push_back(pkf);
//            }
            
            vector<Matrix3d> oldR_b_a;
            vector<Vector3d> oldT_b_a;
            
            Matrix3d oldKF_r;
            Vector3d oldKF_t;
            old_kf->getOriginPose(oldKF_t, oldKF_r);
            
            
            int temp_index=0;
            //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
            for(KeyFrame* pkFi_old: vpCovKFi_old){
                if(temp_index!=0){
                    Matrix3d rwi_old;
                    Vector3d twi_old;
                    pkFi_old->getOriginPose(twi_old, rwi_old);
                    
                    Matrix3d r_b_a;
                    Vector3d t_b_a;
                    r_b_a=oldKF_r.transpose()* rwi_old;
                    t_b_a=oldKF_r.transpose()*(twi_old-oldKF_t);
                    oldR_b_a.push_back(r_b_a);
                    oldT_b_a.push_back(t_b_a);
                }else{
                    oldR_b_a.push_back(Matrix3d::Identity());
                    oldT_b_a.push_back(Vector3d::Zero());
                    temp_index++;//放这里就只要执行一次
                }
            }
            
            int similarNum=0;
            temp_index=0;
            vector<Matrix3d> old_r;
            vector<Vector3d> old_t;
            vector<int> vpkf_index;
            
            for(KeyFrame* pkf_old:vpCovKFi_old){
                Matrix3d r_w_imuOld=R_relative*oldR_b_a[temp_index];
                Vector3d t_w_imuOld=R_relative*oldT_b_a[temp_index] +T_relative;
                Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                Matrix3d r_camOld_w=r_w_camOld.transpose();
                old_r.push_back(r_camOld_w);
                old_t.push_back(-r_camOld_w*t_w_camOld);
                temp_index++;
                
            }
            
            vector<vector<int>> feature_id_old_all;
            vector<double> header_old_all;
//            vector<vector<Eigen::Vector3d>> point_clouds_all;
            vector<int> feature_id_old;
            
            {
                //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
                
                point_clouds_all.clear();
                measurements_old_norm_all.clear();
                vpkf_index.clear();
                header_old_all.clear();
                point_3d_cur.clear();
                feature_id_old_all.clear();
                
                
//                cout<<"当前帧的共视帧：";
                
//                cout<<endl<<endl;
                for(KeyFrame* pKFi : vpCovKFi_cur){
//                    cout<<pKFi->global_index<<" ,";
                    //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
//                    if(pKFi==cur_kf ){
//                        point_clouds_all.push_back(point_3d_cur_real);
//                        measurements_old_norm_all.push_back(measurements_old_norm_real);
//                        vpkf_index.push_back(0);
////                        old_kf->isuse=1;
////                        temp_index++;
//                        continue;
//                    }
                    //描述符得全有
                    if(!pKFi->is_des_end){
                        continue;
                    }
                    
                    vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                    std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                    vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                    vector<int > feature_id_origin_cur=pKFi->features_id_origin;
                    int nPoints = point_clouds_origin_cur.size();
                    int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                   
    //                int num=0;//记录匹配点的数量
                    temp_index=0;//记录遍历到哪个帧了
                    //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                    for(KeyFrame* pkFi_old: vpCovKFi_old)
                    {
                        
//                        if(pKFi==cur_kf && pkFi_old==old_kf){
//                            point_clouds_all.push_back(point_3d_cur_real);
//                            measurements_old_norm_all.push_back(measurements_old_norm_real);
//                            vpkf_index.push_back(0);
//                            temp_index++;
//                            continue;
//                        }
                        //描述符得全有
                        if(!pkFi_old->is_des_end){
                            temp_index++;
                            continue;
                        }
                        
                        measurements_old_coarse.clear();
                        measurements_old_norm_coarse.clear();
                        point_3d_cur.clear();
                        measurements_cur.clear();
                        feature_id_old.clear();
                    
                        vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;
                        vector<int > feature_id_origin_old=pkFi_old->features_id_origin;
                        int nPoints_old = feature_id_origin_old.size();
                        int point2D_len_old=keypoints_old.size()-nPoints_old;

                        //这个得到的是到imu坐标系的位姿
                        Matrix3d r_camOld_w=old_r[temp_index];
                        Vector3d t_camOld_w=old_t[temp_index];
                        
                        for(int i=0;i<nPoints;i++){
                            Vector3d point_main=point_clouds_origin_cur[i];
                            //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                            Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                            //深度必须为正
                            if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
    //                            cout<<"深度不为正"<<endl;
                                continue;
                            }
                            // 投影到图像上
                            double x = p3D_c2[0];
                            double y = p3D_c2[1];
                            double z = p3D_c2[2];
                            //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                            double u=fx*x/z+cx;
                            double v=fy*y/z+cy;
                          
                            
                            if(!pkFi_old->isInImage((float)u, (float)v)){
    //                            cout<<"投影点不在图像内"<<endl;
                                continue;
                            }
                            
                            const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 20);
                            //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                            if(vIndices.empty()){
    //                            cout<<"半径为50个像素 找不到点"<<endl;
                                continue;
                            }
                            //des和keypoints长度不一样
                            //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                            BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                            int bestDist = 256;
                            int bestIndex = -1;
                            for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                            {
            //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                                int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                                if(dis < bestDist)
                                {
                                    bestDist = dis;
                                    bestIndex = *vit;
                                }
                            }
                            
                            if(bestDist<70){
//                                cout<<"最终选择的半径范围："<<u-keypoints_old[bestIndex].pt.x<<" ,"<<v-keypoints_old[bestIndex].pt.y<<endl;
                                point_3d_cur.push_back(point_main);
                                measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                                measurements_cur.push_back(measurements_origin_cur[i]);
                                
                                if(bestIndex<point2D_len_old){
//                                    feature_id_old.push_back(-1);//后续加上去
                                    
                                    feature_id_old.push_back(feature_id_origin_cur[i]);
                                    
                                }else{
//                                    int index=bestIndex-point2D_len_old;
//                                    feature_id_old.push_back(feature_id_origin_old[index]);
                                    
                                    feature_id_old.push_back(-1);
                                }
                            }
                                        
                        }
                        
//                        cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                        if(measurements_cur.size()>=22){
                            pKFi->rejectWithF_server_mapFuse2(measurements_cur, measurements_old_coarse,point_3d_cur,feature_id_old);
//                            cout<<"f矩阵拒绝后的点数："<<measurements_old_coarse.size()<<endl;
                            if(measurements_old_coarse.size()>=16){
                            
                                cv::Point2f norm_pt;
                                int pCount=measurements_old_coarse.size();
                                for(int aa=0;aa<pCount;aa++){
                                    norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                    norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                    measurements_old_coarse[aa]=norm_pt;
                                    
                                }
                                
                                point_clouds_all.push_back(point_3d_cur);
                                measurements_old_norm_all.push_back(measurements_old_coarse);
                                
                                header_old_all.push_back(pkFi_old->header);
                                feature_id_old_all.push_back(feature_id_old);
    //                            num+=point_3d_cur.size();
    //                            real_vpCovKFi_cur.push_back(pkFi_old);
                                vpkf_index.push_back(temp_index);
                                similarNum++;
//                                pkFi_old->isuse=1;
//                                cout<<"两帧匹配上："<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<", "<<measurements_old_coarse.size()<<". "<<endl;
//                                break;
                            }
                            
                        }
                        
                        temp_index++;
    //                    ends=clock();
    //                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                    }
                }
            }
            
//            for(KeyFrame* pkFi_old: vpCovKFi_old)
//            {
//                pkFi_old->isuse=0;
//            }
        
            if(similarNum>=3){
                int optiKf_num=vpkf_index.size()+2;
                //构造优化问题
                double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
                Quaterniond q_array[optiKf_num];
                double euler_array[optiKf_num][3];

                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                //options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
        //            ceres::LossFunction *loss_function_feature;
        //            loss_function_feature = new ceres::CauchyLoss(1.0);
                //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
                ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

                
                Matrix3d curKF_r;
                Vector3d curKF_t;
                old_kf->getOriginPose(curKF_t, curKF_r);
                
                
                Matrix3d tmp_r_old;
                Vector3d tmp_t_old;
                t_array[0][0] = T_relative(0);
                t_array[0][1] = T_relative(1);
                t_array[0][2] = T_relative(2);
                //将矩阵转换为向量
                Eigen::Vector3d euler_angle_old = Utility::R2ypr(R_relative);
                
//                t_array[0][0] = curKF_t(0);
//                t_array[0][1] = curKF_t(1);
//                t_array[0][2] = curKF_t(2);
//                //将矩阵转换为向量
//                Vector3d euler_angle_old = Utility::R2ypr(curKF_r);
                euler_array[0][0] = euler_angle_old.x();
                euler_array[0][1] = euler_angle_old.y();
                euler_array[0][2] = euler_angle_old.z();
                problem.AddParameterBlock(euler_array[0], 3);
                problem.AddParameterBlock(t_array[0], 3);

                
                //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
                map<int,int> resample;
                
                temp_index=0;
                int residual_totalSize=0;
                for(int len=vpkf_index.size(); temp_index<len; temp_index++ ){
                    
                    int kf_index=temp_index+1;
                    Matrix3d relative_r_b_a;
                    Vector3d relative_t_b_a;
                    relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
                    relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
                    Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                    
                    map<int,int>::iterator iter;
                    iter = resample.find(vpkf_index[temp_index]);
                    if(iter != resample.end())
                    {
                        kf_index=resample[vpkf_index[temp_index]];
                    }
                    else
                    {
                        resample[vpkf_index[temp_index]]=kf_index;

                        t_array[kf_index][0] = relative_t_b_a(0);
                        t_array[kf_index][1] = relative_t_b_a(1);
                        t_array[kf_index][2] = relative_t_b_a(2);
                        euler_array[kf_index][0] = relative_r_b_a_euler.x();
                        euler_array[kf_index][1] = relative_r_b_a_euler.y();
                        euler_array[kf_index][2] = relative_r_b_a_euler.z();
                        problem.AddParameterBlock(euler_array[kf_index], 1, angle_local_parameterization);
                        problem.AddParameterBlock(t_array[kf_index], 3);
                        
//                        if(vpkf_index[temp_index]==0){
//                            problem.SetParameterBlockConstant(euler_array[kf_index]);
//                            problem.SetParameterBlockConstant(t_array[kf_index]);
//                        }
                       
                        ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),1);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                        
                    }

                    vector<Vector3d> point_single=point_clouds_all[temp_index];
                    vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
                    for(int a=0,b=point_single.size();a<b;a++){

                        //找到主地图那个点 所在帧的位姿
                        Vector3d pts_i = point_single[a];

                        //相机平面坐标
                        cv::Point2f pt=measure_single[a];

                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pt.x, pt.y, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                        
                        residual_totalSize++;
                        
                     }
                }
                
                
                ceres::Solve(options, &problem, &summary);
//                std::cout <<"地图内部 精确相对位姿："<< summary.BriefReport() << "\n";

                if(summary.termination_type==ceres::CONVERGENCE){
                   
                    old_kf->loop_edge_final=residual_totalSize;
                    old_kf->loop_error_final= summary.final_cost;
//                    cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;

                    Matrix3d Rs_i ;
                    Vector3d Ps_i ;//当前帧
                    cur_kf->getOriginPose(Ps_i, Rs_i);
                    
                    Vector3d q;
                    q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                    Matrix3d Rs_loop = Utility::ypr2R(q);
                    Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);

//                    {
//                     std::ofstream outFile;
//                        outFile.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/1/"+
//                                               to_string(old_kf->global_index)+"&"+
//                                               to_string(cur_kf->global_index)+"_rt.txt");
//
//                        for(int i=0; i<3; i++)
//                     {
//
//                         //写入数据
//                         outFile << Rs_loop(i,0)<<" "<<Rs_loop(i,1)<<" "<<Rs_loop(i,2)<<" "<<Ps_loop[i]<<"\n";
//                     }
//
//                     //关闭文件
//                     outFile.close();
//                    }

//                    Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
////                    Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
//                    double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
//                    double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
//                    double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
//
//                    Quaterniond rs_loop_q=Quaterniond(Rs_loop),rs_i_q=Quaterniond(Rs_i);
//                    Quaterniond rela_q=rs_loop_q.inverse()*rs_i_q;
//
//
//                    cur_kf->relative_pitch=relative_pitch;
//                    cur_kf->relative_roll=relative_roll;
//                    cur_kf->updateLoopConnection(relative_t, relative_yaw);
//                    cur_kf->loop_info_better_q=rela_q;

                    
                    
                    Vector3d relative_t = Rs_i.transpose() * (Ps_loop - Ps_i);
                    double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).x() - Utility::R2ypr(Rs_i).x());
                    double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).y() - Utility::R2ypr(Rs_i).y());
                    double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).z() - Utility::R2ypr(Rs_i).z());
                    
                    Quaterniond rs_loop_q=Quaterniond(Rs_loop),rs_i_q=Quaterniond(Rs_i);
                    Quaterniond rela_q=rs_i_q.inverse()*rs_loop_q;
                    
                    
                    old_kf->relative_pitch=relative_pitch;
                    old_kf->relative_roll=relative_roll;
                    old_kf->updateLoopConnection(relative_t, relative_yaw);
                    old_kf->loop_info_better_q=rela_q;
                    
                  
                    
//                    更新数据
                    old_index=cur_kf->global_index;
                    Vector3d T_w_i_old;
                    Matrix3d R_w_i_old;
                    cur_kf->getOriginPose(T_w_i_old, R_w_i_old);
                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = old_kf->global_index;
                    retrive_data.header = old_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    
                    //前后相对位姿误差
                    Eigen::Matrix3d rela_error=R_w_i_old.transpose()*Rs_loop;
                    Eigen::Vector3d rela_angle=Utility::R2ypr(rela_error);//角度
                    Eigen::Vector3d rela_t=R_w_i_old.transpose()*(Ps_loop-T_w_i_old);
                    double relative_pitch_error = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).y() - Utility::R2ypr(R_w_i_old).y());
                    double relative_roll_error = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).z() - Utility::R2ypr(R_w_i_old).z());
                    old_kf->rela_t=rela_t;
                    old_kf->rela_angle<<relative_pitch_error,relative_roll_error;
                    
                    for (int k = 0; k < 3; k++){
                        retrive_data.loop_pose[k]=Ps_loop[k];//把回环优化后的位姿 告诉回环记录的
                    }
                    retrive_data.loop_pose[3]=rs_loop_q.x();
                    retrive_data.loop_pose[4]=rs_loop_q.y();
                    retrive_data.loop_pose[5]=rs_loop_q.z();
                    retrive_data.loop_pose[6]=rs_loop_q.w();
                    
                    
        //            Quaterniond Q_loop_old(Rs_loop);
        //            retrive_data.P_old = Ps_loop;
        //            retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
//                    retrive_data.measurements = measurements_old_norm_all[0];
                    //这里暂时不给值  因为暂时不发送到客户端
        //                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
        //            retrive_data.features_ids = feature_id_cur_all[0];//当前帧的匹配的特征点的全局id
                    retrive_data.old_index=cur_kf->global_index;

//                    retrive_data.header_all=header_old_all;
                    retrive_data.features_ids_all=feature_id_old_all;//暂时注释
////                    retrive_data.measurements_all=measurements_old_norm_all;
                    retrive_data.point_clouds_all=point_clouds_all;
//                    retrive_data.P_old_all=ps_loop_all;
//                    retrive_data.Q_old_all=rs_loop_all;

                    vins->retrive_pose_data = (retrive_data);
//                                vins->isSendLoopData=true;

                    old_kf->detectLoop(cur_kf->global_index);
                    poseGraph->addLoop(cur_kf->global_index);//todo 这里面 改一下
                    cur_kf->is_looped = 1;
                    loop_old_index = cur_kf->global_index;
                    
                    vins->isSendLoop_another=true;
                    if(old_kf->IsOriginUpdate==true){
                       if (old_kf->sendLoop==false)
                       {
                           old_kf->sendLoop=true;
                           vins->globalOpti_index_mutex.lock();
                           vins->kf_global_index.push(old_kf->global_index);
                           vins->start_kf_global_index.push(old_kf->loop_index);
                           poseGraph->latest_loop_index=old_kf->global_index;
                           vins->start_global_optimization = true;//先暂停回环 不启动

                           vins->globalOpti_index_mutex.unlock();
//                           cout<<"检测到回环 并通过错误回环的检测："<<old_kf->global_index<<" , "<<getTime()<<endl;
                       }
                   }
                    old_kf->is_get_loop_info=true;
                    preLoopKfId=old_kf->global_index;
//                    cout<<" vins->isSendLoop_another2="<< vins->isSendLoop_another2<<endl;
//                    cout<<"计算相对位姿："<<old_kf->global_index<<" , "<<cur_kf->global_index<<endl;

                
                    
//                    //        ----------子地图分配 调用allocateSubMap----------
//                    relative_mutex.lock();
//                    vector<int> old_vector=old_vector_forSubMap.front();
//                    old_vector_forSubMap.pop_front();
//                    relative_mutex.unlock();
//                    global_featureMap->allocateSubMap(cur_kf, old_kf->global_index,old_vector);
//                    //        ----保存地图内部的回环------
//                    string kfNum_tree_dir="/Users/zhangjianhua/Desktop/2022paper/mh/gt/";
//                    std::ofstream outFile;
//                    outFile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
//                    outFile.precision(16);
//                    //ios::binary|
//                    int agentId=cur_kf->c->getId();
//                    kfNum_tree_dir+=to_string(agentId);
//                    outFile.open(kfNum_tree_dir+"_intra_loop.txt",ios::out|ios::app);
////                    cout<<"kfNum_tree_dir="<<kfNum_tree_dir+"intra_loop.txt" <<endl;
//                    outFile<<cur_kf->global_index<<" , "<<cur_kf->header<<" , "<<old_kf->global_index<<" , "<<old_kf->header<<"\n";
//
//                    //关闭文件
//                    outFile.close();
//                    cout<<"保存保存了"<<endl;
                    
                }
//                else{
//                    cout<<"地图内部 检测失败 未收敛： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
//                    
//                }
            }
            
            poseGraph->hasDetectLoopKfId_mutex.lock();
            poseGraph->hasDetectLoopKfId=old_kf->global_index;
//            cout<<"记录当前检测过的帧id="<<poseGraph->hasDetectLoopKfId<<endl;
            poseGraph->hasDetectLoopKfId_mutex.unlock();
//            else{
//                cout<<"地图内部 回环检测的数量不够:"<<similarNum<<endl;
//            }
        }
        else{
            usleep(10);//论文3 实验1之前是10 ， 后面尝试30
        }
    }

}



//把下面的计算 也加一个过滤
void LoopClosure::loopClosureRun_10(){
    while(true){
        if(!mlp_kf_pairs.empty()){
            relative_mutex.lock();
            vector<cv::Point2f> measurements_old_norm_real=mlvv_measurements_old_norm_single.front();
            mlvv_measurements_old_norm_single.pop_front();
            vector<Eigen::Vector3d> point_3d_cur_real=mlvv_point_clouds_single.front();
            mlvv_point_clouds_single.pop_front();
            std::pair<KeyFrame*, KeyFrame*> kf_pair=mlp_kf_pairs.front();
            mlp_kf_pairs.pop_front();
            KeyFrame* cur_kf=kf_pair.first;
            KeyFrame* old_kf=kf_pair.second;
            relative_mutex.unlock();
            
            
            Client* client_cur=old_kf->c;
            Matrix3d ric_curClient=client_cur->ric_client;
            Vector3d tic_curClient=client_cur->tic_client;
            Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
            const float &fx = client_cur->FOCUS_LENGTH_X_server;
            const float &fy = client_cur->FOCUS_LENGTH_Y_server;
            const float &cx = client_cur->PX_server;
            const float &cy = client_cur->PY_server;
            
            Matrix3d R_relative;
            Vector3d T_relative;
            Matrix3d oldKF_r;
            Vector3d oldKF_t;
            old_kf->getOriginPose(oldKF_t, oldKF_r);
            
            {
                //先验换一下 因为当前的位姿，是乘了偏移的
//                Matrix3d curKF_r;
//                Vector3d curKF_t;
//                old_kf->getPose(curKF_t, curKF_r);
//                oldKF_r=poseGraph->r_drift*oldKF_r;
//                oldKF_t=poseGraph->r_drift*oldKF_t+poseGraph->t_drift;
                
                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                options.trust_region_strategy_type = ceres::DOGLEG;
                //options.minimizer_progress_to_stdout = true;
//                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
//                loss_function = new ceres::CauchyLoss(1.0);
                
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

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection2::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),100.0);
                    block_id =  problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
                    residual_block_ids.push_back( block_id );
                 }

                
                std::vector< ceres::ResidualBlockId > residual_block_ids_temp;
                residual_block_ids_temp.reserve( residual_block_ids.size() );

                // Drop some of the residual to guaruntee the real time performance.
                if ( residual_block_ids.size() > (size_t) 1e5 )
                {
                    residual_block_ids_temp.clear();

                    float  threshold_to_reserve = ( float ) 1e5 / ( float ) residual_block_ids.size();
                    float *probability_to_drop = rand_array_uniform( 0, 1.0, residual_block_ids.size() );
//                    screen_out << "Number of residual blocks too Large, drop them to " << m_maximum_allow_residual_block << endl;
                    for ( size_t i = 0; i < residual_block_ids.size(); i++ )
                    {
                        if ( probability_to_drop[ i ] > threshold_to_reserve )
                        {
                            problem.RemoveResidualBlock( residual_block_ids[ i ] );
                        }
                        else
                        {
                            residual_block_ids_temp.push_back( residual_block_ids[ i ] );
                        }
                    }
                    residual_block_ids = residual_block_ids_temp;
                    delete probability_to_drop;
                }

//                cout<<"测试：residual_block_ids.size11= "<<residual_block_ids.size()<<endl;
                for ( size_t ii = 0; ii < 1; ii++ )
                {
                    options.max_num_iterations = 2;
//                    options.minimizer_progress_to_stdout = false;
//                    options.check_gradients = false;
//                    options.gradient_check_relative_precision = 1e-10;
                    //options.function_tolerance = 1e-100; // default 1e-6

                    
//                    set_ceres_solver_bound( problem, t_array );
                    ceres::Solve( options, &problem, &summary );
//                    std::cout <<"算一个粗糙的相对位姿1："<< summary.BriefReport() << "\n";

                    residual_block_ids_temp.clear();
                    ceres::Problem::EvaluateOptions eval_options;
                    eval_options.residual_blocks = residual_block_ids;
                    double              total_cost = 0.0;
                    std::vector<double> residuals;
                    problem.Evaluate( eval_options, &total_cost, &residuals, nullptr, nullptr );
                    
                    double m_inliner_ratio_threshold = compute_inlier_residual_threshold( residuals, 0.8 );
                    double m_inlier_threshold = std::max( 0.02, m_inliner_ratio_threshold );
//                    cout<<"m_inlier_threshold="<<m_inlier_threshold<<endl;
                    //screen_out << "Inlier threshold is: " << m_inlier_final_threshold << endl;
                    for ( unsigned int i = 0; i < residual_block_ids.size(); i++ )
                    {
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
//                cout<<"测试：residual_block_ids.size22= "<<residual_block_ids.size()<<endl;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                options.max_num_iterations = 20;
//                options.minimizer_progress_to_stdout = false;
//                options.check_gradients = false;
//                options.gradient_check_relative_precision = 1e-10;

                ceres::Solve( options, &problem, &summary );
                std::cout <<"地图内部 算一个粗糙的相对位姿2："<< summary.BriefReport() << "\n";
                

                if(summary.termination_type!=ceres::
                   CONVERGENCE){
                    
                    continue;
                }
//                    cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false);
//                }
//                else{
                    T_relative[0]=t_array[0];
                    T_relative[1]=t_array[1];
                    T_relative[2]=t_array[2];

                    R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));
//                }
            }
            
            vector<vector<cv::Point2f>> measurements_old_norm_all;
            vector<vector<Eigen::Vector3d>> point_clouds_all;
            std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
            std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
            std::vector<Vector3d> point_3d_cur;
            std::vector<cv::Point2f> measurements_cur;//像素坐标
            
            
            vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_old.push_back(vpCovKFi_old[0]);
            vpCovKFi_old[0]=old_kf;
            
//            list<KeyFrame*> lpCovKFi_old;
//            for(KeyFrame* pkf:vpCovKFi_old){
//                lpCovKFi_old.push_back(pkf);
//            }
            
            vector<Matrix3d> oldR_b_a;
            vector<Vector3d> oldT_b_a;
            
            int temp_index=0;
            //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
            for(KeyFrame* pkFi_old: vpCovKFi_old){
                if(temp_index!=0){
                    Matrix3d rwi_old;
                    Vector3d twi_old;
                    pkFi_old->getOriginPose(twi_old, rwi_old);
                    
                    Matrix3d r_b_a;
                    Vector3d t_b_a;
                    r_b_a=oldKF_r.transpose()* rwi_old;
                    t_b_a=oldKF_r.transpose()*(twi_old-oldKF_t);
                    oldR_b_a.push_back(r_b_a);
                    oldT_b_a.push_back(t_b_a);
                }else{
                    oldR_b_a.push_back(Matrix3d::Identity());
                    oldT_b_a.push_back(Vector3d::Zero());
                    temp_index++;//放这里就只要执行一次
                }
            }
            
            int similarNum=0;
            temp_index=0;
            vector<Matrix3d> old_r;
            vector<Vector3d> old_t;
            vector<int> vpkf_index;
            
            for(KeyFrame* pkf_old:vpCovKFi_old){
                Matrix3d r_w_imuOld=R_relative*oldR_b_a[temp_index];
                Vector3d t_w_imuOld=R_relative*oldT_b_a[temp_index] +T_relative;
                Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                Matrix3d r_camOld_w=r_w_camOld.transpose();
                old_r.push_back(r_camOld_w);
                old_t.push_back(-r_camOld_w*t_w_camOld);
                temp_index++;
                
            }
            
            while(!cur_kf->IsOriginUpdate){
                usleep(50);
            }
            std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
            vpCovKFi_cur[0] = cur_kf;
            
            
            {
                //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
                
                point_clouds_all.clear();
                measurements_old_norm_all.clear();
                vpkf_index.clear();
//                cout<<"当前帧的共视帧：";
                
//                cout<<endl<<endl;
                for(KeyFrame* pKFi : vpCovKFi_cur){
//                    cout<<pKFi->global_index<<" ,";
                    //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
//                    if(pKFi==cur_kf ){
//                        point_clouds_all.push_back(point_3d_cur_real);
//                        measurements_old_norm_all.push_back(measurements_old_norm_real);
//                        vpkf_index.push_back(0);
////                        old_kf->isuse=1;
////                        temp_index++;
//                        continue;
//                    }
                    //描述符得全有
                    if(!pKFi->is_des_end){
                        continue;
                    }
                    
                    vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                    std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                    vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                    int nPoints = point_clouds_origin_cur.size();
                    int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                   
    //                int num=0;//记录匹配点的数量
                    temp_index=0;//记录遍历到哪个帧了
                    //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                    for(KeyFrame* pkFi_old: vpCovKFi_old)
                    {
                        
//                        if(pKFi==cur_kf && pkFi_old==old_kf){
//                            point_clouds_all.push_back(point_3d_cur_real);
//                            measurements_old_norm_all.push_back(measurements_old_norm_real);
//                            vpkf_index.push_back(0);
//                            temp_index++;
//                            continue;
//                        }
                        //描述符得全有
                        if(!pkFi_old->is_des_end){
                            temp_index++;
                            continue;
                        }
                        
                        measurements_old_coarse.clear();
                        measurements_old_norm_coarse.clear();
                        point_3d_cur.clear();
                        measurements_cur.clear();
                    
                        vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                        //这个得到的是到imu坐标系的位姿
                        Matrix3d r_camOld_w=old_r[temp_index];
                        Vector3d t_camOld_w=old_t[temp_index];
                        
                        for(int i=0;i<nPoints;i++){
                            Vector3d point_main=point_clouds_origin_cur[i];
                            //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                            Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                            //深度必须为正
                            if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
    //                            cout<<"深度不为正"<<endl;
                                continue;
                            }
                            // 投影到图像上
                            double x = p3D_c2[0];
                            double y = p3D_c2[1];
                            double z = p3D_c2[2];
                            //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                            double u=fx*x/z+cx;
                            double v=fy*y/z+cy;
                          
                            
                            if(!pkFi_old->isInImage((float)u, (float)v)){
    //                            cout<<"投影点不在图像内"<<endl;
                                continue;
                            }
                            
                            const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 20);
                            //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                            if(vIndices.empty()){
    //                            cout<<"半径为50个像素 找不到点"<<endl;
                                continue;
                            }
                            //des和keypoints长度不一样
                            //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                            BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                            int bestDist = 256;
                            int bestIndex = -1;
                            for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                            {
            //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                                int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                                if(dis < bestDist)
                                {
                                    bestDist = dis;
                                    bestIndex = *vit;
                                }
                            }
                            
                            if(bestDist<70){
//                                cout<<"最终选择的半径范围："<<u-keypoints_old[bestIndex].pt.x<<" ,"<<v-keypoints_old[bestIndex].pt.y<<endl;
                                point_3d_cur.push_back(point_main);
                                measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                                measurements_cur.push_back(measurements_origin_cur[i]);
                                
                            }
                                        
                        }
                        
//                        cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                        if(measurements_cur.size()>=22){
                            pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
//                            cout<<"f矩阵拒绝后的点数："<<measurements_cur.size()<<", "<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<endl;
                            if(measurements_old_coarse.size()>=22){
                            
                                cv::Point2f norm_pt;
                                int pCount=measurements_old_coarse.size();
                                for(int aa=0;aa<pCount;aa++){
                                    norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                    norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                    measurements_old_coarse[aa]=norm_pt;
                                    
                                }
                                
                                point_clouds_all.push_back(point_3d_cur);
                                measurements_old_norm_all.push_back(measurements_old_coarse);
    //                            num+=point_3d_cur.size();
    //                            real_vpCovKFi_cur.push_back(pkFi_old);
                                vpkf_index.push_back(temp_index);
                                similarNum++;
//                                pkFi_old->isuse=1;
//                                cout<<"两帧匹配上："<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<", "<<measurements_old_coarse.size()<<". "<<endl;
//                                break;
                            }
                            
                        }
                        
                        temp_index++;
    //                    ends=clock();
    //                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                    }
                }
            }
            
//            for(KeyFrame* pkFi_old: vpCovKFi_old)
//            {
//                pkFi_old->isuse=0;
//            }
        
            if(similarNum>=3){
                int optiKf_num=vpkf_index.size()+2;
                //构造优化问题
                double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
                Quaterniond q_array[optiKf_num];
                double euler_array[optiKf_num][3];

                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                //options.minimizer_progress_to_stdout = true;
//                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
        //            ceres::LossFunction *loss_function_feature;
        //            loss_function_feature = new ceres::CauchyLoss(1.0);
                //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
                ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();
                
                std::vector<ceres::ResidualBlockId> residual_block_ids;
                ceres::ResidualBlockId              block_id;

                
                Matrix3d curKF_r;
                Vector3d curKF_t;
                old_kf->getOriginPose(curKF_t, curKF_r);
                
                
                Matrix3d tmp_r_old;
                Vector3d tmp_t_old;
                t_array[0][0] = curKF_t(0);
                t_array[0][1] = curKF_t(1);
                t_array[0][2] = curKF_t(2);
                //将矩阵转换为向量
                Vector3d euler_angle_old = Utility::R2ypr(curKF_r);
                euler_array[0][0] = euler_angle_old.x();
                euler_array[0][1] = euler_angle_old.y();
                euler_array[0][2] = euler_angle_old.z();
                problem.AddParameterBlock(euler_array[0], 3);
                problem.AddParameterBlock(t_array[0], 3);

                
                //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
                map<int,int> resample;
                
                temp_index=0;
                for(int len=vpkf_index.size(); temp_index<len; temp_index++ ){
                    
                    int kf_index=temp_index+1;
                    Matrix3d relative_r_b_a;
                    Vector3d relative_t_b_a;
                    relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
                    relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
                    Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                    
                    map<int,int>::iterator iter;
                    iter = resample.find(vpkf_index[temp_index]);
                    if(iter != resample.end())
                    {
                        kf_index=resample[vpkf_index[temp_index]];
                    }
                    else
                    {
                        resample[vpkf_index[temp_index]]=kf_index;

                        t_array[kf_index][0] = relative_t_b_a(0);
                        t_array[kf_index][1] = relative_t_b_a(1);
                        t_array[kf_index][2] = relative_t_b_a(2);
                        euler_array[kf_index][0] = relative_r_b_a_euler.x();
                        euler_array[kf_index][1] = relative_r_b_a_euler.y();
                        euler_array[kf_index][2] = relative_r_b_a_euler.z();
                        problem.AddParameterBlock(euler_array[kf_index], 1, angle_local_parameterization);
                        problem.AddParameterBlock(t_array[kf_index], 3);
                       
                        ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),1);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                        
                        
                    }

                    vector<Vector3d> point_single=point_clouds_all[temp_index];
                    vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
                    for(int a=0,b=point_single.size();a<b;a++){

                        //找到主地图那个点 所在帧的位姿
                        Vector3d pts_i = point_single[a];

                        //相机平面坐标
                        cv::Point2f pt=measure_single[a];

                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pt.x, pt.y, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                        block_id =problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                        residual_block_ids.push_back( block_id );
                     }
                }
                
                
                std::vector< ceres::ResidualBlockId > residual_block_ids_temp;
                residual_block_ids_temp.reserve( residual_block_ids.size() );
                
                if ( residual_block_ids.size() > (size_t) 1e5 )
                {
                    residual_block_ids_temp.clear();

                    float  threshold_to_reserve = ( float ) 1e5 / ( float ) residual_block_ids.size();
                    float *probability_to_drop = rand_array_uniform( 0, 1.0, residual_block_ids.size() );
//                    screen_out << "Number of residual blocks too Large, drop them to " << m_maximum_allow_residual_block << endl;
                    for ( size_t i = 0; i < residual_block_ids.size(); i++ )
                    {
                        if ( probability_to_drop[ i ] > threshold_to_reserve )
                        {
                            problem.RemoveResidualBlock( residual_block_ids[ i ] );
                        }
                        else
                        {
                            residual_block_ids_temp.push_back( residual_block_ids[ i ] );
                        }
                    }
                    residual_block_ids = residual_block_ids_temp;
                    delete probability_to_drop;
                }
                
                
                for ( size_t ii = 0; ii < 1; ii++ )
                {
                    options.max_num_iterations = 2;
//                    options.minimizer_progress_to_stdout = false;
//                    options.check_gradients = false;
//                    options.gradient_check_relative_precision = 1e-10;
                    //options.function_tolerance = 1e-100; // default 1e-6

                    
//                    set_ceres_solver_bound( problem, t_array );
                    ceres::Solve( options, &problem, &summary );
//                    std::cout <<"算一个粗糙的相对位姿1："<< summary.BriefReport() << "\n";

                    residual_block_ids_temp.clear();
                    ceres::Problem::EvaluateOptions eval_options;
                    eval_options.residual_blocks = residual_block_ids;
                    double              total_cost = 0.0;
                    std::vector<double> residuals;
                    problem.Evaluate( eval_options, &total_cost, &residuals, nullptr, nullptr );
                    
                    double m_inliner_ratio_threshold = compute_inlier_residual_threshold( residuals, 0.8 );
                    double m_inlier_threshold = std::max( 0.02, m_inliner_ratio_threshold );
//                    cout<<"m_inlier_threshold="<<m_inlier_threshold<<endl;
                    //screen_out << "Inlier threshold is: " << m_inlier_final_threshold << endl;
                    for ( unsigned int i = 0; i < residual_block_ids.size(); i++ )
                    {
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
//                cout<<"测试：residual_block_ids.size22= "<<residual_block_ids.size()<<endl;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                options.max_num_iterations = 20;
                
                ceres::Solve(options, &problem, &summary);
                std::cout <<"精确相对位姿："<< summary.BriefReport() << "\n";

                if(summary.termination_type==ceres::CONVERGENCE){
                   
                        
                    cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;

                    Matrix3d Rs_i ;
                    Vector3d Ps_i ;//当前帧
                    cur_kf->getOriginPose(Ps_i, Rs_i);
                    
      


                    Vector3d q;
                    q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                    Matrix3d Rs_loop = Utility::ypr2R(q);
                    Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
                    
//                    Ps_loop = poseGraph->r_drift * Ps_loop + poseGraph->t_drift;
//                    Rs_loop = poseGraph->r_drift * Rs_loop;
                    
        //                cout<<"优化计算得到的相对位姿："<<Ps_loop[0]<<" ,"<<Ps_loop[1]<<" ,"<<Ps_loop[2]<<" ,"<<euler_array[0][0]<<" ,"<<euler_array[0][1]<<" ,"<<euler_array[0][2]<<endl;

                    Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
        //                Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                    double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                    double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                    double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                    cur_kf->relative_pitch=relative_pitch;
                    cur_kf->relative_roll=relative_roll;
                    cur_kf->updateLoopConnection(relative_t, relative_yaw);


                    //先临时搬过来
                    old_index=old_kf->global_index;
                    Vector3d T_w_i_old;
                    Matrix3d R_w_i_old;
                    old_kf->getPose(T_w_i_old, R_w_i_old);

                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm_all[0];
                    //这里暂时不给值  因为暂时不发送到客户端
    //                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
                    retrive_data.old_index=old_kf->global_index;
                    vins->retrive_pose_data = (retrive_data);
        //                        vins->isSendLoopData=true;

                    cur_kf->detectLoop(old_kf->global_index);
                    poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                    old_kf->is_looped = 1;
                    loop_old_index = old_kf->global_index;
                    vins->isSendLoop_another=true;
                    if(cur_kf->IsOriginUpdate==true){
                       if (cur_kf->sendLoop==false)
                       {

                           cur_kf->sendLoop=true;
                           vins->globalOpti_index_mutex.lock();
                           vins->kf_global_index.push(cur_kf->global_index);
                           vins->start_kf_global_index.push(cur_kf->loop_index);
                           poseGraph->latest_loop_index=cur_kf->global_index;
                           vins->start_global_optimization = true;//先暂停回环 不启动

                           vins->globalOpti_index_mutex.unlock();
//                           cout<<"慢了 慢了检测到回环 并通过错误回环的检测："<<cur_kf->global_index<<endl;
                       }
                   }
                    cur_kf->is_get_loop_info=true;
                    
                }
                else{
                    cout<<"检测失败 未收敛： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
                    
                }
            }else{
                cout<<"回环检测的数量不够"<<endl;
            }
        }
        usleep(10);
    }

}


void LoopClosure::loopClosureRun_7(){
    while(true){
        if(!mlp_kf_pairs.empty()){
            relative_mutex.lock();
            //这里是 新帧的2d点
            vector<cv::Point2f> measurements_old_norm_real=mlvv_measurements_old_norm_single.front();
            mlvv_measurements_old_norm_single.pop_front();
            //这里其实是 老帧的3d点
            vector<Eigen::Vector3d> point_3d_cur_real=mlvv_point_clouds_single.front();
            mlvv_point_clouds_single.pop_front();
            std::pair<KeyFrame*, KeyFrame*> kf_pair=mlp_kf_pairs.front();
            mlp_kf_pairs.pop_front();
            KeyFrame* cur_kf=kf_pair.first;
            KeyFrame* old_kf=kf_pair.second;
            relative_mutex.unlock();
            
            Client* client_cur=cur_kf->c;
            Matrix3d ric_curClient=client_cur->ric_client;
            Vector3d tic_curClient=client_cur->tic_client;
            Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
            const float &fx = client_cur->FOCUS_LENGTH_X_server;
            const float &fy = client_cur->FOCUS_LENGTH_Y_server;
            const float &cx = client_cur->PX_server;
            const float &cy = client_cur->PY_server;
            
            Matrix3d R_relative;
            Vector3d T_relative;
//            Matrix3d oldKF_r;
//            Vector3d oldKF_t;
//            old_kf->getOriginPose(oldKF_t, oldKF_r);
            Matrix3d curKF_r;
            Vector3d curKF_t;
            cur_kf->getOriginPose(curKF_t, curKF_r);
            {
                
                
                
                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                //options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
    //            loss_function = new ceres::CauchyLoss(1.0);

                double t_array[3];//平移数组，其中存放每个关键帧的平移向量
                double euler_array[3];
                t_array[0] = curKF_t(0);
                t_array[1] = curKF_t(1);
                t_array[2] = curKF_t(2);
                //将矩阵转换为向量
                Vector3d euler_angle_old = Utility::R2ypr(curKF_r);
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

                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection2::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z(),100.0);
                    problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
                 }


                ceres::Solve(options, &problem, &summary);
                std::cout <<"地图内部 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";

                if(summary.termination_type!=ceres::
                   CONVERGENCE){
                    continue;
                }

                T_relative[0]=t_array[0];
                T_relative[1]=t_array[1];
                T_relative[2]=t_array[2];

                R_relative=Utility::ypr2R(Vector3d(euler_array[0],euler_array[1],euler_array[2]));
               
            }
            
            
            
            
            vector<vector<cv::Point2f>> measurements_old_norm_all;
            vector<vector<Eigen::Vector3d>> point_clouds_all;
            std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
            std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
            std::vector<Vector3d> point_3d_cur;
            std::vector<cv::Point2f> measurements_cur;//像素坐标
            
            
            vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_old.push_back(vpCovKFi_old[0]);
            vpCovKFi_old[0]=old_kf;
            
            std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
            vpCovKFi_cur[0] = cur_kf;
            
            
            vector<Matrix3d> oldR_b_a;
            vector<Vector3d> oldT_b_a;
            
            int temp_index=0;
            //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
            for(KeyFrame* pkFi_old: vpCovKFi_cur){
                if(temp_index!=0){
                    Matrix3d rwi_old;
                    Vector3d twi_old;
                    pkFi_old->getOriginPose(twi_old, rwi_old);
                    
                    Matrix3d r_b_a;
                    Vector3d t_b_a;
                    r_b_a=curKF_r.transpose()* rwi_old;
                    t_b_a=curKF_r.transpose()*(twi_old-curKF_t);
                    oldR_b_a.push_back(r_b_a);
                    oldT_b_a.push_back(t_b_a);
                }else{
                    oldR_b_a.push_back(Matrix3d::Identity());
                    oldT_b_a.push_back(Vector3d::Zero());
                    temp_index++;//放这里就只要执行一次
                }
            }
            
            int similarNum=0;
            temp_index=0;
            vector<Matrix3d> old_r;
            vector<Vector3d> old_t;
            vector<int> vpkf_index;
            {
                cout<<"候选帧的共视帧：";
                for(KeyFrame* pkf_old:vpCovKFi_cur){
                    cout<<pkf_old->global_index<<" ,";
                    Matrix3d r_w_imuOld=R_relative*oldR_b_a[temp_index];
                    Vector3d t_w_imuOld=R_relative*oldT_b_a[temp_index] +T_relative;
                    Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                    Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                    Matrix3d r_camOld_w=r_w_camOld.transpose();
                    old_r.push_back(r_camOld_w);
                    old_t.push_back(-r_camOld_w*t_w_camOld);
                    temp_index++;
                    
                }
                cout<<endl;
                //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
                
                point_clouds_all.clear();
                measurements_old_norm_all.clear();
                vpkf_index.clear();
                cout<<"当前帧的共视帧：";
                for(KeyFrame* pKFi : vpCovKFi_old){
                    cout<<pKFi->global_index<<" ,";
                    //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
                    if(pKFi->global_index==old_kf->global_index){
                        point_clouds_all.push_back(point_3d_cur_real);
                        measurements_old_norm_all.push_back(measurements_old_norm_real);
                        vpkf_index.push_back(0);
                        continue;
                    }
                    //描述符得全有
                    if(!pKFi->is_des_end){
                        continue;
                    }
                    
                    vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                    std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                    vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                    int nPoints = point_clouds_origin_cur.size();
                    int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                   
    //                int num=0;//记录匹配点的数量
                    temp_index=0;//记录遍历到哪个帧了
                    //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                    for(KeyFrame* pkFi_old: vpCovKFi_cur){
                        //描述符得全有
                        if(!pkFi_old->is_des_end){
                            temp_index++;
                            continue;
                        }
                        
                        measurements_old_coarse.clear();
                        measurements_old_norm_coarse.clear();
                        point_3d_cur.clear();
                        measurements_cur.clear();
                    
                        vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                        //这个得到的是到imu坐标系的位姿
                        Matrix3d r_camOld_w=old_r[temp_index];
                        Vector3d t_camOld_w=old_t[temp_index];
                        
                        for(int i=0;i<nPoints;i++){
                            Vector3d point_main=point_clouds_origin_cur[i];
                            //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                            Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                            //深度必须为正
                            if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
    //                            cout<<"深度不为正"<<endl;
                                continue;
                            }
                            // 投影到图像上
                            double x = p3D_c2[0];
                            double y = p3D_c2[1];
                            double z = p3D_c2[2];
                            //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                            double u=fx*x/z+cx;
                            double v=fy*y/z+cy;
                          
                            
                            if(!pkFi_old->isInImage((float)u, (float)v)){
    //                            cout<<"投影点不在图像内"<<endl;
                                continue;
                            }
                            
                            const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 60);
                            //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                            if(vIndices.empty()){
    //                            cout<<"半径为50个像素 找不到点"<<endl;
                                continue;
                            }
                            //des和keypoints长度不一样
                            //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                            BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                            int bestDist = 256;
                            int bestIndex = -1;
                            for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                            {
            //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                                int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                                if(dis < bestDist)
                                {
                                    bestDist = dis;
                                    bestIndex = *vit;
                                }
                            }
                            
                            if(bestDist<=60){
                                point_3d_cur.push_back(point_main);
                                measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                                measurements_cur.push_back(measurements_origin_cur[i]);
                                
                            }
                                        
                        }
                        
    //                    cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                        if(measurements_cur.size()>=15){
                            pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
    //                        cout<<"f矩阵拒绝后的点数："<<measurements_cur.size()<<", "<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<endl;
                            if(measurements_old_coarse.size()>=15){
                            
                                cv::Point2f norm_pt;
                                int pCount=measurements_old_coarse.size();
                                for(int aa=0;aa<pCount;aa++){
                                    norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                    norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                    measurements_old_coarse[aa]=norm_pt;
                                    
                                }
                                
                                point_clouds_all.push_back(point_3d_cur);
                                measurements_old_norm_all.push_back(measurements_old_coarse);
    //                            num+=point_3d_cur.size();
    //                            real_vpCovKFi_cur.push_back(pkFi_old);
                                vpkf_index.push_back(temp_index);
                                similarNum++;
                                cout<<"两帧匹配上："<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<", "<<measurements_old_coarse.size()<<". "<<endl;
                                break;
                            }
                            
                        }
                        
                        temp_index++;
    //                    ends=clock();
    //                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                    }
                }
            }
        
            if(similarNum>=2){
                int optiKf_num=vpkf_index.size()+2;
                //构造优化问题
                double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
                Quaterniond q_array[optiKf_num];
                double euler_array[optiKf_num][3];

                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                //options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
        //            ceres::LossFunction *loss_function_feature;
        //            loss_function_feature = new ceres::CauchyLoss(1.0);
                //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
                ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

                
//                Matrix3d curKF_r;
//                Vector3d curKF_t;
//                old_kf->getOriginPose(curKF_t, curKF_r);
                
                
                Matrix3d tmp_r_old;
                Vector3d tmp_t_old;
                t_array[0][0] = curKF_t(0);
                t_array[0][1] = curKF_t(1);
                t_array[0][2] = curKF_t(2);
                //将矩阵转换为向量
                Vector3d euler_angle_old = Utility::R2ypr(curKF_r);
                euler_array[0][0] = euler_angle_old.x();
                euler_array[0][1] = euler_angle_old.y();
                euler_array[0][2] = euler_angle_old.z();
                problem.AddParameterBlock(euler_array[0], 3);
                problem.AddParameterBlock(t_array[0], 3);

                
                //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
                map<int,int> resample;
                
                temp_index=0;
                for(int len=vpkf_index.size(); temp_index<len; temp_index++ ){
                    
                    int kf_index=temp_index+1;
                    Matrix3d relative_r_b_a;
                    Vector3d relative_t_b_a;
                    relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
                    relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
                    Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                    
                    map<int,int>::iterator iter;
                    iter = resample.find(vpkf_index[temp_index]);
                    if(iter != resample.end())
                    {
                        kf_index=resample[vpkf_index[temp_index]];
                    }
                    else
                    {
                        resample[vpkf_index[temp_index]]=kf_index;

                        t_array[kf_index][0] = relative_t_b_a(0);
                        t_array[kf_index][1] = relative_t_b_a(1);
                        t_array[kf_index][2] = relative_t_b_a(2);
                        euler_array[kf_index][0] = relative_r_b_a_euler.x();
                        euler_array[kf_index][1] = relative_r_b_a_euler.y();
                        euler_array[kf_index][2] = relative_r_b_a_euler.z();
                        problem.AddParameterBlock(euler_array[kf_index], 1, angle_local_parameterization);
                        problem.AddParameterBlock(t_array[kf_index], 3);
                        ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),1);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                    }

                    vector<Vector3d> point_single=point_clouds_all[temp_index];
                    vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
                    for(int a=0,b=point_single.size();a<b;a++){

                        //找到主地图那个点 所在帧的位姿
                        Vector3d pts_i = point_single[a];

                        //相机平面坐标
                        cv::Point2f pt=measure_single[a];
//                        float xx=pt.x;
//                        float yy=pt.y;
//
//                         Vector2d pts_j ;//要求是归一化图像坐标
//                         pts_j<<xx,yy;

                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pt.x, pt.y, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                        
                     }
                }
                 
//                Matrix3d Rs_i ;
//                Vector3d Ps_i ;//当前帧
//                cur_kf->getPose(Ps_i, Rs_i);
//                Vector3d r_cur=Utility::R2ypr(Rs_i);
//                int  cur_eulerId=vpkf_index.size()+1;
//                t_array[cur_eulerId][0] = Ps_i(0);
//                t_array[cur_eulerId][1] = Ps_i(1);
//                t_array[cur_eulerId][2] = Ps_i(2);
//                euler_array[cur_eulerId][0] = r_cur.x();
//                euler_array[cur_eulerId][1] = r_cur.y();
//                euler_array[cur_eulerId][2] = r_cur.z();
//
//                vector<Vector3d> point_3d_cur=cur_kf->point_clouds_origin;
//                vector<cv::Point2f> measure_2d_cur=cur_kf->measurements_origin;
//                for(int i=0, len=point_3d_cur.size();i<len;i++){
//                    //找到主地图那个点 所在帧的位姿
//                    Vector3d pts_i = point_3d_cur[i];
//
//                    //相机平面坐标
//                    cv::Point2f pt=measure_2d_cur[i];
//                    float xx=pt.x;
//                    float yy=pt.y;
//
//                    xx=(xx-cx)/fx;
//                    yy=(yy-cy)/fy;
//
//                    ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection4::Create( pts_i.x(), pts_i.y(), pts_i.z(),xx, yy, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z());
//                    problem.AddResidualBlock(cost_function, loss_function, euler_array[cur_eulerId],t_array[cur_eulerId]);
//
//                }
                
                ceres::Solve(options, &problem, &summary);
                std::cout <<"精确相对位姿："<< summary.BriefReport() << "\n";

                if(summary.termination_type==ceres::CONVERGENCE){
                   
                        
                    cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;

                    Matrix3d Rs_i ;
                    Vector3d Ps_i ;//当前帧
//                    Ps_i<<euler_array[cur_eulerId][0],euler_array[cur_eulerId][1],euler_array[cur_eulerId][2];
//                    Rs_i = Utility::ypr2R(Ps_i);
//                    Ps_i = Vector3d( t_array[cur_eulerId][0],  t_array[cur_eulerId][1],  t_array[cur_eulerId][2]);
                    old_kf->getOriginPose(Ps_i, Rs_i);
      


                    Vector3d q;
                    q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                    Matrix3d Rs_loop = Utility::ypr2R(q);
                    Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
                    
        //                cout<<"优化计算得到的相对位姿："<<Ps_loop[0]<<" ,"<<Ps_loop[1]<<" ,"<<Ps_loop[2]<<" ,"<<euler_array[0][0]<<" ,"<<euler_array[0][1]<<" ,"<<euler_array[0][2]<<endl;

//                    Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//        //                Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
//                    double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
//                    double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
//                    double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
//                    cur_kf->relative_pitch=relative_pitch;
//                    cur_kf->relative_roll=relative_roll;
//                    cur_kf->updateLoopConnection(relative_t, relative_yaw);
                    
                    
                    Vector3d relative_t = Rs_i.transpose() * (Ps_loop - Ps_i);
        //                Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                    double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).x() - Utility::R2ypr(Rs_i).x());
                    double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).y() - Utility::R2ypr(Rs_i).y());
                    double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).z() - Utility::R2ypr(Rs_i).z());
                    cur_kf->relative_pitch=relative_pitch;
                    cur_kf->relative_roll=relative_roll;
                    cur_kf->updateLoopConnection(relative_t, relative_yaw);


                    //先临时搬过来
                    old_index=old_kf->global_index;
                    Vector3d T_w_i_old;
                    Matrix3d R_w_i_old;
                    old_kf->getPose(T_w_i_old, R_w_i_old);

                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm_all[0];
                    //这里暂时不给值  因为暂时不发送到客户端
    //                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
                    retrive_data.old_index=old_kf->global_index;
                    vins->retrive_pose_data = (retrive_data);
        //                        vins->isSendLoopData=true;

                    cur_kf->detectLoop(old_kf->global_index);
                    poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                    old_kf->is_looped = 1;
                    loop_old_index = old_kf->global_index;
                    vins->isSendLoop_another=true;
                    if(cur_kf->IsOriginUpdate==true){
                       if (cur_kf->sendLoop==false)
                       {

                           cur_kf->sendLoop=true;
                           vins->globalOpti_index_mutex.lock();
                           vins->kf_global_index.push(cur_kf->global_index);
                           vins->start_kf_global_index.push(cur_kf->loop_index);
                           poseGraph->latest_loop_index=cur_kf->global_index;
                           vins->start_global_optimization = true;//先暂停回环 不启动

                           vins->globalOpti_index_mutex.unlock();
                           cout<<"慢了 慢了检测到回环 并通过错误回环的检测："<<cur_kf->global_index<<endl;
                       }
                   }
                    cur_kf->is_get_loop_info=true;
                    
                }
                else{
                    cout<<"检测失败 未收敛： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
                    
                }
            }
        }
        usleep(10);
    }

}



void LoopClosure::loopClosureRun_5(){
    while(true){
        if(!mlp_kf_pairs.empty()){
            relative_mutex.lock();
            vector<cv::Point2f> measurements_old_norm_real=mlvv_measurements_old_norm_single.front();
            mlvv_measurements_old_norm_single.pop_front();
            vector<Eigen::Vector3d> point_3d_cur_real=mlvv_point_clouds_single.front();
            mlvv_point_clouds_single.pop_front();
            vector<Matrix3d> oldR_b_a=mlv_oldR_b_a.front();
            mlv_oldR_b_a.pop_front();
            vector<Vector3d> oldT_b_a=mlv_oldT_b_a.front();
            mlv_oldT_b_a.pop_front();
            Matrix3d R_relative=ml_relative_r.front();
            ml_relative_r.pop_front();
            Vector3d T_relative=ml_relative_t.front();
            ml_relative_t.pop_front();
            std::pair<KeyFrame*, KeyFrame*> kf_pair=mlp_kf_pairs.front();
            mlp_kf_pairs.pop_front();
            KeyFrame* cur_kf=kf_pair.first;
            KeyFrame* old_kf=kf_pair.second;
            relative_mutex.unlock();
            
            Client* client_cur=old_kf->c;
            Matrix3d ric_curClient=client_cur->ric_client;
            Vector3d tic_curClient=client_cur->tic_client;
            Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
            const float &fx = client_cur->FOCUS_LENGTH_X_server;
            const float &fy = client_cur->FOCUS_LENGTH_Y_server;
            const float &cx = client_cur->PX_server;
            const float &cy = client_cur->PY_server;
            
            
            vector<vector<cv::Point2f>> measurements_old_norm_all;
            vector<vector<Eigen::Vector3d>> point_clouds_all;
            std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
            std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
            std::vector<Vector3d> point_3d_cur;
            std::vector<cv::Point2f> measurements_cur;//像素坐标
            
            
            vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_old.push_back(vpCovKFi_old[0]);
            vpCovKFi_old[0]=old_kf;
            
            std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
            vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
            vpCovKFi_cur[0] = cur_kf;
            
            int similarNum=0, temp_index=0;
            vector<Matrix3d> old_r;
            vector<Vector3d> old_t;
            vector<int> vpkf_index;
            {
                for(KeyFrame* pkf_old:vpCovKFi_old){
                    Matrix3d r_w_imuOld=R_relative*oldR_b_a[temp_index];
                    Vector3d t_w_imuOld=R_relative*oldT_b_a[temp_index] +T_relative;
                    Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                    Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                    Matrix3d r_camOld_w=r_w_camOld.transpose();
                    old_r.push_back(r_camOld_w);
                    old_t.push_back(-r_camOld_w*t_w_camOld);
                    temp_index++;
                    
                }
                //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
                
                point_clouds_all.clear();
                measurements_old_norm_all.clear();
                vpkf_index.clear();
                for(KeyFrame* pKFi : vpCovKFi_cur){
                    //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
                    if(pKFi==cur_kf){
                        point_clouds_all.push_back(point_3d_cur_real);
                        measurements_old_norm_all.push_back(measurements_old_norm_real);
                        vpkf_index.push_back(0);
                        continue;
                    }
                    //描述符得全有
                    if(!pKFi->is_des_end){
                        continue;
                    }
                    
                    vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                    std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                    vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                    int nPoints = point_clouds_origin_cur.size();
                    int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                   
    //                int num=0;//记录匹配点的数量
                    temp_index=0;//记录遍历到哪个帧了
                    //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                    for(KeyFrame* pkFi_old: vpCovKFi_old){
                        //描述符得全有
                        if(!pkFi_old->is_des_end){
                            temp_index++;
                            continue;
                        }
                        
                        measurements_old_coarse.clear();
                        measurements_old_norm_coarse.clear();
                        point_3d_cur.clear();
                        measurements_cur.clear();
                    
                        vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                        //这个得到的是到imu坐标系的位姿
                        Matrix3d r_camOld_w=old_r[temp_index];
                        Vector3d t_camOld_w=old_t[temp_index];
                        
                        for(int i=0;i<nPoints;i++){
                            Vector3d point_main=point_clouds_origin_cur[i];
                            //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                            Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                            //深度必须为正
                            if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
    //                            cout<<"深度不为正"<<endl;
                                continue;
                            }
                            // 投影到图像上
                            double x = p3D_c2[0];
                            double y = p3D_c2[1];
                            double z = p3D_c2[2];
                            //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                            double u=fx*x/z+cx;
                            double v=fy*y/z+cy;
                          
                            
                            if(!pkFi_old->isInImage((float)u, (float)v)){
    //                            cout<<"投影点不在图像内"<<endl;
                                continue;
                            }
                            
                            const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 60);
                            //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                            if(vIndices.empty()){
    //                            cout<<"半径为50个像素 找不到点"<<endl;
                                continue;
                            }
                            //des和keypoints长度不一样
                            //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                            BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                            int bestDist = 256;
                            int bestIndex = -1;
                            for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                            {
            //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                                int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                                if(dis < bestDist)
                                {
                                    bestDist = dis;
                                    bestIndex = *vit;
                                }
                            }
                            
                            if(bestDist<=60){
                                point_3d_cur.push_back(point_main);
                                measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                                measurements_cur.push_back(measurements_origin_cur[i]);
                                
                            }
                                        
                        }
                        
    //                    cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                        if(measurements_cur.size()>=15){
                            pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
    //                        cout<<"f矩阵拒绝后的点数："<<measurements_cur.size()<<", "<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<endl;
                            if(measurements_old_coarse.size()>=15){
                            
                                cv::Point2f norm_pt;
                                int pCount=measurements_old_coarse.size();
                                for(int aa=0;aa<pCount;aa++){
                                    norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                    norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                    measurements_old_coarse[aa]=norm_pt;
                                    
                                }
                                
                                point_clouds_all.push_back(point_3d_cur);
                                measurements_old_norm_all.push_back(measurements_old_coarse);
    //                            num+=point_3d_cur.size();
    //                            real_vpCovKFi_cur.push_back(pkFi_old);
                                vpkf_index.push_back(temp_index);
                                similarNum++;
                                
                                break;
                            }
                            
                        }
                        
                        temp_index++;
    //                    ends=clock();
    //                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                    }
                }
            }
        
            if(similarNum>=2){
                int optiKf_num=vpkf_index.size()+1;
                //构造优化问题
                double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
                Quaterniond q_array[optiKf_num];
                double euler_array[optiKf_num][3];

                ceres::Problem problem;
                ceres::Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                //options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 20;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
        //            ceres::LossFunction *loss_function_feature;
        //            loss_function_feature = new ceres::CauchyLoss(1.0);
                //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
                ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

                
                Matrix3d curKF_r;
                Vector3d curKF_t;
                old_kf->getPose(curKF_t, curKF_r);
                
                
                Matrix3d tmp_r_old;
                Vector3d tmp_t_old;
                t_array[0][0] = curKF_t(0);
                t_array[0][1] = curKF_t(1);
                t_array[0][2] = curKF_t(2);
                //将矩阵转换为向量
                Vector3d euler_angle_old = Utility::R2ypr(curKF_r);
                euler_array[0][0] = euler_angle_old.x();
                euler_array[0][1] = euler_angle_old.y();
                euler_array[0][2] = euler_angle_old.z();
                problem.AddParameterBlock(euler_array[0], 3);
                problem.AddParameterBlock(t_array[0], 3);

                
                //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
                map<int,int> resample;
                
                temp_index=0;
                for(int len=vpkf_index.size(); temp_index<len; temp_index++ ){
                    
                    int kf_index=temp_index+1;
                    Matrix3d relative_r_b_a;
                    Vector3d relative_t_b_a;
                    relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
                    relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
                    Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                    
                    map<int,int>::iterator iter;
                    iter = resample.find(vpkf_index[temp_index]);
                    if(iter != resample.end())
                    {
                        kf_index=resample[vpkf_index[temp_index]];
                    }
                    else
                    {
                        resample[vpkf_index[temp_index]]=kf_index;

                        t_array[kf_index][0] = relative_t_b_a(0);
                        t_array[kf_index][1] = relative_t_b_a(1);
                        t_array[kf_index][2] = relative_t_b_a(2);
                        euler_array[kf_index][0] = relative_r_b_a_euler.x();
                        euler_array[kf_index][1] = relative_r_b_a_euler.y();
                        euler_array[kf_index][2] = relative_r_b_a_euler.z();
                        problem.AddParameterBlock(euler_array[kf_index], 1, angle_local_parameterization);
                        problem.AddParameterBlock(t_array[kf_index], 3);
                        ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),1);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                    }

                    vector<Vector3d> point_single=point_clouds_all[temp_index];
                    vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
                    for(int a=0,b=point_single.size();a<b;a++){

                        //找到主地图那个点 所在帧的位姿
                        Vector3d pts_i = point_single[a];

                        //相机平面坐标
                        cv::Point2f pt=measure_single[a];
                        float xx=pt.x;
                        float yy=pt.y;

                         Vector2d pts_j ;//要求是归一化图像坐标
                         pts_j<<xx,yy;

                        ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                        
                     }
                }
                 
                
                ceres::Solve(options, &problem, &summary);
                std::cout <<"精确相对位姿："<< summary.BriefReport() << "\n";

                if(summary.termination_type==ceres::CONVERGENCE){
                    //重新计算一遍
                    //投影
                    T_relative<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                    R_relative= Utility::ypr2R(T_relative);
                    T_relative = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
        
                    
                    
                    /**
                    vector<Matrix3d> oldR_b_a_opti=oldR_b_a;
                    vector<Vector3d> oldT_b_a_opti=oldT_b_a;
                    for(int i=0,len=vpkf_index.size(); i<len; i++){
                        int j=i+1;
                        Vector3d q;
                        q<<euler_array[j][0],euler_array[j][1],euler_array[j][2];
                        Matrix3d Rs_loop = Utility::ypr2R(q);
                        Vector3d Ps_loop = Vector3d( t_array[j][0],  t_array[j][1],  t_array[j][2]);
                        oldR_b_a_opti[vpkf_index[i]]=Rs_loop;
                        oldT_b_a_opti[vpkf_index[i]]=Ps_loop;
                        
                    }*/
                    
                    
                     similarNum=0;
                     temp_index=0;
                    {
                        
                        //这里oldR_b_a可以更新一下
                        for(KeyFrame* pkf_old:vpCovKFi_old){
                            Matrix3d r_w_imuOld=R_relative*oldR_b_a[temp_index];
                            Vector3d t_w_imuOld=R_relative*oldT_b_a[temp_index] +T_relative;
                            Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
                            Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
                            Matrix3d r_camOld_w=r_w_camOld.transpose();
                            old_r[temp_index]=r_camOld_w;
                            old_t[temp_index]=-r_camOld_w*t_w_camOld;
                            temp_index++;
                        }
                        //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
                        
                        
                        point_clouds_all.clear();
                        measurements_old_norm_all.clear();
                        vpkf_index.clear();
                        for(KeyFrame* pKFi : vpCovKFi_cur){
                            //因为第一帧是当前帧 所以跳过 这里待会改，记录前面的值
                            if(pKFi==cur_kf){
                                point_clouds_all.push_back(point_3d_cur_real);
                                measurements_old_norm_all.push_back(measurements_old_norm_real);
    //                            real_vpCovKFi_cur.push_back(old_kf);
                                vpkf_index.push_back(0);
    //                            temp_index++;
                                continue;
                            }
                            //描述符得全有
                            if(!pKFi->is_des_end){
                                continue;
                            }
                            
                            vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
                            std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
                            vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
                            int nPoints = point_clouds_origin_cur.size();
                            int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
                           
            //                int num=0;//记录匹配点的数量
                            temp_index=0;//记录遍历到哪个帧了
                            //这里应该遍历候选帧的共视帧 ，只要共视点数量超过一定阈值 22
                            for(KeyFrame* pkFi_old: vpCovKFi_old){
            //                    start=clock();
                                
                                //描述符得全有
                                if(!pkFi_old->is_des_end){
                                    temp_index++;
                                    continue;
                                }
                                
                                measurements_old_coarse.clear();
                                measurements_old_norm_coarse.clear();
                                point_3d_cur.clear();
                                measurements_cur.clear();
                                

                                vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;

                                //这个得到的是到imu坐标系的位姿
                                Matrix3d r_camOld_w=old_r[temp_index];
                                Vector3d t_camOld_w=old_t[temp_index];
                                
                                for(int i=0;i<nPoints;i++){
                                    Vector3d point_main=point_clouds_origin_cur[i];
                                    //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                                    Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
                                    //深度必须为正
                                    if(p3D_c2[2]<0.0 || p3D_c2[2]>30.0){
            //                            cout<<"深度不为正"<<endl;
                                        continue;
                                    }
                                    // 投影到图像上
                                    double x = p3D_c2[0];
                                    double y = p3D_c2[1];
                                    double z = p3D_c2[2];
                                    //这里可能得加一个畸变校正 应该可以不用去校正 把他投影在一个没有畸变的图像上
                                    double u=fx*x/z+cx;
                                    double v=fy*y/z+cy;
                                  
                                    
                                    if(!pkFi_old->isInImage((float)u, (float)v)){
            //                            cout<<"投影点不在图像内"<<endl;
                                        continue;
                                    }
                                    
                                    const vector<int> vIndices=pkFi_old->GetFeaturesInArea_1((float)u, (float)v, 40);
                                    //这里可以改为30个像素里面找不到 再扩大到50个像素 暂时未做
                                    if(vIndices.empty()){
            //                            cout<<"半径为50个像素 找不到点"<<endl;
                                        continue;
                                    }
                                    //des和keypoints长度不一样
                                    //找在这个半径内最相似的 这里可能有些问题 因为3D点的描述子 个人感觉有问题
                                    BRIEF::bitset point_main_des=pKFi->descriptors[point2D_len_cur+i];
                                    int bestDist = 256;
                                    int bestIndex = -1;
                                    for(vector<int>::const_iterator vit=vIndices.begin(), vit_end=vIndices.end(); vit!=vit_end; vit++)
                                    {
                    //                    cout<<"*vit"<<*vit<<endl;发现 keypoints数量不等于des数量
                                        int dis = pkFi_old->HammingDis(point_main_des, pkFi_old->descriptors[*vit]);
                                        if(dis < bestDist)
                                        {
                                            bestDist = dis;
                                            bestIndex = *vit;
                                        }
                                    }
                                    
                                    if(bestDist<=50){
                                        point_3d_cur.push_back(point_main);
                                        measurements_old_coarse.push_back(keypoints_old[bestIndex].pt);
                                        measurements_cur.push_back(measurements_origin_cur[i]);
                                        
                                    }
                                                
                                }
                                
                                
            //                    cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                                if(measurements_cur.size()>=15){
                                    pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur);
            //                        cout<<"f矩阵拒绝后的点数："<<measurements_cur.size()<<", "<<pKFi->global_index<<" ,"<<pkFi_old->global_index<<endl;
                                    if(measurements_old_coarse.size()>=15){
                                    
                                        cv::Point2f norm_pt;
                                        int pCount=measurements_old_coarse.size();
                                        for(int aa=0;aa<pCount;aa++){
                                            norm_pt.x = (measurements_old_coarse[aa].x -  client_cur->PX_server)/ client_cur->FOCUS_LENGTH_X_server;
                                            norm_pt.y = (measurements_old_coarse[aa].y -  client_cur->PY_server)/ client_cur->FOCUS_LENGTH_Y_server;
                                            measurements_old_coarse[aa]=norm_pt;
                                            
                                        }
                                        
                                        point_clouds_all.push_back(point_3d_cur);
                                        measurements_old_norm_all.push_back(measurements_old_coarse);
            //                            num+=point_3d_cur.size();
    //                                    real_vpCovKFi_cur.push_back(pkFi_old);
                                        vpkf_index.push_back(temp_index);
                                        similarNum++;
                                        
                                        break;
                                    }
                                    
                                }
                                
                                temp_index++;
            //                    ends=clock();
            //                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
                            }
                        }
                        
                    }
                    
                    if(similarNum>=2){
                        int optiKf_num=vpkf_index.size()+1;
                        //构造优化问题
                        double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
                        Quaterniond q_array[optiKf_num];
                        double euler_array[optiKf_num][3];

                        ceres::Problem problem;
                        ceres::Solver::Options options;
                        //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                        options.linear_solver_type = ceres::DENSE_SCHUR;
                        //options.minimizer_progress_to_stdout = true;
                        options.max_num_iterations = 20;
                        ceres::Solver::Summary summary;
                        ceres::LossFunction *loss_function;
                        loss_function = new ceres::HuberLoss(1.0);
                //            ceres::LossFunction *loss_function_feature;
                //            loss_function_feature = new ceres::CauchyLoss(1.0);
                        //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
                        ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

                //            int i=0;
                        //这里应该可以只一个优化项，在当前帧位姿误差下，世界坐标系到老帧的位姿
                        //把第一项给老帧 匹配帧
                        
//                        Matrix3d curKF_r;
//                        Vector3d curKF_t;
//                        old_kf->getPose(curKF_t, curKF_r);

                        Matrix3d tmp_r_old;
                        Vector3d tmp_t_old;
                        t_array[0][0] = T_relative(0);
                        t_array[0][1] = T_relative(1);
                        t_array[0][2] = T_relative(2);
                        //将矩阵转换为向量
                        Vector3d euler_angle_old = Utility::R2ypr(R_relative);
                        euler_array[0][0] = euler_angle_old.x();
                        euler_array[0][1] = euler_angle_old.y();
                        euler_array[0][2] = euler_angle_old.z();
                        problem.AddParameterBlock(euler_array[0], 3);
                        problem.AddParameterBlock(t_array[0], 3);

                        
                        //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
                        map<int,int> resample;
                        
            //            temp_index=0;
                        for(int temp_index=0, len=vpkf_index.size(); temp_index<len; temp_index++ ){
                            
                            int kf_index=temp_index+1;
                            Matrix3d relative_r_b_a;
                            Vector3d relative_t_b_a;
                            relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
                            relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
                            Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
                            
                            map<int,int>::iterator iter;
                            iter = resample.find(vpkf_index[temp_index]);
                            if(iter != resample.end())
                            {
                                kf_index=resample[vpkf_index[temp_index]];
                            }
                            else
                            {
                                   
                                resample[vpkf_index[temp_index]]=kf_index;

                               
                                
                                t_array[kf_index][0] = relative_t_b_a(0);
                                t_array[kf_index][1] = relative_t_b_a(1);
                                t_array[kf_index][2] = relative_t_b_a(2);
                                euler_array[kf_index][0] = relative_r_b_a_euler.x();
                                euler_array[kf_index][1] = relative_r_b_a_euler.y();
                                euler_array[kf_index][2] = relative_r_b_a_euler.z();
                                problem.AddParameterBlock(euler_array[kf_index], 1, angle_local_parameterization);
                                problem.AddParameterBlock(t_array[kf_index], 3);
                                ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0),1);
                                problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                            }

                            vector<Vector3d> point_single=point_clouds_all[temp_index];
                            vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
                            for(int a=0,b=point_single.size();a<b;a++){

                                //找到主地图那个点 所在帧的位姿
                                Vector3d pts_i = point_single[a];

                                //相机平面坐标
                                cv::Point2f pt=measure_single[a];
//                                float xx=pt.x;
//                                float yy=pt.y;
//
//                                 Vector2d pts_j ;//要求是归一化图像坐标
//                                 pts_j<<xx,yy;

                                ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pt.x, pt.y, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2),100.0);
                                problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                                
                                
                             }
                        }
                         
                        
                        ceres::Solve(options, &problem, &summary);
                        std::cout <<"精确相对位姿："<< summary.BriefReport() << "\n";

                        if(summary.termination_type==ceres::CONVERGENCE){
                        
                            cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;

                            Matrix3d Rs_i ;
                            Vector3d Ps_i ;//当前帧
                            cur_kf->getOriginPose(Ps_i, Rs_i);
                //                Vector3d q_i;
                //                q_i<<euler_array[1][0],euler_array[1][1],euler_array[1][2];
                //                Matrix3d Rs_i = Utility::ypr2R(q_i);
                //                Vector3d Ps_i = Vector3d( t_array[1][0],  t_array[1][1],  t_array[1][2]);


                            Vector3d q;
                            q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
                            Matrix3d Rs_loop = Utility::ypr2R(q);
                            Vector3d Ps_loop = Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
                            
                //                cout<<"优化计算得到的相对位姿："<<Ps_loop[0]<<" ,"<<Ps_loop[1]<<" ,"<<Ps_loop[2]<<" ,"<<euler_array[0][0]<<" ,"<<euler_array[0][1]<<" ,"<<euler_array[0][2]<<endl;

                            Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
                //                Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
                            double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                            double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
                            double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
                            cur_kf->relative_pitch=relative_pitch;
                            cur_kf->relative_roll=relative_roll;
                            cur_kf->updateLoopConnection(relative_t, relative_yaw);


                            //先临时搬过来
                            old_index=old_kf->global_index;
                            Vector3d T_w_i_old;
                            Matrix3d R_w_i_old;
                            old_kf->getPose(T_w_i_old, R_w_i_old);

                            Quaterniond Q_loop_old(R_w_i_old);
                            RetriveData retrive_data;
                            retrive_data.cur_index = cur_kf->global_index;
                            retrive_data.header = cur_kf->header;
                            retrive_data.P_old = T_w_i_old;
                            retrive_data.Q_old = Q_loop_old;
                            retrive_data.use = true;
                            retrive_data.measurements = measurements_old_norm_all[0];
                            //这里暂时不给值  因为暂时不发送到客户端
            //                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
                            retrive_data.old_index=old_kf->global_index;
                            vins->retrive_pose_data = (retrive_data);
                //                        vins->isSendLoopData=true;

                            cur_kf->detectLoop(old_kf->global_index);
                            poseGraph->addLoop(old_kf->global_index);//todo 这里面 改一下
                            old_kf->is_looped = 1;
                            loop_old_index = old_kf->global_index;
                            vins->isSendLoop_another=true;
                            if(cur_kf->IsOriginUpdate==true){
                               if (cur_kf->sendLoop==false)
                               {

                                   cur_kf->sendLoop=true;
                                   vins->globalOpti_index_mutex.lock();
                                   vins->kf_global_index.push(cur_kf->global_index);
                                   vins->start_kf_global_index.push(cur_kf->loop_index);
                                   poseGraph->latest_loop_index=cur_kf->global_index;
                                   vins->start_global_optimization = true;//先暂停回环 不启动

                                   vins->globalOpti_index_mutex.unlock();
                                   cout<<"慢了 慢了检测到回环 并通过错误回环的检测："<<cur_kf->global_index<<endl;
                               }
                           }
                            cur_kf->is_get_loop_info=true;
                            
                        }
                        else{
                            cout<<"检测失败 未收敛： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
                            
                        }
                    }
                }
            }
        }
        usleep(10);
    }

}

void LoopClosure::setPoseGraph(PoseGraph* _poseGraph){
    poseGraph=_poseGraph;
}

void LoopClosure::setVINS(VINS* _vins){
    vins=_vins;
}
