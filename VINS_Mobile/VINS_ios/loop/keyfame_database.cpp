//
//  keyfame_database.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2017/5/2.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "keyfame_database.h"
KeyFrameDatabase::KeyFrameDatabase()
{
    earliest_loop_index = -1;
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    max_frame_num = 500;
    total_length = 0;
    last_P = Eigen::Vector3d(0, 0, 0);
    cur_seg_index = max_seg_index = 0;
    
    //ljl
    start_global_optimization=false;
    start_global_optimization_multiClient=false;
//    lastKF_index=-1;
}
void KeyFrameDatabase::add(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    
    Eigen::Vector3d P;
    Eigen::Matrix3d R;
    pKF->getPose(P, R);
    
    pKF->updatePose_2Server(P, R);
    
    P = r_drift * P + t_drift;
    R = r_drift * R;
    pKF->updatePose(P, R);
    
    
    keyFrameList.push_back(pKF);
    
    Eigen::Quaterniond Q;
    Q = R;
    
    total_length += (P - last_P).norm();
    last_P = P;
    
    path_time.push_back(pKF->header);
    
    // add key frame to path for visualization
    refine_path.push_back(P.cast<float>());
    refine_r.push_back(Q);
    segment_indexs.push_back(pKF->segment_index);
    lock.unlock();
    
}
//没用上
void KeyFrameDatabase::resample(vector<int> &erase_index)
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    if ((int)keyFrameList.size() < max_frame_num)
        return;
    
    double min_dis = total_length / (0.8 * max_frame_num);
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    Eigen::Vector3d last_P = Eigen::Vector3d(0, 0, 0);
    double dis = 0;
    for (; it != keyFrameList.end(); )
    {
        Eigen::Vector3d tmp_t;
        Eigen::Matrix3d tmp_r;
        (*it)->getPose(tmp_t, tmp_r);
        dis += (tmp_t - last_P).norm();
        if(it == keyFrameList.begin() || dis > min_dis || (*it)->has_loop || (*it)->is_looped)
        {
            dis = 0;
            last_P = tmp_t;
            it++;
        }
        else
        {
            last_P = tmp_t;
            erase_index.push_back((*it)->global_index);
            delete (*it);
            it = keyFrameList.erase(it);
        }
    }
    lock.unlock();
    updateVisualization();
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    list<KeyFrame*>::iterator it = find(keyFrameList.begin(), keyFrameList.end(), pKF);
    assert(it != keyFrameList.end());
    if (it != keyFrameList.end())
        keyFrameList.erase(it);
}

int KeyFrameDatabase::size()
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    return (int)keyFrameList.size();
}

KeyFrame* KeyFrameDatabase::getKeyframe(int index)
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    for (; it != keyFrameList.end(); it++)
    {
        if((*it)->global_index == index)
            break;
    }
    if (it != keyFrameList.end())
        return *it;
    else
        return NULL;
}

KeyFrame* KeyFrameDatabase::getLastKeyframe()
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
    assert(rit != keyFrameList.rend());
    return *rit;
}

KeyFrame* KeyFrameDatabase::getLastKeyframe(int last_index)
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
    for (int i = 0; i < last_index; i++)
    {
        rit++;
        assert(rit != keyFrameList.rend());
    }
    return *rit;
}

KeyFrame*  KeyFrameDatabase::getLastKeyframe_index(int last_index){
    unique_lock<mutex> lock(mMutexkeyFrameList);
    list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
    int len=keyFrameList.size();
    for (int i = 0; i < len; i++)
    {
        
        if((*rit)->global_index == last_index)
            break;
        rit++;
    }
    if (rit != keyFrameList.rend())
        return *rit;
    else
        return NULL;

}

KeyFrame* KeyFrameDatabase::getLastUncheckKeyframe()
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
    for (; rit != keyFrameList.rend(); rit++)
    {
        if ((*rit)->check_loop == 1)
            break;
    }
    assert(rit != keyFrameList.rbegin());
    return *(--rit);
}
//位姿图中的优化，这里是4个自由度的位姿优化
void KeyFrameDatabase::optimize4DoFLoopPoseGraph(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r)
{
    printf("loop global pose graph\n");
    KeyFrame* cur_kf = getKeyframe(cur_index);
    int loop_index = cur_kf->loop_index;
    printf("loop bug current %d %d\n", cur_kf->global_index, cur_kf->loop_index);
    //earliest_loop_index当中存放的是数据库中第一个和滑动窗口中关键帧形成闭环的关键帧的index
    if (earliest_loop_index > loop_index || earliest_loop_index == -1)
        earliest_loop_index = loop_index;
    assert(cur_kf->has_loop == 1);
    //max_length为要优化的变量最大个数
    int max_length = cur_index + 1;
    
    // w^t_i   w^q_i
    double t_array[max_length][3];//平移数组，其中存放每个关键帧的平移向量
    Eigen::Quaterniond q_array[max_length];
    double euler_array[max_length][3];
    vector<bool> need_resample;
    
    ceres::Problem problem;
    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 5;
    ceres::Solver::Summary summary;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);
    //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
    ceres::LocalParameterization* angle_local_parameterization =
    AngleLocalParameterization::Create();
    
    //resample
    double min_dis = total_length / (1.0 * max_frame_num);
    //遍历关键帧列表
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    Eigen::Vector3d last_P = Eigen::Vector3d(0, 0, 0);
    double dis = 0;
    
    //resample pose graph, keep the relative pose sparse
//    printf("debug resample");
    for (; it != keyFrameList.end(); it++)
    {
        //earliest_loop_index为第一次闭环帧的index,需要优化的关键帧为从第一次闭环帧到当前帧间的所有关键帧
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Eigen::Vector3d tmp_t;
        Eigen::Matrix3d tmp_r;
        //获取关键帧it的位姿
        (*it)->getPose(tmp_t, tmp_r);
        dis += (tmp_t - last_P).norm();//模
        if((*it)->global_index == earliest_loop_index || dis > min_dis || (*it)->has_loop || (*it)->is_looped || keyFrameList.size() < max_frame_num)
        {
            dis = 0;
            last_P = tmp_t;
            need_resample.push_back(0);
//            printf("debug %d\n",0);
        }
        else
        {
            last_P = tmp_t;
            need_resample.push_back(1);
//            printf("debug %d\n",1);
        }
    }
    
    int i = 0;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        (*it)->resample_index = i;
        Eigen::Quaterniond tmp_q;
        Eigen::Matrix3d tmp_r;
        Eigen::Vector3d tmp_t;
        (*it)->getOriginPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i] = tmp_q;
        //将矩阵转换为向量
        Eigen::Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
        euler_array[i][0] = euler_angle.x();
        euler_array[i][1] = euler_angle.y();
        euler_array[i][2] = euler_angle.z();
        //将关键帧列表中所有index>=earliest_loop_index的关键帧的位姿加入到参数块当中
        problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
        problem.AddParameterBlock(t_array[i], 3);
        //设置约束：如果该帧是最早的闭环帧的情况下，则固定它的位姿
        if ((*it)->global_index == earliest_loop_index)
        {
//            cout<<"测试 earliest_loop_index 平移和旋转变不变："<<t_array[i][0]<<" "<<t_array[i][1]<<" "<<t_array[i][2]<<" "<<euler_array[i][0]<<" "<<euler_array[i][1]<<" "<<euler_array[i][2]<<" "<<endl;
            problem.SetParameterBlockConstant(euler_array[i]);
            problem.SetParameterBlockConstant(t_array[i]);
        }
        
        if(need_resample[i])
        {
            i++;
            continue;
        }
        
//        printf("debug add %dth pose\n", i);
        //add edge
        int j = 1, sequence_link_cnt = 0;
//        这里添加的是序列边，是指通过VIO计算的两帧之间的相对位姿，每帧分别与其前边最多四帧构成序列边
//        顺序边的测量方程：p̂_{ij}^{i} = {R̂_i^w}^{-1} (p̂_j^w - p̂_i^w)\hat{ψ}_ij = \hat{ψ}_j − \hat{ψ̂}_i
//        两个关键帧之间的相对位姿，由两个关键帧之间的VIO位姿估计变换得到
        while(sequence_link_cnt < 5)
        {
            if (i - j >= 0)
            {
                list<KeyFrame*>::iterator tmp = it;
                std::advance (tmp, -j);
                if(need_resample[i-j])
                {
                    j++;
                    continue;
                }
                else
                {
                    sequence_link_cnt++;
                }
                Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                //p̂_j^w - p̂_i^w 计算平移量的偏差
                Eigen::Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                //p̂_{ij}^{i} = {R̂_i^w}^{-1} (p̂_j^w - p̂_i^w)
                //p̂ ij= (R̂ iw ) (p̂ jw − p̂ iw )
                relative_t = q_array[i-j].inverse() * relative_t;
                //ψ̂ _ij = ψ̂ _j − ψ̂ _i
                double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                          relative_yaw, euler_conncected.y(), euler_conncected.z());
                problem.AddResidualBlock(cost_function, loss_function, euler_array[i-j],
                                         t_array[i-j],
                                         euler_array[i],
                                         t_array[i]);
            }
            else
            {
                break;
            }
            j++;
        }
//        printf("debug sequence_link_cnt %d\n", sequence_link_cnt);
        //add loop edge
//        添加的是闭环边，是指检测到闭环的两帧
        if((*it)->has_loop)
        {
            int connected_index = getKeyframe((*it)->loop_index)->resample_index;
            if((*it)->loop_index < earliest_loop_index)
            {
                printf("loop bug %d %d  %d\n", (*it)->global_index, (*it)->loop_index , (*it)->loop_index);
                assert(false);
            }
            Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
            Eigen::Vector3d relative_t((*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2));
            double relative_yaw = (*it)->loop_info(7);
            ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),relative_yaw, euler_conncected.y(), euler_conncected.z());
            problem.AddResidualBlock(cost_function, NULL, euler_array[connected_index],
                                     t_array[connected_index],
                                     euler_array[i],
                                     t_array[i]);
            
            cout<<"opti4DoFLooppg 服务器不应该出现"<<endl;
            cout<<"相对位姿"<<relative_yaw<<" , "<<relative_t[0]<<endl;
            
        }
        if ((*it)->global_index == cur_index)
            break;
        i++;
    }
    TS(t_global_loop);
//开始优化
    ceres::Solve(options, &problem, &summary);
    TE(t_global_loop);
//    std::cout << summary.BriefReport() << "\n";
    //优化完成，使用优化后的位姿来更新关键帧列表中index大于等于earliest_loop_index的所有关键帧的位姿
    i = 0;
    int seg_index_cur, seg_index_old;
    seg_index_cur = cur_kf->segment_index;
    seg_index_old = getKeyframe(cur_kf->loop_index)->segment_index;
    cur_seg_index = seg_index_old;
    Eigen::Vector3d t_drift_it = Eigen::Vector3d::Zero();
    Eigen::Matrix3d r_drift_it = Eigen::Matrix3d::Identity();
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        
        if ((*it)->global_index == earliest_loop_index)
        {
            cout<<"测试 earliest_loop_index 平移和旋转："<<t_array[i][0]<<" "<<t_array[i][1]<<" "<<t_array[i][2]<<" "<<euler_array[i][0]<<" "<<euler_array[i][1]<<" "<<euler_array[i][2]<<" "<<endl;
            
        }
        Eigen::Quaterniond tmp_q;
        //向量转换为矩阵
        tmp_q = Utility::ypr2R(Eigen::Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
        Eigen::Vector3d tmp_t = Eigen::Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
        Eigen::Matrix3d tmp_r = tmp_q.toRotationMatrix();
        if(need_resample[i])
        {
            (*it)-> updatePose(r_drift_it * tmp_t + t_drift_it, r_drift_it * tmp_r);
        }
        else
        {
            Eigen::Vector3d origin_t_it;
            Eigen::Matrix3d origin_r_it;
            (*it)->getOriginPose(origin_t_it, origin_r_it);
            r_drift_it = tmp_r * origin_r_it.transpose();
            t_drift_it = tmp_t - r_drift_it * origin_t_it;
            (*it)-> updatePose(tmp_t, tmp_r);
        }
        if ((*it)->global_index == cur_index)
            break;
        i++;
        if((*it)->segment_index == seg_index_cur)
            (*it)->segment_index = seg_index_old;
    }
    //根据计算出当前帧的drift，更新全部关键帧位姿
    Eigen::Vector3d cur_t, origin_t;
    Eigen::Matrix3d cur_r, origin_r;
    //获取优化后当前帧的位姿cur_t,cur_r
    cur_kf->getPose(cur_t, cur_r);
    //获取优化前有漂移的当前帧的位姿vio_t,vio_r
    cur_kf->getOriginPose(origin_t, origin_r);
    yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(origin_r).x();
    r_drift = Utility::ypr2R(Eigen::Vector3d(yaw_drift, 0, 0));
    t_drift = cur_t - r_drift * origin_t;
    
    //下面代码为把当前关键帧it之后的关键帧的位姿通过求解的偏移量转换到world坐标系下
    for (; it != keyFrameList.end(); it++)
    {
        Eigen::Vector3d P;
        Eigen::Matrix3d R;
        (*it)->getOriginPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)-> updatePose(P, R);
    }
    loop_correct_t = t_drift;
    loop_correct_r = r_drift;
    updateVisualization();
}

void KeyFrameDatabase::updateVisualization()
{
    total_length = 0;
    last_P = Eigen::Vector3d(0, 0, 0);
    //update visualization
    list<KeyFrame*>::iterator it;
    
    refine_path.clear();
    refine_r.clear();
    segment_indexs.clear();
    all_keyframes.clear();
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        Eigen::Vector3d P;
        Eigen::Matrix3d R;
        (*it)->getPose(P, R);
        Eigen::Quaterniond Q;
        Q = R;
        
        total_length += (P - last_P).norm();
        last_P = P;
        
        // add key frame to path for visualization
        refine_path.push_back(P.cast<float>());
        refine_r.push_back(Q);
        segment_indexs.push_back((*it)->segment_index);
        
        KEYFRAME_DATA keyframe_data;
        keyframe_data.header = (*it)->header;
        keyframe_data.translation = P;
        keyframe_data.rotation = Q;
        all_keyframes.push_back(keyframe_data);
    }
    printf("loop update visualization\n");
}

//位姿图中的优化，这里是4个自由度的位姿优化
void KeyFrameDatabase::optimize4DoFLoopPoseGraph_my1(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r)
{
    printf("loop global pose graph\n");
    KeyFrame* cur_kf = getKeyframe(cur_index);
    int loop_index = cur_kf->loop_index;
    printf("loop bug current %d %d\n", cur_kf->global_index, cur_kf->loop_index);
    //earliest_loop_index当中存放的是数据库中第一个和滑动窗口中关键帧形成闭环的关键帧的index
    if (earliest_loop_index > loop_index || earliest_loop_index == -1)
        earliest_loop_index = loop_index;
    assert(cur_kf->has_loop == 1);
    //max_length为要优化的变量最大个数
    int max_length = cur_index + 1;
    
    // w^t_i   w^q_i
    double t_array[max_length][3];//平移数组，其中存放每个关键帧的平移向量
    Eigen::Quaterniond q_array[max_length];
    double euler_array[max_length][3];
    vector<bool> need_resample;
    
    ceres::Problem problem;
    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 5;
    ceres::Solver::Summary summary;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);
    //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
    ceres::LocalParameterization* angle_local_parameterization =
    AngleLocalParameterization::Create();
    
    //resample
    double min_dis = total_length / (1.0 * max_frame_num);
    //遍历关键帧列表
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    Eigen::Vector3d last_P = Eigen::Vector3d(0, 0, 0);
    double dis = 0;
    
    //resample pose graph, keep the relative pose sparse
//    printf("debug resample");
    for (; it != keyFrameList.end(); it++)
    {
        //earliest_loop_index为第一次闭环帧的index,需要优化的关键帧为从第一次闭环帧到当前帧间的所有关键帧
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Eigen::Vector3d tmp_t;
        Eigen::Matrix3d tmp_r;
        //获取关键帧it的位姿
        (*it)->getPose(tmp_t, tmp_r);
        dis += (tmp_t - last_P).norm();//模
        if((*it)->global_index == earliest_loop_index || dis > min_dis || (*it)->has_loop || (*it)->is_looped || keyFrameList.size() < max_frame_num)
        {
            dis = 0;
            last_P = tmp_t;
            need_resample.push_back(0);
//            printf("debug %d\n",0);
        }
        else
        {
            last_P = tmp_t;
            need_resample.push_back(1);
//            printf("debug %d\n",1);
        }
    }
    
    int i = 0;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        (*it)->resample_index = i;
        Eigen::Quaterniond tmp_q;
        Eigen::Matrix3d tmp_r;
        Eigen::Vector3d tmp_t;
        (*it)->getPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i] = tmp_q;
        //将矩阵转换为向量
        Eigen::Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
        euler_array[i][0] = euler_angle.x();
        euler_array[i][1] = euler_angle.y();
        euler_array[i][2] = euler_angle.z();
        //将关键帧列表中所有index>=earliest_loop_index的关键帧的位姿加入到参数块当中
        problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
        problem.AddParameterBlock(t_array[i], 3);
        //设置约束：如果该帧是最早的闭环帧的情况下，则固定它的位姿
        if ((*it)->global_index == earliest_loop_index)
        {
            problem.SetParameterBlockConstant(euler_array[i]);
            problem.SetParameterBlockConstant(t_array[i]);
        }
        
        if(need_resample[i])
        {
            i++;
            continue;
        }
        
//        printf("debug add %dth pose\n", i);
        //add edge
        int j = 1, sequence_link_cnt = 0;
//        这里添加的是序列边，是指通过VIO计算的两帧之间的相对位姿，每帧分别与其前边最多四帧构成序列边
//        顺序边的测量方程：p̂_{ij}^{i} = {R̂_i^w}^{-1} (p̂_j^w - p̂_i^w)\hat{ψ}_ij = \hat{ψ}_j − \hat{ψ̂}_i
//        两个关键帧之间的相对位姿，由两个关键帧之间的VIO位姿估计变换得到
        while(sequence_link_cnt < 5)
        {
            if (i - j >= 0)
            {
                list<KeyFrame*>::iterator tmp = it;
                std::advance (tmp, -j);
                if(need_resample[i-j])
                {
                    j++;
                    continue;
                }
                else
                {
                    sequence_link_cnt++;
                }
                Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                //p̂_j^w - p̂_i^w 计算平移量的偏差
                Eigen::Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                //p̂_{ij}^{i} = {R̂_i^w}^{-1} (p̂_j^w - p̂_i^w)
                //p̂ ij= (R̂ iw ) (p̂ jw − p̂ iw )
                relative_t = q_array[i-j].inverse() * relative_t;
                //ψ̂ _ij = ψ̂ _j − ψ̂ _i
                double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                          relative_yaw, euler_conncected.y(), euler_conncected.z());
                problem.AddResidualBlock(cost_function, loss_function, euler_array[i-j],
                                         t_array[i-j],
                                         euler_array[i],
                                         t_array[i]);
            }
            else
            {
                break;
            }
            j++;
        }
        printf("debug sequence_link_cnt %d\n", sequence_link_cnt);
        //add loop edge
//        添加的是闭环边，是指检测到闭环的两帧
        if((*it)->has_loop)
        {
            int connected_index = getKeyframe((*it)->loop_index)->resample_index;
            if((*it)->loop_index < earliest_loop_index)
            {
                printf("loop bug %d %d\n", (*it)->global_index, (*it)->loop_index);
                assert(false);
            }
            Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
            Eigen::Vector3d relative_t((*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2));
            double relative_yaw = (*it)->loop_info(7);
            ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),relative_yaw, euler_conncected.y(), euler_conncected.z());
            problem.AddResidualBlock(cost_function, NULL, euler_array[connected_index],
                                     t_array[connected_index],
                                     euler_array[i],
                                     t_array[i]);
            cout<<"opti4DoFLooppg_my1 服务器不应该出现"<<endl;
            
        }
        if ((*it)->global_index == cur_index)
            break;
        i++;
    }
    TS(t_global_loop);
//开始优化
    ceres::Solve(options, &problem, &summary);
    TE(t_global_loop);
//    std::cout << summary.BriefReport() << "\n";
    //优化完成，使用优化后的位姿来更新关键帧列表中index大于等于earliest_loop_index的所有关键帧的位姿
    i = 0;
    int seg_index_cur, seg_index_old;
    seg_index_cur = cur_kf->segment_index;
    seg_index_old = getKeyframe(cur_kf->loop_index)->segment_index;
    cur_seg_index = seg_index_old;
    Eigen::Vector3d t_drift_it = Eigen::Vector3d::Zero();
    Eigen::Matrix3d r_drift_it = Eigen::Matrix3d::Identity();
    //根据计算出当前帧的drift，更新全部关键帧位姿
    Eigen::Vector3d cur_t, origin_t;
    Eigen::Matrix3d cur_r, origin_r;
    //获取优化前有漂移的当前帧的位姿vio_t,vio_r
    cur_kf->getPose(origin_t, origin_r);
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Eigen::Quaterniond tmp_q;
        //向量转换为矩阵
        tmp_q = Utility::ypr2R(Eigen::Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
        Eigen::Vector3d tmp_t = Eigen::Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
        Eigen::Matrix3d tmp_r = tmp_q.toRotationMatrix();
        
        if(need_resample[i])
        {
            (*it)-> updatePose(r_drift_it * tmp_t + t_drift_it, r_drift_it * tmp_r);
        }
        else
        {
            Eigen::Vector3d origin_t_it;
            Eigen::Matrix3d origin_r_it;
            (*it)->getPose(origin_t_it, origin_r_it);
            r_drift_it = tmp_r * origin_r_it.transpose();
            t_drift_it = tmp_t - r_drift_it * origin_t_it;
            (*it)-> updatePose(tmp_t, tmp_r);
        }
        if ((*it)->global_index == cur_index)
            break;
        i++;
        if((*it)->segment_index == seg_index_cur)
            (*it)->segment_index = seg_index_old;
    }
    
    //获取优化后当前帧的位姿cur_t,cur_r
    cur_kf->getPose(cur_t, cur_r);
    
//    yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(origin_r).x();
//    r_drift = r_drift_it;
//    t_drift = t_drift_it;
    
    yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(origin_r).x();
    r_drift = Utility::ypr2R(Eigen::Vector3d(yaw_drift, 0, 0));
    t_drift = cur_t - r_drift * origin_t;
    
    //下面代码为把当前关键帧it之后的关键帧的位姿通过求解的偏移量转换到world坐标系下
    for (; it != keyFrameList.end(); it++)
    {
        Eigen::Vector3d P;
        Eigen::Matrix3d R;
        (*it)->getPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)-> updatePose(P, R);
    }
    loop_correct_t = t_drift;
    loop_correct_r = r_drift;
    updateVisualization();
}

void KeyFrameDatabase::addLoop(int loop_index)
{
    KeyFrame* cur_KF = getLastKeyframe();
    
    KeyFrame* connected_KF = getKeyframe(loop_index);
    Eigen::Vector3d conncected_P, P;
    Eigen::Matrix3d connected_R, R;
    cur_KF->getPose(P, R);
    connected_KF->getPose(conncected_P, connected_R);
}

void KeyFrameDatabase::optimize4DoFLoopPoseGraph_server(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r){
    //打印回环前的位姿
//    for(auto iter=keyFrameList.begin(),iter_end=keyFrameList.end();iter!=iter_end;iter++){
//        Eigen::Vector3d P;
//        Eigen::Matrix3d R;
//
//        (*iter)->getPose(P, R);
//        cout<<"P1  "<<P.transpose()<<"R  "<<R<<endl;
//    }
    
    
    
    vector<Eigen::Vector3d> t_global_test=t_global.front();
    t_global.pop();
    vector<Eigen::Matrix3d> r_global_test=r_global.front();
    r_global.pop();
    
    printf("loop global pose graph\n");
    cout<<"keyFrameList.size="<<keyFrameList.size()<<endl;
    
    KeyFrame* cur_kf = getKeyframe(cur_index);

    //这里还是可以用cur_kf->loop_index
    int loop_index=loopKF_index.front();
    loopKF_index.pop();
    
    cout<<"loop_index="<<loop_index<<" curKF_loop_index="<<cur_index<<endl;
    
    if (earliest_loop_index > loop_index || earliest_loop_index == -1  )
        earliest_loop_index=loop_index;
    printf("loop bug current %d %d\n", cur_kf->global_index, cur_kf->loop_index);
    //earliest_loop_index当中存放的是数据库中第一个和滑动窗口中关键帧形成闭环的关键帧的index
    
   
    
    assert(cur_kf->has_loop == 1);//没有设置成功
    //max_length为要优化的变量最大个数
    int max_length = cur_index + 1;
    
    // w^t_i   w^q_i
    double t_array[max_length][3];//平移数组，其中存放每个关键帧的平移向量
    Eigen::Quaterniond q_array[max_length];
    double euler_array[max_length][3];
    vector<bool> need_resample;
    
        
    //resample
    double min_dis = total_length / (1.0 * max_frame_num);
    //遍历关键帧列表
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    Eigen::Vector3d last_P = Eigen::Vector3d(0, 0, 0);
    double dis = 0;
    
    //resample pose graph, keep the relative pose sparse
//    printf("debug resample");
    for (; it != keyFrameList.end(); it++)
    {
        //earliest_loop_index为第一次闭环帧的index,需要优化的关键帧为从第一次闭环帧到当前帧间的所有关键帧
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Eigen::Vector3d tmp_t;
        Eigen::Matrix3d tmp_r;
        //获取关键帧it的位姿
        (*it)->getPose(tmp_t, tmp_r);
        dis += (tmp_t - last_P).norm();//模
        if((*it)->global_index == earliest_loop_index || dis > min_dis || (*it)->has_loop || (*it)->is_looped || keyFrameList.size() < max_frame_num)
        {
            dis = 0;
            last_P = tmp_t;
            need_resample.push_back(0);
//            printf("debug %d\n",0);
        }
        else
        {
            last_P = tmp_t;
            need_resample.push_back(1);
//            printf("debug %d\n",1);
        }
    }
    
    int i = 0;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        (*it)->resample_index = i;
        Eigen::Quaterniond tmp_q;
        Eigen::Matrix3d tmp_r;
        Eigen::Vector3d tmp_t;
        (*it)->getOriginPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i] = tmp_q;
        //将矩阵转换为向量
        Eigen::Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
        euler_array[i][0] = euler_angle.x();
        euler_array[i][1] = euler_angle.y();
        euler_array[i][2] = euler_angle.z();
        
        if(need_resample[i])
        {
            i++;
            continue;
        }
        
//        printf("debug add %dth pose\n", i);
        if ((*it)->global_index == cur_index)
            break;
        i++;
    }
    TS(t_global_loop);
    //优化完成，使用优化后的位姿来更新关键帧列表中index大于等于earliest_loop_index的所有关键帧的位姿
    i = 0;
    int seg_index_cur, seg_index_old;
    seg_index_cur = cur_kf->segment_index;
    seg_index_old = getKeyframe(cur_kf->loop_index)->segment_index;//待改 涉及到地图融合 序列号
    cur_seg_index = seg_index_old;
    
    Eigen::Vector3d t_drift_it = Eigen::Vector3d::Zero();
    Eigen::Matrix3d r_drift_it = Eigen::Matrix3d::Identity();
    
    int globalChangedKF_index=0;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index){
            continue;
        }
        Eigen::Quaterniond tmp_q;
        //向量转换为矩阵

        Eigen::Vector3d tmp_t ;
        Eigen::Matrix3d tmp_r ;
        if(need_resample[i])
        {
            tmp_q = Utility::ypr2R(Eigen::Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
            tmp_t = Eigen::Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
            tmp_r = tmp_q.toRotationMatrix();
            (*it)-> updatePose(r_drift_it * tmp_t + t_drift_it, r_drift_it * tmp_r);
        }
        else
        {
            Eigen::Vector3d origin_t_it;
            Eigen::Matrix3d origin_r_it;
            (*it)->getOriginPose(origin_t_it, origin_r_it);
            
            tmp_r=r_global_test[globalChangedKF_index];
            tmp_t=t_global_test[globalChangedKF_index];
            
            r_drift_it = tmp_r * origin_r_it.transpose();
            t_drift_it = tmp_t - r_drift_it * origin_t_it;
            (*it)-> updatePose(tmp_t, tmp_r);
            globalChangedKF_index++;
        }
        if ((*it)->global_index == cur_index)
            break;
        i++;
        if((*it)->segment_index == seg_index_cur)
            (*it)->segment_index = seg_index_old;
    }
    
//    Eigen::Vector3d cur_t, origin_t;
//    Eigen::Matrix3d cur_r, origin_r;
//    //获取优化后当前帧的位姿cur_t,cur_r
//    cur_kf->getOriginPose(origin_t, origin_r);
//
//    cur_r=r_global[globalChangedKF_index];
//    cur_t=t_global[globalChangedKF_index];
//
//    yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(origin_r).x();
//    r_drift = Utility::ypr2R(Eigen::Vector3d(yaw_drift, 0, 0));
//    t_drift = cur_t - r_drift * origin_t;
    
    r_drift=r_global_test[globalChangedKF_index];
    t_drift=t_global_test[globalChangedKF_index];
    

    //下面代码为把当前关键帧it之后的关键帧的位姿通过求解的偏移量转换到world坐标系下
    for (; it != keyFrameList.end(); it++)
    {
        Eigen::Vector3d P;
        Eigen::Matrix3d R;
        (*it)->getOriginPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)-> updatePose(P, R);
        
    }
    
    //拿到最后一个关键帧，告诉服务器后续这些加上偏移了
    lastKF_index.push(keyFrameList.back()->global_index);
    
    loop_correct_t = t_drift;
    loop_correct_r = r_drift;
    updateVisualization();
    

    
    //打印回环前的位姿
    for(auto iter=keyFrameList.begin(),iter_end=keyFrameList.end();iter!=iter_end;iter++){
        Eigen::Vector3d P;
        Eigen::Matrix3d R;

        (*iter)->getPose(P, R);
//        cout<<"P2  "<<P.transpose()<<"R  "<<R<<endl;
    }

}


void KeyFrameDatabase::optimize4DoFLoopPoseGraph_server2(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r){
    //打印回环前的位姿
//    for(auto iter=keyFrameList.begin(),iter_end=keyFrameList.end();iter!=iter_end;iter++){
//        Eigen::Vector3d P;
//        Eigen::Matrix3d R;
//
//        (*iter)->getPose(P, R);
//        cout<<"P1  "<<P.transpose()<<"R  "<<R<<endl;
//    }
    cout<<"earliest_queue="<<earliest_queue.size()<<endl;
    
    //服务器那边发过来的 测试用的 担心全局优化堆在一起，导致 后面根据里程计切割序列边可能不一致
    special_kf_intra_mutex.lock();
    vector<Eigen::Vector3d> t_global_test=t_global.front();
    t_global.pop();
    vector<Eigen::Matrix3d> r_global_test=r_global.front();
    r_global.pop();
    vector<int> special_numIs0_kfs=special_kf_inOpti_intra.front();
    special_kf_inOpti_intra.pop();
    int earliest_loop_index=earliest_queue.front();
    earliest_queue.pop();
    //这里还是可以用cur_kf->loop_index
    int loop_index=loopKF_index.front();
    loopKF_index.pop();
    special_kf_intra_mutex.unlock();
    int kf_numIs0_i=0, kf_numIs0_len=special_numIs0_kfs.size();
    
   
    KeyFrame* cur_kf = getKeyframe(cur_index);

   
    
    if (earliest_loop_index > loop_index || earliest_loop_index == -1  ){
        cout<<earliest_loop_index<<" "<<loop_index<<endl;
        assert(false);
//        earliest_loop_index=loop_index;
    }
    printf("loop bug current %d %d\n", cur_kf->global_index, cur_kf->loop_index);
    //earliest_loop_index当中存放的是数据库中第一个和滑动窗口中关键帧形成闭环的关键帧的index
    assert(cur_kf->global_index==cur_index);
    assert(cur_kf->has_loop == 1);//没有设置成功
    //max_length为要优化的变量最大个数
    int max_length = cur_index + 1;
    
    // w^t_i   w^q_i
    double t_array[max_length][3];//平移数组，其中存放每个关键帧的平移向量
    Eigen::Quaterniond q_array[max_length];
    double euler_array[max_length][3];
    vector<bool> need_resample;
    
        
    //resample
//    double min_dis = total_length / (1.0 * max_frame_num);
    //遍历关键帧列表
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    Eigen::Vector3d last_P = Eigen::Vector3d(0, 0, 0);
//    double dis = 0;
    
    
    
    //resample pose graph, keep the relative pose sparse
//    printf("debug resample");
    for (; it != keyFrameList.end(); it++)
    {
        //earliest_loop_index为第一次闭环帧的index,需要优化的关键帧为从第一次闭环帧到当前帧间的所有关键帧
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Eigen::Vector3d tmp_t;
        Eigen::Matrix3d tmp_r;
        //获取关键帧it的位姿
        (*it)->getPose(tmp_t, tmp_r);
//        dis += (tmp_t - last_P).norm();//模
        
        
        
        if(kf_numIs0_i< kf_numIs0_len){
            if((*it)->global_index == special_numIs0_kfs[kf_numIs0_i]){
                kf_numIs0_i++;
                need_resample.push_back(0);
            }else{
                need_resample.push_back(1);
            }
        }else{
            cout<<(*it)->global_index<<" "<<(*it)->has_loop<<" "<<(*it)->is_looped<<" "<<(*it)->has_global_loop<<" "<<keyFrameList.size()<< " " <<cur_index<<"  "<<kf_numIs0_len<<" "
            <<earliest_loop_index<<endl;
            assert(false);//报错了
        }
        
        if((*it)->global_index ==cur_index)
            break;
        
    }
    
    if(kf_numIs0_i!=kf_numIs0_len)
        assert(false);
    
//    cout<<kf_numIs0_i<<" test kf_len "<<kf_numIs0_len<<endl;
    
    int i = 0;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        (*it)->resample_index = i;
        Eigen::Quaterniond tmp_q;
        Eigen::Matrix3d tmp_r;
        Eigen::Vector3d tmp_t;
        (*it)->getOriginPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i] = tmp_q;
        //将矩阵转换为向量
        Eigen::Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
        euler_array[i][0] = euler_angle.x();
        euler_array[i][1] = euler_angle.y();
        euler_array[i][2] = euler_angle.z();
        
        if(need_resample[i])
        {
            i++;
            continue;
        }
        
//        printf("debug add %dth pose\n", i);
        if ((*it)->global_index == cur_index)
            break;
        i++;
    }
    TS(t_global_loop);
    //优化完成，使用优化后的位姿来更新关键帧列表中index大于等于earliest_loop_index的所有关键帧的位姿
    i = 0;
    int seg_index_cur, seg_index_old;
    seg_index_cur = cur_kf->segment_index;
    seg_index_old = getKeyframe(cur_kf->loop_index)->segment_index;//待改 涉及到地图融合 序列号
    cur_seg_index = seg_index_old;
    
    Eigen::Vector3d t_drift_it = Eigen::Vector3d::Zero();
    Eigen::Matrix3d r_drift_it = Eigen::Matrix3d::Identity();
    
    int globalChangedKF_index=0;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index){
            continue;
        }
        Eigen::Quaterniond tmp_q;
        //向量转换为矩阵

        Eigen::Vector3d tmp_t ;
        Eigen::Matrix3d tmp_r ;
        if(need_resample[i])
        {
            tmp_q = Utility::ypr2R(Eigen::Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
            tmp_t = Eigen::Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
            tmp_r = tmp_q.toRotationMatrix();
            (*it)-> updatePose(r_drift_it * tmp_t + t_drift_it, r_drift_it * tmp_r);
        }
        else
        {
            Eigen::Vector3d origin_t_it;
            Eigen::Matrix3d origin_r_it;
            (*it)->getOriginPose(origin_t_it, origin_r_it);
            
            tmp_r=r_global_test[globalChangedKF_index];
            tmp_t=t_global_test[globalChangedKF_index];
            
            r_drift_it = tmp_r * origin_r_it.transpose();
            t_drift_it = tmp_t - r_drift_it * origin_t_it;
            (*it)-> updatePose(tmp_t, tmp_r);
            globalChangedKF_index++;
        }
        if ((*it)->global_index == cur_index)
            break;
        i++;
        if((*it)->segment_index == seg_index_cur)
            (*it)->segment_index = seg_index_old;
    }
    

    
    if(globalChangedKF_index!=(t_global_test.size()-1)){
        cout<<globalChangedKF_index<<" opti test "<<t_global_test.size()-1<<" "<<kf_numIs0_i<<endl;
        assert(false);
    }
    r_drift=r_global_test[globalChangedKF_index];
    t_drift=t_global_test[globalChangedKF_index];
    

    //下面代码为把当前关键帧it之后的关键帧的位姿通过求解的偏移量转换到world坐标系下
    for (; it != keyFrameList.end(); it++)
    {
        Eigen::Vector3d P;
        Eigen::Matrix3d R;
        (*it)->getOriginPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)-> updatePose(P, R);
        
    }
    
    //拿到最后一个关键帧，告诉服务器后续这些加上偏移了
    lastKF_index.push(keyFrameList.back()->global_index);
    
    loop_correct_t = t_drift;
    loop_correct_r = r_drift;
    updateVisualization();
    

   

}

int KeyFrameDatabase::getKFListSize(){
    return keyFrameList.size();
}


//需要考虑 可能存在多次融合优化同时发生，导致 里程计值不统一吗
//这里暂时不考虑
void KeyFrameDatabase::optimize4DoFLoopPoseGraph_server_multiClient(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r){
    
    vector<Eigen::Vector3d> t_global_test=t_global_multiClient.front();
    t_global_multiClient.pop();
    vector<Eigen::Matrix3d> r_global_test=r_global_multiClient.front();
    r_global_multiClient.pop();
    
    printf("loop global multiClient pose graph\n");
    cout<<"keyFrameList.size="<<keyFrameList.size()<<endl;
    
    KeyFrame* cur_kf = getKeyframe(cur_index);

    //这里还是可以用cur_kf->loop_index
    int loop_index=loopKF_index_multiClient.front();
    loopKF_index_multiClient.pop();
    
    cout<<"loop_index="<<loop_index<<" curKF_loop_index="<<cur_index<<endl;
    
    if (earliest_loop_index > loop_index || earliest_loop_index == -1  )
        earliest_loop_index=loop_index;
    printf("loop bug current %d %d\n", cur_kf->global_index, cur_kf->loop_index);
    //earliest_loop_index当中存放的是数据库中第一个和滑动窗口中关键帧形成闭环的关键帧的index
    
   
    
//    assert(cur_kf->has_loop == 1);//没有设置成功
    //max_length为要优化的变量最大个数
    int max_length = cur_index + 1;
    
    // w^t_i   w^q_i
    double t_array[max_length][3];//平移数组，其中存放每个关键帧的平移向量
    Eigen::Quaterniond q_array[max_length];
    double euler_array[max_length][3];
    vector<bool> need_resample;
    
        
    //resample
    int max_frame_num_multiClient=max_frame_num_global;
    vector<int> kf_id_hasComm=kf_id_hasComPlace_withOtherMap.front();
    kf_id_hasComPlace_withOtherMap.pop();
    cout<<"max_frame_num_multiClient="<<max_frame_num_multiClient<<" "<<keyFrameList.size()<<endl;
    
    
    //遍历关键帧列表
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    Eigen::Vector3d last_P = Eigen::Vector3d(0, 0, 0);
    double dis = 0;
    
    //记录 已经找到哪个kf_id
    int kf_id_i=0,kf_id=-1, kf_id_len=kf_id_hasComm.size();
    kf_id_len--;
    if(kf_id_len>=0){
        kf_id =kf_id_hasComm[kf_id_i];
    }
    
    //服务器那边发过来的 测试用的 担心全局优化堆在一起，导致 后面根据里程计切割序列边可能不一致
    vector<int> special_numIs0_kfs=special_kf_inOpti.front();//服务器端needresample 的下标
    special_kf_inOpti.pop();
    int kf_numIs0_i=0, kf_numIs0_len=special_numIs0_kfs.size();
    
    if(special_numIs0_kfs.size()!=(t_global_test.size()-1)){
        cout<<"resample下标数量不一致："<<special_numIs0_kfs.size()<<" , "<<t_global_test.size()<<endl;
        assert(false);
    }
    
    
    cout<<"earliest_loop_index="<<earliest_loop_index<<endl;
    
    //resample pose graph, keep the relative pose sparse
//    printf("debug resample");
    for (; it != keyFrameList.end(); it++)
    {
        //earliest_loop_index为第一次闭环帧的index,需要优化的关键帧为从第一次闭环帧到当前帧间的所有关键帧
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Eigen::Vector3d tmp_t;
        Eigen::Matrix3d tmp_r;
        //获取关键帧it的位姿
        (*it)->getPose(tmp_t, tmp_r);
        dis += (tmp_t - last_P).norm();//模
        
        if((*it)->global_index== kf_id){
            (*it)->has_global_loop=1;
            if(kf_id_i<kf_id_len){
                kf_id_i++;
                kf_id=kf_id_hasComm[kf_id_i];
            }
        }
        
        if((*it)->global_index>cur_index)
            break;
        
        
//        if(kf_numIs0_i< kf_numIs0_len){
//            if((*it)->global_index == special_numIs0_kfs[kf_numIs0_i]){
//                kf_numIs0_i++;
//                need_resample.push_back(0);
//            }else{
//                need_resample.push_back(1);
////                cout<<(*it)->global_index<<" "<<special_numIs0_kfs[kf_numIs0_i]<<" "<<kf_numIs0_i<<endl;
////                assert(false);//说明里程计计算的距离 有问题了
//            }
//        }else{
//            cout<<(*it)->global_index<<" "<<(*it)->has_loop<<" "<<(*it)->is_looped<<" "<<(*it)->has_global_loop<<" "<<keyFrameList.size()<<" "<<max_frame_num_multiClient<< " " <<cur_index<<endl;
//            assert(false);//报错了
//        }
        
//        20220404
        if(kf_numIs0_i< kf_numIs0_len){
            if((*it)->global_index == special_numIs0_kfs[kf_numIs0_i]){
                kf_numIs0_i++;
                need_resample.push_back(0);
                continue;
            }
//            else{
//                need_resample.push_back(1);
////                cout<<(*it)->global_index<<" "<<special_numIs0_kfs[kf_numIs0_i]<<" "<<kf_numIs0_i<<endl;
////                assert(false);//说明里程计计算的距离 有问题了
//            }
        }
        need_resample.push_back(1);
//        else{
//            cout<<(*it)->global_index<<" "<<(*it)->has_loop<<" "<<(*it)->is_looped<<" "<<(*it)->has_global_loop<<" "<<keyFrameList.size()<<" "<<max_frame_num_multiClient<< " " <<cur_index<<endl;
//            assert(false);//报错了
//        }

    }
    
    int i = 0;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        (*it)->resample_index = i;
        Eigen::Quaterniond tmp_q;
        Eigen::Matrix3d tmp_r;
        Eigen::Vector3d tmp_t;
        (*it)->getOriginPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i] = tmp_q;
        //将矩阵转换为向量
        Eigen::Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
        euler_array[i][0] = euler_angle.x();
        euler_array[i][1] = euler_angle.y();
        euler_array[i][2] = euler_angle.z();
        
        if(need_resample[i])
        {
            i++;
            continue;
        }
        
//        printf("debug add %dth pose\n", i);
        if ((*it)->global_index == cur_index)
            break;
        i++;
    }
    TS(t_global_loop);
    //优化完成，使用优化后的位姿来更新关键帧列表中index大于等于earliest_loop_index的所有关键帧的位姿
    i = 0;
    int seg_index_cur, seg_index_old;
    seg_index_cur = cur_kf->segment_index;
    if(cur_kf->has_loop)
    {
        seg_index_old = getKeyframe(cur_kf->loop_index)->segment_index;//待改 涉及到地图融合 序列号
        cur_seg_index = seg_index_old;
    }
    else{
        seg_index_old=seg_index_cur;
    }
    
    Eigen::Vector3d t_drift_it = Eigen::Vector3d::Zero();
    Eigen::Matrix3d r_drift_it = Eigen::Matrix3d::Identity();
    
    int globalChangedKF_index=0;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index){
            continue;
        }
        Eigen::Quaterniond tmp_q;
        //向量转换为矩阵

        Eigen::Vector3d tmp_t ;
        Eigen::Matrix3d tmp_r ;
        if(need_resample[i])
        {
            tmp_q = Utility::ypr2R(Eigen::Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
            tmp_t = Eigen::Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
            tmp_r = tmp_q.toRotationMatrix();
            (*it)-> updatePose(r_drift_it * tmp_t + t_drift_it, r_drift_it * tmp_r);
        }
        else
        {
            Eigen::Vector3d origin_t_it;
            Eigen::Matrix3d origin_r_it;
            (*it)->getOriginPose(origin_t_it, origin_r_it);
            
            tmp_r=r_global_test[globalChangedKF_index];
            tmp_t=t_global_test[globalChangedKF_index];
            
            r_drift_it = tmp_r * origin_r_it.transpose();
            t_drift_it = tmp_t - r_drift_it * origin_t_it;
            (*it)-> updatePose(tmp_t, tmp_r);
            globalChangedKF_index++;
        }
        if ((*it)->global_index == cur_index)
            break;
        i++;
        if((*it)->segment_index == seg_index_cur)
            (*it)->segment_index = seg_index_old;
    }
   
    
    
    
    int  r_global_test_len=r_global_test.size();
    r_global_test_len--;
    
    if(globalChangedKF_index!=r_global_test_len){
        cout<<"globalChangedKF_index= "<< globalChangedKF_index<<" , "<<r_global_test_len <<endl;
        assert(false);//报错了 应该是传过来的数据 出错了
    }
    
    r_drift=r_global_test[r_global_test_len];
    t_drift=t_global_test[r_global_test_len];
//    cout<<"测试 t_drift="<<t_drift[0]<<" "<<t_drift[1]<<" "<<t_drift[2]<<endl;
    

    //下面代码为把当前关键帧it之后的关键帧的位姿通过求解的偏移量转换到world坐标系下
    for (; it != keyFrameList.end(); it++)
    {
        Eigen::Vector3d P;
        Eigen::Matrix3d R;
        (*it)->getOriginPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)-> updatePose(P, R);
        
    }
    
    //拿到最后一个关键帧，告诉服务器后续这些加上偏移了
    lastKF_index.push(keyFrameList.back()->global_index);
    
    loop_correct_t = t_drift;
    loop_correct_r = r_drift;
    updateVisualization();
    


}
