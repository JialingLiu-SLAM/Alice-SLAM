//
//  PoseGraph.cpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/29.
//  Copyright © 2020 zx. All rights reserved.
//

#include "PoseGraph.hpp"
#include <pangolin/pangolin.h>
#include "PoseGraphGlobal.hpp"
#include <fstream>

//PoseGraph::PoseGraph()
//{
//    earliest_loop_index = -1;
//    latest_loop_index=-1;
//    t_drift = Eigen::Vector3d(0, 0, 0);
//    yaw_drift = 0;
//    r_drift = Eigen::Matrix3d::Identity();
//    max_frame_num = 500;
//    total_length = 0;
//    last_P = Eigen::Vector3d(0, 0, 0);
//    cur_seg_index = -1;//改动 从0到-1 画图用
//    max_seg_index = 0;
//
//    loopKF_index=-1;
//    curKF_loop_index=-1;
//    isSendGlobalData=false;
////    lastKF_index=-1;
//    isEnd=false;
//
//    fusion_otherGraph.resize(10);
//    fusion_relative_isUpdate.resize(10);
//    relative_r_mainToOther.resize(10);
//    relative_t_mainToOther.resize(10);
//    relative_score_min.resize(10);
//
//    for(int i=0;i<10;i++){
//        fusion_relative_isUpdate[i]=-2;
//        relative_score_min[i]=-1;
//    }
//
//    is_fusion=0;
//
//    start_global_fuse_opti_mutex.lock();
//    start_global_fuse_opti=0;
//    start_global_fuse_opti_mutex.unlock();
//
//    loop_correct_t = Eigen::Vector3d(0, 0, 0);
//    loop_correct_r = Eigen::Matrix3d::Identity();
//}

PoseGraph::PoseGraph(const char *_voc_file, int _image_w, int _image_h):demo_global_in_poseGraph(_voc_file,_image_w, _image_h)
{
    earliest_loop_index = -1;
    latest_loop_index=-1;
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    max_frame_num = 500;
    total_length = 0;
    last_P = Eigen::Vector3d(0, 0, 0);
    cur_seg_index = -1;//改动 从0到-1 画图用
    max_seg_index = 0;
    
//    loopKF_index=-1;
//    curKF_loop_index=-1;
    isSendGlobalData=false;
    isSendGloablData_multiClient=false;
//    lastKF_index=-1;
    isEnd=false;
    
    fusion_otherGraph.resize(10);
    fusion_relative_isUpdate.resize(10);
    relative_r_mainToOther.resize(10);
    relative_t_mainToOther.resize(10);
    relative_score_min.resize(10);
    
    fuseClientId.resize(10);
    fromI2OtherClient_r.resize(10);
    fromI2OtherClient_t.resize(10);
    for(int i=0;i<10;i++){
        fusion_relative_isUpdate[i]=-2;
        relative_score_min[i]=-1;
        
        fuseClientId[i]=false;
    }
    
    readWriteLock_is_fusion_mutex.writeLock();
    is_fusion=0;
    readWriteLock_is_fusion_mutex.writeUnLock();
    
    start_global_fuse_opti_mutex.lock();
    start_global_fuse_opti=0;
    start_global_fuse_opti_mutex.unlock();
    
    loop_correct_t = Eigen::Vector3d(0, 0, 0);
    loop_correct_r = Eigen::Matrix3d::Identity();
    
    
    kfNum_tree=0;
    treeId_kf.clear();
    
    prePoseGraphOpti=0;
    waitOpti=-1;
    hasDetectLoopKfId=-1;
    waitNum=0;
//    cout<<"poseGraph init"<<endl;
    
//    keyFrameList_global_fusion_mutex.lock();
//    pitch_relativeOtherMap=0.0;
//    roll_relativeOtherMap=0.0;
//    relativeOtherMap_clientId=-1;
//    keyFrameList_global_fusion_mutex.unlock();
    
    
}

void PoseGraph::add(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    keyFrameList.push_back(pKF);
    Vector3d P;
    Matrix3d R;
    pKF->getPose(P, R);
    P = r_drift * P + t_drift;
    R = r_drift * R;
    pKF->updatePose(P, R);
    
    pKF->updatePose_old(P, R);

    Quaterniond Q;
    Q = R;
    
    
    total_length += (P - last_P).norm();
    pKF->vio_length=total_length;
    last_P = P;
    
    // add key frame to path for visualization
    refine_path.push_back(P.cast<float>());
    refine_path_r.push_back(R.cast<float>());
    
    //实验用
    path_time.push_back(pKF->header);
    
    segment_indexs.push_back(pKF->segment_index);
    lock.unlock();
    
   
    
    //画图用 画的图 画着画着不见了，可能和这个有关
    curKF_P=P;
    curKF_R=R;
    
}

void PoseGraph::resample(vector<int> &erase_index)
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    if ((int)keyFrameList.size() < max_frame_num)
        return;
    
    double min_dis = total_length / (0.8 * max_frame_num);
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    Vector3d last_P = Vector3d(0, 0, 0);
    double dis = 0;
    for (; it != keyFrameList.end(); )
    {
        Vector3d tmp_t;
        Matrix3d tmp_r;
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
    cout<<"resample 调用 ";
    updateVisualization();
}

void PoseGraph::erase(KeyFrame* pKF)
{
    list<KeyFrame*>::iterator it = find(keyFrameList.begin(), keyFrameList.end(), pKF);
    assert(it != keyFrameList.end());
    if (it != keyFrameList.end())
        keyFrameList.erase(it);
}

int PoseGraph::size()
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    int len=(int)keyFrameList.size();
    lock.unlock();
    return len;
    
}

KeyFrame* PoseGraph::getKeyframe(int index)
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

KeyFrame* PoseGraph::getKeyframe2(int index)
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    advance(it, index);
    if(index<keyFrameList.size()){
        if((*it)->global_index!=index){
            assert(false);
            return NULL;
        }
        return *it;
    }
    else{
        assert(false);
        return NULL;
    }

}

list<KeyFrame*>::iterator PoseGraph::getKeyframe_iter(int index)
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    for (; it != keyFrameList.end(); it++)
    {
        if((*it)->global_index == index)
            break;
    }
    if (it != keyFrameList.end())
        return it;
    else
        return keyFrameList.end();
}


KeyFrame* PoseGraph::getLastKeyframe()
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    if(keyFrameList.size()!=0){
        list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
        assert(rit != keyFrameList.rend());
        return *rit;
    }else{
        return nullptr;
    }
}

KeyFrame* PoseGraph::getLastKeyframe(int last_index)
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

KeyFrame*  PoseGraph::getLastKeyframe_index(int last_index){
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

KeyFrame* PoseGraph::getLastUncheckKeyframe()
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

//void PoseGraph::setPoseGraphGlobal(PoseGraphGlobal* poseGraphGlobal){
//    mpPoseGraphGlobal=poseGraphGlobal;
//}

void PoseGraph::setClient(Client* _c){
    c=_c;
}


//位姿图中的优化，这里是4个自由度的位姿优化 这个是之前实验的最终版本
void PoseGraph::optimize4DoFLoopPoseGraph(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r)
{
//    keyFrameList_global_fusion_mutex.lock();
    vector<Matrix3d> r_global_test;
    vector<Vector3d> t_global_test;
//    r_global.clear();
//    t_global.clear();
//    printf("loop global pose graph\n");
    KeyFrame* cur_kf = getKeyframe(cur_index);
    int loop_index = cur_kf->loop_index;
//    printf("loop bug current %d %d %d\n", cur_kf->global_index, cur_kf->loop_index,cur_index);
    //earliest_loop_index当中存放的是数据库中第一个和滑动窗口中关键帧形成闭环的关键帧的index
    if (earliest_loop_index > loop_index || earliest_loop_index == -1)
        earliest_loop_index = loop_index;
    assert(cur_kf->has_loop == 1);
    //max_length为要优化的变量最大个数
    int max_length = cur_index + 1;
    
    //记录要发送的数据
//    curKF_loop_index=cur_index;
//    loopKF_index=loop_index;
    curKF_loop_index.push(cur_index);
    loopKF_index.push(loop_index);
    
    // w^t_i   w^q_i
    double t_array[max_length][3];//平移数组，其中存放每个关键帧的平移向量
    Quaterniond q_array[max_length];
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
    Vector3d last_P = Vector3d(0, 0, 0);
    double dis = 0;
    
    //resample pose graph, keep the relative pose sparse
    printf("debug resample");
    for (; it != keyFrameList.end(); it++)
    {
        //earliest_loop_index为第一次闭环帧的index,需要优化的关键帧为从第一次闭环帧到当前帧间的所有关键帧
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Vector3d tmp_t;
        Matrix3d tmp_r;
        //获取关键帧it的位姿
        (*it)->getPose(tmp_t, tmp_r);
        dis += (tmp_t - last_P).norm();
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
        Quaterniond tmp_q;
        Matrix3d tmp_r;
        Vector3d tmp_t;
        (*it)->getOriginPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i] = tmp_q;
        //将矩阵转换为向量
        Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
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
                Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                //p̂_j^w - p̂_i^w 计算平移量的偏差
                Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
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
        if((*it)->has_loop&&(*it)->is_get_loop_info   )
        {
            int connected_index = getKeyframe((*it)->loop_index)->resample_index;
            if((*it)->loop_index < earliest_loop_index)
            {
                //说明检测回环的可能是检测到连续的好几帧的回环了
                printf("loop bug %d %d\n", (*it)->global_index, (*it)->loop_index);
                assert(false);
            }
            Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
            Vector3d relative_t((*it)->loop_info_better(0), (*it)->loop_info_better(1), (*it)->loop_info_better(2));
            double relative_yaw = (*it)->loop_info_better(3);
            
            //这里 还存在错误的值
//            cout<<"(*it)->loop_info(0)"<<(*it)->loop_info_better(0)<<" (*it)->loop_info(1)"<<(*it)->loop_info_better(1)<<" (*it)->loop_info(2)"<<(*it)->loop_info_better(2)<<" (*it)->loop_info(3)"<<(*it)->loop_info_better(3)<<" "<<i<<endl;
            
            ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(), relative_yaw, euler_conncected.y(), euler_conncected.z(),10.0);
            problem.AddResidualBlock(cost_function, NULL, euler_array[connected_index],
                                     t_array[connected_index],
                                     euler_array[i],
                                     t_array[i]);
            
        }
        if ((*it)->global_index == cur_index)
            break;
        i++;
    }
    TS(t_global_loop);
//开始优化
    ceres::Solve(options, &problem, &summary);
    TE(t_global_loop);
    std::cout << summary.BriefReport() << "\n";
    //优化完成，使用优化后的位姿来更新关键帧列表中index大于等于earliest_loop_index的所有关键帧的位姿
    i = 0;
    int seg_index_cur, seg_index_old;
    seg_index_cur = cur_kf->segment_index;
    seg_index_old = getKeyframe(cur_kf->loop_index)->segment_index;
    cur_seg_index = seg_index_old;
    Vector3d t_drift_it = Vector3d::Zero();
    Matrix3d r_drift_it = Matrix3d::Identity();
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Quaterniond tmp_q;
        //向量转换为矩阵
        tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
        Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
        Matrix3d tmp_r = tmp_q.toRotationMatrix();
        if(need_resample[i])
        {
            (*it)-> updatePose(r_drift_it * tmp_t + t_drift_it, r_drift_it * tmp_r);
        }
        else
        {
            Vector3d origin_t_it;
            Matrix3d origin_r_it;
            (*it)->getOriginPose(origin_t_it, origin_r_it);
            r_drift_it = tmp_r * origin_r_it.transpose();
            t_drift_it = tmp_t - r_drift_it * origin_t_it;
            (*it)-> updatePose(tmp_t, tmp_r);
            
            r_global_test.push_back(tmp_r);
            t_global_test.push_back(tmp_t);
        }
        (*it)->r_drift=r_drift_it;
        (*it)->t_drift=t_drift_it;
        
        if ((*it)->global_index == cur_index)
            break;
        i++;
        if((*it)->segment_index == seg_index_cur)
            (*it)->segment_index = seg_index_old;
    }
    //根据计算出当前帧的drift，更新全部关键帧位姿
    Vector3d cur_t, origin_t;
    Matrix3d cur_r, origin_r;
    //获取优化后当前帧的位姿cur_t,cur_r
    cur_kf->getPose(cur_t, cur_r);
    //获取优化前有漂移的当前帧的位姿vio_t,vio_r
    cur_kf->getOriginPose(origin_t, origin_r);
    yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(origin_r).x();
    r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
    t_drift = cur_t - r_drift * origin_t;
    
    r_global_test.push_back(r_drift);
    t_global_test.push_back(t_drift);
    r_global.push(r_global_test);
    t_global.push(t_global_test);
    
    //下面代码为把当前关键帧it之后的关键帧的位姿通过求解的偏移量转换到world坐标系下
    for (; it != keyFrameList.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getOriginPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)-> updatePose(P, R);
    }
    
//    keyFrameList_global_fusion_mutex.unlock();
    lastKF_index.push(keyFrameList.back()->global_index);
    
    loop_correct_t = t_drift;
    loop_correct_r = r_drift;
    updateVisualization();
    
    //打印全局优化后的位姿
//        for(auto iter=keyFrameList.begin(),iter_end=keyFrameList.end();iter!=iter_end;iter++){
//            Vector3d P;
//            Matrix3d R;
//    
//            (*iter)->getPose(P, R);
//            cout<<"P1  "<<P.transpose()<<"R  "<<R<<endl;
//        }
    
}

//位姿图中的优化，这里是4个自由度的位姿优化 新的实验，加降低权重边
void PoseGraph::optimize4DoFLoopPoseGraph5(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r)
{
//    keyFrameList_global_fusion_mutex.lock();
    vector<Matrix3d> r_global_test;
    vector<Vector3d> t_global_test;
//    r_global.clear();
//    t_global.clear();
//    printf("loop global pose graph\n");
    KeyFrame* cur_kf = getKeyframe(cur_index);
    int loop_index = cur_kf->loop_index;
//    printf("loop bug current %d %d %d\n", cur_kf->global_index, cur_kf->loop_index,cur_index);
    //earliest_loop_index当中存放的是数据库中第一个和滑动窗口中关键帧形成闭环的关键帧的index
    if (earliest_loop_index > loop_index || earliest_loop_index == -1)
        earliest_loop_index = loop_index;
    
    cout<<"优化开始opti test "<<cur_kf->global_index<<" , "<<cur_index<<" , "<<cur_kf->loop_index<<endl;//<<" , "<<getTime()
    assert(cur_kf->has_loop == 1);
    //max_length为要优化的变量最大个数
    int max_length = cur_index + 1;
    
    //记录要发送的数据
//    curKF_loop_index=cur_index;
//    loopKF_index=loop_index;
    curKF_loop_index.push(cur_index);
    loopKF_index.push(loop_index);
    earliest_queue.push(earliest_loop_index);
    
    // w^t_i   w^q_i
    double t_array[max_length][3];//平移数组，其中存放每个关键帧的平移向量
    Quaterniond q_array[max_length];
    double euler_array[max_length][3];
    vector<bool> need_resample;
    
    ceres::Problem problem;
    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 6;
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
    Vector3d last_P = Vector3d(0, 0, 0);
    double dis = 0;
    
    vector<int> special_kf_inOpti_main;
    //resample pose graph, keep the relative pose sparse
//    printf("debug resample");
    for (; it != keyFrameList.end(); it++)
    {
        //earliest_loop_index为第一次闭环帧的index,需要优化的关键帧为从第一次闭环帧到当前帧间的所有关键帧
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Vector3d tmp_t;
        Matrix3d tmp_r;
        //获取关键帧it的位姿
        (*it)->getPose(tmp_t, tmp_r);
        dis += (tmp_t - last_P).norm();
        if((*it)->global_index == earliest_loop_index || dis > min_dis || (*it)->has_loop || (*it)->is_looped || keyFrameList.size() < max_frame_num)
        {
            dis = 0;
            last_P = tmp_t;
            need_resample.push_back(0);
//            printf("debug %d\n",(*it)->global_index);
            
            special_kf_inOpti_main.push_back((*it)->global_index);
        }
        else
        {
            last_P = tmp_t;
            need_resample.push_back(1);
//            printf("debug %d\n",1);
        }
        
        if ((*it)->global_index == cur_index)
            break;
    }
//    cout<<"opti639= "<<special_kf_inOpti_main.size()<<" "<<earliest_loop_index<<endl;
    
    special_kf_mutex_intra.lock();
    special_kf_inOpti_intra.push(special_kf_inOpti_main);
    special_kf_mutex_intra.unlock();
    
    int i = 0;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        (*it)->resample_index = i;
        Quaterniond tmp_q;
        Matrix3d tmp_r;
        Vector3d tmp_t;
        (*it)->getOriginPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i] = tmp_q;
        //将矩阵转换为向量
        Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
        euler_array[i][0] = euler_angle.x();
        euler_array[i][1] = euler_angle.y();
        euler_array[i][2] = euler_angle.z();
//        cout<<"添加参数块为："<<(*it)->global_index<<endl;
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
                Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                //p̂_j^w - p̂_i^w 计算平移量的偏差
                Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
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
        if((*it)->has_loop&&(*it)->is_get_loop_info   )
        {
            int connected_index = getKeyframe((*it)->loop_index)->resample_index;
            if((*it)->loop_index < earliest_loop_index)
            {
                //说明检测回环的可能是检测到连续的好几帧的回环了
                printf("loop bug %d %d\n", (*it)->global_index, (*it)->loop_index);
                assert(false);
            }
            Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
            Vector3d relative_t((*it)->loop_info_better(0), (*it)->loop_info_better(1), (*it)->loop_info_better(2));
            double relative_yaw = (*it)->loop_info_better(3);
            
            //这里 还存在错误的值
//            cout<<"(*it)->loop_info(0)"<<(*it)->loop_info_better(0)<<" (*it)->loop_info(1)"<<(*it)->loop_info_better(1)<<" (*it)->loop_info(2)"<<(*it)->loop_info_better(2)<<" (*it)->loop_info(3)"<<(*it)->loop_info_better(3)<<" "<<i<<endl;
            
            
            if((*it)->isRemove_loop){//权重为3
                ceres::CostFunction* cost_function = FourDOFWeightError_forRemove::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                relative_yaw, euler_conncected.y(), euler_conncected.z(),3.0);
                problem.AddResidualBlock(cost_function, NULL, euler_array[connected_index],
                                         t_array[connected_index],
                                         euler_array[i],
                                         t_array[i]);
                cout<<"降权重 发生过"<<endl;
            }else{//论文三实验一之前权重为10，后面尝试50
                ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                relative_yaw, euler_conncected.y(), euler_conncected.z(),10.0);
                problem.AddResidualBlock(cost_function, NULL, euler_array[connected_index],
                                         t_array[connected_index],
                                         euler_array[i],
                                         t_array[i]);
            }
            
            
            
        }
        if ((*it)->global_index == cur_index)
            break;
        i++;
    }
//    TS(t_global_loop);
//开始优化
    ceres::Solve(options, &problem, &summary);
//    TE(t_global_loop);
//    std::cout << summary.BriefReport() << "\n";
    //优化完成，使用优化后的位姿来更新关键帧列表中index大于等于earliest_loop_index的所有关键帧的位姿
    i = 0;
    int seg_index_cur, seg_index_old;
    seg_index_cur = cur_kf->segment_index;
    seg_index_old = getKeyframe(cur_kf->loop_index)->segment_index;
    cur_seg_index = seg_index_old;
    Vector3d t_drift_it = Vector3d::Zero();
    Matrix3d r_drift_it = Matrix3d::Identity();
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Quaterniond tmp_q;
        //向量转换为矩阵
        tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
        Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
        Matrix3d tmp_r = tmp_q.toRotationMatrix();
        if(need_resample[i])
        {
            (*it)-> updatePose(r_drift_it * tmp_t + t_drift_it, r_drift_it * tmp_r);
            
        }
        else
        {
            Vector3d origin_t_it;
            Matrix3d origin_r_it;
            (*it)->getOriginPose(origin_t_it, origin_r_it);
            r_drift_it = tmp_r * origin_r_it.transpose();
            t_drift_it = tmp_t - r_drift_it * origin_t_it;
            (*it)-> updatePose(tmp_t, tmp_r);
            
            r_global_test.push_back(tmp_r);
            t_global_test.push_back(tmp_t);
        }
        (*it)->r_drift=r_drift_it;
        (*it)->t_drift=t_drift_it;
        
        if ((*it)->global_index == cur_index)
            break;
        i++;
        if((*it)->segment_index == seg_index_cur)
            (*it)->segment_index = seg_index_old;
    }
    //根据计算出当前帧的drift，更新全部关键帧位姿
    Vector3d cur_t, origin_t;
    Matrix3d cur_r, origin_r;
    //获取优化后当前帧的位姿cur_t,cur_r
    cur_kf->getPose(cur_t, cur_r);
    //获取优化前有漂移的当前帧的位姿vio_t,vio_r
    cur_kf->getOriginPose(origin_t, origin_r);
    yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(origin_r).x();
    r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
    t_drift = cur_t - r_drift * origin_t;
    
    r_global_test.push_back(r_drift);
    t_global_test.push_back(t_drift);
    r_global.push(r_global_test);
    t_global.push(t_global_test);
    
//    cout<<r_global.size()<<" opti5 test "<<t_global.size()<<" "<<special_kf_inOpti_main.size()<<endl;
    
    //下面代码为把当前关键帧it之后的关键帧的位姿通过求解的偏移量转换到world坐标系下
    for (; it != keyFrameList.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getOriginPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)-> updatePose(P, R);
        
//        cout<<"PoseGraph::opti5 837 测试位姿哪一步变差了：";
//        for(int i=0;i<3;i++){
//            cout<<P[i]<<" ";
//        }
//        cout<<endl;
    }
    
//    keyFrameList_global_fusion_mutex.unlock();
    lastKF_index.push(keyFrameList.back()->global_index);
    
    loop_correct_t = t_drift;
    loop_correct_r = r_drift;
//    cout<<"opti5 调用 ";
    updateVisualization();
    cout<<"回环优化结束"<<cur_kf->global_index<<endl;//<<" , "<<getTime()
    //打印全局优化后的位姿
//        for(auto iter=keyFrameList.begin(),iter_end=keyFrameList.end();iter!=iter_end;iter++){
//            Vector3d P;
//            Matrix3d R;
//
//            (*iter)->getPose(P, R);
//            cout<<"P1  "<<P.transpose()<<"R  "<<R<<endl;
//        }
    
}

//基于5的基础 加每段相对位姿的精度 作为权重
void PoseGraph::optimize4DoFLoopPoseGraph7(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r)
{
//    keyFrameList_global_fusion_mutex.lock();
    vector<Matrix3d> r_global_test;
    vector<Vector3d> t_global_test;
//    r_global.clear();
//    t_global.clear();
//    printf("loop global pose graph\n");
    KeyFrame* cur_kf = getKeyframe(cur_index);
    int loop_index = cur_kf->loop_index;
//    printf("loop bug current %d %d %d\n", cur_kf->global_index, cur_kf->loop_index,cur_index);
    //earliest_loop_index当中存放的是数据库中第一个和滑动窗口中关键帧形成闭环的关键帧的index
    if (earliest_loop_index > loop_index || earliest_loop_index == -1)
        earliest_loop_index = loop_index;
    
    cout<<"opti test "<<cur_kf->global_index<<" , "<<cur_index<<" , "<<cur_kf->loop_index<<endl;
    assert(cur_kf->has_loop == 1);
    //max_length为要优化的变量最大个数
    int max_length = cur_index + 1;
    
    //记录要发送的数据
//    curKF_loop_index=cur_index;
//    loopKF_index=loop_index;
    curKF_loop_index.push(cur_index);
    loopKF_index.push(loop_index);
    earliest_queue.push(earliest_loop_index);
    
    // w^t_i   w^q_i
    double t_array[max_length][3];//平移数组，其中存放每个关键帧的平移向量
    Quaterniond q_array[max_length];
    double euler_array[max_length][3];
    vector<bool> need_resample;
    
    ceres::Problem problem;
    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 6;
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
    Vector3d last_P = Vector3d(0, 0, 0);
    double dis = 0;
    
    vector<int> special_kf_inOpti_main;
    //resample pose graph, keep the relative pose sparse
//    printf("debug resample");
    for (; it != keyFrameList.end(); it++)
    {
        //earliest_loop_index为第一次闭环帧的index,需要优化的关键帧为从第一次闭环帧到当前帧间的所有关键帧
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Vector3d tmp_t;
        Matrix3d tmp_r;
        //获取关键帧it的位姿
        (*it)->getPose(tmp_t, tmp_r);
        dis += (tmp_t - last_P).norm();
        if((*it)->global_index == earliest_loop_index || dis > min_dis || (*it)->has_loop || (*it)->is_looped || keyFrameList.size() < max_frame_num)
        {
            dis = 0;
            last_P = tmp_t;
            need_resample.push_back(0);
//            printf("debug %d\n",(*it)->global_index);
            
            special_kf_inOpti_main.push_back((*it)->global_index);
        }
        else
        {
            last_P = tmp_t;
            need_resample.push_back(1);
//            printf("debug %d\n",1);
        }
        
        if ((*it)->global_index == cur_index)
            break;
    }
//    cout<<"opti639= "<<special_kf_inOpti_main.size()<<" "<<earliest_loop_index<<endl;
    
    special_kf_mutex_intra.lock();
    special_kf_inOpti_intra.push(special_kf_inOpti_main);
    special_kf_mutex_intra.unlock();
    
    int i = 0;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        (*it)->resample_index = i;
        Quaterniond tmp_q;
        Matrix3d tmp_r;
        Vector3d tmp_t;
        (*it)->getOriginPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i] = tmp_q;
        //将矩阵转换为向量
        Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
        euler_array[i][0] = euler_angle.x();
        euler_array[i][1] = euler_angle.y();
        euler_array[i][2] = euler_angle.z();
//        cout<<"添加参数块为："<<(*it)->global_index<<endl;
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
                Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                //p̂_j^w - p̂_i^w 计算平移量的偏差
                Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
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
                
//                传权重 里程计 3D点数量 单次移动
//                可以跑一下mh02 v系列原始误差较小的和然后设置阈值
//                cout<<"窗口中3D点占比"<<edge_single_test/edge <<endl;+ vio_length
//                cout<<"imu激励"<<var_imu<<endl;
//                loop_error/loop_edge+loop_error_final/loop_edge_final
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
        if((*it)->has_loop&&(*it)->is_get_loop_info   )
        {
            int connected_index = getKeyframe((*it)->loop_index)->resample_index;
            if((*it)->loop_index < earliest_loop_index)
            {
                //说明检测回环的可能是检测到连续的好几帧的回环了
                printf("loop bug %d %d\n", (*it)->global_index, (*it)->loop_index);
                assert(false);
            }
            Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
            Vector3d relative_t((*it)->loop_info_better(0), (*it)->loop_info_better(1), (*it)->loop_info_better(2));
            double relative_yaw = (*it)->loop_info_better(3);
            
            //这里 还存在错误的值
//            cout<<"(*it)->loop_info(0)"<<(*it)->loop_info_better(0)<<" (*it)->loop_info(1)"<<(*it)->loop_info_better(1)<<" (*it)->loop_info(2)"<<(*it)->loop_info_better(2)<<" (*it)->loop_info(3)"<<(*it)->loop_info_better(3)<<" "<<i<<endl;
            
            
            if((*it)->isRemove_loop){//权重为3
                ceres::CostFunction* cost_function = FourDOFWeightError_forRemove::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                relative_yaw, euler_conncected.y(), euler_conncected.z(),3.0);
                problem.AddResidualBlock(cost_function, NULL, euler_array[connected_index],
                                         t_array[connected_index],
                                         euler_array[i],
                                         t_array[i]);
                cout<<"降权重 发生过"<<endl;
            }else{//权重为10
                ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                relative_yaw, euler_conncected.y(), euler_conncected.z(),10.0);
                problem.AddResidualBlock(cost_function, NULL, euler_array[connected_index],
                                         t_array[connected_index],
                                         euler_array[i],
                                         t_array[i]);
            }
            
            
            
        }
        if ((*it)->global_index == cur_index)
            break;
        i++;
    }
//    TS(t_global_loop);
//开始优化
    ceres::Solve(options, &problem, &summary);
//    TE(t_global_loop);
//    std::cout << summary.BriefReport() << "\n";
    //优化完成，使用优化后的位姿来更新关键帧列表中index大于等于earliest_loop_index的所有关键帧的位姿
    i = 0;
    int seg_index_cur, seg_index_old;
    seg_index_cur = cur_kf->segment_index;
    seg_index_old = getKeyframe(cur_kf->loop_index)->segment_index;
    cur_seg_index = seg_index_old;
    Vector3d t_drift_it = Vector3d::Zero();
    Matrix3d r_drift_it = Matrix3d::Identity();
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Quaterniond tmp_q;
        //向量转换为矩阵
        tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
        Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
        Matrix3d tmp_r = tmp_q.toRotationMatrix();
        if(need_resample[i])
        {
            (*it)-> updatePose(r_drift_it * tmp_t + t_drift_it, r_drift_it * tmp_r);
            
        }
        else
        {
            Vector3d origin_t_it;
            Matrix3d origin_r_it;
            (*it)->getOriginPose(origin_t_it, origin_r_it);
            r_drift_it = tmp_r * origin_r_it.transpose();
            t_drift_it = tmp_t - r_drift_it * origin_t_it;
            (*it)-> updatePose(tmp_t, tmp_r);
            
            r_global_test.push_back(tmp_r);
            t_global_test.push_back(tmp_t);
        }
        (*it)->r_drift=r_drift_it;
        (*it)->t_drift=t_drift_it;
        
        if ((*it)->global_index == cur_index)
            break;
        i++;
        if((*it)->segment_index == seg_index_cur)
            (*it)->segment_index = seg_index_old;
    }
    //根据计算出当前帧的drift，更新全部关键帧位姿
    Vector3d cur_t, origin_t;
    Matrix3d cur_r, origin_r;
    //获取优化后当前帧的位姿cur_t,cur_r
    cur_kf->getPose(cur_t, cur_r);
    //获取优化前有漂移的当前帧的位姿vio_t,vio_r
    cur_kf->getOriginPose(origin_t, origin_r);
    yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(origin_r).x();
    r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
    t_drift = cur_t - r_drift * origin_t;
    
    r_global_test.push_back(r_drift);
    t_global_test.push_back(t_drift);
    r_global.push(r_global_test);
    t_global.push(t_global_test);
    
//    cout<<r_global.size()<<" opti5 test "<<t_global.size()<<" "<<special_kf_inOpti_main.size()<<endl;
    
    //下面代码为把当前关键帧it之后的关键帧的位姿通过求解的偏移量转换到world坐标系下
    for (; it != keyFrameList.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getOriginPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)-> updatePose(P, R);
        
//        cout<<"PoseGraph::opti5 837 测试位姿哪一步变差了：";
//        for(int i=0;i<3;i++){
//            cout<<P[i]<<" ";
//        }
//        cout<<endl;
    }
    
//    keyFrameList_global_fusion_mutex.unlock();
    lastKF_index.push(keyFrameList.back()->global_index);
    
    loop_correct_t = t_drift;
    loop_correct_r = r_drift;
//    cout<<"opti5 调用 ";
    updateVisualization();
    cout<<"回环优化结束"<<endl;
    //打印全局优化后的位姿
//        for(auto iter=keyFrameList.begin(),iter_end=keyFrameList.end();iter!=iter_end;iter++){
//            Vector3d P;
//            Matrix3d R;
//
//            (*iter)->getPose(P, R);
//            cout<<"P1  "<<P.transpose()<<"R  "<<R<<endl;
//        }
    
}

//位姿图中的优化，这里是4个自由度的位姿优化 新的实验，加降低权重边
void PoseGraph::optimize4DoFLoopPoseGraph6(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r)
{
//    keyFrameList_global_fusion_mutex.lock();
    vector<Matrix3d> r_global_test;
    vector<Vector3d> t_global_test;
//    r_global.clear();
//    t_global.clear();
//    printf("loop global pose graph\n");
    KeyFrame* cur_kf = getKeyframe(cur_index);
    int loop_index = cur_kf->loop_index;
//    printf("loop bug current %d %d %d\n", cur_kf->global_index, cur_kf->loop_index,cur_index);
    //earliest_loop_index当中存放的是数据库中第一个和滑动窗口中关键帧形成闭环的关键帧的index
    if (earliest_loop_index > loop_index || earliest_loop_index == -1)
        earliest_loop_index = loop_index;
    
//    cout<<"opti test "<<earliest_loop_index<<endl;
    assert(cur_kf->has_loop == 1);
    //max_length为要优化的变量最大个数
    int max_length = cur_index + 1;
    
    //记录要发送的数据
//    curKF_loop_index=cur_index;
//    loopKF_index=loop_index;
    curKF_loop_index.push(cur_index);
    loopKF_index.push(loop_index);
    earliest_queue.push(earliest_loop_index);
    
    // w^t_i   w^q_i
    double t_array[max_length][3];//平移数组，其中存放每个关键帧的平移向量
    Quaterniond q_array[max_length];
    double euler_array[max_length][3];
    double quaterniond_array_main[max_length][4];
    vector<bool> need_resample;
    
    ceres::Problem problem;
    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 6;
    ceres::Solver::Summary summary;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);
    //AngleLocalParameterization类的主要作用是指定yaw角优化变量的迭代更新，重载了括号运算
//    ceres::LocalParameterization* angle_local_parameterization =
//    AngleLocalParameterization::Create();
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
    
    //resample
    double min_dis = total_length / (1.0 * max_frame_num);
    //遍历关键帧列表
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    Vector3d last_P = Vector3d(0, 0, 0);
    double dis = 0;
    
    vector<int> special_kf_inOpti_main;
    //resample pose graph, keep the relative pose sparse
//    printf("debug resample");
    for (; it != keyFrameList.end(); it++)
    {
        //earliest_loop_index为第一次闭环帧的index,需要优化的关键帧为从第一次闭环帧到当前帧间的所有关键帧
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Vector3d tmp_t;
        Matrix3d tmp_r;
        //获取关键帧it的位姿
        (*it)->getPose(tmp_t, tmp_r);
        dis += (tmp_t - last_P).norm();
        if((*it)->global_index == earliest_loop_index || dis > min_dis || (*it)->has_loop || (*it)->is_looped || keyFrameList.size() < max_frame_num)
        {
            dis = 0;
            last_P = tmp_t;
            need_resample.push_back(0);
//            printf("debug %d\n",(*it)->global_index);
            
            special_kf_inOpti_main.push_back((*it)->global_index);
        }
        else
        {
            last_P = tmp_t;
            need_resample.push_back(1);
//            printf("debug %d\n",1);
        }
        
        if ((*it)->global_index == cur_index)
            break;
    }
//    cout<<"opti639= "<<special_kf_inOpti_main.size()<<" "<<earliest_loop_index<<endl;
    
    special_kf_mutex_intra.lock();
    special_kf_inOpti_intra.push(special_kf_inOpti_main);
    special_kf_mutex_intra.unlock();
    
    int i = 0;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        (*it)->resample_index = i;
        Quaterniond tmp_q;
        Matrix3d tmp_r;
        Vector3d tmp_t;
        (*it)->getOriginPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i] = tmp_q;
        
        quaterniond_array_main[i][0]=tmp_q.w();
        quaterniond_array_main[i][1]=tmp_q.x();
        quaterniond_array_main[i][2]=tmp_q.y();
        quaterniond_array_main[i][3]=tmp_q.z();
        
        //将矩阵转换为向量
        Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
        euler_array[i][0] = euler_angle.x();
        euler_array[i][1] = euler_angle.y();
        euler_array[i][2] = euler_angle.z();
        //将关键帧列表中所有index>=earliest_loop_index的关键帧的位姿加入到参数块当中
//        problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
        problem.AddParameterBlock(quaterniond_array_main[i], 4, local_parameterization);
        problem.AddParameterBlock(t_array[i], 3);
        //设置约束：如果该帧是最早的闭环帧的情况下，则固定它的位姿
        if ((*it)->global_index == earliest_loop_index)
        {
//            problem.SetParameterBlockConstant(euler_array[i]);
            problem.SetParameterBlockConstant(quaterniond_array_main[i]);
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
                Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                //p̂_j^w - p̂_i^w 计算平移量的偏差
                Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                //p̂_{ij}^{i} = {R̂_i^w}^{-1} (p̂_j^w - p̂_i^w)
                //p̂ ij= (R̂ iw ) (p̂ jw − p̂ iw )
                relative_t = q_array[i-j].inverse() * relative_t;
                //ψ̂ _ij = ψ̂ _j − ψ̂ _i
//                double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
//                ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
//                                                                          relative_yaw, euler_conncected.y(), euler_conncected.z());
//                problem.AddResidualBlock(cost_function, loss_function, euler_array[i-j],
//                                         t_array[i-j],
//                                         euler_array[i],
//                                         t_array[i]);
                
                Quaterniond relative_q=q_array[i-j].inverse()*q_array[i];
                ceres::CostFunction* cost_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),1.0,1.0);
                problem.AddResidualBlock(cost_function, loss_function, quaterniond_array_main[i-j],
                                         t_array[i-j],
                                         quaterniond_array_main[i],
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
        if((*it)->has_loop&&(*it)->is_get_loop_info   )
        {
            int connected_index = getKeyframe((*it)->loop_index)->resample_index;
            if((*it)->loop_index < earliest_loop_index)
            {
                //说明检测回环的可能是检测到连续的好几帧的回环了
                printf("loop bug %d %d\n", (*it)->global_index, (*it)->loop_index);
                assert(false);
            }
            Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
            Vector3d relative_t((*it)->loop_info_better(0), (*it)->loop_info_better(1), (*it)->loop_info_better(2));
            double relative_yaw = (*it)->loop_info_better(3);
            
            //这里 还存在错误的值
//            cout<<"(*it)->loop_info(0)"<<(*it)->loop_info_better(0)<<" (*it)->loop_info(1)"<<(*it)->loop_info_better(1)<<" (*it)->loop_info(2)"<<(*it)->loop_info_better(2)<<" (*it)->loop_info(3)"<<(*it)->loop_info_better(3)<<" "<<i<<endl;
            
            Quaterniond relative_q=(*it)->loop_info_better_q;
            if((*it)->isRemove_loop){//权重为3
//                ceres::CostFunction* cost_function = FourDOFWeightError_forRemove::Create( relative_t.x(), relative_t.y(), relative_t.z(),
//                                                                                relative_yaw, euler_conncected.y(), euler_conncected.z(),3.0);
//                problem.AddResidualBlock(cost_function, NULL, euler_array[connected_index],
//                                         t_array[connected_index],
//                                         euler_array[i],
//                                         t_array[i]);
                
                ceres::CostFunction* cost_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),3.0,3.0/10.0);
                problem.AddResidualBlock(cost_function, NULL, quaterniond_array_main[connected_index],
                                         t_array[connected_index],
                                         quaterniond_array_main[i],
                                         t_array[i]);
                cout<<"降权重 发生过"<<endl;
            }else{//权重为10
//                ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
//                                                                                relative_yaw, euler_conncected.y(), euler_conncected.z());
//                problem.AddResidualBlock(cost_function, NULL, euler_array[connected_index],
//                                         t_array[connected_index],
//                                         euler_array[i],
//                                         t_array[i]);
                
                ceres::CostFunction* cost_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),10.0,1.0);
                problem.AddResidualBlock(cost_function, NULL, quaterniond_array_main[connected_index],
                                         t_array[connected_index],
                                         quaterniond_array_main[i],
                                         t_array[i]);
            }
            
            
            
        }
        if ((*it)->global_index == cur_index)
            break;
        i++;
    }
//    TS(t_global_loop);
//开始优化
    ceres::Solve(options, &problem, &summary);
//    TE(t_global_loop);
//    std::cout << summary.BriefReport() << "\n";
    //优化完成，使用优化后的位姿来更新关键帧列表中index大于等于earliest_loop_index的所有关键帧的位姿
    i = 0;
    int seg_index_cur, seg_index_old;
    seg_index_cur = cur_kf->segment_index;
    seg_index_old = getKeyframe(cur_kf->loop_index)->segment_index;
    cur_seg_index = seg_index_old;
    Vector3d t_drift_it = Vector3d::Zero();
    Matrix3d r_drift_it = Matrix3d::Identity();
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
//        Quaterniond tmp_q;
//        //向量转换为矩阵
//        tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
        
        Quaterniond tmp_q(quaterniond_array_main[i][0],quaterniond_array_main[i][1],quaterniond_array_main[i][2],quaterniond_array_main[i][3]);
        
        Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
        Matrix3d tmp_r = tmp_q.toRotationMatrix();
        if(need_resample[i])
        {
            (*it)-> updatePose(r_drift_it * tmp_t + t_drift_it, r_drift_it * tmp_r);
            
        }
        else
        {
            Vector3d origin_t_it;
            Matrix3d origin_r_it;
            (*it)->getOriginPose(origin_t_it, origin_r_it);
            r_drift_it = tmp_r * origin_r_it.transpose();
            t_drift_it = tmp_t - r_drift_it * origin_t_it;
            (*it)-> updatePose(tmp_t, tmp_r);
            
            r_global_test.push_back(tmp_r);
            t_global_test.push_back(tmp_t);
        }
        (*it)->r_drift=r_drift_it;
        (*it)->t_drift=t_drift_it;
        
        if ((*it)->global_index == cur_index)
            break;
        i++;
        if((*it)->segment_index == seg_index_cur)
            (*it)->segment_index = seg_index_old;
    }
    //根据计算出当前帧的drift，更新全部关键帧位姿
    Vector3d cur_t, origin_t;
    Matrix3d cur_r, origin_r;
    //获取优化后当前帧的位姿cur_t,cur_r
    cur_kf->getPose(cur_t, cur_r);
    //获取优化前有漂移的当前帧的位姿vio_t,vio_r
    cur_kf->getOriginPose(origin_t, origin_r);
    yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(origin_r).x();
//    r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
//    cout<<"r_drift1="<< r_drift<<endl;
    r_drift=cur_r*origin_r.transpose();
//    cout<<"r_drift2="<< r_drift<<endl;
    t_drift = cur_t - r_drift * origin_t;
    
    r_global_test.push_back(r_drift);
    t_global_test.push_back(t_drift);
    r_global.push(r_global_test);
    t_global.push(t_global_test);
    
//    cout<<r_global.size()<<" opti5 test "<<t_global.size()<<" "<<special_kf_inOpti_main.size()<<endl;
    
    //下面代码为把当前关键帧it之后的关键帧的位姿通过求解的偏移量转换到world坐标系下
    for (; it != keyFrameList.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getOriginPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)-> updatePose(P, R);
        
//        cout<<"PoseGraph::opti5 837 测试位姿哪一步变差了：";
//        for(int i=0;i<3;i++){
//            cout<<P[i]<<" ";
//        }
//        cout<<endl;
    }
    
//    keyFrameList_global_fusion_mutex.unlock();
    lastKF_index.push(keyFrameList.back()->global_index);
    
    loop_correct_t = t_drift;
    loop_correct_r = r_drift;
//    cout<<"opti5 调用 ";
    updateVisualization();
    
    //打印全局优化后的位姿
//        for(auto iter=keyFrameList.begin(),iter_end=keyFrameList.end();iter!=iter_end;iter++){
//            Vector3d P;
//            Matrix3d R;
//
//            (*iter)->getPose(P, R);
//            cout<<"P1  "<<P.transpose()<<"R  "<<R<<endl;
//        }
    
}

int pose_index_save=0;
void PoseGraph::updateVisualization()
{
    total_length = 0;
    last_P = Vector3d(0, 0, 0);
    //update visualization
    list<KeyFrame*>::iterator it;
    
    drawMutex.lock();
//    cout<<"开始清空数据"<<endl;
    refine_path.clear();
    refine_path_r.clear();
    
    //
//    refine_r.clear();
//    refine_t.clear();
    
    segment_indexs.clear();
    all_keyframes.clear();
    
    
    
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);
        Quaterniond Q;
        Q = R;
        
//        refine_r.push_back(Q);
//        refine_t.push_back(P);
        
        total_length += (P - last_P).norm();
        last_P = P;
        
        // add key frame to path for visualization
        refine_path.push_back(P.cast<float>());
        refine_path_r.push_back(R.cast<float>());
        segment_indexs.push_back((*it)->segment_index);
        
        
        
        
        KEYFRAME_DATA keyframe_data;
        keyframe_data.header = (*it)->header;
        keyframe_data.translation = P;
        keyframe_data.rotation = Q;
        all_keyframes.push_back(keyframe_data);
    }

//    std::ofstream outFile;
//    string path= "/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/pose"+std::to_string(pose_index_save)+".txt";
//    outFile.open(path);
//    pose_index_save++;
//    if(!outFile){
//        cout<<"打开文件失败"<<endl;
//    }
//    for(auto iter=refine_path.begin(),iter_end=refine_path.end();iter!=iter_end;iter++){
//        outFile<<(*iter).x()<<" "<<(*iter).y()<<" "<<(*iter).z()<<"\n";
//    }
//    outFile.close();
    
//    cout<<"清空并更新完数据"<<pose_index_save<<endl;
    drawMutex.unlock();
//    printf("loop update visualization\n");
}

void PoseGraph::addLoop(int loop_index)
{
    KeyFrame* cur_KF = getLastKeyframe();
    
    KeyFrame* connected_KF = getKeyframe(loop_index);
    Vector3d conncected_P, P;
    Matrix3d connected_R, R;
    cur_KF->getPose(P, R);
    connected_KF->getPose(conncected_P, connected_R);
}



string PoseGraph::getTime()
{
    time_t timep;
    time (&timep); //获取time_t类型的当前时间
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );//对日期和时间进行格式化
    return tmp;
}

void PoseGraph::globalLoopRun(){
    
    while(true){
        
        vins->globalOpti_index_mutex.lock();
        if(vins->start_global_optimization)
        {
            vins->start_global_optimization = false;
            
            int toGlobalOptiNum=vins->kf_global_index.size();
            int x_cur=vins->kf_global_index.front(), y_start=vins->start_kf_global_index.front();
            int cur_global_index=x_cur, start_global_index=y_start;
            vins->kf_global_index.pop();
            vins->start_kf_global_index.pop();
            for(int i=1;i<toGlobalOptiNum;i++){
                x_cur=vins->kf_global_index.front();
                y_start=vins->start_kf_global_index.front();
                cur_global_index=cur_global_index>x_cur?cur_global_index:x_cur;
                start_global_index=start_global_index<y_start?start_global_index:y_start;
                vins->kf_global_index.pop();
                vins->start_kf_global_index.pop();
            }
            vins->globalOpti_index_mutex.unlock();
            
//            TS(loop_thread);
            //暂时注释
//                optimize4DoFLoopPoseGraph_2(cur_global_index, loop_correct_t, loop_correct_r);
            
//            optimize4DoFLoopPoseGraph_3( start_global_index, cur_global_index, loop_correct_t, loop_correct_r);
            vins->t_drift = loop_correct_t;
            vins->r_drift = loop_correct_r;
//            TE(loop_thread);
            isSendGlobalData=true;
//            cout<<"检测到回环 并完成一次全局优化：poseGraph "<<getTime()<<endl;
            
            usleep(1170);
        }else{
            vins->globalOpti_index_mutex.unlock();
        }
        
        //判断 是不是有融合优化了
        usleep(30);
    }
}

void PoseGraph::setVINS(VINS* _vins){
    vins=_vins;
}

void PoseGraph::viewPointClouds()
{
    //暂时注释，点云还没给清楚
//可能会画一些重复的点，因为用的是窗口里面的点的投影 后续把点的id传过来
    
    list<KeyFrame*>::iterator iterator_keyframe;
    for(iterator_keyframe = keyFrameList.begin(); iterator_keyframe != keyFrameList.end(); iterator_keyframe++)
    {
    //    glPushMatrix();
        glPointSize(1); //设备被渲染点的宽度，以像素位单位，默认为1
        glBegin(GL_POINTS);      //把每一个顶点当做一个独立的点进行处理
        //glColor3f(0.0,0.0,0.0);  //点的颜色为黑色
        glColor3f(1.0,1.0,1.0);  //点的颜色为白色
    for(auto it = (*iterator_keyframe)->point_clouds.begin(); it!= (*iterator_keyframe)->point_clouds.end(); it++)
        {
            Vector3d pointCloudWorld;
            pointCloudWorld = (*iterator_keyframe)->relocalize_r * (*it) + (*iterator_keyframe)->relocalize_t;//空指
            glVertex3f((float)(pointCloudWorld.x()), (float)(pointCloudWorld.y()), (float)(pointCloudWorld.z()));
        }
        glEnd();
    
    }
     
}
void PoseGraph::viewPath()
{
    
    
    glLineWidth(3);
    glBegin ( GL_LINES );
    glColor3f ( 0.8f,0.f,0.f );
    glVertex3f( 0,0,0 );
    glVertex3f( 1,0,0 );
    glColor3f( 0.f,0.8f,0.f);
    glVertex3f( 0,0,0 );
    glVertex3f( 0,1,0 );
    glColor3f( 0.2f,0.2f,1.f);
    glVertex3f( 0,0,0 );
    glVertex3f( 0,0,1 );
    glEnd();
    
    
    
//    Eigen::Vector3f tmp_path;
    glColor3f(1.0f, 0.0f, 0.0f);
    glLineWidth(2);
    glBegin(GL_LINE_STRIP);
    
    
    drawMutex.lock();
    
//    cout<<"refine_path.size="<<refine_path.size()<<endl;
//    TS(copy_refine_path);
    unique_lock<mutex> lock_2(mMutexkeyFrameList);
    
    refine_path_r_draw=refine_path_r;
//    TE(copy_refine_path);
//
    
    
    unique_lock<mutex> lock_3(refine_path_draw_mutex);
    
    refine_path_draw=refine_path;
    lock_2.unlock();
    vector<Vector3f>::iterator iterator_keyframe;
    for(iterator_keyframe = refine_path_draw.begin(); iterator_keyframe != refine_path_draw.end(); iterator_keyframe++)
    {
        glVertex3f((*iterator_keyframe).x(), (*iterator_keyframe).y(), (*iterator_keyframe).z());
    }
    lock_3.unlock();
    drawMutex.unlock();
    glEnd();
    
    
        
//   int fusion_graph_len=fusion_otherGraph.size();
//   if(fusion_graph_len!=0){
//
//       unique_lock<mutex> lock_4(refine_path_mutex);
//       for(int j=0;j<fusion_graph_len;j++){
//         PoseGraph* poseGraph_fusionOther=fusion_otherGraph[j];
//
//           int updateKf_index=fusion_relative_isUpdate[j];
//
//           drawMutex.lock();
//           unique_lock<mutex> lock_5(mMutexkeyFrameList);
//           vector<Vector3f> refine_path_other=poseGraph_fusionOther->refine_path;
//           lock_5.unlock();
//           drawMutex.unlock();
//           int hasKf_len=refine_path_other.size();
//
//           int best_relativePose_index=best_relative_pose_index[j];
//           Matrix3f r_relative_best=relative_r_mainToOther[j][best_relativePose_index];
//           Vector3f t_relative_best=relative_t_mainToOther[j][best_relativePose_index];
//
//
//
//        unique_lock<mutex> lock_6(refine_path_draw_mutex);
//           if(updateKf_index!=hasKf_len){
//               for(int startUpdateIndex=updateKf_index;startUpdateIndex<hasKf_len;startUpdateIndex++){
//                   Vector3f update_t=r_relative_best*refine_path_other[startUpdateIndex]+t_relative_best;
//
//                   poseGraph_fusionOther->refine_path_draw.push_back(update_t);
//               }
//
//               fusion_relative_isUpdate[j]=hasKf_len;
//
//           }
//
//         glColor3f(0.0f, 1.0f, 0.0f);
//         glLineWidth(2);
//         glBegin(GL_LINE_STRIP);
//
//
//
//         for(auto iter=poseGraph_fusionOther->refine_path_draw.begin(),iter_end=poseGraph_fusionOther->refine_path_draw.end();iter!=iter_end;iter++){
//             glVertex3f((*iter).x(), (*iter).y(), (*iter).z());
//         }
//
//
//        lock_6.unlock();
//        glEnd();
//       }
//        lock_4.unlock();
//    }
   
}

void PoseGraph::viewPath_2()
{
    
    
    glLineWidth(3);
    glBegin ( GL_LINES );
    glColor3f ( 0.8f,0.f,0.f );
    glVertex3f( 0,0,0 );
    glVertex3f( 1,0,0 );
    glColor3f( 0.f,0.8f,0.f);
    glVertex3f( 0,0,0 );
    glVertex3f( 0,1,0 );
    glColor3f( 0.2f,0.2f,1.f);
    glVertex3f( 0,0,0 );
    glVertex3f( 0,0,1 );
    glEnd();
    
    
    
//    Eigen::Vector3f tmp_path;
    glColor3f(1.0f, 0.0f, 0.0f);
    glLineWidth(2);
    glBegin(GL_LINE_STRIP);
    
    unique_lock<mutex> lock_3(refine_path_draw_mutex);
    unique_lock<mutex> lock_2(mMutexkeyFrameList);
    //防止正在清空数据
    drawMutex.lock();
    refine_path_r_draw=refine_path_r;
    refine_path_draw=refine_path;
    drawMutex.unlock();
    lock_2.unlock();
    
    vector<Vector3f>::iterator iterator_keyframe;
    for(iterator_keyframe = refine_path_draw.begin(); iterator_keyframe != refine_path_draw.end(); iterator_keyframe++)
    {
        glVertex3f((*iterator_keyframe).x(), (*iterator_keyframe).y(), (*iterator_keyframe).z());
    }
    lock_3.unlock();
    
    glEnd();
    
      
       for(int j=0;j<10 ;j++){
           if(fusion_relative_isUpdate[j]==-2)
               continue;

           fusion_poseGraph_mutex.lock();
           PoseGraph* poseGraph_fusionOther=fusion_otherGraph[j];//这个没赋值
           fusion_poseGraph_mutex.unlock();
           
           unique_lock<mutex> lock_4(poseGraph_fusionOther->refine_path_mutex);
           int updateKf_index=fusion_relative_isUpdate[j];
           Matrix3f r_relative_best=relative_r_mainToOther[j];
           Vector3f t_relative_best=relative_t_mainToOther[j];
           lock_4.unlock();
            
           poseGraph_fusionOther->drawMutex.lock();
           unique_lock<mutex> lock_5(poseGraph_fusionOther->mMutexkeyFrameList);
           vector<Vector3f> refine_path_other=poseGraph_fusionOther->refine_path;
           vector<Matrix3f> refine_path_r_other=poseGraph_fusionOther->refine_path_r;
           lock_5.unlock();
           poseGraph_fusionOther->drawMutex.unlock();
           int hasKf_len=refine_path_other.size();
           
            
           unique_lock<mutex> lock_6(poseGraph_fusionOther->refine_path_draw_mutex);
           if(updateKf_index!=hasKf_len){
               int refine_path_draw_len=poseGraph_fusionOther->refine_path_draw.size();

               for(int startUpdateIndex=updateKf_index;startUpdateIndex<hasKf_len;startUpdateIndex++){
                   Vector3f update_t=r_relative_best.transpose()*(refine_path_other[startUpdateIndex]-t_relative_best);
                   Matrix3f update_r=r_relative_best.transpose()*refine_path_r_other[startUpdateIndex];
                   if(startUpdateIndex<refine_path_draw_len){
                       poseGraph_fusionOther->refine_path_draw[startUpdateIndex]=update_t;
                       poseGraph_fusionOther->refine_path_r_draw[startUpdateIndex]=update_r;
                   }else{
                       poseGraph_fusionOther->refine_path_draw.push_back(update_t);
                       poseGraph_fusionOther->refine_path_r_draw.push_back(update_r);
                   }
               }

               fusion_relative_isUpdate[j]=hasKf_len;
               
           }
         glColor3f(0.0f, 1.0f, 0.0f);
         glLineWidth(2);
         glBegin(GL_LINE_STRIP);
               
        
           
         for(auto iter=poseGraph_fusionOther->refine_path_draw.begin(),iter_end=poseGraph_fusionOther->refine_path_draw.end();iter!=iter_end;iter++){
             glVertex3f((*iter).x(), (*iter).y(), (*iter).z());
         }
           
        lock_6.unlock();
        glEnd();
        
//       }
        
    }
   
}
bool hasData=false;
void PoseGraph::viewPath_3()
{
    
    
    glLineWidth(3);
    glBegin ( GL_LINES );
    glColor3f ( 0.8f,0.f,0.f );
    glVertex3f( 0,0,0 );
    glVertex3f( 1,0,0 );
    glColor3f( 0.f,0.8f,0.f);
    glVertex3f( 0,0,0 );
    glVertex3f( 0,1,0 );
    glColor3f( 0.2f,0.2f,1.f);
    glVertex3f( 0,0,0 );
    glVertex3f( 0,0,1 );
    glEnd();
    
    
    
//    Eigen::Vector3f tmp_path;
    glColor3f(1.0f, 0.0f, 0.0f);
    glLineWidth(2);
    glBegin(GL_LINE_STRIP);
    
    unique_lock<mutex> lock_3(refine_path_draw_mutex);
//    unique_lock<mutex> lock_2(mMutexkeyFrameList);
    //防止正在清空数据
    drawMutex.lock();
    refine_path_r_draw.clear();
    refine_path_draw.clear();
    refine_path_r_draw=refine_path_r;
    refine_path_draw=refine_path;
    
//    lock_2.unlock();
    
    if(refine_path_draw.size()>0){
        hasData=true;
    }else{
        if(hasData){
            cout<<"画图消失  没有数据了"<<endl;
        }
    }
    
    vector<Vector3f>::iterator iterator_keyframe;
    for(iterator_keyframe = refine_path_draw.begin(); iterator_keyframe != refine_path_draw.end(); iterator_keyframe++)
    {
        glVertex3f((*iterator_keyframe).x(), (*iterator_keyframe).y(), (*iterator_keyframe).z());
    }
    drawMutex.unlock();
    lock_3.unlock();
    
    glEnd();
    
   
       for(int j=1;j<10 ;j++){
           
           if(fusion_relative_isUpdate[j]==-2)
               continue;
//           cout<<"cur j="<<j<<", "<<fusion_relative_isUpdate[j]<<endl;
           
           fusion_poseGraph_mutex.lock();
           PoseGraph* poseGraph_fusionOther=fusion_otherGraph[j];//这个没赋值
           fusion_poseGraph_mutex.unlock();
           
//           unique_lock<mutex> lock_4(poseGraph_fusionOther->refine_path_mutex);
//           int updateKf_index=fusion_relative_isUpdate[j];
//           lock_4.unlock();
//poseGraph_fusionOther 空指
           unique_lock<mutex> lock_6(poseGraph_fusionOther->refine_path_draw_mutex);
//           unique_lock<mutex> lock_5(poseGraph_fusionOther->mMutexkeyFrameList);
           poseGraph_fusionOther->drawMutex.lock();
           
           poseGraph_fusionOther->refine_path_draw=poseGraph_fusionOther->refine_path;
           poseGraph_fusionOther->refine_path_r_draw=poseGraph_fusionOther->refine_path_r;
           
           
//           lock_5.unlock();
           
           
           if(j==1)
               glColor3f(0.0f, 1.0f, 0.0f);
           else
               glColor3f(0.0f, 0.0f, 1.0f);
           glLineWidth(2);
           glBegin(GL_LINE_STRIP);
              
             for(auto iter=poseGraph_fusionOther->refine_path_draw.begin(),iter_end=poseGraph_fusionOther->refine_path_draw.end();iter!=iter_end;iter++){
                 glVertex3f((*iter).x(), (*iter).y(), (*iter).z());
             }
               poseGraph_fusionOther->drawMutex.unlock();
            lock_6.unlock();
            glEnd();
        
//       }
        
    }
   
}



vector<Vector3d> PoseGraph::get_t_global(){
    vector<Vector3d> front_t=t_global.front();
    t_global.pop();
    return front_t;
}
vector<double> PoseGraph::get_yaw_global(){
    return yaw_global;
}
vector<Matrix3d> PoseGraph::get_r_global(){
    vector<Matrix3d> front_r=r_global.front();
    r_global.pop();
    return front_r;
}
//Vector3d PoseGraph::get_t_drift(){
//    return t_drift;
//}
//double PoseGraph::get_yaw_drift(){
//    return yaw_drift;
//}
//Matrix3d PoseGraph::get_r_drift(){
//    return  r_drift;
//}

void PoseGraph::updateCorrectLoopPose(int endIndex){
    unique_lock<mutex> lock(mMutexkeyFrameList);
    int kfSum=keyFrameList.size();
    if(kfSum!=0){
        list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
        list<KeyFrame*>::reverse_iterator rit_end = keyFrameList.rend();
        
        int fromIndex_reverse=kfSum-endIndex;
        int endIndex_reverse=kfSum-lastKF_index.front();
        lastKF_index.pop();
        
        for(int i=0;rit!=rit_end;rit++,i++){
            if(i>=fromIndex_reverse){
                if(i<=endIndex_reverse){
                    Vector3d P;
                    Matrix3d R;
                    (*rit)->getOriginPose(P, R);
//                    (*rit)->getPose(P, R);
                    
                    P = r_drift * P + t_drift;
                    R = r_drift * R;
                    (*rit)-> updatePose(P, R);
                    
//                    cout<<"PoseGraph::updateCorrectLoopPose 测试位姿哪一步变差了：";
//                    for(int i=0;i<3;i++){
//                        cout<<P[i]<<" ";
//                    }
//                    cout<<endl;
                }
                
            }
        }
        cout<<"updateCorrectLoopPose 调用 ";
        updateVisualization();
        
    }
    
}


//void PoseGraph::globalLoopRun(){
//
//    while(true){
//
//        vins->globalOpti_index_mutex.lock();
//        if(vins->start_global_optimization)
//        {
//            vins->start_global_optimization = false;
//
//            int toGlobalOptiNum=vins->kf_global_index.size();
//            int x_cur=vins->kf_global_index.front(), y_start=vins->start_kf_global_index.front();
//            int cur_global_index=x_cur, start_global_index=y_start;
//            vins->kf_global_index.pop();
//            vins->start_kf_global_index.pop();
//            for(int i=1;i<toGlobalOptiNum;i++){
//                x_cur=vins->kf_global_index.front();
//                y_start=vins->start_kf_global_index.front();
//                cur_global_index=cur_global_index>x_cur?cur_global_index:x_cur;
//                start_global_index=start_global_index<y_start?start_global_index:y_start;
//                vins->kf_global_index.pop();
//                vins->start_kf_global_index.pop();
//            }
//            vins->globalOpti_index_mutex.unlock();
//
//            TS(loop_thread);
//            optimize4DoFLoopPoseGraph_2(cur_global_index, loop_correct_t, loop_correct_r);
////            optimize4DoFLoopPoseGraph_3( start_global_index, cur_global_index, loop_correct_t, loop_correct_r);
//            vins->t_drift = loop_correct_t;
//            vins->r_drift = loop_correct_r;
//            TE(loop_thread);
//            isSendGlobalData=true;
//            cout<<"检测到回环 并完成一次全局优化："<<getTime()<<endl;
//
//            usleep(1170);
//        }else{
//            vins->globalOpti_index_mutex.unlock();
//        }
//        usleep(30);
//    }
//}


vector<Vector3d> PoseGraph::get_t_global_multiClient(){
    rt_global_multiClient_mutex.lock();
    vector<Vector3d> t_test=t_global_multiClient.front();
    t_global_multiClient.pop();
    rt_global_multiClient_mutex.unlock();
    return t_test;
}
vector<Matrix3d> PoseGraph::get_r_global_multiClient(){
    rt_global_multiClient_mutex.lock();
    vector<Matrix3d> r_test=r_global_multiClient.front();
    r_global_multiClient.pop();
    rt_global_multiClient_mutex.unlock();
    return r_test;
}
vector<int> PoseGraph::get_kf_id_hasComPlace_withOtherMap(){
    rt_global_multiClient_mutex.lock();
    vector<int> kf_id_test=kf_id_hasComPlace_withOtherMap.front();
    kf_id_hasComPlace_withOtherMap.pop();
    rt_global_multiClient_mutex.unlock();
    return kf_id_test;
}

void PoseGraph::add2_t_global_multiClient(vector<Vector3d> t){
    rt_global_multiClient_mutex.lock();
    t_global_multiClient.push(t);
    rt_global_multiClient_mutex.unlock();
}
void PoseGraph::add2_r_global_multiClient(vector<Matrix3d> r){
    rt_global_multiClient_mutex.lock();
    r_global_multiClient.push(r);
    rt_global_multiClient_mutex.unlock();
}
void PoseGraph::add2_kf_id_hasComPlace_withOtherMap(vector<int> kf_id){
    rt_global_multiClient_mutex.lock();
    kf_id_hasComPlace_withOtherMap.push(kf_id);
    rt_global_multiClient_mutex.unlock();
}

void PoseGraph::pop_kf_id_hasComPlace_withOtherMap(){
    rt_global_multiClient_mutex.lock();
    kf_id_hasComPlace_withOtherMap.pop();
    rt_global_multiClient_mutex.unlock();
}

//void PoseGraph::clear_t_global_multiClient(){
//    t_global_multiClient.clear();
//}
//void PoseGraph::clear_r_global_multiClient(){
//    r_global_multiClient.clear();
//}
//void PoseGraph::clear_kf_id_hasComPlace_withOtherMap(){
//    kf_id_hasComPlace_withOtherMap.clear();
//}
