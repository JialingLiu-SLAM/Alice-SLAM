//
//  KeyFrame.cpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/27.
//  Copyright © 2020 zx. All rights reserved.
//

#include "KeyFrame.hpp"

KeyFrame::KeyFrame(){
    use_retrive = 0;
    is_looped = 0;
    has_loop = 0;
    check_loop = 0;
    
    is_global_looped=0;
    is_global_looped_index=-1;
    has_global_loop=0;
//    check_global_loop=0;
    
    IsOriginUpdate=false;
    sendLoop=false;
    sendRejectWithF=false;
    
    
    has_fusion=0;
    is_fusioned=0;
    
    is_des_end=false;
    is_get_loop_info=false;
    pointNum=0;
    
    t_drift.setZero();
    r_drift.setIdentity();
    
//    check_global_loop.resize(10);
    for(int i=0;i<10;i++){
        check_global_loop[i]=0;
    }
    
    is_Send=false;
    isRemove_loop=false;
    isRemove_globalLoop=false;
    
    
//    isuse=0;
    T_wOld_i.setZero();
    R_wOld_i.setIdentity();
    
    is_bowIndex_end=false;
    bitNum=0;
    
    resample_globalIndex=-1;
        
}

template <typename T>
void reduceVector2(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

template <typename T>
void reduceVector3(vector<T> &v, vector<T> &v_origin, vector<uchar> status_server)
{
    //虽然前面的匹配，它是匹配上的坏点，但是新的一次匹配，不需要减掉前面得到的坏点（因为这个是相对于那一次匹配上的点）
    //后续可以验证看看 status的值 和前面一次的 应该不是差不多
    
//    //先计算新的status
//    for(int i=0;i<send_status.size();i++){
//        if(send_status[i] && !status_server[i]){
//
//        }
//    }
   
   
    
    int j = 0;
    for (int i = 0; i < int(v_origin.size()); i++)
        if (status_server[i])
            v[j++] = v_origin[i];
    v.resize(j);
}

//可能先和别人发生回环，再和自己发生回环
void KeyFrame::rejectWithF(vector<cv::Point2f> &measurements_old,
                           vector<cv::Point2f> &measurements_old_norm)
{
    if (measurements_old.size() >= 8)
    {
        measurements_old_norm.clear();
        
        for (unsigned int i = 0; i < measurements_old.size(); i++)
        {
            cv::Point2f norm_pt;
            norm_pt.x = (measurements_old[i].x - c->PX_server)/c->FOCUS_LENGTH_X_server;
            norm_pt.y = (measurements_old[i].y - c->PY_server)/c->FOCUS_LENGTH_Y_server;
            measurements_old_norm.push_back(norm_pt);
        }
        
//        cout<<"global_index="<<global_index<<endl;
        
        vector<uchar> status;
        //旧的是不会变的 永远用旧的做匹配
        cv::findFundamentalMat(measurements_origin, measurements_old, cv::FM_RANSAC, 2.0, 0.99, status);
//        reduceVector2(point_clouds, status);
        reduceVector3(point_clouds,point_clouds_origin, status);//0720改
//        reduceVector2(measurements, status);
        reduceVector3(measurements,measurements_origin, status);
        reduceVector2(measurements_old, status);
        reduceVector2(measurements_old_norm, status);
//        reduceVector2(features_id, status);
        reduceVector3(features_id,features_id_origin, status);
        //新加的
//        reduceVector3(win_keyPoint_depth,win_keyPoint_depth_origin, status);
        
        //告诉客户端 剔除外点
        sendRejectWithF=true;
        send_status=status;
    }
}

/**
 ** search matches by guide descriptor match
 **当前关键帧与闭环候选帧进行BRIEF描述子匹配，这里相当于是在2d-2d之间进行匹配
 **/
void KeyFrame::searchByDes(std::vector<cv::Point2f> &measurements_old,
                           std::vector<cv::Point2f> &measurements_old_norm,
                           const std::vector<BRIEF::bitset> &descriptors_old,
                           const std::vector<cv::KeyPoint> &keypoints_old)
{
//    printf("loop_match before cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
    std::vector<int> dis_cur_old;
    std::vector<uchar> status;
    for(int i = 0; i < window_descriptors.size(); i++)
    {
        int bestDist = 256;
        int bestIndex = -1;
        for(int j = 0; j < descriptors_old.size(); j++)
        {
            int dis = HammingDis(window_descriptors[i], descriptors_old[j]);
            if(dis < bestDist)
            {
                bestDist = dis;
                bestIndex = j;
            }
        }
        if(bestDist < 256)
        {
            measurements_old.push_back(keypoints_old[bestIndex].pt);
            dis_cur_old.push_back(bestDist);
        }
    }
    rejectWithF(measurements_old, measurements_old_norm);
//    printf("loop_match after cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
}

bool KeyFrame::findConnectionWithOldFrame(const KeyFrame* old_kf,
                                          const std::vector<cv::Point2f> &cur_pts, const std::vector<cv::Point2f> &old_pts,
                                          std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm)
{
//    cout<<global_index<<" test loop global_index cur-old"<<old_kf->global_index<<endl;
    searchByDes(measurements_old, measurements_old_norm, old_kf->descriptors, old_kf->keypoints);
    return true;
}


//这个似乎不再使用了
bool KeyFrame::solveRelativePoseByPnP(std::vector<cv::Point2f> &measurements_cur_norm, Matrix3f &R_relative, Vector3f &T_relative,vector<cv::Point3f> &pts_3_vector,bool isAdd_otherClientId)
{
    //这里可以给初值
    //solve PnP get pose refine
    cv::Mat r, rvec(3,1,CV_64FC1), t(3,1,CV_64FC1), D, tmp_r;
    //2020 8 把double换成float
    cv::Mat K = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    
//    vector<cv::Point3f> pts_3_vector;
    bool pnp_succ = false;
    
  
    assert(pts_3d_server.size()==measurements.size());
    //如果点云有问题 则用2D图像坐标恢复出3d相机坐标系
    for(int i=0,len=pts_3d_server.size();i<len;i++){
        Vector3d corre_pts_3=pts_3d_server[i];
        pts_3_vector.push_back(cv::Point3f((float)corre_pts_3.x(),(float)corre_pts_3.y(),(float)corre_pts_3.z()));
    }
    cout<<"匹配点的数量"<<pts_3_vector.size()<<endl;
    //2020 8 pnp——kf关键帧之间的求解，设的15 求解老帧的位姿，设的30，但是这个没有用
    if(pts_3_vector.size()>=15)
    {
        if(!use_retrive)
        {
            vector<int> status;
            if(isAdd_otherClientId){
                
                //说明有初始值
                cv::eigen2cv(R_relative, tmp_r);
                cv::Rodrigues(tmp_r, rvec);
                cv::eigen2cv(T_relative, t);
                pnp_succ=cv::solvePnPRansac(pts_3_vector, measurements_cur_norm, K, D, rvec, t,1,100,6.0,0.99,status,cv::SOLVEPNP_EPNP);
//                pnp_succ = cv::solvePnP(pts_3_vector, measurements_cur_norm, K, D, rvec, t, 1,cv::SOLVEPNP_EPNP);
            }else{
//                pnp_succ=cv::solvePnPRansac(pts_3_vector, measurements_cur_norm, K, D, rvec, t,0,100,8.0,0.99,status,cv::SOLVEPNP_EPNP);
                pnp_succ = cv::solvePnP(pts_3_vector, measurements_cur_norm, K, D, rvec, t, 0,cv::SOLVEPNP_EPNP);
            }
            
        }
        
    }else{
        cout<<"pts_3_vector.size()>=15 ? "<<pts_3_vector.size()<<endl;
    }
    
    if(!pnp_succ)
    {
        cout << "multi client loop pnp failed !" << endl;
        return false;
    }
    else
    {
        cout << "multi client  loop pnp succ !" << endl;
    }
    cout<<"rvec:"<<endl<<rvec<<endl;
    
    
    cout<<"t:"<<endl<<t<<endl;
    
    
    cv::Rodrigues(rvec, r);
    
    cv::cv2eigen(r, R_relative);
    
    cv::cv2eigen(t, T_relative);
    
//    cout<<"R_relative:"<<endl;
//    for (int i=0; i<3; i++) {
//        for (int j=0; j<3; j++) {
//            cout<<R_relative(i,j)<<" ";
//        }
//        cout<<endl;
//    }
//    cout<<endl;
//
//    cout<<"T_relative:"<<endl;
//    for (int j=0; j<3; j++) {
//        cout<<T_relative[j]<<" ";
//    }
//    cout<<endl;
    
    
    return true;
}

//3d点是世界坐标系的点
bool KeyFrame::solveRelativePoseByPnP_2(std::vector<cv::Point2f> &measurements_cur_norm, Matrix3f &R_relative, Vector3f &T_relative,vector<cv::Point3f> &pts_3_vector,bool isAdd_otherClientId)
{
    //这里可以给初值
    //solve PnP get pose refine
    cv::Mat r, rvec(3,1,CV_64FC1), t(3,1,CV_64FC1), D, tmp_r;
    //2020 8 把double换成float
    cv::Mat K = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    
//    vector<cv::Point3f> pts_3_vector;
    bool pnp_succ = false;
    
  
    assert(point_clouds.size()==measurements.size());
    //如果点云有问题 则用2D图像坐标恢复出3d相机坐标系
    for(int i=0,len=point_clouds.size();i<len;i++){
        Vector3d corre_pts_3=point_clouds[i];
        pts_3_vector.push_back(cv::Point3f((float)corre_pts_3.x(),(float)corre_pts_3.y(),(float)corre_pts_3.z()));
        
//        Matrix3d r_w_i=R_w_i;
//        Vector3d t_w_i=T_w_i;
//        
//        Matrix3d ric=c->ric_client;
//        Vector3d tic=c->tic_client;
//        
//        Matrix3d r_w_c=r_w_i*ric;
//        Vector3d t_w_c=r_w_i*tic+ t_w_i;
//        
//        Matrix3d r_c_w=r_w_c.transpose();
//        Vector3d t_c_w=-r_c_w*t_w_c;
//        
//        Vector3d p3D=r_c_w*corre_pts_3+t_w_c;
//        if(p3D[2]>20){
//            pts_3_vector.pop_back();
//            for(int j=i;j<len-1;j++){
//                measurements_cur_norm[j]=measurements_cur_norm[j+1];
//            }
//            measurements_cur_norm.pop_back();
//            cout<<"排除异常点"<<endl;
//        }
    }
    cout<<"匹配点的数量"<<pts_3_vector.size()<<endl;
    //2020 8 pnp——kf关键帧之间的求解，设的15 求解老帧的位姿，设的30，但是这个没有用
    if(pts_3_vector.size()>=15)
    {
        if(!use_retrive)
        {
            vector<int> status;
            if(isAdd_otherClientId){
                
                //说明有初始值
                cv::eigen2cv(R_relative, tmp_r);
                cv::Rodrigues(tmp_r, rvec);
                cv::eigen2cv(T_relative, t);
                pnp_succ=cv::solvePnPRansac(pts_3_vector, measurements_cur_norm, K, D, rvec, t,1,100,6.0,0.99,status,cv::SOLVEPNP_EPNP);
//                pnp_succ = cv::solvePnP(pts_3_vector, measurements_cur_norm, K, D, rvec, t, 1,cv::SOLVEPNP_EPNP);
            }else{
//                pnp_succ=cv::solvePnPRansac(pts_3_vector, measurements_cur_norm, K, D, rvec, t,0,100,8.0,0.99,status,cv::SOLVEPNP_EPNP);
                pnp_succ = cv::solvePnP(pts_3_vector, measurements_cur_norm, K, D, rvec, t, 0,cv::SOLVEPNP_EPNP);
            }
            
        }
        
    }else{
        cout<<"pts_3_vector.size()>=15 ? "<<pts_3_vector.size()<<endl;
    }
    
    if(!pnp_succ)
    {
        cout << "multi client loop pnp failed !" << endl;
        return false;
    }
    else
    {
        cout << "multi client  loop pnp succ !" << endl;
    }
   
//    cout<<"rvec:"<<endl<<rvec<<endl;
//    cout<<"t:"<<endl<<t<<endl;
    
    
//    cv::Rodrigues(rvec, r);
//
//    cv::cv2eigen(r, R_relative);
//
//    cv::cv2eigen(t, T_relative);
    
    
    
    cv::Rodrigues(rvec, r);
    Matrix3d R_loop;
    cv::cv2eigen(r, R_loop);
    Vector3d T_loop;
    cv::cv2eigen(t, T_loop);
    
    
//    Vector3d old_T_drift;
//    Matrix3d old_R_drift;
//    cam2Imu(T_loop, R_loop, old_T_drift, old_R_drift);
    
//    R_relative=old_R_drift.cast<float>();
//    T_relative=old_T_drift.cast<float>();
    
        R_relative=R_loop.cast<float>();
        T_relative=T_loop.cast<float>();
    //这个是有初始值的情况
//    T_relative = T_w_i_old + R_w_i_old * old_R_drift.transpose() * (T_w_i - old_T_drift);
//    R_relative = R_w_i_old * old_R_drift.transpose() * R_w_i;

    
        return true;
}

bool KeyFrame::solveRelativePoseByPnP_3(std::vector<cv::Point2f> measurements_cur_norm, Matrix3d &R_relative, Vector3d &T_relative,vector<Vector3d> point_3d_cur_real,bool isAdd_otherClientId)
{
    //这里可以给初值
    //solve PnP get pose refine
    cv::Mat r, rvec(3,1,CV_64FC1), t(3,1,CV_64FC1), D, tmp_r;
    //2020 8 把double换成float
    cv::Mat K = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    
//    vector<cv::Point3f> pts_3_vector;
    bool pnp_succ = false;
    
    vector<cv::Point3f> pts_3_vector;
    assert(point_3d_cur_real.size()==measurements_cur_norm.size());
    //如果点云有问题 则用2D图像坐标恢复出3d相机坐标系
    for(int i=0,len=point_3d_cur_real.size();i<len;i++){
        Vector3d corre_pts_3=point_clouds[i];
        pts_3_vector.push_back(cv::Point3f((float)corre_pts_3.x(),(float)corre_pts_3.y(),(float)corre_pts_3.z()));
        
    }
  
    //2020 8 pnp——kf关键帧之间的求解，设的15 求解老帧的位姿，设的30，但是这个没有用
    if(pts_3_vector.size()>=15)
    {
        if(!use_retrive)
        {
            vector<int> status;
            if(isAdd_otherClientId){
                
                //说明有初始值
                cv::eigen2cv(R_relative, tmp_r);
                cv::Rodrigues(tmp_r, rvec);
                cv::eigen2cv(T_relative, t);
                pnp_succ=cv::solvePnPRansac(pts_3_vector, measurements_cur_norm, K, D, rvec, t,1,100,6.0,0.99,status,cv::SOLVEPNP_EPNP);
//                pnp_succ = cv::solvePnP(pts_3_vector, measurements_cur_norm, K, D, rvec, t, 1,cv::SOLVEPNP_EPNP);
            }else{
//                pnp_succ=cv::solvePnPRansac(pts_3_vector, measurements_cur_norm, K, D, rvec, t,0,100,8.0,0.99,status,cv::SOLVEPNP_EPNP);
                pnp_succ = cv::solvePnP(pts_3_vector, measurements_cur_norm, K, D, rvec, t, 0,cv::SOLVEPNP_EPNP);
            }
            
        }
        
    }else{
        cout<<"pts_3_vector.size()>=15 ? "<<pts_3_vector.size()<<endl;
    }
    
    if(!pnp_succ)
    {
        cout << "multi client loop pnp failed !" << endl;
        return false;
    }
    else
    {
        cout << "multi client  loop pnp succ !" << endl;
    }
       
    
    cv::Rodrigues(rvec, r);
//    Matrix3d R_loop;
    cv::cv2eigen(r, R_relative);
//    Vector3d T_loop;
    cv::cv2eigen(t, T_relative);
    
    
    
//    R_relative=R_loop;
//    T_relative=T_loop;

    return true;
}


void KeyFrame::cam2Imu(Eigen::Vector3d T_c_w,
                       Eigen::Matrix3d R_c_w,
                       Eigen::Vector3d &t_w_i,
                       Eigen::Matrix3d &r_w_i)
{
    r_w_i = (c->ric_client * R_c_w).inverse();
    t_w_i = -R_c_w.inverse() * T_c_w - r_w_i * c->tic_client;
    
//    r_w_i = (qic* R_c_w).inverse();
//    t_w_i = -R_c_w.inverse() * T_c_w - r_w_i * tic;
    

}

void KeyFrame::cam2Imu(Eigen::Vector3f T_c_w,
                       Eigen::Matrix3f R_c_w,
                       Eigen::Vector3f &t_w_i,
                       Eigen::Matrix3f &r_w_i)
{
    r_w_i = (c->ric_client.cast<float>() * R_c_w).inverse();
    t_w_i = -R_c_w.inverse() * T_c_w - r_w_i * c->tic_client.cast<float>();
//    cout<<"keyframe :"<<endl;
//    cout<<qic<<endl;
//    cout<<tic<<endl;
    
//    r_w_i = (qic.cast<float>()* R_c_w).inverse();
//    t_w_i = -R_c_w.inverse() * T_c_w - r_w_i * tic.cast<float>();
    
     
    

}

//老帧的3D坐标 新帧的2D坐标
void KeyFrame::rejectWithF_server(vector<cv::Point2f> &measurements_cur,
                           vector<cv::Point2f> &measurements_cur_norm,Client* c_old)
{
    if (measurements_cur.size() >= 8)
    {
        measurements_cur_norm.clear();
        
        for (unsigned int i = 0; i < measurements_cur.size(); i++)
        {
            cv::Point2f norm_pt;
            norm_pt.x = (measurements_cur[i].x - c_old->PX_server)/c_old->FOCUS_LENGTH_X_server;
            norm_pt.y = (measurements_cur[i].y - c_old->PY_server)/c_old->FOCUS_LENGTH_Y_server;
            measurements_cur_norm.push_back(norm_pt);
        }
        
        vector<uchar> status;
        //永远都用旧的匹配 但是不用做减法 可以让新的做减法
        cv::findFundamentalMat(measurements_origin, measurements_cur, cv::FM_RANSAC, 2.0, 0.99, status);
        //只留下status公共为1的
//        reduceVector3(point_clouds, status);//点云信息 暂时没改
        reduceVector3(point_clouds,point_clouds_origin, status);//算相对位姿
        reduceVector3(measurements,measurements_origin, status);
        reduceVector2(measurements_cur, status );//如果前面回环检测没有问题，那么这里应该和上面一样减 要求出相对位姿
        reduceVector2(measurements_cur_norm, status);
        reduceVector3(features_id,features_id_origin, status);//这个的数量和measurements是一样的
        
//        reduceVector3(win_point_z, win_point_z_origin, status);
//        reduceVector3(win_keyPoint_depth,win_keyPoint_depth_origin, status);
//        reduceVector3(pts_3d_server, pts_3d_server_origin, status);
        //告诉客户端 剔除外点
//        sendRejectWithF=true;
//        send_status=status;
        
        
    }
}

void KeyFrame::searchByDes_server(std::vector<cv::Point2f> &measurements_cur,
                           std::vector<cv::Point2f> &measurements_cur_norm,
                           const std::vector<BRIEF::bitset> &descriptors_cur,
                           const std::vector<cv::KeyPoint> &keypoints_cur,Client* c_old)
{
//    printf("loop_match before cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_cur.size());
    std::vector<int> dis_cur_old;
    std::vector<uchar> status;
    for(int i = 0; i < window_descriptors.size(); i++)
    {
        int bestDist = 256;
        int bestIndex = -1;
        for(int j = 0; j < descriptors_cur.size(); j++)
        {
            int dis = HammingDis(window_descriptors[i], descriptors_cur[j]);
            if(dis < bestDist)
            {
                bestDist = dis;
                bestIndex = j;
            }
        }
        if(bestDist < 256)
        {
            measurements_cur.push_back(keypoints_cur[bestIndex].pt);
            dis_cur_old.push_back(bestDist);
        }
    }
    rejectWithF_server(measurements_cur, measurements_cur_norm,c_old);
//    printf("loop_match after cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_cur.size());
}

void KeyFrame::rejectWithF_server_mapFuse(vector<cv::Point2f> &measurements_old,
                           vector<cv::Point2f> &measurements_cur,vector<Eigen::Vector3d> &pointsCloud_old_3d)
{
//    if (measurements_cur.size() >= 8)
//    {
         
    //这里改一下 改成cv::Mat
    
    int ptCount = (int)measurements_old.size();
    cv::Mat p1(ptCount, 2, CV_32F);
    cv::Mat p2(ptCount, 2, CV_32F);
     
    // 把Keypoint转换为cv::Mat
    cv::Point2f pt;
    for (int i=0; i<ptCount; i++)
    {
         pt = measurements_cur[i];
         p1.at<float>(i, 0) = pt.x;
         p1.at<float>(i, 1) = pt.y;
      
         pt = measurements_old[i];
         p2.at<float>(i, 0) = pt.x;
         p2.at<float>(i, 1) = pt.y;
    }

    
        vector<uchar> status;
        //永远都用旧的匹配 但是不用做减法 可以让新的做减法
        cv::findFundamentalMat(p1, p2, cv::FM_RANSAC, 2.0, 0.99, status);
        //只留下status公共为1的
 
        reduceVector2(measurements_cur, status);
        reduceVector2(measurements_old, status );//如果前面回环检测没有问题，那么这里应该和上面一样减 要求出相对位姿
        reduceVector2(pointsCloud_old_3d, status);
        
        
//    }
}


void KeyFrame::rejectWithF_server_mapFuse2(vector<cv::Point2f> &measurements_old,
                           vector<cv::Point2f> &measurements_cur, vector<Eigen::Vector3d> &pointsCloud_old_3d, vector<int> &featureId_cur)
{
//    if (measurements_cur.size() >= 8)
//    {
         
    //这里改一下 改成cv::Mat
    
    int ptCount = (int)measurements_old.size();
    cv::Mat p1(ptCount, 2, CV_32F);
    cv::Mat p2(ptCount, 2, CV_32F);
     
    // 把Keypoint转换为cv::Mat
    cv::Point2f pt;
    for (int i=0; i<ptCount; i++)
    {
         pt = measurements_cur[i];
         p1.at<float>(i, 0) = pt.x;
         p1.at<float>(i, 1) = pt.y;
      
         pt = measurements_old[i];
         p2.at<float>(i, 0) = pt.x;
         p2.at<float>(i, 1) = pt.y;
    }

    
        vector<uchar> status;
        //永远都用旧的匹配 但是不用做减法 可以让新的做减法
        cv::findFundamentalMat(p1, p2, cv::FM_RANSAC, 2.0, 0.99, status);
        //只留下status公共为1的
 
        reduceVector2(measurements_cur, status);
        reduceVector2(measurements_old, status );//如果前面回环检测没有问题，那么这里应该和上面一样减 要求出相对位姿
        reduceVector2(pointsCloud_old_3d, status);
    reduceVector2(featureId_cur, status);
        
        
//    }
}

//老帧的3D 新帧的2D 这里需要老帧调用
bool KeyFrame::findConnectionWithOldFrame_server(const KeyFrame* cur_kf,
                                          const std::vector<cv::Point2f> &cur_pts, const std::vector<cv::Point2f> &old_pts,
                                          std::vector<cv::Point2f> &measurements_cur, std::vector<cv::Point2f> &measurements_cur_norm)
{
    searchByDes_server(measurements_cur, measurements_cur_norm, cur_kf->descriptors, cur_kf->keypoints,cur_kf->c);
    return true;
}
//vector<int> score_hanming;

//-------------------------------这里是老帧2D坐标，新帧3D坐标
void KeyFrame::rejectWithF_server_old(vector<cv::Point2f> &measurements_old,
                           vector<cv::Point2f> &measurements_old_norm,Client* c_old)
{
    if (measurements_old.size() >= 8)
    {
        measurements_old_norm.clear();

        for (unsigned int i = 0; i < measurements_old.size(); i++)
        {
            cv::Point2f norm_pt;
            norm_pt.x = (measurements_old[i].x -  c_old->PX_server)/ c_old->FOCUS_LENGTH_X_server;
            norm_pt.y = (measurements_old[i].y -  c_old->PY_server)/ c_old->FOCUS_LENGTH_Y_server;
            measurements_old_norm.push_back(norm_pt);
        }
//        cout<<c->FOCUS_LENGTH_Y_server<<" test Focus_y new-old"<<c_old->FOCUS_LENGTH_Y_server<<endl;

        vector<uchar> status;
        //永远都用旧的匹配 但是不用做减法 可以让新的做减法
        cv::findFundamentalMat(measurements_origin, measurements_old, cv::FM_RANSAC, 2.0, 0.99, status);
        //只留下status公共为1的
//        reduceVector3(point_clouds, status);//点云信息 暂时没改
        reduceVector3(point_clouds,point_clouds_origin, status);//算相对位姿
        reduceVector3(measurements,measurements_origin, status);
        reduceVector2(measurements_old, status );//如果前面回环检测没有问题，那么这里应该和上面一样减 要求出相对位姿
        reduceVector2(measurements_old_norm, status);
        reduceVector3(features_id,features_id_origin, status);//这个的数量和measurements是一样的

//        //保存2d-2d测试是不是匹配点误差大
//        std::ofstream outFile;
//        //打开文件
//        outFile.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/"+to_string(global_index)+".txt");
//
//        for(auto iter=measurements.begin(), iter_end=measurements.end(); iter!=iter_end; iter++)
//        {
//            //写入数据
//            outFile << (*iter).x<<","<<(*iter).y<<" ";
//        }
//        outFile<<"\n";
//        for(auto iter=measurements_old.begin(), iter_end=measurements_old.end(); iter!=iter_end; iter++)
//        {
//            //写入数据
//            outFile << (*iter).x<<","<<(*iter).y<<" ";
//        }
//        outFile<<"\n";
//        //关闭文件
//        outFile.close();



//        reduceVector2(score_hanming, status);

//        for(int i=0;i<score_hanming.size();i++){
//            cout<<"移除外点后 汉明距离："<<score_hanming[i]<<" ";
//        }
//        cout<<endl;
//        reduceVector3(win_point_z, win_point_z_origin, status);
//        reduceVector3(win_keyPoint_depth,win_keyPoint_depth_origin, status);
//        reduceVector3(pts_3d_server, pts_3d_server_origin, status);
        //告诉客户端 剔除外点
//        sendRejectWithF=true;
//        send_status=status;






    }
}

void KeyFrame::rejectWithF_server_old2(vector<cv::Point2f> &measurements_old,
                           vector<cv::Point2f> &measurements_old_norm,Client* c_old,std::vector<uchar> &status1)
{
    if (measurements_old.size() >= 8)
    {
        measurements_old_norm.clear();

        for (unsigned int i = 0; i < measurements_old.size(); i++)
        {
            cv::Point2f norm_pt;
            norm_pt.x = (measurements_old[i].x -  c_old->PX_server)/ c_old->FOCUS_LENGTH_X_server;
            norm_pt.y = (measurements_old[i].y -  c_old->PY_server)/ c_old->FOCUS_LENGTH_Y_server;
            measurements_old_norm.push_back(norm_pt);
        }
//        cout<<c->FOCUS_LENGTH_Y_server<<" test Focus_y new-old"<<c_old->FOCUS_LENGTH_Y_server<<endl;

        reduceVector3(point_clouds,point_clouds_origin, status1);//算相对位姿
        reduceVector3(measurements,measurements_origin, status1);
        reduceVector3(features_id,features_id_origin, status1);//这个的数量和measurements是一样的
        
        vector<uchar> status;
        //永远都用旧的匹配 但是不用做减法 可以让新的做减法
        cv::findFundamentalMat(measurements, measurements_old, cv::FM_RANSAC, 2.0, 0.99, status);
        //只留下status公共为1的
        reduceVector2(point_clouds, status);//算相对位姿
        reduceVector2(measurements,status);
        reduceVector2(measurements_old, status );//如果前面回环检测没有问题，那么这里应该和上面一样减 要求出相对位姿
        reduceVector2(measurements_old_norm, status);
        reduceVector2(features_id,status);//这个的数量和measurements是一样的
    }
}

void KeyFrame::searchByDes_server_old(std::vector<cv::Point2f> &measurements_old,
                           std::vector<cv::Point2f> &measurements_old_norm,
                           const std::vector<BRIEF::bitset> &descriptors_old,
                           const std::vector<cv::KeyPoint> &keypoints_old,Client* c_old)
{
//    score_hanming.clear();
//    printf("loop_match before cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
//    std::vector<int> dis_cur_old;
    
//    std::vector<uchar> status;
    for(int i = 0; i < window_descriptors.size(); i++)
    {
        int bestDist = 256;
        int bestIndex = -1;
//        int bestDist2 = 256;
        for(int j = 0; j < descriptors_old.size(); j++)
        {
            int dis = HammingDis(window_descriptors[i], descriptors_old[j]);
            if(dis < bestDist)
            {
//                bestDist2 = bestDist;
                bestDist = dis;
                bestIndex = j;
            }
//            else if(dis < bestDist2)
//            {
//                bestDist2 = dis;
//            }
        }
        //256
        if(bestDist <256)
        {
            measurements_old.push_back(keypoints_old[bestIndex].pt);

//            status.push_back(1);
        }
//        else{
//            status.push_back(0);
//        }
    }
    rejectWithF_server_old(measurements_old, measurements_old_norm,c_old);
//    printf("loop_match after cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
}

void KeyFrame::searchByDes_server_old2(std::vector<cv::Point2f> &measurements_old,
                           std::vector<cv::Point2f> &measurements_old_norm,
                           const std::vector<BRIEF::bitset> &descriptors_old,
                           const std::vector<cv::KeyPoint> &keypoints_old,Client* c_old)
{
//    score_hanming.clear();
//    printf("loop_match before cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
//    std::vector<int> dis_cur_old;
    std::vector<uchar> status;
    for(int i = 0; i < window_descriptors.size(); i++)
    {
        int bestDist = 256;
        int bestIndex = -1;
        int bestDist2 = 256;
        for(int j = 0; j < descriptors_old.size(); j++)
        {
            int dis = HammingDis(window_descriptors[i], descriptors_old[j]);
            if(dis < bestDist)
            {
                bestDist2 = bestDist;
                bestDist = dis;
                bestIndex = j;
            }else if(dis < bestDist2)
            {
                bestDist2 = dis;
            }
        }
        //256
        if(bestDist <=0.9* bestDist2)
        {
            measurements_old.push_back(keypoints_old[bestIndex].pt);
//            dis_cur_old.push_back(bestDist);
//            cout<<"测试真正匹配的点 bestDist:"<<bestDist<<endl;
//            score_hanming.push_back(bestDist);
            status.push_back(1);
        }else{
            status.push_back(0);
        }
    }
    rejectWithF_server_old2(measurements_old, measurements_old_norm,c_old,status);
//    printf("loop_match after cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
}
/**
//临时改一下，做实验用，上面那个是真正跑数据
void KeyFrame::rejectWithF_server_old(vector<cv::Point2f> &measurements_old,
                           vector<cv::Point2f> &measurements_old_norm,std::vector<int> dis_cur_old,Client* c_old)
{
    if (measurements_old.size() >= 8)
    {
        measurements_old_norm.clear();
        
        //实验用
        vector<int> keys_index_cur;
        keys_index_cur.clear();
        int start = keypoints.size() - measurements_origin.size();
        
        for (unsigned int i = 0; i < measurements_old.size(); i++)
        {
            cv::Point2f norm_pt;
            norm_pt.x = (measurements_old[i].x -  c_old->PX_server)/ c_old->FOCUS_LENGTH_X_server;
            norm_pt.y = (measurements_old[i].y -  c_old->PY_server)/ c_old->FOCUS_LENGTH_Y_server;
            measurements_old_norm.push_back(norm_pt);
            
            keys_index_cur.push_back(start+i);
        }
//        cout<<c->FOCUS_LENGTH_Y_server<<" test Focus_y new-old"<<c_old->FOCUS_LENGTH_Y_server<<endl;
        
        vector<uchar> status;
        //永远都用旧的匹配 但是不用做减法 可以让新的做减法
        cv::findFundamentalMat(measurements_origin, measurements_old, cv::FM_RANSAC, 2.0, 0.99, status);
        //只留下status公共为1的
//        reduceVector3(point_clouds, status);//点云信息 暂时没改
        reduceVector3(point_clouds,point_clouds_origin, status);//算相对位姿
        reduceVector3(measurements,measurements_origin, status);
        reduceVector2(measurements_old, status );//如果前面回环检测没有问题，那么这里应该和上面一样减 要求出相对位姿
        reduceVector2(measurements_old_norm, status);
        reduceVector3(features_id,features_id_origin, status);//这个的数量和measurements是一样的
        
        reduceVector2(dis_cur_old, status );
        reduceVector2(keys_index_cur, status );
        
//        //保存2d-2d测试是不是匹配点误差大
        std::ofstream outFile;
        //打开文件
        outFile.open("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/"+
                     to_string(global_index)+"&"+to_string(header)+".txt");

        for(auto iter=measurements.begin(), iter_end=measurements.end(); iter!=iter_end; iter++)
        {
            //写入数据
            outFile << (*iter).x<<","<<(*iter).y<<" ";
        }
        outFile<<"\n";
        for(auto iter=keys_index_cur.begin(), iter_end=keys_index_cur.end(); iter!=iter_end; iter++)
        {
            //写入数据
            outFile << (*iter)<<" ";
        }
        outFile<<"\n";
        for(auto iter=measurements_old.begin(), iter_end=measurements_old.end(); iter!=iter_end; iter++)
        {
            //写入数据
            outFile << (*iter).x<<","<<(*iter).y<<" ";
        }
        outFile<<"\n";
        for(auto iter=dis_cur_old.begin(), iter_end=dis_cur_old.end(); iter!=iter_end; iter++)
        {
            //写入数据
            outFile << (*iter)<<" ";
        }
        outFile<<"\n";
        for(int i=0,j=measurements_old.size();i<j;i++)
        {
            //写入数据
            outFile << i<<" ";
        }
        outFile<<"\n";
        //关闭文件
        outFile.close();
       
        
        
//        reduceVector2(score_hanming, status);
        
//        for(int i=0;i<score_hanming.size();i++){
//            cout<<"移除外点后 汉明距离："<<score_hanming[i]<<" ";
//        }
//        cout<<endl;
//        reduceVector3(win_point_z, win_point_z_origin, status);
//        reduceVector3(win_keyPoint_depth,win_keyPoint_depth_origin, status);
//        reduceVector3(pts_3d_server, pts_3d_server_origin, status);
        //告诉客户端 剔除外点
//        sendRejectWithF=true;
//        send_status=status;
        
        
        

        
        
    }
}

void KeyFrame::searchByDes_server_old(std::vector<cv::Point2f> &measurements_old,
                           std::vector<cv::Point2f> &measurements_old_norm,
                           const std::vector<BRIEF::bitset> &descriptors_old,
                           const std::vector<cv::KeyPoint> &keypoints_old,Client* c_old)
{
//    score_hanming.clear();
//    printf("loop_match before cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
    std::vector<int> dis_cur_old;
   
    
    std::vector<uchar> status;
    for(int i = 0; i < window_descriptors.size(); i++)
    {
        int bestDist = 256;
        int bestIndex = -1;
        for(int j = 0; j < descriptors_old.size(); j++)
        {
            int dis = HammingDis(window_descriptors[i], descriptors_old[j]);
            if(dis < bestDist)
            {
                bestDist = dis;
                bestIndex = j;
            }
        }
        //256
        if(bestDist < 256)
        {
            measurements_old.push_back(keypoints_old[bestIndex].pt);
            
           
            dis_cur_old.push_back(bestIndex);
//            cout<<"测试真正匹配的点 bestDist:"<<bestDist<<endl;
//            score_hanming.push_back(bestDist);
        }
    }
    rejectWithF_server_old(measurements_old, measurements_old_norm,dis_cur_old,c_old);
//    printf("loop_match after cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
}
 */

//这里需要新帧调用
bool KeyFrame::findConnectionWithOldFrame_server_old(const KeyFrame* old_kf, std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm)
{
    searchByDes_server_old(measurements_old, measurements_old_norm, old_kf->descriptors, old_kf->keypoints,old_kf->c);
    return true;
}

bool KeyFrame::findConnectionWithOldFrame_server_old2(const KeyFrame* old_kf, std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm)
{
    searchByDes_server_old2(measurements_old, measurements_old_norm, old_kf->descriptors, old_kf->keypoints,old_kf->c);
    return true;
}
//--------------------------------位姿更新-------------------------------
void KeyFrame::updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    unique_lock<mutex> lock(mMutexPose);
    T_w_i = _T_w_i;
    R_w_i = _R_w_i;
}

void KeyFrame::updateOriginPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    unique_lock<mutex> lock(mMutexPose);
    origin_T_w_i = _T_w_i;
    origin_R_w_i = _R_w_i;
}

void KeyFrame::updatePose_old(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    unique_lock<mutex> lock(mMutexPose);
    T_wOld_i = _T_w_i;
    R_wOld_i = _R_w_i;
}

void KeyFrame::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    unique_lock<mutex> lock(mMutexPose);
    _T_w_i = T_w_i;
    _R_w_i = R_w_i;
}

void KeyFrame::getOriginPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    unique_lock<mutex> lock(mMutexPose);
    _T_w_i = origin_T_w_i;
    _R_w_i = origin_R_w_i;
}

void KeyFrame::getPose_old(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    unique_lock<mutex> lock(mMutexPose);
    _T_w_i = T_wOld_i;
    _R_w_i = R_wOld_i;
}

void KeyFrame::update_w1Pose(const Eigen::Vector3d &_T_w1_i, const Eigen::Matrix3d &_R_w1_i)
{
    unique_lock<mutex> lock(mMutex_w1Pose);
    T_w1_i = _T_w1_i;
    R_w1_i = _R_w1_i;
}


void KeyFrame::get_w1Pose(Eigen::Vector3d &_T_w1_i, Eigen::Matrix3d &_R_w1_i)
{
    unique_lock<mutex> lock(mMutex_w1Pose);
    T_w1_i = _T_w1_i;
    R_w1_i = _R_w1_i;
}

void KeyFrame::addConnection(int index, KeyFrame* connected_kf)
{
    Vector3d connected_t, relative_t;
    Matrix3d connected_r;
    Quaterniond relative_q;
    connected_kf->getPose(connected_t, connected_r);
    
    relative_q = connected_r.transpose() * R_w_i;
    relative_t = connected_r.transpose() * (T_w_i - connected_t);
    double relative_yaw;
    relative_yaw = Utility::R2ypr(R_w_i).x() - Utility::R2ypr(connected_r).x();
    Eigen::Matrix<double, 8, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
    relative_yaw;
    connection_list.push_back(make_pair(index, connected_info));
}

void KeyFrame::addConnection(int index, KeyFrame* connected_kf, Vector3d relative_t, Quaterniond relative_q, double relative_yaw)
{
    Eigen::Matrix<double, 8, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
    relative_yaw;
    connection_list.push_back(make_pair(index, connected_info));
}

void KeyFrame::addLoopConnection(int index, KeyFrame* loop_kf)
{
    assert(index == loop_index);
    Vector3d connected_t, relative_t;
    Matrix3d connected_r;
    Quaterniond relative_q;
    loop_kf->getPose(connected_t, connected_r);
    
//    relative_q = connected_r.transpose() * R_w_i;
    relative_t = connected_r.transpose() * (T_w_i - connected_t);
    double relative_yaw;
    relative_yaw = Utility::R2ypr(R_w_i).x() - Utility::R2ypr(connected_r).x();
//    Eigen::Matrix<double, 8, 1> connected_info;
//    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
//    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
//    relative_yaw;
//    loop_info = connected_info;
    Eigen::Matrix<double, 4, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),relative_yaw;
    loop_info_better = connected_info;
}

//void KeyFrame::updateLoopConnection(Vector3d relative_t, Quaterniond relative_q, double relative_yaw)
//{
//    Eigen::Matrix<double, 8, 1> connected_info;
//    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
//    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
//    relative_yaw;
//    loop_info = connected_info;
//}

void KeyFrame::updateLoopConnection(Vector3d relative_t, double relative_yaw)
{
    Eigen::Matrix<double, 4, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),relative_yaw;
    loop_info_better = connected_info;
}

void KeyFrame::update_globalLoopConnection(Vector3d relative_t, Quaterniond relative_q, double relative_yaw)
{
    Eigen::Matrix<double, 8, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
    relative_yaw;
    global_loop_info = connected_info;
}

void KeyFrame::detect_globalLoop(int index)
{
    has_global_loop = true;
//    cout<<"global loop:"<<index<<endl;
    global_loop_index = index;
}

void KeyFrame::is_detected_globalLoop(int index)
{
    is_global_looped = true;
//    cout<<"is global looped:"<<index<<endl;
    is_global_looped_index = index;
}

void KeyFrame::detectLoop(int index)
{
    has_loop = true;
    loop_index = index;
}

void KeyFrame::removeLoop()
{
    has_loop = false;
    //update_loop_info = 0;
}

int KeyFrame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
    BRIEF::bitset xor_of_bitset = a ^ b;
    int dis = xor_of_bitset.count();
    return dis;
}

void  KeyFrame::setWin_keyPoint_des(){
    window_keypoints.clear();
    window_descriptors.clear();
    int winLen=measurements.size();//此处应该是win_keypoints的值 ，但是还没给到它
    int start=keypoints.size()-winLen;
    
//    cout<<"winLen="<<winLen<<" start="<<start<<endl;
    for(int i=0;i<winLen;i++){
        window_keypoints.push_back(keypoints[start + i]);
        window_descriptors.push_back(descriptors[start + i]);
    }
}

void KeyFrame::getPath(Eigen::Vector3d& path)
{
    path = T_w_i;
}


void KeyFrame::detectFusion(int old_index){
    has_fusion=true;
    fusion_index=old_index;
}


bool KeyFrame::isInImage(float x, float y){
    int width=c->getWidth();
    int height=c->getHeight();
    return x>=0 && x<=width && y>=0 && y<=height;
}

vector<int> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r)
{
   
    vector<int> vIndices;

    int len=measurements_origin.size();
    assert(len==features_id_origin.size());
    for(int i=0;i<len;i++){
        //像素坐标
        cv::Point2f measure_single=measurements_origin[i];
        float x_measure_origin=measure_single.x;
        float y_measure_origin=measure_single.y;
        
        const float distx = x_measure_origin-x;
        const float disty = y_measure_origin-y;

        if(fabs(distx)<r && fabs(disty)<r)
            vIndices.push_back(i);
    }
    
    return vIndices;
}

//这种坐标去畸变了
vector<int> KeyFrame::GetFeaturesInArea_1(const float &x, const float &y, const float &r)
{
   
    vector<int> vIndices;

    int len=keypoints.size();
    int des_len=descriptors.size();
    int min_len=len<des_len?len:des_len;
    
    //因为有时候数据并没有完全发送过来
    for(int i=0;i<min_len;i++){
        //像素坐标
        cv::Point2f measure_single=keypoints[i].pt;
        float x_measure_origin=measure_single.x;
        float y_measure_origin=measure_single.y;
        
        const float distx = x_measure_origin-x;
        const float disty = y_measure_origin-y;

        if(fabs(distx)<r && fabs(disty)<r)
            vIndices.push_back(i);
    }
    
    return vIndices;
}


void KeyFrame::AddConnection_weight(KeyFrame *pKF, const int &weight)
{
    {
        mMutexConnections.lock();
        
        int i=mvpOrderedConnectedKeyFrames.size();
        vector<KeyFrame*>::reverse_iterator iter=mvpOrderedConnectedKeyFrames.rbegin();
        for(auto iter_end=mvpOrderedConnectedKeyFrames.rend(); iter!=iter_end;iter++)
        {
            i--;
            if((*iter)==pKF){
                //找到了
                if(mvOrderedWeights[i]!=weight){
                    mvOrderedWeights[i]=weight;
                }else{
                    return;
                }
            }
            
        }
        if(iter == mvpOrderedConnectedKeyFrames.rend()){
            //说明没找到
            mvpOrderedConnectedKeyFrames.push_back(pKF);
            mvOrderedWeights.push_back(weight);
        }
        mMutexConnections.unlock();
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    //mvpOrderedConnectedKeyFrames这个基本有序 ，可以考虑其他排序算法
    
    mMutexConnections.lock();
    
    int conn_kf_len=mvpOrderedConnectedKeyFrames.size();
   
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(conn_kf_len);
    // 取出所有连接的关键帧，mConnectedKeyFrameWeights的类型为std::map<KeyFrame*,int>，而vPairs变量将共视的3D点数放在前面，利于排序
    for(int i=0;i<conn_kf_len;i++){
       vPairs.push_back(make_pair(mvOrderedWeights[i],mvpOrderedConnectedKeyFrames[i]));
    }

    // 按照权重进行排序
    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs; // keyframe
    list<int> lWs; // weight
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    // 权重从大到小
    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
    mMutexConnections.unlock();
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
//    cout<<"测试是不是因为这里不会释放锁而报错"<<endl;
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}


vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames_old(const int &N)
{
//    cout<<"测试是不是因为这里不会释放锁而报错"<<endl;
    vector<KeyFrame*> mvpOrderedConnectedKeyFrames_old;
    vector<KeyFrame*> mvpOrderedConnectedKeyFrames_all;
    mMutexConnections.lock();
    mvpOrderedConnectedKeyFrames_all=mvpOrderedConnectedKeyFrames;
    mMutexConnections.unlock();
    int i=0;
    for(KeyFrame* pkfi: mvpOrderedConnectedKeyFrames_all){
        if(i>=N){
            break;
        }
        if(pkfi->global_index<global_index){
            mvpOrderedConnectedKeyFrames_old.push_back(pkfi);
            i++;
        }
        
    }
    return mvpOrderedConnectedKeyFrames_old;

}


