//
//  prior_map_feature.cpp
//  VINS_ios
//
//  Created by 张剑华 on 2022/9/19.
//  Copyright © 2022 栗大人. All rights reserved.
//

#include "prior_map_feature.hpp"


//判断跟踪的特征点是否在图像边界
bool inBorder_pts(float x,float y)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(x);
    int img_y = cvRound(y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}


template <typename T>
void reduceVector(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


//先验地图特征提取

void PriorMapFeature::detectPriorMapFeatures(KeyFrame* old_kf,KeyFrame* cur_kf,KeyFrame* curLoopKf,double* loop_pose_priorMap)
{
    if(cur_kf->isProjection_findPriorFeature==old_kf->global_index){
        measurements_cur_coarse_pixel_new.clear();
        measurements_cur_coarse_new.clear();
        point_3d_old_new.clear();
        feature_id_cur_new.clear();
        header=0.0;
        return;
    }
    cout<<"跟踪线程中"<<curLoopKf->global_index<<" , "<<old_kf->global_index<<" , "<<cur_kf->global_index<<endl;
//    cout<<fixed << setprecision(15) <<curLoopKf->header<<" , "<<old_kf->header<<" , "<<cur_kf->header<<endl;
    cur_kf->isProjection_findPriorFeature=old_kf->global_index;
    int nmatches = 0;
    int HISTO_LENGTH=30;
    float th=20;
    int TH_HIGH=100;
    
    
//    到当前的回环帧的相对位姿
    Eigen::Vector3d t_w2_curLoop(loop_pose_priorMap[0],loop_pose_priorMap[1],loop_pose_priorMap[2]);
    Eigen::Quaterniond q_w2_curLoop(loop_pose_priorMap[6],loop_pose_priorMap[3],loop_pose_priorMap[4],loop_pose_priorMap[5]);
    Eigen::Matrix3d r_w2_curLoop=q_w2_curLoop.toRotationMatrix();
    Eigen::Matrix3d r_w_curLoop;
    Eigen::Vector3d t_w_curLoop;
    curLoopKf->getPose(t_w_curLoop,r_w_curLoop);
    Eigen::Matrix3d r_w_cur;
    Eigen::Vector3d t_w_cur;
    cur_kf->getPose(t_w_cur,r_w_cur);
    Eigen::Matrix3d r_curLoop_cur;
    Eigen::Vector3d t_curLoop_cur;
    r_curLoop_cur=r_w_curLoop.transpose()*r_w_cur;
    t_curLoop_cur=r_w_curLoop.transpose()*(t_w_cur-t_w_curLoop);
    Eigen::Matrix3d r_wOld_cur;
    Eigen::Vector3d t_wOld_cur;
    r_wOld_cur=r_w2_curLoop*r_curLoop_cur;
    t_wOld_cur=r_w2_curLoop*t_curLoop_cur+t_w2_curLoop;
//    转为世界坐标到相机坐标系
    Eigen::Matrix3d r_w_camOld=r_wOld_cur*ric_curClient;
    Eigen::Vector3d t_w_camOld=r_wOld_cur*tic_curClient+t_wOld_cur;
    Eigen::Matrix3d r_camOld_w=r_w_camOld.transpose();
    Eigen::Vector3d t_camOld_w=-r_camOld_w*t_w_camOld;
    
//    vector<vector<cv::Point2f>> measurements_cur_all;
//    vector<vector<Eigen::Vector3d>> point_clouds_all;
//    vector<vector<int>> feature_id_cur_all;
    
    std::vector<cv::Point2f> measurements_cur_coarse;//相机平面坐标
//    std::vector<cv::Point2f> measurements_cur_norm_coarse;//图像坐标
    std::vector<Eigen::Vector3d> point_3d_old;
//    std::vector<cv::Point2f> measurements_old;//像素坐标
    std::vector<int> feature_id_cur;
    std::vector<cv::Point2f> measurements_cur_coarse_pixel;//像素坐标
    
//    用于实验 看点对匹配是否正确
//    std::vector<cv::Point2f> measurements_cur_coarse_experiment;
//    std::vector<cv::Point2f> measurements_old_coarse_experiment;
//    std::vector<cv::KeyPoint> keypoints_cur_coarse_all=cur_kf->keypoints_distorted;
//    std::vector<cv::Point2f> measurements_cur_coarse_experiment_undistorted;
//    std::vector<cv::Point2f> measurements_old_coarse_experiment_undistorted;
//    std::vector<double> header_all;
    
    vector<cv::KeyPoint> keypoints_cur=cur_kf->keypoints;
    vector<cv::KeyPoint> keypoints_dis_cur=cur_kf->keypoints_distorted;
    vector<int > feature_id_origin_cur=cur_kf->features_id_origin;
    int nPoints_cur = feature_id_origin_cur.size();
    int point2D_len_cur=keypoints_cur.size()-nPoints_cur;
    
    int findPossiblePoint=0;
    for (auto &it_per_id : feature_local)
    {
        // Project 投影到当前帧
        Eigen::Vector3d pts_i = it_per_id.point;
        Eigen::Vector3d x3Dc = r_camOld_w*pts_i+t_camOld_w;

        const double xc = x3Dc[0];
        const double yc = x3Dc[1];
        const double invzc = 1.0/x3Dc[2];

        if(invzc<0)
            continue;

        double x=fx*xc*invzc+cx;
        double y=fy*yc*invzc+cy;
        if(!inBorder_pts((float)x, (float)y)){
            continue;
        }
        
        // Search in a window. Size depends on scale
        float radius = th;//CurrentFrame.mvScaleFactors[nLastOctave]

        const vector<int> vIndices = cur_kf->GetFeaturesInArea_1((float)x,(float)y, radius);
        if(vIndices.empty())
            continue;
        findPossiblePoint++;
        DVision::BRIEF::bitset point_old_des = it_per_id.des;
        int bestDist = 256;
        int bestIdx2 = -1;
        for(vector<int>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const int i2 = *vit;
            int dist = cur_kf->HammingDis(point_old_des, cur_kf->descriptors[i2]);
            if(dist<bestDist)
            {
                bestDist=dist;
                bestIdx2=i2;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            point_3d_old.push_back(pts_i);
//            measurements_cur_coarse.push_back(keypoints_cur[bestIdx2].pt);
            float keypoints_cur_x=keypoints_cur[bestIdx2].pt.x;
            float keypoints_cur_y=keypoints_cur[bestIdx2].pt.y;
            float keypoints_dis_cur_x=keypoints_dis_cur[bestIdx2].pt.x;
            float keypoints_dis_cur_y=keypoints_dis_cur[bestIdx2].pt.y;
            measurements_cur_coarse_pixel.push_back(cv::Point2f(keypoints_dis_cur_x,keypoints_dis_cur_y));
            keypoints_cur_x = (keypoints_cur_x -  cx)/ fx;
            keypoints_cur_y = (keypoints_cur_y -  cy)/ fy;
            cv::Point2f norm_pt(keypoints_cur_x,keypoints_cur_y);
            measurements_cur_coarse.push_back(norm_pt);
            
            if(bestIdx2<point2D_len_cur){
                feature_id_cur.push_back(-1);//后续加上去 未跟踪的点
            }else{
                int index=bestIdx2-point2D_len_cur;
                feature_id_cur.push_back(feature_id_origin_cur[index]);//已跟踪的点
            }
            nmatches++;
            
            
//            实验
//            measurements_old_coarse_experiment.push_back(it_per_id.measurements_coarse);
//            measurements_cur_coarse_experiment.push_back(keypoints_cur_coarse_all[bestIdx2].pt);
//
//            measurements_old_coarse_experiment_undistorted.push_back(it_per_id.measurements_coarse_undistorted);
//            measurements_cur_coarse_experiment_undistorted.push_back(keypoints_cur[bestIdx2].pt);
//            header_all.push_back(it_per_id.header);
        }
        
    }

//    RetriveData_localMapping retrive_data_localMapping;
    
//    vins.q_old_3d_mutex.lock();
//    double rt_double[7];
//    for(int i=0;i<3;i++){
//        retrive_data_localMapping.loop_pose_forSlideWindow[i]=t_wOld_cur[i];
//    }
//    Eigen::Quaterniond q(r_wOld_cur);
//    retrive_data_localMapping.loop_pose_forSlideWindow[3]=q.x();
//    retrive_data_localMapping.loop_pose_forSlideWindow[4]=q.y();
//    retrive_data_localMapping.loop_pose_forSlideWindow[5]=q.z();
//    retrive_data_localMapping.loop_pose_forSlideWindow[6]=q.w();
//    retrive_data_localMapping.header_cur=cur_kf->header;
//    retrive_data_localMapping.measurements_cur_norm=measurements_cur_coarse;
//    retrive_data_localMapping.point_3d_old=point_3d_old;
//    retrive_data_localMapping.features_ids_cur=feature_id_cur;
////    retrive_data_localMapping.loop_pose_forSlideWindow=rt_double;
//    vins.retrive_pose_data_localMapping.push(retrive_data_localMapping);
//    vins.q_old_3d_mutex.unlock();
    
    cout<<"投影区域找到的可能点对数量="<<findPossiblePoint<<endl;
    cout<<"测试每次最终能找到多少投影的点对="<<nmatches<<endl;
    
//    measurements_cur_coarse_new=measurements_cur_coarse;
//    measurements_cur_coarse_pixel_new=measurements_cur_coarse_pixel;
//    point_3d_old_new=point_3d_old;
//    feature_id_cur_new=feature_id_cur;
//    header=cur_kf->header;
    
    if(nmatches<12){
        feature_local.clear();//这里还需要考虑 在清除的时候 又有更新了怎么办，所以应该有个备份
        measurements_cur_coarse_new.clear();
        measurements_cur_coarse_pixel_new.clear();
        point_3d_old_new.clear();
        feature_id_cur_new.clear();
        header=0.0;
    }else{
        measurements_cur_coarse_new=measurements_cur_coarse;
        measurements_cur_coarse_pixel_new=measurements_cur_coarse_pixel;
        point_3d_old_new=point_3d_old;
        feature_id_cur_new=feature_id_cur;
        header=cur_kf->header;
    }
   
//    输出匹配的点对的 未去畸变的像素坐标
//    for(int i=0,j=measurements_old_coarse_experiment.size();i<j;i++){
//        printf("%.20lf\n",header_all[i]);
//        printf("%.1f,%.1f,%.1f,%.1f\n",measurements_old_coarse_experiment[i].x,measurements_old_coarse_experiment[i].y,measurements_cur_coarse_experiment[i].x,measurements_cur_coarse_experiment[i].y);
//    }
//    vector<uchar> status;
//    cv::findFundamentalMat(measurements_old_coarse_experiment_undistorted, measurements_cur_coarse_experiment_undistorted, cv::FM_RANSAC, 2.0, 0.99, status);
//    reduceVector(measurements_old_coarse_experiment, status);
//    reduceVector(measurements_cur_coarse_experiment, status);
//    cout<<"过滤误匹配后："<<endl;
//    for(int i=0,j=measurements_old_coarse_experiment.size();i<j;i++){
//        printf("%.1f,%.1f,%.1f,%.1f\n",measurements_old_coarse_experiment[i].x,measurements_old_coarse_experiment[i].y,measurements_cur_coarse_experiment[i].x,measurements_cur_coarse_experiment[i].y);
//    }
//    return nmatches;//这里返回的是 匹配的总数量 后续应该做一个判断，总数量够不够
}


void PriorMapFeature::setLoopKf(double (&_loop_pose_forFeatureTracker)[7]){
    for(int i=0;i<7;i++){
        loop_pose[i]=_loop_pose_forFeatureTracker[i];
    }
    
}
void PriorMapFeature::setParameter(){
    tic_curClient =Eigen::Vector3d( TIC_X,TIC_Y,TIC_Z);
    ric_curClient = Utility::ypr2R(Eigen::Vector3d(RIC_y,RIC_p,RIC_r));
    
    
    fx=FOCUS_LENGTH_X;
    fy=FOCUS_LENGTH_Y;
    cx=PX;
    cy=PY;
}

PriorMapFeature::PriorMapFeature(){
    isUpdate=false;
}
