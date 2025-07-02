//
//  feature_tracker.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "feature_tracker.hpp"

int FeatureTracker::n_id = 0;
int FeatureTracker::n_priorMap_id = 0;

FeatureTracker::FeatureTracker()
:mask{ROW, COL, CV_8UC1},update_finished{false},img_cnt{0},current_time{-1.0},use_pnp{false}
{
    start_playback_dataEuroc=false;
    start_playback_dataKitti0930=false;
    start_playback_mvsec=false;
    
    isUpdate=false;
    header_forwKf=0.0;
    printf("init ok\n");
}
/*********************************************************tools function for feature tracker start*****************************************************/
//判断跟踪的特征点是否在图像边界
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

bool inBorder_car(cv::Point2f pt)
{
   
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
//    return (img_x>0 && img_x<=155 && img_y>0 && img_y<=36) || (img_x>=155 && img_x<= 209 && img_y>0 && img_y<= 84) || (img_x>= 209 && img_x<=409 && img_y>0 && img_y<=105) || (img_x>= 409 && img_x<=485 && img_y>0 && img_y<= 96) || (img_x>= 485 && img_x<=527 && img_y>0 && img_y<=89) || (img_x>= 527 && img_x<=586 && img_y>0 && img_y<=79) || (img_x>= 586 && img_x<=628 && img_y>0 && img_y<=51) || (img_x>= 628 && img_x<=635 && img_y>0 && img_y<=36)  || (img_x>= 635 && img_x<=682 && img_y>0 && img_y<=27);
    
    //9.683900
//    return (img_x>=155 && img_x<= 586 && img_y>0 && img_y<= 105) || (img_x>= 586 && img_x<=682 && img_y>0 && img_y<=51) ;
    
    //9.28 9.285812 3.82 这个是continue写错位置了
    
    //6.304897
    return (img_x>0 && img_x<= 682 && img_y>0 && img_y<= 37) || (img_x>= 155 && img_x<=586 && img_y>37 && img_y<=105) || (img_x>= 586 && img_x<=628 && img_y>0 && img_y<=51);
  
    
    //12.168084
//    return (img_x>0 && img_x<= 682 && img_y>0 && img_y<= 37) || (img_x>= 155 && img_x<=586 && img_y>37 && img_y<=105);

}

//去除无法跟踪的特征点
template <typename T>
void reduceVector(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

//添将新检测到的特征点n_pts
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);//像素坐标系的点
        ids.push_back(-1);
        track_cnt.push_back(1);
        max_min_pts tmp;
        tmp.min = p;
        tmp.max = p;
        parallax_cnt.push_back(tmp);
    }
}

//添加新跟踪到的地图点
void FeatureTracker::addPoints3()
{
    int i=0;
    for (auto &p : measurements_cur_coarse_pixel_new)
    {
        forw_pMap_pts.push_back(p);//像素坐标系的点
        priorMap_ids.push_back(feature_id_cur_new[i]);
        priorMap_track_cnt.push_back(1);
        i++;
    }
}

//添将新检测到的特征点n_pts
void FeatureTracker::addPoints2()
{
    for (auto &p : n_pts)
    {
        //去掉车头的像素
//        if((p.x>132 && p.x<683 && p.y>0 && p.y<5) || (p.x>211 && p.x<573 && p.y>0 && p.y<98)){
//            continue;
//        }
        if(inBorder_car(p)){
            continue;
        }
        
        
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
        max_min_pts tmp;
        tmp.min = p;
        tmp.max = p;
        parallax_cnt.push_back(tmp);
    }
}

//对跟踪点进行排序并去除密集点
void FeatureTracker::setMask()
{

    mask.setTo(255);
    // prefer to keep features that are tracked for long time
    
    vector<pair<pair<int, max_min_pts>, pair<cv::Point2f, int>>> cnt_pts_id;
    
    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(make_pair(track_cnt[i], parallax_cnt[i]), make_pair(forw_pts[i], ids[i])));
    
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<pair<int, max_min_pts>, pair<cv::Point2f, int>> &a, const pair<pair<int, max_min_pts>, pair<cv::Point2f, int>> &b)
         {
             return a.first.first > b.first.first;
         });
    
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();
    parallax_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
            //if(true)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first.first);
            parallax_cnt.push_back(it.first.second);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
    //for (auto &it: pre_pts)
    //{
    //    cv::circle(mask, it, MIN_DIST, 0, -1);
    //}
}

//利用F矩阵剔除外点
void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        vector<uchar> status;
        if(isUndistorted){
            vector<cv::Point2f> un_cur_pts(pre_pts.size()), un_forw_pts(forw_pts.size());
            for (unsigned int i = 0; i < pre_pts.size(); i++)
            {
                Eigen::Vector3d tmp_p;
                m_camera->liftProjective(Eigen::Vector2d(pre_pts[i].x, pre_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

                m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
            }
            cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
            reduceVector(cur_un_pts, status);
        }
        else{
            cv::findFundamentalMat(pre_pts, forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        }
        reduceVector(pre_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        reduceVector(parallax_cnt, status);
        
    }
}

//利用F矩阵剔除外点
void FeatureTracker::rejectWithF_priorMap()
{
    if (pre_pMap_pts.size() >= 8)
    {
        vector<uchar> status;
        if(isUndistorted){
            vector<cv::Point2f> un_cur_pts(pre_pMap_pts.size()), un_forw_pts(forw_pMap_pts.size());
            for (unsigned int i = 0; i < pre_pMap_pts.size(); i++)
            {
                Eigen::Vector3d tmp_p;
                m_camera->liftProjective(Eigen::Vector2d(pre_pts[i].x, pre_pMap_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

                m_camera->liftProjective(Eigen::Vector2d(forw_pMap_pts[i].x, forw_pMap_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
            }
            cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
//            reduceVector(cur_un_pts, status);TODO 到时候补上
        }
        else{
            cv::findFundamentalMat(pre_pts, forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        }
        reduceVector(pre_pMap_pts, status);
        reduceVector(cur_pMap_pts, status);
        reduceVector(forw_pMap_pts, status);
        reduceVector(priorMap_ids, status);
        reduceVector(priorMap_track_cnt, status);
    }
}

/*********************************************************tools function for feature tracker ending*****************************************************/
//寻找第K帧中与滑窗中相同的路标点，计算该路标点在归一化相机系中的坐标
bool FeatureTracker::solveVinsPnP(double header, Eigen::Vector3d &P, Eigen::Matrix3d &R, bool vins_normal)
{
    if(!vins_normal)
        return false;
    /*
     if(solved_features.size() < 2)
     {
     printf("pnp not enough features\n");
     return false;
     }
     */
    vector<IMG_MSG_LOCAL> feature_msg;
    int i = 0;
    for (auto &it : solved_features)
    {
        while(ids[i] < it.id)
        {
            i++;
        }
        if(ids[i] == it.id)
        {
            IMG_MSG_LOCAL tmp;
            tmp = it;
            if(isUndistorted){
                vector<cv::Point2f>  un_forw_pts(forw_pts.size());
                for (unsigned int j = 0; j < forw_pts.size(); j++)
                {
                    Eigen::Vector3d tmp_p;
                    m_camera->liftProjective(Eigen::Vector2d(forw_pts[j].x, forw_pts[j].y), tmp_p);
//                    tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                    tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                    
                    tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                    tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                    un_forw_pts[j] = cv::Point2f(tmp_p.x(), tmp_p.y());
                }
//                tmp.observation=Eigen::Vector2d(un_forw_pts[i].x ,un_forw_pts[i].y);
                tmp.observation = (Eigen::Vector2d((un_forw_pts[i].x - PX)/FOCUS_LENGTH_X, (un_forw_pts[i].y - PY)/FOCUS_LENGTH_Y));
                
            }
            else{
                tmp.observation = (Eigen::Vector2d((forw_pts[i].x - PX)/FOCUS_LENGTH_X, (forw_pts[i].y - PY)/FOCUS_LENGTH_Y));
            }
            feature_msg.push_back(tmp);
        }
    }
    /*
     if(feature_msg.size() < 2 )
     {
     printf("pnp Not enough solved feature!\n");
     return false;
     }
     */
    vins_pnp.setInit(solved_vins);
//    printf("pnp imu header: ");
    for(auto &it : imu_msgs)
    {
        double t = it.header;
        if (current_time < 0)
            current_time = t;
        double dt = (t - current_time);
        if(start_playback_dataEuroc){//以纳秒为单位
            dt*=1.0e-9;
        }
        if(start_playback_dataKitti0930){//以微秒为单位
            dt*=1.0e-9;
        }
        
        if(start_playback_mvsec){//以微秒为单位
            dt*=1.0e-6;
        }
        current_time = t;
//        printf("dt=%lf \n ",dt);
        vins_pnp.processIMU(dt, it.acc, it.gyr);
    }
//    printf("image %lf\n", header);
    vins_pnp.processImage(feature_msg, header, use_pnp);
    
    P = vins_pnp.Ps[PNP_SIZE - 1];
    R = vins_pnp.Rs[PNP_SIZE - 1];
    Eigen::Vector3d R_ypr = Utility::R2ypr(R);
    return true;
}

//对图像使用光流法进行特征点跟踪
void FeatureTracker::readImage(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, vector<cv::Point2f> &good_pts, vector<double> &track_len, double header, Eigen::Vector3d &P, Eigen::Matrix3d &R, bool vins_normal)
{
    
    result = _img;
    if(forw_img.empty())
        pre_img = cur_img = forw_img = _img;//prev_img： 上一次发布数据时对应的图像
    else
        forw_img = _img;

    
    
    forw_pts.clear();
    //track
    {
        if(cur_pts.size()>0)
        {
            vector<uchar> status;
            vector<float> err;
            
            //TS(time_track);
//            对前一帧的特征点cur_pts进行金字塔光流跟踪，得到forw_pts
//         status标记了cur_pts中各个特征点的跟踪状态，根据status将跟踪失败的特征点从prev_pts、cur_pts和forw_pts中剔除，而且在记录特征点id的ids，和记录特征点被跟踪次数的track_cnt中，也要把这些跟踪失败的特征点对应位置的记录删除
//            被status标记为跟踪正常的特征点，在当前帧图像中的位置可能已经处于图像边界外了，这些特征点也应该被删除，删除操作同上。
            calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
            //TE(time_track);
            //根据opencv判断的跟踪质量及是否在边界，剔除部分outlier
            //将位于图像边界外的点标记为0
            for (int i = 0; i < int(forw_pts.size()); i++)
                if (status[i] && !inBorder(forw_pts[i]))
                    status[i] = 0;
            // status为1，则保留容器对应索引处的值
            reduceVector(pre_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(forw_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(parallax_cnt, status);
            if(isUndistorted){
                reduceVector(cur_un_pts, status);
            }
            
            //reject outliers
            if (forw_pts.size() >= 8)
            {
               
                vector<uchar> status;
                if(isUndistorted){
                    vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
                    for (unsigned int i = 0; i < cur_pts.size(); i++)
                    {
                        Eigen::Vector3d tmp_p;
                        m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
//                        tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                        tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                        
                        tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

                        m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
//                        tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                        tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                        
                        tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
                    }
                    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
                    reduceVector(cur_un_pts, status);
                }
                else{
                    cv::findFundamentalMat(cur_pts, forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
                }
                
                reduceVector(cur_pts, status);
                reduceVector(pre_pts, status);
                reduceVector(forw_pts, status);
                reduceVector(ids, status);
                reduceVector(track_cnt, status);
                reduceVector(parallax_cnt, status);
                
            }
            
            solveVinsPnP(header, P, R, vins_normal);//这里应该也要去畸变
            
            if(img_cnt!=0)
            {
                for (int i = 0; i< forw_pts.size(); i++)
                {
                    if(isUndistorted){
                        Eigen::Vector3d tmp_p;
                        cv::Point2f un_forw_pts;
                        m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
//                        tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                        tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                        
                        tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_forw_pts=cv::Point2f(tmp_p.x(),tmp_p.y());
                        //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                        good_pts.push_back(un_forw_pts);
                        if(un_forw_pts.x < parallax_cnt[i].min.x || un_forw_pts.y < parallax_cnt[i].min.y)
                        {
                            parallax_cnt[i].min = un_forw_pts;
                        }
                        else if(un_forw_pts.x > parallax_cnt[i].max.x || un_forw_pts.y > parallax_cnt[i].max.y)
                        {
                            parallax_cnt[i].max = un_forw_pts;
                        }
                    }else{
                        //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                        good_pts.push_back(forw_pts[i]);
                        if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
                        {
                            parallax_cnt[i].min = forw_pts[i];
                        }
                        else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
                        {
                            parallax_cnt[i].max = forw_pts[i];
                        }
                    }
                    
                    double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                    track_len.push_back(std::min(1.0, 1.0 * parallax/30));
                }
            }
        }
    }
    //detect
    {
        
        if(img_cnt==0)
        {
//            如果需要发布当前帧的数据,通过F矩阵去除outliers。剩下的特征点track_cnt都加1
            rejectWithF();
            for (int i = 0; i< forw_pts.size(); i++)
            {
                if(isUndistorted){
                    Eigen::Vector3d tmp_p;
                    cv::Point2f un_forw_pts;
                    m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
//                    tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                    tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                    
                    tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                    tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                    un_forw_pts=cv::Point2f(tmp_p.x(),tmp_p.y());
                    //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                    good_pts.push_back(un_forw_pts);
                    if(un_forw_pts.x < parallax_cnt[i].min.x || un_forw_pts.y < parallax_cnt[i].min.y)
                    {
                        parallax_cnt[i].min = un_forw_pts;
                    }
                    else if(un_forw_pts.x > parallax_cnt[i].max.x || un_forw_pts.y > parallax_cnt[i].max.y)
                    {
                        parallax_cnt[i].max = un_forw_pts;
                    }
                }else{
                    good_pts.push_back(forw_pts[i]);
                    if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
                    {
                        parallax_cnt[i].min = forw_pts[i];
                    }
                    else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
                    {
                        parallax_cnt[i].max = forw_pts[i];
                    }
                }
                double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                track_len.push_back(std::min(1.0, 1.0 * parallax/50));
            }
            //光流追踪成功,特征点被成功跟踪的次数就加1,数值代表被追踪的次数，数值越大，说明被追踪的就越久
            for (auto &n : track_cnt)
                n++;
//            通过设置一个mask，使跟踪的特征点在整幅图像中能够均匀分布，防止特征点扎堆
//            对光流跟踪到的特征点forw_pts，按照被跟踪到的次数降序排列，然后按照降序遍历这些特征点。每选中一个特征点，在mask中将该点周围半径为MIN_DIST的区域设置为0，后面不再选取该区域内的特征点。这样会删去一些特征点，使得特征点分布得更加均匀，同时尽可能地保留被跟踪次数更多的特征点
//            保证相邻的特征点之间要相隔30个像素
            setMask();
            //计算是否需要提取新的特征点
            int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
            
            if(n_max_cnt>0)
            {
                n_pts.clear();
//                TS(time_goodfeature);
                //goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.10, MIN_DIST, mask, 3, false, 0.04);
//                只有需要发布数据时，才会检测新的特征点，否则只跟踪，不检测新的特征点
//                在mask中不为0的区域检测新的特征点，将特征点数量补充至指定数量 id初始化为-1，track_cnt初始化为1
//                在这里得到跟踪的点
                goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.01, MIN_DIST, mask);
                
//                TE(time_goodfeature);
            }
            else
            {
                n_pts.clear();
            }
//            添将新检测到的特征点n_pts到forw_pts中，id初始化-1,track_cnt初始化为1.
            addPoints();//补齐足够的特征点
            //printf("features num after detect: %d\n",static_cast<int>(forw_pts.size()));
            pre_img = forw_img;
            pre_pts = forw_pts;
            if(isUndistorted){
                prev_un_pts=cur_un_pts;
            }

            //draw
            for (int i = 0; i < n_pts.size(); i++)
            {
                good_pts.push_back(n_pts[i]);
                track_len.push_back(0);
            }
            //result = mask;
            
        }
        cur_img = forw_img;
        cur_pts = forw_pts;
    }
    
   //像素坐标系的点 去畸变,这里要加个判断 手机这种不需要去畸变
    if(isUndistorted){
        undistortedPoints();
    }
    
//把像素坐标系的点 变成图像坐标系的点
    if(img_cnt == 0)
    {
        //update id and msg
        image_msg.clear();
        distorted_image_msg.clear();
        int num_new = 0;
        
        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            completed |= updateID(i);
            if (!completed)
                break;
        }
        if(isUndistorted){
            for(int i = 0; i<ids.size(); i++)
            {
                double x = cur_un_pts[i].x;
                double y = cur_un_pts[i].y;
                 
                double z = 1.0;
                image_msg[(ids[i])] = (Eigen::Vector3d(x, y, z));//添加特征点
                
//                Eigen::Vector3d tmp_p;
//                tmp_p.x() = FOCAL_LENGTH * x / z + COL / 2.0;
//                tmp_p.y() = FOCAL_LENGTH * y / z + ROW / 2.0;
//                tmp_p.x() = (tmp_p.x() - PX)/FOCUS_LENGTH_X;
//                tmp_p.y() =(tmp_p.y() - PY)/FOCUS_LENGTH_Y;
//                tmp_p.z()=1;
//                image_msg[(ids[i])] =tmp_p;//添加特征点
                
                
              
                distorted_image_msg[(ids[i])] = (Eigen::Vector3d(cur_pts[i].x, cur_pts[i].y, 1.0));//添加特征点
                
            }
        }else{
            for(int i = 0; i<ids.size(); i++)
            {
                double x = (cur_pts[i].x - PX)/FOCUS_LENGTH_X;
                double y = (cur_pts[i].y - PY)/FOCUS_LENGTH_Y;
                double z = 1.0;
                image_msg[(ids[i])] = (Eigen::Vector3d(x, y, z));//添加特征点
                
    //            cout<<"x="<<x<<" y="<<y<<endl;
            }
        }
    }
    
   

    //finished and tell solver the data is ok
    update_finished = true;
}

//对图像使用光流法进行特征点跟踪
void FeatureTracker::readImage3(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, vector<cv::Point2f> &good_pts, vector<double> &track_len, double header, Eigen::Vector3d &P, Eigen::Matrix3d &R, bool vins_normal)
{
    result = _img;
    if(forw_img.empty())
        pre_img = cur_img = forw_img = _img;//prev_img： 上一次发布数据时对应的图像
    else
        forw_img = _img;

    
    forw_pts.clear();
    //track
    {
        if(cur_pts.size()>0)
        {
            vector<uchar> status;
            vector<float> err;
            
            //TS(time_track);
//            对前一帧的特征点cur_pts进行金字塔光流跟踪，得到forw_pts
//         status标记了cur_pts中各个特征点的跟踪状态，根据status将跟踪失败的特征点从prev_pts、cur_pts和forw_pts中剔除，而且在记录特征点id的ids，和记录特征点被跟踪次数的track_cnt中，也要把这些跟踪失败的特征点对应位置的记录删除
//            被status标记为跟踪正常的特征点，在当前帧图像中的位置可能已经处于图像边界外了，这些特征点也应该被删除，删除操作同上。
            calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
            //TE(time_track);
            //根据opencv判断的跟踪质量及是否在边界，剔除部分outlier
            //将位于图像边界外的点标记为0
            for (int i = 0; i < int(forw_pts.size()); i++)
                if (status[i] && !inBorder(forw_pts[i]))
                    status[i] = 0;
            // status为1，则保留容器对应索引处的值
            reduceVector(pre_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(forw_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(parallax_cnt, status);
            if(isUndistorted){
                reduceVector(cur_un_pts, status);
            }
            
            //reject outliers
            if (forw_pts.size() >= 8)
            {
               
                vector<uchar> status;
                if(isUndistorted){
                    vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
                    for (unsigned int i = 0; i < cur_pts.size(); i++)
                    {
                        Eigen::Vector3d tmp_p;
                        m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
//                        tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                        tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                        
                        tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

                        m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
//                        tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                        tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                        
                        tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
                    }
                    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
                    reduceVector(cur_un_pts, status);
                }
                else{
                    cv::findFundamentalMat(cur_pts, forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
                }
                
                reduceVector(cur_pts, status);
                reduceVector(pre_pts, status);
                reduceVector(forw_pts, status);
                reduceVector(ids, status);
                reduceVector(track_cnt, status);
                reduceVector(parallax_cnt, status);
                
            }
            
            solveVinsPnP(header, P, R, vins_normal);//这里应该也要去畸变
            
            if(img_cnt!=0)
            {
                for (int i = 0; i< forw_pts.size(); i++)
                {
                    if(isUndistorted){
                        Eigen::Vector3d tmp_p;
                        cv::Point2f un_forw_pts;
                        m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
//                        tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                        tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                        
                        tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_forw_pts=cv::Point2f(tmp_p.x(),tmp_p.y());
                        //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                        good_pts.push_back(un_forw_pts);
                        if(un_forw_pts.x < parallax_cnt[i].min.x || un_forw_pts.y < parallax_cnt[i].min.y)
                        {
                            parallax_cnt[i].min = un_forw_pts;
                        }
                        else if(un_forw_pts.x > parallax_cnt[i].max.x || un_forw_pts.y > parallax_cnt[i].max.y)
                        {
                            parallax_cnt[i].max = un_forw_pts;
                        }
                    }else{
                        //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                        good_pts.push_back(forw_pts[i]);
                        if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
                        {
                            parallax_cnt[i].min = forw_pts[i];
                        }
                        else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
                        {
                            parallax_cnt[i].max = forw_pts[i];
                        }
                    }
                    
                    double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                    track_len.push_back(std::min(1.0, 1.0 * parallax/30));
                }
            }
        }
    }
    //detect
    {
        
        if(img_cnt==0)
        {
//            如果需要发布当前帧的数据,通过F矩阵去除outliers。剩下的特征点track_cnt都加1
            rejectWithF();//pre_pts在这里被用
            for (int i = 0; i< forw_pts.size(); i++)
            {
                if(isUndistorted){
                    Eigen::Vector3d tmp_p;
                    cv::Point2f un_forw_pts;
                    m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
//                    tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                    tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                    
                    tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                    tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                    un_forw_pts=cv::Point2f(tmp_p.x(),tmp_p.y());
                    //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                    good_pts.push_back(un_forw_pts);
                    if(un_forw_pts.x < parallax_cnt[i].min.x || un_forw_pts.y < parallax_cnt[i].min.y)
                    {
                        parallax_cnt[i].min = un_forw_pts;
                    }
                    else if(un_forw_pts.x > parallax_cnt[i].max.x || un_forw_pts.y > parallax_cnt[i].max.y)
                    {
                        parallax_cnt[i].max = un_forw_pts;
                    }
                }else{
                    good_pts.push_back(forw_pts[i]);
                    if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
                    {
                        parallax_cnt[i].min = forw_pts[i];
                    }
                    else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
                    {
                        parallax_cnt[i].max = forw_pts[i];
                    }
                }
                double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                track_len.push_back(std::min(1.0, 1.0 * parallax/50));
            }
            //光流追踪成功,特征点被成功跟踪的次数就加1,数值代表被追踪的次数，数值越大，说明被追踪的就越久
            for (auto &n : track_cnt)
                n++;
//            通过设置一个mask，使跟踪的特征点在整幅图像中能够均匀分布，防止特征点扎堆
//            对光流跟踪到的特征点forw_pts，按照被跟踪到的次数降序排列，然后按照降序遍历这些特征点。每选中一个特征点，在mask中将该点周围半径为MIN_DIST的区域设置为0，后面不再选取该区域内的特征点。这样会删去一些特征点，使得特征点分布得更加均匀，同时尽可能地保留被跟踪次数更多的特征点
//            保证相邻的特征点之间要相隔30个像素
            setMask();
            //计算是否需要提取新的特征点
            int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
            
            if(n_max_cnt>0)
            {
                n_pts.clear();
//                TS(time_goodfeature);
                //goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.10, MIN_DIST, mask, 3, false, 0.04);
//                只有需要发布数据时，才会检测新的特征点，否则只跟踪，不检测新的特征点
//                在mask中不为0的区域检测新的特征点，将特征点数量补充至指定数量 id初始化为-1，track_cnt初始化为1
//                在这里得到跟踪的点
                goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.01, MIN_DIST, mask);
                
//                TE(time_goodfeature);
            }
            else
            {
                n_pts.clear();
            }
//            添将新检测到的特征点n_pts到forw_pts中，id初始化-1,track_cnt初始化为1.
            addPoints();//补齐足够的特征点
            //printf("features num after detect: %d\n",static_cast<int>(forw_pts.size()));
            pre_img = forw_img;
            pre_pts = forw_pts;
            if(isUndistorted){
                prev_un_pts=cur_un_pts;
            }

            //draw
            for (int i = 0; i < n_pts.size(); i++)
            {
                good_pts.push_back(n_pts[i]);
                track_len.push_back(0);
            }
            //result = mask;
            
        }
        cur_img = forw_img;
        cur_pts = forw_pts;
    }
    
    //不管怎样，都要投影到下一帧（这个是新增的）,然后进行匹配，相当于追踪（追踪的是之前投影上的）
    
//    追踪
    forw_pMap_pts.clear();
    /**
    {
        measurements_cur_coarse_new.clear();
//        point_3d_old_new.clear();
//        feature_id_cur_new.clear();
        measurements_cur_coarse_pixel_new.clear();
        header_forwKf=0;
        
        if(cur_pMap_pts.size()>0)
        {
            vector<uchar> status;
            vector<float> err;
            
            //TS(time_track);
            calcOpticalFlowPyrLK(cur_img, forw_img, cur_pMap_pts, forw_pMap_pts, status, err, cv::Size(21, 21), 3);
            //TE(time_track);
            //根据opencv判断的跟踪质量及是否在边界，剔除部分outlier
            //将位于图像边界外的点标记为0
            for (int i = 0; i < int(forw_pMap_pts.size()); i++)
                if (status[i] && !inBorder(forw_pMap_pts[i]))
                    status[i] = 0;
            // status为1，则保留容器对应索引处的值
            reduceVector(pre_pMap_pts, status);
            reduceVector(cur_pMap_pts, status);
            reduceVector(forw_pMap_pts, status);
            reduceVector(priorMap_ids, status);
            reduceVector(priorMap_track_cnt, status);
            reduceVector(point_3d_old_new, status);
            reduceVector(feature_id_cur_new, status);
            if (forw_pts.size() >= 8)
            {
                vector<uchar> status;
                if(isUndistorted){
                    vector<cv::Point2f> un_cur_pts(cur_pMap_pts.size()), un_forw_pts(forw_pMap_pts.size());
                    for (unsigned int i = 0; i < forw_pMap_pts.size(); i++)
                    {
                        Eigen::Vector3d tmp_p;
                        m_camera->liftProjective(Eigen::Vector2d(forw_pMap_pts[i].x, forw_pMap_pts[i].y), tmp_p);
                        tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

                        m_camera->liftProjective(Eigen::Vector2d(cur_pMap_pts[i].x, cur_pMap_pts[i].y), tmp_p);
                        tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
                    }
                    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
//                    reduceVector(cur_un_pts, status);
                }else{
                    cv::findFundamentalMat(cur_pMap_pts, forw_pMap_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
                }
                reduceVector(cur_pMap_pts, status);
                reduceVector(pre_pMap_pts, status);
                reduceVector(forw_pMap_pts, status);
                reduceVector(priorMap_ids, status);
                reduceVector(priorMap_track_cnt, status);
                reduceVector(point_3d_old_new, status);
                reduceVector(feature_id_cur_new, status);
            }
            measurements_cur_coarse_pixel_new=forw_pMap_pts;//没有去畸变的
            for (unsigned int i = 0; i < forw_pMap_pts.size(); i++)
            {
                Eigen::Vector3d tmp_p;
                m_camera->liftProjective(Eigen::Vector2d(forw_pMap_pts[i].x, forw_pMap_pts[i].y), tmp_p);
                cv::Point2f pt=cv::Point2f(tmp_p.x()/tmp_p.z(),tmp_p.y()/tmp_p.z());
                measurements_cur_coarse_new.push_back(pt);
            }
//            header_forwKf=header;
        }
    }
     */
//    如果追踪的点不够的话，新增检测的投影点
    {
        if(img_cnt==0)
        {
            measurements_cur_coarse_new.clear();
            point_3d_old_new.clear();
            feature_id_cur_new.clear();
            header_forwKf=0.0;
            measurements_cur_coarse_pixel_new.clear();
            //计算是否需要提取新的特征点
//            int n_max_cnt = MAX_CNT_priorMap - static_cast<int>(forw_pMap_pts.size());
            int n_max_cnt =1;
            if(n_max_cnt>0)
            {
                if(isUpdate){
                    isUpdate=false;
                   
//                    TS(time_goodfeature);
                    priorMapFeature->detectPriorMapFeatures(old_kf_priorMap.front(), cur_kf_priorMap, curLoopKf_priorMap.front(),loop_pose_priorMap.front());
                    measurements_cur_coarse_new=priorMapFeature->measurements_cur_coarse_new;
                    point_3d_old_new=priorMapFeature->point_3d_old_new;
                    feature_id_cur_new=priorMapFeature->feature_id_cur_new;
                    header_forwKf=priorMapFeature->header;
                    measurements_cur_coarse_pixel_new=priorMapFeature->measurements_cur_coarse_pixel_new;
//                    TE(time_goodfeature);
                }else{
                    if(n_max_cnt<MAX_CNT_priorMap && priorMapFeature->feature_local.size()!=0){
//                        TS(time_goodfeature);
                        priorMapFeature->detectPriorMapFeatures(old_kf_priorMap.front(), cur_kf_priorMap, curLoopKf_priorMap.front(), loop_pose_priorMap.front());
                        measurements_cur_coarse_new=priorMapFeature->measurements_cur_coarse_new;
                        point_3d_old_new=priorMapFeature->point_3d_old_new;
                        feature_id_cur_new=priorMapFeature->feature_id_cur_new;
                        header_forwKf=priorMapFeature->header;
                        measurements_cur_coarse_pixel_new=priorMapFeature->measurements_cur_coarse_pixel_new;
//                        TE(time_goodfeature);
                    }else{
                        header_forwKf=0.0;
                    }
                }
                
            }
            else
            {
                measurements_cur_coarse_pixel_new.clear();
            }
//            添将新检测到的特征点n_pts到forw_pts中，id初始化-1,track_cnt初始化为1.
//            addPoints3();//补齐足够的特征点
//
//            pre_pMap_pts = forw_pMap_pts;
            
        }
        
//        cur_pMap_pts = forw_pMap_pts;
    }
    
   //像素坐标系的点 去畸变,这里要加个判断 手机这种不需要去畸变
    if(isUndistorted){
        undistortedPoints();
    }
    
//把像素坐标系的点 变成图像坐标系的点
    if(img_cnt == 0)
    {
        //update id and msg
        image_msg.clear();
        distorted_image_msg.clear();
        int num_new = 0;
        
        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            completed |= updateID(i);
            if (!completed)
                break;
        }
        if(isUndistorted){
            for(int i = 0; i<ids.size(); i++)
            {
                double x = cur_un_pts[i].x;
                double y = cur_un_pts[i].y;
                 
                double z = 1.0;
                image_msg[(ids[i])] = (Eigen::Vector3d(x, y, z));//添加特征点
                distorted_image_msg[(ids[i])] = (Eigen::Vector3d(cur_pts[i].x, cur_pts[i].y, 1.0));//添加特征点
                
            }
        }else{
            for(int i = 0; i<ids.size(); i++)
            {
                double x = (cur_pts[i].x - PX)/FOCUS_LENGTH_X;
                double y = (cur_pts[i].y - PY)/FOCUS_LENGTH_Y;
                double z = 1.0;
                image_msg[(ids[i])] = (Eigen::Vector3d(x, y, z));//添加特征点

            }
        }
    }
    //finished and tell solver the data is ok
    update_finished = true;
}

//对图像使用光流法进行特征点跟踪ljl 对mvsec小车进行处理
void FeatureTracker::readImage2(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, vector<cv::Point2f> &good_pts, vector<double> &track_len, double header, Eigen::Vector3d &P, Eigen::Matrix3d &R, bool vins_normal)
{
    
    result = _img;
    if(forw_img.empty())
        pre_img = cur_img = forw_img = _img;//prev_img： 上一次发布数据时对应的图像
    else
        forw_img = _img;

    
    
    forw_pts.clear();
    //track
    {
        if(cur_pts.size()>0)
        {
            vector<uchar> status;
            vector<float> err;
            
            //TS(time_track);
//            对前一帧的特征点cur_pts进行金字塔光流跟踪，得到forw_pts
//         status标记了cur_pts中各个特征点的跟踪状态，根据status将跟踪失败的特征点从prev_pts、cur_pts和forw_pts中剔除，而且在记录特征点id的ids，和记录特征点被跟踪次数的track_cnt中，也要把这些跟踪失败的特征点对应位置的记录删除
//            被status标记为跟踪正常的特征点，在当前帧图像中的位置可能已经处于图像边界外了，这些特征点也应该被删除，删除操作同上。
            calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
            //TE(time_track);
            //这里可以加 去掉跟踪上的 小车上的点
            for (int i = 0; i < int(forw_pts.size()); i++){
                if (status[i] && !inBorder(forw_pts[i])){
                    status[i] = 0;
                    //ljl
                    continue;
                }
                
                if(status[i] && inBorder_car(forw_pts[i])){
                    status[i] =0;
                }
            }
                
            reduceVector(pre_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(forw_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(parallax_cnt, status);
            if(isUndistorted){
                reduceVector(cur_un_pts, status);
            }
            
            //reject outliers
            if (forw_pts.size() >= 8)
            {
                
                
                
                vector<uchar> status;
                if(isUndistorted){
                    vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
                    for (unsigned int i = 0; i < cur_pts.size(); i++)
                    {
                        Eigen::Vector3d tmp_p;
                        m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
//                        tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                        tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                        
                        tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

                        m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
//                        tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                        tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                        
                        tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
                    }
                    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
                    reduceVector(cur_un_pts, status);
                }
                else{
                    cv::findFundamentalMat(cur_pts, forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
                }
                
                reduceVector(cur_pts, status);
                reduceVector(pre_pts, status);
                reduceVector(forw_pts, status);
                reduceVector(ids, status);
                reduceVector(track_cnt, status);
                reduceVector(parallax_cnt, status);
                
            }
            
            solveVinsPnP(header, P, R, vins_normal);//这里应该也要去畸变
            
            if(img_cnt!=0)
            {
                for (int i = 0; i< forw_pts.size(); i++)
                {
                    if(isUndistorted){
                        Eigen::Vector3d tmp_p;
                        cv::Point2f un_forw_pts;
                        m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
//                        tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                        tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                        
                        tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                        tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                        un_forw_pts=cv::Point2f(tmp_p.x(),tmp_p.y());
                        //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                        good_pts.push_back(un_forw_pts);
                        if(un_forw_pts.x < parallax_cnt[i].min.x || un_forw_pts.y < parallax_cnt[i].min.y)
                        {
                            parallax_cnt[i].min = un_forw_pts;
                        }
                        else if(un_forw_pts.x > parallax_cnt[i].max.x || un_forw_pts.y > parallax_cnt[i].max.y)
                        {
                            parallax_cnt[i].max = un_forw_pts;
                        }
                    }else{
                        //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                        good_pts.push_back(forw_pts[i]);
                        if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
                        {
                            parallax_cnt[i].min = forw_pts[i];
                        }
                        else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
                        {
                            parallax_cnt[i].max = forw_pts[i];
                        }
                    }
                    
                    double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                    track_len.push_back(std::min(1.0, 1.0 * parallax/30));
                }
            }
        }
    }
    //detect
    {
        
        if(img_cnt==0)
        {
//            如果需要发布当前帧的数据,通过F矩阵去除outliers。剩下的特征点track_cnt都加1
            rejectWithF();
            for (int i = 0; i< forw_pts.size(); i++)
            {
                if(isUndistorted){
                    Eigen::Vector3d tmp_p;
                    cv::Point2f un_forw_pts;
                    m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
//                    tmp_p.x() = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//                    tmp_p.y() = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
                    
                    tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                    tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                    un_forw_pts=cv::Point2f(tmp_p.x(),tmp_p.y());
                    //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                    good_pts.push_back(un_forw_pts);
                    if(un_forw_pts.x < parallax_cnt[i].min.x || un_forw_pts.y < parallax_cnt[i].min.y)
                    {
                        parallax_cnt[i].min = un_forw_pts;
                    }
                    else if(un_forw_pts.x > parallax_cnt[i].max.x || un_forw_pts.y > parallax_cnt[i].max.y)
                    {
                        parallax_cnt[i].max = un_forw_pts;
                    }
                }else{
                    good_pts.push_back(forw_pts[i]);
                    if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
                    {
                        parallax_cnt[i].min = forw_pts[i];
                    }
                    else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
                    {
                        parallax_cnt[i].max = forw_pts[i];
                    }
                }
                double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                track_len.push_back(std::min(1.0, 1.0 * parallax/50));
            }
            for (auto &n : track_cnt)
                n++;
//            通过设置一个mask，使跟踪的特征点在整幅图像中能够均匀分布，防止特征点扎堆
//            对光流跟踪到的特征点forw_pts，按照被跟踪到的次数降序排列，然后按照降序遍历这些特征点。每选中一个特征点，在mask中将该点周围半径为MIN_DIST的区域设置为0，后面不再选取该区域内的特征点。这样会删去一些特征点，使得特征点分布得更加均匀，同时尽可能地保留被跟踪次数更多的特征点
            setMask();
            int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
            
            if(n_max_cnt>0)
            {
                n_pts.clear();
//                TS(time_goodfeature);
                //goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.10, MIN_DIST, mask, 3, false, 0.04);
//                只有需要发布数据时，才会检测新的特征点，否则只跟踪，不检测新的特征点
//                在mask中不为0的区域检测新的特征点，将特征点数量补充至指定数量 id初始化为-1，track_cnt初始化为1
                goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.01, MIN_DIST, mask);
                
//                TE(time_goodfeature);
            }
            else
            {
                n_pts.clear();
            }
            
            addPoints2();//补齐足够的特征点
            //printf("features num after detect: %d\n",static_cast<int>(forw_pts.size()));
            pre_img = forw_img;
            pre_pts = forw_pts;
            if(isUndistorted){
                prev_un_pts=cur_un_pts;
            }

            //draw
            for (int i = 0; i < n_pts.size(); i++)
            {
                //去掉车头的像素
                
                if(inBorder_car(n_pts[i])){
                    continue;
                }
                good_pts.push_back(n_pts[i]);
                track_len.push_back(0);
            }
            //result = mask;
            
        }
        cur_img = forw_img;
        cur_pts = forw_pts;
    }
   //像素坐标系的点 去畸变,这里要加个判断 手机这种不需要去畸变
    if(isUndistorted){
        undistortedPoints();
    }
    
//把像素坐标系的点 变成图像坐标系的点
    if(img_cnt == 0)
    {
        //update id and msg
        image_msg.clear();
        distorted_image_msg.clear();
        int num_new = 0;
        
        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            completed |= updateID(i);
            if (!completed)
                break;
        }
        if(isUndistorted){
            for(int i = 0; i<ids.size(); i++)
            {
                double x = cur_un_pts[i].x;
                double y = cur_un_pts[i].y;
                 
                double z = 1.0;
                image_msg[(ids[i])] = (Eigen::Vector3d(x, y, z));//添加特征点
                
//                Eigen::Vector3d tmp_p;
//                tmp_p.x() = FOCAL_LENGTH * x / z + COL / 2.0;
//                tmp_p.y() = FOCAL_LENGTH * y / z + ROW / 2.0;
//                tmp_p.x() = (tmp_p.x() - PX)/FOCUS_LENGTH_X;
//                tmp_p.y() =(tmp_p.y() - PY)/FOCUS_LENGTH_Y;
//                tmp_p.z()=1;
//                image_msg[(ids[i])] =tmp_p;//添加特征点
                
                
              
                distorted_image_msg[(ids[i])] = (Eigen::Vector3d(cur_pts[i].x, cur_pts[i].y, 1.0));//添加特征点
                
            }
        }else{
            for(int i = 0; i<ids.size(); i++)
            {
                double x = (cur_pts[i].x - PX)/FOCUS_LENGTH_X;
                double y = (cur_pts[i].y - PY)/FOCUS_LENGTH_Y;
                double z = 1.0;
                image_msg[(ids[i])] = (Eigen::Vector3d(x, y, z));//添加特征点
                
    //            cout<<"x="<<x<<" y="<<y<<endl;
            }
        }
    }
    
   

    //finished and tell solver the data is ok
    update_finished = true;
}

//更新特征点id
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

//对角点进行去畸变矫正，并计算每个角点的速度
void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
   
    prev_un_pts_map = cur_un_pts_map;
}


void FeatureTracker::undistortedPoints_handlerImg(cv::Mat &img)
{
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    //内参矩阵, 就算复制代码，也不要用我的参数。摄像头都不一样...
    cameraMatrix.at<double>(0, 0) = FOCUS_LENGTH_X;
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = PX;
    cameraMatrix.at<double>(1, 1) = FOCUS_LENGTH_Y;
    cameraMatrix.at<double>(1, 2) = PY;
    //畸变参数，不要用我的参数~
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = k1_global;
    distCoeffs.at<double>(1, 0) = k2_global;
    distCoeffs.at<double>(2, 0) = p1_global;
    distCoeffs.at<double>(3, 0) = p2_global;
    distCoeffs.at<double>(4, 0) = 0;
    cv::Mat view, rview, map1, map2;
    cv::Size imageSize;
    imageSize = img.size();
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),imageSize, CV_16SC2, map1, map2);
    cv::remap(img, img, map1, map2, cv::INTER_LINEAR);
    
    
    
    
//    cv::Mat img_de;
//    img.copyTo(img_de);
//    for (unsigned int i = 0 , j=img_de.rows; i < j; i++)
//    {
//        for(unsigned int a = 0 , b=img_de.cols; a < b; a++){
//
//            Eigen::Vector2d u(i, a);
//            Eigen::Vector3d v;
//            m_camera->liftProjective(u, v);
//            int u_distorted = FOCUS_LENGTH_X*v.x() / v.z()+PX;
//
//            int v_distorted = FOCUS_LENGTH_Y*v.y() / v.z()+PY;
//            img.at<unsigned char>(i, a)=img_de.at<unsigned char>(v_distorted,  u_distorted);
//
//        }
//    }
//    img_de.release();
   
}

//读取相机内参
void FeatureTracker::readIntrinsicParameter()
{
    
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile();
}

void FeatureTracker::setLoopKf(double (&_loop_pose_forFeatureTracker)[7]){
    double loop_pose[7];
    for(int i=0;i<7;i++){
        loop_pose[i]=_loop_pose_forFeatureTracker[i];
    }
    loop_pose_priorMap.push(loop_pose);
}
