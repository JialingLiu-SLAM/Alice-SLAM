#include "keyframe.h"


KeyFrame::KeyFrame(double _header, int _global_index, Eigen::Vector3d _T_w_i, Eigen::Matrix3d _R_w_i,
                   cv::Mat &_image, const char *_brief_pattern_file, const int _segment_index)
:header{_header}, global_index{_global_index}, T_w_i{_T_w_i}, R_w_i{_R_w_i}, image{_image}, BRIEF_PATTERN_FILE(_brief_pattern_file), segment_index(_segment_index)
{
    use_retrive = 0;
    is_looped = 0;
    has_loop = 0;
    origin_T_w_i = _T_w_i;
    origin_R_w_i = _R_w_i;
    check_loop = 0;
    
    has_global_loop=0;
    IsOriginUpdate=false;
    
    //ljl
    loop_index=-1;
    is_des_end=false;
    
    mnTrackReferenceForFrame=-1;
    
    isProjection_findPriorFeature=-1;
}

/*****************************************utility function************************************************/

void KeyFrame::getPose_2Server(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i){
        _T_w_i = T_w_i_2Server;
        _R_w_i = R_w_i_2Server;
}
void KeyFrame::updatePose_2Server(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i){
    T_w_i_2Server = _T_w_i;
    R_w_i_2Server = _R_w_i;
}

//euroc
//#define COL  752
//#define ROW 480

//没有用
bool inBorder2(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
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

void KeyFrame::rejectWithF(vector<cv::Point2f> &measurements_old,
                           vector<cv::Point2f> &measurements_old_norm)
{
    if (measurements_old.size() >= 8)
    {
        measurements_old_norm.clear();
        
        for (unsigned int i = 0; i < measurements_old.size(); i++)
        {
            cv::Point2f norm_pt;
            norm_pt.x = (measurements_old[i].x - PX)/FOCUS_LENGTH_X;
            norm_pt.y = (measurements_old[i].y - PY)/FOCUS_LENGTH_Y;
            measurements_old_norm.push_back(norm_pt);
        }
        
        
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
        
//        vector<uchar> status;
//        cv::findFundamentalMat(measurements, measurements_old, cv::FM_RANSAC, 2.0, 0.99, status);
//        reduceVector2(point_clouds, status);
//        reduceVector2(measurements, status);
//        reduceVector2(measurements_old, status);
//        reduceVector2(measurements_old_norm, status);
//        reduceVector2(features_id, status);
        
    }
}

void KeyFrame::rejectWithF_server_mapFuse(vector<cv::Point2f> &measurements_old,
                           vector<cv::Point2f> &measurements_cur,vector<Eigen::Vector3d> &pointsCloud_old_3d,vector<int> &feature_id_cur)
{
//    if (measurements_cur.size() >= 8)
//    {
         
    //这里改一下 改成mat
    
    int ptCount = (int)measurements_old.size();
    cv::Mat p1(ptCount, 2, CV_32F);
    cv::Mat p2(ptCount, 2, CV_32F);
     
    // 把Keypoint转换为Mat
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
        reduceVector2(feature_id_cur, status);
        
//    }
}

//void KeyFrame::rejectWithF_my1(vector<cv::Point2f> &measurements_old,
//                           vector<cv::Point2f> &measurements_old_norm,vector<cv::Point2f> window_keys)
//{
//    if (measurements_old.size() >= 8)
//    {
//        measurements_old_norm.clear();
//
//        for (unsigned int i = 0; i < measurements_old.size(); i++)
//        {
//            cv::Point2f norm_pt;
//            norm_pt.x = (measurements_old[i].x - PX)/FOCUS_LENGTH_X;
//            norm_pt.y = (measurements_old[i].y - PY)/FOCUS_LENGTH_Y;
//            measurements_old_norm.push_back(norm_pt);
//        }
//
//        vector<uchar> status;
//        cv::findFundamentalMat(window_keys, measurements_old, cv::FM_RANSAC, 2.0, 0.99, status);
////        cv::findFundamentalMat(measurements, measurements_old, cv::FM_RANSAC, 2.0, 0.99, status);
//        reduceVector2(point_clouds, status);
//        reduceVector2(measurements, status);
//        reduceVector2(measurements_old, status);
//        reduceVector2(measurements_old_norm, status);
//        reduceVector2(features_id, status);
//        reduceVector2(window_keypoints, status);
//        reduceVector2(window_descriptors, status);
////        for(int i=0;i<status.size();i++){
////            cout<<"发送send_status   "<<status[i]<<"  size="<<status.size()<<endl;
////        }
//    }
//}

void KeyFrame::rejectWithF_server(vector<uchar> status){
    reduceVector2(point_clouds, status);
    reduceVector2(measurements, status);
    reduceVector2(features_id, status);
}


/*****************************************utility function************************************************/
//70个点的
void KeyFrame::extractBrief(cv::Mat &image)
{
    BriefExtractor extractor(BRIEF_PATTERN_FILE);
    extractor(image, measurements, keypoints, descriptors);
     
    int start = keypoints.size() - measurements.size();
     
    for(int i = 0; i< measurements.size(); i++)
    {
        window_keypoints.push_back(keypoints[start + i]);
        window_descriptors.push_back(descriptors[start + i]);
    }
//    assert(window_keypoints.size()==win_point_z.size());
    is_des_end=true;
}

void KeyFrame::extractBrief(cv::Mat &image,FeatureTracker &featuretracker)
{
    BriefExtractor extractor(BRIEF_PATTERN_FILE);
    extractor(image,  distorted_measurements_origin_uv, keypoints, descriptors);
  
    //实验用
    keypoints_distorted=keypoints;
    
    //因为这个measurements_len是去了畸变的
    int start = keypoints.size() - measurements.size();
    //只给key_points去畸变 去畸变的过程中 把自己从像素坐标系变成了图像坐标系
    for (int i = 0; i < start; i++)
    {
         Eigen::Vector3d tmp_p;
         featuretracker.m_camera->liftProjective(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);
        
        //再改回像素坐标 好像匹配的数量低于阈值
//       float x = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
//       float y = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
        
        float x = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
        float y = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
        cv::Point2f tmp_norm;
        tmp_norm = cv::Point2f(x, y);
        
         keypoints[i].pt=tmp_norm;
    }
//    for(int i=0,j=measurements.size();i<j;i++){
//        Eigen::Vector3d tmp_p;
//        featuretracker.m_camera->liftProjective(Eigen::Vector2d(distorted_measurements_origin_uv[i].x, distorted_measurements_origin_uv[i].y), tmp_p);
//        float x = FOCUS_LENGTH_X * tmp_p.x() / tmp_p.z() + PX;
//        float y = FOCUS_LENGTH_Y * tmp_p.y() / tmp_p.z() + PY;
//        cout<<"测试："<<x<<" "<<y<<" "<<measurements[i].x<<" "<<measurements[i].y<<endl;
//        
//    }
    
    
    for(int i=0,j=measurements.size();i<j;i++)
    {
        keypoints[start+i].pt=measurements[i];//这里应该可以注释
        window_keypoints.push_back(keypoints[start + i]);
        window_descriptors.push_back(descriptors[start + i]);
    }
//    assert(window_keypoints.size()==win_point_z.size());
    is_des_end=true;
}

void KeyFrame::setExtrinsic(Eigen::Vector3d T, Eigen::Matrix3d R)
{
    qic = R;
    tic = T;
}
//没有调用
void KeyFrame::initPtsByReprojection(Eigen::Vector3d Ti_predict,
                                     Eigen::Matrix3d Ri_predict,
                                     std::vector<cv::Point2f> &measurements_predict)
{
    measurements_predict.clear();
    Eigen::Vector3d pts_predict;
    for(int i = 0; i < (int)point_clouds.size(); i++)
    {
        Eigen::Vector3d pts_w = point_clouds[i];
        Eigen::Vector3d pts_imu_j = Ri_predict.inverse() * (pts_w - Ti_predict);
        Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
        pts_predict <<  pts_camera_j.x()/pts_camera_j.z(),
        pts_camera_j.y()/pts_camera_j.z(),
        1.0;
        Eigen::Vector2d point_uv;
        point_uv.x() = FOCUS_LENGTH_X * pts_predict.x() + PX;
        point_uv.y() = FOCUS_LENGTH_Y * pts_predict.y() + PY;
        measurements_predict.push_back(cv::Point2f(point_uv.x(), point_uv.y()));
    }
    if(measurements_predict.size() == 0)
    {
        measurements_predict = measurements;
    }
}

//没有用
void KeyFrame::initPoseForPnP(Eigen::Vector3d &T_c_w,
                              Eigen::Matrix3d &R_c_w)
{
    Eigen::Matrix3d R_w_c = R_w_i * qic;
    Eigen::Vector3d T_w_c = T_w_i + R_w_i * tic;
    
    R_c_w = R_w_c.inverse();
    T_c_w = -(R_c_w * T_w_c);
}
//没有用
void KeyFrame::cam2Imu(Eigen::Vector3d T_c_w,
                       Eigen::Matrix3d R_c_w,
                       Eigen::Vector3d &t_w_i,
                       Eigen::Matrix3d &r_w_i)
{
    r_w_i = (qic * R_c_w).inverse();
    t_w_i = -R_c_w.inverse() * T_c_w - r_w_i * tic;
}

double round(double r)
{
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}
//将空间的3D点构建当前关键帧的特征点
void KeyFrame::buildKeyFrameFeatures(VINS &vins)
{
    map<KeyFrame*,int> KFcounter; // 关键帧-权重，权重为其它关键帧与当前关键帧共视3d点的个数
    
    pts_3d_server.clear();
    for (auto &it_per_id : vins.f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        int frame_size = it_per_id.feature_per_frame.size();
        if(it_per_id.start_frame + frame_size >= WINDOW_SIZE - 1&& frame_size >=3)
        {
            FeaturePerFrame feature_per_frame_test=it_per_id.feature_per_frame[WINDOW_SIZE - 2 - it_per_id.start_frame];
            
            if(isUndistorted){
                Eigen::Vector3d distorted_point_uv=feature_per_frame_test.distorted_point_uv;
                distorted_measurements_origin_uv.push_back(cv::Point2f(distorted_point_uv.x(),distorted_point_uv.y()));
                
                //实验记录一下
//                assert(feature_per_frame_test.point.size()==feature_per_frame_test.distorted_point_uv.size());
                
            }
            
            //features current measurements 这里用window_size-2是因为插入关键帧 是window_size-2时确定的
            Eigen::Vector3d point = feature_per_frame_test.point;//相机平面坐标系的点 xy知道了，深度不知道设为1
            
            
            
            Eigen::Vector2d point_uv;
            point_uv.x() = FOCUS_LENGTH_X * point.x()/point.z() + PX;
            point_uv.y() = FOCUS_LENGTH_Y * point.y()/point.z() + PY;
            measurements.push_back(cv::Point2f(point_uv.x(), point_uv.y()));
            pts_normalize.push_back(cv::Point2f(point.x()/point.z(), point.y()/point.z()));
            
            features_id.push_back(it_per_id.feature_id);
            //features 3D pos from first measurement and inverse depth
            Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            
            
            pts_3d_server.push_back(pts_i);
            point_clouds.push_back(vins.Rs[it_per_id.start_frame] * (vins.ric * pts_i + vins.tic) + vins.Ps[it_per_id.start_frame]);//世界坐标系 3D位置完全知道了的
            
            
            
            
            //下面应该更新 老的kf关联的kf的权重
            for(auto mit=it_per_id.per_kf.begin(), mend=it_per_id.per_kf.end(); mit!=mend; mit++)
            {
                // 除去自身，自己与自己不算共视 这里会默认为0？？？
                KFcounter[*mit]++;
            }
            //ljl 存储哪些地图点观测到哪些关键帧
            it_per_id.per_kf.push_back(this);
        }
    }
    
    measurements_origin  = measurements;
    point_clouds_origin = point_clouds;
    features_id_origin = features_id;
//    assert(measurements.size()==point_clouds.size());
    
    // This should not happen 这个只应该第一帧发生，没有共视帧
    if(KFcounter.empty()){
        printf("这个只能出现一次！！\n");
        return;
    }
    
    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    // vPairs记录与其它关键帧共视帧数大于th的关键帧
    // pair<int,KeyFrame*>将关键帧的权重写在前面，关键帧写在后面方便后面排序
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        
        if(mit->second>nmax)
        {
            nmax=mit->second;
            // 找到对应权重最大的关键帧（共视程度最高的关键帧）
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            // 对应权重需要大于阈值，对这些关键帧建立连接
            vPairs.push_back(make_pair(mit->second,mit->first));
            // 更新KFcounter中该关键帧的mConnectedKeyFrameWeights
            // 更新其它KeyFrame的mConnectedKeyFrameWeights，更新其它关键帧与当前帧的连接权重
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    // 如果没有超过阈值的权重，则对权重最大的关键帧建立连接
    if(vPairs.empty())
    {
        // 如果每个关键帧与它共视的关键帧的个数都少于th，
        // 那就只更新与其它关键帧共视程度最高的关键帧的mConnectedKeyFrameWeights
        // 这是对之前th这个阈值可能过高的一个补丁
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    // vPairs里存的都是相互共视程度比较高的关键帧和共视权重，由大到小
    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        mMutexConnections.lock();

        // 更新图的连接(权重)
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

// 更新生成树的连接
//        if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
//        {
//
//            // 初始化该关键帧的父关键帧为共视程度最高的那个关键帧
//            mpParent = mvpOrderedConnectedKeyFrames.front();
//            // 建立双向连接关系
//            mpParent->AddChild(this);
//            mbFirstConnection = false;
//        }
        mMutexConnections.unlock();

    }
    
    
    
    
    
    
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        mMutexConnections.lock();
        // std::map::count函数只可能返回0或1两种情况
        if(!mConnectedKeyFrameWeights.count(pKF)) // count函数返回0，mConnectedKeyFrameWeights中没有pKF，之前没有连接
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight) // 之前连接的权重不一样
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
        
        mMutexConnections.unlock();
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    mMutexConnections.lock();
    // http://stackoverflow.com/questions/3389648/difference-between-stdliststdpair-and-stdmap-in-c-stl
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    // 取出所有连接的关键帧，mConnectedKeyFrameWeights的类型为std::map<KeyFrame*,int>，而vPairs变量将共视的3D点数放在前面，利于排序
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

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

/**
 ** search matches by guide descriptor match
 **当前关键帧与闭环候选帧进行BRIEF描述子匹配，这里相当于是在2d-2d之间进行匹配
 **/
void KeyFrame::searchByDes(std::vector<cv::Point2f> &measurements_old,
                           std::vector<cv::Point2f> &measurements_old_norm,
                           const std::vector<BRIEF::bitset> &descriptors_old,
                           const std::vector<cv::KeyPoint> &keypoints_old)
{
    printf("loop_match before cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
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
    printf("loop_match after cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
}

//void KeyFrame::searchByDes_my1(std::vector<cv::Point2f> &measurements_old,
//                           std::vector<cv::Point2f> &measurements_old_norm,
//                           const std::vector<BRIEF::bitset> &descriptors_old,
//                           const std::vector<cv::KeyPoint> &keypoints_old)
//{
//    printf("loop_match before cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
//    std::vector<int> dis_cur_old;
//    std::vector<uchar> status;
//    vector<cv::Point2f> window_keys;
//    for(int i = 0; i < window_descriptors.size(); i++)
//    {
//        int bestDist = 256;
//        int bestIndex = -1;
//        for(int j = 0; j < descriptors_old.size(); j++)
//        {
//            int dis = HammingDis(window_descriptors[i], descriptors_old[j]);
//            if(dis < bestDist)
//            {
//                bestDist = dis;
//                bestIndex = j;
//            }
//        }
//        if(bestDist < 256)
//        {
//            measurements_old.push_back(keypoints_old[bestIndex].pt);
//            dis_cur_old.push_back(bestDist);
//            window_keys.push_back(window_keypoints[i].pt);
//        }
//    }
//    rejectWithF_my1(measurements_old, measurements_old_norm,window_keys);
//    printf("loop_match after cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
//}

/**
 *** return refined pose of the current frame 没有用
 **/
bool KeyFrame::solveOldPoseByPnP(std::vector<cv::Point2f> &measurements_old_norm,
                                 const Eigen::Vector3d T_w_i_old, const Eigen::Matrix3d R_w_i_old,
                                 Eigen::Vector3d &T_w_i_refine, Eigen::Matrix3d &R_w_i_refine)
{
    //solve PnP get pose refine
    cv::Mat r, rvec, t, D, tmp_r;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    Eigen::Matrix3d R_inital;
    Eigen::Vector3d P_inital;
    Eigen::Matrix3d R_w_c = R_w_i_old * qic;
    Eigen::Vector3d T_w_c = T_w_i_old + R_w_i * tic;
    
    R_inital = R_w_c.inverse();
    P_inital = -(R_inital * T_w_c);
    
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);
    
    vector<cv::Point3f> pts_3_vector;
    bool pnp_succ = false;
    for(auto &it: point_clouds)
        pts_3_vector.push_back(cv::Point3f((float)it.x(),(float)it.y(),(float)it.z()));
    if(pts_3_vector.size()>=30)
    {
        if(!use_retrive)
        {
            pnp_succ = cv::solvePnP(pts_3_vector, measurements_old_norm, K, D, rvec, t, 1);
        }
        else
        {
            initPoseForPnP(P_inital,R_inital);
            cv::eigen2cv(R_inital, tmp_r);
            cv::Rodrigues(tmp_r, rvec);
            cv::eigen2cv(P_inital, t);
            pnp_succ = cv::solvePnP(pts_3_vector, measurements_old_norm, K, D, rvec, t, 1);
        }
    }
    
    if(!pnp_succ)
    {
        cout << "loop pnp failed !" << endl;
        return false;
    }
    else
    {
        cout << "loop pnp succ !" << endl;
    }
    cv::Rodrigues(rvec, r);
    Eigen::Matrix3d R_loop;
    cv::cv2eigen(r, R_loop);
    Eigen::Vector3d T_loop;
    cv::cv2eigen(t, T_loop);
    
    Eigen::Vector3d old_T_drift;
    Eigen::Matrix3d old_R_drift;
    cam2Imu(T_loop, R_loop, old_T_drift, old_R_drift);
    
    T_w_i_refine = T_w_i_old + R_w_i_old * old_R_drift.transpose() * (T_w_i - old_T_drift);
    R_w_i_refine = R_w_i_old * old_R_drift.transpose() * R_w_i;
    
    //printf("loop current T: %2lf %2lf %2lf\n", T_w_i(0),T_w_i(1),T_w_i(2));
    
    //printf("loop refined T: %2lf %2lf %2lf\n", T_w_i_refine(0),T_w_i_refine(1),T_w_i_refine(2));
    return true;
}

/**
 *** interface to VINS
 *** input: looped old keyframe which include image and pose, and feature correnspondance given by BoW
 *** output: ordered old feature correspondance with current KeyFrame and the translation drift
 **/
bool KeyFrame::findConnectionWithOldFrame(const KeyFrame* old_kf,
                                          const std::vector<cv::Point2f> &cur_pts, const std::vector<cv::Point2f> &old_pts,
                                          std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm)
{
    searchByDes(measurements_old, measurements_old_norm, old_kf->descriptors, old_kf->keypoints);
    return true;
}

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

void KeyFrame::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = T_w_i;
    _R_w_i = R_w_i;
}

void KeyFrame::getOriginPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = origin_T_w_i;
    _R_w_i = origin_R_w_i;
}
//没有用
void KeyFrame::addConnection(int index, KeyFrame* connected_kf)
{
    Eigen::Vector3d connected_t, relative_t;
    Eigen::Matrix3d connected_r;
    Eigen::Quaterniond relative_q;
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
//没有用
void KeyFrame::addConnection(int index, KeyFrame* connected_kf, Eigen::Vector3d relative_t, Eigen::Quaterniond relative_q, double relative_yaw)
{
    Eigen::Matrix<double, 8, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
    relative_yaw;
    connection_list.push_back(make_pair(index, connected_info));
}
//没有用
void KeyFrame::addLoopConnection(int index, KeyFrame* loop_kf)
{
    assert(index == loop_index);
    Eigen::Vector3d connected_t, relative_t;
    Eigen::Matrix3d connected_r;
    Eigen::Quaterniond relative_q;
    loop_kf->getPose(connected_t, connected_r);
    
    relative_q = connected_r.transpose() * R_w_i;
    relative_t = connected_r.transpose() * (T_w_i - connected_t);
    double relative_yaw;
    relative_yaw = Utility::R2ypr(R_w_i).x() - Utility::R2ypr(connected_r).x();
    Eigen::Matrix<double, 8, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
    relative_yaw;
    loop_info = connected_info;
}

void KeyFrame::updateLoopConnection(Eigen::Vector3d relative_t, Eigen::Quaterniond relative_q, double relative_yaw)
{
    Eigen::Matrix<double, 8, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
    relative_yaw;
    loop_info = connected_info;
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

int  KeyFrame::getWinPointsSize(){
    
    return window_keypoints.size();
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
#define HEIGHT 480
#define WIDTH 752
bool KeyFrame::isInImage(float x, float y){
    int width=WIDTH;
    int height=HEIGHT;
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

void KeyFrame::updateLoopConnection(Eigen::Vector3d relative_t, double relative_yaw)
{
    Eigen::Matrix<double, 4, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),relative_yaw;
    loop_info_better = connected_info;
}

BriefExtractor::BriefExtractor(const std::string &pattern_file)
{
    // The DVision::BRIEF extractor computes a random pattern by default when
    // the object is created.
    // We load the pattern that we used to build the vocabulary, to make
    // the descriptors compatible with the predefined vocabulary
    
    // loads the pattern
    cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
    if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;
    
    vector<int> x1, y1, x2, y2;
    fs["x1"] >> x1;
    fs["x2"] >> x2;
    fs["y1"] >> y1;
    fs["y2"] >> y2;
    
    m_brief.importPairs(x1, y1, x2, y2);
}

//这里因为measurements已经去过畸变了 所以取得的描述符可能不那么好
void BriefExtractor::operator() (const cv::Mat &im, const std::vector<cv::Point2f> window_pts,
                                 vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
{
    // extract FAST keypoints with opencv
    const int fast_th = 20; // corner detector response threshold
    cv::FAST(im, keys, fast_th, true);
    for(int i = 0; i < window_pts.size(); i++)
    {
        cv::KeyPoint key;
        key.pt = window_pts[i];
        keys.push_back(key);
    }
    // compute their BRIEF descriptor
    m_brief.compute(im, keys, descriptors);
}

