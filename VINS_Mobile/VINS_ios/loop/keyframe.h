#include <opencv2/opencv.hpp>
#ifndef __KEY_FRAME_
#define __KEY_FRAME_


#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "utility.hpp"
#include <algorithm>
#include "math.h"
#include "global_param.hpp"
#include "VINS.hpp"
#include "loop_closure.h"

#include "BowVector.h"
#include "FeatureVector.h"

#include "feature_tracker.hpp"


//using namespace Eigen;
using namespace std;
//using namespace cv;

class BriefExtractor: public FeatureExtractor<FBrief::TDescriptor>
{
public:
    virtual void operator()(const cv::Mat &im, const std::vector<cv::Point2f> window_pts,
                            vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const;
    
    BriefExtractor(const std::string &pattern_file);
    
private:
    DVision::BRIEF m_brief;
};

struct matchCluster
{
    //stores nearest points with old vins point
    std::vector<int> indexs; //bow points index
    int best_index;
};

class VINS;

class FeatureTracker;

class KeyFrame
{
public:
    KeyFrame(double _header, int _global_index, Eigen::Vector3d _T_w_c, Eigen::Matrix3d _R_w_c, cv::Mat &_image, const char *_brief_pattern_file, const int _segment_index);
    void setExtrinsic(Eigen::Vector3d T, Eigen::Matrix3d R);
    void initPtsByReprojection(Eigen::Vector3d Ti_predict,
                               Eigen::Matrix3d Ri_predict,
                               std::vector<cv::Point2f> &measurements_predict);
    void initPoseForPnP(Eigen::Vector3d &T_c_w,
                        Eigen::Matrix3d &R_c_w);
    void cam2Imu(Eigen::Vector3d T_c_w,
                 Eigen::Matrix3d R_c_w,
                 Eigen::Vector3d &T_w_i,
                 Eigen::Matrix3d &R_w_i);
    void rejectWithF(vector<cv::Point2f> &measurements_old,
                     vector<cv::Point2f> &measurements_old_norm);
    void extractBrief(cv::Mat &image);
    void extractBrief(cv::Mat &image,FeatureTracker &feature_tracker);
    void searchInBoW(std::vector<cv::Point2f> &cur_pts,
                     std::vector<cv::Point2f> &old_pts,
                     std::vector<cv::Point2f> &old_measurements);//没实现
    void buildKeyFrameFeatures(VINS &vins);
    
    void searchByDes(std::vector<cv::Point2f> &measurements_old,
                     std::vector<cv::Point2f> &measurements_old_norm,
                     const std::vector<BRIEF::bitset> &descriptors_old,
                     const std::vector<cv::KeyPoint> &keypoints_old);
    
    bool solveOldPoseByPnP(std::vector<cv::Point2f> &measurements_old_norm,
                           const Eigen::Vector3d T_w_i_old, const Eigen::Matrix3d R_w_i_old,
                           Eigen::Vector3d &T_w_i_refine, Eigen::Matrix3d &R_w_i_refine);
    
    bool findConnectionWithOldFrame(const KeyFrame* old_kf,
                                    const std::vector<cv::Point2f> &cur_pts, const std::vector<cv::Point2f> &old_pts,
                                    std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm);
    void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
    
    void updateOriginPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
    
    void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
    
    void getOriginPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
    
    void addConnection(int index, KeyFrame* connected_kf);
    
    void addConnection(int index, KeyFrame* connected_kf, Eigen::Vector3d relative_t, Eigen::Quaterniond relative_q, double relative_yaw);
    
    void addLoopConnection(int index, KeyFrame* loop_kf);
    
    void updateLoopConnection(Eigen::Vector3d relative_t, Eigen::Quaterniond relative_q, double relative_yaw);
    
    void detectLoop(int index);
    
    void removeLoop();
    
    int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);
    
    int getWinPointsSize();
    
    // data
    double header;
//    滑动窗口第二新帧时，会变成关键帧，然后从feature中读取3d点，此后没有再矫正过
    std::vector<Eigen::Vector3d> point_clouds, point_clouds_origin;//世界坐标系 基本可以说完全没有用到过 也是带了绝对尺度的
    //feature in origin image plane
    //因为这些都考虑了相机内参了 后续处理可以直接令相机内参为单位矩阵
    //measurements_origin 观测到该帧下的特征点的其它帧下 该点的坐标 相机平面坐标
    //measurements 存到是窗口里的点 提取特征点时
    //是像素坐标
    std::vector<cv::Point2f> measurements, measurements_origin , distorted_measurements_origin_uv;//汉明距离最小的特征点 70个点的坐标 searchByDes前后有变化，做了回环检测后，会有一些删除不好的点（外点）
    //feature in normalize image plane
    std::vector<cv::Point2f> pts_normalize;//关键点归一化后的坐标 3D点归一化坐标 没有用
    //feature ID全局id
    std::vector<int> features_id, features_id_origin;//拒绝外点回环过的特征点id  当前帧，滑动窗口范围内符合条件的全部特征点id
    //feature descriptor
    std::vector<BRIEF::bitset> descriptors;//570个 500+70
    //keypoints
    std::vector<cv::KeyPoint> keypoints;//570个点 后面就没有改变了
    //实验用
    std::vector<cv::KeyPoint> keypoints_distorted;//570个点 后面就没有改变了
    //3d 空间点 应该没有填错
    std::vector<Eigen::Vector3d> pts_3d_server;
    
    int global_index;//全局index
    cv::Mat image;
    Eigen::Matrix3d qic;
    Eigen::Vector3d tic;
    bool use_retrive;
    bool has_loop;
    bool check_loop;
    // looped by other frame
    bool is_looped;
    int loop_index;//回环帧的index
    int resample_index;
    const char *BRIEF_PATTERN_FILE;
    // index t_w t_y t_z q_w q_x q_y q_z yaw
    list<pair<int, Eigen::Matrix<double, 8, 1 > > > connection_list;
    Eigen::Matrix<double, 8, 1 > loop_info;//和闭环帧的相对位姿 yaw
    int segment_index;//序列号
    
    
    bool has_global_loop;
    //窗口点的深度
//    std::vector<double> win_keyPoint_depth;
    
    //应该是像素坐标
    std::vector<cv::KeyPoint> window_keypoints;
    std::vector<BRIEF::bitset> window_descriptors;//70个 searchByDes前后没变化
    
    
    //ljl
    void rejectWithF_server(vector<uchar> status);
//    std::vector<double> win_point_z;
    
//    void rejectWithF_my1(vector<cv::Point2f> &measurements_old,
//                                   vector<cv::Point2f> &measurements_old_norm,vector<cv::Point2f> window_keys);
//    void searchByDes_my1(std::vector<cv::Point2f> &measurements_old,
//    std::vector<cv::Point2f> &measurements_old_norm,
//    const std::vector<BRIEF::bitset> &descriptors_old,
//                         const std::vector<cv::KeyPoint> &keypoints_old);
    
    void getPose_2Server(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
    void updatePose_2Server(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
    
    std::mutex mMutexConnections;
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights; ///< 与该关键帧连接的关键帧与权重
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames; ///< 排序后的关键帧
    std::vector<int> mvOrderedWeights; ///< 排序后的权重(从大到小)
    //添加两帧之间 新的观测关系
    void AddConnection(KeyFrame *pKF, const int &weight);
    //更新两帧之间的权重
    void UpdateBestCovisibles();
    //返回共视程度最高的N帧
    vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    bool IsOriginUpdate;//收到原始位姿
    
    bool isInImage(float x, float y);
    vector<int> GetFeaturesInArea(const float &x, const float &y, const float &r);//找的是measurements 到时候用的是世界坐标系下的点
    vector<int> GetFeaturesInArea_1(const float &x, const float &y, const float &r);//找到是keyPoints 这个到时候提供keypoints 图像坐标系的点
    
    //------------地图融合
    void rejectWithF_server_mapFuse(vector<cv::Point2f> &measurements_old, vector<cv::Point2f> &measurements_cur, vector<Eigen::Vector3d> &pointsCloud_old_3d,vector<int> &feature_id_cur);
    
    Eigen::Matrix<double, 4, 1 > loop_info_better;
    void updateLoopConnection(Eigen::Vector3d relative_t, double relative_yaw);
    Eigen::Quaterniond loop_info_better_q;
    double relative_pitch;
    double relative_roll;
    
    int mnTrackReferenceForFrame;//防止重复添加到局部地图中
    
    bool is_des_end;//因为des是分开发的，记录一下是否接收完毕了
    vector<uchar> bitstream;
    std::vector< int> visualWords;
    
    int isProjection_findPriorFeature;//记录找过的先验信息是谁 kfId
    
    int edge=0;
    int edge_single=0;
    double var_imu=0.0;
private:
    Eigen::Vector3d T_w_i;//优化后的 位置向量，偏移都已经加上去了 有尺度信息，世界坐标系
    Eigen::Matrix3d R_w_i;
    Eigen::Vector3d origin_T_w_i;//优化前 有漂移的位姿
    Eigen::Matrix3d origin_R_w_i;
    std::mutex mMutexPose;
    
    //这个存放的是 没有乘以偏移量的 把这个发送给服务器，然后在服务器端乘以偏移量
    Eigen::Vector3d T_w_i_2Server;
    Eigen::Matrix3d R_w_i_2Server;
    
    
};

#endif

