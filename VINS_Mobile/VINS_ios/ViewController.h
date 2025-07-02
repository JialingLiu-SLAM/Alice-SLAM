//
//  ViewController.h
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//
#import "feature_tracker.hpp"
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/videoio/cap_ios.h>
#import <UIKit/UIKit.h>

#import "MetalView.h"


#import <mach/mach_time.h>
#import "global_param.hpp"
#import "VINS.hpp"
#include <queue>
#import "draw_result.hpp"
#import <CoreMotion/CoreMotion.h>
#include "keyframe.h"
#include "loop_closure.h"
#include "keyfame_database.h"
#include "SocketSingleton.h"
#include "HandleData.h"
#import <sys/utsname.h>
#include "feature_coder.h"


#import <CoreTelephony/CTCellularData.h>
@interface ViewController : UIViewController<CvVideoCameraDelegate,UITextViewDelegate>
{
    CvVideoCamera* videoCamera;
    BOOL isCapturing;
    cv::Ptr<FeatureTracker> feature_tracker;
    cv::Size frameSize;
    uint64_t prevTime;
    NSCondition *_condition;
    NSThread *mainLoop;
    NSThread *draw;
    NSThread *saveData;
    NSThread *saveData_2;
    NSThread *loop_thread;
    NSThread *globalLoopThread;
    NSThread *loop_thread_server;
    NSThread *globalLoopThread_server;
    
    UITextView *textY;
    
    //ljl
//    NSThread *runFromFileThread;
    
    //两个线程接力
    list<vector<vector<cv::Point2f> > > mlvv_measurements_old_norm_all;
    list<vector<vector<Eigen::Vector3d>> > mlvv_point_clouds_all;
    
    list<vector<cv::Point2f> > mlvv_measurements_old_norm_single;
    list<vector<Eigen::Vector3d> > mlvv_point_clouds_single;

    list<vector<Eigen::Matrix3d> > mlv_oldR_b_a;
    list<vector<Eigen::Vector3d> > mlv_oldT_b_a;
    list<Eigen::Matrix3d> ml_relative_r;
    list<Eigen::Vector3d> ml_relative_t;
    list<vector<int> > mlv_vpkf_index;
    list<std::pair<KeyFrame* , KeyFrame*> > mlp_kf_pairs;
    std::mutex relative_mutex;
    
    
}

@property (nonatomic, strong) CvVideoCamera* videoCamera;
@property (nonatomic, strong) IBOutlet UIImageView* imageView;
@property (nonatomic, strong) IBOutlet UIImageView* featureImageView;

//ljl
@property (weak, nonatomic) IBOutlet UITextField *hostText;
@property (weak, nonatomic) IBOutlet UITextField *portText;
@property (weak, nonatomic) IBOutlet UIButton *saveButton;


@property (weak, nonatomic) IBOutlet UIButton *loopButton;
@property (weak, nonatomic) IBOutlet UIButton *reinitButton;

- (IBAction)recordButtonPressed:(id)sender;
- (IBAction)playbackButtonPressed:(id)sender;

@property (weak, nonatomic) IBOutlet UISegmentedControl *switchUI;

@property (nonatomic) BOOL switchUIAREnabled;
@property (weak, nonatomic) IBOutlet UIButton *arButton;

@property (strong, nonatomic) MetalView *ARview;
@property (weak, nonatomic) IBOutlet MetalView *metalARView;

- (void)showInputView;

- (void)setVisibleAnimated:(BOOL)visible;

//用于缓存需要录制的IMU数据结构
struct IMU_MSG {
    NSTimeInterval header;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
};
//用于存储提取的特征点
struct IMG_MSG {
    NSTimeInterval header;
    map<int, Eigen::Vector3d> point_clouds;
    map<int, Eigen::Vector3d> distorted_point_clouds;
};
//用于缓存需要录制的图像数据结构
struct IMG_DATA {
    NSTimeInterval header;
    UIImage *image;
};

struct IMG_DATA_CACHE {
    NSTimeInterval header;
    cv::Mat equ_image;
    UIImage *image;
};

struct VINS_DATA_CACHE {
    NSTimeInterval header;
    Eigen::Vector3f P;
    Eigen::Matrix3f R;
};

typedef shared_ptr <IMU_MSG const > ImuConstPtr;
typedef shared_ptr <IMG_MSG const > ImgConstPtr;

//ljl
- (void)connectServer;
//- (void) readImuRun:(string)line;
//-(void)readImageRun:(string)line;
//-(void)RunFromFileSLAM:(ifstream&)fin_img:(ifstream&)fin_imu;

- (void)networkAuthStatus;
-(void)loop_thread_continue;
-(void)loop_thread_continue2;
@end

