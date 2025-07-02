//
//  ViewController.m
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#import "ViewController.h"
#import "utility.hpp"
#import "CameraUtils.h"
#import "OptimizeRelativePose.hpp"
#import "ViewController+MetalRendering.h"


@interface ViewController ()
@property (weak, nonatomic) IBOutlet UILabel *X_label;
@property (weak, nonatomic) IBOutlet UILabel *Y_label;
@property (weak, nonatomic) IBOutlet UILabel *Z_label;
@property (weak, nonatomic) IBOutlet UILabel *buf_label;
@property (weak, nonatomic) IBOutlet UILabel *total_odom_label;
@property (weak, nonatomic) IBOutlet UILabel *loop_label;
@property (weak, nonatomic) IBOutlet UILabel *feature_label;
@property (weak, nonatomic) IBOutlet UILabel *feature_label2;
@property (weak, nonatomic) IBOutlet UILabel *feature_label3;
@property (weak, nonatomic) IBOutlet UISlider *fovSlider;
@property (weak, nonatomic) IBOutlet UILabel *fovLabel;
@end

@implementation ViewController

/*************************** Save data for debug ***************************/
//ljl
long des_sum=0;

bool start_record = false  ;

bool start_record_2 = false  ;//录第2种数据

bool start_playback = false   ;//跑手机录的数据

bool start_playback_dataEuroc =true  ;

bool start_playback_dataKitti0930 = false ;

bool start_playback_mvsec = false;

bool start_playback_vins = false;

bool isOurAR=false;//这个不需要改

unsigned long imageDataIndex = 0;

unsigned long imageDataReadIndex = 0;

unsigned long imuDataIndex = 0;

unsigned long imuDataReadIndex = 0;

unsigned long vinsDataIndex = 0;

unsigned long vinsDataReadIndex = 0;

queue<IMG_DATA> imgDataBuf;

NSMutableData *imuDataBuf = [[NSMutableData alloc] init];

NSData *imuReader;

NSMutableData *vinsDataBuf = [[NSMutableData alloc] init];

NSData *vinsReader;

static IMG_DATA imgData;

static IMU_MSG imuData;

KEYFRAME_DATA vinsData;

/*************************** Save data for debug ***************************/

/******************************* UI CONFIG *******************************/

// false:  VINS trajectory is the main view, AR image is in left bottom
// true: AR image is the main view, VINS is in left bottom
bool ui_main = false;

bool box_in_AR = false;

bool box_in_trajectory = false;

// If initialized finished, start show is true
bool start_show = false;

// Indicate the initialization progress rate
UIActivityIndicatorView *indicator;

// Used for show VINS trajectory and AR
@synthesize imageView;

// Used for show initialization UI
@synthesize featureImageView;

@synthesize videoCamera;

// Used for show alert if vocabulary is not ready
UIAlertView *alertView;

// Textview for showing vins status
int loop_old_index = -1;

float x_view_last = -5000;

float y_view_last = -5000;

float z_view_last = -5000;

float total_odom = 0;

/******************************* UI CONFIG *******************************/

FeatureTracker featuretracker;

VINS vins;

// Store the fesature data processed by featuretracker
queue<ImgConstPtr> img_msg_buf;

// Store the IMU data for vins
queue<ImuConstPtr> imu_msg_buf;

// Store the IMU data for motion-only vins
queue<IMU_MSG_LOCAL> local_imu_msg_buf;

// The number of measurements waiting to be processed
int waiting_lists = 0;

int frame_cnt = 0;

// Lock the feature and imu data buffer
std::mutex m_buf;

std::condition_variable con;

NSTimeInterval current_time = -1;

NSTimeInterval lateast_imu_time = -1;

int imu_prepare = 0;

// MotionManager for read imu data
CMMotionManager *motionManager;

// Segment the trajectory using color when re-initialize
int segmentation_index = 0;

// Set true:  30 HZ pose output and AR rendering in front-end (very low latency)
// Set false: 10 HZ pose output and AR rendering in back-end
bool USE_PNP = false;

// Lock the solved VINS data feedback to featuretracker
std::mutex m_depth_feedback;

// Lock the IMU data feedback to featuretracker
std::mutex m_imu_feedback;

// Solved VINS feature feedback to featuretracker
list<IMG_MSG_LOCAL> solved_features;//特征点3D坐标乘了尺度的，也计算了漂移 暂时不要乘以尺度漂移（不然会换到另一个坐标系中）

// Solved VINS status feedback to featuretracker
VINS_RESULT solved_vins;

/******************************* Loop Closure ******************************/

// Raw image data buffer for extracting FAST feature
queue<pair<cv::Mat, double>> image_buf_loop;

// Lock the image_buf_loop
std::mutex m_image_buf_loop;

// Detect loop
LoopClosure *loop_closure;

// Keyframe database
KeyFrameDatabase keyframe_database;

// Control the loop detection frequency
int keyframe_freq = 0;

// Index the keyframe
int global_frame_cnt = 0;//回环检测的关键帧数目

// Record the checked loop frame
int loop_check_cnt = 0;

// Indicate if breif vocabulary read finish
bool voc_init_ok = false;

// Indicate the loop frame index
int old_index = -1;

// Translation drift
Eigen::Vector3d loop_correct_t = Eigen::Vector3d(0, 0, 0);//回环之后计算得到的漂移

// Rotation drift
Eigen::Matrix3d loop_correct_r = Eigen::Matrix3d::Identity();

//ljl
KeyFrame *old_kf_priorMap, *curLoopKf_priorMap;

/******************************* Loop Closure ******************************/

// MARK: Unity Camera Mode Switching
// Ground truth from UI switch property "self.switchUIAREnabled"

// Implied, updated by updateCameraMode()
bool imuPredictEnabled = false;

// Implied, updated by updateCameraMode()
bool cameraMode = true;

// Implied, updated by updateCameraMode()
bool imageCacheEnabled = cameraMode && !USE_PNP;

//ljl
int imu_cnt=-1;//两帧imu才处理一次

ifstream fin_imu_euroc;
ifstream fin_img_euroc;
string imgName;
string imuPath;

NSArray *_allLinedStrings_imgTime;
NSArray *_allLinedStrings_imuTime;
NSString *documentsPath_kf;


int count_img=0;

//实验用
//double cur_kf_index=-1;

//测试
int triangulation_num=0;

// MARK: ViewController Methods

#pragma mark - 用户方法,将输出信息写入到dr.log文件中；
// 将打印信息保存到Document目录下的文件中
- (void)redirectNSlogToDocumentFolder
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentDirectory = [paths objectAtIndex:0];
    NSString *fileName = [NSString stringWithFormat:@"dr.log"];// 注意不是NSData!
    NSString *logFilePath = [documentDirectory stringByAppendingPathComponent:fileName];
    // 先删除已经存在的文件
    NSFileManager *defaultManager = [NSFileManager defaultManager];
    [defaultManager removeItemAtPath:logFilePath error:nil];
      
    // 将log输入到文件
//    freopen([logFilePath cStringUsingEncoding:NSASCIIStringEncoding], "a+", stdout);
    freopen([logFilePath cStringUsingEncoding:NSASCIIStringEncoding], "a+", stderr);
}

- (void)networkAuthStatus {
    CTCellularData *cellularData = [[CTCellularData alloc]init];
    cellularData.cellularDataRestrictionDidUpdateNotifier = ^(CTCellularDataRestrictedState state) {
        if (state == kCTCellularDataRestricted) {
            printf("网络请求连接拒绝\n");
            //拒绝
            [self networkSettingAlert];
        } else if (state == kCTCellularDataNotRestricted) {
            printf("网络请求连接允许\n");
            //允许
            
        } else {
            printf("网络请求连接未知\n");
            //未知
            [self unknownNetwork];
        }
    };
}

- (void)networkSettingAlert {
    dispatch_async(dispatch_get_main_queue(), ^{
        UIAlertController *alertController = [UIAlertController alertControllerWithTitle:@"提示" message:@"您尚未授权“app”访问网络的权限，请前往设置开启网络授权" preferredStyle:UIAlertControllerStyleAlert];
        
        [alertController addAction:[UIAlertAction actionWithTitle:@"取消" style:UIAlertActionStyleDestructive handler:^(UIAlertAction * _Nonnull action) {
            
        }]];
        
        [alertController addAction:[UIAlertAction actionWithTitle:@"去设置" style:UIAlertActionStyleDefault handler:^(UIAlertAction * _Nonnull action) {
            [[UIApplication sharedApplication] openURL:[NSURL URLWithString:UIApplicationOpenSettingsURLString] options:@{} completionHandler:nil];
        }]];
        
        [self presentViewController:alertController animated:YES completion:nil];
//        [self presentViewController:alertDevice animated:YES completion:nil];
    });
}

- (void)unknownNetwork {
    dispatch_async(dispatch_get_main_queue(), ^{
        UIAlertController *alertController = [UIAlertController alertControllerWithTitle:@"提示" message:@"未知网络" preferredStyle:UIAlertControllerStyleAlert];
        
        [alertController addAction:[UIAlertAction actionWithTitle:@"确定" style:UIAlertActionStyleDefault handler:^(UIAlertAction * _Nonnull action) {
        }]];
        
        [self presentViewController:alertController animated:YES completion:nil];
    });
}
- (void)viewDidLoad {
    [super viewDidLoad];
    [self redirectNSlogToDocumentFolder];
    /*******************************************Camera setup*******************************************/
    self.videoCamera = [[CvVideoCamera alloc]
                        initWithParentView:imageView];

    self.videoCamera.delegate = self;
    //后置摄像头
    self.videoCamera.defaultAVCaptureDevicePosition =
    AVCaptureDevicePositionBack;
    //相机方向为纵向
    self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait;
    //设置采集质量
    self.videoCamera.defaultAVCaptureSessionPreset =
    AVCaptureSessionPreset640x480;
#ifdef DATA_EXPORT
    self.videoCamera.defaultFPS = 1;
#else
    self.videoCamera.defaultFPS =10;//30 2 10
#endif

    isCapturing = NO;

    [CameraUtils setExposureOffset: -1.0f];

 
    //跑euroc数据
    if(start_playback_dataEuroc){
        //这里是根据文件名 先得到存储数据的路径， 文件里面的内容对应着要读取的数据
        string imgPath=[[[NSBundle mainBundle]pathForResource:@"v2_03_img" ofType:@"csv"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
        imgName_Path=imgPath;
        imgName_Path.erase(imgName_Path.end()-13,imgName_Path.end());
//        fin_img_euroc.open(imgPath);//一次一次的读

        NSString *filePath_imgTime=[NSString stringWithCString:imgPath.c_str() encoding:[NSString defaultCStringEncoding]];

        NSError *error = nil;
        unsigned long encode = CFStringConvertEncodingToNSStringEncoding(kCFStringEncodingGB_18030_2000);
        NSString *fileContents_imgTime = [NSString stringWithContentsOfFile:filePath_imgTime encoding:encode error:&error];
        //取出每一行的image+time数据
        _allLinedStrings_imgTime = [fileContents_imgTime componentsSeparatedByString:@"\n"];
        count_img=_allLinedStrings_imgTime.count;
        count_img--;

        imuPath=[[[NSBundle mainBundle]pathForResource:@"v2_03_imu" ofType:@"csv"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
        //这里是一次读一行 但是流从来没关过
//        fin_imu_euroc.open(imuPath);
        //改成通过一次性读取出来存储
        NSString *filePath=[NSString stringWithCString:imuPath.c_str() encoding:[NSString defaultCStringEncoding]];
        NSString *fileContents = [NSString stringWithContentsOfFile:filePath encoding:encode error:&error];
//        //取出每一行的imu+time数据
        _allLinedStrings_imuTime = [fileContents componentsSeparatedByString:@"\n"];

        //一次性读一行，然后关掉流
    }
    else if(start_playback_dataKitti0930){
        //因为要做图片和imu的对齐，所以不能直接起始点为0  还是要不上最开始的时间戳
        //因为图片的文件名和 时间戳文件里名字不一样，是递增的，而且前面存在0 ，可以定义一个全局变量 不停++ 并且补0
        //imu数据 一开始的获得 和图片同理 然后再选取一些
            string imgPath=[[[NSBundle mainBundle]pathForResource:@"img10" ofType:@"txt"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
            imgName_Path=imgPath;
            imgName_Path.erase(imgName_Path.end()-9,imgName_Path.end());
    //        fin_img_euroc.open(imgPath);//一次一次的读

            NSString *filePath_imgTime=[NSString stringWithCString:imgPath.c_str() encoding:[NSString defaultCStringEncoding]];

            NSError *error = nil;
            unsigned long encode = CFStringConvertEncodingToNSStringEncoding(kCFStringEncodingGB_18030_2000);
            NSString *fileContents_imgTime = [NSString stringWithContentsOfFile:filePath_imgTime encoding:encode error:&error];
            //取出每一行的image+time数据
            _allLinedStrings_imgTime = [fileContents_imgTime componentsSeparatedByString:@"\n"];
            count_img=_allLinedStrings_imgTime.count;
            count_img--;



            imuPath=[[[NSBundle mainBundle]pathForResource:@"imu10" ofType:@"txt"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
            //这里是一次读一行 但是流从来没关过
    //        fin_imu_euroc.open(imuPath);
            //改成通过一次性读取出来存储
            NSString *filePath=[NSString stringWithCString:imuPath.c_str() encoding:[NSString defaultCStringEncoding]];
            NSString *fileContents = [NSString stringWithContentsOfFile:filePath encoding:encode error:&error];
    //        //取出每一行的imu+time数据
            _allLinedStrings_imuTime = [fileContents componentsSeparatedByString:@"\n"];

            //一次性读一行，然后关掉流


        }
   else if(start_playback_mvsec){
        //这里是根据文件名 先得到存储数据的路径， 文件里面的内容对应着要读取的数据
        string imgPath=[[[NSBundle mainBundle]pathForResource:@"imgNight3" ofType:@"txt"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
        imgName_Path=imgPath;
        imgName_Path.erase(imgName_Path.end()-13,imgName_Path.end());
//        fin_img_euroc.open(imgPath);//一次一次的读

        NSString *filePath_imgTime=[NSString stringWithCString:imgPath.c_str() encoding:[NSString defaultCStringEncoding]];

        NSError *error = nil;
        unsigned long encode = CFStringConvertEncodingToNSStringEncoding(kCFStringEncodingGB_18030_2000);
        NSString *fileContents_imgTime = [NSString stringWithContentsOfFile:filePath_imgTime encoding:encode error:&error];
        //取出每一行的image+time数据
        _allLinedStrings_imgTime = [fileContents_imgTime componentsSeparatedByString:@"\n"];
        count_img=_allLinedStrings_imgTime.count;
        count_img--;



        imuPath=[[[NSBundle mainBundle]pathForResource:@"imuNight3" ofType:@"csv"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
        //这里是一次读一行 但是流从来没关过
//        fin_imu_euroc.open(imuPath);
        //改成通过一次性读取出来存储
        NSString *filePath=[NSString stringWithCString:imuPath.c_str() encoding:[NSString defaultCStringEncoding]];
        NSString *fileContents = [NSString stringWithContentsOfFile:filePath encoding:encode error:&error];
//        //取出每一行的imu+time数据
        _allLinedStrings_imuTime = [fileContents componentsSeparatedByString:@"\n"];

        //一次性读一行，然后关掉流
    }

    [self imuStartUpdate];
//    printf("开始跑数据 ********* 开始录数据\n");
    [videoCamera start];//对应处理方法 processImage


    /***************************************UI configuration*****************************************/
    UIPanGestureRecognizer *resultPanGestureRecognizer = [[UIPanGestureRecognizer alloc]
                                                          initWithTarget:self
                                                          action:@selector(handlePan:)];
    resultPanGestureRecognizer.minimumNumberOfTouches = 1;
    resultPanGestureRecognizer.maximumNumberOfTouches = 2;
    [self.imageView addGestureRecognizer:resultPanGestureRecognizer];

    UIPinchGestureRecognizer *resultPinchGestureRecognizer = [[UIPinchGestureRecognizer alloc]
                                                              initWithTarget:self
                                                              action:@selector(handlePinch:)];
    [self.imageView addGestureRecognizer:resultPinchGestureRecognizer];

    UITapGestureRecognizer *resultTapGestureRecognizer = [[UITapGestureRecognizer alloc]
                                                          initWithTarget:self
                                                          action:@selector(handleTap:)];
    [self.imageView addGestureRecognizer:resultTapGestureRecognizer];

    UILongPressGestureRecognizer *resultLongPressGestureRecognizer = [[UILongPressGestureRecognizer alloc]
                                                                      initWithTarget:self
                                                                      action:@selector(handleLongPress:)];
    [self.imageView addGestureRecognizer:resultLongPressGestureRecognizer];

    if (!feature_tracker){
        feature_tracker = new FeatureTracker();
    }
    if(start_playback_dataEuroc){
        feature_tracker->start_playback_dataEuroc=true;
        featuretracker.start_playback_dataEuroc=true;
    }
    if(start_playback_dataKitti0930){
        feature_tracker->start_playback_dataKitti0930=true;
        featuretracker.start_playback_dataKitti0930=true;
    }
    if(start_playback_mvsec){
        feature_tracker->start_playback_mvsec=true;
        featuretracker.start_playback_mvsec=true;
    }
    //give projection variance
    vins.setIMUModel();
    vins.setFeature_tracker(feature_tracker);

    //UI
    _loopButton.layer.zPosition = 1;
    _reinitButton.layer.zPosition = 1;
    alertView = [[UIAlertView alloc]initWithTitle:@"WARN" message:@"please wait for vocabulary loading!"
                                         delegate:self cancelButtonTitle:@"confirm" otherButtonTitles:@"cancel", nil];

    indicator = [[UIActivityIndicatorView alloc] initWithActivityIndicatorStyle:UIActivityIndicatorViewStyleWhiteLarge];
    indicator.center = CGPointMake(self.imageView.frame.size.width * 0.5, self.imageView.frame.size.height * 0.22);
    indicator.color = [UIColor darkGrayColor];
    [indicator startAnimating];
    [self.view addSubview:indicator];

    
    
    
    /****************************************Init all the thread****************************************/
    _condition=[[NSCondition alloc] init];
    mainLoop=[[NSThread alloc]initWithTarget:self selector:@selector(run) object:nil];
    [mainLoop setName:@"mainLoop"];

    if(start_record){
        saveData=[[NSThread alloc]initWithTarget:self selector:@selector(saveData) object:nil];
        [saveData setName:@"saveData"];
    }
    if(start_record_2){
        saveData_2=[[NSThread alloc]initWithTarget:self selector:@selector(saveData_2) object:nil];
        [saveData_2 setName:@"saveData_2"];
    }



    if(LOOP_CLOSURE)
    {
        //loop closure thread
        loop_thread = [[NSThread alloc]initWithTarget:self selector:@selector(loop_thread2) object:nil];
        [loop_thread setName:@"loop_thread2"];
        [loop_thread start];

        globalLoopThread=[[NSThread alloc]initWithTarget:self selector:@selector(globalLoopThread) object:nil];
        [globalLoopThread setName:@"globalLoopThread"];
        [globalLoopThread start];
    }
    else if(LOOP_CLOSURE_SERVER || LOOP_CLOSURE_SERVER_noLoop){
        //loop closure thread 这个只是用来算描述子
        loop_thread_server = [[NSThread alloc]initWithTarget:self selector:@selector(loop_thread_server) object:nil];
        [loop_thread_server setName:@"loop_thread_server"];
        [loop_thread_server start];
        //服务器那边告诉我漂移 我这边更新位姿
        globalLoopThread_server=[[NSThread alloc]initWithTarget:self selector:@selector(globalLoopThread_server) object:nil];
        [globalLoopThread_server setName:@"globalLoopThread_server"];
        [globalLoopThread_server start];


        [self connectServer];

        [SocketSingleton sharedInstance].kfdb=&keyframe_database;
        [SocketSingleton sharedInstance].vins=&vins;
        [SocketSingleton sharedInstance].featureTracker=&featuretracker;
//        [SocketSingleton sharedInstance].old_kf_priorMap=old_kf_priorMap;
//        [SocketSingleton sharedInstance].curLoopKf_priorMap=curLoopKf_priorMap;


    }

    /************************************Device and iOS version check************************************/
//    DeviceType deviceType=deviceName();
    DeviceType deviceType= euroc;
    bool deviceCheck = setGlobalParam(deviceType);
    if(!deviceCheck)
    {
        UIAlertController *alertDevice = [UIAlertController alertControllerWithTitle:@"Unsupported Device"
                                                                             message:@"Supported devices are: iPhone7 Plus, iPhone7, iPhone6s Plus, iPhone6s" preferredStyle:UIAlertControllerStyleAlert];

        UIAlertAction *okAction = [UIAlertAction
                                   actionWithTitle:@"OK"
                                   style:UIAlertActionStyleDefault
                                   handler:^(UIAlertAction * _Nonnull action) {

                                   }];

        [alertDevice addAction:okAction];

        dispatch_async(dispatch_get_main_queue(), ^ {
            [self presentViewController:alertDevice animated:YES completion:nil];
        });
    }
    vins.setExtrinsic();
    vins.setIMUModel();
    featuretracker.vins_pnp.setExtrinsic();
    featuretracker.vins_pnp.setIMUModel();
    //去畸变
    featuretracker.readIntrinsicParameter();
    featuretracker.priorMapFeature=new PriorMapFeature();
    featuretracker.priorMapFeature->setParameter();
    bool versionCheck = iosVersion();
    if(!versionCheck)
    {
        UIAlertController *alertVersion = [UIAlertController alertControllerWithTitle:@"Warn"
                                                                              message:@"Please upgrade your iOS version!" preferredStyle:UIAlertControllerStyleAlert];
        UIAlertAction *cancelAction = [UIAlertAction actionWithTitle:@"Cancel" style:UIAlertActionStyleCancel handler:^(UIAlertAction * action)
                                       {exit(0);}];
        UIAlertAction *okAction = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action)
                                   {exit(0);}];
        [alertVersion addAction:cancelAction];
        [alertVersion addAction:okAction];
        dispatch_async(dispatch_get_main_queue(), ^ {
            [self presentViewController:alertVersion animated:YES completion:nil];
        });
    }


    /*********************************************Start VINS*******************************************/
    if(versionCheck && deviceCheck)
    {


        if(LOOP_CLOSURE_SERVER || LOOP_CLOSURE_SERVER_noLoop){
            sendCameraParam(deviceType);
        }



        isCapturing = YES;
        [mainLoop start];
        motionManager = [[CMMotionManager alloc] init];
        if(start_playback_dataEuroc){
            frameSize=cv::Size(752,480);
        }
        else if(start_playback_dataKitti0930){
            frameSize=cv::Size(1392,512);
        }
        else if(start_playback_mvsec){
            frameSize=cv::Size(752,480);
        }
        else{
            frameSize = cv::Size(videoCamera.imageWidth,
                             videoCamera.imageHeight);
        }


        //这里是手机记录数据
        if(start_record){
            [saveData start];
        }
        if(start_record_2){
            [saveData_2 start];
        }
        //这里跑的是手机记录的数据
         if(start_playback)
         {
             TS(read_imu);
             NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
             NSString *documentsPath = [paths objectAtIndex:0];
             NSString *filePath = [documentsPath stringByAppendingPathComponent:@"IMU"]; //Add the file name
             imuReader = [NSData dataWithContentsOfFile:filePath];
             TE(read_imu);
         }

    }

    //保存每一个kf的信息
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    documentsPath_kf = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"KF"]; //Get the docs directory
    [self checkDirectoryPath:0 withObject:documentsPath_kf];

    //    发送网络请求
        [self networkAuthStatus];

}

vector<KeyFrame*> mvpLocalKeyFrames;
list<Feature_PriorMap> feature_local;
//寻找匹配点：直接投影，找匹配点（后续的优化中，标记这个点的3d位置不变）
void localMapping(int oldKf_id,int forw_id,int curLoopKf_id,double loop_pose[7]) {
    //这里是回环传进来的
    //这里是当前正在处理的帧
    cout<<"老帧id"<<oldKf_id<<" , 当前的回环帧id"<<curLoopKf_id<<" , 当前帧id"<<forw_id<<endl;
    KeyFrame* old_kf=keyframe_database.getKeyframe(oldKf_id);
//    得到子地图的帧（和当前位置相关的）
    UpdateLocalKeyFrames(old_kf,forw_id);
    cout<<"找到的共视帧的数量"<<mvpLocalKeyFrames.size() <<endl;
//    得到当前位置这片区域的地图点
    UpdateLocalPoints();
    cout<<"找到的3d点的数量"<<feature_local.size() <<endl;
//    投影到当前帧【如何确定是哪一帧呢-featuretracker正在处理的那一帧 找到像素点位置
//    TrackWithRt();
    KeyFrame* cur_kf=keyframe_database.getLastKeyframe_index(forw_id);
    KeyFrame* curLoopKf=keyframe_database.getLastKeyframe_index(curLoopKf_id);
    cout<<std::setprecision(16)<<"老帧的时间戳"<<old_kf->header<<" , 当前帧的时间戳"<<cur_kf->header<<endl;
    cout<<"是否提取特征点了："<<cur_kf->features_id_origin.size()<<" , "<<cur_kf->keypoints.size()<<endl;
    SearchByProjection( cur_kf,curLoopKf,loop_pose);
//    光流法跟踪当前帧 这些新增的像素点
//    featuretracker.addPoints();
}

void UpdateLocalKeyFrames(KeyFrame* oldKf,int curKf_id)
{
//    KeyFrame* oldKf=getKeyframe(oldKf_id);
    mvpLocalKeyFrames.clear();
//    策略1：得到回环帧后面的共视帧
    mvpLocalKeyFrames=oldKf->GetBestCovisibilityKeyFrames(20);//因为只要后面的 所以其实只要了一半20/2=6
    
//    cout<<"mvpLocalKeyFrames.size="<<mvpLocalKeyFrames.size()<<endl;
    int oldkf_id=oldKf->global_index;
    int real_oldVisibleKf_num=0;
    for(int i=0,j=mvpLocalKeyFrames.size();i<j;i++){
        int kf_id=mvpLocalKeyFrames[i]->global_index;
        if(kf_id>=oldkf_id){
            mvpLocalKeyFrames[real_oldVisibleKf_num]=mvpLocalKeyFrames[i];
            mvpLocalKeyFrames[real_oldVisibleKf_num]->mnTrackReferenceForFrame=curKf_id;
            real_oldVisibleKf_num++;
        }
    }
    mvpLocalKeyFrames.resize(real_oldVisibleKf_num);
    
    cout<<"real_oldVisibleKf_num="<< real_oldVisibleKf_num<<endl;
    
    // 策略2：与策略1得到的局部关键帧共视程度很高的关键帧作为局部关键帧
    for(int i=0;i<real_oldVisibleKf_num;i++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80) // 80
            break;

        KeyFrame* pKF = mvpLocalKeyFrames[i];

        // 策略2.1:最佳共视的10帧
        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        // 添加一个就break
        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            
            // 防止重复添加
            if(pNeighKF->mnTrackReferenceForFrame!=curKf_id)
            {
                mvpLocalKeyFrames.push_back(pNeighKF);
                pNeighKF->mnTrackReferenceForFrame=curKf_id;
                cout<<pNeighKF->global_index<<" , ";
                break;
            }
            
        }
    }

}

void UpdateLocalPoints()
{
    // 步骤1：清空局部MapPoints
    feature_local.clear();
    vector<int> v_feature;
// 步骤2：遍历局部关键帧mvpLocalKeyFrames
    for(vector<KeyFrame*>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
    {
        KeyFrame* pKF = *itKF;
        std::vector<Eigen::Vector3d> vpMPs = pKF->point_clouds_origin;
        std::vector<int> features_id_origin = pKF->features_id_origin;
        std::vector<cv::Point2f> measurements_origin= pKF->measurements_origin;//后续需要剪切
        std::vector<BRIEF::bitset> des_origin= pKF->descriptors;//后续需要剪切
        std::vector<cv::Point2f> measurements_origin_distorted= pKF->distorted_measurements_origin_uv;
        
        int keypoint_len=des_origin.size()-features_id_origin.size();
        
//        Eigen::Vector3d t_cur;
//        Eigen::Matrix3d r_cur;
//        pKF->getOriginPose(t_cur, r_cur);
//        Eigen::Matrix3d r_wc=r_cur*vins.ric;
//        Eigen::Vector3d t_wc=r_cur*vins.tic+t_cur;
//        r_cur=r_wc.transpose();
//        t_cur=-r_cur*t_cur;

        //查找featureid 判断是否已经加入该3d点
        for(int i=0,j=vpMPs.size();i<j;i++)
        {
            
//            先查找这个id是否加入过
            int featureId_cur=features_id_origin[i];
            vector<int>::iterator iter;
            iter = find(v_feature.begin(), v_feature.end(), featureId_cur);
            if(iter!= v_feature.end()){
                continue;
            }
//            存储3d点
            Eigen::Vector3d pts_i = vpMPs[i];//世界坐标系下的点
//            pts_i=r_cur*pts_i+t_cur;//相机坐标系下点
//            FeaturePerFrame f_per_fra(pts_i/pts_i[2]);
//            f_manager.feature.push_back(FeaturePerId(features_id_origin[i], 被哪一帧看到)); //give id and start frame
//            f_manager.feature.back().feature_per_frame.push_back(f_per_fra);    //give point
           //深度存储
            
//            存储到feature_local中
            Feature_PriorMap f_loc(featureId_cur);
            f_loc.des=des_origin[keypoint_len+i];
            f_loc.point=pts_i;
            
            f_loc.measurements_coarse =measurements_origin_distorted[i];
            f_loc.measurements_coarse_undistorted=measurements_origin[i];
            f_loc.header=pKF->header;
            
            feature_local.push_back(f_loc);
            
            v_feature.push_back(featureId_cur);
        }
    }
    featuretracker.priorMapFeature->feature_local=feature_local;
    assert(featuretracker.priorMapFeature->feature_local.size()==feature_local.size());
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

int SearchByProjection(KeyFrame* cur_kf,KeyFrame* curLoopKf,double loop_pose[7])
{
    int nmatches = 0;
    int HISTO_LENGTH=30;
    float th=20;
    int TH_HIGH=100;
    
    Eigen::Matrix3d ric_curClient= vins.ric;
    Eigen::Vector3d tic_curClient= vins.tic;
    const float &fx = FOCUS_LENGTH_X;
    const float &fy = FOCUS_LENGTH_Y;
    const float &cx = PX;
    const float &cy = PY;
    
//    到当前的回环帧的相对位姿
    Eigen::Vector3d t_w2_curLoop(loop_pose[0],loop_pose[1],loop_pose[2]);
    Eigen::Quaterniond q_w2_curLoop(loop_pose[6],loop_pose[3],loop_pose[4],loop_pose[5]);
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
    
    std::vector<cv::Point2f> measurements_cur_coarse;//像素坐标
//    std::vector<cv::Point2f> measurements_cur_norm_coarse;//图像坐标
    std::vector<Eigen::Vector3d> point_3d_old;
//    std::vector<cv::Point2f> measurements_old;//像素坐标
    std::vector<int> feature_id_cur;
    
//    用于实验 看点对匹配是否正确
    std::vector<cv::Point2f> measurements_cur_coarse_experiment;
    std::vector<cv::Point2f> measurements_old_coarse_experiment;
    std::vector<cv::KeyPoint> keypoints_cur_coarse_all=cur_kf->keypoints_distorted;
    std::vector<cv::Point2f> measurements_cur_coarse_experiment_undistorted;
    std::vector<cv::Point2f> measurements_old_coarse_experiment_undistorted;
    std::vector<double> header_all;
    
    vector<cv::KeyPoint> keypoints_cur=cur_kf->keypoints;
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
        if(!cur_kf->isInImage((float)x, (float)y)){
            continue;
        }
        
        // Search in a window. Size depends on scale
        float radius = th;//CurrentFrame.mvScaleFactors[nLastOctave]

        const vector<int> vIndices = cur_kf->GetFeaturesInArea_1((float)x,(float)y, radius);
        if(vIndices.empty())
            continue;
        findPossiblePoint++;
        BRIEF::bitset point_old_des = it_per_id.des;
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
            measurements_old_coarse_experiment.push_back(it_per_id.measurements_coarse);
            measurements_cur_coarse_experiment.push_back(keypoints_cur_coarse_all[bestIdx2].pt);
            
            measurements_old_coarse_experiment_undistorted.push_back(it_per_id.measurements_coarse_undistorted);
            measurements_cur_coarse_experiment_undistorted.push_back(keypoints_cur[bestIdx2].pt);
            header_all.push_back(it_per_id.header);
        }
        
    }

    RetriveData_localMapping retrive_data_localMapping;
    vins.q_old_3d_mutex.lock();
    double rt_double[7];
    for(int i=0;i<3;i++){
        retrive_data_localMapping.loop_pose_forSlideWindow[i]=t_wOld_cur[i];
    }
    Eigen::Quaterniond q(r_wOld_cur);
    retrive_data_localMapping.loop_pose_forSlideWindow[3]=q.x();
    retrive_data_localMapping.loop_pose_forSlideWindow[4]=q.y();
    retrive_data_localMapping.loop_pose_forSlideWindow[5]=q.z();
    retrive_data_localMapping.loop_pose_forSlideWindow[6]=q.w();
    retrive_data_localMapping.header_cur=cur_kf->header;
    retrive_data_localMapping.measurements_cur_norm=measurements_cur_coarse;
    retrive_data_localMapping.point_3d_old=point_3d_old;
    retrive_data_localMapping.features_ids_cur=feature_id_cur;
//    retrive_data_localMapping.loop_pose_forSlideWindow=rt_double;
    vins.retrive_pose_data_localMapping.push(retrive_data_localMapping);
    vins.q_old_3d_mutex.unlock();
    
    cout<<"投影区域找到的可能点对数量="<<findPossiblePoint<<endl;
    cout<<"测试每次最终能找到多少投影的点对="<<nmatches<<endl;
   
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
    return nmatches;//这里返回的是 匹配的总数量 后续应该做一个判断，总数量够不够
}
/*
 Main process image thread: this thread detects and track feature between two continuous images
 and takes the newest VINS result and the corresponding image to draw AR and trajectory.
 */
queue<IMG_DATA_CACHE> image_pool;
queue<VINS_DATA_CACHE> vins_pool;
IMG_DATA_CACHE image_data_cache;
cv::Mat lateast_equa;
UIImage *lateast_image;
Eigen::Vector3f lateast_P;
Eigen::Matrix3f lateast_R;

cv::Mat pnp_image;
Eigen::Vector3d pnp_P;
Eigen::Matrix3d pnp_R;

int img_num=0;




//bool m_imu_lock=false;
//bool m_buf_lock=false;
bool sum_img=false;

- (void)processImage:(cv::Mat&)image
{
//    @autoreleasepool {


    if(isCapturing == YES)
    {
        float lowPart = image.at<float>(0,0);  //modify opencv library, timestamp was stored at index 0,0
        float highPart = image.at<float>(0,1);
        //image.at<float>(0,0) = image.at<float>(1,0);
        //image.at<float>(0,1) = image.at<float>(1,1);
        shared_ptr<IMG_MSG> img_msg(new IMG_MSG());
        //cout << (videoCamera->grayscaleMode) << endl;
        //img_msg->header = [[NSDate date] timeIntervalSince1970];
        img_msg->header = [[NSProcessInfo processInfo] systemUptime];
        float Group[2];
        Group[0] = lowPart;
        Group[1] = highPart;
        double* time_now_decode = (double*)Group;
        double time_stamp = *time_now_decode;

        if(lateast_imu_time <= 0)
        {
            cv::cvtColor(image, image, CV_BGRA2RGB);
            cv::flip(image,image,-1);//上下翻转
            return;
        }
        //img_msg->header = lateast_imu_time;
        img_msg->header = time_stamp;
        BOOL isNeedRotation = image.size() != frameSize;

        //for save data
        cv::Mat input_frame;
        if(start_playback)
        {
            //TS(readImg);
            bool still_play;
            still_play = [self readImageTime:imageDataReadIndex];
            [self readImage:imageDataReadIndex];
            if(!still_play){
                TS(sleep_end);
//                usleep(8000);
                [NSThread sleepForTimeInterval:30];
                TE(sleep_end);
                return;
            }
            imageDataReadIndex++;
#ifdef DATA_EXPORT
            [self tapSaveImageToIphone:imgData.image];
#endif


            UIImageToMat(imgData.image,image);
            UIImageToMat(imgData.image,input_frame);
            imgData.image=nil;


            img_msg->header = imgData.header;
            //TE(readImg);
#ifdef DATA_EXPORT
            printf("record play image: %lf\n",imgData.header,imageDataReadIndex);
#endif
//            printf("record play image: %lf , %ld\n",imgData.header,imageDataReadIndex);
        }
        else if(start_playback_dataEuroc){
            //实验部分 第一次从0-1500 ， 第二次从900-1742
            //1500 1742
            //跑全部count_img
            //MH_01 0-2200 2100-3682
            //MH_01 0-2120 2020-
            //MH_03 0-1257 1258-2700
            //MH_03 0-1505 1405-2700
            //MH_03 0-1305 1205-2700
            //MH_04 0-1502 1360-2033
            //MH_02 0-1800 1700-3040 0.075
            //MH_02 0-2200 2100-3040 0.055
            //MH_05 0-1100 900-


            if(imageDataReadIndex==count_img){
//                cout<<"关键帧的总数量："<<keyframe_database.size()<<endl;
//                cout<<"图片的总数量："<<count_img+1<<endl;
//                cout<<"描述符的总数量："<<des_sum<<endl;
//                cout<<"汇总特征点数量：";
//                for(int a=0,b=keys_sum.size();a<b;a++){
//                    cout<<keys_sum[a]<<" , ";
//                }
//                cout<<endl<<endl;
//                cout<<"汇总原生态bits：";
//                for(int a=0,b=bits_nature.size();a<b;a++){
//                    cout<<bits_nature[a]<<" , ";
//                }
//                cout<<endl<<endl;
//                cout<<"汇总压缩bits：";
//                for(int a=0,b=bits_encode.size();a<b;a++){
//                    cout<<bits_encode[a]<<" , ";
//                }
//                cout<<endl<<endl;
                
//                cout<<"汇总总byte：";
//                for(int a=0,b=bytes_sum_keyframe.size();a<b;a++){
//                    cout<<bytes_sum_keyframe[a]<<" , ";
//                }
//                cout<<endl<<endl;
//                cout<<"汇总总keys：";
//                for(int a=0,b=bytes_keys_keyframe.size();a<b;a++){
//                    cout<<bytes_keys_keyframe[a]<<" , ";
//                }
//                cout<<endl<<endl;
//                cout<<"汇总总des：";
//                for(int a=0,b=bytes_des_keyframe.size();a<b;a++){
//                    cout<<bytes_des_keyframe[a]<<" , ";
//                }
//                cout<<endl<<endl;
                
                
                
//                cout<<"汇总耗时time_compress：";
//                for(int a=0,b=time_compress.size();a<b;a++){
//                    cout<<time_compress[a]<<" , ";
//                }
//                cout<<endl<<endl;
//                cout<<"汇总耗时time_compress_decode：";
//                for(int a=0,b=time_compress_decode.size();a<b;a++){
//                    cout<<time_compress_decode[a]<<" , ";
//                }
//                cout<<endl<<endl;
                
                if(sum_img==false){
                sum_img=true;
                [NSThread sleepForTimeInterval:5];//给80s去把优化做完

                NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
                NSString *documentsPath = [paths objectAtIndex:0];




                NSString *filePath_tum_pose = [documentsPath stringByAppendingPathComponent:@"tum_pose"];
                [self checkFilePath:filePath_tum_pose];

//                [NSThread sleepForTimeInterval:30];
                //这里可能得加个锁 不安全
                NSMutableData *poseDataBuf_time_r_t = [[NSMutableData alloc] init];
                vector<double> time=keyframe_database.path_time;
                vector<Eigen::Vector3f> path=keyframe_database.refine_path;
                vector<Eigen::Quaterniond> pose_r=keyframe_database.refine_r;
                int len=time.size();
                for(int i=0;i<len;i++){
                    [poseDataBuf_time_r_t appendBytes:&(time[i]) length:sizeof(double)];
                    [poseDataBuf_time_r_t appendBytes:&(path[i]) length:sizeof(path[i])];
                    [poseDataBuf_time_r_t appendBytes:&(pose_r[i]) length:sizeof(pose_r[i])];
                }
//                printf("位姿保存完了 验证一下 %f, %f, %f, %f\n",pose_r[0].x(), pose_r[0].y(),pose_r[0].z(),pose_r[0].w());
                BOOL writeImuSuccess_time=[poseDataBuf_time_r_t writeToFile:filePath_tum_pose atomically:YES];
                if(writeImuSuccess_time){
                        NSLog(@"位姿时间戳已保存完整succ");
                }else{
                        NSLog(@"位姿时间戳已保存完整fail");
                }



                NSString *documentsPath_kf = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"KF"]; //Get the docs directory
                [self checkDirectoryPath:0 withObject:documentsPath_kf];
//                Eigen::Matrix3d r;
//                Eigen::Vector3d t;
//                //保存每一帧的信息，每一帧一个文件
//                for(int i=0;i<len;i++){
//                    NSString *filename = [NSString stringWithFormat:@"%lu", i];
//                    NSString *filePath_kf = [documentsPath_kf stringByAppendingPathComponent:filename]; //Add the file name
//                    [self checkFilePath:filePath_kf];
//                    NSMutableData *kf_buf = [[NSMutableData alloc] init];
//
//                    KeyFrame *kf=keyframe_database.getKeyframe(i);
//                    vector<cv::KeyPoint> kf_keypoint=kf->keypoints;
//                    vector<BRIEF::bitset> kf_descriptors=kf->descriptors;
//                    vector<cv::Point2f> kf_measurements_origin=kf->measurements_origin;
//                    vector<Eigen::Vector3d> kf_point_clouds_origin=kf->point_clouds_origin;
//                    vector<int> kf_features_id_origin=kf->features_id_origin;
//                    kf->getPose(t,r);
//                    int kf_id=kf->global_index;
//
//                    [kf_buf appendBytes:&(kf_id) length:sizeof(int)];
//                    [kf_buf appendBytes:&(time[i]) length:sizeof(double)];
//                    for(int a=0;a<3;a++){
//                        [kf_buf appendBytes:&(t[a]) length:sizeof(double)];
//                    }
//
//                    for(int a=0;a<3;a++){
//                        for(int b=0;b<3;b++){
//                            [kf_buf appendBytes:&(r(a,b)) length:sizeof(double)];
//                        }
//                    }
//
//                    int keypoint_len=kf_keypoint.size();
////                    cout<<"keypoint_len="<<keypoint_len<<endl;
////                    cout<<"sizeof(double)"<<sizeof(double)<<endl;
////                    cout<<"sizeof(float)"<<sizeof(float)<<endl;
////                    cout<<"sizeof(int)"<<sizeof(int)<<endl;
////                    cout<<"kf_buf.length"<<kf_buf.length<<endl;
//                    [kf_buf appendBytes:&(keypoint_len) length:sizeof(int)];
//                    for(int a=0;a<keypoint_len;a++){
//                        cv::KeyPoint kf_key=kf_keypoint[a];
//                        [kf_buf appendBytes:&(kf_key.pt.x) length:sizeof(float)];
//                        [kf_buf appendBytes:&(kf_key.pt.y) length:sizeof(float)];
//                        [kf_buf appendBytes:&(kf_key.size) length:sizeof(float)];
//                        [kf_buf appendBytes:&(kf_key.angle) length:sizeof(float)];
//                        [kf_buf appendBytes:&(kf_key.response) length:sizeof(float)];
//                        [kf_buf appendBytes:&(kf_key.octave) length:sizeof(int)];
//                        [kf_buf appendBytes:&(kf_key.class_id) length:sizeof(int)];
////                        cout<<"kf_key.pt.x="<<kf_key.pt.x<<endl;
//                    }
//
//                    for(int a=0;a<keypoint_len;a++){
//                        string str;
//                        to_string(kf_descriptors[a],str);
////                        cout<<"str_des="<<str<<endl;
//                        for(auto ite=str.rbegin(),ite_end=str.rend();ite!=ite_end;ite++){
//                            [kf_buf appendBytes:&(*ite) length:sizeof(char)];//后面用的时候，要翻转过来 才是真正的描述符 翻过来了
//                        }
//
//                        assert(str.size()==256);
//                    }
//
////                    cout<<"kf_buf.length"<<kf_buf.length<<endl;
//                    int measure_len=kf_measurements_origin.size();
////                    cout<<"measure_len"<<measure_len<<endl;
//                    [kf_buf appendBytes:&(measure_len) length:sizeof(int)];
//                    for(int a=0;a<measure_len;a++){
//                        cv::Point2f kf_mea=kf_measurements_origin[a];
//                        [kf_buf appendBytes:&(kf_mea.x) length:sizeof(float)];
//                        [kf_buf appendBytes:&(kf_mea.y) length:sizeof(float)];
//                    }
//
//                    assert(kf_point_clouds_origin.size()==measure_len);
//                    for(int a=0;a<measure_len;a++){
//                        Eigen::Vector3d kf_point=kf_point_clouds_origin[a];
//                        for(int b=0;b<3;b++){
//                            [kf_buf appendBytes:&(kf_point[b]) length:sizeof(double)];
//                        }
//                    }
//
//                    assert(kf_features_id_origin.size()==measure_len);
//                    for(int a=0;a<measure_len;a++){
//                        [kf_buf appendBytes:&(kf_features_id_origin[a]) length:sizeof(int)];
//                    }
////                    cout<<"kf_buf.length end"<<kf_buf.length<<endl;
//                    BOOL writeImuSuccess_time=[kf_buf writeToFile:filePath_kf atomically:YES];
//                    if(!writeImuSuccess_time){
//                            NSLog(@"kf已保存完整fail");
//                    }
//
//                }
//                NSLog(@"kf 保存结束 结束程序");
                }
                return;
            }


            //一次性读取完图片的时间 读了img.csv
            [self fileContents_img_2_buf:imageDataReadIndex];

            //流一直不关 获得图片
            [self readImage_euroc];

            imageDataReadIndex++;
#ifdef DATA_EXPORT
            [self tapSaveImageToIphone:imgData.image];
#endif


            UIImageToMat(imgData.image,image);
//            cout<<"image.rows="<<image.rows<<" image.cols"<<image.cols<<endl;
            UIImageToMat(imgData.image,input_frame);

            imgData.image=nil;

            img_msg->header = imgData.header;
            //TE(readImg);
#ifdef DATA_EXPORT
            printf("record play image: %lf\n",imgData.header,imageDataReadIndex);
#endif
//            printf("record play image: %lf , %ld\n",imgData.header,imageDataReadIndex);
        }
        else if(start_playback_dataKitti0930){
            if(imageDataReadIndex==count_img){
                if(sum_img==false){
                    sum_img=true;
                    [NSThread sleepForTimeInterval:5];//给80s去把优化做完

                    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
                    NSString *documentsPath = [paths objectAtIndex:0];




                    NSString *filePath_tum_pose = [documentsPath stringByAppendingPathComponent:@"tum_pose"];
                    [self checkFilePath:filePath_tum_pose];

    //                [NSThread sleepForTimeInterval:30];
                    //这里可能得加个锁 不安全
                    NSMutableData *poseDataBuf_time_r_t = [[NSMutableData alloc] init];
                    vector<double> time=keyframe_database.path_time;
                    vector<Eigen::Vector3f> path=keyframe_database.refine_path;
                    vector<Eigen::Quaterniond> pose_r=keyframe_database.refine_r;
                    int len=time.size();
                    for(int i=0;i<len;i++){
                        [poseDataBuf_time_r_t appendBytes:&(time[i]) length:sizeof(double)];
                        [poseDataBuf_time_r_t appendBytes:&(path[i]) length:sizeof(path[i])];
                        [poseDataBuf_time_r_t appendBytes:&(pose_r[i]) length:sizeof(pose_r[i])];
                    }
    //                printf("位姿保存完了 验证一下 %f, %f, %f, %f\n",pose_r[0].x(), pose_r[0].y(),pose_r[0].z(),pose_r[0].w());
                    BOOL writeImuSuccess_time=[poseDataBuf_time_r_t writeToFile:filePath_tum_pose atomically:YES];
                    if(writeImuSuccess_time){
                            NSLog(@"位姿时间戳已保存完整succ");
                    }else{
                            NSLog(@"位姿时间戳已保存完整fail");
                    }



//                    NSString *documentsPath_kf = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"KF"]; //Get the docs directory
//                    [self checkDirectoryPath:0 withObject:documentsPath_kf];
//                    Eigen::Matrix3d r;
//                    Eigen::Vector3d t;
//                    //保存每一帧的信息，每一帧一个文件
//                    for(int i=0;i<len;i++){
//                        NSString *filename = [NSString stringWithFormat:@"%lu", i];
//                        NSString *filePath_kf = [documentsPath_kf stringByAppendingPathComponent:filename]; //Add the file name
//                        [self checkFilePath:filePath_kf];
//                        NSMutableData *kf_buf = [[NSMutableData alloc] init];
//
//                        KeyFrame *kf=keyframe_database.getKeyframe(i);
//                        vector<cv::KeyPoint> kf_keypoint=kf->keypoints;
//                        vector<BRIEF::bitset> kf_descriptors=kf->descriptors;
//                        vector<cv::Point2f> kf_measurements_origin=kf->measurements_origin;
//                        vector<Eigen::Vector3d> kf_point_clouds_origin=kf->point_clouds_origin;
//                        vector<int> kf_features_id_origin=kf->features_id_origin;
//                        kf->getPose(t,r);
//                        int kf_id=kf->global_index;
//
//                        [kf_buf appendBytes:&(kf_id) length:sizeof(int)];
//                        [kf_buf appendBytes:&(time[i]) length:sizeof(double)];
//                        for(int a=0;a<3;a++){
//                            [kf_buf appendBytes:&(t[a]) length:sizeof(double)];
//                        }
//
//                        for(int a=0;a<3;a++){
//                            for(int b=0;b<3;b++){
//                                [kf_buf appendBytes:&(r(a,b)) length:sizeof(double)];
//                            }
//                        }
//
//                        int keypoint_len=kf_keypoint.size();
//                        cout<<"keypoint_len="<<keypoint_len<<endl;
//    //                    cout<<"sizeof(double)"<<sizeof(double)<<endl;
//    //                    cout<<"sizeof(float)"<<sizeof(float)<<endl;
//    //                    cout<<"sizeof(int)"<<sizeof(int)<<endl;
//                        cout<<"kf_buf.length"<<kf_buf.length<<endl;
//                        [kf_buf appendBytes:&(keypoint_len) length:sizeof(int)];
//                        for(int a=0;a<keypoint_len;a++){
//                            cv::KeyPoint kf_key=kf_keypoint[a];
//                            [kf_buf appendBytes:&(kf_key.pt.x) length:sizeof(float)];
//                            [kf_buf appendBytes:&(kf_key.pt.y) length:sizeof(float)];
//                            [kf_buf appendBytes:&(kf_key.size) length:sizeof(float)];
//                            [kf_buf appendBytes:&(kf_key.angle) length:sizeof(float)];
//                            [kf_buf appendBytes:&(kf_key.response) length:sizeof(float)];
//                            [kf_buf appendBytes:&(kf_key.octave) length:sizeof(int)];
//                            [kf_buf appendBytes:&(kf_key.class_id) length:sizeof(int)];
//    //                        cout<<"kf_key.pt.x="<<kf_key.pt.x<<endl;
//                        }
//
//                        for(int a=0;a<keypoint_len;a++){
//                            string str;
//                            to_string(kf_descriptors[a],str);
//    //                        cout<<"str_des="<<str<<endl;
//                            for(auto ite=str.rbegin(),ite_end=str.rend();ite!=ite_end;ite++){
//                                [kf_buf appendBytes:&(*ite) length:sizeof(char)];//后面用的时候，要翻转过来 才是真正的描述符 翻过来了
//                            }
//
//                            assert(str.size()==256);
//                        }
//
//                        cout<<"kf_buf.length"<<kf_buf.length<<endl;
//                        int measure_len=kf_measurements_origin.size();
//                        cout<<"measure_len"<<measure_len<<endl;
//                        [kf_buf appendBytes:&(measure_len) length:sizeof(int)];
//                        for(int a=0;a<measure_len;a++){
//                            cv::Point2f kf_mea=kf_measurements_origin[a];
//                            [kf_buf appendBytes:&(kf_mea.x) length:sizeof(float)];
//                            [kf_buf appendBytes:&(kf_mea.y) length:sizeof(float)];
//                        }
//
//                        assert(kf_point_clouds_origin.size()==measure_len);
//                        for(int a=0;a<measure_len;a++){
//                            Eigen::Vector3d kf_point=kf_point_clouds_origin[a];
//                            for(int b=0;b<3;b++){
//                                [kf_buf appendBytes:&(kf_point[b]) length:sizeof(double)];
//                            }
//                        }
//
//                        assert(kf_features_id_origin.size()==measure_len);
//                        for(int a=0;a<measure_len;a++){
//                            [kf_buf appendBytes:&(kf_features_id_origin[a]) length:sizeof(int)];
//                        }
//                        cout<<"kf_buf.length end"<<kf_buf.length<<endl;
//                        BOOL writeImuSuccess_time=[kf_buf writeToFile:filePath_kf atomically:YES];
//                        if(!writeImuSuccess_time){
//                                NSLog(@"kf已保存完整fail");
//                        }
//
//                    }
//                    NSLog(@"kf 保存结束 结束程序");
                    }
                    return;
                }


            //一次性读取完图片的时间 读了img.csv
            [self fileContents_img_2_buf_kitti:imageDataReadIndex];

            //流一直不关 获得图片
            [self readImage_euroc];

            imageDataReadIndex++;
#ifdef DATA_EXPORT
            [self tapSaveImageToIphone:imgData.image];
#endif


            UIImageToMat(imgData.image,image);
//            cout<<"image.rows="<<image.rows<<" image.cols"<<image.cols<<endl;
            UIImageToMat(imgData.image,input_frame);

            imgData.image=nil;

            img_msg->header = imgData.header;
            //TE(readImg);
#ifdef DATA_EXPORT
            printf("record play image: %lf\n",imgData.header,imageDataReadIndex);
#endif
        }
        else if(start_playback_mvsec){


            if(imageDataReadIndex==count_img){
                if(sum_img==false){
                sum_img=true;
                [NSThread sleepForTimeInterval:5];//给80s去把优化做完

                NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
                NSString *documentsPath = [paths objectAtIndex:0];




                NSString *filePath_tum_pose = [documentsPath stringByAppendingPathComponent:@"tum_pose"];
                [self checkFilePath:filePath_tum_pose];

//                [NSThread sleepForTimeInterval:30];
                //这里可能得加个锁 不安全
                NSMutableData *poseDataBuf_time_r_t = [[NSMutableData alloc] init];
                vector<double> time=keyframe_database.path_time;
                vector<Eigen::Vector3f> path=keyframe_database.refine_path;
                vector<Eigen::Quaterniond> pose_r=keyframe_database.refine_r;
                int len=time.size();
                for(int i=0;i<len;i++){
                    [poseDataBuf_time_r_t appendBytes:&(time[i]) length:sizeof(double)];
                    [poseDataBuf_time_r_t appendBytes:&(path[i]) length:sizeof(path[i])];
                    [poseDataBuf_time_r_t appendBytes:&(pose_r[i]) length:sizeof(pose_r[i])];
                }
//                printf("位姿保存完了 验证一下 %f, %f, %f, %f\n",pose_r[0].x(), pose_r[0].y(),pose_r[0].z(),pose_r[0].w());
                BOOL writeImuSuccess_time=[poseDataBuf_time_r_t writeToFile:filePath_tum_pose atomically:YES];
                if(writeImuSuccess_time){
                        NSLog(@"位姿时间戳已保存完整succ");
                }else{
                        NSLog(@"位姿时间戳已保存完整fail");
                }



                NSString *documentsPath_kf = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"KF"]; //Get the docs directory
                [self checkDirectoryPath:0 withObject:documentsPath_kf];
//                Eigen::Matrix3d r;
//                Eigen::Vector3d t;
//                //保存每一帧的信息，每一帧一个文件
//                for(int i=0;i<len;i++){
//                    NSString *filename = [NSString stringWithFormat:@"%lu", i];
//                    NSString *filePath_kf = [documentsPath_kf stringByAppendingPathComponent:filename]; //Add the file name
//                    [self checkFilePath:filePath_kf];
//                    NSMutableData *kf_buf = [[NSMutableData alloc] init];
//
//                    KeyFrame *kf=keyframe_database.getKeyframe(i);
//                    vector<cv::KeyPoint> kf_keypoint=kf->keypoints;
//                    vector<BRIEF::bitset> kf_descriptors=kf->descriptors;
//                    vector<cv::Point2f> kf_measurements_origin=kf->measurements_origin;
//                    vector<Eigen::Vector3d> kf_point_clouds_origin=kf->point_clouds_origin;
//                    vector<int> kf_features_id_origin=kf->features_id_origin;
//                    kf->getPose(t,r);
//                    int kf_id=kf->global_index;
//
//                    [kf_buf appendBytes:&(kf_id) length:sizeof(int)];
//                    [kf_buf appendBytes:&(time[i]) length:sizeof(double)];
//                    for(int a=0;a<3;a++){
//                        [kf_buf appendBytes:&(t[a]) length:sizeof(double)];
//                    }
//
//                    for(int a=0;a<3;a++){
//                        for(int b=0;b<3;b++){
//                            [kf_buf appendBytes:&(r(a,b)) length:sizeof(double)];
//                        }
//                    }
//
//                    int keypoint_len=kf_keypoint.size();
//                    cout<<"keypoint_len="<<keypoint_len<<endl;
////                    cout<<"sizeof(double)"<<sizeof(double)<<endl;
////                    cout<<"sizeof(float)"<<sizeof(float)<<endl;
////                    cout<<"sizeof(int)"<<sizeof(int)<<endl;
//                    cout<<"kf_buf.length"<<kf_buf.length<<endl;
//                    [kf_buf appendBytes:&(keypoint_len) length:sizeof(int)];
//                    for(int a=0;a<keypoint_len;a++){
//                        cv::KeyPoint kf_key=kf_keypoint[a];
//                        [kf_buf appendBytes:&(kf_key.pt.x) length:sizeof(float)];
//                        [kf_buf appendBytes:&(kf_key.pt.y) length:sizeof(float)];
//                        [kf_buf appendBytes:&(kf_key.size) length:sizeof(float)];
//                        [kf_buf appendBytes:&(kf_key.angle) length:sizeof(float)];
//                        [kf_buf appendBytes:&(kf_key.response) length:sizeof(float)];
//                        [kf_buf appendBytes:&(kf_key.octave) length:sizeof(int)];
//                        [kf_buf appendBytes:&(kf_key.class_id) length:sizeof(int)];
////                        cout<<"kf_key.pt.x="<<kf_key.pt.x<<endl;
//                    }
//
//                    for(int a=0;a<keypoint_len;a++){
//                        string str;
//                        to_string(kf_descriptors[a],str);
////                        cout<<"str_des="<<str<<endl;
//                        for(auto ite=str.rbegin(),ite_end=str.rend();ite!=ite_end;ite++){
//                            [kf_buf appendBytes:&(*ite) length:sizeof(char)];//后面用的时候，要翻转过来 才是真正的描述符 翻过来了
//                        }
//
//                        assert(str.size()==256);
//                    }
//
//                    cout<<"kf_buf.length"<<kf_buf.length<<endl;
//                    int measure_len=kf_measurements_origin.size();
//                    cout<<"measure_len"<<measure_len<<endl;
//                    [kf_buf appendBytes:&(measure_len) length:sizeof(int)];
//                    for(int a=0;a<measure_len;a++){
//                        cv::Point2f kf_mea=kf_measurements_origin[a];
//                        [kf_buf appendBytes:&(kf_mea.x) length:sizeof(float)];
//                        [kf_buf appendBytes:&(kf_mea.y) length:sizeof(float)];
//                    }
//
//                    assert(kf_point_clouds_origin.size()==measure_len);
//                    for(int a=0;a<measure_len;a++){
//                        Eigen::Vector3d kf_point=kf_point_clouds_origin[a];
//                        for(int b=0;b<3;b++){
//                            [kf_buf appendBytes:&(kf_point[b]) length:sizeof(double)];
//                        }
//                    }
//
//                    assert(kf_features_id_origin.size()==measure_len);
//                    for(int a=0;a<measure_len;a++){
//                        [kf_buf appendBytes:&(kf_features_id_origin[a]) length:sizeof(int)];
//                    }
//                    cout<<"kf_buf.length end"<<kf_buf.length<<endl;
//                    BOOL writeImuSuccess_time=[kf_buf writeToFile:filePath_kf atomically:YES];
//                    if(!writeImuSuccess_time){
//                            NSLog(@"kf已保存完整fail");
//                    }
//
//                }
                NSLog(@"kf 保存结束 结束程序");
                }
                return;
            }


            //一次性读取完图片的时间 读了img.csv
            [self fileContents_img_2_buf_mvsec:imageDataReadIndex];

            //流一直不关 获得图片
            [self readImage_euroc];

            imageDataReadIndex++;
#ifdef DATA_EXPORT
            [self tapSaveImageToIphone:imgData.image];
#endif


            UIImageToMat(imgData.image,image);
//            cout<<"image.rows="<<image.rows<<" image.cols"<<image.cols<<endl;
            UIImageToMat(imgData.image,input_frame);

            imgData.image=nil;

            img_msg->header = imgData.header;
            //TE(readImg);
#ifdef DATA_EXPORT
            printf("record play image: %lf\n",imgData.header,imageDataReadIndex);
#endif
//            printf("record play image: %lf , %ld\n",imgData.header,imageDataReadIndex);
        }
        else
        {
            input_frame = image;
        }

        if(start_record)
        {
            imgData.header = img_msg->header;
            imgData.image = MatToUIImage(image);
            imgDataBuf.push(imgData);
            img_num++;
//            cout<<"processImage img_num:"<<img_num<<endl;
            return;
        }
        else if(start_record_2)
        {
            imgData.header = img_msg->header;
            imgData.image = MatToUIImage(image);
            imgDataBuf.push(imgData);
            img_num++;
//            cout<<"processImage img_num:"<<img_num<<endl;
            return;
        }
        else
        {
            img_num++;
//            cout<<"img_num:"<<img_num<<endl;
            if(!imgDataBuf.empty())
                return;
        }

        prevTime = mach_absolute_time();

        cv::Mat gray;
        if(start_playback_dataEuroc){
            gray=input_frame.clone();
        }else if(start_playback_dataKitti0930){
            gray=input_frame.clone();
        }
        else if(start_playback_mvsec){
//            gray=input_frame.clone();
            cv::cvtColor(input_frame, gray, CV_RGBA2GRAY);
        }
        else{
            cv::cvtColor(input_frame, gray, CV_RGBA2GRAY);
        }
        cv::Mat img_with_feature;
        cv::Mat img_equa;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(3);
        clahe->apply(gray, img_equa);

        //img_equa = gray;
//        TS(time_feature);

        m_depth_feedback.lock();
        featuretracker.solved_features = solved_features;//list 观测点和位置
        featuretracker.solved_vins = solved_vins;//vins 结果反馈
        m_depth_feedback.unlock();

        m_imu_feedback.lock();
//        m_imu_lock=true;
        featuretracker.imu_msgs = getImuMeasurements(img_msg->header);//vector imu 加速度和角速度
        m_imu_feedback.unlock();
//        m_imu_lock=false;

        vector<cv::Point2f> good_pts;
        vector<double> track_len;
        bool vins_normal = (vins.solver_flag == VINS::NON_LINEAR);
        featuretracker.use_pnp = USE_PNP;
        
        
        
        //另外这里还求了r,t 这个是粗糙的显示点的 并没有用于最终的位姿求解
        featuretracker.readImage3(img_equa, img_with_feature,frame_cnt, good_pts, track_len, img_msg->header, pnp_P, pnp_R, vins_normal);//特征点的跟踪和生成

        //临时的------------------------------------
//        cv::Mat img_test;
//        input_frame.copyTo(img_test);
//        featuretracker.undistortedPoints_handlerImg(img_test);
//        NSData *data = [NSData dataWithBytes:img_test.data length:img_test.elemSize()*img_test.total()];
//
//        CGColorSpaceRef colorSpace;
//        CGBitmapInfo bitmapInfo;
//
//         if (img_test.elemSize() == 1) {
//             colorSpace = CGColorSpaceCreateDeviceGray();
//             bitmapInfo = kCGImageAlphaNone | kCGBitmapByteOrderDefault;
//         } else {
//             colorSpace = CGColorSpaceCreateDeviceRGB();
//             bitmapInfo = kCGBitmapByteOrder32Little | (
//                                                        img_test.elemSize() == 3? kCGImageAlphaNone : kCGImageAlphaNoneSkipFirst
//                                                        );
//         }
//
//         CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
//
//         // Creating CGImage from cv::Mat
//         CGImageRef imageRef = CGImageCreate(
//                                             img_test.cols,                 //width
//                                             img_test.rows,                 //height
//                                             8,                          //bits per component
//                                             8 * img_test.elemSize(),       //bits per pixel
//                                             img_test.step[0],              //bytesPerRow
//                                             colorSpace,                 //colorspace
//                                             bitmapInfo,                 // bitmap info
//                                             provider,                   //CGDataProviderRef
//                                             NULL,                       //decode
//                                             false,                      //should interpolate
//                                             kCGRenderingIntentDefault   //intent
//                                             );
//
//         // Getting UIImage from CGImage
//         UIImage *finalImage = [UIImage imageWithCGImage:imageRef];
//         CGImageRelease(imageRef);
//         CGDataProviderRelease(provider);
//         CGColorSpaceRelease(colorSpace);
//
//        UIImageWriteToSavedPhotosAlbum(finalImage, self, @selector(image:didFinishSavingWithError:contextInfo:), NULL);


        //------------------------------------


//        TE(time_feature);
        //cvtColor(img_equa, img_equa, CV_GRAY2BGR);
//        cout<<"good_pts.size="<<good_pts.size()<<endl;//这个点是没有去畸变的
        for (int i = 0; i < good_pts.size(); i++)
        {

//            cv::circle(image, good_pts[i], 0, cv::Scalar(255 * (1 - track_len[i]), 0, 255 * track_len[i]), 7); //BGR
            cv::circle(image, good_pts[i], 0, cv::Scalar(255 * (1 - track_len[i]), 0, 255 * track_len[i]), 7); //BGR
        }
//      cout<<"821 "<<featuretracker.img_cnt<<endl;
        //image msg buf
        if(featuretracker.img_cnt==0)
        {
//            cout<<"840"<<endl;
            //去畸变了的 这个其实也是新增的特征点
            img_msg->point_clouds = featuretracker.image_msg;//相机平面坐标系 不能完全说是归一化 是因为2维到3维只能恢复出两个维度，另一个维度设为1 已经去掉畸变了
            //没有去畸变
            img_msg->distorted_point_clouds = featuretracker.distorted_image_msg;
            //img_msg callback
            m_buf.lock();
//            m_buf_lock=true;
            img_msg_buf.push(img_msg);

            //NSLog(@"Img timestamp %lf",img_msg_buf.front()->header);
            m_buf.unlock();
//            m_buf_lock=false;
            con.notify_one();
            if(imageCacheEnabled)
            {
                image_data_cache.header = img_msg->header;
                image_data_cache.image = MatToUIImage(image);
                image_pool.push(image_data_cache);
            }

            if(LOOP_CLOSURE)
            {
                m_image_buf_loop.lock();
                cv::Mat loop_image = gray.clone();
                image_buf_loop.push(make_pair(loop_image, img_msg->header));
                if(image_buf_loop.size() > WINDOW_SIZE){
//                    printf("image_buf_loop.front().second:%lf\n",image_buf_loop.front().second);
                    image_buf_loop.pop();
                }
                m_image_buf_loop.unlock();
            }

            else if(LOOP_CLOSURE_SERVER || LOOP_CLOSURE_SERVER_noLoop){
                m_image_buf_loop.lock();
                cv::Mat loop_image = gray.clone();
                image_buf_loop.push(make_pair(loop_image, img_msg->header));
                if(image_buf_loop.size() > WINDOW_SIZE)
                    image_buf_loop.pop();
                m_image_buf_loop.unlock();
            }
        }


        featuretracker.img_cnt = (featuretracker.img_cnt + 1) % FREQ;
        for (int i = 0; i < good_pts.size(); i++)
        {
            cv::circle(image, good_pts[i], 0, cv::Scalar(255 * (1 - track_len[i]), 0, 255 * track_len[i]), 7); //BGR
        }

//        TS(visualize);
        if(imageCacheEnabled)
        {

            //use aligned vins and image
            if(!vins_pool.empty() && !image_pool.empty())
            {
                while(vins_pool.size() > 1)
                {
                    vins_pool.pop();
                }
                while(!image_pool.empty() && fabs(image_pool.front().header-vins_pool.front().header)>0.00001&&(image_pool.front().header < (vins_pool.front().header+0.00001)))
//                while(!image_pool.empty() && image_pool.front().header < vins_pool.front().header)
                {
                    image_pool.pop();
                }

                if(!vins_pool.empty() && !image_pool.empty())
                {
                    lateast_image = image_pool.front().image;
                    lateast_P = vins_pool.front().P;
                    lateast_R = vins_pool.front().R;
                    UIImageToMat(lateast_image, image);
                }
            }
            else if(!image_pool.empty())
            {
                if(image_pool.size() > 10)
                    image_pool.pop();
            }
        }

        if(USE_PNP)
        {
            lateast_P = pnp_P.cast<float>();
            lateast_R = pnp_R.cast<float>();
        }

        if(ui_main || start_show == false || vins.solver_flag != VINS::NON_LINEAR)  //show image and AR
        {

            cv::Mat tmp2;
            if(vins.solver_flag == VINS::NON_LINEAR && start_show)
            {
                cv::Mat tmp;
                vins.drawresult.startInit = true;

                vins.drawresult.drawAR(vins.imageAI, vins.correct_point_cloud, lateast_P, lateast_R);
                

//                画我们自己的物体,暂时注释
                if(0 && isOurAR){
                   
                    Eigen::Matrix3f RIC, r_cw,r_wc;
                    Eigen::Vector3f t_cw,tic,t_wc;
                    RIC = Utility::ypr2R(Eigen::Vector3d(RIC_y,RIC_p,RIC_r)).cast<float>();
                    tic=(Eigen::Vector3d(TIC_X,TIC_Y,TIC_Z)).cast<float>();
                    r_wc=lateast_R*RIC;
                    t_wc=lateast_R*tic +lateast_P;
                    r_cw=(lateast_R*RIC).transpose();
                    t_cw=-r_cw*(lateast_R*tic +lateast_P);
                    
                    float tx=t_cw[0],ty=t_cw[1],tz=t_cw[2];
                    float rxx=r_cw(0,0),rxy=r_cw(0,1),rxz=r_cw(0,2),ryx=r_cw(1,0),ryy=r_cw(1,1),ryz=r_cw(1,2),rzx=r_cw(2,0),rzy=r_cw(2,1),rzz=r_cw(2,2);
                    cv::Mat T=(cv::Mat_<float>(3,1)<<tx, ty, tz);
                    cv::Mat R=(cv::Mat_<float>(3,3)<<rxx,rxy,rxz,ryx,ryy,ryz,rzx,rzy,rzz);
                    
//                    Eigen::Vector3f i_r_cw=Utility::R2ypr(r_cw.cast<double>()).cast<float>();
//                    r_cw=Utility::ypr2R(Eigen::Vector3d(i_r_cw[1],i_r_cw[0],i_r_cw[2])).cast<float>();
//                    float rxx=r_cw(0,0),rxy=r_cw(0,1),rxz=r_cw(0,2),ryx=r_cw(1,0),ryy=r_cw(1,1),ryz=r_cw(1,2),rzx=r_cw(2,0),rzy=r_cw(2,1),rzz=r_cw(2,2);
//                    cv::Mat R=(cv::Mat_<float>(3,3)<<rxx,rxy,rxz,ryx,ryy,ryz,rzx,rzy,rzz);
                    
//                    float tx=t_wc[0],ty=t_wc[1],tz=t_wc[2];
//                    float rxx=r_wc(0,0),rxy=r_wc(0,1),rxz=r_wc(0,2),ryx=r_wc(1,0),ryy=r_wc(1,1),ryz=r_wc(1,2),rzx=r_wc(2,0),rzy=r_wc(2,1),rzz=r_wc(2,2);
//                    cv::Mat T=(cv::Mat_<float>(3,1)<<tx, ty, tz);
//                    cv::Mat R=(cv::Mat_<float>(3,3)<<rxx,rxy,rxz,ryx,ryy,ryz,rzx,rzy,rzz);
                   
//                    Eigen::Vector3f center_v3f= -lateast_R.transpose()*lateast_P;
//                    cv::Mat center=(cv::Mat_<float>(3,1)<<center_v3f[0], center_v3f[1], center_v3f[2]);
//                    [self addPose:SCNVector3Make(-center.at<float>(0), center.at<float>(1), center.at<float>(2))];
                    cout<<"帧的位姿"<<endl;
                    
                    for (size_t ii = 0; ii < 3; ii++)
                    {
                        for (size_t jj = 0; jj < 3; jj++)
                        {
                            cout << R.at<float>(ii,jj)<< ",";
                        }
                        cout << T.at<float>(ii) << endl;
                    }
                    cout<<endl<<endl;
                    
                    [self drawObjectWith2:R andT:T];
                }
//             std::vector<Edge> edges=   vins.drawresult.drawAR(vins.imageAI, vins.correct_point_cloud, lateast_P, lateast_R);

//                NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
//                NSString *documentsPath_triangulate = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"triangulate"]; //Get the docs directory
//                if(triangulation_num==0)
//                    [self checkDirectoryPath:0 withObject:documentsPath_triangulate];
////                int triangulate_i=0;
//
//                NSString *filename_triangulate = [NSString stringWithFormat:@"%lu", triangulation_num];
//                NSString *filePath_triangulate = [documentsPath_triangulate stringByAppendingPathComponent:filename_triangulate]; //Add the file name
//                [self checkFilePath:filePath_triangulate];
//                NSMutableData *triangulate_buf = [[NSMutableData alloc] init];
//                for(auto e = begin(edges); e != end(edges); e++) {
//                    cv::Point2f pts, pts2;
//                    pts.x=(*e).p1.x;
//                    pts.y=(*e).p1.y;
//                    pts2.x=(*e).p2.x;
//                    pts2.y=(*e).p2.y;
//
//
//
//
//                    [triangulate_buf appendBytes:&(pts.x) length:sizeof(float)];
//                    [triangulate_buf appendBytes:&(pts.y) length:sizeof(float)];
//                    [triangulate_buf appendBytes:&(pts2.x) length:sizeof(float)];
//                    [triangulate_buf appendBytes:&(pts2.y) length:sizeof(float)];
////                    triangulate_i++;
//
//
//                }
//           BOOL writeImuSuccess_triangulate=[triangulate_buf writeToFile:filePath_triangulate atomically:YES];
//                              if(!writeImuSuccess_triangulate){
//                                      NSLog(@"triangulate已保存完整fail");
//                              }
//                triangulation_num++;

                if(start_playback_dataEuroc || start_playback_dataKitti0930){

                    cv::cvtColor(image, tmp, CV_GRAY2BGR);
                }
                //ios start_playback_mvsec
                else{
                    cv::cvtColor(image, tmp, CV_RGBA2BGR);
                }
                cv::Mat mask;//图像掩膜
                cv::Mat imageAI = vins.imageAI;
                if(!imageAI.empty())
                    cv::cvtColor(imageAI, mask, CV_RGB2GRAY);


                imageAI.copyTo(tmp,mask);
                cv::cvtColor(tmp, image, CV_BGRA2BGR);

                vins.drawresult.grounds_ar_mutex.lock();
                if(vins.drawresult.Grounds_send.size()>0){
                    GroundPoint ar_send=vins.drawresult.Grounds_send.front();//ar发送给服务器，服务器就有了，这边就不需要再保存这份了
                    vins.drawresult.Grounds_send.pop();
                    vins.drawresult.grounds_ar_mutex.unlock();
                    dispatch_async(dispatch_get_main_queue(), ^{ sendAR(ar_send.ori, ar_send.cox, ar_send.coy, ar_send.coz, ar_send.size);
                    });
                }else{
                    vins.drawresult.grounds_ar_mutex.unlock();
                }
            }
            if(DEBUG_MODE)
            {
                cv::flip(lateast_equa, image, -1);
            }
            else
            {
                if(start_playback_dataEuroc || start_playback_dataKitti0930 || start_playback_mvsec){
                    transpose(image, tmp2);
                    cv::flip(tmp2,tmp2,0);
//
//                    tmp2=image;

//                    cv::flip(image,tmp2,-1);//左右 上下翻转
                }
                else{
//                    transpose(image, tmp2);
//                    cv::flip(tmp2,tmp2,0);
//                    tmp2=image;
                    
                    cv::flip(image,tmp2,-1);//暂时注释

                }
                image = tmp2;
                if(vins.solver_flag != VINS::NON_LINEAR || !start_show){
                    if(start_playback_dataEuroc || start_playback_dataKitti0930){
                        cv::cvtColor(image, image, CV_GRAY2BGR);
                    }
                    //ios start_playback_mvsec
                    else{
                        cv::cvtColor(image, image, CV_RGBA2BGR);
                    }
                }
            }
        }
        else //show VINS
        {

            if(vins.solver_flag == VINS::NON_LINEAR)
            {

                vins.drawresult.pose.clear();
                vins.drawresult.pose = keyframe_database.refine_path;
                vins.drawresult.segment_indexs = keyframe_database.segment_indexs;
                vins.drawresult.Reprojection(vins.image_show, vins.correct_point_cloud, vins.correct_Rs, vins.correct_Ps, box_in_trajectory);

            }

            cv::Mat tmp2 = vins.image_show;
//            printf("tmp2 row %d ,col %d\n",tmp2.rows,tmp2.cols);

            if(start_playback_dataEuroc){
                transpose(image, image);
                cv::flip(image,image,1);//左右翻转
            }
            if(start_playback_dataKitti0930){
                transpose(image, image);
                cv::flip(image,image,1);//左右翻转
            }
            if(start_playback_mvsec){
                transpose(image, image);
                cv::flip(image,image,1);//左右翻转
            }

            cv::Mat down_origin_image;
            if(start_playback_dataEuroc){
                //euroc
                cv::resize(image.t(), down_origin_image, cv::Size(150 , 230));
            }else if(start_playback_dataKitti0930){
                //kitti
                cv::resize(image.t(), down_origin_image, cv::Size(140 , 280));
            }else if(start_playback_mvsec){
                //euroc
                cv::resize(image.t(), down_origin_image, cv::Size(150 , 230));
            }
            else{
                //ios
                cv::resize(image.t(), down_origin_image, cv::Size(200, 150));
            }


            //xiaomi
//            cv::resize(image.t(), down_origin_image, cv::Size(150 , 200));
//            printf("down_origin_image row %d ,col %d\n",down_origin_image.rows,down_origin_image.cols);

            if(start_playback_dataEuroc){

                cv::cvtColor(down_origin_image, down_origin_image, CV_GRAY2RGB);

            }else if(start_playback_dataKitti0930){
                cv::cvtColor(down_origin_image, down_origin_image, CV_GRAY2RGB);
            }else if(start_playback_mvsec){
//                cv::cvtColor(down_origin_image, down_origin_image, CV_GRAY2RGB);
                cv::cvtColor(down_origin_image, down_origin_image, CV_BGRA2RGB);
            }
            else{
                cv::cvtColor(down_origin_image, down_origin_image, CV_BGRA2RGB);
            }

            cv::flip(down_origin_image,down_origin_image,0);//左右翻转


            cv::Mat imageROI;
            imageROI = tmp2(cv::Rect(10,COL - down_origin_image.rows- 10, down_origin_image.cols,down_origin_image.rows));

            cv::Mat mask;


            cv::cvtColor(down_origin_image, mask, CV_RGB2GRAY);

            down_origin_image.copyTo(imageROI, mask);


            cv::cvtColor(tmp2, image, CV_BGRA2BGR);
            cv::flip(image,tmp2,1);// 左右翻转
            if (isNeedRotation)
                image = tmp2.t();
        }

//        TE(visualize);



    } else {
        // Not capturing, means not started yet
        cv::cvtColor(image, image, CV_BGRA2RGB);
        cv::flip(image,image,-1);
        //BOOL isNeedRotation = image.size() != frameSize;
        //if (isNeedRotation)
        //    image = image.t();
    }


}


/*
 Send imu data and visual data into VINS
 */
std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
    while (true)
    {
        if (imu_msg_buf.empty() || img_msg_buf.empty()){
//            cout<<"哪个没有数据"<<imu_msg_buf.empty()<<" "<<img_msg_buf.empty()<<endl;
            return measurements;
        }
//        cout<<"有数据了"<<endl;
//        printf("imu_msg_buf.size=%d , img_msg_buf.size=%d\n",imu_msg_buf.size(),img_msg_buf.size());
//        cout<<std::fixed<<"img header"<<img_msg_buf.front()->header<<endl;
//        cout<<std::fixed<<"imu header"<<imu_msg_buf.front()->header<<endl;
//        if (!(imu_msg_buf.back()->header > img_msg_buf.front()->header))
        if (!(imu_msg_buf.back()->header > img_msg_buf.front()->header))
        {
//            cout<<"imu_msg_buf.size "<<imu_msg_buf.size()<<endl;
//            cout.setf(ios::fixed,ios::floatfield);
        cout<<fixed << setprecision(15) <<"imu_msg_buf.back()->header: "<<imu_msg_buf.back()->header<<" "<<img_msg_buf.front()->header<<endl;
            NSLog(@"wait for imu, only should happen at the beginning");
            return measurements;
        }
//        if (!(imu_msg_buf.front()->header < img_msg_buf.front()->header))
        if (!(imu_msg_buf.front()->header < img_msg_buf.front()->header))
        {
//            cout<<"imu_msg_buf.size "<<imu_msg_buf.size()<<" "<<img_msg_buf.size()<<endl;
//            cout.setf(ios::fixed,ios::floatfield);
//            cout<<"imu_msg_buf.front()->header: "<<imu_msg_buf.front()->header<<" "<<img_msg_buf.front()->header<<endl;
            NSLog(@"throw img, only should happen at the beginning");
            img_msg_buf.pop();
            continue;
        }
        ImgConstPtr img_msg = img_msg_buf.front();
        if (img_msg==nullptr) {
            printf("img_msg nullptr\n");
        }
//        cout<<setprecision(18)<<"img header:"<<img_msg->header<<endl;
//        printf("img_msg: %lf\n",img_msg->header);
        img_msg_buf.pop();

        std::vector<ImuConstPtr> IMUs;
//        while (imu_msg_buf.front()->header <= img_msg->header)
        while (imu_msg_buf.front()->header <= img_msg->header)
        {
//            cout<<setprecision(18)<<"imu header:"<<imu_msg_buf.front()->header<<endl;
            IMUs.emplace_back(imu_msg_buf.front());
            imu_msg_buf.pop();
        }
//        if(fabs(imu_msg_buf.front()->header - img_msg->header)<0.00001){
//            cout<<setprecision(18)<<"imu header:"<<imu_msg_buf.front()->header<<" img:"<<img_msg->header<<endl;
//            IMUs.emplace_back(imu_msg_buf.front());
//            imu_msg_buf.pop();
//        }
//        cout<<"img_msg->header="<<img_msg->header<<" 加入imu的数量"<<IMUs .size()<<" 剩余imu header"<<imu_msg_buf.front()->header<<endl;

//        cout<<"1175 "<<IMUs.size()<<endl;
        //NSLog(@"IMU_buf = %d",IMUs.size());
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

vector<IMU_MSG_LOCAL> getImuMeasurements(double header)
{
    vector<IMU_MSG_LOCAL> imu_measurements;
    static double last_header = -1;
    if(last_header < 0 || local_imu_msg_buf.empty())
    {
        last_header = header;
        return imu_measurements;
    }
     while(!local_imu_msg_buf.empty() && local_imu_msg_buf.front().header <= (last_header+0.00001)){
//    while(!local_imu_msg_buf.empty() && local_imu_msg_buf.front().header <= last_header){

        local_imu_msg_buf.pop();
    }

    while(!local_imu_msg_buf.empty() && local_imu_msg_buf.front().header <= (header+0.00001))
//    while(!local_imu_msg_buf.empty() && local_imu_msg_buf.front().header <= header)
    {
        imu_measurements.emplace_back(local_imu_msg_buf.front());
        local_imu_msg_buf.pop();
    }
    last_header = header;
    return imu_measurements;
}

void send_imu(const ImuConstPtr &imu_msg)
{
    NSTimeInterval t = imu_msg->header;
    if (current_time < 0)
        current_time = t;
    double dt = (t - current_time);
//    printf("current_time=%.9lf , t=%.9lf\n",current_time,t);

    //进行单位转换 转为s euroc时间戳单位为ns
    if(start_playback_dataEuroc){
        dt*=1.0e-9;
    }
    if(start_playback_dataKitti0930){
        dt*=1.0e-9;
    }
    if(start_playback_mvsec){
        dt*=1.0e-6;
    }
//    printf("send_imu dt=%lf \n ",dt);
    current_time = t;

    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};

    double dx = imu_msg->acc.x() - ba[0];
    double dy = imu_msg->acc.y() - ba[1];
    double dz = imu_msg->acc.z() - ba[2];

    double rx = imu_msg->gyr.x() - bg[0];
    double ry = imu_msg->gyr.y() - bg[1];
    double rz = imu_msg->gyr.z() - bg[2];
//    NSLog(@"IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, imu_msg->acc.x(), imu_msg->acc.y(), imu_msg->acc.z(), imu_msg->gyr.x(), imu_msg->gyr.y(), imu_msg->gyr.z());
//    NSLog(@"IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);
//    调用imu的预积分，调用push_back函数，函数中将时间，加速度和角速度分别存入相应的缓存中，同时调用了propagation函数 ,计算对应的状态量、协方差和雅可比矩阵
    vins.processIMU(dt, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));
}

/*
 VINS thread: this thread tightly fuses the visual measurements and imu data and solves pose, velocity, IMU bias, 3D feature for all frame in WINNDOW
 If the newest frame is keyframe, then push it into keyframe database
 */
-(void)run{
    //MH_01 2100 1975
    //MH_02 1700 2100
//    imageDataReadIndex=2100;
    [_condition lock];
    while (![[NSThread currentThread] isCancelled])
    {
//        TS(process_img);
        [self process];
//        TE(process_img);
//        [NSThread sleepForTimeInterval:0.01];
        [NSThread sleepForTimeInterval:0.02];
    }
    [_condition unlock];

}



int kf_global_index;
bool start_global_optimization = false;
bool sendG_imu=true;
-(void)process
{


    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
    std::unique_lock<std::mutex> lk(m_buf);
    con.wait(lk, [&]
             {
                 return (measurements = getMeasurements()).size() != 0;
             });
    lk.unlock();
    waiting_lists = measurements.size();
//    cout<<"waiting_lists="<<waiting_lists<<endl;
    for(auto &measurement : measurements)
    {
        for(auto &imu_msg : measurement.first)
        {
            send_imu(imu_msg);//计算偏差
        }
//        printf("send_imu frame_count=%d\n",vins.frame_count);

        auto img_msg = measurement.second;
        map<int, Eigen::Vector3d> image = img_msg->point_clouds;
        //NSLog(@"Image timestamp = %lf",img_msg->header);
        double header = img_msg->header;
//        TS(process_image);

//        if(fabs(featuretracker.header_forwKf)>0.000001){
//            featuretracker.priorMap_mutex.lock();
//            vins.measurements_cur_coarse_new=featuretracker.measurements_cur_coarse_new;
//            vins.point_3d_old_new=featuretracker.point_3d_old_new;
//            vins.feature_id_cur_new=featuretracker.feature_id_cur_new;
//            vins.header_forwkf=featuretracker.header_forwKf;
//
//            featuretracker.priorMap_mutex.unlock();
//
//            for(int a=WINDOW_SIZE - 2;a>=0;a--){
//                if(fabs(vins.Headers[a] - vins.header_forwkf)<0.00001){
//                    cout<<"最终投影的是滑动窗口中的第几帧="<<a <<endl;
//                    cout<<fixed << setprecision(15) <<vins.Headers[a]<<" , "<<vins.header_forwkf<<" , "<<keyframe_database.getLastKeyframe()->header<<endl;
//
//                }
//            }
//
//        }else{
//            vins.measurements_cur_coarse_new.clear();
//            vins.point_3d_old_new.clear();
//            vins.feature_id_cur_new.clear();
//            vins.header_forwkf=0.0;
//        }
        
        
//        assert(vins.header_forwkf==9);
//        //image是一帧图像的特征点(归一化坐标)集合,键是帧号,值是相机id和归一化坐标
        if(isUndistorted){
            map<int, Eigen::Vector3d> distorted_image = img_msg->distorted_point_clouds;
            assert(image.size()==distorted_image.size());
            vins.processImage(image,header,waiting_lists,distorted_image);
        }else{
            vins.processImage(image,header,waiting_lists);
        }
//        TE(process_image);

        //未完 可能重力也不需要发 发送初始化好的重力方向,陀螺仪偏差不需要发（后续回环检测要不要用 不清楚） 尺度因子不需要发 的确不需要发
//        if(sendG_imu && vins.solver_flag==VINS::NON_LINEAR){
//            if(LOOP_CLOSURE_SERVER){
//                dispatch_async(dispatch_get_main_queue(), ^{
//                    sendInit_aligmentParam(vins.g);
//                });
//            }
//                sendG_imu=false;
//
//        }

//        double time_now = [[NSProcessInfo processInfo] systemUptime];
//        double time_vins = vins.Headers[WINDOW_SIZE];
//        NSLog(@"vins delay %lf", time_now - time_vins);

        //update feature position for front-end
        if(vins.solver_flag == vins.NON_LINEAR)
        {
            m_depth_feedback.lock();
            solved_vins.header = vins.Headers[WINDOW_SIZE - 1];
            solved_vins.Ba = vins.Bas[WINDOW_SIZE - 1];
            solved_vins.Bg = vins.Bgs[WINDOW_SIZE - 1];
            solved_vins.P = vins.correct_Ps[WINDOW_SIZE-1].cast<double>();
            solved_vins.R = vins.correct_Rs[WINDOW_SIZE-1].cast<double>();
            solved_vins.V = vins.Vs[WINDOW_SIZE - 1];
            Eigen::Vector3d R_ypr = Utility::R2ypr(solved_vins.R);
            solved_features.clear();
            //遍历所有的特征点
            for (auto &it_per_id : vins.f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();//该特征点被多少帧看到
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;
                if (it_per_id.solve_flag != 1)//三角化了
                    continue;
                int imu_i = it_per_id.start_frame;
                Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                IMG_MSG_LOCAL tmp_feature;
                tmp_feature.id = it_per_id.feature_id;
                tmp_feature.position = vins.r_drift * vins.Rs[imu_i] * (vins.ric * pts_i + vins.tic) + vins.r_drift * vins.Ps[imu_i] + vins.t_drift;
//                暂时修改 不要乘以尺度漂移
//                tmp_feature.position =  vins.Rs[imu_i] * (vins.ric * pts_i + vins.tic) + vins.Ps[imu_i] ;
                tmp_feature.track_num = (int)it_per_id.feature_per_frame.size();
                solved_features.push_back(tmp_feature);
            }
            m_depth_feedback.unlock();
        }

        if(imageCacheEnabled)
        {
            //add state into vins buff for alignwith image
            if(vins.solver_flag == VINS::NON_LINEAR && start_show)
            {
                VINS_DATA_CACHE vins_data_cache;
                vins_data_cache.header = vins.Headers[WINDOW_SIZE-1];
                vins_data_cache.P = vins.correct_Ps[WINDOW_SIZE-1];
                vins_data_cache.R = vins.correct_Rs[WINDOW_SIZE-1];
                vins_pool.push(vins_data_cache);
            }
            else if(vins.failure_occur == true)
            {
                vins.drawresult.change_color = true;
                vins.drawresult.indexs.push_back(vins.drawresult.pose.size());
                segmentation_index++;
                keyframe_database.max_seg_index++;
                keyframe_database.cur_seg_index = keyframe_database.max_seg_index;

                while(!vins_pool.empty())
                    vins_pool.pop();
            }
        }
//        cout<<"loop closure will"<<endl;

        /**
         *** start build keyframe database for loop closure
         **/
        if(LOOP_CLOSURE)
        {
            static bool first_frame = true;
            if(vins.solver_flag != vins.NON_LINEAR)
                first_frame = true;
            if(vins.marginalization_flag == vins.MARGIN_OLD && vins.solver_flag == vins.NON_LINEAR && !image_buf_loop.empty())
            {
                first_frame = false;
                if(!first_frame && keyframe_freq % LOOP_FREQ == 0)
                {
                    keyframe_freq = 0;
                    /**
                     ** save the newest keyframe to the keyframe database
                     ** only need to save the pose to the keyframe database
                     **/
                    Eigen::Vector3d T_w_i = vins.Ps[WINDOW_SIZE - 2];
                    Eigen::Matrix3d R_w_i = vins.Rs[WINDOW_SIZE - 2];
                    m_image_buf_loop.lock();
                     while(!image_buf_loop.empty() && image_buf_loop.front().second < vins.Headers[WINDOW_SIZE - 2])
//                    while(!image_buf_loop.empty() && image_buf_loop.front().second < vins.Headers[WINDOW_SIZE - 2])
                    {
//                        pair<cv::Mat, double> p=image_buf_loop.front();
                        image_buf_loop.pop();
//                        cv::Mat img_test=p.first;

//                        img_test.release();
                    }
//                    assert(vins.Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second);
//                    cout<<fixed<<"vins.Headers[WINDOW_SIZE - 2]"<<(image_buf_loop.front().second==vins.Headers[WINDOW_SIZE - 2])<<vins.Headers[WINDOW_SIZE - 2]<<" image_buf_loop.front().second"<<image_buf_loop.front().second<<endl;
                    if(fabs(vins.Headers[WINDOW_SIZE - 2] - image_buf_loop.front().second)<0.00001)
//                    if(vins.Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second)
                    {
                        const char *pattern_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_pattern" ofType:@"yml"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
                        KeyFrame* keyframe = new KeyFrame(vins.Headers[WINDOW_SIZE - 2], global_frame_cnt, T_w_i, R_w_i, image_buf_loop.front().first, pattern_file, keyframe_database.cur_seg_index);
                        keyframe->setExtrinsic(vins.tic, vins.ric);
                        /*
                         ** we still need save the measurement to the keyframe(not database) for add connection with looped old pose
                         ** and save the pointcloud to the keyframe for reprojection search correspondance
                         */
                        keyframe->buildKeyFrameFeatures(vins);
                        keyframe_database.add(keyframe);

                        global_frame_cnt++;

                    }
                    m_image_buf_loop.unlock();

                }
                else
                {
                    first_frame = false;
                }
                // update loop info
                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    if(fabs(vins.Headers[i] - vins.front_pose.header)<0.00001)
//                    if(vins.Headers[i] == vins.front_pose.header)
                    {
                        KeyFrame* cur_kf = keyframe_database.getKeyframe(vins.front_pose.cur_index);
                        if (abs(vins.front_pose.relative_yaw) > 30.0 || vins.front_pose.relative_t.norm() > 10.0)
                        {
                            printf("Wrong loop\n");
//                            cout<<"vins.front_pose.relative_yaw="<<vins.front_pose.relative_yaw<<endl;
//                            cout<<" vins.front_pose.relative_t.norm()="<< vins.front_pose.relative_t.norm()<<endl;
                            cur_kf->removeLoop();
                            break;
                        }
//                        else{//ljl
//                            KeyFrame* old_kf = keyframe_database.getKeyframe(vins.front_pose.old_index);
//                            Eigen::Matrix3d Rs_loop;
//                            Eigen::Vector3d Ps_loop;
//                            old_kf->getOriginPose(Ps_loop,Rs_loop);
//
//
////                            Eigen::Matrix3d Rs_i= vins.Rs[i];
////                            Eigen::Vector3d Ps_i= vins.Ps[i] ;
//                            Eigen::Matrix3d Rs_i;
//                            Eigen::Vector3d Ps_i;
//                            cur_kf->getOriginPose(Ps_i, Rs_i);
//
//                            Eigen::Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                            Eigen::Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
//                            double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
//
//                            Eigen::Vector3d real_t=-vins.front_pose.relative_q.toRotationMatrix()*relative_q.transpose()*relative_t+vins.front_pose.relative_t;
//                            double real_yaw_test_test=relative_yaw-vins.front_pose.relative_yaw;
//
//                            cout<<fixed<<setprecision(6)<<"打印一下相差多少："<<real_t.norm()<<" "<<real_yaw_test_test<<endl;
//                            if(real_t.norm()<0.03 && abs(real_yaw_test_test)<1.5){
//                                cout<<"测试删除"<<endl;
//                                cur_kf->removeLoop();
//
//                                break;
//                            }
//                        }
                        cur_kf->updateLoopConnection(vins.front_pose.relative_t,
                                                     vins.front_pose.relative_q,
                                                     vins.front_pose.relative_yaw);
                        
                        //临时 实验记录一下
//                        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
//                        NSString *documentsPath = [paths objectAtIndex:0];
//
//                        NSString *filePath_tum_pose = [documentsPath stringByAppendingPathComponent:@"relative_pose"];
//
////                        if(cur_kf->global_index==0){
////                            [self checkDirectoryPath:0 withObject:filePath_tum_pose];
////                        }else{
//                            [self checkDirectoryPath:1 withObject:filePath_tum_pose];
////                        }
//
//                        NSString *filename = [NSString stringWithFormat:@"%lu&%lu", cur_kf->global_index,cur_kf->loop_index];
//                        NSString *filePath_kf = [filePath_tum_pose stringByAppendingPathComponent:filename];
//                        [self checkFilePath:filePath_kf];
//
//                        Eigen::Vector3d relative_t=vins.front_pose.relative_t;
//                        Eigen::Matrix3d relative_q=vins.front_pose.relative_q;
//
//        //                [NSThread sleepForTimeInterval:30];
//                        //这里可能得加个锁 不安全
//                        NSMutableData *poseDataBuf_time_r_t = [[NSMutableData alloc] init];
//
//                        for(int a=0;a<3;a++){
//                            [poseDataBuf_time_r_t appendBytes:&(relative_t[a]) length:sizeof(double)];
//
//                        }
//                        for(int a=0;a<3;a++){
//                            for(int b=0;b<3;b++){
//                                double r=relative_q(a,b);
//                                [poseDataBuf_time_r_t appendBytes:&(r) length:sizeof(double)];
//
//                            }
//                        }
//                        BOOL writeImuSuccess_time=[poseDataBuf_time_r_t writeToFile:filePath_kf atomically:YES];
                       
                        break;
                    }
                }
                /*
                 ** update the keyframe pose when this frame slides out the window and optimize loop graph
                 */
//                cout<<"准备回环错误检测"<<endl;
                int search_cnt = 0;
                for(int i = 0; i < keyframe_database.size(); i++)
                {

                    search_cnt++;
                    KeyFrame* kf = keyframe_database.getLastKeyframe(i);

//                    cout<<fixed<<setprecision(0)<<"回环处："<<kf->header<<" "<<vins.Headers[0]<<" "<<kf->header - vins.Headers[0]<<" "<<i<<endl;

                    if(fabs(kf->header - vins.Headers[0])<0.00001)
//                    if(kf->header == vins.Headers[0])//当是滑动窗口里最老的一帧，更新它的原始位姿
                    {
//                        cout<<"glboal_index test header[0]="<<kf->global_index<<endl;
                        Eigen::Vector3d t;
                        Eigen::Matrix3d r;
                        kf->getPose(t, r);
//                        cout<<endl<<"pose:"<<t<<endl<<r<<"origin pose:"<<vins.Ps[0]<<endl<<vins.Rs[0]<<endl;;

                        kf->updateOriginPose(vins.Ps[0], vins.Rs[0]);
                        kf->IsOriginUpdate=true;
                        //update edge
                        // if loop happens in this frame, update pose graph;
                        if (kf->has_loop)
                        {

                            kf_global_index = kf->global_index;
                            start_global_optimization = true;
                            cout<<"检测到回环 并通过错误回环的检测："<<getTime()<<endl;
                        }
                        break;
                    }
                    else
                    {
                        if(search_cnt > 2 * WINDOW_SIZE)
                            break;
                    }
                }
//                cout<<"准备回环错误检测 end"<<endl;
                keyframe_freq++;
            }
        }
        else if(LOOP_CLOSURE_SERVER){
            static bool first_frame = true;
            if(vins.solver_flag != vins.NON_LINEAR)
                first_frame = true;
            if(vins.marginalization_flag == vins.MARGIN_OLD && vins.solver_flag == vins.NON_LINEAR && !image_buf_loop.empty())
            {
                first_frame = false;
                if(!first_frame && keyframe_freq % LOOP_FREQ == 0)
                {
//                    bool isKeyFrame=false;
                    keyframe_freq = 0;
                    /**
                     ** save the newest keyframe to the keyframe database
                     ** only need to save the pose to the keyframe database
                     **/
                    Eigen::Vector3d T_w_i = vins.Ps[WINDOW_SIZE - 2];
                    Eigen::Matrix3d R_w_i = vins.Rs[WINDOW_SIZE - 2];
                    m_image_buf_loop.lock();
                    while(!image_buf_loop.empty() && fabs(image_buf_loop.front().second-vins.Headers[WINDOW_SIZE - 2])>0.00001 &&image_buf_loop.front().second < (vins.Headers[WINDOW_SIZE - 2]+0.00001))
//                    while(!image_buf_loop.empty() && image_buf_loop.front().second < vins.Headers[WINDOW_SIZE - 2])
                    {
                        image_buf_loop.pop();
                    }
                    //assert(vins.Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second);
                    if(fabs(vins.Headers[WINDOW_SIZE - 2] - image_buf_loop.front().second)<0.00001)
//                    if(vins.Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second)
                    {
                        const char *pattern_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_pattern" ofType:@"yml"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
                        KeyFrame* keyframe = new KeyFrame(vins.Headers[WINDOW_SIZE - 2], global_frame_cnt, T_w_i, R_w_i, image_buf_loop.front().first, pattern_file, keyframe_database.cur_seg_index);
                        keyframe->setExtrinsic(vins.tic, vins.ric);
                        /*
                         ** we still need save the measurement to the keyframe(not database) for add connection with looped old pose
                         ** and save the pointcloud to the keyframe for reprojection search correspondance
                         */
                        keyframe->buildKeyFrameFeatures(vins);
                        keyframe_database.add(keyframe);//pose只有这里和全局优化那里会更新
                        keyframe->edge=vins.edge;
                        keyframe->edge_single=vins.edge_single;
                        keyframe->var_imu=vins.var_imu;
//                        printf("处理线程中%d\n",keyframe->global_index);
                        //把关键帧在这发出去 缺其它特征点和描述子

                        global_frame_cnt++;
//                        isKeyFrame=true;
                    }
                    m_image_buf_loop.unlock();
                    /**
                    if(isKeyFrame){
                        if(LOOP_CLOSURE_SERVER){
                            if(!vins.retrive_pose_data_server.empty()){
                    //            cout<<"retrive_pose_data_server 不空"<<endl;
                                if(!(fabs(vins.front_pose.header - vins.retrive_pose_data_server.front().header)<0.00001))
                    //            if(front_pose.header != retrive_pose_data_server.front().header)
                                {
                                    if((vins.front_pose.header - vins.Headers[0])>-0.00001 && vins.front_pose.isRemove!=0){
                                        vins.sendServer_relative=true;
                                        vins.relative_q_sendServer=vins.front_pose.relative_q;
                                        vins.relative_t_sendServer=vins.front_pose.relative_t;
                                        vins.relative_yaw_sendServer=vins.front_pose.relative_yaw;
                                        vins.relative_cur_index_sendServer=vins.front_pose.cur_index;
                                        vins.relative_pitch_sendServer=vins.front_pose.relative_pitch;
                                        vins.relative_roll_sendServer=vins.front_pose.relative_roll;
                                        vins.isRemove_sendServer=vins.front_pose.isRemove;
                                    }
                                    
                                    Eigen::Matrix<double, 8, 1> connected_info_test;
                                    connected_info_test <<vins.relative_t_sendServer.x(), vins.relative_t_sendServer.y(), vins.relative_t_sendServer.z(),
                                    vins.relative_q_sendServer.w(), vins.relative_q_sendServer.x(), vins.relative_q_sendServer.y(), vins.relative_q_sendServer.z(),vins.relative_yaw_sendServer;
                   
                                    
                                    vins.front_pose = vins.retrive_pose_data_server.front();  //need lock
                                    vins.retrive_pose_data_server.pop();
                                    
                                    if(!((vins.front_pose.header - vins.Headers[0])>-0.00001)){
                                        int num_loop=vins.retrive_pose_data_server.size();
                                        for(int i=0;i<num_loop;i++){
                                            vins.front_pose=vins.retrive_pose_data_server.front();
                                            vins.retrive_pose_data_server.pop();
                                            if((vins.front_pose.header -vins.Headers[0])>-0.00001)
                                                break;
                                        }
                                    }
                                    vins.front_pose.isRemove=2;
                    //                front_pose.sendRelativeData_server=true;
                                    
                  
                                    printf("use loop\n");
                    
                                }
                            }
                            if(!vins.front_pose.point_clouds_all.empty()){
                                double front_pose_header=vins.front_pose.header;
                                if((front_pose_header - vins.Headers[0])>-0.00001){
                                    localMapping(vins.front_pose.old_index ,global_frame_cnt-2,vins.front_pose.cur_index,vins.front_pose.loop_pose);
                                }
                            }
                        }
                    }
                     */
                }
                else
                {
                    first_frame = false;
                }
                // update loop info
                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    if(fabs(vins.Headers[i] - vins.front_pose.header)<0.00001)
//                    if(vins.Headers[i] == vins.front_pose.header)
                    {
                        //ljl
//                        if(vins.front_pose.measurements.empty())
//                            break;
//                        cout<<"检测过错误的回环没"<<endl;

                        KeyFrame* cur_kf = keyframe_database.getKeyframe(vins.front_pose.cur_index);
//                        暂时注释
//                        if (abs(vins.front_pose.relative_yaw) > 30.0 || vins.front_pose.relative_t.norm() > 10.0)
//                        {
//                            printf("Wrong loop%d\n",cur_kf->global_index);
////                            cout<<"vins.front_pose.relative_yaw="<<vins.front_pose.relative_yaw<<" "<<cur_kf->global_index<<endl;
////                            cout<<" vins.front_pose.relative_t.norm()="<< vins.front_pose.relative_t.norm()<<endl;
//                            cur_kf->removeLoop();
//                            vins.front_pose.isRemove=0;
//                            //告诉服务器 这个可以不用告诉服务器 因为没收到loop_info 他就不会用
////                            dispatch_async(dispatch_get_main_queue(), ^{
////                                sendErrorLoop(vins.front_pose.cur_index);
////                            });
//
//                            break;
//                        }
                        vins.front_pose.isRemove=2;
//                        else{//ljl
//                            KeyFrame* old_kf = keyframe_database.getKeyframe(vins.front_pose.old_index);
//                            Eigen::Matrix3d Rs_loop;
//                            Eigen::Vector3d Ps_loop;
//                            old_kf->getOriginPose(Ps_loop,Rs_loop);
//
//
////                            Eigen::Matrix3d Rs_i= vins.Rs[i];
////                            Eigen::Vector3d Ps_i= vins.Ps[i] ;
//                            Eigen::Matrix3d Rs_i;
//                            Eigen::Vector3d Ps_i;
//                            cur_kf->getOriginPose(Ps_i, Rs_i);
//
//                            Eigen::Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                            Eigen::Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
//                            double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
//
//                            Eigen::Vector3d real_t=-vins.front_pose.relative_q.toRotationMatrix()*relative_q.transpose()*relative_t+vins.front_pose.relative_t;
//                            double real_yaw_test_test=relative_yaw-vins.front_pose.relative_yaw;
//
//                            cout<<fixed<<setprecision(6)<<"打印一下相差多少："<<real_t.norm()<<" "<<real_yaw_test_test<<endl;
//                            if(real_t.norm()<0.03 && abs(real_yaw_test_test)<1.5){
//                                cout<<"测试需要减少权重 "<<cur_kf->global_index<<endl;
//                                //临时添加
////                                cur_kf->removeLoop();
////                                vins.front_pose.isRemove=true;
////                                break;
//
//                                //暂时注释
//                                vins.front_pose.isRemove=1;
//                            }else{
//                                vins.front_pose.isRemove=2;
//                            }
//                        }
                        
//                        暂时注释
//                        cur_kf->updateLoopConnection(vins.front_pose.relative_t,                                                     vins.front_pose.relative_q,                                                     vins.front_pose.relative_yaw);
                        break;
                    }
                }

                //这里表示 在当前滑动窗口中发生了两次回环，前一个回环 还没有到滑动窗口第一帧，但是relative_pose不再更新了，所以这个时候要更新出去
                //暂时注释 想不起来 这个的意义了
                //有新的回环匹配了
                //这里暂时注释 不需要发送相对位姿
//                if(vins.sendServer_relative){
//                    //第一个数据不要发 因为这是第一次进来 其实是没有数据的
//                    if(vins.relative_cur_index_sendServer!=0){
////                        cout<<"front_pose sendServer_relative 被取代过 "<<endl;
//                        vins.sendServer_relative=false;
//                        dispatch_async(dispatch_get_main_queue(), ^{
//
//                            if(vins.isRemove_sendServer==2){
//                                sendFrontPoseRelative_add_pitch_roll(vins.relative_t_sendServer,vins.relative_yaw_sendServer,vins.relative_cur_index_sendServer,vins.relative_pitch_sendServer,vins.relative_roll_sendServer);
//                            }
////                            else if(vins.isRemove_sendServer==1){
//////cout<<"降权重 发送过"<<endl;
////                                sendFrontPoseRelative_add_pitch_roll_forRemove(vins.relative_t_sendServer,vins.relative_yaw_sendServer,vins.relative_cur_index_sendServer,vins.relative_pitch_sendServer,vins.relative_roll_sendServer);
////                            }
////                            cout<<"relative_cur_index_sendServer"<<vins.relative_cur_index_sendServer<<" "<<vins.relative_t_sendServer.x()<<" "<<vins.relative_t_sendServer.y()<<" "<<vins.relative_t_sendServer.z()<<" "<<vins.relative_yaw_sendServer<<" "<<vins.relative_pitch_sendServer<<" "<<vins.relative_roll_sendServer<<endl;
//                        });
//                    }
//                }
//
//                //说明里面有数据了
//                if(!vins.front_pose.measurements.empty())
//                {
//                    //the retrive pose is in the current window
//                    if(fabs(vins.front_pose.header - vins.Headers[0])<0.00001 && vins.front_pose.isRemove!=0 )
////                    if(vins.front_pose.header == vins.Headers[0])
//                    {
//
//                        dispatch_async(dispatch_get_main_queue(), ^{
//                            //先改成下面那样 多发一个pitch 和 roll
////                            sendFrontPoseRelative(vins.front_pose.relative_t,vins.front_pose.relative_q,vins.front_pose.relative_yaw,vins.front_pose.cur_index);
//
//                            if(vins.front_pose.isRemove==2){
//                                sendFrontPoseRelative_add_pitch_roll(vins.front_pose.relative_t,vins.front_pose.relative_yaw,vins.front_pose.cur_index,vins.front_pose.relative_pitch,vins.front_pose.relative_roll);
//                            }
////                            else if(vins.front_pose.isRemove==1){
////                               cout<<"降权重 发送过"<<endl;
////                                //这个并没有作用
////                                //忘记之前为什么注释这个没有作用了 按理，这个应该是发送降权重的边， 除非是否定了降权重的想法
////                                sendFrontPoseRelative_add_pitch_roll_forRemove(vins.front_pose.relative_t,vins.front_pose.relative_yaw,vins.front_pose.cur_index,vins.front_pose.relative_pitch,vins.front_pose.relative_roll);
////                            }
////                            cout<<"vins.front_pose.cur_index"<<vins.front_pose.cur_index<<" "<<vins.front_pose.relative_t.x()<<" "<<vins.front_pose.relative_t.y()<<" "<<vins.front_pose.relative_t.z()<<" "<<vins.front_pose.relative_yaw<<" "<<vins.front_pose.relative_pitch<<" "<<vins.front_pose.relative_roll<<endl;
//                        });
//
//
//                    }
//
//                }

                /*
                 ** update the keyframe pose when this frame slides out the window and optimize loop graph
                 */
                int search_cnt = 0;
                for(int i = 0; i < keyframe_database.size(); i++)
                {
                    search_cnt++;
                    KeyFrame* kf = keyframe_database.getLastKeyframe(i);
                    if(fabs(kf->header - vins.Headers[0])<0.00001)
//                    if(kf->header == vins.Headers[0])
                    {
//                        cout<<"glboal_index test header[0]="<<kf->global_index<<endl;
                        Eigen::Vector3d t;
                        Eigen::Matrix3d r;
                        kf->getPose(t, r);
//                        cout<<endl<<"pose:"<<t<<endl<<r<<"origin pose:"<<vins.Ps[0]<<endl<<vins.Rs[0]<<endl;;

                        kf->updateOriginPose(vins.Ps[0], vins.Rs[0]);
                        kf->IsOriginUpdate=true;
                        //发送窗口不再优化的位姿 需要改
                        dispatch_async(dispatch_get_main_queue(), ^{

                            sendKeyFrameOriginPose(kf);
                        });

                        //update edge
                        // if loop happens in this frame, update pose graph;
                        if (kf->has_loop)
                        {
                            kf_global_index = kf->global_index;
                            start_global_optimization = true;

                        }
                        break;
                    }
                    else
                    {
                        if(search_cnt > 2 * WINDOW_SIZE)
                            break;
                    }
                }
                keyframe_freq++;
            }
        }
        else if(LOOP_CLOSURE_SERVER_noLoop){
            static bool first_frame = true;
            if(vins.solver_flag != vins.NON_LINEAR)
                first_frame = true;
            if(vins.marginalization_flag == vins.MARGIN_OLD && vins.solver_flag == vins.NON_LINEAR && !image_buf_loop.empty())
            {
                first_frame = false;
                if(!first_frame && keyframe_freq % LOOP_FREQ == 0)
                {
                    keyframe_freq = 0;
                    /**
                     ** save the newest keyframe to the keyframe database
                     ** only need to save the pose to the keyframe database
                     **/
                    Eigen::Vector3d T_w_i = vins.Ps[WINDOW_SIZE - 2];
                    Eigen::Matrix3d R_w_i = vins.Rs[WINDOW_SIZE - 2];
                    m_image_buf_loop.lock();
                    while(!image_buf_loop.empty() && fabs(image_buf_loop.front().second-vins.Headers[WINDOW_SIZE - 2])>0.00001 &&image_buf_loop.front().second < (vins.Headers[WINDOW_SIZE - 2]+0.00001))
//                    while(!image_buf_loop.empty() && image_buf_loop.front().second < vins.Headers[WINDOW_SIZE - 2])
                    {
                        image_buf_loop.pop();
                    }
                    //assert(vins.Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second);
                    if(fabs(vins.Headers[WINDOW_SIZE - 2] - image_buf_loop.front().second)<0.00001)
//                    if(vins.Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second)
                    {
                        const char *pattern_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_pattern" ofType:@"yml"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
                        KeyFrame* keyframe = new KeyFrame(vins.Headers[WINDOW_SIZE - 2], global_frame_cnt, T_w_i, R_w_i, image_buf_loop.front().first, pattern_file, keyframe_database.cur_seg_index);
                        keyframe->setExtrinsic(vins.tic, vins.ric);
                        /*
                         ** we still need save the measurement to the keyframe(not database) for add connection with looped old pose
                         ** and save the pointcloud to the keyframe for reprojection search correspondance
                         */
                        keyframe->buildKeyFrameFeatures(vins);
                        keyframe_database.add(keyframe);//pose只有这里和全局优化那里会更新

                        //把关键帧在这发出去 缺其它特征点和描述子
                        featuretracker.cur_kf_priorMap=keyframe;

                        global_frame_cnt++;

                    }
                    m_image_buf_loop.unlock();

                }
                else
                {
                    first_frame = false;
                }
                // update loop info
                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    if(fabs(vins.Headers[i] - vins.front_pose.header)<0.00001)
//                    if(vins.Headers[i] == vins.front_pose.header)
                    {
                        //ljl
//                        if(vins.front_pose.measurements.empty())
//                            break;
//                        cout<<"检测过错误的回环没"<<endl;

                        KeyFrame* cur_kf = keyframe_database.getKeyframe(vins.front_pose.cur_index);
//                        暂时注释
//                        if (abs(vins.front_pose.relative_yaw) > 30.0 || vins.front_pose.relative_t.norm() > 10.0)
//                        {
//                            printf("Wrong loop%d\n",cur_kf->global_index);
////                            cout<<"vins.front_pose.relative_yaw="<<vins.front_pose.relative_yaw<<" "<<cur_kf->global_index<<endl;
////                            cout<<" vins.front_pose.relative_t.norm()="<< vins.front_pose.relative_t.norm()<<endl;
//                            cur_kf->removeLoop();
//                            vins.front_pose.isRemove=0;
//                            //告诉服务器 这个可以不用告诉服务器 因为没收到loop_info 他就不会用
////                            dispatch_async(dispatch_get_main_queue(), ^{
////                                sendErrorLoop(vins.front_pose.cur_index);
////                            });
//
//                            break;
//                        }
                        vins.front_pose.isRemove=2;
//                        else{//ljl
//                            KeyFrame* old_kf = keyframe_database.getKeyframe(vins.front_pose.old_index);
//                            Eigen::Matrix3d Rs_loop;
//                            Eigen::Vector3d Ps_loop;
//                            old_kf->getOriginPose(Ps_loop,Rs_loop);
//
//
////                            Eigen::Matrix3d Rs_i= vins.Rs[i];
////                            Eigen::Vector3d Ps_i= vins.Ps[i] ;
//                            Eigen::Matrix3d Rs_i;
//                            Eigen::Vector3d Ps_i;
//                            cur_kf->getOriginPose(Ps_i, Rs_i);
//
//                            Eigen::Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                            Eigen::Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
//                            double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
//
//                            Eigen::Vector3d real_t=-vins.front_pose.relative_q.toRotationMatrix()*relative_q.transpose()*relative_t+vins.front_pose.relative_t;
//                            double real_yaw_test_test=relative_yaw-vins.front_pose.relative_yaw;
//
//                            cout<<fixed<<setprecision(6)<<"打印一下相差多少："<<real_t.norm()<<" "<<real_yaw_test_test<<endl;
//                            if(real_t.norm()<0.03 && abs(real_yaw_test_test)<1.5){
//                                cout<<"测试需要减少权重 "<<cur_kf->global_index<<endl;
//                                //临时添加
////                                cur_kf->removeLoop();
////                                vins.front_pose.isRemove=true;
////                                break;
//
//                                //暂时注释
//                                vins.front_pose.isRemove=1;
//                            }else{
//                                vins.front_pose.isRemove=2;
//                            }
//                        }
                        
//                        暂时注释
//                        cur_kf->updateLoopConnection(vins.front_pose.relative_t,                                                     vins.front_pose.relative_q,                                                     vins.front_pose.relative_yaw);
                        break;
                    }
                }

                //这里表示 在当前滑动窗口中发生了两次回环，前一个回环 还没有到滑动窗口第一帧，但是relative_pose不再更新了，所以这个时候要更新出去
                //暂时注释 想不起来 这个的意义了
                //有新的回环匹配了
                //这里暂时注释 不需要发送相对位姿
//                if(vins.sendServer_relative){
//                    //第一个数据不要发 因为这是第一次进来 其实是没有数据的
//                    if(vins.relative_cur_index_sendServer!=0){
////                        cout<<"front_pose sendServer_relative 被取代过 "<<endl;
//                        vins.sendServer_relative=false;
//                        dispatch_async(dispatch_get_main_queue(), ^{
//
//                            if(vins.isRemove_sendServer==2){
//                                sendFrontPoseRelative_add_pitch_roll(vins.relative_t_sendServer,vins.relative_yaw_sendServer,vins.relative_cur_index_sendServer,vins.relative_pitch_sendServer,vins.relative_roll_sendServer);
//                            }
////                            else if(vins.isRemove_sendServer==1){
//////cout<<"降权重 发送过"<<endl;
////                                sendFrontPoseRelative_add_pitch_roll_forRemove(vins.relative_t_sendServer,vins.relative_yaw_sendServer,vins.relative_cur_index_sendServer,vins.relative_pitch_sendServer,vins.relative_roll_sendServer);
////                            }
////                            cout<<"relative_cur_index_sendServer"<<vins.relative_cur_index_sendServer<<" "<<vins.relative_t_sendServer.x()<<" "<<vins.relative_t_sendServer.y()<<" "<<vins.relative_t_sendServer.z()<<" "<<vins.relative_yaw_sendServer<<" "<<vins.relative_pitch_sendServer<<" "<<vins.relative_roll_sendServer<<endl;
//                        });
//                    }
//                }
//
//                //说明里面有数据了
//                if(!vins.front_pose.measurements.empty())
//                {
//                    //the retrive pose is in the current window
//                    if(fabs(vins.front_pose.header - vins.Headers[0])<0.00001 && vins.front_pose.isRemove!=0 )
////                    if(vins.front_pose.header == vins.Headers[0])
//                    {
//
//                        dispatch_async(dispatch_get_main_queue(), ^{
//                            //先改成下面那样 多发一个pitch 和 roll
////                            sendFrontPoseRelative(vins.front_pose.relative_t,vins.front_pose.relative_q,vins.front_pose.relative_yaw,vins.front_pose.cur_index);
//
//                            if(vins.front_pose.isRemove==2){
//                                sendFrontPoseRelative_add_pitch_roll(vins.front_pose.relative_t,vins.front_pose.relative_yaw,vins.front_pose.cur_index,vins.front_pose.relative_pitch,vins.front_pose.relative_roll);
//                            }
////                            else if(vins.front_pose.isRemove==1){
////                               cout<<"降权重 发送过"<<endl;
////                                //这个并没有作用
////                                //忘记之前为什么注释这个没有作用了 按理，这个应该是发送降权重的边， 除非是否定了降权重的想法
////                                sendFrontPoseRelative_add_pitch_roll_forRemove(vins.front_pose.relative_t,vins.front_pose.relative_yaw,vins.front_pose.cur_index,vins.front_pose.relative_pitch,vins.front_pose.relative_roll);
////                            }
////                            cout<<"vins.front_pose.cur_index"<<vins.front_pose.cur_index<<" "<<vins.front_pose.relative_t.x()<<" "<<vins.front_pose.relative_t.y()<<" "<<vins.front_pose.relative_t.z()<<" "<<vins.front_pose.relative_yaw<<" "<<vins.front_pose.relative_pitch<<" "<<vins.front_pose.relative_roll<<endl;
//                        });
//
//
//                    }
//
//                }

                /*
                 ** update the keyframe pose when this frame slides out the window and optimize loop graph
                 */
                int search_cnt = 0;
                for(int i = 0; i < keyframe_database.size(); i++)
                {
                    search_cnt++;
                    KeyFrame* kf = keyframe_database.getLastKeyframe(i);
                    if(fabs(kf->header - vins.Headers[0])<0.00001)
//                    if(kf->header == vins.Headers[0])
                    {
//                        cout<<"glboal_index test header[0]="<<kf->global_index<<endl;
                        Eigen::Vector3d t;
                        Eigen::Matrix3d r;
                        kf->getPose(t, r);
//                        cout<<endl<<"pose:"<<t<<endl<<r<<"origin pose:"<<vins.Ps[0]<<endl<<vins.Rs[0]<<endl;;

                        kf->updateOriginPose(vins.Ps[0], vins.Rs[0]);
                        kf->IsOriginUpdate=true;
                        //发送窗口不再优化的位姿 需要改
                        dispatch_async(dispatch_get_main_queue(), ^{

                            sendKeyFrameOriginPose(kf);
                        });

                        //update edge
                        // if loop happens in this frame, update pose graph;
                        if (kf->has_loop)
                        {
                            kf_global_index = kf->global_index;
                            start_global_optimization = true;

                        }
                        break;
                    }
                    else
                    {
                        if(search_cnt > 2 * WINDOW_SIZE)
                            break;
                    }
                }
                keyframe_freq++;
            }
        }
        waiting_lists--;
//        cout<<"process end"<<endl;
        
        //finish solve one frame
        [self performSelectorOnMainThread:@selector(showInputView) withObject:nil waitUntilDone:YES];
    }
}


/*
 Loop detection thread: this thread detect loop for newest keyframe and retrieve features
 */
//原始版本 备份
-(void)loop_thread{

    if(LOOP_CLOSURE && loop_closure == NULL)
    {
        NSLog(@"loop start load voc");
        TS(load_voc);
        const char *voc_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_k10L6" ofType:@"bin"]
                                cStringUsingEncoding:[NSString defaultCStringEncoding]];
        loop_closure = new LoopClosure(voc_file, COL, ROW);
        TE(load_voc);
        NSLog(@"loop load voc finish");

        voc_init_ok = true;
    }
    while(![[NSThread currentThread] isCancelled] )
    {
        if(!LOOP_CLOSURE)
        {
            [NSThread sleepForTimeInterval:0.5];
            continue;
        }

        bool loop_succ = false;
        if (loop_check_cnt < global_frame_cnt)
        {
            KeyFrame* cur_kf = keyframe_database.getLastKeyframe();
            //assert(loop_check_cnt == cur_kf->global_index);
            loop_check_cnt++;
            if(cur_kf->check_loop==0){
                cur_kf->check_loop = 1;

                cv::Mat current_image;
                current_image = cur_kf->image;

                std::vector<cv::Point2f> measurements_old;
                std::vector<cv::Point2f> measurements_old_norm;
                std::vector<cv::Point2f> measurements_cur;
                std::vector<int> features_id;
                std::vector<cv::Point2f> measurements_cur_origin = cur_kf->measurements;

                vector<cv::Point2f> cur_pts;
                vector<cv::Point2f> old_pts;
                if(isUndistorted){
                    cur_kf->extractBrief(current_image,featuretracker);//提取描述符 和 关键点
                }else{
                    cur_kf->extractBrief(current_image);//提取描述符 和 关键点
                }
//                printf("loop extract %d feature\n", cur_kf->keypoints.size());
//                printf("win feature %d\n",cur_kf->getWinPointsSize());

                //发送给服务器 关键帧信息，做回环检测用
    //            dispatch_async(dispatch_get_main_queue(), ^{
    //
    //                sendKeyFrame(cur_kf);
    //            });


                loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index,cur_kf->global_index);//检测到回环的存在

    //            cout<<"cur_kf->keypoints="<<cur_kf->keypoints.size()<<" cur_kf->descriptors="<<cur_kf->descriptors.size()<<" cur_pts="<<cur_pts.size()<<" old_pts="<<old_pts.size()<<" old_index="<<old_index<<" global_index "<<cur_kf->global_index<<endl;

                if(loop_succ)
                {
                    KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);//拿到回环帧
                    if (old_kf == NULL)
                    {
                        printf("NO such %dth frame in keyframe_database\n", old_index);
                        assert(false);
                    }

                    assert(old_index!=-1);

                    Eigen::Vector3d T_w_i_old;
                    Eigen::Matrix3d R_w_i_old;

                    old_kf->getPose(T_w_i_old, R_w_i_old);
                    cur_kf->findConnectionWithOldFrame(old_kf, cur_pts, old_pts,
                                                       measurements_old, measurements_old_norm);//找到匹配的特征点
                    measurements_cur = cur_kf->measurements;
                    features_id = cur_kf->features_id;

                    printf("%d loop succ with %drd image %d\n",cur_kf->global_index, old_index,measurements_old_norm.size());

                    if(measurements_old_norm.size()>MIN_LOOP_NUM)
                    {

                        Eigen::Quaterniond Q_loop_old(R_w_i_old);
                        RetriveData retrive_data;
                        retrive_data.cur_index = cur_kf->global_index;
                        retrive_data.header = cur_kf->header;
                        retrive_data.P_old = T_w_i_old;
                        retrive_data.Q_old = Q_loop_old;
                        retrive_data.use = true;
                        retrive_data.measurements = measurements_old_norm;//回环帧 检测的那500点 匹配上的id
                        retrive_data.features_ids = features_id;//当前帧的匹配的特征点的全局id
                        //ljl
                        retrive_data.old_index=old_index;
                        vins.retrive_pose_data = (retrive_data);

                        //cout << "old pose " << T_w_i_old.transpose() << endl;
                        //cout << "refinded pose " << T_w_i_refine.transpose() << endl;
                        // add loop edge in current frame
                        cur_kf->detectLoop(old_index);//这个之后 才会全局优化
                        keyframe_database.addLoop(old_index);
                        old_kf->is_looped = 1;
                        loop_old_index = old_index;

//                        cout<<"检测到回环："<<getTime()<<endl;
//                        cout<<fixed<<setprecision(0)<<"匹配的两帧的header:"<<cur_kf->header<<" "<<old_kf->header<<"匹配的点数："<<measurements_old_norm.size()<<endl;

                    }
                }
                cur_kf->image.release();

//                int cur_id=cur_kf->global_index;
//                NSString *filename = [NSString stringWithFormat:@"%lu", cur_id];
//                NSString *filePath_kf = [documentsPath_kf stringByAppendingPathComponent:filename];
//
//                Eigen::Matrix3d r;
//                Eigen::Vector3d t;
//                [self checkFilePath:filePath_kf];
//                NSMutableData *kf_buf = [[NSMutableData alloc] init];
//                vector<cv::KeyPoint> kf_keypoint=cur_kf->keypoints;
//                vector<BRIEF::bitset> kf_descriptors=cur_kf->descriptors;
//                vector<cv::Point2f> kf_measurements_origin=cur_kf->measurements_origin;
//                vector<Eigen::Vector3d> kf_point_clouds_origin=cur_kf->point_clouds_origin;
//                vector<int> kf_features_id_origin=cur_kf->features_id_origin;
//                cur_kf->getPose(t,r);
//
//                [kf_buf appendBytes:&(cur_id) length:sizeof(int)];
//                [kf_buf appendBytes:&(cur_kf->header) length:sizeof(double)];
//                for(int a=0;a<3;a++){
//                    [kf_buf appendBytes:&(t[a]) length:sizeof(double)];
//                }
//
//                for(int a=0;a<3;a++){
//                    for(int b=0;b<3;b++){
//                        [kf_buf appendBytes:&(r(a,b)) length:sizeof(double)];
//                    }
//                }
//
//                int keypoint_len=kf_keypoint.size();
//                cout<<"keypoint_len="<<keypoint_len<<endl;
//
//                cout<<"kf_buf.length"<<kf_buf.length<<endl;
//                [kf_buf appendBytes:&(keypoint_len) length:sizeof(int)];
//                for(int a=0;a<keypoint_len;a++){
//                    cv::KeyPoint kf_key=kf_keypoint[a];
//                    [kf_buf appendBytes:&(kf_key.pt.x) length:sizeof(float)];
//                    [kf_buf appendBytes:&(kf_key.pt.y) length:sizeof(float)];
//                    [kf_buf appendBytes:&(kf_key.size) length:sizeof(float)];
//                    [kf_buf appendBytes:&(kf_key.angle) length:sizeof(float)];
//                    [kf_buf appendBytes:&(kf_key.response) length:sizeof(float)];
//                    [kf_buf appendBytes:&(kf_key.octave) length:sizeof(int)];
//                    [kf_buf appendBytes:&(kf_key.class_id) length:sizeof(int)];
////                        cout<<"kf_key.pt.x="<<kf_key.pt.x<<endl;
//                }
//
//                for(int a=0;a<keypoint_len;a++){
//                    string str;
//                    to_string(kf_descriptors[a],str);
////                        cout<<"str_des="<<str<<endl;
//                    for(auto ite=str.rbegin(),ite_end=str.rend();ite!=ite_end;ite++){
//                        [kf_buf appendBytes:&(*ite) length:sizeof(char)];//后面用的时候，要翻转过来 才是真正的描述符 翻过来了
//                    }
//
//                    assert(str.size()==256);
//                }
//
//                cout<<"kf_buf.length"<<kf_buf.length<<endl;
//
//                cout<<"keypoint_len="<<keypoint_len<<endl;
//                int measure_len=kf_measurements_origin.size();
//                cout<<"measure_len"<<measure_len<<endl;
//                [kf_buf appendBytes:&(measure_len) length:sizeof(int)];
//                for(int a=0;a<measure_len;a++){
//                    cv::Point2f kf_mea=kf_measurements_origin[a];
//                    [kf_buf appendBytes:&(kf_mea.x) length:sizeof(float)];
//                    [kf_buf appendBytes:&(kf_mea.y) length:sizeof(float)];
//                }
//
//                assert(kf_point_clouds_origin.size()==measure_len);
//                for(int a=0;a<measure_len;a++){
//                    Eigen::Vector3d kf_point=kf_point_clouds_origin[a];
//                    for(int b=0;b<3;b++){
//                        [kf_buf appendBytes:&(kf_point[b]) length:sizeof(double)];
//                    }
//                }
//
//                assert(kf_features_id_origin.size()==measure_len);
//                for(int a=0;a<measure_len;a++){
//                    [kf_buf appendBytes:&(kf_features_id_origin[a]) length:sizeof(int)];
//                }
//                cout<<"kf_buf.length end"<<kf_buf.length<<endl;
//                BOOL writeImuSuccess_time=[kf_buf writeToFile:filePath_kf atomically:YES];
//                if(!writeImuSuccess_time){
//                        NSLog(@"kf已保存完整fail");
//                }
            }

        }

        if(loop_succ)
            [NSThread sleepForTimeInterval:2.0];
        [NSThread sleepForTimeInterval:0.05];
    }
    //[self process_loop_detection];
}

-(void)loop_thread2{

    if(LOOP_CLOSURE && loop_closure == NULL)
    {
        NSLog(@"loop start load voc");
        TS(load_voc);
        const char *voc_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_k10L6" ofType:@"bin"]
                                cStringUsingEncoding:[NSString defaultCStringEncoding]];
        loop_closure = new LoopClosure(voc_file, COL, ROW);
        TE(load_voc);
        NSLog(@"loop load voc finish");

        voc_init_ok = true;
    }
    while(![[NSThread currentThread] isCancelled] )
    {
        if(!LOOP_CLOSURE)
        {
            [NSThread sleepForTimeInterval:0.5];
            continue;
        }

        bool loop_succ = false;
        if (loop_check_cnt < global_frame_cnt)
        {
            KeyFrame* cur_kf = keyframe_database.getLastKeyframe();
            //assert(loop_check_cnt == cur_kf->global_index);
            loop_check_cnt++;
            if(cur_kf->check_loop==0){
                cur_kf->check_loop = 1;

                cv::Mat current_image;
                current_image = cur_kf->image;

                std::vector<cv::Point2f> measurements_old;
                std::vector<cv::Point2f> measurements_old_norm;
                std::vector<cv::Point2f> measurements_cur;
                std::vector<int> features_id;
                std::vector<cv::Point2f> measurements_cur_origin = cur_kf->measurements;

                vector<cv::Point2f> cur_pts;
                vector<cv::Point2f> old_pts;
                if(isUndistorted){
                    cur_kf->extractBrief(current_image,featuretracker);//提取描述符 和 关键点
                }else{
                    cur_kf->extractBrief(current_image);//提取描述符 和 关键点
                }


                assert(cur_kf->measurements_origin.size()==cur_kf->features_id_origin.size());
                loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index,cur_kf->global_index);//检测到回环的存在



                if(loop_succ)
                {
                    KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);//拿到回环帧
                    if (old_kf == NULL)
                    {
                        printf("NO such %dth frame in keyframe_database\n", old_index);
                        assert(false);
                    }

                    assert(old_index!=-1);

//                    Eigen::Vector3d T_w_i_old;
//                    Eigen::Matrix3d R_w_i_old;
//
//                    old_kf->getPose(T_w_i_old, R_w_i_old);
                    
//                    新帧3d 老帧2d
                    cur_kf->findConnectionWithOldFrame(old_kf, cur_pts, old_pts,
                                                       measurements_old, measurements_old_norm);//找到匹配的特征点
                    measurements_cur = cur_kf->measurements;
                    features_id = cur_kf->features_id;
                    
//                    新帧2d 老帧3d
//                    old_kf->findConnectionWithOldFrame(cur_kf,old_pts, cur_pts,
//                                                       measurements_old, measurements_old_norm);//找到匹配的特征点
//                    measurements_cur = old_kf->measurements;
//                    features_id = old_kf->features_id;

                    printf("%d loop succ with %drd image %d\n",cur_kf->global_index, old_index,measurements_old_norm.size());

                    if(measurements_old_norm.size()>MIN_LOOP_NUM)
                    {
                        //                    新帧3d 老帧2d
                        relative_mutex.lock();
                        mlvv_point_clouds_single.push_back(cur_kf->point_clouds);
                        mlvv_measurements_old_norm_single.push_back(measurements_old_norm);
                        mlp_kf_pairs.push_back(std::make_pair(cur_kf, old_kf));
                        relative_mutex.unlock();
                        [self loop_thread_continue2];
                       
                        
                        //                    新帧2d 老帧3d
//                        relative_mutex.lock();
//                        mlvv_point_clouds_single.push_back(old_kf->point_clouds);
//                        mlvv_measurements_old_norm_single.push_back(measurements_old_norm);
//                        mlp_kf_pairs.push_back(std::make_pair(old_kf,cur_kf));
//                        relative_mutex.unlock();
//                        [self loop_thread_continue2];
                        
//                        Eigen::Quaterniond Q_loop_old(R_w_i_old);
//                        RetriveData retrive_data;
//                        retrive_data.cur_index = cur_kf->global_index;
//                        retrive_data.header = cur_kf->header;
//                        retrive_data.P_old = T_w_i_old;
//                        retrive_data.Q_old = Q_loop_old;
//                        retrive_data.use = true;
//                        retrive_data.measurements = measurements_old_norm;//回环帧 检测的那500点 匹配上的id
//                        retrive_data.features_ids = features_id;//当前帧的匹配的特征点的全局id
//                        //ljl
//                        retrive_data.old_index=old_index;
//                        vins.retrive_pose_data = (retrive_data);
//
//                        //cout << "old pose " << T_w_i_old.transpose() << endl;
//                        //cout << "refinded pose " << T_w_i_refine.transpose() << endl;
//                        // add loop edge in current frame
//                        cur_kf->detectLoop(old_index);//这个之后 才会全局优化
//                        keyframe_database.addLoop(old_index);
//                        old_kf->is_looped = 1;
//                        loop_old_index = old_index;

//                        cout<<"检测到回环："<<getTime()<<endl;
//                        cout<<fixed<<setprecision(0)<<"匹配的两帧的header:"<<cur_kf->header<<" "<<old_kf->header<<"匹配的点数："<<measurements_old_norm.size()<<endl;

                    }
                }
                cur_kf->image.release();

            }

        }

        if(loop_succ)
            [NSThread sleepForTimeInterval:2.0];
        [NSThread sleepForTimeInterval:0.05];
    }
    //[self process_loop_detection];
}
//老帧的2D点 新帧的3D点
-(void)loop_thread_continue{
   
    relative_mutex.lock();
    vector<cv::Point2f> measurements_old_norm_real=mlvv_measurements_old_norm_single.front();//反一下 当前帧的2d点
    mlvv_measurements_old_norm_single.pop_front();
    vector<Eigen::Vector3d> point_3d_cur_real=mlvv_point_clouds_single.front();//老帧3d点
    mlvv_point_clouds_single.pop_front();
    std::pair<KeyFrame*, KeyFrame*> kf_pair=mlp_kf_pairs.front();
    mlp_kf_pairs.pop_front();
    KeyFrame* cur_kf=kf_pair.first;//老帧
    KeyFrame* old_kf=kf_pair.second;//当前帧
    relative_mutex.unlock();
    
    
    
    Eigen::Matrix3d ric_curClient= vins.ric;
    Eigen::Vector3d tic_curClient=vins.tic;
    Eigen::Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
    const float &fx = FOCUS_LENGTH_X;
    const float &fy = FOCUS_LENGTH_Y;
    const float &cx = PX;
    const float &cy = PY;
    
    Eigen::Matrix3d R_relative;
    Eigen::Vector3d T_relative;
//    新帧3d点
    Eigen::Matrix3d oldKF_r;
    Eigen::Vector3d oldKF_t;
    old_kf->getOriginPose(oldKF_t, oldKF_r);//old_kf 用的当前帧，0720改成cur_kf老帧
    
//    Eigen::Matrix3d oldKF_r_flag;
//    Eigen::Vector3d oldKF_t_flag;
//    cur_kf->getOriginPose(oldKF_t_flag, oldKF_r_flag);
    
    {
        //先验换一下 因为当前的位姿，是乘了偏移的
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
//        新帧3D点
        t_array[0] = oldKF_t(0);
        t_array[1] = oldKF_t(1);
        t_array[2] = oldKF_t(2);
        //将矩阵转换为向量
        Eigen::Vector3d euler_angle_old = Utility::R2ypr(oldKF_r);
        
//        t_array[0] = oldKF_t_flag(0);
//        t_array[1] = oldKF_t_flag(1);
//        t_array[2] = oldKF_t_flag(2);
//        //将矩阵转换为向量
//        Eigen::Vector3d euler_angle_old = Utility::R2ypr(oldKF_r_flag);
        
        euler_array[0] = euler_angle_old.x();
        euler_array[1] = euler_angle_old.y();
        euler_array[2] = euler_angle_old.z();
        problem.AddParameterBlock(euler_array, 3);
        problem.AddParameterBlock(t_array, 3);

        for(int a=0,b=point_3d_cur_real.size();a<b;a++){
            //找到主地图那个点 所在帧的位姿
            Eigen::Vector3d pts_i = point_3d_cur_real[a];

            //相机平面坐标
            cv::Point2f pt=measurements_old_norm_real[a];
            float xx=pt.x;
            float yy=pt.y;

             Eigen::Vector2d pts_j ;//要求是归一化图像坐标
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
            return;
        }
//                    cur_kf->solveRelativePoseByPnP_3(measurements_old_norm_real ,R_relative, T_relative,point_3d_cur_real,false);
//                }
//                else{
        //这里代表是 在当前帧的误差基础上，老帧的世界位姿
            T_relative[0]=t_array[0];
            T_relative[1]=t_array[1];
            T_relative[2]=t_array[2];

            R_relative=Utility::ypr2R(Eigen::Vector3d(euler_array[0],euler_array[1],euler_array[2]));
//                }
    }
    
    vector<double> header_cur_all;
    vector<double> header_old_all;
    vector<vector<cv::Point2f>> measurements_old_norm_all;
    vector<vector<Eigen::Vector3d>> point_clouds_all;
    vector<vector<int>> feature_id_cur_all;
    vector<vector<int>> feature_id_old_all;
    std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
    std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
    std::vector<Eigen::Vector3d> point_3d_cur;
    std::vector<cv::Point2f> measurements_cur;//像素坐标
    std::vector<int> feature_id_cur;
    std::vector<int> feature_id_old;
    
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
    
    vector<Eigen::Matrix3d> oldR_b_a;
    vector<Eigen::Vector3d> oldT_b_a;
    
//    老帧3d
//    Eigen::Matrix3d oldKF_r;
//    Eigen::Vector3d oldKF_t;
//    old_kf->getOriginPose(oldKF_t, oldKF_r);//old_kf 用的当前帧，0720改成cur_kf老帧
    
    int temp_index=0;
    //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
    for(KeyFrame* pkFi_old: vpCovKFi_old){
        if(temp_index!=0){
            Eigen::Matrix3d rwi_old;
            Eigen::Vector3d twi_old;
            pkFi_old->getOriginPose(twi_old, rwi_old);
            
            Eigen::Matrix3d r_b_a;
            Eigen::Vector3d t_b_a;
            r_b_a=oldKF_r.transpose()* rwi_old;
            t_b_a=oldKF_r.transpose()*(twi_old-oldKF_t);
            oldR_b_a.push_back(r_b_a);
            oldT_b_a.push_back(t_b_a);
        }else{
            oldR_b_a.push_back(Eigen::Matrix3d::Identity());
            oldT_b_a.push_back(Eigen::Vector3d::Zero());
            temp_index++;//放这里就只要执行一次
        }
    }
    
    int similarNum=0;
    temp_index=0;
    vector<Eigen::Matrix3d> old_r;
    vector<Eigen::Vector3d> old_t;
    vector<int> vpkf_index;
    
    for(KeyFrame* pkf_old:vpCovKFi_old){
        Eigen::Matrix3d r_w_imuOld=R_relative*oldR_b_a[temp_index];
        Eigen::Vector3d t_w_imuOld=R_relative*oldT_b_a[temp_index] +T_relative;
        Eigen::Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
        Eigen::Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
        Eigen::Matrix3d r_camOld_w=r_w_camOld.transpose();
        old_r.push_back(r_camOld_w);
        old_t.push_back(-r_camOld_w*t_w_camOld);
        temp_index++;
        
    }
    
//    while(!cur_kf->IsOriginUpdate){
//        usleep(50);
//    }
    std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
    cout<<"共视帧的数量"<<cur_kf->mvpOrderedConnectedKeyFrames.size()<<" , "<<cur_kf->global_index<<endl;
    vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
    vpCovKFi_cur[0] = cur_kf;
    
    
    {
        //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
        header_cur_all.clear();
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
            //描述符得全有 这里肯定有
            if(!pKFi->is_des_end){
                continue;
            }
            
            vector<int > feature_id_origin_cur=pKFi->features_id_origin;
            vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
            std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
            vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
            int nPoints = point_clouds_origin_cur.size();
            int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
            assert(feature_id_origin_cur.size()==point_clouds_origin_cur.size());
            
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
                //描述符得全有 可能会有被跳过的
                if(!pkFi_old->is_des_end){
                    temp_index++;
                    continue;
                }
                
                measurements_old_coarse.clear();
                measurements_old_norm_coarse.clear();
                point_3d_cur.clear();
                measurements_cur.clear();
            
                vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;
                vector<int > feature_id_origin_old=pkFi_old->features_id_origin;
                int nPoints_old = feature_id_origin_old.size();
                int point2D_len_old=keypoints_old.size()-nPoints_old;
                
                //这个得到的是到imu坐标系的位姿
                Eigen::Matrix3d r_camOld_w=old_r[temp_index];
                Eigen::Vector3d t_camOld_w=old_t[temp_index];
                
                
                for(int i=0;i<nPoints;i++){
                    Eigen::Vector3d point_main=point_clouds_origin_cur[i];
                    //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                    Eigen::Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
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
                        feature_id_cur.push_back(feature_id_origin_cur[i]);
                        if(bestIndex<point2D_len_old){
                            feature_id_old.push_back(-1);//后续加上去
                        }else{
                            int index=bestIndex-point2D_len_old;
                            feature_id_old.push_back(feature_id_origin_old[index]);
                        }
                        
                    }
                                
                }
                
//                        cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                if(measurements_cur.size()>=22){
                    pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur,feature_id_cur);
//                            cout<<"f矩阵拒绝后的点数："<<measurements_old_coarse.size()<<endl;
                    if(measurements_old_coarse.size()>=16){
                    
                        cv::Point2f norm_pt;
                        int pCount=measurements_old_coarse.size();
                        for(int aa=0;aa<pCount;aa++){
                            norm_pt.x = (measurements_old_coarse[aa].x -  PX)/ FOCUS_LENGTH_X;
                            norm_pt.y = (measurements_old_coarse[aa].y -  PY)/ FOCUS_LENGTH_Y;
                            measurements_old_coarse[aa]=norm_pt;
                            
                        }
                        header_cur_all.push_back(pKFi->header);
                        header_old_all.push_back(pkFi_old->header);
                        point_clouds_all.push_back(point_3d_cur);
                        measurements_old_norm_all.push_back(measurements_old_coarse);
                        feature_id_cur_all.push_back(feature_id_cur);
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
        Eigen::Quaterniond q_array[optiKf_num];
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

        
        Eigen::Matrix3d curKF_r;
        Eigen::Vector3d curKF_t;
        old_kf->getOriginPose(curKF_t, curKF_r);
        
        
        Eigen::Matrix3d tmp_r_old;
        Eigen::Vector3d tmp_t_old;
        t_array[0][0] = curKF_t(0);
        t_array[0][1] = curKF_t(1);
        t_array[0][2] = curKF_t(2);
        //将矩阵转换为向量
        Eigen::Vector3d euler_angle_old = Utility::R2ypr(curKF_r);
        euler_array[0][0] = euler_angle_old.x();
        euler_array[0][1] = euler_angle_old.y();
        euler_array[0][2] = euler_angle_old.z();
        problem.AddParameterBlock(euler_array[0], 3);
        problem.AddParameterBlock(t_array[0], 3);

        
        //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
        map<int,int> resample;
        vector<int> record_kfIndex;
        
        temp_index=0;
        for(int len=vpkf_index.size(); temp_index<len; temp_index++ ){
            
            int kf_index=temp_index+1;
            Eigen::Matrix3d relative_r_b_a;
            Eigen::Vector3d relative_t_b_a;
            relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
            relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
            Eigen::Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
            
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
               
                ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0));
                problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                
            }
            record_kfIndex.push_back(kf_index);

            vector<Eigen::Vector3d> point_single=point_clouds_all[temp_index];
            vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
            for(int a=0,b=point_single.size();a<b;a++){

                //找到主地图那个点 所在帧的位姿
                Eigen::Vector3d pts_i = point_single[a];

                //相机平面坐标
                cv::Point2f pt=measure_single[a];

                ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pt.x, pt.y, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2));
                problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                
             }
        }
        
        
        ceres::Solve(options, &problem, &summary);
              

        if(summary.termination_type==ceres::CONVERGENCE){
           
                
//                    cout<<"检测到回环： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;

            Eigen::Matrix3d Rs_i ;
            Eigen::Vector3d Ps_i ;//当前帧
            cur_kf->getOriginPose(Ps_i, Rs_i);
            



            Eigen::Vector3d q;
            q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
            Eigen::Matrix3d Rs_loop = Utility::ypr2R(q);
            Eigen::Vector3d Ps_loop = Eigen::Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
            
            vector<Eigen::Vector3d> ps_loop_all;
            vector<Eigen::Quaterniond> rs_loop_all;
            
            ps_loop_all.push_back(Ps_loop);
            Eigen::Quaterniond Q_loop_kf_cur(Rs_loop);
            rs_loop_all.push_back(Q_loop_kf_cur);
            
            assert(record_kfIndex.size()==measurements_old_norm_all.size());
            for(int a=0,len=record_kfIndex.size(); a<len; a++ ){
                int kf_index=record_kfIndex[a];
                Eigen::Vector3d q_kf;
                q_kf<<euler_array[kf_index][0],euler_array[kf_index][1],euler_array[kf_index][2];
                Eigen::Matrix3d Rs_loop_kf = Utility::ypr2R(q_kf);
                Eigen::Vector3d Ps_loop_kf = Eigen::Vector3d( t_array[kf_index][0],  t_array[kf_index][1],  t_array[kf_index][2]);
                Ps_loop_kf=Rs_loop*Ps_loop_kf+Ps_loop;
                Rs_loop_kf=Rs_loop*Rs_loop_kf;
                ps_loop_all.push_back(Ps_loop_kf);
                Eigen::Quaterniond Q_loop_kf(Rs_loop_kf);
                rs_loop_all.push_back(Q_loop_kf);
            }

           
            Eigen::Vector3d relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
//                    Eigen::Matrix3d relative_q = Rs_loop.transpose() * Rs_i;
            double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
            double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_i).y() - Utility::R2ypr(Rs_loop).y());
            double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_i).z() - Utility::R2ypr(Rs_loop).z());
            
            Eigen::Quaterniond rs_loop_q=Eigen::Quaterniond(Rs_loop),rs_i_q=Eigen::Quaterniond(Rs_i);
            Eigen::Quaterniond rela_q=rs_loop_q.inverse()*rs_i_q;
            
            
            cur_kf->relative_pitch=relative_pitch;
            cur_kf->relative_roll=relative_roll;
            cur_kf->updateLoopConnection(relative_t, relative_yaw);
            cur_kf->loop_info_better_q=rela_q;
//                    cur_kf->loop_info_better_q=relative_q;
            
        
            
            
//            数据换过来 新帧3d
            //先临时搬过来
            old_index=old_kf->global_index;
            Eigen::Vector3d T_w_i_old;
            Eigen::Matrix3d R_w_i_old;
            old_kf->getOriginPose(T_w_i_old, R_w_i_old);
            RetriveData retrive_data;
            retrive_data.cur_index = cur_kf->global_index;
            retrive_data.header = cur_kf->header;
            Eigen::Quaterniond Q_loop_old(R_w_i_old);
            retrive_data.P_old = T_w_i_old;
            retrive_data.Q_old = Q_loop_old;
//            Eigen::Quaterniond Q_loop_old(Rs_loop);
//            retrive_data.P_old = Ps_loop;
//            retrive_data.Q_old = Q_loop_old;
            retrive_data.use = true;
            
           
            
            retrive_data.measurements = measurements_old_norm_all[0];
            //这里暂时不给值  因为暂时不发送到客户端
//                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
            retrive_data.features_ids = feature_id_cur_all[0];//当前帧的匹配的特征点的全局id
            retrive_data.old_index=old_kf->global_index;

            retrive_data.header_all=header_cur_all;
            retrive_data.features_ids_all=feature_id_cur_all;
            retrive_data.measurements_all=measurements_old_norm_all;
            
           
            retrive_data.P_old_all=ps_loop_all;
            retrive_data.Q_old_all=rs_loop_all;

            vins.retrive_pose_data = (retrive_data);
//                        vins->isSendLoopData=true;

            cur_kf->detectLoop(old_kf->global_index);
            keyframe_database.addLoop(old_kf->global_index);//todo 这里面 改一下
            old_kf->is_looped = 1;
            loop_old_index = old_kf->global_index;
           
            
//            数据换过来 老帧3d
            //先临时搬过来
//            old_index=cur_kf->global_index;
//            Eigen::Vector3d T_w_i_old;
//            Eigen::Matrix3d R_w_i_old;
//            cur_kf->getOriginPose(T_w_i_old, R_w_i_old);
//            Eigen::Quaterniond Q_loop_old(R_w_i_old);
//            RetriveData retrive_data;
//            retrive_data.cur_index = old_kf->global_index;
//            retrive_data.header = old_kf->header;
//            retrive_data.P_old = T_w_i_old;
//            retrive_data.Q_old = Q_loop_old;
////            Eigen::Quaterniond Q_loop_old(Rs_loop);
////            retrive_data.P_old = Ps_loop;
////            retrive_data.Q_old = Q_loop_old;
//            retrive_data.use = true;
////            retrive_data.measurements = measurements_old_norm_all[0];
//            //这里暂时不给值  因为暂时不发送到客户端
////                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
////            retrive_data.features_ids = feature_id_cur_all[0];//当前帧的匹配的特征点的全局id
//            retrive_data.old_index=cur_kf->global_index;
//
//            retrive_data.header_all=header_old_all;
//            retrive_data.features_ids_all=feature_id_old_all;
//            retrive_data.measurements_all=measurements_old_norm_all;
//            retrive_data.point_clouds_all=point_clouds_all;
//            retrive_data.P_old_all=ps_loop_all;
//            retrive_data.Q_old_all=rs_loop_all;
//
//            vins.retrive_pose_data = (retrive_data);
////                        vins->isSendLoopData=true;
//
//            old_kf->detectLoop(cur_kf->global_index);
//            keyframe_database.addLoop(cur_kf->global_index);//todo 这里面 改一下
//            cur_kf->is_looped = 1;
//            loop_old_index = cur_kf->global_index;
            
//            vins->isSendLoop_another=true;
//            if(cur_kf->IsOriginUpdate==true){
//               if (cur_kf->sendLoop==false)
//               {
//
//                   cur_kf->sendLoop=true;
//                   vins->globalOpti_index_mutex.lock();
//                   vins->kf_global_index.push(cur_kf->global_index);
//                   vins->start_kf_global_index.push(cur_kf->loop_index);
//                   poseGraph->latest_loop_index=cur_kf->global_index;
//                   vins->start_global_optimization = true;//先暂停回环 不启动
//
//                   vins->globalOpti_index_mutex.unlock();
////                           cout<<"慢了 慢了检测到回环 并通过错误回环的检测："<<cur_kf->global_index<<endl;
//               }
//           }
//            cur_kf->is_get_loop_info=true;
            


        }
        else{
                    cout<<"地图内部 检测失败 未收敛： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
            ;
        }
    }else{
                cout<<"地图内部 回环检测的数量不够:"<<similarNum<<endl;
        ;
    }
    cout<<"loop_thread_continue end3551"<<endl;

}
//新帧的2D点 老帧的3D点
-(void)loop_thread_continue2{
    relative_mutex.lock();
    vector<cv::Point2f> measurements_old_norm_real=mlvv_measurements_old_norm_single.front();//反一下 当前帧的2d点
    mlvv_measurements_old_norm_single.pop_front();
    vector<Eigen::Vector3d> point_3d_cur_real=mlvv_point_clouds_single.front();//老帧3d点
    mlvv_point_clouds_single.pop_front();
    std::pair<KeyFrame*, KeyFrame*> kf_pair=mlp_kf_pairs.front();
    mlp_kf_pairs.pop_front();
    KeyFrame* old_kf=kf_pair.first;// 当前帧
    KeyFrame* cur_kf =kf_pair.second;//老帧
    relative_mutex.unlock();
    
    Eigen::Matrix3d ric_curClient= vins.ric;
    Eigen::Vector3d tic_curClient=vins.tic;
    Eigen::Vector3d euler_angle_ric_main=Utility::R2ypr(ric_curClient);
    const float &fx = FOCUS_LENGTH_X;
    const float &fy = FOCUS_LENGTH_Y;
    const float &cx = PX;
    const float &cy = PY;
    
    Eigen::Matrix3d R_relative;
    Eigen::Vector3d T_relative;

    Eigen::Matrix3d oldKF_r_flag;
    Eigen::Vector3d oldKF_t_flag;
//    新帧3d点
    cur_kf->getOriginPose(oldKF_t_flag, oldKF_r_flag);//old_kf 用的当前帧，0720改成cur_kf老帧
//    老帧3d点
//    cur_kf->getOriginPose(oldKF_t_flag, oldKF_r_flag);
    
    {
        //先验换一下 因为当前的位姿，是乘了偏移的
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
        t_array[0] = oldKF_t_flag(0);
        t_array[1] = oldKF_t_flag(1);
        t_array[2] = oldKF_t_flag(2);
        //将矩阵转换为向量
        Eigen::Vector3d euler_angle_old = Utility::R2ypr(oldKF_r_flag);
        euler_array[0] = euler_angle_old.x();
        euler_array[1] = euler_angle_old.y();
        euler_array[2] = euler_angle_old.z();
        problem.AddParameterBlock(euler_array, 3);
        problem.AddParameterBlock(t_array, 3);

        for(int a=0,b=point_3d_cur_real.size();a<b;a++){
            //找到主地图那个点 所在帧的位姿
            Eigen::Vector3d pts_i = point_3d_cur_real[a];

            //相机平面坐标
            cv::Point2f pt=measurements_old_norm_real[a];
            float xx=pt.x;
            float yy=pt.y;

             Eigen::Vector2d pts_j ;//要求是归一化图像坐标
             pts_j<<xx,yy;

            ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection2::Create( pts_i.x(), pts_i.y(), pts_i.z(),pts_j.x(), pts_j.y(), tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z());
            block_id =  problem.AddResidualBlock(cost_function, loss_function, euler_array,t_array);
            residual_block_ids.push_back( block_id );
            
         }

     
        
        std::vector< ceres::ResidualBlockId > residual_block_ids_temp;
        residual_block_ids_temp.reserve( residual_block_ids.size() );

  
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

        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.max_num_iterations = 20;
//                options.minimizer_progress_to_stdout = false;
//                options.check_gradients = false;
//                options.gradient_check_relative_precision = 1e-10;

        ceres::Solve( options, &problem, &summary );
//                std::cout <<"地图内部 算一个粗糙的相对位姿："<< summary.BriefReport() << "\n";
        

        if(summary.termination_type!=ceres::
           CONVERGENCE){
            return;
        }

        //这里代表是 在当前帧的误差基础上，老帧的世界位姿
            T_relative[0]=t_array[0];
            T_relative[1]=t_array[1];
            T_relative[2]=t_array[2];

            R_relative=Utility::ypr2R(Eigen::Vector3d(euler_array[0],euler_array[1],euler_array[2]));
        
        Eigen::Matrix3d curKF_r_flag;
        Eigen::Vector3d curKF_t_flag;
        old_kf->getOriginPose(curKF_t_flag, curKF_r_flag);
        
        R_relative=curKF_r_flag.transpose()*R_relative;
        T_relative=curKF_r_flag.transpose()*(T_relative-curKF_t_flag);
        R_relative=oldKF_r_flag*R_relative.transpose();
        T_relative=-R_relative*T_relative+oldKF_t_flag;

    }
    
    vector<double> header_cur_all;
    vector<double> header_old_all;
    vector<vector<cv::Point2f>> measurements_old_norm_all;
    vector<vector<Eigen::Vector3d>> point_clouds_all;
    vector<vector<int>> feature_id_cur_all;
    vector<vector<int>> feature_id_old_all;
    std::vector<cv::Point2f> measurements_old_coarse;//像素坐标
    std::vector<cv::Point2f> measurements_old_norm_coarse;//图像坐标
    std::vector<Eigen::Vector3d> point_3d_cur;
    std::vector<cv::Point2f> measurements_cur;//像素坐标
    std::vector<int> feature_id_cur;
    std::vector<int> feature_id_old;
    
    std::vector<KeyFrame*> vpCovKFi_cur = cur_kf->GetBestCovisibilityKeyFrames(5);
    cout<<"共视帧的数量"<<cur_kf->mvpOrderedConnectedKeyFrames.size()<<" , "<<cur_kf->global_index<<endl;
    vpCovKFi_cur.push_back(vpCovKFi_cur[0]);
    vpCovKFi_cur[0] = cur_kf;
    
//    while(!old_kf->IsOriginUpdate){
//        usleep(50);
//    }
//            cout<<"cur_kf id:"<<cur_kf->global_index<<" , " << old_kf->global_index<<endl;
    vector<KeyFrame*> vpCovKFi_old=old_kf->GetBestCovisibilityKeyFrames(5);
//            可能存在和第0帧共视了 第0帧没有共视帧
    if(vpCovKFi_old.size()==0){
        vpCovKFi_old.push_back(old_kf);
    }else{
        vpCovKFi_old.push_back(vpCovKFi_old[0]);//为空
        vpCovKFi_old[0]=old_kf;
    }
    
    
    vector<Eigen::Matrix3d> oldR_b_a;
    vector<Eigen::Vector3d> oldT_b_a;
    
//    老帧3d
    Eigen::Matrix3d oldKF_r;
    Eigen::Vector3d oldKF_t;
    old_kf->getOriginPose(oldKF_t, oldKF_r);//old_kf 用的当前帧，0720改成cur_kf老帧
    
    int temp_index=0;
    //先算出来 在当前帧误差下，世界坐标系 到 侯选帧的共视帧的位姿关系
    for(KeyFrame* pkFi_old: vpCovKFi_old){
        if(temp_index!=0){
            Eigen::Matrix3d rwi_old;
            Eigen::Vector3d twi_old;
            pkFi_old->getOriginPose(twi_old, rwi_old);
            
            Eigen::Matrix3d r_b_a;
            Eigen::Vector3d t_b_a;
            r_b_a=oldKF_r.transpose()* rwi_old;
            t_b_a=oldKF_r.transpose()*(twi_old-oldKF_t);
            oldR_b_a.push_back(r_b_a);
            oldT_b_a.push_back(t_b_a);
        }else{
            oldR_b_a.push_back(Eigen::Matrix3d::Identity());
            oldT_b_a.push_back(Eigen::Vector3d::Zero());
            temp_index++;//放这里就只要执行一次
        }
    }
    
    int similarNum=0;
    temp_index=0;
    vector<Eigen::Matrix3d> old_r;
    vector<Eigen::Vector3d> old_t;
    vector<int> vpkf_index;
    
    for(KeyFrame* pkf_old:vpCovKFi_old){
        Eigen::Matrix3d r_w_imuOld=R_relative*oldR_b_a[temp_index];
        Eigen::Vector3d t_w_imuOld=R_relative*oldT_b_a[temp_index] +T_relative;
        Eigen::Matrix3d r_w_camOld=r_w_imuOld*ric_curClient;
        Eigen::Vector3d t_w_camOld=r_w_imuOld*tic_curClient+t_w_imuOld;
        Eigen::Matrix3d r_camOld_w=r_w_camOld.transpose();
        old_r.push_back(r_camOld_w);
        old_t.push_back(-r_camOld_w*t_w_camOld);
        temp_index++;
        
    }
    
    
    {
        //当前帧的共视帧 与 侯选帧的2级共视帧的匹配，通过重投影找
        header_cur_all.clear();
        point_clouds_all.clear();
        measurements_old_norm_all.clear();
        vpkf_index.clear();
        feature_id_cur_all.clear();
        feature_id_old.clear();
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
            //描述符得全有 这里肯定有
            if(!pKFi->is_des_end){
                continue;
            }
            
            vector<int > feature_id_origin_cur=pKFi->features_id_origin;
            vector<Eigen::Vector3d> point_clouds_origin_cur= pKFi->point_clouds_origin;
            std::vector<cv::Point2f> measurements_origin_cur=pKFi->measurements_origin;
            vector<cv::KeyPoint> keypoints_cur=pKFi->keypoints;
            int nPoints = point_clouds_origin_cur.size();
            int point2D_len_cur=keypoints_cur.size()-point_clouds_origin_cur.size();
            assert(feature_id_origin_cur.size()==point_clouds_origin_cur.size());
            
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
                //描述符得全有 可能会有被跳过的
                if(!pkFi_old->is_des_end){
                    temp_index++;
                    continue;
                }
                
                measurements_old_coarse.clear();
                measurements_old_norm_coarse.clear();
                point_3d_cur.clear();
                measurements_cur.clear();
            
                vector<cv::KeyPoint> keypoints_old=pkFi_old->keypoints;
                vector<int > feature_id_origin_old=pkFi_old->features_id_origin;
                int nPoints_old = feature_id_origin_old.size();
                int point2D_len_old=keypoints_old.size()-nPoints_old;
                
                //这个得到的是到imu坐标系的位姿
                Eigen::Matrix3d r_camOld_w=old_r[temp_index];
                Eigen::Vector3d t_camOld_w=old_t[temp_index];
                
                
                for(int i=0;i<nPoints;i++){
                    Eigen::Vector3d point_main=point_clouds_origin_cur[i];
                    //把主地图下 世界坐标系的点 转到小地图上一帧的图像坐标系上
                    Eigen::Vector3d p3D_c2=r_camOld_w*point_main+t_camOld_w;
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
//                        feature_id_cur.push_back(feature_id_origin_cur[i]);
                        if(bestIndex<point2D_len_old){
                            feature_id_old.push_back(-1);//后续加上去
                        }else{
                            int index=bestIndex-point2D_len_old;
                            feature_id_old.push_back(feature_id_origin_old[index]);
                        }
                        
                    }
                                
                }
                
//                        cout<<"50个像素内,找到的点数："<<point_3d_cur.size()<<endl;
                if(measurements_cur.size()>=22){
                    pKFi->rejectWithF_server_mapFuse(measurements_cur, measurements_old_coarse,point_3d_cur,feature_id_old);
//                            cout<<"f矩阵拒绝后的点数："<<measurements_old_coarse.size()<<endl;
                    if(measurements_old_coarse.size()>=16){
                    
                        cv::Point2f norm_pt;
                        int pCount=measurements_old_coarse.size();
                        for(int aa=0;aa<pCount;aa++){
                            norm_pt.x = (measurements_old_coarse[aa].x -  PX)/ FOCUS_LENGTH_X;
                            norm_pt.y = (measurements_old_coarse[aa].y -  PY)/ FOCUS_LENGTH_Y;
                            measurements_old_coarse[aa]=norm_pt;
                            
                        }
                        header_cur_all.push_back(pKFi->header);
                        header_old_all.push_back(pkFi_old->header);
                        point_clouds_all.push_back(point_3d_cur);
                        measurements_old_norm_all.push_back(measurements_old_coarse);
//                        feature_id_cur_all.push_back(feature_id_cur);
                        feature_id_old_all.push_back(feature_id_old);
                        
//                            num+=point_3d_cur.size();
//                            real_vpCovKFi_cur.push_back(pkFi_old);
                        vpkf_index.push_back(temp_index);
                        similarNum++;
//                                pkFi_old->isuse=1;

//                                break;
                    }
                    
                }
                
                temp_index++;
//                    ends=clock();
//                    cout<<"为一个帧投影一次 找匹配，花费的时间："<<ends-start<<endl;
            }
        }
    }
    


    if(similarNum>=3){
        int optiKf_num=vpkf_index.size()+2;
        //构造优化问题
        double t_array[optiKf_num][3];//平移数组，其中存放每个关键帧的平移向量
        Eigen::Quaterniond q_array[optiKf_num];
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

        
        Eigen::Matrix3d curKF_r;
        Eigen::Vector3d curKF_t;
        old_kf->getOriginPose(curKF_t, curKF_r);
        
        
        Eigen::Matrix3d tmp_r_old;
        Eigen::Vector3d tmp_t_old;
        t_array[0][0] = T_relative(0);
        t_array[0][1] = T_relative(1);
        t_array[0][2] = T_relative(2);
        //将矩阵转换为向量
        Eigen::Vector3d euler_angle_old = Utility::R2ypr(R_relative);
        euler_array[0][0] = euler_angle_old.x();
        euler_array[0][1] = euler_angle_old.y();
        euler_array[0][2] = euler_angle_old.z();
        problem.AddParameterBlock(euler_array[0], 3);
        problem.AddParameterBlock(t_array[0], 3);

        
        //再加一个根据当前帧与侯选帧之间的匹配关系，构造最开始的那个3d点 到候选帧的误差
        map<int,int> resample;
        vector<int> record_kfIndex;
        
        temp_index=0;
        for(int len=vpkf_index.size(); temp_index<len; temp_index++ ){
            
            int kf_index=temp_index+1;
            Eigen::Matrix3d relative_r_b_a;
            Eigen::Vector3d relative_t_b_a;
            relative_r_b_a=oldR_b_a[vpkf_index[temp_index]];
            relative_t_b_a=oldT_b_a[vpkf_index[temp_index]];
            Eigen::Vector3d relative_r_b_a_euler=Utility::R2ypr(relative_r_b_a);
            
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
                
//                if(vpkf_index[temp_index]==0){
//                    problem.SetParameterBlockConstant(euler_array[kf_index]);
//                    problem.SetParameterBlockConstant(t_array[kf_index]);
//                }
               
                ceres::CostFunction* cost_function = FourSixDOFError::Create( relative_t_b_a(0),relative_t_b_a(1),relative_t_b_a(2),relative_r_b_a_euler(0));
                problem.AddResidualBlock(cost_function, loss_function, euler_array[kf_index],t_array[kf_index]);
                
            }
            record_kfIndex.push_back(kf_index);

            vector<Eigen::Vector3d> point_single=point_clouds_all[temp_index];
            vector<cv::Point2f> measure_single=measurements_old_norm_all[temp_index];
            for(int a=0,b=point_single.size();a<b;a++){

                //找到主地图那个点 所在帧的位姿
                Eigen::Vector3d pts_i = point_single[a];

                //相机平面坐标
                cv::Point2f pt=measure_single[a];

                ceres::CostFunction* cost_function = FourSixDOFWeightError_reprojection_3::Create( pts_i.x(), pts_i.y(), pts_i.z(),pt.x, pt.y, tic_curClient[0], tic_curClient[1], tic_curClient[2], euler_angle_ric_main.x(), euler_angle_ric_main.y(), euler_angle_ric_main.z() ,relative_r_b_a_euler(1),relative_r_b_a_euler(2));
                problem.AddResidualBlock(cost_function, loss_function, euler_array[0],t_array[0], euler_array[kf_index],t_array[kf_index]);
                
             }
        }
        
        
        ceres::Solve(options, &problem, &summary);

        if(summary.termination_type==ceres::CONVERGENCE){

            Eigen::Matrix3d Rs_i ;
            Eigen::Vector3d Ps_i ;//当前帧
            cur_kf->getOriginPose(Ps_i, Rs_i);
            
            Eigen::Vector3d q;
            q<<euler_array[0][0],euler_array[0][1],euler_array[0][2];
            Eigen::Matrix3d Rs_loop = Utility::ypr2R(q);
            Eigen::Vector3d Ps_loop = Eigen::Vector3d( t_array[0][0],  t_array[0][1],  t_array[0][2]);
            
            vector<Eigen::Vector3d> ps_loop_all;
            vector<Eigen::Quaterniond> rs_loop_all;
            
//            ps_loop_all.push_back(Ps_loop);
//            Eigen::Quaterniond Q_loop_kf_cur(Rs_loop);
//            rs_loop_all.push_back(Q_loop_kf_cur);
            
            assert(record_kfIndex.size()==measurements_old_norm_all.size());
            for(int a=0,len=record_kfIndex.size(); a<len; a++ ){
                int kf_index=record_kfIndex[a];
                Eigen::Vector3d q_kf;
                q_kf<<euler_array[kf_index][0],euler_array[kf_index][1],euler_array[kf_index][2];
                Eigen::Matrix3d Rs_loop_kf = Utility::ypr2R(q_kf);
                Eigen::Vector3d Ps_loop_kf = Eigen::Vector3d( t_array[kf_index][0],  t_array[kf_index][1],  t_array[kf_index][2]);
                Ps_loop_kf=Rs_loop*Ps_loop_kf+Ps_loop;
                Rs_loop_kf=Rs_loop*Rs_loop_kf;
                ps_loop_all.push_back(Ps_loop_kf);
                Eigen::Quaterniond Q_loop_kf(Rs_loop_kf);
                rs_loop_all.push_back(Q_loop_kf);
            }
            
            Eigen::Vector3d relative_t = Rs_i.transpose() * (Ps_loop - Ps_i);
            double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).x() - Utility::R2ypr(Rs_i).x());
            double relative_pitch = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).y() - Utility::R2ypr(Rs_i).y());
            double relative_roll = Utility::normalizeAngle(Utility::R2ypr(Rs_loop).z() - Utility::R2ypr(Rs_i).z());
            
            Eigen::Quaterniond rs_loop_q=Eigen::Quaterniond(Rs_loop),rs_i_q=Eigen::Quaterniond(Rs_i);
            Eigen::Quaterniond rela_q=rs_i_q.inverse()*rs_loop_q;
            
            
            old_kf->relative_pitch=relative_pitch;
            old_kf->relative_roll=relative_roll;
            old_kf->updateLoopConnection(relative_t, relative_yaw);
            old_kf->loop_info_better_q=rela_q;
            old_kf->updateLoopConnection(relative_t,rela_q, relative_yaw);
            
            
//            数据换过来 老帧3d
            //先临时搬过来
            old_index=cur_kf->global_index;
            Eigen::Vector3d T_w_i_old;
            Eigen::Matrix3d R_w_i_old;
            cur_kf->getOriginPose(T_w_i_old, R_w_i_old);
            Eigen::Quaterniond Q_loop_old(R_w_i_old);
            RetriveData retrive_data;
            retrive_data.cur_index = old_kf->global_index;
            retrive_data.header = old_kf->header;
            retrive_data.P_old = T_w_i_old;
            retrive_data.Q_old = Q_loop_old;
//            Eigen::Quaterniond Q_loop_old(Rs_loop);
//            retrive_data.P_old = Ps_loop;
//            retrive_data.Q_old = Q_loop_old;
            retrive_data.use = true;
//            retrive_data.measurements = measurements_old_norm_all[0];
            //这里暂时不给值  因为暂时不发送到客户端
//                retrive_data.features_ids = features_id_cur_real;//这个并没有赋值 赋值了
//            retrive_data.features_ids = feature_id_cur_all[0];//当前帧的匹配的特征点的全局id
            retrive_data.old_index=cur_kf->global_index;

            retrive_data.header_all=header_old_all;
            retrive_data.features_ids_all=feature_id_old_all;//暂时注释
            retrive_data.measurements_all=measurements_old_norm_all;
            retrive_data.point_clouds_all=point_clouds_all;
            retrive_data.P_old_all=ps_loop_all;
            retrive_data.Q_old_all=rs_loop_all;
//            assert(header_old_all.size()==ps_loop_all.size());

            vins.retrive_pose_data = (retrive_data);

            old_kf->detectLoop(cur_kf->global_index);
            keyframe_database.addLoop(cur_kf->global_index);//todo 这里面 改一下
            cur_kf->is_looped = 1;
            loop_old_index = cur_kf->global_index;
            
            kf_global_index=old_kf->global_index;
            start_global_optimization = true;
            
        }
        else{
                    cout<<"地图内部 检测失败 未收敛： old_index="<<old_kf->global_index<<" , cur_index="<<cur_kf->global_index<<endl;
            ;
        }
    }else{
                cout<<"地图内部 回环检测的数量不够:"<<similarNum<<endl;
        ;
    }
    cout<<"loop_thread_continue end3551"<<endl;

}

/*
 GLobal Pose graph thread: optimize global pose graph based on realative pose from vins and update the keyframe database
 */
-(void)globalLoopThread{
    while (![[NSThread currentThread] isCancelled])
    {
        if(start_global_optimization)
        {
            cout<<"检测到回环 并准备开始全局优化："<<getTime()<<endl;
            start_global_optimization = false;
            TS(loop_thread);
            keyframe_database.optimize4DoFLoopPoseGraph(kf_global_index,
                                                        loop_correct_t,
                                                        loop_correct_r);
//            keyframe_database.optimize4DoFLoopPoseGraph_my1(kf_global_index,
//            loop_correct_t,
//            loop_correct_r);

            vins.t_drift = loop_correct_t;
            vins.r_drift = loop_correct_r;
            TE(loop_thread);
            cout<<"检测到回环 并完成一次全局优化："<<getTime()<<endl;
            [NSThread sleepForTimeInterval:1.17];
        }
        [NSThread sleepForTimeInterval:0.03];
    }
}
vector<int> keys_sum,bits_nature,bits_encode;
vector<double> time_compress,time_compress_decode;

vector<int> bytes_sum_keyframe,bytes_keys_keyframe,bytes_des_keyframe;


//临时测试 是否能稳定压缩---------
struct HuffmanNode {
    int weight; // 权重，出现的次数或者频率
    char ch; // 存储符号
    string code; // 存储该符号对应的编码
    int leftChild, rightChild, parent; // 左、右孩子，父结点
};
 
class HuffmanCode {
public:
    HuffmanCode(string str); // 构造函数
    ~HuffmanCode(); // 析构函数
    void getMin(int &first, int &second, int parent); // 选取两个较小的元素
    void Merge(int first, int second, int parent); // 合并
    string Encode(); // 编码:利用哈夫曼编码原理对数据进行加密
    string Decode(string str); // 解码
    string real_encode(string str);
private:
    HuffmanNode *HuffmanTree; // 数组
    int leafSize; // 统计不同字符的个数
};
 
// 构造函数
HuffmanCode::HuffmanCode(string str) {
    int len = (int)str.size(); // 字符串的长度
    int arr[256], i; // 存储字符串各个字符的个数
    HuffmanTree = new HuffmanNode[256]; // 动态分配空间
    
    // 1.初始化HuffmanTree数组
    for(i = 0; i < (2 * len - 1); i++) { // 叶子结点为len,则树最多有2*len-1个结点
        HuffmanTree[i].leftChild = HuffmanTree[i].rightChild = HuffmanTree[i].parent = -1;
        HuffmanTree[i].code = "";
    }
    // 2.统计输入的字符串的各个字符出现的个数
    memset(arr, 0, sizeof(arr)); // 清零
    for(i = 0; i < len; i++) // 统计次数
        arr[str[i]]++; // str[i] -> 转成对应的ASCII码，如'0'->48
    leafSize = 0; // 出现不同字符的个数
    for(i = 0; i < 256; i++) {
        if(arr[i] != 0) { // 有出现的字符
            // cout << "字符:" << (char)i << "次数为：" << arr[i] << endl;
            HuffmanTree[leafSize].ch = (char)i; // 将数字转成对应的字符
            HuffmanTree[leafSize].weight = arr[i]; // 权重
            leafSize++;
        }
    }
    
    // 3.选取两个较小值合并
    int first, second; // 两个较小的结点
    for(i = leafSize; i < (2*leafSize-1); i++) { // 做leafSize-1趟
        getMin(first, second, i); // 选取两个较小的元素
        Merge(first,second,i); // 合并
    }
}
 
// 析构函数
HuffmanCode::~HuffmanCode() {
    delete []HuffmanTree;
}
 
// 选取权值两个较小的元素
void HuffmanCode::getMin(int &first, int &second, int parent) {
    double weight = 0;
    int i;
    
    // 找权重最小元素
    for(i = 0; i < parent; i++) {
        if(HuffmanTree[i].parent != -1) // 已选过，直接跳过
            continue;
        if(weight == 0) { // 第一次找到没选过的结点
            weight = HuffmanTree[i].weight;
            first = i;
        }
        else if(HuffmanTree[i].weight < weight) { // 权值更小
            weight = HuffmanTree[i].weight;
            first = i;
        }
    }
    // 找权重次小元素
    weight = 0;
    for(i = 0; i < parent; i++) {
        if(HuffmanTree[i].parent != -1 || i == first) // 已选过，直接跳过
            continue;
        if(weight == 0) { // 第一次找到没选过的结点
            weight = HuffmanTree[i].weight;
            second = i;
        }
        else if(HuffmanTree[i].weight < weight) { // 权值更小
            weight = HuffmanTree[i].weight;
            second = i;
        }
    }
}
 
// 合并
void HuffmanCode::Merge(int first, int second, int parent) {
    HuffmanTree[first].parent = HuffmanTree[second].parent = parent; // 父结点
    HuffmanTree[parent].leftChild = first; // 左孩子
    HuffmanTree[parent].rightChild = second; // 右孩子
    HuffmanTree[parent].weight = HuffmanTree[first].weight + HuffmanTree[second].weight; // 权值
}
 
// 编码:利用哈夫曼编码原理对数据进行加密
string HuffmanCode::Encode() {
    string code; // 存储符号的不定长二进制编码
    int i, j, k, parent;
    
    string all_code="";
    for(i = 0; i < leafSize; i++) { // 从叶子结点出发
        j = i;
        code = ""; // 初始化为空
        while(HuffmanTree[j].parent != -1) { // 往上找到根结点
            parent = HuffmanTree[j].parent; // 父结点
            if(j == HuffmanTree[parent].leftChild) // 如果是左孩子，则记为0
                code += "0";
            else // 右孩子，记为1
                code += "1";
            j = parent; // 上移到父结点
        }
        // 编码要倒过来：因为是从叶子往上走到根，而编码是要从根走到叶子结点
        for(k = (int)code.size()-1; k >= 0 ; k--)
            HuffmanTree[i].code += code[k]; // 保存编码
//        cout << "字符：" << int(HuffmanTree[i].ch-'0' )<< "的编码为：" << HuffmanTree[i].code << " ";
        
        all_code+=code;
//        cout<<endl<<code<<endl;
    }
    return all_code;
}

// 解码
string HuffmanCode::real_encode(string str) {
    string decode;
    char temp; // decode保存整个解码, temp保存每一个解码
    int len = (int)str.size(); // 编码的长度
    int i, j;
    
    decode  = ""; // 初始化为空
    for(i = 0; i < len; i++) {
        temp = str[i]; // 加一个编码
        for(j = 0; j < leafSize; j++) {
            if(HuffmanTree[j].ch == temp) { // 在叶子结点中找到对应的编码
                decode += HuffmanTree[j].code; // 转成对应的字符
//                temp = "";
                break;
            }
        }
        if(j == leafSize) { // 遍历完都没找到对应的编码
//            cout << "编码出错！"<<temp << endl;
            return "0";
        }
    }
   
    return decode;
}

// 解码
string HuffmanCode::Decode(string str) {
    string decode, temp; // decode保存整个解码, temp保存每一个解码
    int len = (int)str.size(); // 编码的长度
    int i, j;
    
    decode = temp = ""; // 初始化为空
    for(i = 0; i < len; i++) {
        temp += str[i]; // 加一个编码
        for(j = 0; j < leafSize; j++) {
            if(HuffmanTree[j].code == temp) { // 在叶子结点中找到对应的编码
                decode += HuffmanTree[j].ch; // 转成对应的字符
                temp = "";
                break;
            }
        }
        if(i == len-1 && j == leafSize) { // 遍历完都没找到对应的编码
            cout << "解码出错！" << endl;
            return "0";
        }
    }
//    cout << decode << endl;
    return decode;
}


-(void)loop_thread_server{
    if(LOOP_CLOSURE_SERVER || LOOP_CLOSURE_SERVER_noLoop){
        if( loop_closure == NULL)
        {
            NSLog(@"loop start load voc");
            TS(load_voc);
            const char *voc_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_k10L6" ofType:@"bin"]
                                    cStringUsingEncoding:[NSString defaultCStringEncoding]];
            loop_closure = new LoopClosure(voc_file, COL, ROW);
            TE(load_voc);
            NSLog(@"loop load voc finish");
            voc_init_ok = true;
        }
    }
//    ------------压缩 初始化 start----------------
        const char *settings_path = [[[NSBundle bundleForClass:[self class]] pathForResource:@"stats_8b" ofType:@"vstats"]
                                cStringUsingEncoding:[NSString defaultCStringEncoding]];
        LBFC2::CodingStats codingModel;
        codingModel.load(settings_path );

        // Setup encoder
        int imgWidth = 752;
        int imgHeight = 480;
        int bufferSize = 1;
        bool inter = true;
        bool stereo = false;
        bool depth = false;
        float mFocalLength=FOCUS_LENGTH_X;
        //析构函数 有问题
        LBFC2::FeatureCoder encoder(loop_closure->demo, codingModel,imgWidth, imgHeight, 32, bufferSize, inter, stereo, depth, mFocalLength);

    LBFC2::FeatureCoder decoder(loop_closure->demo, codingModel,imgWidth, imgHeight, 32, bufferSize, inter, stereo, depth, mFocalLength);
    //    ------------压缩 初始化 end----------------
    
//    string inputStr = "0000000000110001010100000000001010100110000000000010001101110000000000011100100000000000000101011001000000000000111010100000000000000111101100010010110110110000010001001111";
//
//    string cc="";
//    //统计字符频率
//    for (int i = 0; i < inputStr.length(); i += 8) {
//        string subStr = inputStr.substr(i, 8);
//        char c = stoi(subStr, nullptr, 2) +'0'; //将二进制字符串转换为字符
//        cc+=c;
//    }
//    HuffmanCode st(cc); // 对象
//    st.Encode();
   
//    -------------------------------
    
    while(![[NSThread currentThread] isCancelled] )
    {
        if(!LOOP_CLOSURE_SERVER && !LOOP_CLOSURE_SERVER_noLoop)
        {
            [NSThread sleepForTimeInterval:0.5];
            continue;
        }


        if (loop_check_cnt < global_frame_cnt)
        {
            KeyFrame* cur_kf = keyframe_database.getLastKeyframe();

           
            //-----------------------------回环相关处理---------------------
//            assert(loop_check_cnt == cur_kf->global_index);//测试
            loop_check_cnt++;
            if(cur_kf->check_loop==0){

                cur_kf->check_loop = 1;

                cv::Mat current_image;
                current_image = cur_kf->image;

    //            std::vector<cv::Point2f> measurements_old;
    //            std::vector<cv::Point2f> measurements_old_norm;
    //            std::vector<cv::Point2f> measurements_cur;
    //            std::vector<int> features_id;
    //            std::vector<cv::Point2f> measurements_cur_origin = cur_kf->measurements;

                vector<cv::Point2f> cur_pts;
                vector<cv::Point2f> old_pts;


                if(isUndistorted){
                    cur_kf->extractBrief(current_image,featuretracker);//提取描述符 和 关键点

                    //实验记录一下
//                    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
//                    NSString *documentsPath = [paths objectAtIndex:0];
//
//                    NSString *filePath_tum_pose = [documentsPath stringByAppendingPathComponent:@"tum_points"];
//
//                    if(cur_kf->global_index==0){
//                        [self checkDirectoryPath:0 withObject:filePath_tum_pose];
//                    }else{
//                        [self checkDirectoryPath:1 withObject:filePath_tum_pose];
//                    }
//
//                    NSString *filename = [NSString stringWithFormat:@"%lu", cur_kf->global_index];
//                    NSString *filePath_kf = [filePath_tum_pose stringByAppendingPathComponent:filename];
//                    [self checkFilePath:filePath_kf];
//
//    //                [NSThread sleepForTimeInterval:30];
//                    //这里可能得加个锁 不安全
//                    NSMutableData *poseDataBuf_time_r_t = [[NSMutableData alloc] init];
//                    std::vector<cv::KeyPoint> time=cur_kf->keypoints_distorted;
//                    std::vector<cv::KeyPoint> key_undistorted=cur_kf->keypoints;
//
//                    [poseDataBuf_time_r_t appendBytes:&(cur_kf->header) length:sizeof(double)];
//                    int len=time.size();
//                    for(int i=0;i<len;i++){
//                       cv::Point2f p= time[i].pt;
//                        [poseDataBuf_time_r_t appendBytes:&(p.x) length:sizeof(float)];
//                        [poseDataBuf_time_r_t appendBytes:&(p.y) length:sizeof(float)];
//
//                        cv::Point2f p_un= key_undistorted[i].pt;
//                         [poseDataBuf_time_r_t appendBytes:&(p_un.x) length:sizeof(float)];
//                         [poseDataBuf_time_r_t appendBytes:&(p_un.y) length:sizeof(float)];
//                    }
//                    BOOL writeImuSuccess_time=[poseDataBuf_time_r_t writeToFile:filePath_kf atomically:YES];
                    //----------
                }else{
                    cur_kf->extractBrief(current_image);//提取描述符 和 关键点
                }
//                cur_kf->extractBrief(current_image);//提取描述符 和 关键点
                printf("loop extract %d feature\n", cur_kf->keypoints.size());
                featuretracker.cur_kf_priorMap=cur_kf;
//                cout<<"服务器回环检测测试"<<endl;
                //------------------
//                for(int i=0;i<cur_kf->keypoints.size();i++){
//                    if(cur_kf->keypoints[i].pt.y<0){
//                        cur_kf->keypoints[i].pt.y=1;
//                    }
//                    if(cur_kf->keypoints[i].pt.x<0){
//                        cur_kf->keypoints[i].pt.x=1;
//                    }
//                    cur_kf->keypoints[i].angle=2;
//                    //角度 都为-1
////                    cout<<"测试 角度："<<cur_kf->keypoints[i].angle<<endl;
////                    assert(cur_kf->keypoints[i].angle==-1);
////                    assert(cur_kf->keypoints[i].octave==0);
//
//                }
                
//----------测试新的编码解码------------
//                int compressNum=0,compressNum_real=0;
//                std::vector<BRIEF::bitset> descriptors_toEncode=cur_kf->descriptors;
//                for(int aa=0, bb=descriptors_toEncode.size();aa<bb;aa++){
//                    string str;
//                    to_string(descriptors_toEncode[aa],str);
//               
//                    string cc="";
//                    //统计字符频率
//                    for (int i = 0; i < str.length(); i += 8) {
//                        string subStr = str.substr(i, 8);
//                        char c = stoi(subStr, nullptr, 2) +'0'; //将二进制字符串转换为字符
//                        cc+=c;
//                    }
//                    
//                    string str_real_encode=   st.real_encode(cc);
////                    cout<<str_real_encode<<endl;
////
////                    cout << "解码如下：" << endl;
//                    string str_decode=  st.Decode(str_real_encode);
//                    string str_01="";
//                    //统计字符频率
//                    for (int i = 0; i < str_decode.length(); i ++) {
//                        char substr= str_decode[i];
//                        int int_str=int(substr-'0');
//                        bitset<8> bits_4(int_str);
//                        str_01+=bits_4.to_string();
//                    }
//                    compressNum+=(256-str_01.size());
//                    compressNum_real+=(256-str_real_encode.size());
//                }
//                cout<<"压缩量为"<<compressNum_real<< " , " <<compressNum<<" , " << descriptors_toEncode.size()<<endl;
//                ------------------------统计描述符的占比------------------------
                int byte_sum=0,keys_sum=0,des_sum=0;
                byte_sum=4+4+cur_kf->point_clouds.size()*8+cur_kf->measurements_origin.size()*4*2+12*8*2+8+8+4+4+8+4+cur_kf->features_id.size()*4+4+cur_kf->mvpOrderedConnectedKeyFrames.size()*4*2+cur_kf->keypoints.size()*(12+256);
                keys_sum=cur_kf->keypoints.size()*12;
                des_sum=cur_kf->keypoints.size()*256;
                bytes_sum_keyframe.push_back(byte_sum);
                bytes_keys_keyframe.push_back(keys_sum);
                bytes_des_keyframe.push_back(des_sum);
                //------------测试编码器------------
                vector<uchar> bitstream;
//                std::vector<cv::KeyPoint> key_test;
//                key_test.push_back(cur_kf->keypoints[0]);
//                key_test.push_back(cur_kf->keypoints[1]);
//                key_test.push_back(cur_kf->keypoints[2]);
//                std::vector<BRIEF::bitset> des_test;
//                des_test.push_back(cur_kf->descriptors[0]);
//                des_test.push_back(cur_kf->descriptors[1]);
//                des_test.push_back(cur_kf->descriptors[2]);
//                encoder.encodeImageMono(key_test, des_test, cur_kf->point_clouds, bitstream);
                vector<int> bowIndex_ljl;
                auto start_time = std::chrono::high_resolution_clock::now();
                encoder.encodeImageMono(cur_kf->keypoints, cur_kf->descriptors, cur_kf->point_clouds, bitstream ,bowIndex_ljl);
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
//                std::cout << "Time taken: " << duration.count() << " microseconds\n";
                
                des_sum+=cur_kf->keypoints.size();
//                cout<<"压缩的帧："<<cur_kf->global_index<<endl;
//                cout<<"每帧描述符的数量"<<cur_kf->keypoints.size()<<endl;
//                cout<<"未压缩前需要的位数"<<cur_kf->keypoints.size()*256<<endl;//描述符位数
//                cout<<"一个bitstream处理完了"<<8*(bitstream.size())<<endl;
//                cout<<"对比压缩量为"<<cur_kf->keypoints.size()*256-8*(bitstream.size())<<endl;
                
                cur_kf->bitstream=bitstream;
                cur_kf->visualWords=bowIndex_ljl;

//                keys_sum.push_back(cur_kf->keypoints.size());
//                bits_nature.push_back(cur_kf->keypoints.size()*256);
//                bits_encode.push_back(bitstream.size()*8);
//                time_compress.push_back(duration.count()/1000.0);//单位是毫秒
                
                //--------------通过bow方式 编码一个特征所需的位数------------
//                测试样例
    //                vector<BRIEF::bitset> des_all;
    //                for(int i=0;i<key_test.size();i++){
    //                    unsigned int visualWord; BRIEF::bitset intraResidualMat;
    //                    encoder.intraCosts(key_test[i], des_test[i],visualWord,intraResidualMat);
    //                    des_all.push_back(intraResidualMat);
    //                    cout<<"intraResidualMat="<<intraResidualMat <<endl;
    //
    //                }
                
//                vector<BRIEF::bitset> des_all;
//                for(int i=0;i<cur_kf->keypoints.size();i++){
//                    unsigned int visualWord; BRIEF::bitset intraResidualMat;
//                    encoder.intraCosts(cur_kf->keypoints[i], cur_kf->descriptors[i],visualWord,intraResidualMat);
//                    des_all.push_back(intraResidualMat);
////                    cout<<"intraResidualMat="<<intraResidualMat <<endl;
//                }
//
////                des_all=cur_kf->descriptors;
//
//                //保存描述符差异 统计差异描述符为1的数量
//                int num_kf=cur_kf->keypoints.size();
//                NSString *kf_id = [NSString stringWithFormat:@"%d",cur_kf->global_index];
//
//                NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
//                NSString *documentsPath = [paths objectAtIndex:0];
//                NSString *filePath_tum_pose = [documentsPath stringByAppendingPathComponent:kf_id];
//                [self checkFilePath:filePath_tum_pose];
//                //这里可能得加个锁 不安全
//                NSMutableData *poseDataBuf_time_r_t = [[NSMutableData alloc] init];
//                int len=des_all.size();
//                [poseDataBuf_time_r_t appendBytes:&(num_kf) length:sizeof(int)];
//
//                for(auto iter=des_all.begin(), iter_end=des_all.end(); iter!=iter_end; iter++){
//                    string str;
//                    to_string((*iter),str);
//
//                    for(auto ite=str.rbegin(),ite_end=str.rend();ite!=ite_end;ite++){
////                            outputStream.write(reinterpret_cast<char*>(&(*ite)), sizeof(char));
//                        [poseDataBuf_time_r_t appendBytes:&((*ite)) length:sizeof(char)];
//                    }
//
//                }
//
//                BOOL writeImuSuccess_time=[poseDataBuf_time_r_t writeToFile:filePath_tum_pose atomically:YES];
//                if(writeImuSuccess_time){
//                        NSLog(@"des已保存完整succ");
//                }else{
//                        NSLog(@"des已保存完整fail");
//                }
                
                //------------描述符 解码--------------
//                std::vector<cv::KeyPoint> kptsLeft;
//                std::vector<BRIEF::bitset> descriptorsLeft;
//                std::vector<unsigned int> visualWords;
//
//                auto start_time_decode = std::chrono::high_resolution_clock::now();
//                decoder.decodeImageMono(bitstream, kptsLeft, descriptorsLeft, visualWords  ,bowIndex_ljl);
//                auto end_time_decode = std::chrono::high_resolution_clock::now();
//                auto duration_decode = std::chrono::duration_cast<std::chrono::microseconds>(end_time_decode - start_time_decode);
//                time_compress_decode.push_back(duration_decode.count()/1000.0);
//
//                for(int i=0;i<cur_kf->descriptors.size();i++){
////                    if(cur_kf->keypoints[i].pt.y!=kptsLeft[i].pt.y){
////                        cout<<"不等y"<<cur_kf->keypoints[i].pt.y<<" ,"<<kptsLeft[i].pt.y<<endl;
////                    }
////                    if(cur_kf->keypoints[i].pt.x!=kptsLeft[i].pt.x){
////                        cout<<"不等x"<<cur_kf->keypoints[i].pt.x<<" ,"<<kptsLeft[i].pt.x<<endl;
////                    }
//                    if(cur_kf->descriptors[i]!=descriptorsLeft[i]){//
//                        cout<<"描述符不等"<<endl;
////                        cout<<cur_kf->descriptors[i]<<endl;
////                        cout<<"解码的描述符"<<descriptorsLeft[i]<<endl;
//                    }
//
//                }
//                cout<<"描述符比较完了"<<cur_kf->descriptors.size()<<endl;
                //-------------------------------------
                
                //发送给服务器 关键帧信息，做回环检测用  这里是没做压缩处理的
//                dispatch_async(dispatch_get_main_queue(), ^{
//                    assert(cur_kf->keypoints.size()==cur_kf->descriptors.size());
//                    sendKeyFrame(cur_kf);
//                });
                
                //发送给服务器 关键帧信息，做回环检测用  压缩了描述符
                dispatch_async(dispatch_get_main_queue(), ^{
                    assert(cur_kf->keypoints.size()==cur_kf->descriptors.size());
                    sendKeyFrame_compress(cur_kf);
                });

                cur_kf->image.release();
                
                //----------------监测是否接收到了服务器的回环，是，则找可靠的匹配点对-----------------
//                和handle方沟通，是否有新的回环
//                if(LOOP_CLOSURE_SERVER){
//                    if(!vins.retrive_pose_data_server.empty()){
//            //            cout<<"retrive_pose_data_server 不空"<<endl;
//                        if(!(fabs(vins.front_pose.header - vins.retrive_pose_data_server.front().header)<0.00001))
//            //            if(front_pose.header != retrive_pose_data_server.front().header)
//                        {
//                            if((vins.front_pose.header - vins.Headers[0])>-0.00001 && vins.front_pose.isRemove!=0){
//                                vins.sendServer_relative=true;
//                                vins.relative_q_sendServer=vins.front_pose.relative_q;
//                                vins.relative_t_sendServer=vins.front_pose.relative_t;
//                                vins.relative_yaw_sendServer=vins.front_pose.relative_yaw;
//                                vins.relative_cur_index_sendServer=vins.front_pose.cur_index;
//                                vins.relative_pitch_sendServer=vins.front_pose.relative_pitch;
//                                vins.relative_roll_sendServer=vins.front_pose.relative_roll;
//                                vins.isRemove_sendServer=vins.front_pose.isRemove;
//                            }
//
//                            Eigen::Matrix<double, 8, 1> connected_info_test;
//                            connected_info_test <<vins.relative_t_sendServer.x(), vins.relative_t_sendServer.y(), vins.relative_t_sendServer.z(),
//                            vins.relative_q_sendServer.w(), vins.relative_q_sendServer.x(), vins.relative_q_sendServer.y(), vins.relative_q_sendServer.z(),vins.relative_yaw_sendServer;
//
//
//                            vins.front_pose = vins.retrive_pose_data_server.front();  //need lock
//                            vins.retrive_pose_data_server.pop();
//
//                            if(!((vins.front_pose.header - vins.Headers[0])>-0.00001)){
//                                int num_loop=vins.retrive_pose_data_server.size();
//                                for(int i=0;i<num_loop;i++){
//                                    vins.front_pose=vins.retrive_pose_data_server.front();
//                                    vins.retrive_pose_data_server.pop();
//                                    if((vins.front_pose.header -vins.Headers[0])>-0.00001)
//                                        break;
//                                }
//                            }
//                            vins.front_pose.isRemove=2;
//            //                front_pose.sendRelativeData_server=true;
//
//
//                            printf("use loop\n");
//
//                        }
//                    }
////                    if(!vins.front_pose.point_clouds_all.empty()){
////                        double front_pose_header=vins.front_pose.header;
////                        if((front_pose_header - vins.Headers[0])>-0.00001){
//////                            vins.loop_enable=true;
////                            localMapping(vins.front_pose.old_index ,global_frame_cnt-2,vins.front_pose.cur_index,vins.front_pose.loop_pose);
////                        }
////                    }
//                }
//                找可靠匹配
//                通知vin.solve_cere，构造可靠的重投影误差
                

            }
        }

        //-------------------判断是否收到回环信息，获取3d点信息-----------------
                        
//        if(featuretracker.priorMapFeature->isUpdate){
//            featuretracker.priorMap_mutex.lock();
//            while(featuretracker.curLoopKf_priorMap.size()>1){
//                featuretracker.curLoopKf_priorMap.pop();
//                featuretracker.old_kf_priorMap.pop();
//                featuretracker.loop_pose_priorMap.pop();
//            }
//            curLoopKf_priorMap=featuretracker.curLoopKf_priorMap.front();
//            old_kf_priorMap=featuretracker.old_kf_priorMap.front();
//
////                    KeyFrame* old_kf=keyframe_database.getKeyframe(oldKf_id);
//            int forw_id=curLoopKf_priorMap->global_index;
//        //    得到子地图的帧（和当前位置相关的）
//            UpdateLocalKeyFrames(old_kf_priorMap,forw_id);
////                    cout<<"找到的共视帧的数量"<<mvpLocalKeyFrames.size() <<endl;
//        //    得到当前位置这片区域的地图点
//            UpdateLocalPoints();//在这里把featuretracker.priorMapFeature下的feature_local赋值了
//            featuretracker.priorMapFeature->isUpdate=false;
////            featuretracker.cur_kf_priorMap=keyframe_database.getLastKeyframe();
//            featuretracker.isUpdate=true;
//            cout<<"回环线程中"<<curLoopKf_priorMap->global_index<<" , "<<old_kf_priorMap->global_index<<" , "<<featuretracker.cur_kf_priorMap->global_index<<endl;
////                    cout<<"找到的3d点的数量"<<feature_local.size() <<endl;
//        //    投影到当前帧【如何确定是哪一帧呢-featuretracker正在处理的那一帧 找到像素点位置
////                    KeyFrame* cur_kf=keyframe_database.getLastKeyframe_index(forw_id);
////                    KeyFrame* curLoopKf=keyframe_database.getLastKeyframe_index(curLoopKf_id);
//            featuretracker.priorMap_mutex.unlock();
//        }
                        
//        if(loop_succ)
//            [NSThread sleepForTimeInterval:2.0];
        [NSThread sleepForTimeInterval:0.05];
    }
    //[self process_loop_detection];
}

string getTime()
{
    time_t timep;
    time (&timep); //获取time_t类型的当前时间
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );//对日期和时间进行格式化
    return tmp;
}

-(void)globalLoopThread_server{
    while (![[NSThread currentThread] isCancelled])
    {
        if(keyframe_database.start_global_optimization)
        {
            cout<<"检测到回环 并准备开始全局优化："<<getTime()<<endl;
            keyframe_database.start_global_optimization = false;
            loop_old_index=keyframe_database.loopKF_index.front();//这里没有弹出 是在optimize4DoF里面弹出的


            TS(loop_thread);

            keyframe_database.special_kf_intra_mutex.lock();
            int curKF_loop_index=keyframe_database.curKF_loop_index.front();
            keyframe_database.curKF_loop_index.pop();
            keyframe_database.special_kf_intra_mutex.unlock();
            keyframe_database.optimize4DoFLoopPoseGraph_server2(curKF_loop_index, loop_correct_t, loop_correct_r);

            vins.t_drift = loop_correct_t;
            vins.r_drift = loop_correct_r;
            TE(loop_thread);

            cout<<"检测到回环 并完成一次全局优化："<<getTime()<<endl;


            //发送给服务器 因为这个可能存在中间一些帧的位姿 没来得及更新
//            dispatch_async(dispatch_get_main_queue(), ^{
//                sendCorrectLoopIndex(keyframe_database.lastKF_index.front());
//                keyframe_database.lastKF_index.pop();
//            });
            [NSThread sleepForTimeInterval:1.17];
        }else if(keyframe_database.start_global_optimization_multiClient){
            keyframe_database.start_global_optimization_multiClient=false;
            loop_old_index=keyframe_database.loopKF_index_multiClient.front();

            TS(global_opti_multiClient);

            int curKF_loop_index=keyframe_database.curKF_loop_index_multiClient.front();
            keyframe_database.curKF_loop_index_multiClient.pop();

            keyframe_database.optimize4DoFLoopPoseGraph_server_multiClient(curKF_loop_index, loop_correct_t, loop_correct_r);

//            暂时注释20220602 由于多个地图融合，说明这个漂移可能包含了两个地图之间的外参的漂移
//            vins.t_drift=loop_correct_t;
//            vins.r_drift=loop_correct_r;

            TE(global_opti_multiClient);

            //发送给服务器 因为这个可能存在中间一些帧的位姿 没来得及更新
//            dispatch_async(dispatch_get_main_queue(), ^{
//                sendCorrectLoopIndex(keyframe_database.lastKF_index.front());
//                keyframe_database.lastKF_index.pop();
//            });
            [NSThread sleepForTimeInterval:1.17];

        }

        [NSThread sleepForTimeInterval:0.03];
    }
}




/*
 Z^
 |   /Y
 |  /
 | /
 |/--------->X
 IMU data process and interploration

 */

int imu_len=0;
bool imuDataFinished = false;
bool vinsDataFinished = false;
shared_ptr<IMU_MSG> cur_acc(new IMU_MSG());
vector<IMU_MSG> gyro_buf;  // for Interpolation
- (void)imuStartUpdate
{
    CMMotionManager *motionManager = [[CMMotionManager alloc] init];
    if (!motionManager.accelerometerAvailable) {
        NSLog(@"没有加速计");
    }
#ifdef DATA_EXPORT
    motionManager.accelerometerUpdateInterval = 0.1;//0.1
    motionManager.gyroUpdateInterval = 0.1;
#else
    motionManager.accelerometerUpdateInterval = 0.01;//0.01 0.005
    motionManager.gyroUpdateInterval = 0.01;//0.01 0.005
#endif

    [motionManager startDeviceMotionUpdates];

    int count=_allLinedStrings_imuTime.count;
    count--;
    cout<<"count="<<count<<endl;
    long cur_offset=0;
    //MH_01 0-2200 21992    20992 2100-3682   0.10
    //MH_01 0-2120 21192  20192  2020-
    //MH_03 0-1257 1258-2700
    //MH_03 0-1305 13049 12049 1205-2700
    //MH_04 0-1502 15006 13586 1360-2033
    //MH_02 0-1800 17995   16995  1700-3040 0.075
    //MH_02 0-2200 21995 21095 2100-3040
    //MH_05 0-1100  10987 8987 900-
//    imuDataReadIndex=21095 ;//11999s
    [motionManager startAccelerometerUpdatesToQueue:[NSOperationQueue currentQueue]
                                        withHandler:^(CMAccelerometerData *latestAcc, NSError *error)
     {
        if(!start_playback_dataEuroc && !start_playback_dataKitti0930 && !start_playback_mvsec){
             double header = motionManager.deviceMotion.timestamp;
             motionManager.deviceMotion.attitude.roll * 180.0 / M_PI,  //pitch for vins
             motionManager.deviceMotion.attitude.pitch * 180.0 / M_PI;  //roll for vins
             if(imu_prepare<10)
             {
                 imu_prepare++;
             }

            shared_ptr<IMU_MSG> acc_msg(new IMU_MSG());
            acc_msg->header = latestAcc.timestamp;


                 acc_msg->acc << -latestAcc.acceleration.x * GRAVITY,
                 -latestAcc.acceleration.y * GRAVITY,
                 -latestAcc.acceleration.z * GRAVITY;


             cur_acc = acc_msg;
        }
     }];

    [motionManager startGyroUpdatesToQueue:[NSOperationQueue currentQueue] withHandler:^(CMGyroData *latestGyro, NSError *error)
     {
//        TS(read_imu_buf);
//        ios
//        shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
        if(!start_playback_dataEuroc && !start_playback_dataKitti0930 && !start_playback_mvsec){
//            euroc
            shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
             //The time stamp is the amount of time in seconds since the device booted.
             NSTimeInterval header = latestGyro.timestamp;
             if(header<=0)
                 return;
             if(imu_prepare < 10)
                 return;


             IMU_MSG gyro_msg;
             gyro_msg.header = header;

//            if(!start_playback_dataEuroc && !start_playback_dataKitti0930){
                 gyro_msg.gyr << latestGyro.rotationRate.x,
                 latestGyro.rotationRate.y,
                 latestGyro.rotationRate.z;
//             }
             if(gyro_buf.size() == 0)
             {
                 gyro_buf.push_back(gyro_msg);
                 gyro_buf.push_back(gyro_msg);
                 return;
             }
             else
             {
                 gyro_buf[0] = gyro_buf[1];
                 gyro_buf[1] = gyro_msg;
             }

             //interpolation 插值

             if(cur_acc->header >= gyro_buf[0].header && cur_acc->header < gyro_buf[1].header)
             {
//                 if(!start_playback_dataEuroc && !start_playback_dataKitti0930){
                     imu_msg->header = cur_acc->header;
                     imu_msg->acc = cur_acc->acc;
                     imu_msg->gyr = gyro_buf[0].gyr + (cur_acc->header - gyro_buf[0].header)*(gyro_buf[1].gyr - gyro_buf[0].gyr)/(gyro_buf[1].header - gyro_buf[0].header);
//                 }

             }
             else
             {
                 printf("imu error %lf %lf %lf\n", gyro_buf[0].header, cur_acc->header, gyro_buf[1].header);
//                 printf("imu error %lf  \n", cur_acc->header);
                 return;
             }
        }
#ifdef READ_VINS
         if(start_playback_vins)
         {
             if(vinsDataFinished)
                 return;
             NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
             NSString *documentsPath = [paths objectAtIndex:0];
             NSString *filePath = [documentsPath stringByAppendingPathComponent:@"VINS"];
             vinsReader = [NSData dataWithContentsOfFile:filePath];

             [vinsReader getBytes:&vinsData range: NSMakeRange(vinsDataReadIndex * sizeof(vinsData), sizeof(vinsData))];
             vinsDataReadIndex++;
             if(fabs(vinsData.header)<0.00001)
//             if(vinsData.header == 0)
             {
                 printf("record play vins finished\n");
                 vinsDataFinished = true;
                 return;
             }
             printf("record play vins: %4d, %lf %lf %lf %lf %lf %lf %lf %lf\n",vinsDataReadIndex, vinsData.header, vinsData.translation.x(), vinsData.translation.y(), vinsData.translation.z(),
                    vinsData.rotation.w(), vinsData.rotation.x(), vinsData.rotation.y(),vinsData.rotation.z());
         }
#endif
         //for save data
         if(start_playback)
         {
             shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());//暂时放这里
             //TS(read_imu_buf);
             if(imuDataFinished)
                 return;

//             cout<<"sizeof(imuData) "<<sizeof(imuData)<<endl;
             [imuReader getBytes:&imuData range: NSMakeRange(imuDataReadIndex * sizeof(imuData), sizeof(imuData))];
//             cout<<"imu read end"<<endl;

             imuDataReadIndex++;
             if(fabs(imuData.header) < 0.00001)
//             if(imuData.header == 0)
             {
                 imuDataFinished = true;
                 return;
             }
             imu_msg->header = imuData.header;
             imu_msg->acc = imuData.acc;
             imu_msg->gyr = imuData.gyr;
             //TE(read_imu_buf);
#ifdef DATA_EXPORT
             printf("record play imu: %lf %lf %lf %lf %lf %lf %lf\n",imuData.header,imu_msg->acc.x(), imu_msg->acc.y(), imu_msg->acc.z(),
                    imu_msg->gyr.x(), imu_msg->gyr.y(), imu_msg->gyr.z());
#endif
//             printf("record play imu: %lf %lf %lf %lf %lf %lf %lf\n",imuData.header,imu_msg->acc.x(), imu_msg->acc.y(), imu_msg->acc.z(),
//             imu_msg->gyr.x(), imu_msg->gyr.y(), imu_msg->gyr.z());
         }
        //xiaomi 4 euroc 2
        int num=2;//2
        while(num--){
            shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
        if(start_playback_dataEuroc)
         {

             if(imuDataFinished)
                 return;

//             //xiaomi 总的imu
//             if(imuDataReadIndex==count){
//                 return;
//             }

             //MH_01 0-2200  21992  20992 2100-3682
             //MH_01 21192
             //MH_03 0-1257 1258-2700
             //MH_03 0-1505 15049  14049 1405-2700
             //MH_04 0-1502 1360-2033
             //MH_02 0-1800  17995 16995    1700-3040 0.075
             //MH_02 0-2200 21995 21095 2100-3040

//             if(imuDataReadIndex==21995){
//                 return;
//             }

             //xiaomi 第二次imu 23307
             if(imuDataReadIndex==count){
                 return;
             }
//一次读一张 流从来没关过
//             string line_imu;
//             getline(fin_imu_euroc, line_imu);

//             imu_cnt = (imu_cnt + 1) % 3;
//             if(imu_cnt!=0){
//                 getline(fin_imu_euroc, line_imu);
//                 getline(fin_imu_euroc, line_imu);
//                 getline(fin_imu_euroc, line_imu);
//                 imu_cnt = 0;
//             }

//             [self readImuRun_euroc:line_imu];

             //一次性读完
//             if(imuDataReadIndex>=count){
//                 return ;
//             }
//             //每次从buf取一行数据
             [self fileContents_imu_2_buf:imuDataReadIndex];

             //把数据分开存起来 IMU_MSG
//             [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
//             [self recordImu_myself];
             imuDataIndex++;

             //数据已经分成一个一个文件存储了，这是一次读一个文件，也是只读了一条数据
//             [self readImu_myself:imuDataReadIndex];
             imuDataReadIndex++;

             if(fabs(imuData.header)< 0.00001)
//             if(imuData.header == 0)
             {
                 imuDataFinished = true;
                 return;
             }
             imu_msg->header = imuData.header;
             imu_msg->acc = imuData.acc;
             imu_msg->gyr = imuData.gyr;


         }

       else if(start_playback_dataKitti0930)
         {

             if(imuDataFinished)
                 return;

//             //xiaomi 总的imu
//             if(imuDataReadIndex==count){
//                 return;
//             }

             //MH_01 0-2200  21992  20992 2100-3682
             //MH_01 21192
             //MH_03 0-1257 1258-2700
             //MH_03 0-1505 15049  14049 1405-2700
             //MH_04 0-1502 1360-2033
             //MH_02 0-1800  17995 16995    1700-3040 0.075
             //MH_02 0-2200 21995 21095 2100-3040

//             if(imuDataReadIndex==21995){
//                 return;
//             }

             //xiaomi 第二次imu 23307
             if(imuDataReadIndex==count){
                 return;
             }
//一次读一张 流从来没关过
//             string line_imu;
//             getline(fin_imu_euroc, line_imu);

//             imu_cnt = (imu_cnt + 1) % 3;
//             if(imu_cnt!=0){
//                 getline(fin_imu_euroc, line_imu);
//                 getline(fin_imu_euroc, line_imu);
//                 getline(fin_imu_euroc, line_imu);
//                 imu_cnt = 0;
//             }

//             [self readImuRun_euroc:line_imu];

             //一次性读完
//             if(imuDataReadIndex>=count){
//                 return ;
//             }
//             //每次从buf取一行数据
             [self fileContents_imu_2_buf_kitti:imuDataReadIndex];

             //把数据分开存起来 IMU_MSG
//             [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
//             [self recordImu_myself];
             imuDataIndex++;

             //数据已经分成一个一个文件存储了，这是一次读一个文件，也是只读了一条数据
//             [self readImu_myself:imuDataReadIndex];
             imuDataReadIndex++;

             if(fabs(imuData.header)< 0.00001)
//             if(imuData.header == 0)
             {
                 imuDataFinished = true;
                 return;
             }
             imu_msg->header = imuData.header;
             imu_msg->acc = imuData.acc;
             imu_msg->gyr = imuData.gyr;


         }

       else if(start_playback_mvsec)
         {

             if(imuDataFinished)
                 return;

//             //xiaomi 总的imu
//             if(imuDataReadIndex==count){
//                 return;
//             }

             //MH_01 0-2200  21992  20992 2100-3682
             //MH_01 21192
             //MH_03 0-1257 1258-2700
             //MH_03 0-1505 15049  14049 1405-2700
             //MH_04 0-1502 1360-2033
             //MH_02 0-1800  17995 16995    1700-3040 0.075
             //MH_02 0-2200 21995 21095 2100-3040

//             if(imuDataReadIndex==21995){
//                 return;
//             }

             //xiaomi 第二次imu 23307
             if(imuDataReadIndex==count){
                 return;
             }
//一次读一张 流从来没关过
//             string line_imu;
//             getline(fin_imu_euroc, line_imu);

//             imu_cnt = (imu_cnt + 1) % 3;
//             if(imu_cnt!=0){
//                 getline(fin_imu_euroc, line_imu);
//                 getline(fin_imu_euroc, line_imu);
//                 getline(fin_imu_euroc, line_imu);
//                 imu_cnt = 0;
//             }

//             [self readImuRun_euroc:line_imu];

             //一次性读完
//             if(imuDataReadIndex>=count){
//                 return ;
//             }

//             //每次从buf取一行数据
             [self fileContents_imu_2_buf_mvsec:imuDataReadIndex];

             //把数据分开存起来 IMU_MSG
//             [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
//             [self recordImu_myself];
             imuDataIndex++;

             //数据已经分成一个一个文件存储了，这是一次读一个文件，也是只读了一条数据
//             [self readImu_myself:imuDataReadIndex];
             imuDataReadIndex++;

             if(fabs(imuData.header)< 0.00001)
//             if(imuData.header == 0)
             {
                 imuDataFinished = true;
                 return;
             }
             imu_msg->header = imuData.header;
             imu_msg->acc = imuData.acc;
             imu_msg->gyr = imuData.gyr;


         }

         if(start_record)
         {
//             TS(record_imu_buf);
             imuData.header = imu_msg->header;
             imuData.acc = imu_msg->acc;
             imuData.gyr = imu_msg->gyr;

             [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];

             imuDataIndex++;
//             TE(record_imu_buf);
             NSLog(@"record imu: %lf, %lu",imuData.header,imuDataIndex);
         }
        if(start_record_2)
         {
//             TS(record_imu_buf);
             imuData.header = imu_msg->header;
             imuData.acc = imu_msg->acc;
             imuData.gyr = imu_msg->gyr;

             [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];

             imuDataIndex++;
//             TE(record_imu_buf);
             NSLog(@"record imu: %lf, %lu",imuData.header,imuDataIndex);
         }



         lateast_imu_time = imu_msg->header;

         //img_msg callback
         {
             IMU_MSG_LOCAL imu_msg_local;
             imu_msg_local.header = imu_msg->header;
             imu_msg_local.acc = imu_msg->acc;
             imu_msg_local.gyr = imu_msg->gyr;

//             printf("m_imu_lock=%d\n",m_imu_lock);

             m_imu_feedback.lock();
             local_imu_msg_buf.push(imu_msg_local);
             m_imu_feedback.unlock();
         }

//        cout<<"2413 "<<imu_msg_buf.size()<<endl;


//        printf("m_buf_lock=%d\n",m_buf_lock);
         m_buf.lock();
         imu_msg_buf.push(imu_msg);
         m_buf.unlock();
         con.notify_one();
        imu_len++;
        }
//        TE(read_imu_buf);

     }];
}

/********************************************************************UI View Controler********************************************************************/
- (void)showInputView
{
    NSString *stringView;
    static bool finish_init = false;
    if(vins.solver_flag != vins.NON_LINEAR)
    {
        finish_init = false;
        switch (vins.init_status) {
            case vins.FAIL_IMU:
                stringView = [NSString stringWithFormat:@"STA: FAIL_IMU"];
                break;
            case vins.FAIL_PARALLAX:
                stringView = [NSString stringWithFormat:@"STA: FAIL_PARA"];
                break;
            case vins.FAIL_RELATIVE:
                stringView = [NSString stringWithFormat:@"STA: FAIL_RELA"];
                break;
            case vins.FAIL_SFM:
                stringView = [NSString stringWithFormat:@"STA: FAIL_SFM"];
                break;
            case vins.FAIL_PNP:
                stringView = [NSString stringWithFormat:@"STA: FAIL_PNP"];
                break;
            case vins.FAIL_ALIGN:
                stringView = [NSString stringWithFormat:@"STA: FAIL_ALIGN"];
                break;
            case vins.FAIL_CHECK:
                stringView = [NSString stringWithFormat:@"STA: FAIL_COST"];
                break;
            case vins.SUCC:
                stringView = [NSString stringWithFormat:@"STA: SUCC!"];
                break;
            default:
                break;
        }
        [_X_label setText:stringView];
        stringView = [NSString stringWithFormat:@"FAIL: %d times", vins.fail_times];
        [_Y_label setText:stringView];
        stringView = [NSString stringWithFormat:@"PARALLAX: %d", vins.parallax_num_view];
        [_Z_label setText:stringView];

        stringView = [NSString stringWithFormat:@"Initializing: %d%%", vins.initProgress];
        [_feature_label2 setText:stringView];

        [_feature_label2 setHidden:NO];
        [_feature_label3 setHidden:NO];
        [indicator setHidden:NO];
        [featureImageView setHidden:NO];
    }
    else
    {
        if(finish_init == false)
        {
            //Hide init UI
            [_feature_label2 setHidden:YES];
            [_feature_label3 setHidden:YES];
            [indicator setHidden:YES];
            [featureImageView setHidden:YES];

            start_show = true;
            finish_init = true;
        }

        float x_view = (float)vins.correct_Ps[frame_cnt][0];
        float y_view = (float)vins.correct_Ps[frame_cnt][1];
        float z_view = (float)vins.correct_Ps[frame_cnt][2];
        if(x_view_last == -5000)
        {
            x_view_last = x_view;
            
            y_view_last = y_view;
            z_view_last = z_view;
        }
        total_odom += sqrt(pow((x_view - x_view_last), 2) +
                           pow((y_view - y_view_last), 2) +
                           pow((z_view - z_view_last), 2));
        x_view_last = x_view;
        y_view_last = y_view;
        z_view_last = z_view;

        
        stringView = [NSString stringWithFormat:@"X:%d",global_frame_cnt];
//        stringView = [NSString stringWithFormat:@"X:%.2f",x_view];
        [_X_label setText:stringView];
        stringView = [NSString stringWithFormat:@"TOTAL:%.2f",total_odom];
        //stringView = [NSString stringWithFormat:@"COST:%.2lf",vins.final_cost];
        //stringView = [NSString stringWithFormat:@"COST: %d, %.2lf",vins.visual_factor_num, vins.visual_cost];
        [_total_odom_label setText:stringView];
        stringView = [NSString stringWithFormat:@"Y:%.2f",y_view];
        [_Y_label setText:stringView];
        stringView = [NSString stringWithFormat:@"Z:%.2f",z_view];
        [_Z_label setText:stringView];
    }
    stringView = [NSString stringWithFormat:@"BUF:%d",waiting_lists];
    [_buf_label setText:stringView];
    //NSString *stringZ = [NSString stringWithFormat:@"Z:%.2f",z_view, vins.f_manager.getFeatureCount()];
    if(loop_old_index != -1)
    {
        stringView = [NSString stringWithFormat:@"LOOP with %d",loop_old_index];
        [_loop_label setText:stringView];
    }
    stringView = [NSString stringWithFormat:@"FEATURE: %d",vins.feature_num];
    [_feature_label setText:stringView];
}

-(void)showOutputImage:(UIImage*)image
{
    [featureImageView setImage:image];
}
/********************************************************************UI View Controler********************************************************************/


/********************************************************************UI Button Controler********************************************************************/

-(IBAction)switchUI:(UISegmentedControl *)sender
{
    switch (_switchUI.selectedSegmentIndex)
    {
        case 0:
            self.switchUIAREnabled = YES;

//            printf("show AR\n");
            ui_main = true;
            box_in_AR= true;
            USE_PNP = true;
            imageCacheEnabled = cameraMode && !USE_PNP;
            break;
        case 1:
            self.switchUIAREnabled = NO;

            ui_main = false;
            if (box_in_AR)
                box_in_trajectory = true;
            USE_PNP = false;
            imageCacheEnabled = cameraMode && !USE_PNP;
//            printf("show VINS\n");
            break;
        default:
            break;
    }
}
- (IBAction)arButtonPressed:(id)sender {
    ui_main = true;//显示图像
    box_in_AR= true;//对应判断 改成画轨迹的
    USE_PNP = true;
    imageCacheEnabled = cameraMode && !USE_PNP;
    
    
//    self.metalARView =[[MetalView alloc] initWithFrame:[UIScreen mainScreen].applicationFrame];
//    self.metalARView.backgroundColor=[UIColor clearColor];
//    self.metalARView.frame=CGRectMake(0, 0,self.imageView.frame.size.width , self.imageView.frame.size.height);//40 0
//
//
//
//
//    cout<<"图层大小"<<self.imageView.frame.size.width<<" , "<<self.imageView.frame.size.height<<endl;
//    [self.imageView.layer addSublayer:self.metalARView.layer];
    [self initRendering];
    
    isOurAR=true;
}



- (IBAction)fovSliderValueChanged:(id)sender {
    self.fovLabel.text = [[NSNumber numberWithFloat:self.fovSlider.value] stringValue];

}

- (void) handlePan:(UIPanGestureRecognizer*) recognizer
{
    if(ui_main and 0)
        return;

    if (!ui_main)
    {
        CGPoint translation = [recognizer translationInView:self.view];
        CGFloat velocityX = [recognizer velocityInView:self.view].x;
        CGFloat velocityY = [recognizer velocityInView:self.view].y;
        //recognizer.view.center = CGPointMake(recognizer.view.center.x + translation.x,
        static CGFloat vx_last = 0;
        static CGFloat vy_last = 0;

        CGFloat vx_smooth = 0.5*velocityX + 0.5*vx_last;
        CGFloat vy_smooth = 0.5*velocityY + 0.5*vy_last;
        vx_last = vx_smooth;
        vy_last = vy_smooth;
        if(recognizer.numberOfTouches == 2)
        {
            vins.drawresult.Y0 += vx_smooth/100.0;
            vins.drawresult.X0 += vy_smooth/100.0;
        }
        else
        {
            vins.drawresult.theta += vy_smooth/100.0;
            vins.drawresult.theta = fmod(vins.drawresult.theta, 360.0);
            vins.drawresult.phy += vx_smooth/100.0;
            vins.drawresult.phy = fmod(vins.drawresult.phy, 360.0);
        }

        vins.drawresult.change_view_manualy = true;
    }
    else
    {
        CGPoint translation = [recognizer translationInView:self.view];
        CGFloat velocityX = [recognizer velocityInView:self.view].x;
        CGFloat velocityY = [recognizer velocityInView:self.view].y;
        //CGFloat translationX =
        //CGFloat translationY = [recognizer translationInView:self.view].y;
        //NSLog(@"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!pipikk test x: %f y: %f", translationX, translationY);
        //NSLog(@"!!!!!!!!!!!!!!!!!!!!!!%f  %f", imageView.frame.size.height, imageView.frame.size.width);
        CGPoint point = [recognizer locationInView:self.view];
        //NSLog(@"X Location: %f", point.x);
        //NSLog(@"Y Location: %f",point.y);

        //recognizer.view.center = CGPointMake(recognizer.view.center.x + translation.x,
        static CGFloat vx_lastAR = 0;
        static CGFloat vy_lastAR = 0;

        CGFloat vx_smooth = 0.5*velocityX + 0.5*vx_lastAR;
        CGFloat vy_smooth = 0.5*velocityY + 0.5*vy_lastAR;
        vx_lastAR = vx_smooth;
        vy_lastAR = vy_smooth;
        if(recognizer.numberOfTouches == 2)
        {

            vins.drawresult.Y0AR += vx_smooth/100.0;
            vins.drawresult.X0AR += vy_smooth/100.0;

//            vins.drawresult.locationXT2 = point.x * 640.0 / imageView.frame.size.width;
//            vins.drawresult.locationYT2 = point.y * 480.0 / imageView.frame.size.height;
            
            vins.drawresult.locationXT2 = point.x * 480.0 / imageView.frame.size.width;
            vins.drawresult.locationYT2 = point.y * 752.0 / imageView.frame.size.height;

            vins.drawresult.finger_s = 0;
            vins.drawresult.finger_p = 0;
            if ((vins.drawresult.finger_d ++) > 7)
                vins.drawresult.finger_state = 2;
        }
        else
        {
            vins.drawresult.thetaAR += vy_smooth/100.0;
            //vins.drawresult.thetaAR = fmod(vins.drawresult.thetaAR, 360.0);
            vins.drawresult.phyAR += vx_smooth/100.0;
            //vins.drawresult.phyAR = fmod(vins.drawresult.phyAR, 360.0);

//            vins.drawresult.locationX = point.x * 640.0 / imageView.frame.size.width;
//            vins.drawresult.locationY = point.y * 480.0 / imageView.frame.size.height;
            
            vins.drawresult.locationX = point.x * 480.0 / imageView.frame.size.width;
            vins.drawresult.locationY = point.y * 752.0 / imageView.frame.size.height;

            vins.drawresult.finger_d = 0;
            vins.drawresult.finger_p = 0;
            if ((vins.drawresult.finger_s ++) > 7)
                vins.drawresult.finger_state = 1;
        }
    }


}

- (void) handlePinch:(UIPinchGestureRecognizer*) recognizer
{
    if(ui_main and 0)
        return;

    if (!ui_main)
    {
        vins.drawresult.change_view_manualy = true;
        if(vins.drawresult.radius > 5 || recognizer.velocity < 0)
            vins.drawresult.radius -= recognizer.velocity * 0.5;
        else
        {
            vins.drawresult.Fx += recognizer.velocity * 15;
            if(vins.drawresult.Fx < 50)
                vins.drawresult.Fx = 50;
            vins.drawresult.Fy += recognizer.velocity * 15;
            if(vins.drawresult.Fy < 50)
                vins.drawresult.Fy = 50;
        }
    }
    else{

        vins.drawresult.finger_s = 0;
        vins.drawresult.finger_d = 0;
        if ((vins.drawresult.finger_p ++) > 7)
            vins.drawresult.finger_state = 3;

        CGPoint point = [recognizer locationInView:self.view];
//        vins.drawresult.locationXP = point.x * 640.0 / imageView.frame.size.width;
//        vins.drawresult.locationYP = point.y * 480.0 / imageView.frame.size.height;
        
       vins.drawresult.locationXP = point.x * 480.0 / imageView.frame.size.width;
        vins.drawresult.locationYP = point.y * 752.0 / imageView.frame.size.height;

        //NSLog(@"pipikk_radius: %f velocity: ", vins.drawresult.radiusAR, recognizer.velocity);

        //if(vins.drawresult.radiusAR > 5 || recognizer.velocity < 0)
        //{
        vins.drawresult.radiusAR -= recognizer.velocity * 0.5;
        //}
    }

}

- (void) handleTap:(UITapGestureRecognizer*) recognizer
{
//    NSLog(@"打印 添加物体");

    if (!ui_main)
    {

    }
    else{

        /*vins.drawresult.finger_s = 0;
         vins.drawresult.finger_d = 0;
         if ((vins.drawresult.finger_p ++) > 7)
         vins.drawresult.finger_state = 3;*/
//480 752
        CGPoint point = [recognizer locationInView:self.view];
//        vins.drawresult.locationTapX = point.x * 640.0 / imageView.frame.size.width;
//        vins.drawresult.locationTapY = point.y * 480.0 / imageView.frame.size.height;
        
        vins.drawresult.locationTapX = point.x * 480.0 / imageView.frame.size.width;
        vins.drawresult.locationTapY = point.y * 752.0 / imageView.frame.size.height;

        vins.drawresult.tapFlag = true;
//        NSLog(@"成功改变 标签");
    }

}

- (void) handleLongPress:(UILongPressGestureRecognizer*) recognizer
{
    if (!ui_main)
    {

    }
    {
        CGPoint point = [recognizer locationInView:self.view];
//        vins.drawresult.locationLongPressX = point.x * 640.0 / imageView.frame.size.width;
//        vins.drawresult.locationLongPressY = point.y * 480.0 / imageView.frame.size.height;
        
        vins.drawresult.locationLongPressX = point.x * 480.0 / imageView.frame.size.width;
        vins.drawresult.locationLongPressY = point.y * 752.0 / imageView.frame.size.height;
        vins.drawresult.longPressFlag = true;
    }
}

- (IBAction)loopButtonPressed:(id)sender {
    if(LOOP_CLOSURE)
    {
        LOOP_CLOSURE = false;
        [_loopButton setTitle:@"ENLOOP" forState:UIControlStateNormal];
    }
    else
    {
        LOOP_CLOSURE = true;
        [_loopButton setTitle:@"UNLOOP" forState:UIControlStateNormal];
    }

//     start_record = !start_record;
//     if(start_record)
//     {
//         start_playback = false;
//        //     [_recordButton setTitle:@"Stop" forState:UIControlStateNormal];
//         [saveData start];
//     }
//     else
//     {
//         TS(record_imu);
//         imuData.header = 0; // as the ending marker
//         imuData.acc << 0,0,0;
//         imuData.gyr << 0,0,0;
//         [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
//         [self recordImu];
//         TE(record_imu);
////         [_recordButton setTitle:@"Record" forState:UIControlStateNormal];
//     }

}

- (IBAction)reinitButtonPressed:(id)sender {
    vins.drawresult.planeInit = false;
    vins.failure_hand = true;
    vins.drawresult.change_color = true;
    vins.drawresult.indexs.push_back(vins.drawresult.pose.size());
    segmentation_index++;
    keyframe_database.max_seg_index++;
    keyframe_database.cur_seg_index = keyframe_database.max_seg_index;


//     start_playback = !start_playback;
//    start_playback=true;
//     if(start_playback)
//     {
//     TS(read_imu);
//     NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
//     NSString *documentsPath = [paths objectAtIndex:0];
//     NSString *filePath = [documentsPath stringByAppendingPathComponent:@"IMU"]; //Add the file name
//     imuReader = [NSData dataWithContentsOfFile:filePath];
//     TE(read_imu);
//     start_record = false;
//     [_playbackButton setTitle:@"Stop" forState:UIControlStateNormal];
//     }
//     else
//     [_playbackButton setTitle:@"Playback" forState:UIControlStateNormal];

}
- (IBAction)saveButtonPressed:(id)sender {
    if(start_record){
        start_record=false;

         imuData.header = 0; // as the ending marker
         imuData.acc << 0,0,0;
         imuData.gyr << 0,0,0;
         [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
         [self recordImu];

        NSLog(@"record data end_will");
    }

    if(start_record_2){
        start_record_2=false;

         imuData.header = 0; // as the ending marker
         imuData.acc << 0,0,0;
         imuData.gyr << 0,0,0;
         [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
         [self recordImu_2];

        NSLog(@"record data end_will");
    }
//    cout<<"img_num"<<img_num<<endl;

        if (isCapturing)
        {
            [videoCamera stop];
        }
        [mainLoop cancel];
        [draw cancel];
    #ifdef LOOP_CLOSURE
        [loop_thread cancel];
    #endif
    #ifdef LOOP_CLOSURE_SERVER
        [loop_thread_server cancel];
    #endif
    #ifdef LOOP_CLOSURE_SERVER_noLoop
        [loop_thread_server cancel];
    #endif

    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [paths objectAtIndex:0];

    NSString *filePath_tum_pose = [documentsPath stringByAppendingPathComponent:@"tum_pose"];
    [self checkFilePath:filePath_tum_pose];

//                [NSThread sleepForTimeInterval:30];
    //这里可能得加个锁 不安全
    NSMutableData *poseDataBuf_time_r_t = [[NSMutableData alloc] init];
    vector<double> time=keyframe_database.path_time;
    vector<Eigen::Vector3f> path=keyframe_database.refine_path;
    vector<Eigen::Quaterniond> pose_r=keyframe_database.refine_r;
    int len=time.size();
    for(int i=0;i<len;i++){
        [poseDataBuf_time_r_t appendBytes:&(time[i]) length:sizeof(double)];
        [poseDataBuf_time_r_t appendBytes:&(path[i]) length:sizeof(path[i])];
        [poseDataBuf_time_r_t appendBytes:&(pose_r[i]) length:sizeof(pose_r[i])];
    }
//                printf("位姿保存完了 验证一下 %f, %f, %f, %f\n",pose_r[0].x(), pose_r[0].y(),pose_r[0].z(),pose_r[0].w());
    BOOL writeImuSuccess_time=[poseDataBuf_time_r_t writeToFile:filePath_tum_pose atomically:YES];
//    if(writeImuSuccess_time){
//            NSLog(@"位姿时间戳已保存完整succ");
//    }else{
//            NSLog(@"位姿时间戳已保存完整fail");
//    }


}

//ljl
- (void)connectServer{
    [SocketSingleton sharedInstance].socketHost = self.hostText.text;// host设定
    [SocketSingleton sharedInstance].socketPort = self.portText.text.integerValue;// port设定

    // 在连接前先进行手动断开
    [SocketSingleton sharedInstance].socket.userData = SocketOfflineByUser;
    [[SocketSingleton sharedInstance] cutOffSocket];

    // 确保断开后再连，如果对一个正处于连接状态的socket进行连接，会出现崩溃
    [SocketSingleton sharedInstance].socket.userData = SocketOfflineByServer;
    [[SocketSingleton sharedInstance] socketConnectHost];

}


/********************************************************************UI Button Controler********************************************************************/


/***********************************************************About record and playback data for debug********************************************************/

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

-(void)saveData{
    while (![[NSThread currentThread] isCancelled])
    {
        @autoreleasepool
        {
            if(!imgDataBuf.empty())
            {
                IMG_DATA tmp_data;
                tmp_data = imgDataBuf.front();
                imgDataBuf.pop();
                [self recordImageTime:tmp_data];
                [self recordImage:tmp_data];
                imageDataIndex++;
                NSLog(@"record img: %lf %lu",tmp_data.header,imageDataIndex);

//                UIImage *img_test=tmp_data.image;

            }

            if(start_record==false && imgDataBuf.empty()){
//                [self recordImu];
                [saveData cancel];
                cout<<"record end"<<endl;

            }
        }
        [NSThread sleepForTimeInterval:0.02];//0.04
    }
}

-(void)saveData_2{
    while (![[NSThread currentThread] isCancelled])
    {
        @autoreleasepool
        {
            if(!imgDataBuf.empty())
            {
                IMG_DATA tmp_data;
                tmp_data = imgDataBuf.front();
                imgDataBuf.pop();
                [self recordImageTime_2:tmp_data];
                [self recordImage_2:tmp_data];
                imageDataIndex++;
                NSLog(@"record img: %lf %lu",tmp_data.header,imageDataIndex);

//                UIImage *img_test=tmp_data.image;

            }

            if(start_record_2==false && imgDataBuf.empty()){
//                [self recordImu];
                [saveData_2 cancel];
                cout<<"record end"<<endl;

            }
        }
        [NSThread sleepForTimeInterval:0.02];//0.04
    }
}

- (void)tapSaveImageToIphone:(UIImage*)image
{
    UIImageWriteToSavedPhotosAlbum(image, self, @selector(image:didFinishSavingWithError:contextInfo:), nil);
}

- (void)image:(UIImage *)image didFinishSavingWithError:(NSError *)error contextInfo:(void *)contextInfo{

    if (error == nil) {
        NSLog(@"save access");
    }else{
        NSLog(@"save failed");
    }
}

- (void)checkDirectoryPath:(unsigned long)index withObject:(NSString*)directoryPath
{
    //delete already exist directory first time
    NSError *error;
    if (index == 0 && [[NSFileManager defaultManager] fileExistsAtPath:directoryPath])    //Does directory exist?
    {
        if (![[NSFileManager defaultManager] removeItemAtPath:directoryPath error:&error])    //Delete it
        {
            NSLog(@"Delete directory error: %@", error);
        }
    }

    //creat file directory if it does not exist
    if (![[NSFileManager defaultManager] fileExistsAtPath:directoryPath])
    {
        NSLog(@"directory does not exist :%@",directoryPath);
        if (![[NSFileManager defaultManager] createDirectoryAtPath:directoryPath
                                       withIntermediateDirectories:NO
                                                        attributes:nil
                                                             error:&error])
        {
            NSLog(@"Create directory error: %@", error);
        }
    }
}

//只调用一次
- (void)checkFilePath:(NSString*)FilePath
{
    //delete already exist file first time
    BOOL isDir = NO;
    NSError *error;
    if ( [[NSFileManager defaultManager] fileExistsAtPath:FilePath isDirectory:&isDir])    //Does directory exist?
    {
        if (![[NSFileManager defaultManager] removeItemAtPath:FilePath error:&error])    //Delete it
        {
            NSLog(@"Delete file error: %@", error);
        }
    }

    //creat file directory if it does not exist
    if (![[NSFileManager defaultManager] fileExistsAtPath:FilePath])
    {
        NSLog(@"file does not exist ，%@",FilePath);
        [[NSFileManager defaultManager] createFileAtPath:FilePath contents:nil
                                              attributes:nil];
    }
}

- (void)recordImu
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [paths objectAtIndex:0];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:@"IMU"]; //Add the file name

    [self checkFilePath:filePath];

    BOOL writeImuSuccess=[imuDataBuf writeToFile:filePath atomically:YES];
    if(writeImuSuccess){
            NSLog(@"record imu succ");
    }else{
            NSLog(@"record imu fail");
    }

//    [msgData writeToFile:filePath atomically:YES];
}

- (void)recordImu_myself
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMU"];; //Get the docs directory

    [self checkDirectoryPath:imuDataIndex withObject:documentsPath];

    NSString *filename = [NSString stringWithFormat:@"%lu", imuDataIndex];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name

    BOOL writeImuSuccess=[imuDataBuf writeToFile:filePath atomically:YES];
//    if(writeImuSuccess){
//            NSLog(@"recordImu_myself succ");
//    }else{
//            NSLog(@"recordImu_myself fail");
//    }

    [imuDataBuf resetBytesInRange:NSMakeRange(0, [imuDataBuf length])];

    [imuDataBuf setLength:0];
//    [msgData writeToFile:filePath atomically:YES];
}

- (void)recordVins
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [paths objectAtIndex:0];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:@"VINS"]; //Add the file name

    [vinsDataBuf writeToFile:filePath atomically:YES];
    //[msgData writeToFile:filePath atomically:YES];
}

- (void)recordImageTime:(IMG_DATA&)image_data
{
        NSLog(@"record image");
    double time = image_data.header;
    NSData *msgData = [NSData dataWithBytes:&time length:sizeof(time)];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME"];; //Get the docs directory

//    [self checkDirectoryPath:imageDataIndex withObject:documentsPath];
//
//    NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
//    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
//
//    BOOL writeImgSuccess=[msgData writeToFile:filePath atomically:YES];
//    if(writeImgSuccess){
//            NSLog(@"record img_time succ");
//    }else{
//            NSLog(@"record img_time fail");
//    }
}

- (void)recordImage:(IMG_DATA&)image_data
{
    NSData *msgData = UIImagePNGRepresentation(image_data.image);
    //NSData *msgData = [NSData dataWithBytes:&image_data length:sizeof(image_data)];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE"];; //Get the docs directory

    [self checkDirectoryPath:imageDataIndex withObject:documentsPath];

    NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name

   BOOL writeImgSuccess= [msgData writeToFile:filePath atomically:YES];
    if(writeImgSuccess){
            NSLog(@"record img_data succ");
    }else{
            NSLog(@"record img_data fail");
    }

    msgData=nil;


//    @autoreleasepool {
//        CFURLRef url=(__bridge CFURLRef)[NSURL fileURLWithPath:filePath];
//        CGImageDestinationRef destination=CGImageDestinationCreateWithURL(url, kUTTypePNG, 1 , NULL);
//        if(!destination){
//
//        }
//
//        CGImageDestinationAddImage(destination, image_data.image.CGImage, nil);
//        if(!CGImageDestinationFinalize(destination)){
//
//        }
//        CFRelease(destination);
//    }


}


//----------------------------------记录第2中数据

- (void)recordImage_2:(IMG_DATA&)image_data
{
    NSData *msgData = UIImagePNGRepresentation(image_data.image);
    //NSData *msgData = [NSData dataWithBytes:&image_data length:sizeof(image_data)];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_3"];; //Get the docs directory

    [self checkDirectoryPath:imageDataIndex withObject:documentsPath];

    NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name

   BOOL writeImgSuccess= [msgData writeToFile:filePath atomically:YES];
    if(writeImgSuccess){
            NSLog(@"record img_data succ");
    }else{
            NSLog(@"record img_data fail");
    }

    msgData=nil;

}

- (void)recordImageTime_2:(IMG_DATA&)image_data
{
        NSLog(@"record image");
    double time = image_data.header;
    NSData *msgData = [NSData dataWithBytes:&time length:sizeof(time)];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME_2"];; //Get the docs directory

    [self checkDirectoryPath:imageDataIndex withObject:documentsPath];

    NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name

    BOOL writeImgSuccess=[msgData writeToFile:filePath atomically:YES];
    if(writeImgSuccess){
            NSLog(@"record img_time succ");
    }else{
            NSLog(@"record img_time fail");
    }
}

- (void)recordImu_2
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [paths objectAtIndex:0];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:@"IMU_2"]; //Add the file name

    [self checkFilePath:filePath];

    BOOL writeImuSuccess=[imuDataBuf writeToFile:filePath atomically:YES];
    if(writeImuSuccess){
            NSLog(@"record imu succ");
    }else{
            NSLog(@"record imu fail");
    }

    //[msgData writeToFile:filePath atomically:YES];
}

//*************************************读数据


-(bool)readImageTime:(unsigned long)index
{
    bool file_exist;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME"]; //Get the docs directory
    NSString *filename = [NSString stringWithFormat:@"%lu", index];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name

    //check file exists
    if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
    {
        NSData *file1 = [[NSData alloc] initWithContentsOfFile:filePath];
        if (file1)
        {
            double time;
            [file1 getBytes:&time length:sizeof(time)];
            imgData.header = time;
        }
        file_exist = true;
    }
    else
    {
        file_exist = false;
        //NSLog(@"File does not exist");
    }
    return file_exist;
}

//imu信息保存成了一个一个的文件
-(bool)readImu_myself:(unsigned long)index
{
    bool file_exist;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMU"]; //Get the docs directory
    NSString *filename = [NSString stringWithFormat:@"%lu", index];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name

    //check file exists
    if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
    {
        NSData *file1 = [[NSData alloc] initWithContentsOfFile:filePath];
        if (file1)
        {
            IMU_MSG curr_imu;
            [file1 getBytes:&curr_imu length:sizeof(curr_imu)];
            imuData = curr_imu;
//            cout<<"imuData "<<imuData.header<<" "<<imuData.acc[0]<<" "<<imuData.acc[1]<<" "<<imuData.acc[2]<<" "<<imuData.gyr[0]<<" "<<imuData.gyr[1]<<" "<<imuData.gyr[2]<<endl;
        }
        file_exist = true;
    }
    else
    {
        file_exist = false;
        NSLog(@"readImu_myself File does not exist");
    }
    return file_exist;
}

-(bool)readImage:(unsigned long)index
{
    bool file_exist;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE"]; //Get the docs directory
    NSString *filename = [NSString stringWithFormat:@"%lu", index];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name

    //check file exists
    if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
    {
        NSData *pngData = [[NSData alloc] initWithContentsOfFile:filePath];//会自动释放

        imgData.image = [[UIImage alloc] initWithData:pngData];//会自动释放
        pngData=nil;
        file_exist = true;

    }
    else
    {
        file_exist = false;
        //NSLog(@"File does not exist");
    }
    return file_exist;
}



//这个用来读euroc的数据


-(void)readImage_euroc
{
    string imgPathAddName=imgName_Path+imgName;

    NSString *imgPathAddName_NS = [NSString stringWithCString:imgPathAddName.c_str() encoding:[NSString defaultCStringEncoding]];
//    cv::Mat src =cv::imread(imgPathAddName);
    if ([[NSFileManager defaultManager] fileExistsAtPath:imgPathAddName_NS])
    {
        NSData *pngData = [[NSData alloc] initWithContentsOfFile:imgPathAddName_NS];//会自动释放

        imgData.image = [[UIImage alloc] initWithData:pngData];//会自动释放
        pngData=nil;



    }else{
        cout<<"读取的图片为空"<<imgPathAddName<<endl;
    }


}

/**************************************************************About record and playback data for debug**********************************************************/

- (void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
}

- (void)viewDidDisappear:(BOOL)animated
{
    [super viewDidDisappear:animated];

//    if(start_record){
//        [self recordImu];
//        [saveData cancel];
//        NSLog(@"record data end");
//    }

    if (isCapturing)
    {
        [videoCamera stop];
    }
    [mainLoop cancel];
    [draw cancel];
#ifdef LOOP_CLOSURE
    [loop_thread cancel];
#endif
#ifdef LOOP_CLOSURE_SERVER
    [loop_thread_server cancel];
#endif
#ifdef LOOP_CLOSURE_SERVER_noLoop
    [loop_thread_server cancel];
#endif

}

-(void)viewDidUnload{
    [motionManager stopAccelerometerUpdates];
    [motionManager stopDeviceMotionUpdates];
    [motionManager stopGyroUpdates];
    [motionManager stopMagnetometerUpdates];
    [super viewDidUnload];
}

- (void)dealloc
{
    videoCamera.delegate = nil;
}

/*
 Check the device
 */
DeviceType deviceName()
{
    struct utsname systemInfo;
    uname(&systemInfo);

    NSString *device = [NSString stringWithCString:systemInfo.machine
                                          encoding:NSUTF8StringEncoding];
    DeviceType device_type;
    if(([device compare:@"iPhone10,1"] == NSOrderedSame) ||
       ([device compare:@"iPhone10,3"] == NSOrderedSame))
    {
        printf("Device iPhone8\n");
        device_type = iPhone8;
    }
    else if(([device compare:@"iPhone9,1"] == NSOrderedSame) ||
       ([device compare:@"iPhone9,3"] == NSOrderedSame))
    {
        printf("Device iPhone7\n");
        device_type = iPhone7;
    }
    else if(([device compare:@"iPhone9,2"] == NSOrderedSame) ||
            ([device compare:@"iPhone9,4"] == NSOrderedSame))
    {
        printf("Device iPhone7 plus\n");
        device_type = iPhone7P;
    }
    else if(([device compare:@"iPhone8,2"] == NSOrderedSame))
    {
        printf("Device iPhone6s plus\n");
        device_type = iPhone6sP;
    }
    else if(([device compare:@"iPhone8,1"] == NSOrderedSame))
    {
        printf("Device iPhone6s\n");
        device_type = iPhone6s;
    }
    else if(([device compare:@"iPad6,3"] == NSOrderedSame)||
            ([device compare:@"iPad6,4"] == NSOrderedSame))
    {
        printf("Device iPad pro 9.7\n");
        device_type = iPadPro97;
    }
    else if(([device compare:@"iPad6,7"] == NSOrderedSame)||
            ([device compare:@"iPad6,8"] == NSOrderedSame))
    {
        printf("Device iPad pro 12.9\n");
        device_type = iPadPro129;
    }
    else
    {
        printf("Device undefine\n");
        device_type = unDefine;
    }
    return device_type;
}

bool iosVersion()
{
    NSComparisonResult order = [[UIDevice currentDevice].systemVersion compare: @"10.2.1" options: NSNumericSearch];
    if (order == NSOrderedSame || order == NSOrderedDescending) {
        printf("system version >= 10.2.1\n");
        return true;
    } else {
        printf("system version < 10.2.1\n");
        return false;
    }
}




string imgName_Path="";




- (void) fileContents_img_2_buf:(int)index{
//    for(id obj in _allLinedStrings){
//        NSLog(@"%@",obj);
//        if([obj length]<=0){
//            break;
//        }
        NSArray *imgs = [[_allLinedStrings_imgTime objectAtIndex:index] componentsSeparatedByString:@","];

        NSString *time = [imgs objectAtIndex:0];
        NSString *img_name = [imgs objectAtIndex:1];


        NSDecimalNumber *time_number = [NSDecimalNumber decimalNumberWithString:time];
        double time_cur=[time_number doubleValue];

        imgData.header = time_cur;

        imgName=[img_name UTF8String];
        //euroc 需要去掉文件末尾的/r
    size_t n = imgName.find_last_not_of( "\r" );
    if( n != string::npos )
    {
        imgName.erase( n + 1 , imgName.size() - n );
    }

//    printf("img time=%.9lf\n",time_cur);
//    }
}

- (void) fileContents_imu_2_buf:(int)index{
//    for(id obj in _allLinedStrings){
//        NSLog(@"%@",obj);
//        if([obj length]<=0){
//            break;
//        }
        NSArray *imgs = [[_allLinedStrings_imuTime objectAtIndex:index] componentsSeparatedByString:@","];

        NSString *time = [imgs objectAtIndex:0];
        NSString *gyr_x = [imgs objectAtIndex:1];
        NSString *gyr_y = [imgs objectAtIndex:2];
        NSString *gyr_z = [imgs objectAtIndex:3];
        NSString *acc_x = [imgs objectAtIndex:4];
        NSString *acc_y = [imgs objectAtIndex:5];
        NSString *acc_z = [imgs objectAtIndex:6];

        NSDecimalNumber *time_number = [NSDecimalNumber decimalNumberWithString:time];
        double time_cur=[time_number doubleValue];

        NSDecimalNumber *gyr_x_number = [NSDecimalNumber decimalNumberWithString:gyr_x];
        double gyr_x_cur=[gyr_x_number doubleValue];
        NSDecimalNumber *gyr_y_number = [NSDecimalNumber decimalNumberWithString:gyr_y];
        double gyr_y_cur=[gyr_y_number doubleValue];
        NSDecimalNumber *gyr_z_number = [NSDecimalNumber decimalNumberWithString:gyr_z];
        double gyr_z_cur=[gyr_z_number doubleValue];
        NSDecimalNumber *acc_x_number = [NSDecimalNumber decimalNumberWithString:acc_x];
        double acc_x_cur=[acc_x_number doubleValue];
        NSDecimalNumber *acc_y_number = [NSDecimalNumber decimalNumberWithString:acc_y];
        double acc_y_cur=[acc_y_number doubleValue];
        NSDecimalNumber *acc_z_number = [NSDecimalNumber decimalNumberWithString:acc_z];
        double acc_z_cur=[acc_z_number doubleValue];
        imuData.header = time_cur;
        imuData.acc <<acc_x_cur,acc_y_cur,acc_z_cur;
        imuData.gyr <<gyr_x_cur,gyr_y_cur,gyr_z_cur;

//         NSLog(@"imu time: %@,%@,%@,%@,%@,%@,%@",time,gyr_x,gyr_y,gyr_z,acc_x,acc_y,acc_z);
//    }
}


- (void) fileContents_img_2_buf_kitti:(int)index{
//    for(id obj in _allLinedStrings){
//        NSLog(@"%@",obj);
//        if([obj length]<=0){
//            break;
//        }
        NSArray *imgs = [[_allLinedStrings_imgTime objectAtIndex:index] componentsSeparatedByString:@","];

        NSString *time = [imgs objectAtIndex:0];
        NSString *img_name = [imgs objectAtIndex:1];


        NSDecimalNumber *time_number = [NSDecimalNumber decimalNumberWithString:time];
        double time_cur=[time_number doubleValue];
    time_cur*=1.0e9;// 这个是要的 euroc
        imgData.header = time_cur;

        imgName=[img_name UTF8String];
        //euroc 需要去掉文件末尾的/r
    size_t n = imgName.find_last_not_of( "\r" );
    if( n != string::npos )
    {
        imgName.erase( n + 1 , imgName.size() - n );
    }

//    printf("img time=%.9lf\n",time_cur);
//    }
}

- (void) fileContents_imu_2_buf_kitti:(int)index{

        NSArray *imgs = [[_allLinedStrings_imuTime objectAtIndex:index] componentsSeparatedByString:@","];

        NSString *time = [imgs objectAtIndex:0];
        NSString *gyr_x = [imgs objectAtIndex:1];
        NSString *gyr_y = [imgs objectAtIndex:2];
        NSString *gyr_z = [imgs objectAtIndex:3];
        NSString *acc_x = [imgs objectAtIndex:4];
        NSString *acc_y = [imgs objectAtIndex:5];
        NSString *acc_z = [imgs objectAtIndex:6];

        NSDecimalNumber *time_number = [NSDecimalNumber decimalNumberWithString:time];
        double time_cur=[time_number doubleValue];
        time_cur*=1.0e9;//这个是要的
        NSDecimalNumber *gyr_x_number = [NSDecimalNumber decimalNumberWithString:gyr_x];
        double gyr_x_cur=[gyr_x_number doubleValue];
        NSDecimalNumber *gyr_y_number = [NSDecimalNumber decimalNumberWithString:gyr_y];
        double gyr_y_cur=[gyr_y_number doubleValue];
        NSDecimalNumber *gyr_z_number = [NSDecimalNumber decimalNumberWithString:gyr_z];
        double gyr_z_cur=[gyr_z_number doubleValue];
        NSDecimalNumber *acc_x_number = [NSDecimalNumber decimalNumberWithString:acc_x];
        double acc_x_cur=[acc_x_number doubleValue];
        NSDecimalNumber *acc_y_number = [NSDecimalNumber decimalNumberWithString:acc_y];
        double acc_y_cur=[acc_y_number doubleValue];
        NSDecimalNumber *acc_z_number = [NSDecimalNumber decimalNumberWithString:acc_z];
        double acc_z_cur=[acc_z_number doubleValue];
        imuData.header = time_cur;
        imuData.acc <<acc_x_cur,acc_y_cur,acc_z_cur;
        imuData.gyr <<gyr_x_cur,gyr_y_cur,gyr_z_cur;

//         NSLog(@"imu time: %@,%@,%@,%@,%@,%@,%@",time_cur,gyr_x,gyr_y,gyr_z,acc_x,acc_y,acc_z);
//    }

}


- (void) fileContents_img_2_buf_mvsec:(int)index{

    NSArray *imgs = [[_allLinedStrings_imgTime objectAtIndex:index] componentsSeparatedByString:@","];

    NSString *time = [imgs objectAtIndex:0];
    NSString *img_name = [imgs objectAtIndex:1];


    NSDecimalNumber *time_number = [NSDecimalNumber decimalNumberWithString:time];
    double time_cur=[time_number doubleValue];
    time_cur*=1.0e6;
    imgData.header = time_cur;

    imgName=[img_name UTF8String];
    //euroc 需要去掉文件末尾的/r
    size_t n = imgName.find_last_not_of( "\r" );
    if( n != string::npos )
    {
        imgName.erase( n + 1 , imgName.size() - n );
    }

//    printf("img time=%.9lf\n",time_cur);
}

- (void) fileContents_imu_2_buf_mvsec:(int)index{

    NSArray *imgs = [[_allLinedStrings_imuTime objectAtIndex:index] componentsSeparatedByString:@","];

    NSString *time = [imgs objectAtIndex:0];
    NSString *gyr_x = [imgs objectAtIndex:1];
    NSString *gyr_y = [imgs objectAtIndex:2];
    NSString *gyr_z = [imgs objectAtIndex:3];
    NSString *acc_x = [imgs objectAtIndex:4];
    NSString *acc_y = [imgs objectAtIndex:5];
    NSString *acc_z = [imgs objectAtIndex:6];

    NSDecimalNumber *time_number = [NSDecimalNumber decimalNumberWithString:time];
    double time_cur=[time_number doubleValue];

    NSDecimalNumber *gyr_x_number = [NSDecimalNumber decimalNumberWithString:gyr_x];
    double gyr_x_cur=[gyr_x_number doubleValue];
    NSDecimalNumber *gyr_y_number = [NSDecimalNumber decimalNumberWithString:gyr_y];
    double gyr_y_cur=[gyr_y_number doubleValue];
    NSDecimalNumber *gyr_z_number = [NSDecimalNumber decimalNumberWithString:gyr_z];
    double gyr_z_cur=[gyr_z_number doubleValue];
    NSDecimalNumber *acc_x_number = [NSDecimalNumber decimalNumberWithString:acc_x];
    double acc_x_cur=[acc_x_number doubleValue];
    NSDecimalNumber *acc_y_number = [NSDecimalNumber decimalNumberWithString:acc_y];
    double acc_y_cur=[acc_y_number doubleValue];
    NSDecimalNumber *acc_z_number = [NSDecimalNumber decimalNumberWithString:acc_z];
    double acc_z_cur=[acc_z_number doubleValue];

    time_cur*=1.0e-3;
    imuData.header = time_cur;
    imuData.acc <<acc_x_cur,acc_y_cur,acc_z_cur;
    imuData.gyr <<gyr_x_cur,gyr_y_cur,gyr_z_cur;

//         NSLog(@"imu time: %@,%@,%@,%@,%@,%@,%@",time,gyr_x,gyr_y,gyr_z,acc_x,acc_y,acc_z);

}

@end
