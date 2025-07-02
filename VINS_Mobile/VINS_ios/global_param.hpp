//
//  global_param.h
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/20.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef global_param_h
#define global_param_h

enum DeviceType
{
    iPhone8,
    iPhone7P,
    iPhone7,
    iPhone6sP,
    iPhone6s,
    iPadPro97,
    iPadPro129,
    euroc,
    xiaoMi,
    kitti0930,
    kitti1003,
    tum,
    day1,
    night1,
    unDefine
};
//extrinsic param ios
//#define RIC_y ((double)0.0)
//#define RIC_p ((double)0.0)
//#define RIC_r ((double)180.0)
//角度euroc
#define RIC_y ((double)89.148)
#define RIC_p ((double)1.47693)
#define RIC_r ((double)0.215286)
//xiaomi
//#define RIC_y ((double)-90)
//#define RIC_p ((double)0)
//#define RIC_r ((double)180)
//kitti0930 -28.7097 -89.4896  119.561
//#define RIC_y ((double)-89.5523)
//#define RIC_p ((double)-0.849974)
//#define RIC_r ((double)-90.2518)
//kitti1003
//#define RIC_y ((double)-89.4985)
//#define RIC_p ((double)-0.799884)
//#define RIC_r ((double)-90.2818)
//day1
//#define RIC_y ((double)-10.4633)
//#define RIC_p ((double)17.1681)
//#define RIC_r ((double)-1.94527)

//day1
//#define RIC_y ((double)-0.489331)
//#define RIC_p ((double)0.138947)
//#define RIC_r ((double)0.525142)


//day2
//#define RIC_y ((double)-0.369508)
//#define RIC_p ((double)-0.133858)
//#define RIC_r ((double)0.555428)

//night1
//#define RIC_y ((double)-0.549434)
//#define RIC_p ((double)-0.0326162)
//#define RIC_r ((double)0.37786)

//night3
//#define RIC_y ((double)-0.439472)
//#define RIC_p ((double)-0.0593143)
//#define RIC_r ((double)0.26293)


//euroc 这个注释掉了
//#define MIN_LOOP_NUM 13
#define MIN_LOOP_NUM 22
#define LOOP_FREQ 3
#define WINDOW_SIZE 10
#define PNP_SIZE 6
#define SIZE_POSE 7
#define SIZE_SPEEDBIAS 9
#define SIZE_SPEED 3
#define SIZE_BIAS  6

#define SIZE_FEATURE 1

#define NUM_OF_F 1000
#define NUM_OF_CAM 1
#define DEBUG_MODE false
#define C_PI 3.1415926

//ios
//#define GRAVITY ((double)9.805)
//#define ACC_N ((double)0.5)  //0.02
//#define ACC_W ((double)0.002)
//#define GYR_N ((double)0.2)  //0.02
//#define GYR_W ((double)4.0e-5)

//euroc
#define GRAVITY ((double)9.81007)
#define ACC_N ((double)0.2)  //0.025
#define ACC_W ((double)0.0002)//0.02
#define GYR_N ((double)0.02)  //0.0025
#define GYR_W ((double)2.0e-5)//8e-04

//xiaomi
//#define GRAVITY ((double)9.805)
//#define ACC_N ((double)0.02)  //0.025
//#define ACC_W ((double)0.025)//0.02
//#define GYR_N ((double)8e-04)  //0.0025
//#define GYR_W ((double)0.0025)//8e-04


//kitti0930
//#define GRAVITY ((double)9.81007)
//#define ACC_N ((double)0.1)  //0.025
//#define ACC_W ((double)0.001)//0.02
//#define GYR_N ((double)0.01)  //0.0025
//#define GYR_W ((double)1.0e-4)//8e-04


//kitti1003
//#define GRAVITY ((double)9.81007)
//#define ACC_N ((double)0.08)  //0.025
//#define ACC_W ((double)0.00004)//0.02
//#define GYR_N ((double)0.004)  //0.0025
//#define GYR_W ((double)2.0e-6)//8e-04


//day1
//#define GRAVITY ((double)9.81007)
//#define ACC_N ((double)0.2)  //0.025
//#define ACC_W ((double)0.0002)//0.02
//#define GYR_N ((double)0.02)  //0.0025
//#define GYR_W ((double)2.0e-5)//8e-04


//#define BIAS_ACC_THRESHOLD ((double)0.5)
//euroc
#define BIAS_ACC_THRESHOLD ((double)0.1)
#define BIAS_GYR_THRESHOLD ((double)0.1)
#define G_THRESHOLD ((double)3.0)
//#define G_NORM ((double)9.805)
//euroc kitti
#define G_NORM ((double)9.81007)
#define INIT_KF_THRESHOLD ((double)18)
#define SFM_R_THRESHOLD ((double)180)
#define MAX_FEATURE_CNT 150

extern double FOCUS_LENGTH_Y;
extern double PY;
extern double FOCUS_LENGTH_X;
extern double PX;
extern double SOLVER_TIME;
extern int FREQ;

//extrinsic param
extern double TIC_X;
extern double TIC_Y;
extern double TIC_Z;


extern bool isUndistorted;//不需要畸变则为false
extern double k1_global;
extern double k2_global;
extern double p1_global;
extern double p2_global;
extern double FOCAL_LENGTH;
/* IMU
 Z^
 |   /Y
 |  /
 | /
 |/--------->X
 
 */
enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

bool setGlobalParam(DeviceType device);

#if 1
#define TS(name) int64 t_##name = cv::getTickCount()
#define TE(name) printf("TIMER_" #name ": %.2fms\n", \
1000.*((cv::getTickCount() - t_##name) / cv::getTickFrequency()))
#else
#define TS(name)
#define TE(name)
#endif

#endif /* global_param_h */

