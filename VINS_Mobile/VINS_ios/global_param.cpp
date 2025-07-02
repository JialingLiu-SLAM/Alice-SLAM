//
//  global_param.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2017/05/09.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include <stdio.h>
#include "global_param.hpp"

double FOCUS_LENGTH_Y;
double PY;
double FOCUS_LENGTH_X;//相机内参 焦距
double PX;
double SOLVER_TIME;
int FREQ;

//extrinsic param
double TIC_X;
double TIC_Y;
double TIC_Z;

bool isUndistorted;//不需要畸变则为false
double k1_global;
double k2_global;
double p1_global;
double p2_global;
double FOCAL_LENGTH;

bool setGlobalParam(DeviceType device)
{
    switch (device) {
        case iPhone8:
            printf("Device iPhone8 param\n");
            FOCUS_LENGTH_X = 530.233;
            FOCUS_LENGTH_Y = 531.082;
            PX = 250.286;
            PY = 316.520  ;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.01;
            
            isUndistorted=false;
            return true;
            break;
            
        case iPhone7P:
            printf("Device iPhone7 plus param\n");
            FOCUS_LENGTH_X = 526.600;
            FOCUS_LENGTH_Y = 526.678;
            PX = 243.481;
            PY = 315.280  ;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.01;
            
            isUndistorted=false;
            return true;
            break;
            
        case iPhone7:
            printf("Device iPhone7 param\n");
            FOCUS_LENGTH_X = 526.958;
            FOCUS_LENGTH_Y = 527.179;
            PX = 244.473;
            PY = 313.844;
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.01;
            
            isUndistorted=false;
            return true;
            break;
            
        case iPhone6s:
            printf("Device iPhone6s param\n");
            FOCUS_LENGTH_Y = 549.477;
            PY = 320.379;
            FOCUS_LENGTH_X = 548.813;
            PX = 238.520;
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.065;
            TIC_Z = 0.0;
            
            isUndistorted=false;
            return true;
            break;
            
        case iPhone6sP:
            printf("Device iPhone6sP param\n");
            FOCUS_LENGTH_X = 547.565;
            FOCUS_LENGTH_Y = 547.998;
            PX = 239.033;
            PY = 309.452;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.065;
            TIC_Z = 0.0;
            
            isUndistorted=false;
            return true;
            break;
            
        case iPadPro97:
            printf("Device ipad97 param\n");
            FOCUS_LENGTH_X = 547.234;
            FOCUS_LENGTH_Y = 547.464;
            PX = 241.549;
            PY = 317.957;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.1;
            
            isUndistorted=false;
            return true;
            break;
            
        case iPadPro129:
            printf("Device iPad129 param\n");
            FOCUS_LENGTH_X = 547.234;
            FOCUS_LENGTH_Y = 547.464;
            PX = 241.549;
            PY = 317.957;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.1;
            
            isUndistorted=false;
            return true;
            break;
            
        case euroc:
            printf("Device euroc param\n");
            FOCUS_LENGTH_X =458.654  ;//  458.654
            FOCUS_LENGTH_Y =457.296   ;//457.296
            PX =367.215  ;//367.215
            PY =248.375   ;// 248.375

            SOLVER_TIME = 0.06;//0.06
            FREQ = 2;//每张图片都处理一次

            //extrinsic param
            TIC_X = -0.0216401454975;
            TIC_Y = -0.064676986768;
            TIC_Z = 0.00981073058949;
            
            isUndistorted=true;
            k1_global=-0.28340811;
            k2_global=0.07395907;
            p1_global=0.00019359;
            p2_global=1.76187114e-05;
            FOCAL_LENGTH=460;
            return true;
            break;
        case xiaoMi:
           printf("Device xiaoMi param\n");
           FOCUS_LENGTH_X = 493.0167;
           FOCUS_LENGTH_Y =   491.55953;
           PX = 317.97856;
           PY =   242.392;

           SOLVER_TIME = 0.06;//0.06
           FREQ = 3;//每张图片都处理一次

           //extrinsic param
           TIC_X = -0.00165;
           TIC_Y = -0.009950000000000001;
           TIC_Z = 0.00067;
           isUndistorted=false;
           return true;
           break;
        case kitti0930:
            printf("Device kitti0930 param\n");
            FOCUS_LENGTH_X = 9.786977e+02;
            FOCUS_LENGTH_Y =   9.717435e+02;
            PX = 690;
            PY =   2.497222e+02;

            SOLVER_TIME = 0.08;//0.06
            FREQ = 1;//每张图片都处理一次

            //extrinsic param
            TIC_X = 1.1439;
            TIC_Y = -0.312718;
            TIC_Z = 0.726546;
            
            isUndistorted=true;
            k1_global=-3.792567e-01;
            k2_global=2.121203e-01;
            p1_global=9.182571e-04;
            p2_global=1.911304e-03;
            FOCAL_LENGTH=980;
            return true;
            break;
        case kitti1003:
            printf("Device kitti1003 param\n");
            FOCUS_LENGTH_X = 9.799200e+02;
            FOCUS_LENGTH_Y =   9.741183e+02;
            PX = 690;
            PY =   2.486443e+02;

            SOLVER_TIME = 0.08;//0.06
            FREQ = 1;//每张图片都处理一次

            //extrinsic param
            TIC_X = 1.10224;
            TIC_Y = -0.319072;
            TIC_Z = 0.746066;
            
            isUndistorted=true;
            k1_global=-3.745594e-01;
            k2_global=2.049385e-01;
            p1_global=1.110145e-03;
            p2_global=1.379375e-03;
            FOCAL_LENGTH=980;
            return true;
            break;
        case day1:
            printf("Device day1 param\n");
            FOCUS_LENGTH_X = 465.6647968958394;
            FOCUS_LENGTH_Y =   465.53104946638166;
            PX = 373.1765292465323;
            PY =   232.29333312263688;

            SOLVER_TIME = 0.06;//0.06
            FREQ = 1;//每张图片都处理一次

            //extrinsic param
//            TIC_X = -0.03921656415229387;
//            TIC_Y = 0.00621263233002485;
//            TIC_Z = 0.0012210059575531885;
            
            //day1
//            TIC_X = 3.1752591876664016e-02;
//            TIC_Y = 2.77070343336362E-02;
//            TIC_Z = 2.6493495655480398e-02;
            
            //day2
            TIC_X = 2.3063094977714534e-02;
            TIC_Y = -1.16321964866048E-02;
            TIC_Z = 6.1072371687497011e-02;
            
            isUndistorted=true;
            k1_global=-2.886519243545528e-01;
            k2_global=0.08251010502519662;
            p1_global=-0.00037458410613943715;
            p2_global=-9.908178225000679e-05;
            FOCAL_LENGTH=466;
            return true;
            break;
        case night1:
            printf("Device night1 param\n");
            FOCUS_LENGTH_X = 466.5414073540218;
            FOCUS_LENGTH_Y =   466.0598963551723;
            PX = 375.27989475026646;
            PY =   235.59142821158878;

            SOLVER_TIME = 0.06;//0.06
            FREQ = 1;//每张图片都处理一次
 
            
            //night1
//            TIC_X = -4.5841197890917076e-02;
//            TIC_Y = 3.10953557480814E-02;
//            TIC_Z = -1.2109117125450619e-01;
            
            //night3
            TIC_X = 3.2782132665616387e-02;
            TIC_Y = 3.25832900257785E-02;
            TIC_Z = -8.4353328830711560e-02;
            
            isUndistorted=true;
            k1_global=-0.29644060428385316;
            k2_global=0.0916048260485308;
            p1_global=-0.0006718845948903008;
            p2_global=0.00028681677399650096;
            FOCAL_LENGTH=466;
            return true;
            break;
        case unDefine:
            return false;
            break;
        default:
            return false;
            break;
    }
}





