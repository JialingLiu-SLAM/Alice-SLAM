//
//  global_param.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2017/05/09.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include <stdio.h>
#include "global_param.hpp"

//double FOCUS_LENGTH_Y;
//double PY;
//double FOCUS_LENGTH_X;
//double PX;
double SOLVER_TIME;
int FREQ;

//extrinsic param
double TIC_X;
double TIC_Y;
double TIC_Z;



int mainClientID;
void setMainClientID(){
    mainClientID=0;
}
bool setGlobalParam(DeviceType device)
{
    
    switch (device) {
        case iPhone7P:
            printf("Device iPhone7 plus param\n");
//            FOCUS_LENGTH_X = 526.600;
//            FOCUS_LENGTH_Y = 526.678;
//            PX = 243.481;
//            PY = 315.280;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.01;
            
            
            return true;
            break;
            
        case iPhone7:
            printf("Device iPhone7 param\n");
//            FOCUS_LENGTH_X = 526.958;
//            FOCUS_LENGTH_Y = 527.179;
//            PX = 244.473;
//            PY = 313.844;
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.01;
            
            
            return true;
            break;
            
        case iPhone6s:
            printf("Device iPhone6s param\n");
//            FOCUS_LENGTH_Y = 549.477;
//            PY = 320.379;
//            FOCUS_LENGTH_X = 548.813;
//            PX = 238.520;
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.065;
            TIC_Z = 0.0;
             
            return true;
            break;
            
        case iPhone6sP:
            printf("Device iPhone6sP param\n");
//            FOCUS_LENGTH_X = 547.565;
//            FOCUS_LENGTH_Y = 547.998;
//            PX = 239.033;
//            PY = 309.452;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.065;
            TIC_Z = 0.0;
             
            return true;
            break;
            
        case iPadPro97:
            printf("Device ipad97 param\n");
//            FOCUS_LENGTH_X = 547.234;
//            FOCUS_LENGTH_Y = 547.464;
//            PX = 241.549;
//            PY = 317.957;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.1;
             
            return true;
            break;
            
        case iPadPro129:
            printf("Device iPad129 param\n");
//            FOCUS_LENGTH_X = 547.234;
//            FOCUS_LENGTH_Y = 547.464;
//            PX = 241.549;
//            PY = 317.957;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.1;
             
            return true;
            break;
            
        case euroc:        
            printf("Device euroc\n");
           SOLVER_TIME = 0.06;//0.06
           FREQ = 3;//每张图片都处理一次

           //extrinsic param
           TIC_X = -0.0216401454975;
           TIC_Y = -0.064676986768;
           TIC_Z = 0.00981073058949;

           
           return true;
           break;
        case xiaoMi:
            printf("Device xiaoMi param\n");
            

            SOLVER_TIME = 0.06;//0.06
            FREQ = 3;//每张图片都处理一次

            //extrinsic param
            TIC_X = -0.00165;
            TIC_Y = -0.009950000000000001;
            TIC_Z = 0.00067;
            
            return true;
            break;
        case day1:
            printf("Device day1 param\n");
           
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





