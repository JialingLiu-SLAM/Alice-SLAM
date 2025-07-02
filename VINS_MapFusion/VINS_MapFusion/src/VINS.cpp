//
//  VINS.cpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/27.
//  Copyright © 2020 zx. All rights reserved.
//

#include "VINS.hpp"

VINS::VINS():f_manager{Rs},fail_times{0},failure_hand{false}
{
//    printf("init VINS begins\n");
//    t_drift.setZero();
//    r_drift.setIdentity();
//    clearState();
//    failure_occur = 0;
//    last_P.setZero();
//    last_R.setIdentity();
//    last_P_old.setZero();
//    last_R_old.setIdentity();
    
    
    isSendLoopData=false;
    isSendLoop_another=false;
    isSendLoop_another2=false;
}

void VINS::setExtrinsic()
{
    tic << TIC_X,
    TIC_Y,
    TIC_Z;
    ric = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r));
}

void VINS::setIMUModel(double FOCUS_LENGTH_X_server)
{
    ProjectionFactor::sqrt_info = FOCUS_LENGTH_X_server / 1.5 * Matrix2d::Identity();
}
