//
//  vins_pnp.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "vins_pnp.hpp"

vinsPnP::vinsPnP()
{
    printf("init VINS_pnp begins\n");
//    clearState()//未完 未实现
}

void vinsPnP::setIMUModel(double FOCUS_LENGTH_X_server)
{
    PerspectiveFactor::sqrt_info = FOCUS_LENGTH_X_server / 1.5 * Matrix2d::Identity();
}


//这个应该是用不上了 所以并没有设置了
void vinsPnP::setExtrinsic()
{
    tic << TIC_X,
    TIC_Y,
    TIC_Z;
    ric = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r));
    printf("pnp set extrinsic %lf %lf %lf\n", tic.x(), tic.y(), tic.z());
}

void vinsPnP::setInit(VINS_RESULT vins_result)
{
    for (int i = 0; i <= PNP_SIZE; i++)
    {
        Bas[i] = vins_result.Ba;
        Bgs[i] = vins_result.Bg;
        if(Headers[i] == vins_result.header)
        {
            find_solved[i] = true;
            find_solved_vins[i].P = vins_result.P;
            find_solved_vins[i].R = vins_result.R;
            find_solved_vins[i].V = vins_result.V;
            Ps[i] = vins_result.P;
            Rs[i] = vins_result.R;
            Vs[i] = vins_result.V;
            printf("pnp find new index %d\n", i);
        }
    }
    //printf("pnp vins index %d header_vins %lf, header_pnp %lf \n", solved_index, vins_result.header, Headers[solved_index]);
    //printf("pnp init value %lf %lf %lf\n",Ps[solved_index].x(), Ps[solved_index].y(), Ps[solved_index].z());
}
void vinsPnP::updateFeatures(vector<IMG_MSG_LOCAL> &feature_msg)
{
}

void vinsPnP::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
}

void vinsPnP::processImage(vector<IMG_MSG_LOCAL> &feature_msg, double header, bool use_pnp)
{
    int track_num;
    printf("pnp %d adding feature points %lu\n", frame_count, feature_msg.size());
    //add feature
    features[frame_count] = feature_msg;
    Headers[frame_count] = header;
    updateFeatures(feature_msg);
    
    if(frame_count < PNP_SIZE)
    {
        frame_count++;
        return;
    }
    if(use_pnp)
        solve_ceres();
    
    slideWindow();
}


void vinsPnP::solve_ceres()
{
}

void vinsPnP::slideWindow()
{
    
}
