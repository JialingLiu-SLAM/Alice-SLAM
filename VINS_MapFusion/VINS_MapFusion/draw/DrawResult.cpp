//
//  DrawResult.cpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/6/2.
//  Copyright © 2020 zx. All rights reserved.
//
#include "DrawResult.hpp"
bool running_flag = true;//变为false没写
bool view_done = false;//主线程结束等到view结束 这个还没写
//Eigen::Vector3d relocalize_t{Eigen::Vector3d(0, 0, 0)};//还没写
//Eigen::Matrix3d relocalize_r{Eigen::Matrix3d::Identity()};

DrawResult::DrawResult(){
    
}
void DrawResult::visualization()
{
    pangolin::CreateWindowAndBind("VINS: Map Visualization",1024,768); //create a display window
    glEnable(GL_DEPTH_TEST); //launch depth test
    glEnable(GL_BLEND);      //use blend function
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA); //set blend alpha value
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175)); //new button and menu
     pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
     pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
     pangolin::Var<bool> menuShowPath("menu.Show Path", true, true);
       // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                    pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                    pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,VISUALLOOKATX, VISUALLOOKATY, VISUALLOOKATZ)
                    );

// Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    while(mpPoseGraph==nullptr){
        usleep(50);
    }
    while(!pangolin::ShouldQuit() & running_flag)
    {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        ViewCameraPose(Twc);
        if(menuFollowCamera)
           s_cam.Follow(Twc);
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f); //背景色设置为白色
        DrawCurrentCamera(Twc);
//        if(menuShowPoints)
//            mpPoseGraph->viewPointClouds();
        if(menuShowPath)
            mpPoseGraph->viewPath_3();//只考虑了平移 没考虑旋转
        pangolin::FinishFrame();

    }
    cout << "pangolin thread end" << endl;
    view_done = true;
    //exit(1);
}

void DrawResult::realDrawResult(){
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//    ViewCameraPose(relocalize_t, relocalize_r, Twc);
//    if(menuFollowCamera)
//       s_cam.Follow(Twc);
//    d_cam.Activate(s_cam);
//    glClearColor(0.0f,0.0f,0.0f,0.0f); //背景色设置为白色
//    DrawCurrentCamera(Twc);
//    if(menuShowPoints)
//        mpPoseGraph->viewPointClouds();
//    if(menuShowPath)
//        mpPoseGraph->viewPath();//只考虑了平移 没考虑旋转
//    pangolin::FinishFrame();
}

void DrawResult::ViewCameraPose( pangolin::OpenGlMatrix &M)
{
//    int idx2 = WINDOW_SIZE - 1;
    if (mpPoseGraph->cur_seg_index != -1)//poseGraph 当前序列有值 但是如果第2个地图来了呢？
    {
//        int i = idx2;
//        Vector3d P = (loop_correct_r * mpVins->Ps[i] + loop_correct_t) + (loop_correct_r * mpVins->Rs[i]) * mpVins->tic;
//        //Quaterniond R = Quaterniond((loop_correct_r * estimator.Rs[i]) * estimator.ric[0]);
//        Eigen::Matrix3d R = (loop_correct_r * mpVins->Rs[i]) * mpVins->ric;

        Vector3d P =mpPoseGraph->curKF_P;
        Eigen::Matrix3d R =mpPoseGraph->curKF_R;
        
        M.m[0] = R(0,0);
        M.m[1] = R(1,0);
        M.m[2] = R(2,0);
        M.m[3] = 0.0;

        M.m[4] = R(0,1);
        M.m[5] = R(1,1);
        M.m[6] = R(2,1);
        M.m[7] = 0.0;

        M.m[8] = R(0,2);
        M.m[9] = R(1,2);
        M.m[10] = R(2,2);
        M.m[11] = 0.0;
        
        
        M.m[12] = P.x();
        M.m[13] = P.y();
        M.m[14] = P.z();
        M.m[15] = 1.0;
    //    cout << "M.m[0]:" <<M.m[0] << "M.m[1]:" << M.m[1] << "M.m[2]" << M.m[2] << endl;
    //    cout << "M.m[4]:" <<M.m[4] << "M.m[5]:" << M.m[5] << "M.m[6]" << M.m[6] << endl;
    //    cout << "M.m[8]:" <<M.m[8] << "M.m[9]:" << M.m[9] << "M.m[10]" << M.m[10] << endl;
      //  cout << "M.m[12]:" <<M.m[12] << "M.m[13]:" << M.m[13] << "M.m[14]" << M.m[14] << endl;
    }
    else
         M.SetIdentity();
}

void DrawResult::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = 0.08f; //mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(2);  //set line width
    glColor3f(0.0f,0.0f,1.0f);   //blue
    glBegin(GL_LINES);           //draw camera
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);
    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);
    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);
    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();
    glPopMatrix();
}
