//
//  Client.cpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/25.
//  Copyright © 2020 zx. All rights reserved.
//

#include "Client.hpp"
#include "Server.hpp"



Client::Client() {
  this->name = (char *) malloc(MAX_NAME_LENGHT+1);
    this->Twclient=cv::Mat::eye(4, 4, CV_64F);//设成单位矩阵
    
    is_add=false;
    id=-1;//给个默认值
    
    status=true;
    
    
    buffer_kindsAndLen_mutex=new std::mutex();
}



void Client::setCam_intrinsic(DeviceType device){
    
    switch (device) {
        case iPhone7P:
            printf("Device iPhone7 plus param client\n");
            FOCUS_LENGTH_X_server = 526.600;
            FOCUS_LENGTH_Y_server = 526.678;
            PX_server = 243.481;
            PY_server = 315.280;
            
            RIC_y_server=0.0;
            RIC_p_server=0.0;
            RIC_r_server=180.0;
            
            TIC_X_server = 0.0;
            TIC_Y_server = 0.092;
            TIC_Z_server = 0.01;
            
            setWidth_Height(480, 640);
            
            isUndistorted=false;
            break;
            
        case iPhone7:
            printf("Device iPhone7 param client\n");
            FOCUS_LENGTH_X_server = 526.958;
            FOCUS_LENGTH_Y_server = 527.179;
            PX_server = 244.473;
            PY_server = 313.844;
            
            RIC_y_server=0.0;
            RIC_p_server=0.0;
            RIC_r_server=180.0;
            
            TIC_X_server = 0.0;
            TIC_Y_server = 0.092;
            TIC_Z_server = 0.01;
            
            setWidth_Height(480, 640);
            
            isUndistorted=false;
            break;
            
        case iPhone6s:
            printf("Device iPhone6s param client\n");
            FOCUS_LENGTH_Y_server = 549.477;
            PY_server = 320.379;
            FOCUS_LENGTH_X_server = 548.813;
            PX_server = 238.520;
            
            RIC_y_server=0.0;
            RIC_p_server=0.0;
            RIC_r_server=180.0;
            
            TIC_X_server = 0.0;
            TIC_Y_server = 0.065;
            TIC_Z_server = 0.0;
            
            setWidth_Height(480, 640);
            
            isUndistorted=false;
            break;
            
        case iPhone6sP:
            printf("Device iPhone6sP param client\n");
            FOCUS_LENGTH_X_server = 547.565;
            FOCUS_LENGTH_Y_server = 547.998;
            PX_server = 239.033;
            PY_server = 309.452;
            
            RIC_y_server=0.0;
            RIC_p_server=0.0;
            RIC_r_server=180.0;
            
            TIC_X_server = 0.0;
            TIC_Y_server = 0.065;
            TIC_Z_server = 0.0;
            
            setWidth_Height(480, 640);
            
            isUndistorted=false;
            break;
            
        case iPadPro97:
            printf("Device ipad97 param client\n");
            FOCUS_LENGTH_X_server = 547.234;
            FOCUS_LENGTH_Y_server = 547.464;
            PX_server = 241.549;
            PY_server = 317.957;
            
            RIC_y_server=0.0;
            RIC_p_server=0.0;
            RIC_r_server=180.0;
            
            TIC_X_server = 0.0;
            TIC_Y_server = 0.092;
            TIC_Z_server = 0.1;
            
            setWidth_Height(480, 640);
            
            isUndistorted=false;
            break;
            
        case iPadPro129:
            printf("Device iPad129 param client\n");
            FOCUS_LENGTH_X_server = 547.234;
            FOCUS_LENGTH_Y_server = 547.464;
            PX_server = 241.549;
            PY_server = 317.957;
            
            RIC_y_server=0.0;
            RIC_p_server=0.0;
            RIC_r_server=180.0;
            
            TIC_X_server = 0.0;
            TIC_Y_server = 0.092;
            TIC_Z_server = 0.1;
            
            setWidth_Height(480, 640);
            
            isUndistorted=false;
            break;
            
        case euroc:
           printf("Device euroc param client\n");
           FOCUS_LENGTH_X_server = 458.654;
           FOCUS_LENGTH_Y_server =   457.296;
           PX_server = 367.215;
           PY_server =   248.375;
            
           RIC_y_server=89.148;
           RIC_p_server=1.47693;
           RIC_r_server=0.215286;
            
           TIC_X_server = -0.0216401454975;
           TIC_Y_server = -0.064676986768;
           TIC_Z_server = 0.00981073058949;
            
           setWidth_Height(752, 480);
            
           isUndistorted=true;
           k1_global=-0.28340811;
           k2_global=0.07395907;
           p1_global=0.00019359;
           p2_global=1.76187114e-05;
           FOCAL_LENGTH=460;
           break;
            
       case xiaoMi:
          printf("Device xiaoMi param client\n");
          FOCUS_LENGTH_X_server = 493.0167;
          FOCUS_LENGTH_Y_server =   491.55953;
          PX_server = 317.97856;
          PY_server = 242.392;
            
          RIC_y_server=-90.0;
          RIC_p_server=0.0;
          RIC_r_server=180.0;
          
          TIC_X_server = -0.00165;
          TIC_Y_server = -0.009950000000000001;
          TIC_Z_server = 0.00067;
            
          setWidth_Height(640, 480);
            
          isUndistorted=false;
          break;
        case unDefine:
            break;
        default:
            break;
    }
    tic_client << TIC_X_server,
    TIC_Y_server,
    TIC_Z_server;
    ric_client = Utility::ypr2R(Vector3d(RIC_y_server,RIC_p_server,RIC_r_server));
    
//    readIntrinsicParameter();//在客户端那边做了
}

void Client::SetName(const char *name) {
  //Copies at most MAX_NAME_LENGHT + 1 (including '\0')
  snprintf(this->name, MAX_NAME_LENGHT+1, name);
}

void Client::SetId(int id) {
  this->id = id;
}

int Client::getId(){
    return id;
}


void Client::setWidth_Height(int width,int height){
    m_imageWidth=width;
    m_imageHeight=height;
}
int Client::getWidth(){
    return m_imageWidth;
}
int Client::getHeight(){
    return m_imageHeight;
}



void Client::undistortedPoints(vector<cv::KeyPoint> &keypoints)
{
   
    
    for (unsigned int i = 0,len=keypoints.size(); i < len; i++)
    {
        cv::Point2f point_single=keypoints[i].pt;
        Eigen::Vector2d a(point_single.x, point_single.y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        point_single=cv::Point2f(b.x() / b.z(), b.y() / b.z());
        keypoints[i].pt=point_single;
        
    }
      
}

void Client::undistortedPoints_2f(vector<Eigen::Vector2f> &vpoints2f)
{
   
    
    for (unsigned int i = 0,len=vpoints2f.size(); i < len; i++)
    {
        Eigen::Vector2f point_single=vpoints2f[i];
        Eigen::Vector2d a(point_single[0], point_single[1]);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        point_single=Eigen::Vector2f(b[0] / b[2], b[1] / b[2]);
        vpoints2f[i]=point_single;
        
    }
      
}

void Client::readIntrinsicParameter()
{
    
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(m_imageWidth,m_imageHeight,k1_global,k2_global,p1_global,p2_global,FOCUS_LENGTH_X_server,FOCUS_LENGTH_Y_server,PX_server,PY_server);
}

//Client* Client::getClientById(int id){
//    for(int i=0;i<Server::clients.size();i++){
//        if(Server::clients[i].getId()==id){
//            return &Server::clients[i];
//        }
//    }
//    return nullptr;
//}
