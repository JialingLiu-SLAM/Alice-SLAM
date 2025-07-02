//
//  HandleData.cpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/27.
//  Copyright © 2020 zx. All rights reserved.
//

#include "HandleData.hpp"
#include "global_param.hpp"

static unsigned long packIdx=1;//可以用来做后续的未接收到的数据的重发
unsigned char saveEnd[5]={0xeb, 0x95, 0x96, '\r', '\n'};

DeviceType receiveCameraParam(const char* buffer){
    unsigned char saveHeader[2] = {0xeb, 0x99};
    unsigned char hd[2];
    long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error device, shouldn't"<<endl;
    
    DeviceType deviceType;
    memcpy(&deviceType, buffer+offset, sizeof(DeviceType));
    offset += sizeof(DeviceType);
    
    return deviceType;
    
//    setGlobalParam(deviceType);
}

Vector3d receiveInit_aligmentParam(const char* buffer){
    unsigned char saveHeader[2] =  {0xea, 0x98};
    unsigned char hd[2];
    long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error g_imu, shouldn't"<<endl;
    
    Vector3d g;
    for(int i=0;i<3;i++){
        memcpy(&g[i], buffer+offset, sizeof(double));
        offset += sizeof(double);
    }
    
    return g;
    
}

KeyFrame* receiveKF(const char* buffer){
    KeyFrame* curKf=new KeyFrame();
    unsigned char saveHeader[2] ={0xe9, 0x97};
    unsigned char hd[2];
    long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error KF_T, shouldn't"<<endl;
    
    //先接收窗口特征点的数量
    int winPointNum;
    memcpy(&winPointNum, buffer+offset, sizeof(int));
    offset += sizeof(int);
//    cout<<"winPointNum size="<<winPointNum<<endl;
    
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error KF_T, shouldn't"<<endl;

 
//    TS(all_keyPoint);
    //全部的特征点
    int pointNum;
    memcpy(&pointNum, buffer+offset, sizeof(int));
    offset += sizeof(int);
//    cout<<"pointNum size="<<pointNum<<endl;
    
    std::vector<cv::KeyPoint> keypoints_test;
    keypoints_test.clear();
    for(int i=0;i<pointNum;i++){
        float angle,x,y,response;
        int octave,class_id;
        
        memcpy(&angle, buffer+offset, sizeof(float));
        offset += sizeof(float);
        memcpy(&octave, buffer+offset, sizeof(int));
        offset += sizeof(int);
        memcpy(&x, buffer+offset, sizeof(float));
        offset += sizeof(float);
        memcpy(&y, buffer+offset, sizeof(float));
        offset += sizeof(float);
        memcpy(&response, buffer+offset, sizeof(float));
        offset += sizeof(float);
        memcpy(&class_id, buffer+offset, sizeof(int));
        offset += sizeof(int);
        
        cv::Point2f p_test;
        p_test.x=x;
        p_test.y=y;
        cv::KeyPoint point_test;
        point_test.pt=p_test;
        point_test.angle=angle;
        point_test.octave=octave;
        point_test.response=response;
        point_test.class_id=class_id;
        keypoints_test.push_back(point_test);
//        cout<<"keypoints x="<<point_test.pt.x<<" y="<<point_test.pt.y<<endl;
    }
    curKf->keypoints=keypoints_test;
//    TE(all_keyPoint);
        
        TS(all_des);
    //全部描述符
    std::vector<BRIEF::bitset> descriptors_test;
    descriptors_test.clear();
    
    for(int i=0;i<pointNum;i++){
        int len;
        memcpy(&len, buffer+offset, sizeof(int));
        offset += sizeof(int);
        
        BRIEF::bitset des_test;
        des_test.clear();
        
        string str_test="";
        for(size_t j=0;j<len;j++){
            char str_test_single;
            memcpy(&str_test_single, buffer+offset, sizeof(char));
            offset += sizeof(char);
            str_test+=str_test_single;
        }
        
        for(auto ite=str_test.crbegin(),ite_end=str_test.crend();ite!=ite_end;ite++){
            if((*ite)=='1'){
                des_test.push_back(1);
            }else{
                des_test.push_back(0);
            }
        }
//        cout<<"des_single ="<<des_test<<endl;
        descriptors_test.push_back(des_test);
    }
    curKf->descriptors=descriptors_test;
    TE(all_des);

    

//    TS(measurement_pointCloud);
    double winPointZ;
    int start=pointNum-winPointNum;
    Eigen::Vector3d point_clouds_test;
    for(int i=0;i<winPointNum;i++){
//        memcpy(&winPointZ, buffer+offset, sizeof(double));
//        offset += sizeof(double);
//        curKf->win_keyPoint_depth.push_back(winPointZ);
        
        cv::KeyPoint curKeyPoint=keypoints_test[start+i];
//        Eigen::Vector3d win_point;
//        win_point<<curKeyPoint.pt.x *winPointZ, curKeyPoint.pt.y *winPointZ,winPointZ;
//        curKf->point_clouds.push_back(win_point);
        

        curKf->measurements.push_back(curKeyPoint.pt);
        

//        cout<<"imgKeyFrame->point_clouds single xyz=";
        for(int i=0;i<3;i++){
            memcpy(&winPointZ, buffer+offset, sizeof(double));
            offset += sizeof(double);
            point_clouds_test(i)=winPointZ;
//            cout<<winPointZ<<" "<<point_clouds_test(i)<<" ";
        }
        curKf->point_clouds.push_back(point_clouds_test);

//        cout<<winPointZ;
    }

    
//    for(int i=0;i<curKf->measurements.size();i++){
//        cout<<"measurements"<<curKf->measurements[i].x  <<" "<<curKf->measurements[i].y<<endl;
//    }
    
    
    curKf->point_clouds_origin=curKf->point_clouds;
    curKf->measurements_origin=curKf->measurements;
//            TE(measurement_pointCloud);
//    cout<<endl;
    
//    for(int i=0;i<curKf->point_clouds_origin.size();i++){
//        cout<<"point_clouds_origin"<<curKf->point_clouds_origin[i](0) <<" "<<curKf->point_clouds_origin[i](1)<<" "<<curKf->point_clouds_origin[i](2)<<endl;
//    }
//
//    for(int i=0;i<curKf->measurements_origin.size();i++){
//        cout<<"measurements_origin"<<curKf->measurements_origin[i].x  <<" "<<curKf->measurements_origin[i].y<<endl;
//    }
    
    
//    TS(win_point);
    //窗口特征点、描述符赋值
    curKf->setWin_keyPoint_des();
    //窗口点的深度、把3D坐标算出来
    
    curKf->is_des_end=true;
//    TE(win_point);
    
    memcpy(hd, buffer+offset, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error KF_T pose, shouldn't"<<endl;

//            TS(POSE);
    //位姿
    Eigen::Vector3d origin_T_w_i;
    Eigen::Matrix3d origin_R_w_i;
//    cout<<"pose T: ";
    for(int i=0;i<3;i++){
        memcpy(&origin_T_w_i[i], buffer+offset, sizeof(double));
        offset += sizeof(double);
//        cout<<origin_T_w_i[i]<<" ";
    }
//    cout<<endl;
//
//    cout<<"pose R: ";
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            memcpy(&origin_R_w_i(i,j), buffer+offset, sizeof(double));
            offset += sizeof(double);
//            cout<<origin_R_w_i(i,j)<<" ";
        }
    }
//    cout<<endl;
    curKf->updatePose(origin_T_w_i, origin_R_w_i);
    curKf->relocalize_t=origin_T_w_i;
    curKf->relocalize_r=origin_R_w_i;
    
//        cout<<"origin pose T: ";
    for(int i=0;i<3;i++){
        memcpy(&origin_T_w_i[i], buffer+offset, sizeof(double));
        offset += sizeof(double);
//        cout<<origin_T_w_i[i]<<" ";
    }
//    cout<<endl;
    
    
//    cout<<"origin pose R: ";

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            memcpy(&origin_R_w_i(i,j), buffer+offset, sizeof(double));
            offset += sizeof(double);
//            cout<<origin_R_w_i(i,j)<<" ";
        }
    }
//    cout<<endl;
    curKf->updateOriginPose(origin_T_w_i, origin_R_w_i);
//    TE(POSE);
    
    //id 自己地图里的id
    int global_index_test;
    memcpy(&global_index_test, buffer+offset, sizeof(int));
    offset += sizeof(int);
    curKf->global_index=global_index_test;
//    cout<<"imgKeyFrame->global_index"<<curKf->global_index<<endl;

    //序列号
    int segment_index;
    memcpy(&segment_index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    curKf->segment_index=segment_index;
//    cout<<"segment_index "<<curKf->segment_index<<endl;
    
    double header;
    memcpy(&header, buffer+offset, sizeof(double));
    offset+=sizeof(double);
    curKf->header=header;
//    cout<<"header "<<curKf->header<<endl;
    
    
    memcpy(hd, buffer+offset, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error KF_T pose, shouldn't"<<endl;
    
//    TS(feature);
    //features_id
    int feature_len;
    vector<int> features_id;
    memcpy(&feature_len, buffer+offset, sizeof(int));
    offset += sizeof(int);
    
//    cout<<"feature_len="<<feature_len<<endl;
//    cout<<"features_id=";
    int feature_id_single;
    for(int i=0;i<feature_len;i++){
        memcpy(&feature_id_single, buffer+offset, sizeof(int));
        offset += sizeof(int);
        features_id.push_back(feature_id_single);
//        cout<<feature_id_single<<" ";
    }
//    cout<<endl;
    curKf->features_id=features_id;
    curKf->features_id_origin=features_id;
//    TE(feature);
    
    //表明还没有做过回环
    curKf->check_loop=false;
    
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error KF_T, shouldn't"<<endl;
    
    
    return curKf;
}
KeyFrame* receiveKF_1_1(const char* buffer){
    KeyFrame* curKf=new KeyFrame();
    unsigned char saveHeader[2] ={0xe9, 0x97};
    unsigned char hd[2];
    long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error KF_T, shouldn't"<<endl;
    
    //先接收窗口特征点的数量
    int winPointNum;
    memcpy(&winPointNum, buffer+offset, sizeof(int));
    offset += sizeof(int);
//    cout<<"winPointNum size="<<winPointNum<<endl;
    
    
    //全部的特征点的数量
    int pointNum;
    memcpy(&pointNum, buffer+offset, sizeof(int));
    offset += sizeof(int);
//    cout<<"pointNum size="<<pointNum<<endl;
    curKf->pointNum=pointNum;
    
//    TS(measurement_pointCloud);
    double winPointZ;
    
    Eigen::Vector3d point_clouds_test;
    for(int i=0;i<winPointNum;i++){

        for(int i=0;i<3;i++){
            memcpy(&winPointZ, buffer+offset, sizeof(double));
            offset += sizeof(double);
            point_clouds_test(i)=winPointZ;
//            cout<<winPointZ<<" "<<point_clouds_test(i)<<" ";
        }
        curKf->point_clouds.push_back(point_clouds_test);

//        cout<<winPointZ;
    }
    curKf->point_clouds_origin=curKf->point_clouds;
    
//    TE(measurement_pointCloud);
    
    cv::Point2f measurement_test;
    float measure_x;
    float measure_y;
    for(int i=0;i<winPointNum;i++){

           
        memcpy(&measure_x, buffer+offset, sizeof(float));
        offset += sizeof(float);
        memcpy(&measure_y, buffer+offset, sizeof(float));
        offset += sizeof(float);
        measurement_test.x=measure_x;
        measurement_test.y=measure_y;
    
        curKf->measurements.push_back(measurement_test);

    //        cout<<winPointZ;
        }
    curKf->measurements_origin=curKf->measurements;
    
    
    //发q_ic t_ic
//    Eigen::Matrix3d qic;
//    Eigen::Vector3d tic;
//    for(int i=0;i<3;i++){
//            memcpy(&tic[i], buffer+offset, sizeof(double));
//            offset += sizeof(double);
//    //        cout<<origin_T_w_i[i]<<" ";
//        }
//    //    cout<<endl;
//    //
//    //    cout<<"pose R: ";
//        for(int i=0;i<3;i++){
//            for(int j=0;j<3;j++){
//                memcpy(&qic(i,j), buffer+offset, sizeof(double));
//                offset += sizeof(double);
//    //            cout<<origin_R_w_i(i,j)<<" ";
//            }
//        }
//    curKf->qic=qic;
//    curKf->tic=tic;
    
    
    
//    TS(POSE);
    //位姿
    Eigen::Vector3d origin_T_w_i;
    Eigen::Matrix3d origin_R_w_i;
//    cout<<"pose T: ";
    for(int i=0;i<3;i++){
        memcpy(&origin_T_w_i[i], buffer+offset, sizeof(double));
        offset += sizeof(double);
//        cout<<origin_T_w_i[i]<<" ";
    }
//    cout<<endl;
//
//    cout<<"pose R: ";
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            memcpy(&origin_R_w_i(i,j), buffer+offset, sizeof(double));
            offset += sizeof(double);
//            cout<<origin_R_w_i(i,j)<<" ";
        }
    }
//    cout<<endl;
    curKf->updatePose(origin_T_w_i, origin_R_w_i);
    curKf->relocalize_t=origin_T_w_i;
    curKf->relocalize_r=origin_R_w_i;
    
//        cout<<"origin pose T: ";
    for(int i=0;i<3;i++){
        memcpy(&origin_T_w_i[i], buffer+offset, sizeof(double));
        offset += sizeof(double);
//        cout<<origin_T_w_i[i]<<" ";
    }
//    cout<<endl;
    
    
//    cout<<"origin pose R: ";

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            memcpy(&origin_R_w_i(i,j), buffer+offset, sizeof(double));
            offset += sizeof(double);
//            cout<<origin_R_w_i(i,j)<<" ";
        }
    }
//    cout<<endl;
    curKf->updateOriginPose(origin_T_w_i, origin_R_w_i);
//    TE(POSE);
//    滑动窗口中总约束
    int edge_test;
    memcpy(&edge_test, buffer+offset, sizeof(int));
    offset += sizeof(int);
    curKf->edge=edge_test;
    int edge_single_test;
    memcpy(&edge_single_test, buffer+offset, sizeof(int));
    offset += sizeof(int);
    curKf->edge_single=edge_single_test;
    double var_imu;
    memcpy(&var_imu, buffer+offset, sizeof(double));
    offset+=sizeof(double);
    curKf->var_imu=var_imu;
    
     //id 自己地图里的id
    int global_index_test;
    memcpy(&global_index_test, buffer+offset, sizeof(int));
    offset += sizeof(int);
    curKf->global_index=global_index_test;
//    cout<<"imgKeyFrame->global_index"<<curKf->global_index<<endl;

    //序列号
    int segment_index;
    memcpy(&segment_index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    curKf->segment_index=segment_index;
//    cout<<"segment_index "<<curKf->segment_index<<endl;
    
    double header;
    memcpy(&header, buffer+offset, sizeof(double));
    offset+=sizeof(double);
    curKf->header=header;
//    cout<<"header "<<curKf->header<<endl;
    
    
//     TS(feature);
    //features_id
    int feature_len;
    vector<int> features_id;
    memcpy(&feature_len, buffer+offset, sizeof(int));
    offset += sizeof(int);
    
//    cout<<"feature_len="<<feature_len<<endl;
//    cout<<"features_id=";
    int feature_id_single;
    for(int i=0;i<feature_len;i++){
        memcpy(&feature_id_single, buffer+offset, sizeof(int));
        offset += sizeof(int);
        features_id.push_back(feature_id_single);
//        cout<<feature_id_single<<" ";
    }
//    cout<<endl;
    curKf->features_id=features_id;
    curKf->features_id_origin=features_id;
//    TE(feature);
    
    
    //发共视的帧id,和权重
    int conn_kfs_len;
    memcpy(&conn_kfs_len, buffer+offset, sizeof(int));
    offset += sizeof(int);
    
    int conn_id_single,conn_weight_single;
    std::vector<int> vOrderedConnectedKeyFrames_global_index;
    std::vector<int> vOrderedWeights;
    if(conn_kfs_len>0){
        for(int i=0;i<conn_kfs_len;i++){
            memcpy(&conn_id_single, buffer+offset, sizeof(int));
            offset += sizeof(int);
            vOrderedConnectedKeyFrames_global_index.push_back(conn_id_single);
            memcpy(&conn_weight_single, buffer+offset, sizeof(int));
            offset += sizeof(int);
            vOrderedWeights.push_back(conn_weight_single);
        }
    }
    
    curKf->mMutexConnections.lock();
    curKf->mvOrderedConnectedKeyFrames_global_index=vOrderedConnectedKeyFrames_global_index;
    curKf->mvOrderedWeights=vOrderedWeights;
    curKf->mMutexConnections.unlock();
    
   
    
    
    
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error KF_T, shouldn't"<<endl;
    
    
    int receive_keys_num;
    memcpy(&receive_keys_num, buffer+offset, sizeof(int));
    offset += sizeof(int);
//    cout<<"receive_keys_num="<<receive_keys_num<<endl;
    
    if(receive_keys_num>0 && receive_keys_num<=pointNum){
    
        for(int i=0;i<receive_keys_num;i++){
//            float angle,x,y,response;
//            int octave,class_id;
            
            float x,y,response;
//            memcpy(&angle, buffer+offset, sizeof(float));
//            offset += sizeof(float);
//            memcpy(&octave, buffer+offset, sizeof(int));
//            offset += sizeof(int);
            memcpy(&x, buffer+offset, sizeof(float));
            offset += sizeof(float);
            memcpy(&y, buffer+offset, sizeof(float));
            offset += sizeof(float);
            memcpy(&response, buffer+offset, sizeof(float));
            offset += sizeof(float);
//            memcpy(&class_id, buffer+offset, sizeof(int));
//            offset += sizeof(int);
            
            cv::Point2f p_test;
            p_test.x=x;
            p_test.y=y;
            cv::KeyPoint point_test;
            point_test.pt=p_test;
//            point_test.angle=angle;
//            point_test.octave=octave;
            point_test.response=response;
//            point_test.class_id=class_id;
            
            point_test.angle=-1;
            point_test.octave=0;
            point_test.class_id=-1;
            curKf->keypoints.push_back(point_test);
        }
   
    }
  
    return curKf;
}

void receiveKF_1_2(const char* buffer,PoseGraph *poseGraph){
//     TS(all_keyPoint);
    unsigned char saveHeader[2] = {0xe0, 0x90};
    unsigned char hd[2];
    long unsigned int offset=0;
     memcpy(hd, buffer, 2);
     offset += 2;
     if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
         cerr<<"header error receiveKF_1_2, shouldn't"<<endl;
    
    int global_index_test;
    memcpy(&global_index_test, buffer+offset, sizeof(int));
    offset += sizeof(int);
//    cout<<"global_index_test="<<global_index_test<<endl;
    //找到该关键帧
    KeyFrame* curKf=poseGraph->getLastKeyframe_index(global_index_test);
    
    
    int keys_sub=curKf->pointNum-curKf->keypoints.size();
    int keys_num;
    bool is_end=false;
    if(keys_sub<=338){
        //说明是最后一躺
        is_end=true;
        keys_num=keys_sub;
        
        //验证一下
        memcpy(hd, buffer, 2);
        offset += 2;
        if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
            cerr<<"header error receiveKF_1_2, shouldn't"<<endl;
        
        int keys_sub_client;
        memcpy(&keys_sub_client, buffer+offset, sizeof(int));
        offset += sizeof(int);
        
//        cout<<"curKf->pointNum="<<curKf->pointNum<<endl;
//        cout<<"keys_sub_client="<<keys_sub_client<<endl;
//        cout<<"keys_num="<<keys_num<<endl;
        if(keys_sub_client!=keys_num){
            cout<<"error receiveKF_1_2 , should't  keys_sub_client!=keys_num"<<endl;
        }
    }else{
        keys_num=338;
    }
    
    

    

    for(int i=0;i<keys_num;i++){
//        float angle,x,y,response;
//        int octave,class_id;
        
        float x,y,response;
        
//        memcpy(&angle, buffer+offset, sizeof(float));
//        offset += sizeof(float);
//        memcpy(&octave, buffer+offset, sizeof(int));
//        offset += sizeof(int);
        memcpy(&x, buffer+offset, sizeof(float));
        offset += sizeof(float);
        memcpy(&y, buffer+offset, sizeof(float));
        offset += sizeof(float);
        memcpy(&response, buffer+offset, sizeof(float));
        offset += sizeof(float);
//        memcpy(&class_id, buffer+offset, sizeof(int));
//        offset += sizeof(int);
        
        cv::Point2f p_test;
        p_test.x=x;
        p_test.y=y;
        cv::KeyPoint point_test;
        point_test.pt=p_test;
//        point_test.angle=angle;
//        point_test.octave=octave;
        point_test.response=response;
//        point_test.class_id=class_id;
        
        point_test.angle=-1;
        point_test.octave=0;
        point_test.class_id=-1;
        curKf->keypoints.push_back(point_test);
//        cout<<"keypoints x="<<point_test.pt.x<<" y="<<point_test.pt.y<<endl;
    }
   
//    TE(all_keyPoint);
    
    
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveKF_1_2, shouldn't"<<endl;
    
    
    if(is_end){
//        cout<<"test curKf->pointNum "<<curKf->pointNum<<" curKf->keypoints.size()="<<curKf->keypoints.size()<<endl;
        
//        int winPointNum=curKf->point_clouds.size();
//        int start=curKf->pointNum-winPointNum;
//        for(int i=0;i<winPointNum;i++){
//        cv::KeyPoint curKeyPoint=curKf->keypoints[start+i];
//        curKf->measurements.push_back(curKeyPoint.pt);
//        }
//        curKf->measurements_origin=curKf->measurements;
        
        curKf->isAllKeypoint=true;
    }
}
//接收剩余的描述子
void receiveKF_add(const char* buffer,PoseGraph *poseGraph){
   unsigned char saveHeader[2] = {0xe1, 0x92};
   unsigned char hd[2];
   long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveKF_add, shouldn't"<<endl;
    
    
    int global_index_test;
    memcpy(&global_index_test, buffer+offset, sizeof(int));
    offset += sizeof(int);
//    cout<<"global_index_test="<<global_index_test<<endl;
    //找到该关键帧
    KeyFrame* curKf=poseGraph->getLastKeyframe_index(global_index_test);
    
    
//    cout<<"curKf->pointNum "<<curKf->pointNum<<" curKf->keypoints.size()="<<curKf->keypoints.size()<<endl;
    int des_sub=curKf->keypoints.size()-curKf->descriptors.size();
    int des_num;
    bool is_end=false;
    if(des_sub<=15){
        //说明是最后一躺
        is_end=true;
        des_num=des_sub;
        
        //验证一下
        memcpy(hd, buffer, 2);
        offset += 2;
        if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
            cerr<<"header error receiveKF_add, shouldn't"<<endl;
        
        int des_sub_client;
        memcpy(&des_sub_client, buffer+offset, sizeof(int));
        offset += sizeof(int);
        
        assert(des_sub_client==des_num);
//        if(des_sub_client!=des_num){
//            cout<<"error receiveKF_add , should't  des_sub_client!=des_num"<<endl;
//        }
    }else{
        des_num=15;
    }
    
    
        
    int len=256;
    for(int i=0;i<des_num;i++){
        
//        memcpy(&len, buffer+offset, sizeof(int));
//        offset += sizeof(int);
        
        BRIEF::bitset des_test;
        des_test.clear();
        
//        int len_1;
//        memcpy(&len_1, buffer+offset, sizeof(int));
//        offset += sizeof(int);

        
        for(size_t j=0;j<len;j++){
            char str_test_single;
            memcpy(&str_test_single, buffer+offset, sizeof(char));
            offset += sizeof(char);
            if(str_test_single=='1'){
                des_test.push_back(1);
            }else{
                des_test.push_back(0);
            }
        }
  
        curKf->descriptors.push_back(des_test);
    }
        
    
    if(is_end){
//        TS(win_point);
       //窗口特征点、描述符赋值
       curKf->setWin_keyPoint_des();
       //窗口点的深度、把3D坐标算出来
//       TE(win_point);
        
        curKf->is_des_end=true;
    }
}

//接收剩余的描述子压缩的参考下标
void receiveKF_add_compressIndex(const char* buffer,PoseGraph *poseGraph,int packLen){
    int allInfo_len=strlen(buffer);
    
   unsigned char saveHeader[2] = {0xe8, 0x94};
   unsigned char hd[2];
   long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveKF_add_compressIndex, shouldn't"<<endl;
    
    
    int global_index_test;
    memcpy(&global_index_test, buffer+offset, sizeof(int));
    offset += sizeof(int);
//    cout<<"global_index_test="<<global_index_test<<endl;
    //找到该关键帧
    KeyFrame* curKf=poseGraph->getLastKeyframe_index(global_index_test);
    
    
    int des_sub=curKf->keypoints.size()-curKf->desBow_index.size();
    int des_num=0;
    bool is_end=false;
    if(des_sub<=1636){
        //说明是最后一躺
        is_end=true;
        des_num=des_sub;

        //验证一下
        memcpy(hd, buffer, 2);
        offset += 2;
        if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
            cerr<<"header error receiveKF_add_compressIndex, shouldn't"<<endl;

        int des_sub_client;
        memcpy(&des_sub_client, buffer+offset, sizeof(int));
        offset += sizeof(int);
        
        assert(des_sub_client==des_num);
        
        int bitNum;
        memcpy(&bitNum, buffer+offset, sizeof(int));
        offset += sizeof(int);
        
        curKf->bitNum=bitNum;
//        if(des_sub_client!=des_num){
//            cout<<"error receiveKF_add , should't  des_sub_client!=des_num"<<endl;
//        }
    }else{
        des_num=1636;
    }
    
    const int des_num_end=des_num;
    std::bitset<1636 *20> binary_num;
    const size_t binary_num_size =binary_num.size();
    int i=0;
    
//    cout<<"offset="<<offset<<" , "<< packLen<<" , "<<sizeof(uchar)<<endl;
//    vector<uchar> bitstream;
//    packLen*=8;
    while(offset<packLen){
        uchar str_test_single;
        memcpy(&str_test_single, buffer+offset, sizeof(uchar));
        offset += sizeof(uchar);
//        bitstream.push_back(str_test_single);
        
        binary_num |= (static_cast<std::bitset<binary_num_size>>(str_test_single)) << (i * 8);
        i++;
        
//        cout<<"binary_num"<<binary_num<<endl;
//        cout<<"(static_cast<std::bitset<binary_num_size>>(str_test_single))"<<(static_cast<std::bitset<binary_num_size>>(str_test_single))<<endl;
    }
    
//    cout<<"接收到的数据";
//    vector<int> indexEnd;
    for(int i = 0; i < des_num; i++) {
        std::bitset<20> bits_single;
        // 将原bitset的末8位复制到新的bitset中
        for(int j = 0; j < 20; j++){
            bits_single[j] = binary_num[j ];
        }
        curKf->desBow_index.push_back(bits_single.to_ulong()); // 每次输出8位二进制数到对应字节中
        binary_num= binary_num >> 20;
        
//        cout<<bits_single.to_ulong()<<" , ";
    }
//    cout<<endl<<endl;
   
    if(is_end){
        curKf->is_bowIndex_end=true;
    }
}


//接收剩余的描述子
void receiveKF_add_compress(const char* buffer,PoseGraph *poseGraph,Client *c,int packLen){
//    int allInfo_len=strlen(buffer);
    
   unsigned char saveHeader[2] = {0xe1, 0x92};
   unsigned char hd[2];
   long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveKF_add_compress, shouldn't"<<endl;
    
    
    int global_index_test;
    memcpy(&global_index_test, buffer+offset, sizeof(int));
    offset += sizeof(int);
//    cout<<"global_index_test="<<global_index_test<<endl;
    //找到该关键帧
    KeyFrame* curKf=poseGraph->getLastKeyframe_index(global_index_test);
    
    
    int des_sub=curKf->bitNum-curKf->bitstream.size();
//    int des_num;
    bool is_end=false;
    if(des_sub<=4090){
        //说明是最后一躺
        is_end=true;
//        des_num=des_sub;

        //验证一下
        memcpy(hd, buffer, 2);
        offset += 2;
        if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
            cerr<<"header error receiveKF_add, shouldn't"<<endl;

        int des_sub_client;
        memcpy(&des_sub_client, buffer+offset, sizeof(int));
        offset += sizeof(int);

        assert(des_sub_client==(packLen-offset));

    }
//    else{
//        des_num=15;
//    }
    
    while(offset<packLen){
        uchar str_test_single;
        memcpy(&str_test_single, buffer+offset, sizeof(uchar));
        offset += sizeof(uchar);
        curKf->bitstream.push_back(str_test_single);
    }
    
    if(curKf->is_bowIndex_end && is_end){
        std::vector<cv::KeyPoint> kptsLeft;
        std::vector<BRIEF::bitset> descriptorsLeft;
        std::vector<unsigned int> visualWords;
        vector<int> bowIndex_ljl;
        c->decoder->decodeImageMono(curKf->bitstream, kptsLeft, descriptorsLeft, visualWords  ,curKf->desBow_index);
        curKf->descriptors=descriptorsLeft;
        curKf->setWin_keyPoint_des();
        curKf->is_des_end=true;
        
        curKf->bitstream.clear();
        curKf->desBow_index.clear();
    }else{
        if(!curKf->is_bowIndex_end && is_end){
            cout<<"数据发送到达顺序不一致 参考的bow下标还没发完，压缩的描述符已经全部发完了 ，可以在server进行处理 或者 发送bow下标那里进行处理"<<endl;
        }
        
    }
    
    
        

//    if(is_end){
//       //窗口特征点、描述符赋值
//       curKf->setWin_keyPoint_des();
//       //窗口点的深度、把3D坐标算出来
//        curKf->is_des_end=true;
//    }
}


//更新origin_pose
KeyFrame* receiveKF_OriginPose(const char* buffer,PoseGraph *poseGraph){
    unsigned char saveHeader[2] = {0xe8, 0x96};
    unsigned char hd[2];
    long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveKF_OriginPose1, shouldn't"<<endl;
    
    int global_index;
    memcpy(&global_index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    
    //接收origin_pose
    Eigen::Vector3d origin_T_w_i;
    Eigen::Matrix3d origin_R_w_i;
    for(int i=0;i<3;i++){
        memcpy(&origin_T_w_i[i], buffer+offset, sizeof(double));
        offset += sizeof(double);
    }
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            memcpy(&origin_R_w_i(i,j), buffer+offset, sizeof(double));
            offset += sizeof(double);
        }
    }

    //找到该关键帧
    KeyFrame* curKf=poseGraph->getLastKeyframe_index(global_index);
    if(curKf!=nullptr){
        curKf->updateOriginPose(origin_T_w_i, origin_R_w_i);
        curKf->IsOriginUpdate=true;
//        cout<<"IsOriginUpdate 找到关键帧"<<endl;
    }else{
        cout<<"IsOriginUpdate 未找到关键帧"<<global_index<<endl;
    }
    
    
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveKF_OriginPose2, shouldn't"<<endl;
    
    return curKf;

}
void receiveErrorLoop(const char* buffer,PoseGraph *poseGraph){
    unsigned char saveHeader[2] = {0xe7, 0x95};
    unsigned char hd[2];
    long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error ErrorLoop, shouldn't"<<endl;
    
    int curKF_Loop_Index;
    memcpy(&curKF_Loop_Index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    
    KeyFrame* curKf=poseGraph->getKeyframe(curKF_Loop_Index);
    curKf->removeLoop();
}
void receiveCorrectLoop(const char* buffer,PoseGraph *poseGraph){
    unsigned char saveHeader[2] = {0xe6, 0x94};
    unsigned char hd[2];
    long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error CorrectLoop, shouldn't"<<endl;
    
    int curKF_CorrectLoop_Index;
    memcpy(&curKF_CorrectLoop_Index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    
    //更新回环检测的位姿的偏移
    poseGraph->updateCorrectLoopPose(curKF_CorrectLoop_Index);
    
    
}

void receiveFrontPoseRelative(const char* buffer,PoseGraph *poseGraph){
    unsigned char saveHeader[2] = {0xe4, 0x92};
    unsigned char hd[2];
    long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveFrontPoseRelative, shouldn't"<<endl;
    
    Vector3d relative_t;
    double t_test;
    for(int i=0;i<3;i++){
        memcpy(&t_test, buffer+offset, sizeof(double));
        offset+=sizeof(double);
        relative_t[i]=t_test;
    }
    

//    double x,y,z,w;
//    memcpy(&x, buffer+offset, sizeof(double));
//    offset+=sizeof(double);
//    memcpy(&y, buffer+offset, sizeof(double));
//    offset+=sizeof(double);
//    memcpy(&z, buffer+offset, sizeof(double));
//    offset+=sizeof(double);
//    memcpy(&w, buffer+offset, sizeof(double));
//    offset+=sizeof(double);
//    Quaterniond relative_q(w,x,y,z);
    
    
    double relative_yaw;
    memcpy(&relative_yaw, buffer+offset, sizeof(double));
    offset+=sizeof(double);
    
    int relative_cur_index;
    memcpy(&relative_cur_index, buffer+offset, sizeof(int));
    offset+=sizeof(int);
    
    KeyFrame* kf=poseGraph->getLastKeyframe_index(relative_cur_index);

    kf->updateLoopConnection(relative_t, relative_yaw);
    kf->is_get_loop_info=true;
    
//    cout<<"测试 kf的回环："<<kf->global_index<<" "<<kf->loop_index<<" "<<kf->c->id<<endl;

    
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveFrontPoseRelative, shouldn't"<<endl;
    
    
//    Eigen::Matrix<double, 4, 1> connected_info_test;
//    connected_info_test <<relative_t.x(), relative_t.y(), relative_t.z(),relative_yaw;
//    cout<<"handleData relative_t_sendServer:"<<connected_info_test(0)<<" "<<connected_info_test(1)<<" "<<connected_info_test(2)<<" "<<connected_info_test(3)<<" cur_index"<<relative_cur_index<<endl;
}

void receiveFrontPoseRelative_add_pitch_roll(const char* buffer,PoseGraph *poseGraph){
    unsigned char saveHeader[2] = {0xe3, 0x91};
    unsigned char hd[2];
    long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveFrontPoseRelative, shouldn't"<<endl;
    
    Vector3d relative_t;
    double t_test;
    for(int i=0;i<3;i++){
        memcpy(&t_test, buffer+offset, sizeof(double));
        offset+=sizeof(double);
        relative_t[i]=t_test;
    }
    

//    double x,y,z,w;
//    memcpy(&x, buffer+offset, sizeof(double));
//    offset+=sizeof(double);
//    memcpy(&y, buffer+offset, sizeof(double));
//    offset+=sizeof(double);
//    memcpy(&z, buffer+offset, sizeof(double));
//    offset+=sizeof(double);
//    memcpy(&w, buffer+offset, sizeof(double));
//    offset+=sizeof(double);
//    Quaterniond relative_q(w,x,y,z);
    
    
    double relative_yaw;
    memcpy(&relative_yaw, buffer+offset, sizeof(double));
    offset+=sizeof(double);
    
    int relative_cur_index;
    memcpy(&relative_cur_index, buffer+offset, sizeof(int));
    offset+=sizeof(int);
    
    double relative_pitch;
    memcpy(&relative_pitch, buffer+offset, sizeof(double));
    offset+=sizeof(double);
    
    double relative_roll;
    memcpy(&relative_roll, buffer+offset, sizeof(double));
    offset+=sizeof(double);
    
    KeyFrame* kf=poseGraph->getLastKeyframe_index(relative_cur_index);
//    if (kf!=NULL) {
//        cout<<"updateLoopConnection handleData"<<endl;
        kf->relative_pitch=relative_pitch;
        kf->relative_roll=relative_roll;
        kf->updateLoopConnection(relative_t, relative_yaw);
        kf->is_get_loop_info=true;
//    }
    
//    cout<<"测试 kf的回环："<<kf->global_index<<" "<<kf->loop_index<<" "<<kf->c->id<<endl;
    
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveFrontPoseRelative, shouldn't"<<endl;
    
    
//    Eigen::Matrix<double, 4, 1> connected_info_test;
//    connected_info_test <<relative_t.x(), relative_t.y(), relative_t.z(),relative_yaw;
//    cout<<"handleData receiveFrontPoseRelative_add_pitch_roll:"<<connected_info_test(0)<<" "<<connected_info_test(1)<<" "<<connected_info_test(2)<<" "<<connected_info_test(3)<<" cur_index"<<relative_cur_index<<endl;
}

void receiveFrontPoseRelative_add_pitch_roll_forRemove(const char* buffer,PoseGraph *poseGraph){
    unsigned char saveHeader[2] = {0xe8, 0x96};
    unsigned char hd[2];
    long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveFrontPoseRelative_add_pitch_roll_forRemove, shouldn't"<<endl;
    
    Vector3d relative_t;
    double t_test;
    for(int i=0;i<3;i++){
        memcpy(&t_test, buffer+offset, sizeof(double));
        offset+=sizeof(double);
        relative_t[i]=t_test;
    }
    

    
    double relative_yaw;
    memcpy(&relative_yaw, buffer+offset, sizeof(double));
    offset+=sizeof(double);
    
    int relative_cur_index;
    memcpy(&relative_cur_index, buffer+offset, sizeof(int));
    offset+=sizeof(int);
    
    double relative_pitch;
    memcpy(&relative_pitch, buffer+offset, sizeof(double));
    offset+=sizeof(double);
    
    double relative_roll;
    memcpy(&relative_roll, buffer+offset, sizeof(double));
    offset+=sizeof(double);
    
    KeyFrame* kf=poseGraph->getLastKeyframe_index(relative_cur_index);
//    if (kf!=NULL) {
//        cout<<"updateLoopConnection handleData"<<endl;
        kf->relative_pitch=relative_pitch;
        kf->relative_roll=relative_roll;
        kf->updateLoopConnection(relative_t, relative_yaw);
        kf->is_get_loop_info=true;
    kf->isRemove_loop=true;
//    }
    
//    cout<<"测试 kf的回环："<<kf->global_index<<" "<<kf->loop_index<<" "<<kf->c->id<<endl;
    
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveFrontPoseRelative_add_pitch_roll_forRemove, shouldn't"<<endl;
    
}







void produceStreamForGloablLoopData(stringstream &outputStream,PoseGraph *poseGraph){
    char name[]="Global";
    int len_name=strlen(name);
    outputStream.write(reinterpret_cast<char*>(&len_name), sizeof(int));
    for (int i=0;i!=len_name;i++) {
        outputStream.write(reinterpret_cast<char*>(&(name[i])), sizeof(char));
    }
    
    unsigned char saveHeader[2] = {0xcb, 0x89};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    vector<Vector3d> t_global=poseGraph->get_t_global();
//    vector<double> yaw_global=poseGraph->get_yaw_global();//但是这个暂时 没更新
    vector<Matrix3d> r_global=poseGraph->get_r_global();
    
   
    //ts
    int ts_len=t_global.size();
    outputStream.write(reinterpret_cast<char*>(&ts_len), sizeof(int));
    for (auto iter=t_global.begin(),iter_end=t_global.end(); iter!=iter_end; iter++) {
        for(int i=0;i<3;i++){
            outputStream.write(reinterpret_cast<char*>(& (*iter)(i)), sizeof(double));
        }
    }
    
    //rs
    for (auto iter=r_global.begin(),iter_end=r_global.end(); iter!=iter_end; iter++) {
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                outputStream.write(reinterpret_cast<char*>(&(*iter)(i,j)), sizeof(double));
            }
        }
    }
    
    
    
    
    //回环帧、匹配帧id
    int loopKf_index=poseGraph->loopKF_index.front();
    poseGraph->loopKF_index.pop();
    int curKf_loop_index=poseGraph->curKF_loop_index.front();
    poseGraph->curKF_loop_index.pop();
    int earliest_test=poseGraph->earliest_queue.front();
    poseGraph->earliest_queue.pop();
    outputStream.write(reinterpret_cast<char*>(&loopKf_index), sizeof(int));
    outputStream.write(reinterpret_cast<char*>(&curKf_loop_index), sizeof(int));
    outputStream.write(reinterpret_cast<char*>(&earliest_test), sizeof(int));
    
    poseGraph->special_kf_mutex_intra.lock();
    vector<int> special_kf_inOpti=poseGraph->special_kf_inOpti_intra.front();
    poseGraph->special_kf_inOpti_intra.pop();
    poseGraph->special_kf_mutex_intra.unlock();
    int special_kf_len=special_kf_inOpti.size();
    outputStream.write(reinterpret_cast<char*>(&special_kf_len), sizeof(int));
//    cout<<"size="<<special_kf_len<<" ->";
    for(int i=0;i<special_kf_len; i++){
        outputStream.write(reinterpret_cast<char*>(&special_kf_inOpti[i]), sizeof(int));
    }
//    cout<<endl;
}
char* sendGlobalLoopData(int &len_end,PoseGraph *poseGraph){
    std::stringstream loopData_Stream;
    produceStreamForGloablLoopData(loopData_Stream,poseGraph);
   
    int len = loopData_Stream.str().length();
    len_end=len+5;
    
    
    long offset = 0;

    char* bytearr=new char[len_end+sizeof(int)];
    memcpy(bytearr,reinterpret_cast<char*>(&len_end), sizeof(int));
    offset+=sizeof(int);
    if (len>0) {
        memcpy(bytearr+offset, loopData_Stream.str().c_str(), len);
        offset += len;
    }
    memcpy(bytearr+offset, saveEnd, 5);
    len_end+=sizeof(int);
    
    
    return bytearr;
    
}


void produceStreamForLoopData(stringstream &outputStream,VINS* vins){
    char name[]="loop";
    int len_name=strlen(name);
    outputStream.write(reinterpret_cast<char*>(&len_name), sizeof(int));
//    cout<<"len_name "<<len_name<<endl;
    for (int i=0;i!=len_name;i++) {
        outputStream.write(reinterpret_cast<char*>(&(name[i])), sizeof(char));
//        cout<<"name[i] "<<name[i]<<endl;
    }
    
    unsigned char saveHeader[2] = {0xca, 0x88};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
    //header
    RetriveData retrive_data=vins->retrive_pose_data;
//    double header=retrive_data.header;
//    outputStream.write(reinterpret_cast<char*>(&header), sizeof(double));
    
    //measurements
    vector<cv::Point2f> measurements=retrive_data.measurements;
    int measurements_len=measurements.size();
    outputStream.write(reinterpret_cast<char*>(&measurements_len), sizeof(int));
    for(int i=0;i<measurements_len;i++){
        cv::Point2f point=measurements[i];
        float x=point.x;
        float y=point.y;
        outputStream.write(reinterpret_cast<char*>(&x), sizeof(float));
        outputStream.write(reinterpret_cast<char*>(&y), sizeof(float));
    }
    
    cout<<"这里不应该传送"<<endl;
    
    //features_ids
    vector<int> features_ids=retrive_data.features_ids;
    int features_len=features_ids.size();
//    cout<<measurements_len<<" test 比较大小 "<<features_len<<endl;
    outputStream.write(reinterpret_cast<char*>(&features_len), sizeof(int));
    for(int i=0;i<features_len;i++){
        int feature_id=features_ids[i];
        outputStream.write(reinterpret_cast<char*>(&feature_id), sizeof(int));
    }
    //cur_index
    int cur_index=retrive_data.cur_index;
    outputStream.write(reinterpret_cast<char*>(&cur_index), sizeof(int));
    
//    Vector3d p_old=retrive_data.P_old;
//    for(int i=0;i<3;i++){
//        outputStream.write(reinterpret_cast<char*>(&p_old(i)), sizeof(double));
//    }
//
//    Quaterniond q_old=retrive_data.Q_old;
//    outputStream.write(reinterpret_cast<char*>(&q_old.x()), sizeof(double));
//    outputStream.write(reinterpret_cast<char*>(&q_old.y()), sizeof(double));
//    outputStream.write(reinterpret_cast<char*>(&q_old.z()), sizeof(double));
//    outputStream.write(reinterpret_cast<char*>(&q_old.w()), sizeof(double));
    
    //回环帧
    outputStream.write(reinterpret_cast<char*>(&retrive_data.old_index), sizeof(int));
    
    
}
char* sendLoopData(int &len_end,VINS* vins){
    std::stringstream loopData_Stream;
     produceStreamForLoopData(loopData_Stream,vins);
    
     int len = loopData_Stream.str().length();
     len_end=len+5;
    
//    cout<<"len_end:"<<len_end<<endl;
    
     long offset=0;
     char* bytearr=new char[len_end+sizeof(int)];
    memcpy(bytearr,reinterpret_cast<char*>(&len_end), sizeof(int));
    offset+=sizeof(int);
     if (len>0) {
         memcpy(bytearr+offset, loopData_Stream.str().c_str(), len);
         offset+=len;
     }
     memcpy(bytearr+offset, saveEnd, 5);
        
    
    len_end+=sizeof(int);
    
     
     return bytearr;

}


void produceStreamForLoopData_another(stringstream &outputStream,VINS* vins){
    char name[]="loop_ano";
    int len_name=strlen(name);
    outputStream.write(reinterpret_cast<char*>(&len_name), sizeof(int));
//    cout<<"len_name "<<len_name<<endl;
    for (int i=0;i!=len_name;i++) {
        outputStream.write(reinterpret_cast<char*>(&(name[i])), sizeof(char));
//        cout<<"name[i] "<<name[i]<<endl;
    }
    
    unsigned char saveHeader[2] = {0xcb, 0x68};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
    //header
    RetriveData retrive_data=vins->retrive_pose_data;

    //cur_index
    int cur_index=retrive_data.cur_index;
    outputStream.write(reinterpret_cast<char*>(&cur_index), sizeof(int));
  
    
    //回环帧
    outputStream.write(reinterpret_cast<char*>(&retrive_data.old_index), sizeof(int));
    
//    cout<<"发送的回环是"<<cur_index<<" ,"<<retrive_data.old_index<<endl;
    
    
}
char* sendLoopData_another(int &len_end,VINS* vins){
    std::stringstream loopData_Stream;
     produceStreamForLoopData_another(loopData_Stream,vins);
    
     int len = loopData_Stream.str().length();
     len_end=len+5;
    
//    cout<<"len_end:"<<len_end<<endl;
    
     long offset=0;
     char* bytearr=new char[len_end+sizeof(int)];
    memcpy(bytearr,reinterpret_cast<char*>(&len_end), sizeof(int));
    offset+=sizeof(int);
     if (len>0) {
         memcpy(bytearr+offset, loopData_Stream.str().c_str(), len);
         offset+=len;
     }
     memcpy(bytearr+offset, saveEnd, 5);
        
    
    len_end+=sizeof(int);
    
     
     return bytearr;

}

void produceStreamForLoopData_another2(stringstream &outputStream,VINS* vins){
    char name[]="loop_an2";
    int len_name=strlen(name);
    outputStream.write(reinterpret_cast<char*>(&len_name), sizeof(int));
//    cout<<"len_name "<<len_name<<endl;
    for (int i=0;i!=len_name;i++) {
        outputStream.write(reinterpret_cast<char*>(&(name[i])), sizeof(char));
//        cout<<"name[i] "<<name[i]<<endl;
    }
    
    unsigned char saveHeader[2] = {0xcb, 0x68};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
    //header
    RetriveData retrive_data=vins->retrive_pose_data;

    //cur_index
    int cur_index=retrive_data.cur_index;
    outputStream.write(reinterpret_cast<char*>(&cur_index), sizeof(int));
  
    
    //回环帧
    outputStream.write(reinterpret_cast<char*>(&retrive_data.old_index), sizeof(int));
    
//    cout<<"发送的回环是"<<cur_index<<" ,"<<retrive_data.old_index<<endl;
   
    for(int i=0;i<7;i++){
        outputStream.write(reinterpret_cast<char*>(&retrive_data.loop_pose[i]), sizeof(double));
    }
    
    outputStream.write(reinterpret_cast<char*>(&retrive_data.header), sizeof(double));
    
    //measurements
//    vector<cv::Point2f> measurements=retrive_data.measurements;
//    int measurements_len=measurements.size();
//    outputStream.write(reinterpret_cast<char*>(&measurements_len), sizeof(int));
//    for(int i=0;i<measurements_len;i++){
//        cv::Point2f point=measurements[i];
//        float x=point.x;
//        float y=point.y;
//        outputStream.write(reinterpret_cast<char*>(&x), sizeof(float));
//        outputStream.write(reinterpret_cast<char*>(&y), sizeof(float));
//    }
//
//    cout<<"这里不应该传送"<<endl;
//
//    //features_ids_all
    vector<vector<int> > features_ids=retrive_data.features_ids_all;
    int features_len=features_ids.size();
    outputStream.write(reinterpret_cast<char*>(&features_len), sizeof(int));
    for(int i=0;i<features_len;i++){
        vector<int> feature_id=features_ids[i];
        int features_single_len=feature_id.size();
        outputStream.write(reinterpret_cast<char*>(&features_single_len), sizeof(int));
        for(int j=0;j<features_single_len;j++){
            int feature_single_id=feature_id[j];
            outputStream.write(reinterpret_cast<char*>(&feature_single_id), sizeof(int));
        }
    }
    
//    point_clouds_all
    vector<vector<Eigen::Vector3d>> point_clouds_all=retrive_data.point_clouds_all;
    int point_clouds_all_len=point_clouds_all.size();
    outputStream.write(reinterpret_cast<char*>(&point_clouds_all_len), sizeof(int));
    for(int i=0;i<point_clouds_all_len;i++){
        vector<Eigen::Vector3d> point_clouds=point_clouds_all[i];
        int point_clouds_single_len=point_clouds.size();
        outputStream.write(reinterpret_cast<char*>(&point_clouds_single_len), sizeof(int));
        for(int j=0;j<point_clouds_single_len;j++){
            Eigen::Vector3d point_clouds_single=point_clouds[i];
            double x=point_clouds_single[0];
            double y=point_clouds_single[1];
            double z=point_clouds_single[2];
            outputStream.write(reinterpret_cast<char*>(&x), sizeof(double));
            outputStream.write(reinterpret_cast<char*>(&y), sizeof(double));
            outputStream.write(reinterpret_cast<char*>(&z), sizeof(double));
        }
    }

    
}
char* sendLoopData_another2(int &len_end,VINS* vins){
    std::stringstream loopData_Stream;
     produceStreamForLoopData_another2(loopData_Stream,vins);
    
     int len = loopData_Stream.str().length();
     len_end=len+5;
    
    cout<<"sendLoopData_another2:"<<len_end<<endl;
    
     long offset=0;
     char* bytearr=new char[len_end+sizeof(int)];
    memcpy(bytearr,reinterpret_cast<char*>(&len_end), sizeof(int));
    offset+=sizeof(int);
     if (len>0) {
         memcpy(bytearr+offset, loopData_Stream.str().c_str(), len);
         offset+=len;
     }
     memcpy(bytearr+offset, saveEnd, 5);
        
    
    len_end+=sizeof(int);
    
     
     return bytearr;

}


void produceStreamForRejectWithF(stringstream &outputStream,VINS* vins){
    char name[]="reject";
    int len_name=strlen(name);
    outputStream.write(reinterpret_cast<char*>(&len_name), sizeof(int));
//    cout<<"len_name "<<len_name<<endl;
    for (int i=0;i!=len_name;i++) {
        outputStream.write(reinterpret_cast<char*>(&(name[i])), sizeof(char));
//        cout<<"name[i] "<<name[i]<<endl;
    }
    
    unsigned char saveHeader[2] = {0xc9, 0x87};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
        
        //拒绝外点的下标
    int cur_reject_index=vins->send_status_index.front();
    vins->send_status_index.pop();
    outputStream.write(reinterpret_cast<char*>(&cur_reject_index), sizeof(int));
    
    vector<uchar> status=vins->send_status.front();
    vins->send_status.pop();
    int status_index=status.size();
    outputStream.write(reinterpret_cast<char*>(&status_index), sizeof(int));
    
    for(int i=0;i<status_index;i++){
        uchar iter=status[i];
        outputStream.write(reinterpret_cast<char*>(&iter), sizeof(uchar));
    }
    
        

}
char* sendRejectWithF(int &len_end,VINS* vins){
    std::stringstream loopData_Stream;
     produceStreamForRejectWithF(loopData_Stream,vins);
    
     int len = loopData_Stream.str().length();
     len_end=len+5;
    
//    cout<<"len_end:"<<len_end<<endl;
    
     long offset=0;
     char* bytearr=new char[len_end+sizeof(int)];
    memcpy(bytearr,reinterpret_cast<char*>(&len_end), sizeof(int));
    offset+=sizeof(int);
     if (len>0) {
         memcpy(bytearr+offset, loopData_Stream.str().c_str(), len);
         offset+=len;
     }
     memcpy(bytearr+offset, saveEnd, 5);
        
    
    len_end+=sizeof(int);
    
     
     return bytearr;
}


int receiveAr(const char* buffer,PoseGraphGlobal *poseGraphGlobal, int clientId){
    unsigned char saveHeader[2] =  {0xe0, 0x98};
    unsigned char hd[2];
    long unsigned int offset=0;
    memcpy(hd, buffer, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveAr, shouldn't"<<endl;
    
    Vector3f ori;
    for(int i=0;i<3;i++){
        memcpy(&ori[i], buffer+offset, sizeof(float));
        offset += sizeof(float);
    }
    
    Vector3f cox;
    for(int i=0;i<3;i++){
        memcpy(&cox[i], buffer+offset, sizeof(float));
        offset += sizeof(float);
    }
    
    Vector3f coy;
    for(int i=0;i<3;i++){
        memcpy(&coy[i], buffer+offset, sizeof(float));
        offset += sizeof(float);
    }
    
    Vector3f coz;
    for(int i=0;i<3;i++){
        memcpy(&coz[i], buffer+offset, sizeof(float));
        offset += sizeof(float);
    }
    
    float size;
    memcpy(&size, buffer+offset, sizeof(float));
    offset += sizeof(float);
    
    memcpy(hd, buffer+offset, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveAr, shouldn't"<<endl;
    
    GroundPoint gp_ar(poseGraphGlobal->Ground_idx++, clientId);
    gp_ar.ori=ori;
    gp_ar.cox=cox;
    gp_ar.coy=coy;
    gp_ar.coz=coz;
    gp_ar.size=size;
    gp_ar.isSendAr[clientId]=1;
    poseGraphGlobal->grounds.push_back(gp_ar);
    
    return poseGraphGlobal->Ground_idx-1;
}


//发送AR给其它client
void produceStreamForAr(stringstream &outputStream,PoseGraphGlobal *poseGraphGlobal, int ground_ar_idx, int clientId){
    char name[]="ar";
    int len_name=strlen(name);
    outputStream.write(reinterpret_cast<char*>(&len_name), sizeof(int));
//    cout<<"len_name "<<len_name<<endl;
    for (int i=0;i!=len_name;i++) {
        outputStream.write(reinterpret_cast<char*>(&(name[i])), sizeof(char));
//        cout<<"name[i] "<<name[i]<<endl;
    }
    
    unsigned char saveHeader[2] = {0xc8, 0x88};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
        
    //拿到AR物体
    GroundPoint gp_send=poseGraphGlobal->grounds[ground_ar_idx];
    
    
    Vector3f ori=gp_send.ori;
    for(int i=0;i<3;i++){
        outputStream.write(reinterpret_cast<char*>(&ori[i]), sizeof(float));
        
    }
    
    Vector3f cox=gp_send.cox;
    for(int i=0;i<3;i++){
        outputStream.write(reinterpret_cast<char*>(&cox[i]), sizeof(float));
        
    }
    
    Vector3f coy=gp_send.coy;
    for(int i=0;i<3;i++){
        outputStream.write(reinterpret_cast<char*>(&coy[i]), sizeof(float));
        
    }
    
    Vector3f coz=gp_send.coz;
    for(int i=0;i<3;i++){
        outputStream.write(reinterpret_cast<char*>(&coz[i]), sizeof(float));
        
    }
        
    outputStream.write(reinterpret_cast<char*>(&gp_send.size), sizeof(float));
    
   
}

char* sendArData(int &len_end,PoseGraphGlobal *poseGraphGlobal, int ground_ar_idx, int clientId){
    std::stringstream loopData_Stream;
     produceStreamForAr(loopData_Stream,poseGraphGlobal,ground_ar_idx, clientId);
    
     int len = loopData_Stream.str().length();
     len_end=len+5;
    
    
     long offset=0;
     char* bytearr=new char[len_end+sizeof(int)];
    memcpy(bytearr,reinterpret_cast<char*>(&len_end), sizeof(int));
    offset+=sizeof(int);
     if (len>0) {
         memcpy(bytearr+offset, loopData_Stream.str().c_str(), len);
         offset+=len;
     }
     memcpy(bytearr+offset, saveEnd, 5);
        
    
    len_end+=sizeof(int);
    
     
     return bytearr;
}


//还需要告知哪几个关键帧 检测到了两个地图之间的公共区域
void produceStreamForGloablLoopData_multiClient(stringstream &outputStream,PoseGraph *poseGraph){
    char name[]="Fusion";
    int len_name=strlen(name);
    outputStream.write(reinterpret_cast<char*>(&len_name), sizeof(int));
    for (int i=0;i!=len_name;i++) {
        outputStream.write(reinterpret_cast<char*>(&(name[i])), sizeof(char));
    }
    
    unsigned char saveHeader[2] = {0xc1, 0x86};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    vector<Vector3d> t_global=poseGraph->get_t_global_multiClient();
    vector<Matrix3d> r_global=poseGraph->get_r_global_multiClient();
    vector<int> kf_id_hasComPlace_withOtherMap=poseGraph->get_kf_id_hasComPlace_withOtherMap();
  
    
    //ts
    int ts_len=t_global.size();
    outputStream.write(reinterpret_cast<char*>(&ts_len), sizeof(int));
    for (auto iter=t_global.begin(),iter_end=t_global.end(); iter!=iter_end; iter++) {
        for(int i=0;i<3;i++){
            outputStream.write(reinterpret_cast<char*>(& (*iter)(i)), sizeof(double));
        }
    }
    //rs
    for (auto iter=r_global.begin(),iter_end=r_global.end(); iter!=iter_end; iter++) {
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                outputStream.write(reinterpret_cast<char*>(&(*iter)(i,j)), sizeof(double));
            }
        }
    }
    
    //kf id
    int kf_len=kf_id_hasComPlace_withOtherMap.size();
    outputStream.write(reinterpret_cast<char*>(&kf_len), sizeof(int));
    for(int i=0;i<kf_len;i++){
        outputStream.write(reinterpret_cast<char*>(&(kf_id_hasComPlace_withOtherMap[i])), sizeof(int));
    }
    
//    outputStream.write(reinterpret_cast<char*>(&poseGraph->max_frame_num_global), sizeof(int));
//    outputStream.write(reinterpret_cast<char*>(&poseGraph->min_dis), sizeof(double));
    
    
    //回环帧、匹配帧id
    int loop_index=poseGraph->loop_index_multiClient.front();
    poseGraph->loop_index_multiClient.pop();
    int curKF_loop_index=poseGraph->curKF_loop_index_multiClient.front();
    poseGraph->curKF_loop_index_multiClient.pop();
    outputStream.write(reinterpret_cast<char*>(&loop_index), sizeof(int));//这个其实并不是当前的回环帧，而是最早的回环帧
    outputStream.write(reinterpret_cast<char*>(&curKF_loop_index), sizeof(int));
   
    
    //验证 可能发送变慢 导致客户端那边全局优化堆起来了， 里程计的值会有变化
    //这里发的是 融合优化时，needresample 值为0的kf 
//    vector<int> special_kf_inOpti=poseGraph->special_kf_inOpti;
//    int special_kf_len=special_kf_inOpti.size();
//    outputStream.write(reinterpret_cast<char*>(&special_kf_len), sizeof(int));
//    for(int i=0;i<special_kf_len; i++){
//        outputStream.write(reinterpret_cast<char*>(&special_kf_inOpti[i]), sizeof(int));
//    }
    
    poseGraph->special_kf_mutex.lock();
    vector<int> special_kf_inOpti=poseGraph->special_kf_inOpti.front();
    poseGraph->special_kf_inOpti.pop();
    poseGraph->special_kf_mutex.unlock();
    int special_kf_len=special_kf_inOpti.size();
    outputStream.write(reinterpret_cast<char*>(&special_kf_len), sizeof(int));
    for(int i=0;i<special_kf_len; i++){
        outputStream.write(reinterpret_cast<char*>(&special_kf_inOpti[i]), sizeof(int));
    }
    
    
    if((t_global.size()-1)!=(r_global.size()-1) || (t_global.size()-1)!=special_kf_inOpti.size()){
        cout<<"优化的rt大小为："<<t_global.size()<<" , "<<r_global.size()<<" , "<<special_kf_inOpti.size()<<endl;
        assert(false);
    }
    
}
char* sendGlobalLoopData_multiClient(int &len_end,PoseGraph *poseGraph){
    std::stringstream loopData_Stream;
    produceStreamForGloablLoopData_multiClient(loopData_Stream,poseGraph);
   
    int len = loopData_Stream.str().length();
    len_end=len+5;
    
    
    long offset = 0;

    char* bytearr=new char[len_end+sizeof(int)];
    memcpy(bytearr,reinterpret_cast<char*>(&len_end), sizeof(int));
    offset+=sizeof(int);
    if (len>0) {
        memcpy(bytearr+offset, loopData_Stream.str().c_str(), len);
        offset += len;
    }
    memcpy(bytearr+offset, saveEnd, 5);
    len_end+=sizeof(int);
    
    
    return bytearr;
    
}
