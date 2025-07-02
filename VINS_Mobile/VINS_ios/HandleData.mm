//
//  HandleData.cpp
//  VINS_ios
//
//  Created by 张剑华 on 2020/5/27.
//  Copyright © 2020 zx. All rights reserved.
//

#include "HandleData.h"
#import <UIKit/UIKit.h>

#include "SocketSingleton.h"
static unsigned long packIdx=1;//可以用来做后续的未接收到的数据的重发
unsigned char saveEnd[5]={0xeb, 0x95, 0x96, '\r', '\n'};

string getTime_handle()
{
    time_t timep;
    time (&timep); //获取time_t类型的当前时间
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );//对日期和时间进行格式化
    return tmp;
}

void produceStreamForCameraImuParam(stringstream &outputStream,DeviceType deviceType){
    unsigned char saveHeader[2] = {0xeb, 0x99};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
    outputStream.write(reinterpret_cast<char*>(&deviceType), sizeof(DeviceType));
}
void sendCameraParam(DeviceType deviceType){
//    cout<<"发送数据 device 11："<<getTime_handle()<<endl;
    TS(device);
    std::stringstream cameraImuStream;
    produceStreamForCameraImuParam(cameraImuStream,deviceType);
    size_t len=cameraImuStream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "device");
    packhead.PackIdx = packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), cameraImuStream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
//    cout<<"socket writeData will"<<endl;
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
    cameraImuStream.clear();
    packIdx++;
//        cout<<"发送数据 device 22："<<getTime_handle()<<endl;
    TE(device);
}



void produceStreamForInit_aligmentParam(stringstream &outputStream,Eigen::Vector3d g){
    unsigned char saveHeader[2] = {0xea, 0x98};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    for(int i=0;i<3;i++){
        outputStream.write(reinterpret_cast<char*>(&g[i]), sizeof(double));
    }
}
void sendInit_aligmentParam(Eigen::Vector3d g){
//        cout<<"发送数据 g_imu 11："<<getTime_handle()<<endl;
    TS(g_imu);
    std::stringstream imu_g_Stream;
    produceStreamForInit_aligmentParam(imu_g_Stream,g);
    size_t len=imu_g_Stream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "g_imu");
    packhead.PackIdx = packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), imu_g_Stream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
    imu_g_Stream.clear();
    packIdx++;
//    cout<<"send g   "<<g.norm()<<"   "<<g.transpose()<<endl;
//    cout<<"发送数据 g_imu 22："<<getTime_handle()<<endl;
    TE(g_imu);
}

void produceStreamForKF(stringstream &outputStream,KeyFrame *imgKeyFrame){
    unsigned char saveHeader[2] = {0xe9, 0x97};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    //先告诉它窗口特征点数量
    int winPointNum=imgKeyFrame->getWinPointsSize();
//    cout<<"winPointNum size="<<winPointNum<<endl;
    outputStream.write(reinterpret_cast<char*>(&winPointNum), sizeof(int));
    
//    for(int i=0;i<winPointNum;i++){
//        cout<<"win_point:"<<imgKeyFrame->window_keypoints[i].pt.x<<" "<<imgKeyFrame->window_keypoints[i].pt.y<<endl;
//    }
    
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    //弄清楚一下 这个特征点做匹配 要用的是不是 只是相机平面坐标
    //先发全部特征点、描述 前面是回环用的特征点，后面是窗口特征点
    int pointNum=imgKeyFrame->keypoints.size();
//    cout<<"pointNum size="<<pointNum<<endl;
    outputStream.write(reinterpret_cast<char*>(&pointNum), sizeof(int));
    
    for(std::vector<cv::KeyPoint>::iterator iter=imgKeyFrame->keypoints.begin(),iter_end=imgKeyFrame->keypoints.end();iter!=iter_end;iter++){
        
        outputStream.write(reinterpret_cast<char*>(&(*iter).angle), sizeof(float));
        outputStream.write(reinterpret_cast<char*>(&(*iter).octave), sizeof(int));
        outputStream.write(reinterpret_cast<char*>(&(*iter).pt.x), sizeof(float));
        outputStream.write(reinterpret_cast<char*>(&(*iter).pt.y), sizeof(float));
        outputStream.write(reinterpret_cast<char*>(&(*iter).response), sizeof(float));
        outputStream.write(reinterpret_cast<char*>(&(*iter).class_id), sizeof(int));
//        cout<<"keypoints x="<<(*iter).pt.x<<" y="<<(*iter).pt.y<<endl;
    }

    //特征点和描述子数量相等
//    cout<<"des size="<<imgKeyFrame->descriptors.size()<<endl;
    for(std::vector<BRIEF::bitset>::iterator iter=imgKeyFrame->descriptors.begin(),iter_end=imgKeyFrame->descriptors.end();iter!=iter_end;iter++){
        int len=(*iter).size();
        outputStream.write(reinterpret_cast<char*>(&len), sizeof(int));
//        cout<<"des_signle size="<<len<<endl;
        
        string str;
        to_string((*iter),str);
        string::iterator ite=str.begin();
        
        for(size_t i=0;i<len;i++){
            outputStream.write(reinterpret_cast<char*>(&(*ite)), sizeof(char));
            ite++;
            
            
        }
//        cout<<"des_single ="<<*iter<<endl;
    }
    
    //pointClouds
//    cout<<"imgKeyFrame->point_clouds size="<<imgKeyFrame->point_clouds.size()<<endl;
    for(std::vector<Eigen::Vector3d>::iterator iter=imgKeyFrame->point_clouds.begin(),iter_end=imgKeyFrame->point_clouds.end();iter!=iter_end;iter++){
//        cout<<"imgKeyFrame->point_clouds single xyz=";
        for(int i=0;i<3;i++){
            outputStream.write(reinterpret_cast<char*>(&(*iter)(i)), sizeof(double));
//            cout<<(*iter)(i)<<" ";
        }
//        cout<<endl;

    }
    
//    for(int i=0;i<imgKeyFrame->measurements.size();i++){
//        cout<<"measurements"<<imgKeyFrame->measurements[i].x  <<" "<<imgKeyFrame->measurements[i].y<<endl;
//    }
    
//    for(int i=0;i<imgKeyFrame->point_clouds_origin.size();i++){
//        cout<<"point_clouds_origin"<<imgKeyFrame->point_clouds_origin[i](0) <<" "<<imgKeyFrame->point_clouds_origin[i](1)<<" "<<imgKeyFrame->point_clouds_origin[i](2)<<endl;
//    }
//
//    for(int i=0;i<imgKeyFrame->measurements_origin.size();i++){
//        cout<<"measurements_origin"<<imgKeyFrame->measurements_origin[i].x  <<" "<<imgKeyFrame->measurements_origin[i].y<<endl;
//    }

    
    
//    cout<<endl;
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
    //发位姿
    Eigen::Vector3d T_w_i;
    Eigen::Matrix3d R_w_i;
    imgKeyFrame->getPose(T_w_i, R_w_i);
//    cout<<"pose T: ";
    for(int i=0;i<3;i++){
        outputStream.write(reinterpret_cast<char*>(&T_w_i[i]), sizeof(double));
//        cout<<T_w_i[i]<<" ";
    }
//    cout<<endl;
//
//    cout<<"pose R: ";
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            outputStream.write(reinterpret_cast<char*>(&R_w_i(i,j)), sizeof(double));
//            cout<<R_w_i(i,j)<<" ";
        }
    }
//    cout<<endl;
    
    //发原始位姿
    imgKeyFrame->getOriginPose(T_w_i, R_w_i);
//    cout<<"origin pose T: ";
    for(int i=0;i<3;i++){
        outputStream.write(reinterpret_cast<char*>(&T_w_i[i]), sizeof(double));
//        cout<<T_w_i[i]<<" ";
    }
//    cout<<endl;
    
    
//    cout<<"origin pose R: ";
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            outputStream.write(reinterpret_cast<char*>(&R_w_i(i,j)), sizeof(double));
//            cout<<R_w_i(i,j)<<" ";
        }
    }
//    cout<<endl;
    
    //发全局id 其实就是keyframeDatabase里面的index
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->global_index), sizeof(int));
//    cout<<"imgKeyFrame->global_index"<<imgKeyFrame->global_index<<endl;
    //序列号
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->segment_index), sizeof(int));
//    cout<<"segment_index "<<imgKeyFrame->segment_index<<endl;
    
    //header有什么用 回环后，窗口内算重定位误差 用来判断匹配帧是不是还在窗口内
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->header), sizeof(double));
//    cout<<"header "<<imgKeyFrame->header<<endl;
    
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
    
    //features_id 回环检测是会有外点移除
    int feature_len=imgKeyFrame->features_id.size();
    outputStream.write(reinterpret_cast<char*>(&feature_len), sizeof(int));
//    cout<<"feature_len="<<feature_len<<endl;
//    cout<<"features_id=";
    for(int i=0;i<feature_len;i++){
        outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->features_id[i]), sizeof(int));
//        cout<<imgKeyFrame->features_id[i]<<" ";
    }
//    cout<<endl;
    
    
        outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
}

//-------------第一篇论文 发送描述符
std::vector<cv::KeyPoint>::iterator produceStreamForKF_1_1(stringstream &outputStream,KeyFrame *imgKeyFrame,int &can_send_keys_num){
    unsigned char saveHeader[2] = {0xe9, 0x97};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    //先告诉它窗口特征点数量
    int winPointNum=imgKeyFrame->getWinPointsSize();
//    cout<<"winPointNum size="<<winPointNum<<endl;
    outputStream.write(reinterpret_cast<char*>(&winPointNum), sizeof(int));
   
    //特征点的数量
    int pointNum=imgKeyFrame->keypoints.size();
//    cout<<"pointNum size="<<pointNum<<endl;
    outputStream.write(reinterpret_cast<char*>(&pointNum), sizeof(int));
    
    //pointClouds
//    cout<<"imgKeyFrame->point_clouds size="<<imgKeyFrame->point_clouds.size()<<endl;
    for(std::vector<Eigen::Vector3d>::iterator iter=imgKeyFrame->point_clouds.begin(),iter_end=imgKeyFrame->point_clouds.end();iter!=iter_end;iter++){
//        cout<<"imgKeyFrame->point_clouds single xyz=";
        for(int i=0;i<3;i++){
            outputStream.write(reinterpret_cast<char*>(&(*iter)(i)), sizeof(double));
//            cout<<(*iter)(i)<<" ";
        }
//        cout<<endl;

    }
    
//    cout<<"imgKeyFrame->measurements_origin size="<<imgKeyFrame->measurements_origin.size()<<endl;
    for(std::vector<cv::Point2f>::iterator iter=imgKeyFrame->measurements_origin.begin(),iter_end=imgKeyFrame->measurements_origin.end();iter!=iter_end;iter++){
    //        cout<<"imgKeyFrame->point_clouds single xyz=";
            
        outputStream.write(reinterpret_cast<char*>(&((*iter).x)), sizeof(float));
        outputStream.write(reinterpret_cast<char*>(&((*iter).y)), sizeof(float));
    }
    
    
    //发位姿
        Eigen::Vector3d T_w_i;
        Eigen::Matrix3d R_w_i;
        imgKeyFrame->getPose_2Server(T_w_i, R_w_i);
    //    cout<<"pose T: ";
        for(int i=0;i<3;i++){
            outputStream.write(reinterpret_cast<char*>(&T_w_i[i]), sizeof(double));
//            cout<<T_w_i[i]<<" ";
        }
//        cout<<endl;
    //
    //    cout<<"pose R: ";
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                outputStream.write(reinterpret_cast<char*>(&R_w_i(i,j)), sizeof(double));
    //            cout<<R_w_i(i,j)<<" ";
            }
        }
    //    cout<<endl;
    
    
    //发原始位姿
        imgKeyFrame->getOriginPose(T_w_i, R_w_i);
//        cout<<"origin pose T: 验证有没有乘以偏移";
        for(int i=0;i<3;i++){
            outputStream.write(reinterpret_cast<char*>(&T_w_i[i]), sizeof(double));
//            cout<<T_w_i[i]<<" ";
        }
//        cout<<endl;
        
        
    //    cout<<"origin pose R: ";
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                outputStream.write(reinterpret_cast<char*>(&R_w_i(i,j)), sizeof(double));
    //            cout<<R_w_i(i,j)<<" ";
            }
        }
    //    cout<<endl;
    
//    发送滑动窗口内总约束
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->edge), sizeof(int));
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->edge_single), sizeof(int));
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->var_imu), sizeof(double));
    std::cout<<"帧"<<imgKeyFrame->global_index<<" ,"<<imgKeyFrame->edge<<" ,"<<imgKeyFrame->edge_single<<" ,"<<imgKeyFrame->var_imu<<std::endl;
    //发全局id 其实就是keyframeDatabase里面的index
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->global_index), sizeof(int));
//        cout<<"imgKeyFrame->global_index"<<imgKeyFrame->global_index<<endl;
        //序列号
        outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->segment_index), sizeof(int));
    //    cout<<"segment_index "<<imgKeyFrame->segment_index<<endl;
        
        //header有什么用 回环后，窗口内算重定位误差 用来判断匹配帧是不是还在窗口内
        outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->header), sizeof(double));
    //    cout<<"header "<<imgKeyFrame->header<<endl;
        
       
        
        
        //features_id 回环检测是会有外点移除
        int feature_len=imgKeyFrame->features_id.size();
        outputStream.write(reinterpret_cast<char*>(&feature_len), sizeof(int));
//        cout<<"feature_len="<<feature_len<<endl;
    //    cout<<"features_id=";
        for(int i=0;i<feature_len;i++){
            outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->features_id[i]), sizeof(int));
    //        cout<<imgKeyFrame->features_id[i]<<" ";
        }
    //    cout<<endl;
    
    
    //发共视的帧id,和权重
    imgKeyFrame->mMutexConnections.lock();
    std::vector<KeyFrame*> vpOrderedConnectedKeyFrames= imgKeyFrame->mvpOrderedConnectedKeyFrames;
    std::vector<int> vOrderedWeights=imgKeyFrame->mvOrderedWeights;
    imgKeyFrame->mMutexConnections.unlock();
    int conn_kfs_len=vpOrderedConnectedKeyFrames.size();
    outputStream.write(reinterpret_cast<char*>(&conn_kfs_len), sizeof(int));
    for(int i=0;i<conn_kfs_len;i++){
        outputStream.write(reinterpret_cast<char*>(&vpOrderedConnectedKeyFrames[i]->global_index), sizeof(int));
        outputStream.write(reinterpret_cast<char*>(&vOrderedWeights[i]), sizeof(int));
    }
    
    
    
    
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
 
    
    size_t output_len=outputStream.str().length();
//    can_send_keys_num=(4096-output_len-26)/24;
    int test=(int)(output_len);
    can_send_keys_num=(4096-test-30)/12;
    
     
   
    std::vector<cv::KeyPoint>::iterator iter=imgKeyFrame->keypoints.begin();
    if(can_send_keys_num>0){
//        cout<<"can_send_keys_num="<<can_send_keys_num<<endl;
//        cout<<"output_len="<<output_len<<endl;
//        cout<<"outputStream.str().length()="<<outputStream.str().length() <<endl;
//        cout<<"(4096-output_len-30)/12="<< (4096-test-30)/12<<endl;
        if(can_send_keys_num>imgKeyFrame->keypoints.size()){
            cout<<"handleData 有问题 keys长度错了 说明还可以发送一些描述符"<<endl;
            can_send_keys_num=(int)(imgKeyFrame->keypoints.size());
           
        }
        
        //这个无论如何都要发，即便算出来不对，因为服务器那边要做对比
        outputStream.write(reinterpret_cast<char*>(&can_send_keys_num), sizeof(int));
        
        
        int i=0;
        for(std::vector<cv::KeyPoint>::iterator iter_end=imgKeyFrame->keypoints.end(); i<can_send_keys_num&&iter!=iter_end; i++,iter++){
//            outputStream.write(reinterpret_cast<char*>(&(*iter).angle), sizeof(float));//永为-1
//            outputStream.write(reinterpret_cast<char*>(&(*iter).octave), sizeof(int));//永为0
            outputStream.write(reinterpret_cast<char*>(&(*iter).pt.x), sizeof(float));
            outputStream.write(reinterpret_cast<char*>(&(*iter).pt.y), sizeof(float));
            outputStream.write(reinterpret_cast<char*>(&(*iter).response), sizeof(float));
//            cout<<"response="<<(*iter).response<<endl;
//            cout<<"class_id="<<(*iter).class_id<<endl;
//            assert((*iter).angle==-1);
//            assert((*iter).octave==0);
//            outputStream.write(reinterpret_cast<char*>(&(*iter).class_id), sizeof(int));//永为-1
//            cout<<"测试特征点的octave是不是一直为1："<<(*iter).octave<<endl;
        }
    }else{
//        cout<<"can_send_keys_num="<<can_send_keys_num<<endl;
//        cout<<"output_len="<<output_len<<endl;
//        cout<<"outputStream.str().length()="<<outputStream.str().length() <<endl;
        cout<<"handleData 有问题 keys长度错了 小于等于0了"<<endl;
//        cout<<"(4096-output_len-30)/12="<<(4096-test-30)/12<<endl;
        //这个无论如何都要发，即便算出来不对，因为服务器那边要做对比
        can_send_keys_num=0;
        outputStream.write(reinterpret_cast<char*>(&can_send_keys_num), sizeof(int));
//        cout<<"can_send_keys_num="<<can_send_keys_num<<endl;
        
    }
    
   
    
    return iter;
 
}

std::vector<cv::KeyPoint>::iterator produceStreamForKF_1_2(stringstream &outputStream,KeyFrame *imgKeyFrame,std::vector<cv::KeyPoint>::iterator iter,int des_num,bool is_end){
    
    
    int des_start=0;
    
    unsigned char saveHeader[2] = {0xe0, 0x90};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
    //发全局id 其实就是keyframeDatabase里面的index
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->global_index), sizeof(int));
    
    //告诉他实际发了多少个
    if(is_end){
//        cout<<"des_num="<<des_num<<endl;
        outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
        outputStream.write(reinterpret_cast<char*>(&des_num), sizeof(int));
    }
    
    //弄清楚一下 这个特征点做匹配 要用的是不是 只是相机平面坐标
        //先发全部特征点、描述 前面是回环用的特征点，后面是窗口特征点
    
        
        for(std::vector<cv::KeyPoint>::iterator iter_end=imgKeyFrame->keypoints.end(); des_start<des_num&&iter!=iter_end; des_start++,iter++){
            
//            outputStream.write(reinterpret_cast<char*>(&(*iter).angle), sizeof(float));
//            outputStream.write(reinterpret_cast<char*>(&(*iter).octave), sizeof(int));
            outputStream.write(reinterpret_cast<char*>(&(*iter).pt.x), sizeof(float));
            outputStream.write(reinterpret_cast<char*>(&(*iter).pt.y), sizeof(float));
            outputStream.write(reinterpret_cast<char*>(&(*iter).response), sizeof(float));
//            outputStream.write(reinterpret_cast<char*>(&(*iter).class_id), sizeof(int));
    //        cout<<"keypoints x="<<(*iter).pt.x<<" y="<<(*iter).pt.y<<endl;
//            cout<<"response="<<(*iter).response<<endl;
//            cout<<"class_id="<<(*iter).class_id<<endl;
//            assert((*iter).angle==-1);
//            assert((*iter).octave==0);
        }
    
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
    return iter;
}

std::vector<BRIEF::bitset>::iterator produceStreamForKF_2(stringstream &outputStream,KeyFrame *imgKeyFrame,int des_num,bool is_end,std::vector<BRIEF::bitset>::iterator iter){
    int des_start=0;
    
    unsigned char saveHeader[2] = {0xe1, 0x92};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    //发全局id 其实就是keyframeDatabase里面的index
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->global_index), sizeof(int));
//    cout<<"imgKeyFrame->global_index"<<imgKeyFrame->global_index<<endl;
    
    //告诉他实际发了多少个
    if(is_end){
        outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
        outputStream.write(reinterpret_cast<char*>(&des_num), sizeof(int));
    }
    
    for(std::vector<BRIEF::bitset>::iterator iter_end=imgKeyFrame->descriptors.end(); des_start<des_num&&iter!=iter_end; des_start++,iter++){
        string str;
        to_string((*iter),str);

        for(auto ite=str.rbegin(),ite_end=str.rend();ite!=ite_end;ite++){
            outputStream.write(reinterpret_cast<char*>(&(*ite)), sizeof(char));

        }

    }
    
   
    return iter;
}

void sendKeyFrame(KeyFrame *imgKeyFrame){
//    cout<<"发送数据 KF_T 11："<<getTime_handle()<<endl;
    int start_key_packIdx=-50,start_des_packIdx=-50;
    int keys_num=0;
//    cout<<"sendKeyFrame ing"<<endl;
        TS(KF_T);
    std::stringstream imgKF_Stream;
    std::vector<cv::KeyPoint>::iterator iter_start=produceStreamForKF_1_1(imgKF_Stream,imgKeyFrame,keys_num);
//    produceStreamForKF(imgKF_Stream,imgKeyFrame);
    size_t len=imgKF_Stream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "KF_T");
    packhead.PackIdx = start_key_packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
//    imgKF_Stream.clear();
    imgKF_Stream.str("");
    start_key_packIdx++;
    start_des_packIdx++;
    
    
    //发送keypoints
    int keys_end=imgKeyFrame->keypoints.size();
    for(;keys_num<=keys_end;){
        if(keys_num!=keys_end){
            int keys_sub=keys_end-keys_num;
            if(keys_sub<=338){//169
//                cout<<"keys_sub="<<keys_sub<<endl;
                iter_start=produceStreamForKF_1_2(imgKF_Stream,imgKeyFrame,iter_start,keys_sub,true);
                keys_num+=keys_sub;
                keys_num++;//这里就是希望它直接跳出去
            }else{
                iter_start=produceStreamForKF_1_2(imgKF_Stream,imgKeyFrame,iter_start,338,false);
                keys_num+=338;
            }
            
            len=imgKF_Stream.str().length();
                
                
            sprintf(packhead.PackName, "%s", "KF_key");
            packhead.PackIdx = start_key_packIdx;
            packhead.PackLen = len;
            
             bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
            memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
            if (len > 0)
                memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
            
            NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
            [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
        //    imgKF_Stream.clear();
            imgKF_Stream.str("");
            start_key_packIdx++;
            
        }else{
            break;
        }
          
        usleep(20);//2000
    }
  
         
    
    //一开始发了7个
    int des_num=0,des_end=imgKeyFrame->descriptors.size();
    std::vector<BRIEF::bitset>::iterator iter_des_start=imgKeyFrame->descriptors.begin();
    for(;des_num<=des_end;){
        
        if(des_num!=des_end){
            int des_sub=des_end-des_num;
            if(des_sub<=15){
                iter_des_start=produceStreamForKF_2(imgKF_Stream,imgKeyFrame,des_sub,true,iter_des_start);
                
                des_num+=des_sub;
            }else{
                iter_des_start=produceStreamForKF_2(imgKF_Stream,imgKeyFrame,15,false,iter_des_start);
                des_num+=15;
            }
            size_t len_des=imgKF_Stream.str().length();
            
            struct PackHead packhead_des;
            sprintf(packhead_des.PackName, "%s", "KF_T_add");
            packhead_des.PackIdx = start_des_packIdx;
            packhead_des.PackLen = len_des;
            
            Byte* bytearr = new Byte[len_des+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
            memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead_des), sizeof(struct PackHead));
            if (len_des > 0)
                memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len_des);
            else{
                cout<<"len_des < 0";
            }
            
            NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead_des.PackLen+sizeof(struct PackHead)];
            [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
//            imgKF_Stream.clear();
            imgKF_Stream.str("");
            start_des_packIdx++;
        }else{
            break;
        }
        
        usleep(20);
//        msleep(1);
//        struct timespec req = {0}, rem = {0};
//        time_t sec = (int)(5 / 1000);
//        unsigned long milisec = 5 - (sec * 1000);
//        req.tv_sec = sec;            //秒
//        req.tv_nsec = milisec * 1000000L;    //纳秒
//        nanosleep(&req, &req);
    }
    
    
    
    
    imgKF_Stream.clear();
//    packIdx++;
//    cout<<"发送数据 KF_T 22："<<getTime_handle()<<endl;
//    cout<<"packLen="<<len<<endl;
    TE(KF_T);
}


//-------------第三篇论文 发送描述符
std::vector<cv::KeyPoint>::iterator produceStreamForKF_1_1_compress(stringstream &outputStream,KeyFrame *imgKeyFrame,int &can_send_keys_num){
    unsigned char saveHeader[2] = {0xe9, 0x97};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    //先告诉它窗口特征点数量
    int winPointNum=imgKeyFrame->getWinPointsSize();
//    cout<<"winPointNum size="<<winPointNum<<endl;
    outputStream.write(reinterpret_cast<char*>(&winPointNum), sizeof(int));
   
    //特征点的数量
    int pointNum=imgKeyFrame->keypoints.size();
//    cout<<"pointNum size="<<pointNum<<endl;
    outputStream.write(reinterpret_cast<char*>(&pointNum), sizeof(int));
    
    //pointClouds
//    cout<<"imgKeyFrame->point_clouds size="<<imgKeyFrame->point_clouds.size()<<endl;
    for(std::vector<Eigen::Vector3d>::iterator iter=imgKeyFrame->point_clouds.begin(),iter_end=imgKeyFrame->point_clouds.end();iter!=iter_end;iter++){
//        cout<<"imgKeyFrame->point_clouds single xyz=";
        for(int i=0;i<3;i++){
            outputStream.write(reinterpret_cast<char*>(&(*iter)(i)), sizeof(double));
//            cout<<(*iter)(i)<<" ";
        }
//        cout<<endl;

    }
    
//    cout<<"imgKeyFrame->measurements_origin size="<<imgKeyFrame->measurements_origin.size()<<endl;
    for(std::vector<cv::Point2f>::iterator iter=imgKeyFrame->measurements_origin.begin(),iter_end=imgKeyFrame->measurements_origin.end();iter!=iter_end;iter++){
    //        cout<<"imgKeyFrame->point_clouds single xyz=";
            
        outputStream.write(reinterpret_cast<char*>(&((*iter).x)), sizeof(float));
        outputStream.write(reinterpret_cast<char*>(&((*iter).y)), sizeof(float));
    }
    
    
    //发位姿
        Eigen::Vector3d T_w_i;
        Eigen::Matrix3d R_w_i;
        imgKeyFrame->getPose_2Server(T_w_i, R_w_i);
    //    cout<<"pose T: ";
        for(int i=0;i<3;i++){
            outputStream.write(reinterpret_cast<char*>(&T_w_i[i]), sizeof(double));
//            cout<<T_w_i[i]<<" ";
        }
//        cout<<endl;
    //
    //    cout<<"pose R: ";
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                outputStream.write(reinterpret_cast<char*>(&R_w_i(i,j)), sizeof(double));
    //            cout<<R_w_i(i,j)<<" ";
            }
        }
    //    cout<<endl;
    
    
    //发原始位姿
        imgKeyFrame->getOriginPose(T_w_i, R_w_i);
//        cout<<"origin pose T: 验证有没有乘以偏移";
        for(int i=0;i<3;i++){
            outputStream.write(reinterpret_cast<char*>(&T_w_i[i]), sizeof(double));
//            cout<<T_w_i[i]<<" ";
        }
//        cout<<endl;
        
        
    //    cout<<"origin pose R: ";
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                outputStream.write(reinterpret_cast<char*>(&R_w_i(i,j)), sizeof(double));
    //            cout<<R_w_i(i,j)<<" ";
            }
        }
    //    cout<<endl;
    
//    发送滑动窗口内总约束
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->edge), sizeof(int));
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->edge_single), sizeof(int));
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->var_imu), sizeof(double));
//    std::cout<<"帧"<<imgKeyFrame->global_index<<" ,"<<imgKeyFrame->edge<<" ,"<<imgKeyFrame->edge_single<<" ,"<<imgKeyFrame->var_imu<<std::endl;
    //发全局id 其实就是keyframeDatabase里面的index
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->global_index), sizeof(int));
//        cout<<"imgKeyFrame->global_index"<<imgKeyFrame->global_index<<endl;
        //序列号
        outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->segment_index), sizeof(int));
    //    cout<<"segment_index "<<imgKeyFrame->segment_index<<endl;
        
        //header有什么用 回环后，窗口内算重定位误差 用来判断匹配帧是不是还在窗口内
        outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->header), sizeof(double));
    //    cout<<"header "<<imgKeyFrame->header<<endl;
        
       
        
        
        //features_id 回环检测是会有外点移除
        int feature_len=imgKeyFrame->features_id.size();
        outputStream.write(reinterpret_cast<char*>(&feature_len), sizeof(int));
//        cout<<"feature_len="<<feature_len<<endl;
    //    cout<<"features_id=";
        for(int i=0;i<feature_len;i++){
            outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->features_id[i]), sizeof(int));
    //        cout<<imgKeyFrame->features_id[i]<<" ";
        }
    //    cout<<endl;
    
    
    //发共视的帧id,和权重
    imgKeyFrame->mMutexConnections.lock();
    std::vector<KeyFrame*> vpOrderedConnectedKeyFrames= imgKeyFrame->mvpOrderedConnectedKeyFrames;
    std::vector<int> vOrderedWeights=imgKeyFrame->mvOrderedWeights;
    imgKeyFrame->mMutexConnections.unlock();
    int conn_kfs_len=vpOrderedConnectedKeyFrames.size();
    outputStream.write(reinterpret_cast<char*>(&conn_kfs_len), sizeof(int));
    for(int i=0;i<conn_kfs_len;i++){
        outputStream.write(reinterpret_cast<char*>(&vpOrderedConnectedKeyFrames[i]->global_index), sizeof(int));
        outputStream.write(reinterpret_cast<char*>(&vOrderedWeights[i]), sizeof(int));
    }
    
    
    
    
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
 
    
    size_t output_len=outputStream.str().length();
//    can_send_keys_num=(4096-output_len-26)/24;
    int test=(int)(output_len);
    can_send_keys_num=(4096-test-30)/12;
    
     
   
    std::vector<cv::KeyPoint>::iterator iter=imgKeyFrame->keypoints.begin();
    if(can_send_keys_num>0){
//        cout<<"can_send_keys_num="<<can_send_keys_num<<endl;
//        cout<<"output_len="<<output_len<<endl;
//        cout<<"outputStream.str().length()="<<outputStream.str().length() <<endl;
//        cout<<"(4096-output_len-30)/12="<< (4096-test-30)/12<<endl;
        if(can_send_keys_num>imgKeyFrame->keypoints.size()){
            cout<<"handleData 有问题 keys长度错了 说明还可以发送一些描述符"<<endl;
            can_send_keys_num=(int)(imgKeyFrame->keypoints.size());
           
        }
        
        //这个无论如何都要发，即便算出来不对，因为服务器那边要做对比
        outputStream.write(reinterpret_cast<char*>(&can_send_keys_num), sizeof(int));
        
        
        int i=0;
        for(std::vector<cv::KeyPoint>::iterator iter_end=imgKeyFrame->keypoints.end(); i<can_send_keys_num&&iter!=iter_end; i++,iter++){
//            outputStream.write(reinterpret_cast<char*>(&(*iter).angle), sizeof(float));//永为-1
//            outputStream.write(reinterpret_cast<char*>(&(*iter).octave), sizeof(int));//永为0
            outputStream.write(reinterpret_cast<char*>(&(*iter).pt.x), sizeof(float));
            outputStream.write(reinterpret_cast<char*>(&(*iter).pt.y), sizeof(float));
            outputStream.write(reinterpret_cast<char*>(&(*iter).response), sizeof(float));
//            cout<<"response="<<(*iter).response<<endl;
//            cout<<"class_id="<<(*iter).class_id<<endl;
//            assert((*iter).angle==-1);
//            assert((*iter).octave==0);
//            outputStream.write(reinterpret_cast<char*>(&(*iter).class_id), sizeof(int));//永为-1
//            cout<<"测试特征点的octave是不是一直为1："<<(*iter).octave<<endl;
        }
    }else{
//        cout<<"can_send_keys_num="<<can_send_keys_num<<endl;
//        cout<<"output_len="<<output_len<<endl;
//        cout<<"outputStream.str().length()="<<outputStream.str().length() <<endl;
        cout<<"handleData 有问题 keys长度错了 小于等于0了"<<endl;
//        cout<<"(4096-output_len-30)/12="<<(4096-test-30)/12<<endl;
        //这个无论如何都要发，即便算出来不对，因为服务器那边要做对比
        can_send_keys_num=0;
        outputStream.write(reinterpret_cast<char*>(&can_send_keys_num), sizeof(int));
//        cout<<"can_send_keys_num="<<can_send_keys_num<<endl;
        
    }
    
   
    
    return iter;
 
}

std::vector<cv::KeyPoint>::iterator produceStreamForKF_1_2_compress(stringstream &outputStream,KeyFrame *imgKeyFrame,std::vector<cv::KeyPoint>::iterator iter,int des_num,bool is_end){
    
    
    int des_start=0;
    
    unsigned char saveHeader[2] = {0xe0, 0x90};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
    //发全局id 其实就是keyframeDatabase里面的index
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->global_index), sizeof(int));
    
    //告诉他实际发了多少个
    if(is_end){
//        cout<<"des_num="<<des_num<<endl;
        outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
        outputStream.write(reinterpret_cast<char*>(&des_num), sizeof(int));
    }
    
    //弄清楚一下 这个特征点做匹配 要用的是不是 只是相机平面坐标
        //先发全部特征点、描述 前面是回环用的特征点，后面是窗口特征点
    
        
        for(std::vector<cv::KeyPoint>::iterator iter_end=imgKeyFrame->keypoints.end(); des_start<des_num&&iter!=iter_end; des_start++,iter++){
            
//            outputStream.write(reinterpret_cast<char*>(&(*iter).angle), sizeof(float));
//            outputStream.write(reinterpret_cast<char*>(&(*iter).octave), sizeof(int));
            outputStream.write(reinterpret_cast<char*>(&(*iter).pt.x), sizeof(float));
            outputStream.write(reinterpret_cast<char*>(&(*iter).pt.y), sizeof(float));
            outputStream.write(reinterpret_cast<char*>(&(*iter).response), sizeof(float));
//            outputStream.write(reinterpret_cast<char*>(&(*iter).class_id), sizeof(int));
    //        cout<<"keypoints x="<<(*iter).pt.x<<" y="<<(*iter).pt.y<<endl;
//            cout<<"response="<<(*iter).response<<endl;
//            cout<<"class_id="<<(*iter).class_id<<endl;
//            assert((*iter).angle==-1);
//            assert((*iter).octave==0);
        }
    
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
    return iter;
}

void produceStreamForKF_2_compress(stringstream &outputStream,int des_sub,bool is_end,int desIndex_num,KeyFrame *imgKeyFrame ,vector<uchar> &bitstream){
    int des_start=0;
    
    unsigned char saveHeader[2] = {0xe1, 0x92};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    //发全局id 其实就是keyframeDatabase里面的index
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->global_index), sizeof(int));
//    cout<<"imgKeyFrame->global_index"<<imgKeyFrame->global_index<<endl;
    
    //告诉他实际发了多少个 因为一次性发完 ，暂时不做验证了
    if(is_end){
        outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
        outputStream.write(reinterpret_cast<char*>(&des_sub), sizeof(int));
    }

    for(int i=desIndex_num,j=desIndex_num+des_sub;i<j;i++){
//    for(auto iter_compress=bitstream.begin(),iter_end_compress=bitstream.end();iter_compress!=iter_end_compress;iter_compress++){
        outputStream.write(reinterpret_cast<char*>(&(bitstream[i])), sizeof(uchar));
    }
    
}
#include <bitset>
void produceStreamForKF_2_1_BowIndexcompress(stringstream &outputStream,int des_num,bool is_end,int desIndex_num,vector<int> &visualWords,KeyFrame *imgKeyFrame){
    int des_start=0;
    
    unsigned char saveHeader[2] = {0xe8, 0x94};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    //发全局id 其实就是keyframeDatabase里面的index
    outputStream.write(reinterpret_cast<char*>(&(imgKeyFrame->global_index)), sizeof(int));
 
    //告诉他实际发了多少个
    if(is_end){
        outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
        outputStream.write(reinterpret_cast<char*>(&des_num), sizeof(int));
        int bitNum=imgKeyFrame->bitstream.size();
        outputStream.write(reinterpret_cast<char*>(&bitNum), sizeof(int));
    }

//    分发
//    cout<<"发送的下标="<<des_num<<" ";
    std::bitset<1636 * 20> bits;//des_num是发送index数量
    int flag=0;
    for(int bowIndex_start=desIndex_num,bowIndex_end=desIndex_num+des_num;bowIndex_start<bowIndex_end;bowIndex_start++){
        int bowIndex_single=visualWords[bowIndex_start];
//        转成20位
        bits |= std::bitset<bits.size()>(bowIndex_single) << flag * 20;
        flag++;
        
//        cout<<bowIndex_single<<" , ";
    }
//    cout<<endl<<endl;
    
    uchar output_buffer[(des_num *20+8-1)/8]; // 3个字节的输出缓冲区，保留20位二进制数
//    cout<<(des_num *20+8-1)/8<<" ,"<<bits.size() <<endl;
    const size_t output_buffer_size = sizeof(output_buffer) / sizeof(*output_buffer);
    for(int i = 0; i < output_buffer_size; i++) {

        std::bitset<8> bits_single;
        // 将原bitset的末8位复制到新的bitset中
        for(int j = 0; j < 8; j++){
            bits_single[j] = bits[j ];
        }
        output_buffer[i] = static_cast<uchar>(bits_single.to_ulong()); // 每次输出8位二进制数到对应字节中
        bits= bits >> 8;
    }


    for(int i=0;i<output_buffer_size;i++){
        outputStream.write(reinterpret_cast<char*>(&output_buffer[i]), sizeof(uchar));
    }
    
    
    
    
//    std::bitset<arr_size *20> binary_num;
//    const size_t binary_num_size =binary_num.size();
//    for(int i = 0; i < (arr_size *20+8-1)/8; i++) {
//       binary_num |= (static_cast<std::bitset<binary_num_size>>(output_buffer[i])) << (i * 8); // 每次读取8位二进制数，并通过位移操作组合成20位二进制数
//   }
    
    
    
//  不拆分
//    std::bitset<20> bits(bowIndex_single);
//    uchar output_buffer[3]; // 3个字节的输出缓冲区，保留20位二进制数
//    for(int i = 0; i < 3; i++) {
//        std::bitset<8> bits_single(bits.to_ulong());
//        output_buffer[i] = static_cast<uchar>(bits_single.to_ulong()); // 每次输出8位二进制数到对应字节中
//        bits= bits >> 8;
//    }
//
//    std::bitset<20> binary_num(0);
//    for(int i = 0; i < 3; i++) {
//       binary_num |= (static_cast<std::bitset<20>>(output_buffer[i])) << (i * 8); // 每次读取8位二进制数，并通过位移操作组合成20位二进制数
//    }
     
}

void sendKeyFrame_compress(KeyFrame *imgKeyFrame){
//    cout<<"发送数据 KF_T 11："<<getTime_handle()<<endl;
    int start_key_packIdx=-50,start_des_packIdx=-50,kf_des_ComIndex=-50,kf_des_compress=-50;
    int keys_num=0;
//    cout<<"sendKeyFrame ing"<<endl;
        TS(KF_T);
    std::stringstream imgKF_Stream;
    std::vector<cv::KeyPoint>::iterator iter_start=produceStreamForKF_1_1_compress(imgKF_Stream,imgKeyFrame,keys_num);
//    produceStreamForKF(imgKF_Stream,imgKeyFrame);
    size_t len=imgKF_Stream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "KF_T");
    packhead.PackIdx = start_key_packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
//    imgKF_Stream.clear();
    imgKF_Stream.str("");
    start_key_packIdx++;
    start_des_packIdx++;
    kf_des_ComIndex++;
    kf_des_compress++;
    
    
    //发送keypoints
    int keys_end=imgKeyFrame->keypoints.size();
    for(;keys_num<=keys_end;){
        if(keys_num!=keys_end){
            int keys_sub=keys_end-keys_num;
            if(keys_sub<=338){//169
//                cout<<"keys_sub="<<keys_sub<<endl;
                iter_start=produceStreamForKF_1_2_compress(imgKF_Stream,imgKeyFrame,iter_start,keys_sub,true);
                keys_num+=keys_sub;
                keys_num++;//这里就是希望它直接跳出去
            }else{
                iter_start=produceStreamForKF_1_2_compress(imgKF_Stream,imgKeyFrame,iter_start,338,false);
                keys_num+=338;
            }
            
            len=imgKF_Stream.str().length();
                
                
            sprintf(packhead.PackName, "%s", "KF_key");
            packhead.PackIdx = start_key_packIdx;
            packhead.PackLen = len;
            
             bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
            memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
            if (len > 0)
                memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
            
            NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
            [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
        //    imgKF_Stream.clear();
            imgKF_Stream.str("");
            start_key_packIdx++;
            
        }else{
            break;
        }
          
        usleep(20);//2000
    }
  
         
    /**
    //一开始发了7个
    int des_num=0,des_end=imgKeyFrame->descriptors.size();
    std::vector<BRIEF::bitset>::iterator iter_des_start=imgKeyFrame->descriptors.begin();
    for(;des_num<=des_end;){
        
        if(des_num!=des_end){
            int des_sub=des_end-des_num;
            if(des_sub<=15){
                iter_des_start=produceStreamForKF_2_compress(imgKF_Stream,imgKeyFrame,des_sub,true,iter_des_start,bitstream);
                
                des_num+=des_sub;
            }else{
                iter_des_start=produceStreamForKF_2_compress(imgKF_Stream,imgKeyFrame,15,false,iter_des_start,bitstream);
                des_num+=15;
            }
            size_t len_des=imgKF_Stream.str().length();
            
            struct PackHead packhead_des;
            sprintf(packhead_des.PackName, "%s", "KF_T_add");
            packhead_des.PackIdx = start_des_packIdx;
            packhead_des.PackLen = len_des;
            
            Byte* bytearr = new Byte[len_des+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
            memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead_des), sizeof(struct PackHead));
            if (len_des > 0)
                memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len_des);
            else{
                cout<<"len_des < 0";
            }
            
            NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead_des.PackLen+sizeof(struct PackHead)];
            [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
//            imgKF_Stream.clear();
            imgKF_Stream.str("");
            start_des_packIdx++;
        }else{
            break;
        }
        
        usleep(20);
//        msleep(1);
//        struct timespec req = {0}, rem = {0};
//        time_t sec = (int)(5 / 1000);
//        unsigned long milisec = 5 - (sec * 1000);
//        req.tv_sec = sec;            //秒
//        req.tv_nsec = milisec * 1000000L;    //纳秒
//        nanosleep(&req, &req);
    }
     */
    
    
    
//    先发一下参考下标
    std::vector< int> visualWords=imgKeyFrame->visualWords;
//    int desIndex_num=0,des_end=visualWords.size();//大小应该是 特征点数量*20
    
    for(int desIndex_num=0,des_end=visualWords.size();desIndex_num<des_end;){
            int des_sub=des_end-desIndex_num;
            if(des_sub<=1636){
                produceStreamForKF_2_1_BowIndexcompress(imgKF_Stream,des_sub,true,desIndex_num,visualWords,imgKeyFrame);
                
                desIndex_num+=des_sub;
            }else{
                produceStreamForKF_2_1_BowIndexcompress(imgKF_Stream,1636,false,desIndex_num,visualWords,imgKeyFrame);
                desIndex_num+=1636;
            }
            size_t len_des=imgKF_Stream.str().length();
            
            struct PackHead packhead_des;
            sprintf(packhead_des.PackName, "%s", "KF_bowI");
            packhead_des.PackIdx = kf_des_ComIndex;
            packhead_des.PackLen = len_des;
            
            Byte* bytearr = new Byte[len_des+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
            memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead_des), sizeof(struct PackHead));
            if (len_des > 0)
                memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len_des);
            else{
                cout<<"len_des < 0";
            }
//        cout<<"len_des="<<len_des<<" , "<<des_sub<<" , "<<sizeof(struct PackHead)<<endl;
        
            NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead_des.PackLen+sizeof(struct PackHead)];
            [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
//            imgKF_Stream.clear();
            imgKF_Stream.str("");
           
        
        kf_des_ComIndex++;
        
        usleep(20);
    }
    
    
    vector<uchar> bitstream=imgKeyFrame->bitstream;
    for(int desIndex_num=0,des_end=bitstream.size();desIndex_num<des_end;){
        int des_sub=des_end-desIndex_num;
        if(des_sub<=4090){

            produceStreamForKF_2_compress(imgKF_Stream,des_sub,true,desIndex_num,imgKeyFrame,bitstream);
            desIndex_num+=des_sub;
        }else{

            produceStreamForKF_2_compress(imgKF_Stream,4090,false,desIndex_num,imgKeyFrame,bitstream);
            desIndex_num+=4090;
        }

   
        size_t len_des=imgKF_Stream.str().length();
        
        struct PackHead packhead_des;
        sprintf(packhead_des.PackName, "%s", "KF_bowC");
        packhead_des.PackIdx = kf_des_compress;
        packhead_des.PackLen = len_des;
        
        Byte* bytearr = new Byte[len_des+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
        memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead_des), sizeof(struct PackHead));
        if (len_des > 0)
            memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len_des);
        else{
            cout<<"len_des < 0";
        }
        
        NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead_des.PackLen+sizeof(struct PackHead)];
        [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
    //            imgKF_Stream.clear();
        imgKF_Stream.str("");
        kf_des_compress++;
        
        imgKF_Stream.clear();
    }
//    packIdx++;
    TE(KF_T);
}


//暂时不用
void produceStreamForKF_Pose(stringstream &outputStream,KeyFrame *imgKeyFrame){
    unsigned char saveHeader[2] = {0xe5, 0x93};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    //先告诉它窗口特征点数量
    int winPointNum=imgKeyFrame->getWinPointsSize();
    outputStream.write(reinterpret_cast<char*>(&winPointNum), sizeof(int));

        
    //发位姿
    Eigen::Vector3d T_w_i;
    Eigen::Matrix3d R_w_i;
    imgKeyFrame->getPose(T_w_i, R_w_i);
    for(int i=0;i<3;i++){
        outputStream.write(reinterpret_cast<char*>(&T_w_i[i]), sizeof(double));
    }
    
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)
        outputStream.write(reinterpret_cast<char*>(&R_w_i(i,j)), sizeof(double));
    }
    
    //发全局id 其实就是keyframeDatabase里面的index
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->global_index), sizeof(int));
    cout<<"imgKeyFrame->global_index"<<imgKeyFrame->global_index<<endl;
    //序列号
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->segment_index), sizeof(int));
    
    //header有什么用 回环后，窗口内算重定位误差 用来判断匹配帧是不是还在窗口内
    outputStream.write(reinterpret_cast<char*>(&imgKeyFrame->header), sizeof(double));
        
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
}
void sendKeyFrame_Pose(KeyFrame *imgKeyFrame){
    
    cout<<"sendKeyFrame_Pose ing"<<endl;
    std::stringstream imgKF_Stream;
    produceStreamForKF_Pose(imgKF_Stream,imgKeyFrame);
    size_t len=imgKF_Stream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "KF_11");
    packhead.PackIdx = packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
    imgKF_Stream.clear();
    packIdx++;
}



void produceStreamForKFOriginPose(stringstream &outputStream,KeyFrame *imgKeyFrame){
    unsigned char saveHeader[2] = {0xe8, 0x96};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    //先告诉它是哪个关键帧
    int global_index=imgKeyFrame->global_index;
    outputStream.write(reinterpret_cast<char*>(&global_index), sizeof(int));

    //发位姿
    Eigen::Vector3d T_w_i;
    Eigen::Matrix3d R_w_i;
    imgKeyFrame->getOriginPose(T_w_i, R_w_i);
    for(int i=0;i<3;i++){
        outputStream.write(reinterpret_cast<char*>(&T_w_i[i]), sizeof(double));
    }
    
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)
        outputStream.write(reinterpret_cast<char*>(&R_w_i(i,j)), sizeof(double));
    }
    
        outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
}
void sendKeyFrameOriginPose(KeyFrame *imgKeyFrame){
//        cout<<"发送数据 KF_Origin 11："<<getTime_handle()<<endl;
//    cout<<"sendKeyFrameOriginPose ing"<<endl;
    TS(KF_Origin);
    std::stringstream imgKF_Stream;
    produceStreamForKFOriginPose(imgKF_Stream,imgKeyFrame);
    size_t len=imgKF_Stream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "KF_Origin");
    packhead.PackIdx = packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
    imgKF_Stream.clear();
    packIdx++;
    
//    cout<<"发送数据 KF_Origin 22："<<getTime_handle()<<endl;
    TE(KF_Origin);
}

void produceStreamForErrorLoop(stringstream &outputStream,int curKF_Loop_Index){
    unsigned char saveHeader[2] = {0xe7, 0x95};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
    //错误的回环匹配 当前帧的/匹配帧的下标
    outputStream.write(reinterpret_cast<char*>(&curKF_Loop_Index), sizeof(int));
}
void sendErrorLoop(int curKF_Loop_Index){
    TS(ErrorLoop);
//    cout<<"发送数据 ErrorLoop 11："<<getTime_handle()<<endl;
//    cout<<"sendErrorLoop ing"<<endl;
    std::stringstream imgKF_Stream;
    produceStreamForErrorLoop(imgKF_Stream,curKF_Loop_Index);
    size_t len=imgKF_Stream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "ErrorLoop");
    packhead.PackIdx = packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
    imgKF_Stream.clear();
    packIdx++;
//    cout<<"发送数据 ErrorLoop 22："<<getTime_handle()<<endl;
        TE(ErrorLoop);
}


void produceStreamForCorrectLoopIndex(stringstream &outputStream,int curKF_CorrectLoop_Index){
    unsigned char saveHeader[2] = {0xe6, 0x94};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
    
    outputStream.write(reinterpret_cast<char*>(&curKF_CorrectLoop_Index), sizeof(int));
}
void sendCorrectLoopIndex(int curKF_CorrectLoop_Index){
        TS(CorrIndex);
//    cout<<"发送数据 CorrIndex 11："<<getTime_handle()<<endl;
//    cout<<"sendCorrectLoopIndex ing"<<endl;
    std::stringstream imgKF_Stream;
    produceStreamForCorrectLoopIndex(imgKF_Stream,curKF_CorrectLoop_Index);
    size_t len=imgKF_Stream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "CorrIndex");
    packhead.PackIdx = packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
    imgKF_Stream.clear();
    packIdx++;
//    cout<<"发送数据 CorrIndex 22："<<getTime_handle()<<endl;
    TE(CorrIndex);
}



 void produceStreamForFrontPoseRelative(stringstream &outputStream,Eigen::Vector3d relative_t,double relative_yaw,int relative_cur_index){
    unsigned char saveHeader[2] = {0xe4, 0x92};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
    
     double t_test;
     for(int i=0;i<3;i++){
         t_test=relative_t[i];
         outputStream.write(reinterpret_cast<char*>(&t_test), sizeof(double));
     }
     
//    outputStream.write(reinterpret_cast<char*>(&relative_q.x()), sizeof(double));
//     outputStream.write(reinterpret_cast<char*>(&relative_q.y()), sizeof(double));
//     outputStream.write(reinterpret_cast<char*>(&relative_q.z()), sizeof(double));
//     outputStream.write(reinterpret_cast<char*>(&relative_q.w()), sizeof(double));
     
     if(relative_cur_index==0)
         assert(false);
    
    outputStream.write(reinterpret_cast<char*>(&relative_yaw), sizeof(double));
     
     outputStream.write(reinterpret_cast<char*>(&relative_cur_index), sizeof(int));
     
     outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
}

 void sendFrontPoseRelative(Eigen::Vector3d relative_t,double relative_yaw,int relative_cur_index){
     TS(FrontPose);
//     cout<<"发送数据 FrontPose 11："<<getTime_handle()<<endl;
//    cout<<"sendFrontPoseRelative ing"<<endl;
    std::stringstream imgKF_Stream;
    produceStreamForFrontPoseRelative(imgKF_Stream,relative_t,relative_yaw,relative_cur_index);
    size_t len=imgKF_Stream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "FrontPose");
    packhead.PackIdx = packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
    imgKF_Stream.clear();
    packIdx++;
//     cout<<"发送数据 FrontPose 22："<<getTime_handle()<<endl;
     TE(FrontPose);
}


void produceStreamForFrontPoseRelative_add_pitch_roll(stringstream &outputStream,Eigen::Vector3d relative_t,double relative_yaw,int relative_cur_index,double relative_pitch,double relative_roll){
    unsigned char saveHeader[2] = {0xe3, 0x91};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);

     double t_test;
     for(int i=0;i<3;i++){
         t_test=relative_t[i];
         outputStream.write(reinterpret_cast<char*>(&t_test), sizeof(double));
     }
     
    if(relative_cur_index==0)
        assert(false);
    
    outputStream.write(reinterpret_cast<char*>(&relative_yaw), sizeof(double));
     
     outputStream.write(reinterpret_cast<char*>(&relative_cur_index), sizeof(int));
    
    outputStream.write(reinterpret_cast<char*>(&relative_pitch), sizeof(double));
    outputStream.write(reinterpret_cast<char*>(&relative_roll), sizeof(double));
     
     outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
     
//     Eigen::Matrix<double, 4, 1> connected_info_test;
//     connected_info_test <<relative_t.x(), relative_t.y(), relative_t.z(),relative_yaw;
//     cout<<"handleData relative_t_sendServer:"<<connected_info_test(0)<<" "<<connected_info_test(1)<<" "<<connected_info_test(2)<<" "<<connected_info_test(3)<<" cur_index"<<relative_cur_index<<endl;
}

    
 void sendFrontPoseRelative_add_pitch_roll(Eigen::Vector3d relative_t,double relative_yaw,int relative_cur_index,double relative_pitch,double relative_roll){
     TS(FrontPose2);
//     cout<<"发送数据 FrontPose 22："<<getTime_handle()<<endl;
    
    std::stringstream imgKF_Stream;
    produceStreamForFrontPoseRelative_add_pitch_roll(imgKF_Stream,relative_t,relative_yaw,relative_cur_index,relative_pitch,relative_roll);
    size_t len=imgKF_Stream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "FrontPos2");
    packhead.PackIdx = packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
    imgKF_Stream.clear();
    packIdx++;
//     cout<<"发送数据 FrontPose 22："<<getTime_handle()<<endl;
     TE(FrontPose2);
}


void produceStreamForFrontPoseRelative_add_pitch_roll_forRemove(stringstream &outputStream,Eigen::Vector3d relative_t,double relative_yaw,int relative_cur_index,double relative_pitch,double relative_roll){
    unsigned char saveHeader[2] = {0xe8, 0x96};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);

     double t_test;
     for(int i=0;i<3;i++){
         t_test=relative_t[i];
         outputStream.write(reinterpret_cast<char*>(&t_test), sizeof(double));
     }
     
    if(relative_cur_index==0)
        assert(false);

    outputStream.write(reinterpret_cast<char*>(&relative_yaw), sizeof(double));
     
     outputStream.write(reinterpret_cast<char*>(&relative_cur_index), sizeof(int));
    
    outputStream.write(reinterpret_cast<char*>(&relative_pitch), sizeof(double));
    outputStream.write(reinterpret_cast<char*>(&relative_roll), sizeof(double));
     
     outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
     
}

//这些是需要减少权重的
 void sendFrontPoseRelative_add_pitch_roll_forRemove(Eigen::Vector3d relative_t,double relative_yaw,int relative_cur_index,double relative_pitch,double relative_roll){
     TS(FrontPose2);
//     cout<<"发送数据 FrontPose 33："<<getTime_handle()<<endl;
    
    std::stringstream imgKF_Stream;
    produceStreamForFrontPoseRelative_add_pitch_roll_forRemove(imgKF_Stream,relative_t,relative_yaw,relative_cur_index,relative_pitch,relative_roll);
    size_t len=imgKF_Stream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "FrontPos3");
    packhead.PackIdx = packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
    imgKF_Stream.clear();
    packIdx++;
//     cout<<"发送数据 FrontPose 33："<<getTime_handle()<<endl;
     TE(FrontPose2);
}


void receiveGlobalData(const char* buffer,long unsigned int offset,KeyFrameDatabase *kfbd){
    unsigned char saveHeader[2] =  {0xcb, 0x89};
    unsigned char hd[2];

    memcpy(hd, buffer+offset, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error globalData, shouldn't"<<endl;
    
    vector<Eigen::Vector3d> t_global;
//    vector<double> yaw_global;
    vector<Eigen::Matrix3d> r_global;
    
    Eigen::Vector3d t_single_global;
//    double yaw_single_global;
    Eigen::Matrix3d r_single_global;
    
    t_global.clear();
//    yaw_global.clear();
    r_global.clear();
     
    int ts_len;
    memcpy(&ts_len, buffer+offset, sizeof(int));
    offset += sizeof(int);

    for(int i=0;i<ts_len;i++){
        for(int i=0;i<3;i++){
            memcpy(&t_single_global(i), buffer+offset, sizeof(double));
            offset += sizeof(double);
        }
        t_global.push_back(t_single_global);
    }
    for(int i=0;i<ts_len;i++){
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                memcpy(&r_single_global(i,j), buffer+offset, sizeof(double));
                offset += sizeof(double);
            }
        }
        r_global.push_back(r_single_global);
    }
//    kfbd->yaw_global=yaw_global;
    
    

    
    //回环帧、匹配帧id
    int loopKF_index,curKF_loop_index;
    memcpy(&loopKF_index, buffer+offset, sizeof(int));
    offset+=sizeof(int);
    memcpy(&curKF_loop_index, buffer+offset, sizeof(int));
    offset+=sizeof(int);
    
    
    int earliest_loop_index;
    memcpy(&earliest_loop_index, buffer+offset, sizeof(int));
    offset+=sizeof(int);
    
    
    //测试用的 needresample值为0的帧
    int special_kf_len;
    memcpy(&special_kf_len, buffer+offset, sizeof(int));
    offset+=sizeof(int);
    
    cout<<"special_kf_len 988= "<<special_kf_len<<endl;
    
    int special_kf_id;
    vector<int> vspecial_kfs;
    for(int i=0;i<special_kf_len; i++){
        memcpy(&special_kf_id, buffer+offset, sizeof(int));
        offset+=sizeof(int);
        vspecial_kfs.push_back(special_kf_id);
    }
    //    kfbd->special_kf_inOpti.clear();
    kfbd->special_kf_intra_mutex.lock();
    kfbd->loopKF_index.push(loopKF_index);
    kfbd->curKF_loop_index.push(curKF_loop_index);
    kfbd->earliest_queue.push(earliest_loop_index);
    kfbd->t_global.push(t_global);
    kfbd->r_global.push(r_global);
    kfbd->special_kf_inOpti_intra.push(vspecial_kfs);
    kfbd->special_kf_intra_mutex.unlock();
    if((t_global.size()-1)!=(r_global.size()-1) || (t_global.size()-1)!=vspecial_kfs.size()){
        cout<<"接收的rt大小为："<<t_global.size()<<" , "<<r_global.size()<<" , "<<vspecial_kfs.size()<<endl;
        assert(false);
    }
    
    unsigned char end[5];
    memcpy(end, buffer+offset, 5);
    offset+=5;
    for(int i=0;i<5;i++){
        if(end[i]!=saveEnd[i]){
            cout<<"end error globalData, should't"<<endl;
        }
    }
}

void receiveLoopData(const char* buffer,long unsigned int offset,VINS *vins){
    unsigned char saveHeader[2] =  {0xca, 0x88};
    unsigned char hd[2];

    memcpy(hd, buffer+offset, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error loopData, shouldn't"<<endl;
    
    RetriveData retrive_data;
    
    //header
//    double header;
//    memcpy(&header, buffer+offset, sizeof(double));
//    offset += sizeof(double);
//    retrive_data.header=header;
    
    //measurements
    vector<cv::Point2f> measurements;
    measurements.clear();
    int measurements_len;
    memcpy(&measurements_len, buffer+offset, sizeof(int));
    offset += sizeof(int);
//    cout<<"测试 全局优化："<<measurements_len<<" "<<header<<endl;
    for(int i=0;i<measurements_len;i++){
        cv::Point2f point;
        float x_y;
        memcpy(&x_y, buffer+offset, sizeof(float));
        offset += sizeof(float);
        point.x=x_y;
        memcpy(&x_y, buffer+offset, sizeof(float));
        offset += sizeof(float);
        point.y=x_y;
        measurements.push_back(point);
    }
    retrive_data.measurements=measurements;

    //features_ids
    vector<int> features_ids;
    features_ids.clear();
    int features_len;
    memcpy(&features_len, buffer+offset, sizeof(int));
    offset += sizeof(int);

    for(int i=0;i<features_len;i++){
        int feature_id;
        memcpy(&feature_id, buffer+offset, sizeof(int));
        offset += sizeof(int);
        features_ids.push_back(feature_id);
    }
    retrive_data.features_ids=features_ids;
    
    int cur_index;
    memcpy(&cur_index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    retrive_data.cur_index=cur_index;
    
//    Eigen::Vector3d p_old;
//    double p_old_123;
//    for(int i=0;i<3;i++){
//        memcpy(&p_old_123, buffer+offset, sizeof(double));
//        offset += sizeof(double);
//        p_old(i)=p_old_123;
//    }
//    retrive_data.P_old=p_old;
//
//
//    double x,y,z,w;
//    memcpy(&x, buffer+offset, sizeof(double));
//    offset += sizeof(double);
//    memcpy(&y, buffer+offset, sizeof(double));
//    offset += sizeof(double);
//    memcpy(&z, buffer+offset, sizeof(double));
//    offset += sizeof(double);
//    memcpy(&w, buffer+offset, sizeof(double));
//    offset += sizeof(double);
//    Eigen::Quaterniond q_old(w,x,y,z);//实部在前
//    retrive_data.Q_old=q_old;
    
    
    retrive_data.use=true;
    
    int old_index;
    memcpy(&old_index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    retrive_data.old_index=old_index;
    
//    cout<<"HandleData cur_index"<<cur_index<<" old_index"<<old_index<<endl;
    
    
    vins->retrive_pose_data_server.push(retrive_data);
            
//    cout<<"HandleData retrive_pose_data_server push "<<vins->retrive_pose_data_server.back().cur_index<<" "<<vins->retrive_pose_data_server.back().old_index<<endl;
//    
//    cout<<"HandleData test front "<<vins->retrive_pose_data_server.front().cur_index<<"  "<<vins->retrive_pose_data_server.front().old_index<<endl;
    
    unsigned char end[5];
    memcpy(end, buffer+offset, 5);
    offset+=5;
    for(int i=0;i<5;i++){
        if(end[i]!=saveEnd[i]){
            cout<<"end error loopData, should't"<<endl;
        }
    }

}


void receiveLoopData_another(const char* buffer,long unsigned int offset,VINS *vins){
    unsigned char saveHeader[2] =  {0xcb, 0x68};
    unsigned char hd[2];

    memcpy(hd, buffer+offset, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error loopData, shouldn't"<<endl;
    
    RetriveData retrive_data;
    
    
    int cur_index;
    memcpy(&cur_index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    retrive_data.cur_index=cur_index;
 
    
    retrive_data.use=true;
    
    int old_index;
    memcpy(&old_index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    retrive_data.old_index=old_index;
    
//    cout<<"HandleData cur_index"<<cur_index<<" old_index"<<old_index<<endl;
    
    
    vins->retrive_pose_data_server.push(retrive_data);
            
//    cout<<"HandleData retrive_pose_data_server push "<<vins->retrive_pose_data_server.back().cur_index<<" "<<vins->retrive_pose_data_server.back().old_index<<endl;
//
//    cout<<"HandleData test front "<<vins->retrive_pose_data_server.front().cur_index<<"  "<<vins->retrive_pose_data_server.front().old_index<<endl;
    
    unsigned char end[5];
    memcpy(end, buffer+offset, 5);
    offset+=5;
    for(int i=0;i<5;i++){
        if(end[i]!=saveEnd[i]){
            cout<<"end error loopData_another, should't"<<endl;
        }
    }

}

void receiveLoopData_another2(const char* buffer,long unsigned int offset,VINS *vins,double (&loop_pose_forFeatureTracker)[7]){
    unsigned char saveHeader[2] =  {0xcb, 0x68};
    unsigned char hd[2];

    memcpy(hd, buffer+offset, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error loopData, shouldn't"<<endl;
    
    RetriveData retrive_data;
    
    
    int cur_index;
    memcpy(&cur_index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    retrive_data.cur_index=cur_index;
 
    
    retrive_data.use=true;
    
    int old_index;
    memcpy(&old_index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    retrive_data.old_index=old_index;
    
//    cout<<"HandleData cur_index"<<cur_index<<" old_index"<<old_index<<endl;
    
//    double  loop_pose[7];
    double p_old_123;
    for(int i=0;i<7;i++){
        memcpy(&p_old_123, buffer+offset, sizeof(double));
        offset += sizeof(double);
        retrive_data.loop_pose[i]=p_old_123;
        loop_pose_forFeatureTracker[i]=p_old_123;
    }
//    retrive_data.loop_pose=loop_pose;

    double header;
    memcpy(&header, buffer+offset, sizeof(double));
    offset += sizeof(double);
    retrive_data.header=header;
    
    //features_ids_all
    vector<vector<int> > features_ids_all;
    vector<int> features_ids_single;
    features_ids_all.clear();
    features_ids_single.clear();
    int features_len;
    memcpy(&features_len, buffer+offset, sizeof(int));
    offset += sizeof(int);

    for(int i=0;i<features_len;i++){
        int feature_single_len;
        memcpy(&feature_single_len, buffer+offset, sizeof(int));
        offset += sizeof(int);
        for(int j=0;j<feature_single_len;j++){
            int feature_id ;
            memcpy(&feature_id, buffer+offset, sizeof(int));
            offset += sizeof(int);
            features_ids_single.push_back(feature_id);
        }
        features_ids_all.push_back(features_ids_single);
    }
    retrive_data.features_ids_all=features_ids_all;
    
    
    //point_clouds_all
    vector<vector<Eigen::Vector3d>> point_clouds_all;
    vector<Eigen::Vector3d> point_clouds_single;
    point_clouds_all.clear();
    point_clouds_single.clear();
    int point_clouds_len;
    memcpy(&point_clouds_len, buffer+offset, sizeof(int));
    offset += sizeof(int);

    for(int i=0;i<point_clouds_len;i++){
        int point_clouds_single_len;
        memcpy(&point_clouds_single_len, buffer+offset, sizeof(int));
        offset += sizeof(int);
        for(int j=0;j<point_clouds_single_len;j++){
            double point_clouds_x,point_clouds_y,point_clouds_z ;
            memcpy(&point_clouds_x, buffer+offset, sizeof(double));
            offset += sizeof(double);
            memcpy(&point_clouds_y, buffer+offset, sizeof(double));
            offset += sizeof(double);
            memcpy(&point_clouds_z, buffer+offset, sizeof(double));
            offset += sizeof(double);
            Eigen::Vector3d p(point_clouds_x,point_clouds_y,point_clouds_z);
            point_clouds_single.push_back(p);
        }
        point_clouds_all.push_back(point_clouds_single);
    }
    retrive_data.point_clouds_all=point_clouds_all;
  
    
    vins->retrive_pose_data_server.push(retrive_data);
            
//    cout<<"HandleData retrive_pose_data_server push "<<vins->retrive_pose_data_server.back().cur_index<<" "<<vins->retrive_pose_data_server.back().old_index<<endl;
//
//    cout<<"HandleData test front "<<vins->retrive_pose_data_server.front().cur_index<<"  "<<vins->retrive_pose_data_server.front().old_index<<endl;
    
    unsigned char end[5];
    memcpy(end, buffer+offset, 5);
    offset+=5;
    for(int i=0;i<5;i++){
        if(end[i]!=saveEnd[i]){
            cout<<"end error loopData_another, should't"<<endl;
        }
    }

}


void receiveRejectWithF(const char* buffer,long unsigned int offset,VINS *vins){
    unsigned char saveHeader[2] =  {0xc9, 0x87};
   unsigned char hd[2];

   memcpy(hd, buffer+offset, 2);
   offset += 2;
   if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
       cerr<<"header error receiveRejectWithF, shouldn't"<<endl;
    
   
    int cur_reject_index;
    memcpy(&cur_reject_index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    
    
    int status_index;
    memcpy(&status_index, buffer+offset, sizeof(int));
    offset += sizeof(int);
    
    vector<uchar> status;
    uchar status_single;
    for(int i=0;i<status_index;i++){
        memcpy(&status_single, buffer+offset, sizeof(uchar));
        offset += sizeof(uchar);
        status.push_back(status_single);
    }
    
    vins->send_status_index.push(cur_reject_index);
    vins->send_status.push(status);
    
    
    unsigned char end[5];
    memcpy(end, buffer+offset, 5);
    offset+=5;
    for(int i=0;i<5;i++){
        if(end[i]!=saveEnd[i]){
            cout<<"end error receiveRejectWithF, should't"<<endl;
        }
    }

}


void receiveAr(const char* buffer,long unsigned int offset,VINS *vins){
    unsigned char saveHeader[2] =  {0xc8, 0x88};
    unsigned char hd[2];

    memcpy(hd, buffer+offset, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveAr, shouldn't"<<endl;
     
    
    Eigen::Vector3f ori;
    for(int i=0;i<3;i++){
        memcpy(&ori[i], buffer+offset, sizeof(float));
        offset += sizeof(float);
    }
   
    Eigen::Vector3f cox;
       for(int i=0;i<3;i++){
           memcpy(&cox[i], buffer+offset, sizeof(float));
           offset += sizeof(float);
       }
       
       Eigen::Vector3f coy;
       for(int i=0;i<3;i++){
           memcpy(&coy[i], buffer+offset, sizeof(float));
           offset += sizeof(float);
       }
       
       Eigen::Vector3f coz;
       for(int i=0;i<3;i++){
           memcpy(&coz[i], buffer+offset, sizeof(float));
           offset += sizeof(float);
       }
       
       float size;
       memcpy(&size, buffer+offset, sizeof(float));
       offset += sizeof(float);
    
     unsigned char end[5];
     memcpy(end, buffer+offset, 5);
     offset+=5;
     for(int i=0;i<5;i++){
         if(end[i]!=saveEnd[i]){
             cout<<"end error receiveAr, should't"<<endl;
         }
     }
    
    GroundPoint gp_ar(vins->drawresult.Ground_idx++);
    gp_ar.ori=ori;
    gp_ar.cox=cox;
    gp_ar.coy=coy;
    gp_ar.coz=coz;
    gp_ar.size=size;
    gp_ar.boxflag=true;
    vins->drawresult.Grounds.push_back(gp_ar);
}

void produceStreamForAR(stringstream &outputStream,Eigen::Vector3f ori, Eigen::Vector3f cox, Eigen::Vector3f coy, Eigen::Vector3f coz, float size){
    unsigned char saveHeader[2] = {0xe0, 0x98};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);

     float ori_test;
     for(int i=0;i<3;i++){
         ori_test=ori[i];
         outputStream.write(reinterpret_cast<char*>(&ori_test), sizeof(float));
     }
    
    for(int i=0;i<3;i++){
        ori_test=cox[i];
        outputStream.write(reinterpret_cast<char*>(&ori_test), sizeof(float));
    }
    
    for(int i=0;i<3;i++){
        ori_test=coy[i];
        outputStream.write(reinterpret_cast<char*>(&ori_test), sizeof(float));
    }
    
    for(int i=0;i<3;i++){
        ori_test=coz[i];
        outputStream.write(reinterpret_cast<char*>(&ori_test), sizeof(float));
    }
    
    outputStream.write(reinterpret_cast<char*>(&size), sizeof(float));
    
     outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
}


void sendAR(Eigen::Vector3f ori, Eigen::Vector3f cox, Eigen::Vector3f coy, Eigen::Vector3f coz, float size){
    TS(sendAR);
    
    std::stringstream imgKF_Stream;
    produceStreamForAR(imgKF_Stream,ori,cox,coy,coz,size);
    size_t len=imgKF_Stream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "Ar");
    packhead.PackIdx = packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
    imgKF_Stream.clear();
    packIdx++;
    
     TE(sendAR);
}

void produceStreamForOurAR(stringstream &outputStream,Eigen::Vector3f ori, Eigen::Vector3f cox, Eigen::Vector3f coy, Eigen::Vector3f coz, float size){
    unsigned char saveHeader[2] = {0xe0, 0x98};
    outputStream.write(reinterpret_cast<char*>(saveHeader), 2);

     float ori_test;
     for(int i=0;i<3;i++){
         ori_test=ori[i];
         outputStream.write(reinterpret_cast<char*>(&ori_test), sizeof(float));
     }
    
    for(int i=0;i<3;i++){
        ori_test=cox[i];
        outputStream.write(reinterpret_cast<char*>(&ori_test), sizeof(float));
    }
    
    for(int i=0;i<3;i++){
        ori_test=coy[i];
        outputStream.write(reinterpret_cast<char*>(&ori_test), sizeof(float));
    }
    
    for(int i=0;i<3;i++){
        ori_test=coz[i];
        outputStream.write(reinterpret_cast<char*>(&ori_test), sizeof(float));
    }
    
    outputStream.write(reinterpret_cast<char*>(&size), sizeof(float));
    
     outputStream.write(reinterpret_cast<char*>(saveHeader), 2);
}


void sendOurAR(Eigen::Vector3f ori, Eigen::Vector3f cox, Eigen::Vector3f coy, Eigen::Vector3f coz, float size){
    TS(sendAR);
    
    std::stringstream imgKF_Stream;
    produceStreamForAR(imgKF_Stream,ori,cox,coy,coz,size);
    size_t len=imgKF_Stream.str().length();
    
    struct PackHead packhead;
    sprintf(packhead.PackName, "%s", "Ar");
    packhead.PackIdx = packIdx;
    packhead.PackLen = len;
    
    Byte* bytearr = new Byte[len+sizeof(struct PackHead)];//sizeof(struct PackHead) is 30
    memcpy((char*)bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
    if (len > 0)
        memcpy((char*)bytearr+sizeof(struct PackHead), imgKF_Stream.str().c_str(), len);
    
    NSData   *dataStream = [NSData dataWithBytes:bytearr length:packhead.PackLen+sizeof(struct PackHead)];
    [[SocketSingleton sharedInstance].socket writeData:dataStream withTimeout:-1 tag:1];
    imgKF_Stream.clear();
    packIdx++;
    
     TE(sendAR);
}



void receiveGlobalData_multiClient(const char* buffer,long unsigned int offset,KeyFrameDatabase *kfbd){
    unsigned char saveHeader[2] =  {0xc1, 0x86};
    unsigned char hd[2];

    memcpy(hd, buffer+offset, 2);
    offset += 2;
    if(hd[0]!=saveHeader[0] || hd[1]!=saveHeader[1])
        cerr<<"header error receiveGlobalData_multiClient, shouldn't"<<endl;
    
    vector<Eigen::Vector3d> t_global;
    vector<Eigen::Matrix3d> r_global;
    
    Eigen::Vector3d t_single_global;
    Eigen::Matrix3d r_single_global;
    
    t_global.clear();
    r_global.clear();
     
    int ts_len;
    memcpy(&ts_len, buffer+offset, sizeof(int));
    offset += sizeof(int);

    for(int i=0;i<ts_len;i++){
        for(int i=0;i<3;i++){
            memcpy(&t_single_global(i), buffer+offset, sizeof(double));
            offset += sizeof(double);
        }
        t_global.push_back(t_single_global);
    }
    for(int i=0;i<ts_len;i++){
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                memcpy(&r_single_global(i,j), buffer+offset, sizeof(double));
                offset += sizeof(double);
            }
        }
        r_global.push_back(r_single_global);
    }
    kfbd->special_kf_intra_mutex.lock();
    kfbd->t_global_multiClient.push(t_global);
    kfbd->r_global_multiClient.push(r_global);
    kfbd->special_kf_intra_mutex.unlock();
        

    
    int kf_len;
    memcpy(&kf_len, buffer+offset, sizeof(int));
    offset += sizeof(int);
    
    vector<int> kf_id_hasComPlace_withOtherMap;
    int kf_id_test;
    for(int i=0;i<kf_len;i++){
        memcpy(&kf_id_test, buffer+offset, sizeof(int));
        offset += sizeof(int);
        kf_id_hasComPlace_withOtherMap.push_back(kf_id_test);
    }
    kfbd->special_kf_intra_mutex.lock();
    kfbd->kf_id_hasComPlace_withOtherMap.push(kf_id_hasComPlace_withOtherMap);
    kfbd->special_kf_intra_mutex.unlock();
    
    
    //全局优化时更新帧 怎么分割帧的里程flag
//    double min_dis_test;
//    int max_frame_num_global_test;
//    memcpy(&max_frame_num_global_test, buffer+offset, sizeof(int));
//    offset+=sizeof(int);
//    memcpy(&min_dis_test, buffer+offset, sizeof(double));
//    offset+=sizeof(double);
//    kfbd->max_frame_num_global=max_frame_num_global_test;
//    kfbd->min_dis=min_dis_test;
        
    //回环帧、匹配帧id
    int loopKF_index,curKF_loop_index;
    memcpy(&loopKF_index, buffer+offset, sizeof(int));
    offset+=sizeof(int);
    memcpy(&curKF_loop_index, buffer+offset, sizeof(int));
    offset+=sizeof(int);
    kfbd->special_kf_intra_mutex.lock();
    kfbd->loopKF_index_multiClient.push(loopKF_index);
    kfbd->curKF_loop_index_multiClient.push(curKF_loop_index);
    kfbd->special_kf_intra_mutex.unlock();
    
    
    //测试用的 needresample值为0的帧
    int special_kf_len;
    memcpy(&special_kf_len, buffer+offset, sizeof(int));
    offset+=sizeof(int);
    
    int special_kf_id;
    vector<int> vspecial_kfs;
    for(int i=0;i<special_kf_len; i++){
        memcpy(&special_kf_id, buffer+offset, sizeof(int));
        offset+=sizeof(int);
        vspecial_kfs.push_back(special_kf_id);
    }
//    kfbd->special_kf_inOpti.clear();
    kfbd->special_kf_intra_mutex.lock();
    kfbd->special_kf_inOpti.push(vspecial_kfs);
    kfbd->special_kf_intra_mutex.unlock();
    
    if((t_global.size()-1)!=(r_global.size()-1) || (t_global.size()-1)!=vspecial_kfs.size()){
        cout<<"接收的rt大小为："<<t_global.size()<<" , "<<r_global.size()<<" , "<<vspecial_kfs.size()<<endl;
        assert(false);
    }
    
    unsigned char end[5];
    memcpy(end, buffer+offset, 5);
    offset+=5;
    for(int i=0;i<5;i++){
        if(end[i]!=saveEnd[i]){
            cout<<"end error receiveGlobalData_multiClient, should't"<<endl;
        }
    }
}
