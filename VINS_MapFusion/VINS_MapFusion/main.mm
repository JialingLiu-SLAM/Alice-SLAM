//
//  main.m
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/25.
//  Copyright © 2020 zx. All rights reserved.
//
#include "Server.hpp"
//#import <Foundation/Foundation.h>

#include <thread>
#include <boost/thread.hpp>
#include "DrawResult.hpp"

#include <vector>

//using namespace cv;

using namespace std;
DrawResult* Server::drawResult=nullptr;
const char *voc_file ="/Users/zhangjianhua/Desktop/VINS_MapFusion/Resources/brief_k10L6.bin";
PoseGraphGlobal* Server::poseGraphGlobal=nullptr;
FeatureMap* Server::global_featureMap=nullptr;
//PoseGraphGlobal* Server::poseGraphGlobal=new PoseGraphGlobal(voc_file, COL, ROW);
//PoseGraphGlobal* Server::poseGraphGlobal=new PoseGraphGlobal(COL, ROW);


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <cassert>
#include <string>

#define CHECK_BIT(var,pos) ((var >> pos) & 1)


#define _USE_MATH_DEFINES
#include "math.h"
#include "Eigen/Dense"
#include <Eigen/SVD>
#include <Eigen/Core>
#include <iomanip>
using namespace  Eigen;
using namespace  std;


#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>


void video2image(string video,string path)
{
    cv::VideoCapture capture(video);
    if(!capture.isOpened())
    {
        cerr<<"Failed to open a video"<<endl;
        return ;
    }
 
    cv::Mat frame;
    int num=1;
    string filename;
    char   temp_file[6];
 
    for(;;)
    {
        capture>>frame;
        if(frame.empty())
            break;
        if(num%24==0){
            sprintf(temp_file,"%06d",num);//每张图片前缀由六个字符构成，不够六个字符的前面用0补完整
            filename = temp_file;
            cout<<temp_file<<endl;
            filename = path+filename+".jpg";
            imwrite(filename,frame);
        }
        
        num++;
        
    }
    capture.release();
}
void mp4_2img(){
//------------视频转图片----------------
    string videoFromfile = "/Volumes/ZX/项目/项目4泳池/莱茵标定测试9.30/4-6/NVR_ch6_main_20220930102900_20220930103300.mp4";  //读取视频
    string Imagespath    = "/Volumes/ZX/项目/项目4泳池/莱茵标定测试9.30/4-6/6/";    //保存图片集路径
    video2image(videoFromfile,Imagespath);
}

void drawPoint(vector<Eigen::Vector3d> v_p){
    //-----------------------图片上画点 start--------------
//    vector<Eigen::Vector3d> v_p;
////    for(int i=0;i<5;i++){
////        Eigen::Vector3d p(i-0.5,i-0.5,19);
////        v_p.push_back(p);
////    }
//
//    Eigen::Vector3d p1(-0.5,-0.5,19),p2(0,0.5,19),p3(0.5,0,19),p4(0.5,0.5,19);
//    v_p.push_back(p1);
//    v_p.push_back(p2);
//    v_p.push_back(p3);
//    v_p.push_back(p4);
    Eigen::Matrix3d k1,k2,r1,r2;
    Eigen::Vector3d t1,t2;
    k1<<3232.67,0,1002.73,0,3251.84,616.14,0,0,1;
    k2<<2134.09,0,1097.13,0,2140.87,700.550,0,0,1;
    r1<<1,0,0,0,1,0,0,0,1;
    r2<<0.53,0.17,-0.83,-0.34,0.94,-0.03,0.77,0.3,0.56;
    t1<<0,0,0;
    t2<<14.88,0.36,-1.35;
    
    
    
//    cv::Mat src1=cv::imread("/Users/zhangjianhua/Downloads/807413133.jpg");
//    cv::Mat src2=cv::imread("/Users/zhangjianhua/Downloads/149512323.jpg");

    cv::Mat src1=cv::imread("/Users/zhangjianhua/Downloads/2-5/test/2/002478.jpg");
    cv::Mat src2=cv::imread("/Users/zhangjianhua/Downloads/2-5/test/5/002478.jpg");

    //读取线段
    

//    cout<<src1.type()<<endl;
//    cout<<CV_8UC3<<" , "<<CV_8UC1<<" , "<<CV_16U<<" , "<<endl;
       //这个是显示成2列
   cv::Mat img_show ( src1.rows, src1.cols*2 , CV_8UC3 );
   src1.copyTo(img_show ( cv::Rect ( 0,0,src1.cols, src1.rows ) ) );
   src2.copyTo(img_show ( cv::Rect ( src1.cols,0,src2.cols , src2.rows ) ) );
//    cout<<src1.rows<<" , "<<src1.cols<<endl;
//    cout<<src2.rows<<" , "<<src2.cols<<endl;
//    cout<<img_show.rows<<" , "<<img_show.cols<<endl;
       cv::Mat img_copy;
    img_copy=img_show.clone();
       for ( int i=0,n=v_p.size();i<n;i++)
       {
//           img_copy=img_show.clone();

           Eigen::Vector3d p=v_p[i];
           Eigen::Vector3d p_img1=k1*(r1*p+t1);
           Eigen::Vector3d p_img2=k2*(r2*p+t2);
           
           Eigen::Vector2d  pixel_prev(p_img1.x()/p_img1.z(),p_img1.y()/p_img1.z());
           Eigen::Vector2d  pixel_now (p_img2.x()/p_img2.z(),p_img2.y()/p_img2.z());

          
           cout<<pixel_prev[0]<<" , "<<pixel_prev[1]<<endl;
//           if ( pixel_now(0,0)<0 || pixel_now(0,0)>=src1.cols || pixel_now(1,0)<0 || pixel_now(1,0)>=src1.rows ){
//   //            num++;
//               continue;
//           }


               float b = 255*float ( rand() ) /RAND_MAX;
               float g = 255*float ( rand() ) /RAND_MAX;
               float r = 255*float ( rand() ) /RAND_MAX;

//           float b = 0;
//           float g = 0;
//           float r = 255;
           //这个是按俩行显示
   //            cv::circle ( img_show, cv::Point2f ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), 8, cv::Scalar ( b,g,r ), 1 );
   //            cv::circle ( img_show, cv::Point2f ( pixel_now ( 0,0 ) , pixel_now ( 1,0 ) + src1.rows ), 8, cv::Scalar ( b,g,r ), 1 );
   //            cv::line ( img_show, cv::Point2f ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 )+ src1.rows ), cv::Scalar ( b,g,r ), 1 );
           //这个是按两列显示
           cv::circle ( img_copy, cv::Point2f ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), 8, cv::Scalar ( b,g,r ), 1 );
           cv::circle ( img_copy, cv::Point2f ( pixel_now ( 0,0 ) + src1.cols , pixel_now ( 1,0 ) ), 8, cv::Scalar ( b,g,r ), 1 );
           cv::line ( img_copy, cv::Point2f ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), cv::Point2f ( pixel_now ( 0,0 )+ src1.cols, pixel_now ( 1,0 )), cv::Scalar ( b,g,r ), 3 );


//               namedWindow("img"+to_string(i),0);
//               cv::imshow("img"+to_string(i),img_copy);
//               waitKey(0);
//           cv::imwrite("/Users/zhangjianhua/Downloads/2-5/test/test/"+to_string(i)+".jpg",img_copy);
       }
    cv::imwrite("/Users/zhangjianhua/Downloads/2-5/test/test/out.jpg",img_copy);
}

void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                                 Vector2d &point0, Vector2d &point1, Vector3d &point_3d)
{
    Matrix4d design_matrix = Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Vector4d triangulated_point;
    triangulated_point =
    design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

vector<string> split(string temp)//按空格分隔字符串
{
    vector<string> str;
    string word;
    stringstream input;//利用字符串流输入
    input << temp;
    while (input >> word)//循环把缓冲区数据读出
    {
        str.push_back(word);
        cout<<word<<" , ";
    }
    return str;
}

//逐行读取2D点 放在一行以0 0结尾
void readTxt2D(string file,vector<Eigen::Vector2f> &measurements_cur,vector<Eigen::Vector2f> &measurements_old)
{
    ifstream infile;
    infile.open(file.data());   //将文件流对象与文件连接起来
    assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行

    string s;
    getline(infile,s);
    //这里做切割
    int num=atoi(s.c_str());
    int i=0;
    
//            cout<<num<<endl;
    
    while (i<num) {
        s="";
        getline(infile,s);
//        cout<<s<<endl;
//        split(s);
        
//        istringstream str(s);
//        string out;
//        str >> out;
//        stringstream ss(out);
        
        stringstream ss;//利用字符串流输入
        ss << s;
        
        float x,y,x2,y2;
        string tmp;
        ss>>tmp;
//        getline(ss, tmp, ' ');
        x=atof(tmp.c_str());
        
        string tmp2;
        ss>>tmp2;
//        getline(ss, tmp, ' ');
        y=atof(tmp2.c_str());
        
        string tmp3;
        ss>>tmp3;
//        getline(ss, tmp, ' ');
        x2=atof(tmp3.c_str());
        
        string tmp4;
        ss>>tmp4;
//        getline(ss, tmp, ' ');
        y2=atof(tmp4.c_str());
        
        cout<<x<<" , "<<y<<", "<<x2<<" , "<<y2<<endl;
//        if(x==0 && y==0){
//            break;
//        }
        measurements_cur.push_back(Eigen::Vector2f(x,y));
        measurements_old.push_back(Eigen::Vector2f(x2,y2));
        i++;
    }
    
    infile.close();             //关闭文件输入流
}

void drawPoint_testMatch(){
    //-----------------------图片上画点 start--------------
//读取图片像素点
    vector<Eigen::Vector2f> measurements_cur,measurements_old;
    readTxt2D("/Users/zhangjianhua/Desktop/relativeCode/data/1324\&565_rt2.txt",measurements_cur,measurements_old);
    
//    readTxt2D("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/pt/199\&1403636959651666432.000000\&253\&1403636710363555584.000000\&0_allPoint.txt",measurements_cur,measurements_old);
    
//    cv::Mat src1=cv::imread("/Users/zhangjianhua/Downloads/807413133.jpg");
//    cv::Mat src2=cv::imread("/Users/zhangjianhua/Downloads/149512323.jpg");

    cv::Mat src1=cv::imread("/Users/zhangjianhua/Desktop/relativeCode/05/image_2/001324.png");
    cv::Mat src2=cv::imread("/Users/zhangjianhua/Desktop/relativeCode/05/image_2/000565.png");
//    cv::Mat src1=cv::imread("/Users/zhangjianhua/Desktop/datasets/MH_02/cam0/data/1403636959651666432.png");
//    cv::Mat src2=cv::imread("/Users/zhangjianhua/Desktop/datasets/MH_01/cam0/data/1403636710363555584.png");

    //读取线段
    // 内参
//    const cv::Mat K = (cv::Mat_<double>(3,3) << 458.654, 0,367.215, 0, 457.296, 248.375, 0,0,1);
//    // 畸变参数
//    const cv::Mat D = (cv::Mat_<double> ( 5,1 ) <<  -0.28340811, 0.07395907, 0.0, 0.00019359, 1.76187114e-05);
//
//    cv::Mat UndistortImage1;
//            cv::undistort(src1, UndistortImage1, K, D, K);
//    cv::Mat UndistortImage2;
//            cv::undistort(src2, UndistortImage2, K, D, K);
 
//    int max_x=UndistortImage1.rows>UndistortImage2.rows?UndistortImage1.rows :UndistortImage2.rows;
//    int max_y=UndistortImage1.cols>UndistortImage2.cols?UndistortImage1.cols :UndistortImage2.cols;
    
    int max_x=src1.rows;
    int max_y=src1.cols;
//    cout<<src1.type()<<endl;
//    cout<<CV_8UC3<<" , "<<CV_8UC1<<" , "<<CV_16U<<" , "<<endl;
       //这个是显示成2列
   cv::Mat img_show ( max_x,max_y*2 , CV_8UC3 );
    src1.copyTo(img_show ( cv::Rect ( 0,0,src1.cols, src1.rows ) ) );
    src2.copyTo(img_show ( cv::Rect ( max_y,0,src2.cols , src2.rows ) ) );
//    cout<<src1.rows<<" , "<<src1.cols<<endl;
//    cout<<src2.rows<<" , "<<src2.cols<<endl;
//    cout<<img_show.rows<<" , "<<img_show.cols<<endl;
       cv::Mat img_copy;
    img_copy=img_show.clone();
       for ( int i=0,n=measurements_cur.size();i<n;i++)
       {
           img_copy=img_show.clone();

           Eigen::Vector2f pixel_prev =measurements_cur[i];
           Eigen::Vector2f pixel_now=measurements_old[i];

          
//           cout<<pixel_prev[0]<<" , "<<pixel_prev[1]<<" , "<<pixel_prev ( 1,0 )<<" , "<<pixel_prev ( 0,0 )<<endl;

           float b = 255*float ( rand() ) /RAND_MAX;
           float g = 255*float ( rand() ) /RAND_MAX;
           float r = 255*float ( rand() ) /RAND_MAX;

           //这个是按两列显示
           cv::circle ( img_copy, cv::Point2f ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), 8, cv::Scalar ( b,g,r ), 1 );
           cv::circle ( img_copy, cv::Point2f ( pixel_now ( 0,0 ) + max_y , pixel_now ( 1,0 ) ), 8, cv::Scalar ( b,g,r ), 1 );
           cv::line ( img_copy, cv::Point2f ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), cv::Point2f ( pixel_now ( 0,0 )+ max_y, pixel_now ( 1,0 )), cv::Scalar ( b,g,r ), 3 );


//               namedWindow("img"+to_string(i),0);
//               cv::imshow("img"+to_string(i),img_copy);
//               waitKey(0);
//           cv::imwrite("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/pt/test3/"+to_string(i)+".jpg",img_copy);
           
           cv::imwrite("/Users/zhangjianhua/Desktop/relativeCode/data/"+to_string(i)+".jpg",img_copy);
       }
//    cv::imwrite("/Users/zhangjianhua/Downloads/2-5/test/test/out.jpg",img_copy);
}

void drawPoint_testMatch(cv::Mat frame1,cv::Mat frame2,vector<cv::KeyPoint> keys1,vector<cv::KeyPoint> keys2){
    //-----------------------图片上画点 start--------------

    int max_x=frame1.rows>frame2.rows?frame1.rows :frame2.rows;
    int max_y=frame1.cols>frame2.cols?frame1.cols :frame2.cols;
//    cout<<src1.type()<<endl;
//    cout<<CV_8UC3<<" , "<<CV_8UC1<<" , "<<CV_16U<<" , "<<endl;
       //这个是显示成2列
   cv::Mat img_show ( max_x,max_y*2 , CV_8UC3 );
    frame1.copyTo(img_show ( cv::Rect ( 0,0,frame1.cols, frame1.rows ) ) );
    frame2.copyTo(img_show ( cv::Rect ( max_y,0,frame2.cols , frame2.rows ) ) );

   cv::Mat img_copy;
    img_copy=img_show.clone();
    int min_num=keys1.size()>keys2.size()?keys2.size():keys1.size();
    int hh=0;
       for ( int i=0,n=min_num;i<n;i++)
       {
           
           hh%=20;
           if(hh==0){
               img_copy=img_show.clone();
           }
           
           cv::Point2f pt=keys2[i].pt;
           

           float b = 255*float ( rand() ) /RAND_MAX;
           float g = 255*float ( rand() ) /RAND_MAX;
           float r = 255*float ( rand() ) /RAND_MAX;

           //这个是按两列显示
           cv::circle ( img_copy, keys1[i].pt, 8, cv::Scalar ( b,g,r ), 1 );
           cv::circle ( img_copy, cv::Point2f ( pt.x + max_y , pt.y ), 8, cv::Scalar ( b,g,r ), 1 );
           cv::line ( img_copy, keys1[i].pt, cv::Point2f ( pt.x + max_y , pt.y ), cv::Scalar ( b,g,r ), 1 );
           if(hh==19){
               cv::imwrite("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/point/"+to_string(i)+".jpg",img_copy);
           }
           hh++;
       }
    cv::imwrite("/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/point/out2.jpg",img_copy);
}
void extractBrief(cv::Mat &image, const std::vector<cv::Point2f> measurements,vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors)
{
    const string BRIEF_PATTERN_FILE="/Users/zhangjianhua/Desktop/hh/VINS_MapFusion/framework/brief_pattern.yml";
    const char *BRIEF_PATTERN_FILE_char=BRIEF_PATTERN_FILE.c_str();
    DVision::BRIEF m_brief;
    const int fast_th = 20; // corner detector response threshold
    cv::FAST(image, keys, fast_th, true);
    for(int i = 0; i < measurements.size(); i++)
    {
        cv::KeyPoint key;
        key.pt = measurements[i];
        keys.push_back(key);
    }
    m_brief.compute(image, keys, descriptors);
}
template <typename T>
void reduceVector4(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
void rejectWithF1( vector<cv::KeyPoint> &keys1,vector<cv::KeyPoint> &measurements_old,std::vector<cv::Point2f> &mea_pt1,std::vector<cv::Point2f> &mea_pt2)
{
    if (measurements_old.size() >= 8)
    {
       
        vector<uchar> status;
        //旧的是不会变的 永远用旧的做匹配
        cv::findFundamentalMat(mea_pt1, mea_pt2, cv::FM_RANSAC, 2.0, 0.99, status);

        reduceVector4(measurements_old, status);
        reduceVector4(keys1, status);
        
        
    }
}
int HammingDis2(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
    BRIEF::bitset xor_of_bitset = a ^ b;
    int dis = xor_of_bitset.count();
    return dis;
}

void searchByDes1( vector<cv::KeyPoint> &keys1,const vector<cv::KeyPoint> &keys2,
                const std::vector<BRIEF::bitset> &descriptors_old1,const std::vector<BRIEF::bitset> &descriptors_old2,
                  std::vector<cv::KeyPoint> &measurements_1,std::vector<cv::KeyPoint> &measurements_2)
{
    std::vector<int> dis_cur_old;
    std::vector<uchar> status;
    std::vector<cv::Point2f> mea_pt1,mea_pt2;
    for(int i = 0; i < descriptors_old1.size(); i++)
    {
        int bestDist = 256;
        int bestIndex = -1;
        int bestDist2 = 256;
        int bestIndex2=-1;
        for(int j = 0; j < descriptors_old2.size(); j++)
        {
            int dis = HammingDis2(descriptors_old1[i], descriptors_old2[j]);
            if(dis < bestDist)
            {
                bestDist2 = bestDist;
                bestIndex2=bestIndex;
                bestDist = dis;
                bestIndex = j;
            }
            else if(dis < bestDist2)
            {
                bestDist2 = dis;
                bestIndex=j;
            }
        }
        if( bestDist<100)
        {
            measurements_1.push_back(keys1[i]);
            measurements_2.push_back(keys2[bestIndex]);
            dis_cur_old.push_back(bestDist);
            mea_pt1.push_back(keys1[bestIndex].pt);
            mea_pt2.push_back(keys2[bestIndex].pt);
            
        }
    }
    rejectWithF1(measurements_1,measurements_2,mea_pt1,mea_pt2);
}

void matchPoint(){
    string path1="/Users/zhangjianhua/Downloads/office1-1_7-package/office1-1/color/1560000083.949196.png";
    string path2="/Users/zhangjianhua/Downloads/office1-1_7-package/office1-6/color/1560244286.276589.png";
    cv::Mat frame1=cv::imread(path1, cv::IMREAD_UNCHANGED);
    cv::Mat frame2=cv::imread(path2, cv::IMREAD_UNCHANGED);
    vector<cv::Point2f> n_pts1,n_pts2;
    int n_max_cnt=150,n_min_cnt=30;
    cv::Mat mask{480,848, CV_8UC1};
    cv::Mat gray1,gray2;
    cv::cvtColor(frame1, gray1, CV_RGBA2GRAY);
    cv::cvtColor(frame2, gray2, CV_RGBA2GRAY);
    cv::Mat img_equa1,img_equa2;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(3);
    clahe->apply(gray1, img_equa1);
    clahe->apply(gray2, img_equa2);
    cv::goodFeaturesToTrack(img_equa1, n_pts1, n_max_cnt, 0.01, n_min_cnt);
    cv::goodFeaturesToTrack(img_equa2, n_pts2, n_max_cnt, 0.01, n_min_cnt);
    vector<cv::KeyPoint> keys1,keys2;
    vector<BRIEF::bitset> descriptors1,descriptors2;
    extractBrief(gray1,n_pts1,keys1,descriptors1);
    extractBrief(gray2,n_pts2,keys2,descriptors2);
//    特征匹配
    cout<<keys1.size()<<" , "<<descriptors1.size()<<" , "<<keys2.size()<<" , "<<descriptors2.size()<<endl;
    std::vector<cv::KeyPoint> measurements_1,measurements_2;
    searchByDes1(keys1,keys2,descriptors1,descriptors2,measurements_1,measurements_2);
    cout<<measurements_1.size()<<" , "<<measurements_2.size()<<endl;
    drawPoint_testMatch(frame1,frame2,measurements_1,measurements_2);
}



#include <iostream>
#include <cstring>
#include <algorithm>
#include <vector>
using namespace std;

const int MAXN = 100;
const int INF = 0x3f3f3f3f;

int n, m; // 二分图中左侧和右侧节点的数量
float w[MAXN][MAXN]; // 边权值
float lx[MAXN], ly[MAXN]; // 左侧和右侧节点的标号
int linky[MAXN]; // 右侧节点匹配的左侧节点
bool visx[MAXN], visy[MAXN]; // 标记左侧和右侧节点是否在交错树上

bool dfs(int x) {
    visx[x] = true;
    for (int y = 0; y < m; y++) {
        if (visy[y]) continue;
        float t = lx[x] + ly[y] - w[x][y];
        if (fabs(t) <= 0.00001) {
            visy[y] = true;
            if (linky[y] == -1 || dfs(linky[y])) {
                linky[y] = x;
                return true;
            }
        }
    }
    return false;
}
float KM() {
    memset(linky, -1, sizeof(linky));
    memset(ly, 0, sizeof(ly));
    for (int i = 0; i < n; i++) {
        lx[i] = -INF;
        for (int j = 0; j < m; j++) {
            lx[i] = max(lx[i], w[i][j]);
        }
    }
    for (int x = 0; x < n; x++) {
        while (true) {
            memset(visx, false, sizeof(visx));
            memset(visy, false, sizeof(visy));
            if (dfs(x)) break;
            float d = INF;
            for (int i = 0; i < n; i++) {
                if (visx[i]) {
                    for (int j = 0; j < m; j++) {
                        if (!visy[j]) {
                            d = min(d, lx[i] + ly[j] - w[i][j]);
                        }
                    }
                }
            }
            if (fabs(d-INF) <= 0.00001) return -1; // 不存在完备匹配
            for (int i = 0; i < n; i++) {
                if (visx[i]) lx[i] -= d;
            }
            for (int j =0; j < m; j++) {
                if (visy[j]) ly[j] += d;
            }
        }
    }
    float res = 0;
    for (int i = 0; i < m; i++) {
        if (linky[i] != -1) {
            res += w[linky[i]][i];
        }
    }
    return res;
}


// 计算欧几里得距离
double euclideanDistance(vector<double> &a, vector<double> &b) {
    double sum = 0;
    for (int i = 0; i < a.size(); i++) {
        sum += pow(a[i] - b[i], 2);
    }
    return sqrt(sum);
}
// 将距离矩阵转换为向量矩阵并归一化
vector<vector<double>> normalize(vector<vector<double>> &matrix) {
    vector<vector<double>> result(matrix.size(), vector<double>(matrix[0].size()));
    double maxDistance = 0;
    for (int i = 0; i < matrix.size(); i++) {
        for (int j = 0; j < matrix[0].size(); j++) {
            maxDistance = max(maxDistance, matrix[i][j]);
        }
    }
    for (int i = 0; i < matrix.size(); i++) {
        for (int j = 0; j < matrix[0].size(); j++) {
            result[i][j] = (maxDistance - matrix[i][j]) / maxDistance;
        }
    }
    return result;
}
// 计算两个距离矩阵之间的距离
double distanceBetweenMatrices(vector<vector<double>> &a, vector<vector<double>> &b) {
    int n = a.size(), m = a[0].size();
    vector<vector<double>> aVector(n, vector<double>(m)), bVector(n, vector<double>(m));
    // 将距离矩阵转换为向量矩阵并归一化
    aVector = normalize(a);
    bVector = normalize(b);
    // 计算两个向量矩阵之间的距离
    double sum = 0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j< n; j++) {
            sum += pow(euclideanDistance(aVector[i], bVector[j]), 2);
        }
    }
    return sqrt(sum);
}

struct Node {
    int code;
    int freq;
    Node *left, *right;

    Node(int code, int freq, Node *left = nullptr, Node *right = nullptr) {
        this->code = code;
        this->freq = freq;
        this->left = left;
        this->right = right;
    }

    ~Node() {
        delete left;
        delete right;
    }
};

struct Compare {
    bool operator()(Node *a, Node *b) {
        return a->freq > b->freq;
    }
};

void encode(Node *root, string code, unordered_map<int, string> &table) {
    if (!root) return;
    if (!root->left && !root->right) {
        table[root->code] = code;
        return;
    }
    encode(root->left, code + "0", table);
    encode(root->right, code + "1", table);
}
/**
string compress(string text, double p, int max_code) {
    // LZW 压缩
    unordered_map<string, int> dict;
    vector<int> compressed;
    int code = 0;
    for (int i = 0; i < 2; i++) {
        string s = string(1, '0' + i);
        dict[s] = code++;
    }
    string curr;
    for (char c : text) {
        string next = curr + c;
        if (dict.count(next)) {
            curr = next;
        } else {
            compressed.push_back(dict[curr]);
            if (code == max_code) {
               dict.clear();
               for (int i = 0; i < 2; i++) {
                   string s = string(1, '0' + i);
                   dict[s] = i;
               }
               code = 2;
               max_code = 256;
                
           }
            dict[next] = code++;
            curr = string(1, c);
            
        }
    }
    if (!curr.empty()) {
        compressed.push_back(dict[curr]);
    }
    // 哈夫曼编码
    priority_queue<Node *, vector<Node *>, Compare> pq;
    unordered_map<int, int> freq;
    for (int code : compressed) {
        freq[code]++;
    }
    for (auto [code, f] : freq) {
        pq.push(new Node(code, f));
    }
    while (pq.size() > 1) {
        Node *a = pq.top(); pq.pop();
        Node *b = pq.top(); pq.pop();
        pq.push(new Node(-1, a->freq + b->freq, a, b));
    }
    Node *root = pq.top();
    unordered_map<int, string> table;
    encode(root, "", table);
    string compressed_bits;
    for (int code : compressed) {
        compressed_bits += table[code];
    }
    delete root;
    // 输出压缩结果
    cout << "原始长度 uchar：" << text.size() << endl;
    cout << "LZW压缩后长度 int：" << compressed.size() << endl;
    cout << "LZW+哈夫曼压缩后长度：" << compressed_bits.size() << endl;
    for(int i=0;i<compressed_bits.size();i++){
        cout<<compressed_bits[i]<<" , ";

    }
    return compressed_bits;
}

void encode(Node *node, string prefix, unordered_map<int, string> &table) {
    if (!node->left && !node->right) {
        table[node->code] = prefix;
        return;
    }
    if (node->left) {
        encode(node->left, prefix + "0", table);
    }
    if (node->right) {
        encode(node->right, prefix + "1", table);
    }
}

 */
vector<int> compress(string text) {
    // 统计每个字符出现的次数
    unordered_map<char, int> freq;
    for (char c : text) {
        freq[c]++;
    }
    // 构建 Huffman 树
    priority_queue<Node *, vector<Node *>, Compare> pq;
    for (auto [c, f] : freq) {
        Node *node = new Node(c, f); pq.push(node);
        
    }
    while (pq.size() > 1) {
        Node *left = pq.top(); pq.pop(); Node *right = pq.top(); pq.pop();
        Node *node = new Node(-1, left->freq + right->freq, left, right);
        pq.push(node);
        
    }
    Node *root = pq.top(); pq.pop();
    // 构建编码表
    unordered_map<int, string> table; encode(root, "", table); delete root;
    // 压缩输入字符串
    vector<int> compressed;
    for (char c : text) {
        compressed.push_back(stoi(table[c], nullptr, 2));
        
    }
    return compressed;
    
}

/**
class ArithmeticEncoder {
public:
    ArithmeticEncoder() : low_(0), high_(0x7FFFFFFF), num_bits_(0) {}

       void encode(int symbol, int freq_total, const unordered_map<int, int> &freq_table) {
           int freq_range = high_ - low_ + 1;
           int freq_cumulative = 0;
           for (auto [s, f] : freq_table) {
               if (s < symbol) {
                   freq_cumulative += f;
               }
           }
           int freq_symbol = freq_table.at(symbol);
           low_ += freq_cumulative * freq_range / freq_total;
           high_ = low_ + freq_symbol * freq_range / freq_total - 1;
           while (true) {
               if ((low_ ^ high_) >= 0x80000000) {
                   // 输出最高位
                   output_bit((low_ >> 31) & 1);
                   // 输出后面的 0
                   while (num_bits_ > 0) {
                       output_bit(~low_ >> 31);
                       num_bits_--;
                   }
               } else if ((low_ & 0x40000000) && !(high_ & 0x40000000)) {
                   // 如果范围跨越了 1/2，则输出最高位，并输出后面的 0 或 1
                   num_bits_++;
                   low_ &= 0x3FFFFFFF;
                   high_ |= 0x40000000;
               } else {
                   break;
               }
           }
       }

       void finish() {
           // 输出最后一位
           num_bits_++;
           if (low_ < 0x40000000) {
               output_bit(0);
               while (num_bits_ > 0) {
                   output_bit(1);
                   num_bits_--;
               }
           } else {
               output_bit(1);
               while (num_bits_ > 0) {
                   output_bit(0);
                   num_bits_--;
               }
           }
       }
        vector<char> get_output() const {
            return output_;
        }
private:
    void output_bit(bool bit) {
        if (num_bits_ == 8) {
            output_.push_back(static_cast<char>(buffer_));
            buffer_ = 0;
            num_bits_ = 0;
        }
        buffer_ = (buffer_ << 1) | bit;
        num_bits_++;
    }

    int low_;
    int high_;
    int num_bits_;
    int buffer_;
    vector<char> output_;
};


vector<int> compress(string text) {
    // 统计每个字符出现的次数
    unordered_map<int, int> freq_table;
    for (char c : text) {
        freq_table[c]++;
    }
    int freq_total = text.length();
    // 编码输入字符串
    ArithmeticEncoder encoder;
    for (char c : text) {
        encoder.encode(c, freq_total, freq_table);
    }
    encoder.finish();
    // 返回压缩结果
    vector<char> output = encoder.get_output();
    vector<int> compressed;
    for (char c : output) {
        compressed.push_back(static_cast<int>(c));
    }
    return compressed;
}

//        string decompress(vector compressed) { // 统计每个字符出现的次数 unordered_map<int, int> freq_table; for (int code : compressed) { freq_table[code]++; } int freq_total = compressed.size(); // 构建频率累积表 unordered_map<int, int> freq_cumulative; int cumul = 0; for (auto [code, freq] : freq_table) { freq_cumulative[code] = cumul; cumul += freq; } // 解码压缩序列 int low = 0; int high = 0x7FFFFFFF; int range = high - low + 1; int code =

 */


#include <cstdio>
#include<iostream>
#include<cstring>
#include<algorithm>
//以下是文件读入输出需要的头文件
#include<fstream>
#include<cstdlib>
#include<streambuf>
#include <string>
using namespace std;
const int maxn=30;
int i,j;
struct node
{
    long number;
    char name[4];
    int score[5];
    int sum;
} pp[maxn];
bool cmp(node a,node b)
{
    if(a.sum>b.sum)
        return true;
    else if(a.sum==b.sum&&a.number<b.number)
        return true;
    return false;
}
void write_to_excel(bool &flag,vector<int> &keys_sum,vector<int> &bits_nature,vector<int> &bits_encode,vector<int> &diff)
{
    ofstream opt;
    opt.open("/Users/zhangjianhua/Desktop/2022paper/mh03_4_2.csv",ios::out|ios::trunc);
   
    long keys=0,nature=0,encode=0,d=0;
    for(i=0; i<keys_sum.size(); i++){
        opt<<keys_sum[i]<<",";
        keys+=keys_sum[i];
    }
    opt<<keys/keys_sum.size()<<endl;
    
    for(i=0; i<bits_nature.size(); i++){
        opt<<bits_nature[i]<<",";
        nature+=bits_nature[i];
    }
    opt<<nature/bits_nature.size()<<endl;
    
    for(i=0; i<bits_encode.size(); i++){
        opt<<bits_encode[i]<<",";
        encode+=bits_encode[i];
    }
    opt<<encode/bits_encode.size()<<endl;
    
    for(i=0; i<diff.size(); i++){
        opt<<diff[i]<<",";
        d+=diff[i];
    }
    opt<<d/diff.size()<<endl;
    
    opt.close();
    flag=1;
}

void writeTime_to_excel(bool &flag,vector<double> &keys_sum,vector<double> &bits_nature)
{
    ofstream opt;
    opt.open("/Users/zhangjianhua/Desktop/2022paper/mh03_4_2.csv",ios::out|ios::trunc);
   
    double keys=0,nature=0;
//    for(i=0; i<keys_sum.size(); i++){
//        opt<<keys_sum[i]<<",";
//        keys+=keys_sum[i];
//    }
//    opt<<keys*1.0/keys_sum.size()<<endl;
    
    for(i=0; i<bits_nature.size(); i++){
        opt<<bits_nature[i]*1.0/keys_sum[i]<<",";
        nature+=(bits_nature[i]*1.0/keys_sum[i]);
    }
    opt<<nature*1.0/bits_nature.size()<<endl;
    
    opt.close();
    flag=1;
}

using namespace std;
typedef long long ll;


long long head = pow(10, 9);
long long second = pow(10, 8);

void encode(vector<int> encode_context) {
    string original = "";//原文
    string code = "";//编码
    vector<int> fre;
    int total = 256;//总共可能出现的字符
    ll underflow = -1;
    ll undernum = 0;//下溢位数
    //初始化Low High
    ll l = 0;
    ll h = 9999999999;
    //读进原文
     
    original += "#";
    //初始化概率表
    for (int i = 0; i <= 255; i++) {
        fre.push_back(1);
    }
    //开始处理原文
    for (int i = 0; i < encode_context.size(); i++) {
        int pos = encode_context[i];
        ll range = (h - l) + 1;
        ll num = 0;
        for (int j = 0; j < pos; j++) {
            num += fre[j];
        }
        h = l + range * (num + fre[pos]) / (total + 1) - 1;
        l = l + range * num / (total + 1);
        total++;
        fre[pos]++;
        //接下来开始移位
        //开始移位
        while ((l / head) == (h / head)) {
            ll d = l / head;
            code = code + char(l / head + '0');
            l = (l  % head) * 10;
            h = (h   % head) * 10 + 9;
            char add = '0';
            if (d == underflow) {///要用长整型！！！！！！
                add = '9';
            }
            while (undernum) {
                undernum--;
                code = code + add;
            }
            underflow = -1;
        }
        //处理下溢
        while (h - l < second) {
            if (underflow == -1)underflow = l / head;
            undernum++;
            int tmp = l / head;
            l = tmp * head + (l%second) * 10;
            tmp = h / head;
            h = tmp * head + (h%second) * 10 + 9;
        }
    }
    //全部遍历完成
    ll m = (l + h) / 2;
    string magic = to_string(m);
    while (magic.length() < 10) {
        magic = magic + "0";
    }
    code = code + magic;
    cout<<"压缩后："<<code<<endl;
    cout<<code.length()<<endl;
    
    // 哈夫曼编码
    priority_queue<Node *, vector<Node *>, Compare> pq;
    unordered_map<int, int> freq;
    for (char code : code) {
        int code_int=code-'0';
        freq[code_int]++;
    }
    for (auto [code, f] : freq) {
        pq.push(new Node(code, f));
    }
    while (pq.size() > 1) {
        Node *a = pq.top(); pq.pop();
        Node *b = pq.top(); pq.pop();
        pq.push(new Node(-1, a->freq + b->freq, a, b));
    }
    Node *root = pq.top();
    unordered_map<int, string> table;
    encode(root, "", table);
    string compressed_bits;
    for (char code : code) {
        int code_int=code-'0';
        compressed_bits += table[code_int];
    }
    delete root;
    // 输出压缩结果
//    cout << "原始长度 uchar：" << text.size() << endl;
//    cout << "LZW压缩后长度 int：" << compressed.size() << endl;
    cout << "LZW+哈夫曼压缩后长度：" << compressed_bits.size() << endl;
//    for(int i=0;i<compressed_bits.size();i++){
//        cout<<compressed_bits[i]<<" , ";
//
//    }
//    return compressed_bits;
    
}


double *
ac_encode_context (int sym,int d_last ,double *p_last)
{
    //要编码的是sym 0-255 共256位
    double *p= new double[2];
    //d_last记录了 在sym 前一个差异位的 下标
    if(d_last!=-1){
        p[0]=p_last[0]+(sym/(256.0-d_last-1))*(p_last[1]-p_last[0]);
        p[1]=p_last[0]+((sym+1)/(256.0-d_last-1))*(p_last[1]-p_last[0]);
        
//        if(sym-1==d_last){
//            //分配给sym大概率0-
//            p[0]=p_last[0];
//            p[1]=p_last[0]+((sym+1)/256.0)*(p_last[1]-p_last[0]);
//        }else{
//            //还是以前等分的概率
//            p[0]=p_last[0]+(sym/256.0)*(p_last[1]-p_last[0]);
//            p[1]=p_last[0]+((sym+1)/256.0)*(p_last[1]-p_last[0]);
//        }
        
    }else{
        //第一个编码
        p[0]=p_last[0]+(sym/256.0)*(p_last[1]-p_last[0]);
        p[1]=p_last[0]+((sym+1)/256.0)*(p_last[1]-p_last[0]);
    }
    
//    double p_high=(sym+1)/255.0,p_low=sym/255.0;
  return p;
}

double* ac_encode_context2 (int sym,double p_last[])
{
//    cout<<" p_last[0]"<<p_last[0]<<" , "<<p_last[1]<<" , "<<sym<<endl;
    //要编码的是sym 0-255 共256位
    double *p= new double[2];
    
        //第一个编码
        p[0]=p_last[0]+(sym/10.0)*(p_last[1]-p_last[0]);
        p[1]=p_last[0]+((sym+1.0)/10.0)*(p_last[1]-p_last[0]);
    cout<<setiosflags(ios::fixed)<<setprecision(15)<<"进来了"<<p[0]<<" , "<<p[1]<<" , "<<sym<<endl;
    
  return p;
}

void IntraEncodeBowResidual(const int residual[], std::vector<unsigned char> &bitstream)
{
    //不同为1 告知了多少残差
    //总共256位，9层 8位。 29位不同，可以用dbow表示 肯定比256要少
    double p_last[2]={0,256};
    int d_last=-1;
    
//    cout.width(16);

    for( int d = 0; d < 256; d++ )
    {
//        cout<<*(residual+d);
        if(*(residual+d)==1){
            double *p=ac_encode_context(d,d_last,p_last);
            p_last[0]=p[0];
            p_last[1]=p[1];
            delete p;
            d_last=d;
//            cout<<setiosflags(ios::fixed)<<setprecision(150)<<"p_last:"<<p_last[0]<<" , "<<p_last[1]<<endl;
        }
    }
    cout<<setiosflags(ios::fixed)<<setprecision(15)<<"最终的p_last:"<<p_last[0]*1000000000000000000<<" , "<<p_last[1]*1000000000000000000<<endl;
    
    //----------------方案二
    vector<int> encode_context;
    for( int d = 0; d < 256; d++ )
    {
//        cout<<*(residual+d);
        if(*(residual+d)==1){
            encode_context.push_back(d);
        }
    }
    encode(encode_context);

    return ;
}

void vector_save2Excel(){
    //-----------------------将数组 存到excel start--------------
    vector<int> keys_sum={61 , 62 , 116 , 118 , 143 , 94 , 659 , 736 , 847 , 771 , 741 , 608 , 165 , 148 , 134 , 103 , 120 , 128 , 89 , 586 , 719 , 631 , 324 , 315 , 298 , 83 , 118 , 158 , 124 , 159 , 212 , 268 , 252 , 269 , 376 , 340 , 309 , 320 , 433 , 609 , 713 , 532 , 402 , 437 , 341 , 168 , 27 , 52 , 88 , 57 , 77 , 49 , 66 , 96 , 311 , 370 , 456 , 502 , 582 , 664 , 481 , 585 , 582 , 461 , 198 , 171 , 103 , 167 , 199 , 182 , 127 , 259 , 395 , 529 , 516 , 453 , 359 , 338 , 222 , 184 , 168 , 156 , 152 , 179 , 241 , 274 , 247 , 209 , 217 , 234 , 201 , 101 , 101 , 65 , 100 , 382 , 973 , 911 , 923 , 429 , 306 , 373 , 732 , 1203 , 597 , 648 , 776 , 1008 , 1184 , 1211 , 828 , 910 , 286 , 222 , 250 , 146 , 235 , 828 , 865 , 193 , 195 , 315 , 108 , 386 , 800 , 652 , 276 , 526 , 406 , 445 , 506 , 957 , 1020 , 951 , 1362 , 1454 , 747 , 417 , 319 , 589 , 905 , 646 , 194 , 195 , 214 , 139 , 290 , 1194 , 895 , 802 , 713 , 404 , 949 , 1286 , 698 , 456 , 435 , 1223 , 1307 , 283 , 371 , 308 , 246 , 102 , 1496 , 629 , 426 , 190 , 146 , 1062 , 614 , 447 , 163 , 97 , 41 , 719 , 511 , 94 , 110 , 243 , 121 , 69 , 138 , 336 , 137 , 135 , 111 , 51 , 357 , 161 , 163 , 168 , 371 , 284 , 279 , 839 , 388 , 336 , 381 , 201 , 219 , 526 , 419 , 428 , 421 , 205 , 321 , 1188 , 379 , 268 , 331 , 137 , 140 , 347 , 1612 , 757 , 669 , 1061 , 799 , 588 , 352 , 1125 , 444 , 504 , 242 , 478 , 190 , 275 , 202 , 924 , 466 , 315 , 139 , 1263 , 1755 , 605 , 299 , 233 , 766 , 1344 , 530 , 330 , 350 , 910 , 360 , 1336 , 1433 , 1865 , 1604 , 1743 , 1531 , 1497 , 1469 , 957 , 1082 , 885 , 653 , 516 , 412 , 405 , 399 , 216 , 131 , 55 , 70 , 40 , 75 , 136 , 155 , 141 , 147 , 165 , 173 , 134 , 148 , 183 , 195 , 86 , 54 , 58 , 75 , 121 , 169 , 98 , 140 , 123 , 214 , 101 , 210 , 235 , 601 , 374 , 510 , 653 , 628 , 502 , 496 , 251 , 69 , 827 };
    vector<int> bits_nature={15616 , 15872 , 29696 , 30208 , 36608 , 24064 , 168704 , 188416 , 216832 , 197376 , 189696 , 155648 , 42240 , 37888 , 34304 , 26368 , 30720 , 32768 , 22784 , 150016 , 184064 , 161536 , 82944 , 80640 , 76288 , 21248 , 30208 , 40448 , 31744 , 40704 , 54272 , 68608 , 64512 , 68864 , 96256 , 87040 , 79104 , 81920 , 110848 , 155904 , 182528 , 136192 , 102912 , 111872 , 87296 , 43008 , 6912 , 13312 , 22528 , 14592 , 19712 , 12544 , 16896 , 24576 , 79616 , 94720 , 116736 , 128512 , 148992 , 169984 , 123136 , 149760 , 148992 , 118016 , 50688 , 43776 , 26368 , 42752 , 50944 , 46592 , 32512 , 66304 , 101120 , 135424 , 132096 , 115968 , 91904 , 86528 , 56832 , 47104 , 43008 , 39936 , 38912 , 45824 , 61696 , 70144 , 63232 , 53504 , 55552 , 59904 , 51456 , 25856 , 25856 , 16640 , 25600 , 97792 , 249088 , 233216 , 236288 , 109824 , 78336 , 95488 , 187392 , 307968 , 152832 , 165888 , 198656 , 258048 , 303104 , 310016 , 211968 , 232960 , 73216 , 56832 , 64000 , 37376 , 60160 , 211968 , 221440 , 49408 , 49920 , 80640 , 27648 , 98816 , 204800 , 166912 , 70656 , 134656 , 103936 , 113920 , 129536 , 244992 , 261120 , 243456 , 348672 , 372224 , 191232 , 106752 , 81664 , 150784 , 231680 , 165376 , 49664 , 49920 , 54784 , 35584 , 74240 , 305664 , 229120 , 205312 , 182528 , 103424 , 242944 , 329216 , 178688 , 116736 , 111360 , 313088 , 334592 , 72448 , 94976 , 78848 , 62976 , 26112 , 382976 , 161024 , 109056 , 48640 , 37376 , 271872 , 157184 , 114432 , 41728 , 24832 , 10496 , 184064 , 130816 , 24064 , 28160 , 62208 , 30976 , 17664 , 35328 , 86016 , 35072 , 34560 , 28416 , 13056 , 91392 , 41216 , 41728 , 43008 , 94976 , 72704 , 71424 , 214784 , 99328 , 86016 , 97536 , 51456 , 56064 , 134656 , 107264 , 109568 , 107776 , 52480 , 82176 , 304128 , 97024 , 68608 , 84736 , 35072 , 35840 , 88832 , 412672 , 193792 , 171264 , 271616 , 204544 , 150528 , 90112 , 288000 , 113664 , 129024 , 61952 , 122368 , 48640 , 70400 , 51712 , 236544 , 119296 , 80640 , 35584 , 323328 , 449280 , 154880 , 76544 , 59648 , 196096 , 344064 , 135680 , 84480 , 89600 , 232960 , 92160 , 342016 , 366848 , 477440 , 410624 , 446208 , 391936 , 383232 , 376064 , 244992 , 276992 , 226560 , 167168 , 132096 , 105472 , 103680 , 102144 , 55296 , 33536 , 14080 , 17920 , 10240 , 19200 , 34816 , 39680 , 36096 , 37632 , 42240 , 44288 , 34304 , 37888 , 46848 , 49920 , 22016 , 13824 , 14848 , 19200 , 30976 , 43264 , 25088 , 35840 , 31488 , 54784 , 25856 , 53760 , 60160 , 153856 , 95744 , 130560 , 167168 , 160768 , 128512 , 126976 , 64256 , 17664 , 211712};
    vector<int> bits_encode={14792 , 14048 , 26776 , 26416 , 30568 , 20712 , 182864 , 195504 , 225328 , 202832 , 197832 , 159232 , 40208 , 36528 , 31936 , 25248 , 27808 , 29536 , 19744 , 157264 , 196784 , 178864 , 98776 , 95720 , 87032 , 16488 , 23160 , 33624 , 26552 , 36456 , 50416 , 63080 , 59848 , 64096 , 91048 , 81880 , 75608 , 78096 , 107616 , 151808 , 180704 , 131392 , 99920 , 120440 , 98072 , 47088 , 6360 , 11512 , 19152 , 12344 , 17176 , 10144 , 14344 , 21416 , 75568 , 90272 , 112216 , 127176 , 147256 , 173136 , 124784 , 151864 , 153120 , 119576 , 48224 , 42360 , 24400 , 41600 , 49312 , 45736 , 31336 , 67488 , 103296 , 136576 , 134872 , 119432 , 94000 , 87624 , 54568 , 45920 , 42784 , 40096 , 37928 , 45360 , 61488 , 71048 , 62856 , 53744 , 55496 , 59096 , 47400 , 23400 , 20752 , 13960 , 23656 , 92680 , 255792 , 227248 , 235584 , 104368 , 73920 , 89224 , 184368 , 313872 , 150016 , 167544 , 198888 , 261400 , 311448 , 316544 , 209816 , 232824 , 69920 , 54344 , 61880 , 35632 , 55200 , 206976 , 213408 , 44912 , 47664 , 79368 , 25776 , 96056 , 205280 , 161152 , 68128 , 133832 , 104128 , 111904 , 124928 , 244096 , 262880 , 239720 , 354048 , 381440 , 183104 , 100720 , 76056 , 145632 , 232088 , 155968 , 45064 , 43440 , 47136 , 27952 , 67808 , 314904 , 226256 , 196272 , 181480 , 95712 , 241728 , 352888 , 179592 , 118640 , 112984 , 330184 , 353408 , 73088 , 94968 , 61560 , 46680 , 20880 , 419088 , 167296 , 117720 , 37208 , 39840 , 307648 , 176168 , 126104 , 42424 , 17896 , 7880 , 202312 , 140424 , 18728 , 21136 , 48704 , 22960 , 14800 , 36048 , 80608 , 27344 , 27128 , 21728 , 10080 , 90992 , 35464 , 34008 , 34040 , 84928 , 62760 , 64664 , 220440 , 82128 , 78648 , 91240 , 44656 , 49504 , 130000 , 94976 , 93960 , 105464 , 46320 , 74600 , 325840 , 84264 , 63512 , 84448 , 32312 , 30768 , 83568 , 464448 , 195968 , 167232 , 272552 , 201584 , 145352 , 83544 , 297512 , 100400 , 126584 , 58208 , 113736 , 43344 , 67128 , 47840 , 243792 , 116360 , 81168 , 33824 , 333312 , 488104 , 150472 , 75280 , 57856 , 189032 , 363320 , 128496 , 85808 , 94432 , 251216 , 93104 , 358152 , 378264 , 498176 , 423328 , 462448 , 403544 , 393840 , 384768 , 249272 , 283504 , 228280 , 164952 , 129000 , 102456 , 112320 , 110448 , 64464 , 37688 , 11328 , 13608 , 8480 , 15688 , 29728 , 34848 , 32120 , 33936 , 39128 , 41968 , 32024 , 35648 , 44896 , 45984 , 18712 , 11136 , 12072 , 19352 , 30016 , 41912 , 19792 , 31280 , 26832 , 59024 , 24408 , 55496 , 65456 , 175976 , 113128 , 151824 , 189712 , 181704 , 145896 , 145232 , 74400 , 18288 , 244088};

    vector<int> diff={2936 , 2699 , 5286 , 5122 , 5808 , 3974 , 39926 , 41953 , 47996 , 43072 , 42056 , 33632 , 8131 , 7459 , 6367 , 5061 , 5455 , 5803 , 3842 , 34114 , 43182 , 39668 , 22805 , 22162 , 19695 , 2995 , 4205 , 6349 , 5008 , 7206 , 10194 , 12656 , 11976 , 12816 , 18443 , 16483 , 15312 , 15693 , 22028 , 30988 , 37377 , 26747 , 20457 , 25890 , 21592 , 10178 , 1223 , 2149 , 3612 , 2313 , 3310 , 1820 , 2637 , 4046 , 15262 , 18243 , 22805 , 26247 , 30433 , 36241 , 26001 , 31735 , 32552 , 25084 , 9810 , 8550 , 4783 , 8465 , 10015 , 9391 , 6276 , 14084 , 21781 , 28523 , 28387 , 25248 , 19611 , 18306 , 11084 , 9270 , 8800 , 8308 , 7704 , 9225 , 12693 , 14829 , 13086 , 11208 , 11419 , 12071 , 9338 , 4545 , 3897 , 2637 , 4696 , 19034 , 54444 , 46645 , 48614 , 21242 , 14947 , 17835 , 37736 , 65945 , 30771 , 35063 , 41327 , 54601 , 65658 , 66494 , 43381 , 48482 , 14194 , 11009 , 12587 , 7139 , 11023 , 42491 , 43546 , 8927 , 9574 , 16358 , 5053 , 19762 , 42649 , 32722 , 13853 , 27646 , 21589 , 23038 , 25548 , 50617 , 54925 , 49406 , 74321 , 80557 , 37211 , 20612 , 15395 , 29907 , 48444 , 31551 , 8996 , 8590 , 9315 , 5144 , 13504 , 66505 , 46812 , 40069 , 37785 , 19275 , 50286 , 76362 , 38446 , 25910 , 24527 , 71326 , 76112 , 15900 , 20608 , 11513 , 8564 , 3929 , 92150 , 36255 , 26363 , 6848 , 8804 , 68281 , 39233 , 27917 , 8983 , 3231 , 1448 , 44586 , 30770 , 3565 , 3827 , 9011 , 4139 , 2766 , 7566 , 16329 , 5063 , 5031 , 4022 , 1808 , 18889 , 6834 , 6403 , 6337 , 16861 , 12041 , 12756 , 46493 , 15814 , 15668 , 18375 , 8589 , 9647 , 26550 , 18759 , 18189 , 21636 , 8959 , 14704 , 70746 , 16391 , 12755 , 17473 , 6371 , 5961 , 16808 , 103687 , 40970 , 34544 , 56814 , 41626 , 29682 , 16712 , 62940 , 19765 , 26122 , 11692 , 22749 , 8520 , 13623 , 9621 , 51392 , 23889 , 16900 , 6746 , 70705 , 106353 , 30790 , 15506 , 11754 , 38480 , 77652 , 25867 , 17691 , 20131 , 54180 , 19432 , 76047 , 79662 , 105559 , 89382 , 97917 , 85131 , 83097 , 81089 , 52229 , 59539 , 47505 , 34082 , 26497 , 21044 , 25097 , 24354 , 14502 , 8421 , 2074 , 2460 , 1528 , 2917 , 5721 , 6728 , 6214 , 6673 , 7837 , 8399 , 6354 , 7143 , 9072 , 9168 , 3580 , 2023 , 2248 , 4028 , 6300 , 8845 , 3668 , 6169 , 5149 , 13225 , 4995 , 12159 , 14655 , 39792 , 26110 , 34677 , 42614 , 40824 , 32899 , 32891 , 16774 , 3747 , 54705};


        bool flag=0;
        write_to_excel(flag,keys_sum,bits_nature,bits_encode,diff);//写进表的函数
}


void vectorTime_save2Excel(){
    //-----------------------将数组 存到excel start--------------
    vector<double> keys_sum={757720 , 601008 , 556140 , 805628 , 971668 , 758572 , 484516 , 681412 , 639212 , 623816 , 969896 , 789180 , 855672 , 773356 , 789044 , 1066148 , 1120352 , 660816 , 502960 , 548476 , 452688 , 476932 , 596764 , 762160 , 812436 , 938964 , 830064 , 742704 , 696528 , 520108 , 802076 , 730508 , 313316 , 335432 , 414944 , 990896 , 530248 , 241256 , 221256 , 351916 , 208216 , 294600 , 1061044 , 347384 , 469744 , 480892 , 854040 , 673592 , 486808 , 702440 , 855968 , 516296 , 738392 , 644200 , 560476 , 774580 , 781208 , 606960 , 512192 , 555072 , 1133172 , 573412 , 870348 , 1118420 , 1035724 , 1006244 , 863180 , 718108 , 771460 , 798736 , 910848 , 858736 , 968208 , 1252140 , 1160040 , 1003608 , 877896 , 988444 , 1010508 , 830232 , 866164 , 807440 , 735808 , 574084 , 600312 , 614788 , 584568 , 549036 , 543320 , 526128 , 578584 , 628824 , 799440 , 705152 , 764036 , 814080 , 879720 , 813812 , 705668 , 555448 , 787148 , 511712 , 480764 , 667860 , 649380 , 546368 , 396476 , 393296 , 512572 , 559312 , 563856 , 554584 , 458928 , 566444 , 674468 , 682188 , 453172 , 467428 , 705644 , 830172 , 869644 , 710444 , 754184 , 805576 , 846144 , 649896 , 634604 , 689672 , 627944 , 835516 , 711896 , 750320 , 655440 , 518244 , 436752 , 442124 , 475312 , 458104 , 480060 , 537576 , 592660 , 613676 , 415304 , 363024 , 345784 , 230276 , 266992 , 249700 , 204176 , 166436 , 176728 , 198932 , 264492 , 279728 , 299848 , 376088 , 573832 , 596904 , 591660 , 573604 , 591084 , 571956 , 530784 , 578256 , 536904 , 490372 , 426024 , 419324 , 571924 , 484064 , 556900 , 518892 , 556568 , 630436 , 593400 , 502952 , 595440 , 446648 , 544420 , 515408 , 654576 , 572016 , 480072 , 501384 , 674516 , 597288 , 613068 , 442212 , 499924 , 669984 , 606596 , 530240 , 508584 , 565956 , 633660 , 553524 , 640872 , 591936 , 589196 , 605564 , 610856 , 657576 , 678616 , 648012 , 661928 , 628616 , 653316 , 660112 , 595760 , 683380 , 668548 , 666672 , 614780 , 513604 , 366976 , 447228 , 535252 , 582548 , 606536 , 746308 , 773964 , 729504 , 753632 , 784360 , 778276 , 738112 , 605992 , 811648 , 688456 , 662864 , 655440 , 695508 , 656460 , 915176 , 791716 , 820260 , 805888 , 633408 , 534736 , 580568 , 516792 , 578292 , 497116 , 391108 , 485540 , 472052 , 554464 , 584848 , 684076 , 674408 , 563732 , 567532 , 609984 , 515988 , 544664 , 598872 , 694140 , 647920 , 606388 , 566440 , 612252 , 674784 , 705108 , 622236 , 595860 , 735276 , 800784 , 785936 , 885440 , 633884 , 838052 , 789340 , 592064 , 600712 , 714584 , 793288 , 795872 , 768756 , 638348 , 719116 , 778412 , 784048 , 650536 , 746548 , 745760 , 687672 , 857840 , 887260 , 915396 , 896208 , 950792 , 1058308 , 1072740 , 1090088 , 970060 , 1006236 , 1118448 , 1137524 , 1178368 , 1159152 , 1061152 , 1159288 , 1107404 , 1133020 , 824880 , 804300 , 765132 , 1007136 , 1175160 , 1052352 , 912316 , 957320 , 808628 , 783720 , 879424 , 715128 , 646080 , 594424 , 671812 , 529836 , 632044 , 654232 , 793860 , 840568 , 883896 , 957892 , 992568 , 1047408 , 907740 , 880928 , 867456 , 829508 , 1046160 , 989896 , 924324 , 966320 , 1000036 , 1001496 , 1021204 , 1011472 , 912748 , 995008 , 599492 , 784860 , 875284 , 1042980 , 1202056 , 1283972 , 842208 , 697964 , 728944 , 876860 , 1149752 , 986588 , 775800 , 505500};
    vector<double> bits_nature={721920 , 572160 , 529408 , 767488 , 925952 , 722944 , 461056 , 648448 , 608768 , 593920 , 923904 , 751616 , 815104 , 736000 , 751104 , 1015552 , 1067264 , 628992 , 478464 , 521216 , 429824 , 453376 , 568064 , 726272 , 773888 , 894464 , 790784 , 707328 , 663040 , 494336 , 763392 , 695296 , 297472 , 318464 , 394752 , 944896 , 505344 , 229120 , 210176 , 334336 , 197376 , 280064 , 1011712 , 330496 , 447232 , 457472 , 813312 , 641280 , 463872 , 669184 , 815360 , 491520 , 702976 , 613376 , 533248 , 737280 , 743936 , 577280 , 487168 , 528128 , 1079808 , 545792 , 829184 , 1065728 , 986880 , 958464 , 821760 , 683776 , 734720 , 760576 , 867584 , 817920 , 922624 , 1193728 , 1105408 , 955904 , 835584 , 941312 , 962304 , 790528 , 824576 , 768512 , 700672 , 546304 , 570880 , 584448 , 556032 , 521728 , 516352 , 499712 , 549888 , 598016 , 760832 , 671232 , 726784 , 774656 , 837632 , 774656 , 671488 , 528128 , 749312 , 486656 , 456960 , 635392 , 617728 , 519168 , 376320 , 373248 , 486912 , 531712 , 536064 , 527104 , 435712 , 538368 , 641280 , 648704 , 430336 , 444160 , 671232 , 790272 , 828160 , 675840 , 717824 , 766720 , 805376 , 618240 , 603904 , 656640 , 597248 , 795648 , 677632 , 714240 , 624128 , 493056 , 415488 , 420608 , 452608 , 435456 , 456448 , 511232 , 564224 , 584192 , 394752 , 344832 , 327936 , 218112 , 253184 , 237056 , 193536 , 157440 , 167168 , 188160 , 250880 , 265472 , 284416 , 357120 , 546048 , 567552 , 562688 , 545536 , 561920 , 543488 , 504320 , 550144 , 510720 , 465920 , 404736 , 398336 , 544000 , 459776 , 529152 , 492800 , 528896 , 599296 , 563968 , 477696 , 566016 , 424192 , 517376 , 489472 , 622592 , 543744 , 455936 , 476416 , 641536 , 568064 , 583168 , 419840 , 474880 , 637440 , 576768 , 504320 , 483584 , 538368 , 602880 , 526592 , 609792 , 562944 , 560384 , 575744 , 580864 , 625664 , 645888 , 616704 , 630016 , 598016 , 622080 , 628736 , 566784 , 650496 , 636416 , 634624 , 584960 , 488192 , 348928 , 425728 , 509440 , 554496 , 577536 , 710912 , 736768 , 694272 , 717312 , 746752 , 740608 , 702464 , 576768 , 772864 , 655104 , 630784 , 623360 , 661760 , 624128 , 871424 , 753664 , 780800 , 767232 , 602368 , 508416 , 551936 , 491008 , 549888 , 472320 , 370944 , 461568 , 448256 , 527104 , 556288 , 651008 , 641536 , 535808 , 539392 , 579840 , 490240 , 517376 , 569344 , 660224 , 616192 , 576768 , 538624 , 582144 , 642304 , 670976 , 591616 , 566528 , 699648 , 762368 , 747776 , 843008 , 602880 , 797696 , 751616 , 562944 , 571392 , 679936 , 755456 , 757760 , 731904 , 607232 , 684032 , 740864 , 746240 , 618496 , 710144 , 709632 , 654080 , 816640 , 844544 , 871680 , 853248 , 905216 , 1007872 , 1021696 , 1038336 , 923648 , 958464 , 1065472 , 1083648 , 1122560 , 1104384 , 1010688 , 1104384 , 1054720 , 1079552 , 784896 , 765440 , 728064 , 958976 , 1119488 , 1002496 , 868608 , 911360 , 769280 , 745728 , 837120 , 680192 , 614144 , 564736 , 638976 , 503296 , 600576 , 622336 , 755456 , 800256 , 841216 , 911872 , 945408 , 997632 , 864000 , 838400 , 825600 , 789248 , 996096 , 942592 , 879872 , 920064 , 952320 , 953856 , 972800 , 963840 , 869888 , 948480 , 570880 , 747520 , 833792 , 994304 , 1145600 , 1223936 , 802304 , 664832 , 694016 , 835072 , 1095680 , 940032 , 739072 , 482304};
    
        bool flag=0;
    writeTime_to_excel(flag,keys_sum,bits_nature);//写进表的函数
}

/**
//定义霍夫曼树节点结构体
struct HuffmanNode {
    int freq;
    char data;
    HuffmanNode* left;
    HuffmanNode* right;
    HuffmanNode(int f, char d) : freq(f), data(d), left(nullptr), right(nullptr) {}
};

//定义比较函数，用于优先队列排序
struct CompareHuffmanNodes {
    bool operator()(HuffmanNode* a, HuffmanNode* b) {
        return a->freq > b->freq;
    }
};

//创建霍夫曼树
HuffmanNode* BuildHuffmanTree(unordered_map<char, int> freqMap) {
    priority_queue<HuffmanNode*, vector<HuffmanNode*>, CompareHuffmanNodes> pq;
    
    //将字符频率转换为叶子节点，并加入优先队列
    for (auto pair : freqMap) {
        HuffmanNode* node = new HuffmanNode(pair.second, pair.first);
        pq.push(node);
    }
    
    //合并节点，直到只剩下一个根节点
    while (pq.size() > 1) {
        HuffmanNode* left = pq.top();
        pq.pop();
        
        HuffmanNode* right = pq.top();
        pq.pop();
        
        HuffmanNode* parent = new HuffmanNode(left->freq + right->freq, '\0');
        parent->left = left;
        parent->right = right;
        pq.push(parent);
    }
    
    return pq.top(); //返回根节点
}

//生成字符到编码的映射表
void GenerateHuffmanCodes(HuffmanNode* node, string code, unordered_map<char, string>& codeMap) {
    if (node == nullptr) {
        cout<<"nullptr"<<endl;
        return;
    }
    
    if (node->left == nullptr && node->right == nullptr) {
        codeMap[node->data] = code;
        cout<<"node->left == nullptr=="<<node->left <<" , "<< node->right<<endl;
        return;
    }
    
    GenerateHuffmanCodes(node->left, code + "0", codeMap);
    GenerateHuffmanCodes(node->right, code + "1", codeMap);
}

//将字符串编码成霍夫曼编码
string EncodeString(string str, unordered_map<char, string>& codeMap) {
    string encodedString = "";
    
    for (char c : str) {
        encodedString += codeMap[c];
    }
    
    return encodedString;
}

//将霍夫曼编码解码成原始字符串
string DecodeString(string encodedStr, HuffmanNode* root) {
    string decodedString = "";
    HuffmanNode* currentNode = root;
    
    for (char c : encodedStr) {
        if (c == '0') {
            currentNode = currentNode->left;
        } else {
            currentNode = currentNode->right;
        }
        
        if (currentNode->left == nullptr && currentNode->right == nullptr) {
            decodedString += currentNode->data;
            currentNode = root;
        }
    }
    
    return decodedString;
}
 */

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
            cout << "编码出错！"<<temp << endl;
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
    cout << decode << endl;
    return decode;
}


#include <bitset>
int main(int argc, const char * argv[]) {
    
//----------------压缩数据汇总----------------
//    vectorTime_save2Excel();
//----------------霍夫曼编码 处理4位----------------
//    string inputStr = "000000000011000101010000000000101010011000000000001000110111000000000001110010000000000000010101100100000000000011101010000000000000011110110001001011011011000001001111";
//    //伯努利序列按照4位分组后的结果
//
////    string inputStr2 =inputStr;
//        string outputStr = "";
//        unordered_map<char, int> freqMap;
//        unordered_map<char, string> codeMap;
//
//    string cc="";
//        //统计字符频率
//        for (int i = 0; i < inputStr.length(); i += 4) {
//            string subStr = inputStr.substr(i, 4);
//            char c = stoi(subStr, nullptr, 2) +'0'; //将二进制字符串转换为字符
//            freqMap[c]++;
//            cout<<c<<",";
//
//            cc+=c;
//        }
//    cout<<freqMap.size()<<endl;
//    cout<<"cc="<<cc<<endl;
//
//
////    cout << "inputStr string: " << inputStr << endl;
////        HuffmanNode* root = BuildHuffmanTree(freqMap);
////
////    cout<<root->data<<" , "<<root->freq<<endl;
////
////        GenerateHuffmanCodes(root, "", codeMap);
////        outputStr = EncodeString(inputStr, codeMap);
////
////    cout<<codeMap.size()<<endl;
////        //输出编码结果
////        cout << "Encoded string: " << outputStr << endl;
////        cout << "Encoded string length: " << outputStr.length() << endl;
//
//
//
//
////        cout << "请输入字符串进行编码：" << endl;
////        cin >> str; // 输入要加密的字符串
//        HuffmanCode st(cc); // 对象
//        cout << "对字符串编码情况如下：" << endl;
//    string str_encode=   st.Encode(); // 编码
//        cout << endl;
//    cout<<inputStr<<endl;
//    cout<<str_encode<<endl;
//
//    string str_real_encode=   st.real_encode(cc);
//    cout<<"str_real_encode="<<str_real_encode<<endl;
//    cout<<str_real_encode.size()<<endl;
////        cout << "请输入要解码的二进制编码:" << endl;
////        cin >> str;
//        cout << "解码如下：" << endl;
//    string str_decode=  st.Decode(str_real_encode);
//    string str_01="";
//        //统计字符频率
//        for (int i = 0; i < str_decode.length(); i ++) {
//            char substr= str_decode[i];
//            int int_str=int(substr-'0');
//            bitset<4> bits_4(int_str);
//            str_01+=bits_4.to_string();
//        }
//    cout<<str_01<<endl;
//    ----------------用于数据压缩与发送的数据预处理--------------------
//    int num[8]={77232,123,234,345,456,567,678,789};//977232
//
////    std::bitset<20*8> bits(num);
//
////    const int arr[] = {1, 3};
////    cout<<sizeof(arr) <<" , "<<sizeof(*arr)<<" , "<<sizeof(int) <<endl;
//    const size_t arr_size = sizeof(num) / sizeof(*num);
////    std::bitset<arr_size * sizeof(int) * 8> bits;
//
//    std::bitset<arr_size * 20> bits;
////    cout<<bits.size()<<endl;
//
//    for (size_t i = 0; i < arr_size; ++i) {
////        cout<< (std::bitset<bits.size()>(num[i]) << i * 20) <<endl;
//        bits |= std::bitset<bits.size()>(num[i]) << i * 20;
//    }
//
//    cout << "Binary representation of " << num << ": "
//             << bits.to_string() << endl;//.substr(bits.to_string().find('1'))
//             // 去掉前导0输出
//    uchar output_buffer[(arr_size *20+8-1)/8]; // 3个字节的输出缓冲区，保留20位二进制数
////    cout<<(arr_size *20+8-1)/8<<" ,"<<bits.size() <<endl;
//    const size_t output_buffer_size = sizeof(output_buffer) / sizeof(*output_buffer);
//    for(int i = 0; i < output_buffer_size; i++) {
////        cout<<(bits >> (i * 8))<<endl;
//
////        cout<<"bits"<<bits.to_string()<<endl;
////        std::bitset<8> bits_single(bits.to_ulong());
//        std::bitset<8> bits_single;
//        // 将原bitset的末8位复制到新的bitset中
//        for(int j = 0; j < 8; j++){
//            bits_single[j] = bits[j ];//+ (bits.size() - 8)
////            cout<<bits[j]<<" , ";
//        }
////        cout<<endl;
////        cout<< static_cast<uchar>(bits_single.to_ulong())<<endl;
//
//        output_buffer[i] = static_cast<uchar>(bits_single.to_ulong()); // 每次输出8位二进制数到对应字节中
//        bits= bits >> 8;
//
////        cout<< bits_single.to_ulong()<<endl;
//    }
////    0000000000110001010100000000001010100110000000000010001101110000000000011100100000000000000101011001000000000000111010100000000000000111101100010010110110110000
//
//    std::cout << "Output buffer: " << output_buffer[0] << " " << output_buffer[1] << " " << output_buffer[2] << std::endl;
//
//    std::bitset<arr_size *20> binary_num;
//    const size_t binary_num_size =binary_num.size();
//    for(int i = 0; i < (arr_size *20+8-1)/8; i++) {
//       binary_num |= (static_cast<std::bitset<binary_num_size>>(output_buffer[i])) << (i * 8); // 每次读取8位二进制数，并通过位移操作组合成20位二进制数
//   }
//
//       std::cout << "Binary number: " << binary_num << std::endl;
//
//    vector<int> indexEnd;
//    for(int i = 0; i < arr_size; i++) {
//        std::bitset<20> bits_single;
//        // 将原bitset的末8位复制到新的bitset中
//        for(int j = 0; j < 20; j++){
//            bits_single[j] = binary_num[j ];
//        }
//        indexEnd.push_back(bits_single.to_ulong()); // 每次输出8位二进制数到对应字节中
//        binary_num= binary_num >> 20;
//
//    }
//
//    for(int i=0;i<indexEnd.size();i++){
//        cout<<indexEnd[i]<<" , ";
//    }
//    cout<<endl;
    //----------------------------最开始的方案----------------------
//    string text="1100111000000000000100001000000000000000000000000000010000010000101100100000100000001000000010000000000010101100000000000101000101000000100000111000010000000000000000101110010000000000000000101001110101110000000000000001100000100000100011101000000000001010";
//    int residual[256];
//    for (int i = 0; i <text.size(); i++)
//    {
//        residual[i] = int(text[i]-'0');
////        cout<<residual[i];
//    }
//
//    std::vector<unsigned char> bitstream;
//    IntraEncodeBowResidual(residual,bitstream);
    //-------------------------哈夫曼编码-------------------------
//    vector<int> haffman=compress(text);
//    cout<<haffman.size()<<endl;
//    for(int i=0;i<haffman.size();i++){
//        cout<<haffman[i]<<" , ";
//    }
//    cout<<endl;
    //----------------------------LZW编码----------------------
//    double p = 0.7; // 伯努利序列的出现概率
////      string text="1100111000000000000100001000000000000000000000000000010000010000101100100000100000001000000010000000000010101100000000000101000101000000100000111000010000000000000000101110010000000000000000101001110101110000000000000001100000100000100011101000000000001010";
//
//    string text="001010110100101001001011010011010100101";
//    vector<int> compressed= compress(text);
//      cout << "原始长度：" << text.size() << endl;
//      cout << "压缩后长度：" << compressed.size() << endl;
//      cout << "压缩比：" << (double)compressed.size()*4 / text.size() << endl;
//    for(int i=0;i<compressed.size();i++){
//        cout<<compressed[i]<<" , ";
//
//    }
    //-------------------------哈夫曼编码---LZW----------------------
//    double p = 0.7; // 伯努利序列的出现概率
//      string text="1100111000000000000100001000000000000000000000000000010000010000101100100000100000001000000010000000000010101100000000000101000101000000100000111000010000000000000000101110010000000000000000101001110101110000000000000001100000100000100011101000000000001010";
//
////    string text="001010110100101001001011010011010100101";
//    compress(text,0,256);
    //-------------------------算术----------------------
    
    // 定义一个 256 位的伯努利序列
//    string text="1100111000000000000100001000000000000000000000000000010000010000101100100000100000001000000010000000000010101100000000000101000101000000100000111000010000000000000000101110010000000000000000101001110101110000000000000001100000100000100011101000000000001010";
//    string text;
//    for (int i = 0; i < 256; i++) {
//           text += (i % 2 == 0 ? '0' : '1');
//       }
//       // 压缩输入序列
//       vector<int> compressed = compress(text);
//       // 输出压缩率和压缩后的序列大小
//       double compression_ratio = static_cast<double>(compressed.size()) / static_cast<double>(text.length());
//       cout << "Compression ratio: " << compression_ratio << endl;
//       cout << "Compressed size: " << compressed.size() << endl;
       // 解压缩压缩序列
//       string decompressed = decompress(compressed);
//       // 比较输入序列和解压缩后的序列是否相同
//       bool is_equal = (text == decompressed);
//       cout << "Input and decompressed sequences are " << (is_equal ? "" : "not ") << "equal" << endl;
      
    //----------------------------------------------
//    matchPoint();
    //----------------------------------------------
//    drawPoint();
    
//    drawPoint_testMatch();
    
//    km算法-----------------
//    n=3;
//   m=7;
//   memset(w, 0, sizeof(w));
//    float aaa[21]={1.2,2.4,4.6,0,0,0,0, 0,1.5,3.8,0,0,0,0, 0,0,0,0,3.2,2.5,4.3};
//    for(int i=0;i<n;i++){
//        for(int j=0;j<m;j++){
////            cout<<lx[i]<<" , "<<ly[j]<<endl;
////            w[i][j] = aaa[(i-1)*m+j-1];
//            w[i][j] = aaa[i*m+j];
////            lx[i] = max(lx[i], w[i][j]);
////            ly[j] = max(ly[j], w[i][j]);
////            printf("%d - %d =%d %d %d\n", i, j,w[i][j],i*m+j,aaa[i*m+j]);
//        }
//    }
//
//    float ans = KM();
//    if (fabs(ans+1) <=0.00001) {
//        printf("不存在完备匹配\n");
//    } else {
//        printf("最大权匹配为：%f\n", ans);
//        for (int i = 0; i < m; i++) {
//            if (linky[i] != -1) {
//                printf("%d - %d =%f\n", linky[i], i,w[linky[i]][i] );
//            }
//        }
//    }
   
    //    距离度量-----------------
    
    // 示例：计算两个距离矩阵之间的距离
//    vector<vector<double>> a = {{0, 1, 2}, {1, 0, 3}, {2, 3, 0}};
////    vector<vector<double>> b = {{0, 1, 2}, {1, 0, 3}, {2, 3, 0}};//2.74874
////    vector<vector<double>> b = {{0, 2, 4}, {2, 0, 6}, {4, 6, 0}};//2.74874 2.74874
//    vector<vector<double>> b = {{0, 1.2, 2.6}, {1.2, 0, 3.6}, {2.6, 3.6, 0}};//2.77889
//    double distance = distanceBetweenMatrices(a, b);
//    cout << "The distance between matrix A and matrix B is: " << distance << endl;
//
//    return 0;
    //----------------生成码------------------------------
//    cv::Mat markerImage;
//    // Load the predefined dictionary
//    cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//
//    for(int i=0;i<180;i++){
//        // Generate the marker
//        cv::aruco::drawMarker(dictionary, i, 1000, markerImage, 1);
//        cv::imwrite("/Users/zhangjianhua/Downloads/泳池/1000/"+to_string(i)+".png", markerImage);
//    }
    //-----------------检测码-----------------------------
//    string path="/Users/zhangjianhua/Downloads/泳池/qq.jpeg";
//    cv::Mat frame=cv::imread(path, cv::IMREAD_UNCHANGED);
//    cout<<path<<endl;
//    cout<<frame.type()<<" , "<<CV_8UC1<<" , "<<CV_8UC3<<endl;
//
//    // Load the dictionary that was used to generate the markers.
//    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//
//    // Initialize the detector parameters using default values
//    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
//
//    // Declare the vectors that would contain the detected marker corners and the rejected marker candidates
//    vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;
//
//    // The ids of the detected markers are stored in a vector
//    vector<int> markerIds;
//
//    // Detect the markers in the image
//    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
//
//    for(int i=0;i<markerIds.size();i++){
//        cout<<markerIds[i]<<endl;
//    }
//    cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
//    cv::imwrite("/Users/zhangjianhua/Downloads/泳池/out.png", frame);

    //--------------------提取棋盘格角点并绘制--------------------------
//    mp4_2img();
//    std::vector<cv::Point2f> corners1;//提取的角点
//    {
//        cv::Mat image_color = cv::imread("/Users/zhangjianhua/Downloads/2-5/test/2/002478.jpg", cv::IMREAD_COLOR);
//        cv::Mat image_gray;
//        cv::cvtColor(image_color, image_gray, cv::COLOR_BGR2GRAY);
//
//        bool ret = cv::findChessboardCorners(image_gray,
//                                             cv::Size(5,6),
//                                             corners1,
//                                             cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
//
//        //指定亚像素计算迭代标注
//        cv::TermCriteria criteria = cv::TermCriteria(
//            cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
//            40,
//            0.1);
//
//    //    cout<<ret<<" , "<<corners.size()<<endl;
//
//        //亚像素检测
//        cv::cornerSubPix(image_gray, corners1, cv::Size(5, 5), cv::Size(-1, -1), criteria);
//
//        for ( int i=0,n=corners1.size();i<n;i++)
//        {
//            float b = 255*float ( rand() ) /RAND_MAX;
//            float g = 255*float ( rand() ) /RAND_MAX;
//            float r = 255*float ( rand() ) /RAND_MAX;
//
//            putText(image_color,to_string(i),corners1[i],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255),1,8);
//            cv::circle ( image_color, cv::Point2f (corners1[i].x, corners1[i].y), 8, cv::Scalar ( b,g,r ), 1 );
//        }
//        cv::imwrite("/Users/zhangjianhua/Downloads/2-5/test/test/out_num.jpg",image_color);
//    }
//    std::vector<cv::Point2f> corners2;//提取的角点
//    {
//        cv::Mat image_color = cv::imread("/Users/zhangjianhua/Downloads/2-5/test/5/002478.jpg", cv::IMREAD_COLOR);
//        cv::Mat image_gray;
//        cv::cvtColor(image_color, image_gray, cv::COLOR_BGR2GRAY);
//
////        std::vector<cv::Point2f> corners;//提取的角点
//
//        bool ret = cv::findChessboardCorners(image_gray,
//                                             cv::Size(5,6),
//                                             corners2,
//                                             cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
//
//        //指定亚像素计算迭代标注
//        cv::TermCriteria criteria = cv::TermCriteria(
//            cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
//            40,
//            0.1);
//
//    //    cout<<ret<<" , "<<corners.size()<<endl;
//
//        //亚像素检测
//        cv::cornerSubPix(image_gray, corners2, cv::Size(5, 5), cv::Size(-1, -1), criteria);
//    }
//
//    {
//
//        Eigen::Matrix3d k1,k2,r1,r2;
//        Eigen::Vector3d t1,t2;
//        k1<<3232.67,0,1002.73,0,3251.84,616.14,0,0,1;
//        k2<<2134.09,0,1097.13,0,2140.87,700.550,0,0,1;
//        r1<<1,0,0,0,1,0,0,0,1;
//        r2<<0.53,0.17,-0.83,-0.34,0.94,-0.03,0.77,0.3,0.56;
//        t1<<0,0,0;
//        t2<<14.88,0.36,-1.35;
//
//        //把2d点导入 像素坐标 以及各自的位姿
//            vector<Eigen::Vector2d> measurements_uv_1,measurements_uv_2;
//        for(int i=0;i<corners1.size();i++){
//            Eigen::Vector2d p1((corners1[i].x-k1(0,2))/k1(0,0),(corners1[i].y-k1(1,2))/k1(1,1));
//            measurements_uv_1.push_back(p1);
//            Eigen::Vector2d p2((corners2[i].x-k2(0,2))/k2(0,0),(corners2[i].y-k2(1,2))/k2(1,1));
//            measurements_uv_2.push_back(p2);
//            cout<<p1[0]<<" , "<<p1[1]<<endl;
//        }
//
//
//            Eigen::Matrix<double, 3, 4> T_i1_w,T_j2_w;
//            T_i1_w.block<3,3>(0,0)=r1;
//            T_i1_w.block<3,1>(0,3)=t1;
//            T_j2_w.block<3,3>(0,0)=r2;
//            T_j2_w.block<3,1>(0,3)=t2;
//
//            //转成相机平面的点
////            c1.undistortedPoints_2f(measurements_uv_1);
////            c2.undistortedPoints_2f(measurements_uv_2);
//            //转化成3d点
//            vector<Vector3d> point3d_2Plane;
//            for(int i=0,j=measurements_uv_1.size();i<j;i++){
//                Eigen::Vector3d point_3d;
//                triangulatePoint(T_i1_w,T_j2_w,measurements_uv_1[i],measurements_uv_2[i],point_3d);
//                cout<<point_3d[0]<<" , "<<point_3d[1]<<" , "<<point_3d[2]<<endl;
//                point3d_2Plane.push_back(point_3d);
//            }
//
//        drawPoint(point3d_2Plane);
//
//
//        for(int j=1;j<30;j++){
//
//                Vector3d p_tes=point3d_2Plane[j-1];
//                Vector3d p_tes_2=point3d_2Plane[j];
//                p_tes_2-=p_tes;
//                double sum=p_tes_2.norm();
//                cout<<sum<<endl;
//
//        }
//    }
    //角点绘制
//    cv::drawChessboardCorners(image_color, cv::Size(5, 6), corners, ret);
//
//    cv::imshow("chessboard corners", image_color);
//    cv::waitKey(0);
    
    
    
//    -------------------------3d投影---------------------
    
//    vector<uchar> status1,status2,status3;
//    status1.push_back(0);
//    status1.push_back(1);
//    status1.push_back(0);
//    status1.push_back(1);
//
//    status2.push_back(0);
//    status2.push_back(1);
//    status2.push_back(1);
//    status2.push_back(0);
//    for(int i=0;i<4;i++){
//        if(status1[i]){
//            cout<<"true"<<endl;
//        }else{
//            cout<<"false"<<endl;
//        }
//        status3.push_back(status1[i]&status2[i]);
//        if(status3[i]){
//            cout<<"status3 true"<<endl;
//        }else{
//            cout<<"status3 false"<<endl;
//        }
//    }
//
//    return 0;
    
 

    //----------------------------------------------
    setMainClientID();

    //create socket server
    Server *s = new Server();
    //建立hash函数
    s->myHashMap={{ "device", true},{ "g_imu",true},{ "KF_T",true},{ "KF_bowI",true},{ "KF_bowC",true},{ "KF_Origin",true},{ "ErrorLoop",true},{ "CorrIndex",true},{ "FrontPose",true},{ "FrontPos2",true},{ "KF_T_add",true},{ "KF_key",true},{ "Ar",true},{ "FrontPos3",true}};

//        const char *voc_file ="/Users/zhangjianhua/Desktop/VINS_MapFusion/Resources/brief_k10L6.bin";
    PoseGraphGlobal *poseGraphGlobal=new PoseGraphGlobal(voc_file, COL, ROW);
    s->poseGraphGlobal=poseGraphGlobal;


    FeatureMap *global_featureMap=new FeatureMap();
    s->global_featureMap=global_featureMap;
    poseGraphGlobal->global_featureMap=global_featureMap;

    DrawResult *drawResult=new DrawResult();
    s->drawResult=drawResult;

    boost::thread globalLoopThread(&Server::AcceptAndDispatch, s);

    //开一个线程
//        boost::thread PoseGraphGlobalRun_mapfusion(&Server::PoseGraphGlobalRun_inServer, s);

    //开一个线程 因为都是和主地图一起做优化 所以没法同时做
//    论文2
//    boost::thread PoseGraphGlobalRun_solveRelativePose(&PoseGraphGlobal::MergeLocal_17, s->poseGraphGlobal);
//    论文3
    boost::thread PoseGraphGlobalRun_solveRelativePose(&PoseGraphGlobal::MergeLocal_18, s->poseGraphGlobal);
    //开一个线程
//        boost::thread PoseGraphGlobalRun_optimize(&PoseGraphGlobal::GlobalFuse_4, s->poseGraphGlobal);

    //接收数据
    drawResult->visualization();
    return 0;
}
