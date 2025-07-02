//
//  DrawResult.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/6/2.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef DrawResult_hpp
#define DrawResult_hpp

#include <stdio.h>
#include<pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include "global_param.hpp"
#include <eigen3/Eigen/Dense>
#include "PoseGraph.hpp"
#include "VINS.hpp"

using namespace std;
using namespace Eigen;

class DrawResult{
public:
    PoseGraph* mpPoseGraph;
    VINS* mpVins;
    float mViewpointX = -0;
    float mViewpointY = -5;
    float mViewpointZ = -10;
    float mViewpointF = 500;

    
    
    DrawResult();
    void visualization();
    void realDrawResult();
    void ViewCameraPose(pangolin::OpenGlMatrix &M);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    
};
#endif /* DrawResult_hpp */
