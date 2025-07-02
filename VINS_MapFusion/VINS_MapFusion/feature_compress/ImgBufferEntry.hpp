//
//  ImgBufferEntry.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2023/4/26.
//  Copyright © 2023 zx. All rights reserved.
//

#ifndef ImgBufferEntry_hpp
#define ImgBufferEntry_hpp


#pragma once


#include <opencv2/opencv.hpp>
#include "DVision.h"
using namespace DVision;

namespace LBFC2
{

class ImgBufferEntry
{
public:
    ImgBufferEntry(int width = -1, int height = -1, int nLevels = 8, bool leftImage = true);

    void allocateSpace( const int nFeatures);

    void addFeature( const int index, const cv::KeyPoint &kp, const BRIEF::bitset &descriptor, const float &fDepth);
    void addFeature( const int index, const cv::KeyPoint &kp, const BRIEF::bitset &descriptor, const unsigned int visualWord, const float &fDepth);
    void addFeatures( const std::vector<cv::KeyPoint> &kpts, const std::vector<BRIEF::bitset> &descriptor, const std::vector<float> &vfDepths);


    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
    void AssignFeatures();
//    std::vector<unsigned int> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const;
    std::vector<unsigned int> GetStereoFeaturesInLine(const float  &yL, const int &octave) const;

public:
    std::vector<BRIEF::bitset> mDescriptors;
    std::vector<cv::KeyPoint> mvKeypoints;
    std::vector<unsigned int> mvVisualWords;
    std::vector<float> mvfDepths;

    int mnWidth;
    int mnHeight;
    int mnLevels;

    bool mbAllocated;
    bool mbLeft;
    long long mnImageId;

    int mnCols = 47;//32
    int mnRows = 30;//24
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;
    std::vector<std::vector<std::vector<unsigned int> > > mGrid;
    std::vector<std::vector<std::vector<unsigned int> > > mvRowIndices;


};

} // END NAMESPACE

#endif /* ImgBufferEntry_hpp */
