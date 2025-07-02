//
//  feature_coder.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2023/4/26.
//  Copyright © 2023 zx. All rights reserved.
//

#ifndef feature_coder_hpp
#define feature_coder_hpp


#pragma once

#include <opencv2/opencv.hpp>

#include <list>
#include <queue>
#include <unordered_map>

#include "ac_extended.hpp"
#include "utils_compress.hpp"
#include "FBrief.h"
#include "TemplatedVocabulary.h"
#include "voc_stats.h"
#include "ImgBufferEntry.hpp"

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
//using namespace Eigen;

// DLoopDetector and DBoW2
#include "DBoW2.h" // defines BriefVocabulary
#include "DLoopDetector.h" // defines BriefLoopDetector
#include "DVision.h" // Brief
#include "demoDetector.hpp"
using namespace DVision;
using namespace DBoW2;
using namespace DLoopDetector;

typedef demoDetector<BriefVocabulary, BriefLoopDetector, FBrief::TDescriptor> Demo;

namespace LBFC2
{

enum CodingMode {
    INTRA = 0,
    INTER = 1,
    INTER_SKIP = 2,
    STEREO_PRED = 3,
    NONE = -1
};

// Signaling
struct EncodeInfo
{
    uint32_t numFeatures;
//    uint32_t numFeaturesRight;
    uint32_t fixedLengthSize;
};


struct ThreadInfo
{
    uint32_t fixedLengthSize;
    uint32_t variableLengthSize;
};

struct Candidate
{
    bool skipMode = false;
    int candidateId = -1;
    int numCandidates = 0;
    int keypointId = -1;
    int imageId = -1;
    BRIEF::bitset residual;
};


struct ModeDecision
{
    CodingMode mode = CodingMode::NONE;
    unsigned int visualWord;
    int nodeId = -1;
    Candidate candidate;
    BRIEF::bitset residual;
    float rate;
    int keypointId = -1;
    int octave = -1;
    int bufferIndex = -1;

    bool stereoMatch = false;//暂时感觉没用
};



class FeatureCoder
{
public:
    FeatureCoder( Demo &demo, CodingStats &model, int imWidth, int imHeight, int angleBins,
            int bufferSize, bool inter, bool stereo = false, bool depth = false, float f = 0.0,
            int threads = 4);

    // Depth coding
    void encodeImageDepth( const std::vector<cv::KeyPoint> &kptsLeft, const std::vector<BRIEF::bitset> &descriptorsLeft,const std::vector<Eigen::Vector3d> &point_clouds, vector<uchar> &bitstream );
    void decodeImageDepth( const vector<uchar> &bitstream, std::vector<cv::KeyPoint> &kpts, std::vector<BRIEF::bitset> &descriptor, std::vector<unsigned int> &visualWords, std::vector<float> &vfDepthValues );


    // Mono coding
    void encodeImageMono( const std::vector<cv::KeyPoint> &kptsLeft, const std::vector<BRIEF::bitset> &descriptorsLeft,const std::vector<Eigen::Vector3d> &point_clouds,vector<uchar> &bitstream , vector<int> &bowIndex_ljl);
    void decodeImageMono( const vector<uchar> &bitstream, std::vector<cv::KeyPoint> &kptsLeft,
        std::vector<BRIEF::bitset> &descriptorsLeft, std::vector<unsigned int> &visualWords , std::vector<int> &bowIndex_ljl);
    
//    后面删一下
//    void encodeImageMono( const std::vector<cv::KeyPoint> &kptsLeft, const std::vector<BRIEF::bitset> &descriptorsLeft,const std::vector<Eigen::Vector3d> &point_clouds,vector<uchar> &bitstream );
//    void decodeImageMono( const vector<uchar> &bitstream, std::vector<cv::KeyPoint> &kptsLeft,
//        std::vector<BRIEF::bitset> &descriptorsLeft, std::vector<unsigned int> &visualWords );
    
    float intraCosts( const cv::KeyPoint &currentKpt, const BRIEF::bitset &descriptor, unsigned int &visualWord, BRIEF::bitset &intraResidualMat);

private:
    // Init models
    void initEncoderModels( ACEncodeContext &globalACCoderContext );
    void initDecoderModels( ACDecodeContext &globalACDecoderContext );


    // Feature level
    unsigned int encodeFeature(const ModeDecision &decision, const cv::KeyPoint &keypoints, const BRIEF::bitset &descriptor,
            const float &fDepth, EncodeContext &globalCoderContext, ACEncodeContext &globalACCoderContext);
    void decodeFeature(cv::KeyPoint &keypoints, BRIEF::bitset &descriptor, unsigned int &visualWord,
            DecodeContext &globalDecoderContext, ACDecodeContext &globalACDecoderContext,int  visualWord_ljl);


    // Cost calculation
    
    float interCandidateSelection( const cv::KeyPoint &currentKpt, const BRIEF::bitset &descriptor, Candidate &cand);
    int matchStereoFeatures(const std::vector<cv::KeyPoint> &kptsLeft, const std::vector<BRIEF::bitset> &descriptorsLeft,std::vector<float> &vfDepthValues);

    // Mode decision
    ModeDecision modeDecision( const cv::KeyPoint &keypoint, const BRIEF::bitset &descriptor, int stereoMatchBufferId = -1 );


    // MODE CODING
    size_t encodeMode(const CodingMode &mode,  EncodeContext &ctxt);
    size_t decodeMode(DecodeContext &modeCtxt, CodingMode &mode);


    // SKIP MODE
    size_t encodeSkipMode(int nMode,  EncodeContext &ctxt);
    int decodeSkipMode(DecodeContext &ctxt);


    // INTRA CODING
    size_t IntraEncodeBow(unsigned int visualWord, EncodeContext &bowCtxt);
    void IntraDecodeBow(DecodeContext &bowCtxt, unsigned int &visualWord );

    size_t IntraEncodeBowAC(unsigned int visualWord, ACEncodeContext &bowCtxt);
    void IntraDecodeBowAC(ACDecodeContext &bowCtxt, unsigned int &visualWord );
    
    //ljl
    void IntraEncodeBowIndex(unsigned int visualWord, std::vector<unsigned char> &bitstream);
    void IntraEncodeBowResidual(const BRIEF::bitset &residual, std::vector<unsigned char> &bitstream);
    size_t IntraEncodeResidual_haffman(const BRIEF::bitset &residual, ACEncodeContext &accontext);
    
    size_t IntraEncodeKeyPoint(const cv::KeyPoint &keypoint, EncodeContext &kptCtxt);
    void IntraDecodeKeyPoint(DecodeContext &kptCtxt, cv::KeyPoint &keypoint);

    size_t IntraEncodeKeyPointAC(const cv::KeyPoint &keypoint, ACEncodeContext &accontext);
    void IntraDecodeKeyPointAC(ACDecodeContext &kptCtxt, cv::KeyPoint &keypoint);

    size_t IntraEncodeResidual(const BRIEF::bitset &residual, ACEncodeContext &resCtxt);
    void IntraDecodeResidual(ACDecodeContext &resCtxt, BRIEF::bitset &residual);

    BRIEF::bitset IntraReconstructDescriptor(const unsigned int &visualWord, BRIEF::bitset &residual);


    // REFERENCE CODING
    size_t encodeReference(int reference, int numCandidates, EncodeContext &ctxt);
    int decodeReference(DecodeContext &kptCtxt, int numCandidates);


    // DEPTH CODING
    size_t IntraEncodeDepth(const float &fDepth, EncodeContext &ctxt);
    void IntraDecodeDepth(DecodeContext &modeCtxt, float &fDepth);

    size_t IntraEncodeQuantizedDepth(const float &fDepth, EncodeContext &ctxt);
    void IntraDecodeQuantizedDepth(DecodeContext &modeCtxt, float &fDepth);


    // INTER CODING
    size_t InterEncodeReferenceAC(int reference, ACEncodeContext &accontext);
    int InterDecodeReferenceAC(ACDecodeContext &accontext);

    size_t InterEncodeKeypoint(const cv::KeyPoint &refKeypoint, const cv::KeyPoint &currentKeypoint, ACEncodeContext &accontext, EncodeContext &context);
    void InterDecodeKeypoint(ACDecodeContext &accontext, DecodeContext &context, const cv::KeyPoint &refKeypoint, cv::KeyPoint &currentKeypoint);

    size_t InterEncodeResidual(const BRIEF::bitset &residual, ACEncodeContext &accontext);
    void InterDecodeResidual(ACDecodeContext &accontext, BRIEF::bitset &residual);

    BRIEF::bitset InterReconstructDescriptor(const BRIEF::bitset &referenceDescriptor, const BRIEF::bitset &residual);

    void KeyPointDiffToIndex(int dx, int dy, int octave, int &index);
    void IndexToKeyPointDiff(int index, int octave, int &x, int &y);





    int GetNumInterCandidates();

    // Fake coding to keep encoder and decoder in sync
    float fakeCodeDepth(const float &fDepth);
    cv::KeyPoint fakeCode(const cv::KeyPoint &keyPoint);

private:
    static long long imageId;

    CodingStats &mModel;
    Demo &mDemo;

    int mnAngleOffset;
    int mnOctaveOffset;


    // General
    std::vector<float> mScaleFactors;
    std::vector<int> mvPyramidWidth;
    std::vector<int> mvPyramidHeight;

    std::vector<float> mvBitsPyramidWidth;
    std::vector<float> mvBitsPyramidHeight;


    // Intra
    std::vector<int> mFreqIntraRes;
    std::vector<int> mFreqIntraBow;
    std::vector<cv::Mat> mFreqIntraPosX;
    std::vector<cv::Mat> mFreqIntraPosY;
    std::vector<float> mLutRIntra;
    std::vector<int> mFreqIntraOctave;
    std::vector<int> mFreqIntraAngle;
    //ljl
    std::vector<int> mFreqIntraRes_ljl;



    // Inter
    std::vector<int> mFreqInterRes;
    std::vector<int> mFreqInterAngleDiff;
    std::vector<int> mFreqInterOctaveDiff;
    std::vector<int> mFreqInterCandidate;
    std::vector<cv::Mat> mFreqInterKeyPoint;
    std::vector<cv::Mat> mPInterKeyPoint;
    std::vector<float> mLutRInter;



    // Depth
    std::vector<float> mvfDepthCodeBook;
    int mnDepthBits;




    // Feature settings
    int mImWidth;
    int mImHeight;

    int mAngleBins;
    float mAngleBinSize;
    unsigned int mBufferSize;
    unsigned int mInterReferenceImages;


    float mnBitsAngle;
    float mnBitsOctave;
    float mnBitsBow;


    // Control variables
    bool mbInterPred;
    bool mbStereo;
    bool mbMonoDepth;

    float mfFocalLength;
//    float mfBaseLineFocalLength;


    // Temporal variables
    unsigned long mCurrentImageId;
//    bool mbCurrentViewRight;


    int nThreads;
    std::vector<EncodeContext> vEncodeContext;
    std::vector<ACEncodeContext> vGlobalACCoderContext;

    std::vector<DecodeContext> vDecodeContext;
    std::vector<ACDecodeContext> vGlobalACDecoderContext;

    // Buffer
    ImgBufferEntry mCurrentImageBuffer;
    std::list<ImgBufferEntry> mLeftImageBuffer;//这里像存放了许多图片 用来做帧间相似性检索的
    std::list<ImgBufferEntry> mRightImageBuffer;
    
    //ljl
    long des_d;
    long des_d_sum=0;
    vector<long> des_d_all;
};

} // END NAMESPACE

#endif /* feature_coder_hpp */
