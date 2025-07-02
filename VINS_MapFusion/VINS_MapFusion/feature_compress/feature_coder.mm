//
//  feature_coder.cpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2023/4/26.
//  Copyright © 2023 zx. All rights reserved.
//

#include "feature_coder.hpp"

#include <chrono>
#include <omp.h>

#define MAX_NUM_FEATURES 2500

//#define ANDROID
//#define NO_DEPTH_Q


namespace LBFC2
{

long long FeatureCoder::imageId = 0;

FeatureCoder::FeatureCoder(Demo &demo, CodingStats &model, int imWidth, int imHeight,int angleBins,
        int bufferSize, bool inter, bool stereo, bool depth, float focalLength,  int threads)
: mModel(model), mDemo(demo), mImWidth(imWidth), mImHeight(imHeight), mAngleBins(angleBins),
  mBufferSize(bufferSize), mbInterPred(inter), mbStereo(stereo), mbMonoDepth(depth), mfFocalLength(focalLength)
{
//    mbCurrentViewRight = false;

    //实验用
    des_d_sum=0;
    
//mAngleBins 32
    mAngleBinSize = 360.0 / mAngleBins;

    // INTRA CODING
    mFreqIntraRes.resize(2);
    mFreqIntraRes[0] = (int)max(1, (int)round( mModel.p0_intra_ * (double)AC_PRECISION ) );//4090 3800
    mFreqIntraRes[1] = AC_PRECISION - mFreqIntraRes[0];//910 1200
    
//    mFreqIntraRes[0] = 3800;
//    mFreqIntraRes[1] = 1200;
    
    //ljl
//    mFreqIntraRes.resize(13);
//    for(int i=0;i<12;i++){
//        mFreqIntraRes[i] = 10;//4090
//    }
//    mFreqIntraRes[12] = 10;

    // Pyramid sizes for intra coding
    mvPyramidWidth.resize(mModel.mOctaves);//1
    mvPyramidHeight.resize(mModel.mOctaves);//1
    mvPyramidWidth[0] = mImWidth;
    mvPyramidHeight[0] = mImHeight;


    // Pyramid bits for intra coding
    mvBitsPyramidWidth.resize(mModel.mOctaves);
    mvBitsPyramidHeight.resize(mModel.mOctaves);
    mvBitsPyramidWidth[0] = log2(mvPyramidWidth[0]);
    mvBitsPyramidHeight[0] = log2(mvPyramidHeight[0]);

    mScaleFactors.resize(mModel.mOctaves);
    mScaleFactors[0] = 1.0;

    for( int d = 1; d < mModel.mOctaves; d++ )
    {
        // Pyramid sizes
        mScaleFactors[d] = mScaleFactors[d-1]*mModel.mScaleFactor;
        mvPyramidWidth[d] = ceil(((float) mImWidth) / mScaleFactors[d]);
        mvPyramidHeight[d] = ceil(((float) mImHeight) / mScaleFactors[d]);

        // Pyramid bits
        mvBitsPyramidWidth[d] = log2(mvPyramidWidth[d]);
        mvBitsPyramidHeight[d] = log2(mvPyramidHeight[d]);
    }


    // Intra keypoint costs
//    mnBitsOctave = log2(mLevels);
    mnBitsAngle = log2(mAngleBins);
    mnBitsBow = log2(mDemo.voc.size());

    mnAngleOffset = mModel.mAngleBins-1;
    mnOctaveOffset =  mModel.mOctaves-1;


    // Intra bow propabilities
    size_t voc_size = demo.voc.size();//977232
//    cout<<"字典的大小："<<voc_size<<endl;
    mFreqIntraBow.resize(voc_size);
    for( size_t i = 0; i < voc_size; i++ )
    {
        const double prob = 1.0 / voc_size;
        //这样子 永为1
        mFreqIntraBow[i] = (int)max(1, (int)round( prob * (double)AC_PRECISION ) );
//        cout<<"intraBow概率:"<<mFreqIntraBow[i]<<endl;
    }


    // Intra keypoint propabilities
    mFreqIntraPosX.resize(mModel.mOctaves);
    mFreqIntraPosY.resize(mModel.mOctaves);
    for( int d = 0; d < mModel.mOctaves; d++ )
    {
        mFreqIntraPosX[d] = cv::Mat(1, mvPyramidWidth[d], CV_32S, cv::Scalar::all(1 ));
        mFreqIntraPosY[d] = cv::Mat(1, mvPyramidHeight[d], CV_32S, cv::Scalar::all(1 ));
    }

    mFreqIntraOctave.resize(1);
    std::fill(mFreqIntraOctave.begin(), mFreqIntraOctave.end(), 1);

    mFreqIntraAngle.resize(mAngleBins);
    std::fill(mFreqIntraAngle.begin(), mFreqIntraAngle.end(), 1);


    // INTER CODING
    // Inter residual coding propabilities
    mFreqInterRes.resize(2);
    mFreqInterRes[0] = (int)max(1, (int)round( mModel.p0_inter_ * (double)AC_PRECISION ) );//4559
    mFreqInterRes[1] = AC_PRECISION - mFreqInterRes[0];//441
    

    mFreqInterAngleDiff.resize(mModel.pAngleDelta_.cols);
    for( int d = 0; d < mModel.pAngleDelta_.cols; d++ )
    {
        const float &prob = mModel.pAngleDelta_.at<float>(d);
        mFreqInterAngleDiff[d] = (int)max(1, (int)round( prob * (double)AC_PRECISION ) );
    }


    mFreqInterOctaveDiff.resize(mModel.pOctaveDelta_.cols);
    for( int d = 0; d < mModel.pOctaveDelta_.cols; d++ )
    {
        float prob = mModel.pOctaveDelta_.at<float>(d);
        mFreqInterOctaveDiff[d] = (int)max(1, (int)round( prob * (double)AC_PRECISION ) );
    }


    // Inter coding propabilities
    mFreqInterKeyPoint.resize(mModel.mOctaves);
    mPInterKeyPoint.resize(mModel.mOctaves);

    for( int d = 0; d < mModel.mOctaves; d++ )
    {
        mPInterKeyPoint[d] = mModel.pPosPerOctave_[d];
        mFreqInterKeyPoint[d] = cv::Mat(mPInterKeyPoint[d].rows, mPInterKeyPoint[d].cols, CV_32S, cv::Scalar::all(0));
        for( int x = 0; x <  mModel.pPosPerOctave_[d].cols; x++)
        {
            for( int y = 0; y <  mModel.pPosPerOctave_[d].rows; y++)
            {
                float prob = mPInterKeyPoint[d].at<float>(y,x);
                mFreqInterKeyPoint[d].at<int>(y,x) = (int)max(1, (int)round( prob * (double)AC_PRECISION ) );
            }
        }
    }


    
    // DEPTH CODING
    if( !mModel.mDepthCodebook.empty() )
    {
        for( int i = 0; i < mModel.mDepthCodebook.rows; i++ )
            mvfDepthCodeBook.push_back(mModel.mDepthCodebook.at<float>(i));
    }

    mnDepthBits = ceil(log2(mvfDepthCodeBook.size()));


    // Cost lookup tables //描述符区别
    mLutRIntra.resize(257);
    mLutRInter.resize(257);
    for( int d = 0; d < 257; d++ )
    {
        mLutRIntra[d] =  -((float)(256-d)) * log2(mModel.p0_intra_) - ((float) d) * log2(1.0 - mModel.p0_intra_);
        mLutRInter[d] =  -((float)(256-d)) * log2(mModel.p0_inter_) - ((float) d) * log2(1.0 - mModel.p0_inter_);
    }



    // Prepare coder
    mCurrentImageId = 0;
    nThreads = threads;
    vEncodeContext.resize(nThreads);
    vGlobalACCoderContext.resize(nThreads);

    for( int i = 0; i < nThreads; i++ )
        initEncoderModels(vGlobalACCoderContext[i]);//初始化编码器模型的数据

    vDecodeContext.resize(nThreads);
    vGlobalACDecoderContext.resize(nThreads);

    for( int i = 0; i < nThreads; i++ )
        initDecoderModels(vGlobalACDecoderContext[i]);
}



void FeatureCoder::encodeImageDepth( const std::vector<cv::KeyPoint> &kptsLeft, const std::vector<BRIEF::bitset> &descriptorsLeft,const std::vector<Eigen::Vector3d> &point_clouds,vector<uchar> &bitstream  )
{
    // Just pass through 这里是不对的，因为下标bowIndex_ljl 没给值 ，也不需要用到
    vector<int> bowIndex_ljl ;
    return encodeImageMono(kptsLeft, descriptorsLeft, point_clouds, bitstream,bowIndex_ljl );
}

void FeatureCoder::decodeImageDepth( const vector<uchar> &bitstream, std::vector<cv::KeyPoint> &keypoints, std::vector<BRIEF::bitset> &descriptors,
        std::vector<unsigned int> &visualWords, std::vector<float> &vfDepthValues )
{
    vector<int> bowIndex_ljl ;
    decodeImageMono( bitstream, keypoints, descriptors, visualWords,bowIndex_ljl );


    vfDepthValues = vector<float>(keypoints.size(),-1.0);
    for( size_t k = 0; k < keypoints.size(); k++ )
        vfDepthValues[k] = keypoints[k].size;
}

void FeatureCoder::encodeImageMono( const std::vector<cv::KeyPoint> &kptsLeft, const std::vector<BRIEF::bitset> &descriptorsLeft,const std::vector<Eigen::Vector3d> &point_clouds,vector<uchar> &bitstream, vector<int> &bowIndex_ljl )
{
    bitstream.clear();
    
    des_d=0;

    //TODO mLevels
//    mCurrentImageBuffer = ImgBufferEntry(mImWidth, mImHeight, 1);
//    mCurrentImageBuffer.mnImageId = imageId++;//TODO 这个或许可以换成关键帧的id 后期换？》

    // Calculate matching
    std::vector<int> maskMatch1(kptsLeft.size(), -1);
    std::vector<float> vfDepthValues(point_clouds.size(), -1.0);

    //暂时注释
    //并不是所有点都有深度 后期需要调整 两个方法：一个只发特征点，一个发3d点
//    if( mbMonoDepth )
//        //这里并没有给vfDepthValues赋值
//        matchStereoFeatures(kptsLeft, descriptorsLeft, vfDepthValues);

//    cout<<"feature_coder.mm 252"<<endl;
    // Pre-calculate decisions for left view为所有特征点分配一个决策
    std::vector<ModeDecision> decisionsLeft(kptsLeft.size());
#pragma omp parallel for
    //这里是 测试每一个特征点(根据花费的位数) 是帧内编码intra 还是帧间编码inter
    for( size_t i = 0; i < kptsLeft.size(); i++ )
    {
        //决策是根据所需位数bits 这时已经将特征点，差异描述符，参考bow下标，返回存到decisionsLeft
        decisionsLeft[i] = modeDecision(kptsLeft[i], descriptorsLeft[i]);
        decisionsLeft[i].keypointId = i;
        decisionsLeft[i].stereoMatch = false;
        
        bowIndex_ljl.push_back(decisionsLeft[i].visualWord);
    }

    int numFeatures = decisionsLeft.size();
    for( size_t i = 0; i < decisionsLeft.size(); i++ )
    {
        decisionsLeft[i].bufferIndex = i;
    }

//计算每个线程要处理多少个数据，处理哪几个下标的特征点
    int featPerThread = ceil(((float) numFeatures) / nThreads);
    std::vector<std::vector<unsigned int> > vLeftThreadDec(nThreads);
    for( size_t i = 0; i < decisionsLeft.size(); i++ )
    {
        ModeDecision &decision = decisionsLeft[i];
        int bufferIndex = decision.bufferIndex;
        int threadId = floor(((float) bufferIndex)  / featPerThread);
        vLeftThreadDec[threadId].push_back(i);
    }


//    mCurrentImageBuffer.allocateSpace(numFeatures);
//    std::vector<int> vLutKeyPointToBuffer(numFeatures, -1);

//    cout<<"编码的描述符数量："<<kptsLeft.size() <<endl;
    // Encode
#pragma omp parallel for
    for( int threadId = 0; threadId < nThreads; threadId++)
    {
        //TODO vEncodeContext这个似乎没赋值 resize会给空间，里面会有值了
        //最终的数据 编码在globalCoderContext和globalACCoderContext里面了
        EncodeContext &globalCoderContext = vEncodeContext[threadId];
        ACEncodeContext &globalACCoderContext = vGlobalACCoderContext[threadId];
        for( size_t j = 0; j < vLeftThreadDec[threadId].size(); j++ )
        {
            size_t i = vLeftThreadDec[threadId][j];
            const ModeDecision &decision = decisionsLeft[i];

            // Let's go
            const int &kptId = decision.keypointId;
            const cv::KeyPoint &keypoint = kptsLeft[kptId];
            const BRIEF::bitset &descriptor = descriptorsLeft[kptId];
            const float &fDepth = vfDepthValues[kptId];
//            cout<<"vfDepthValues.size="<<vfDepthValues.size()<<" , "<<fDepth<<endl;
            //应该是这里报错
            
            encodeFeature(decision, keypoint, descriptor, fDepth, globalCoderContext, globalACCoderContext);
//            vLutKeyPointToBuffer[kptId] = decision.bufferIndex;
        }
    }
    
    const int nCodedLeft = numFeatures;

//    mCurrentImageBuffer.AssignFeatures();
//    mLeftImageBuffer.push_back(mCurrentImageBuffer);
//    // Pop after right view
//    if( mLeftImageBuffer.size() > mBufferSize)
//        mLeftImageBuffer.pop_front();


    for( int threadId = 0; threadId < nThreads; threadId++)
    {
        vEncodeContext[threadId].finish();
        vGlobalACCoderContext[threadId].finish();
    }


    bitstream.resize(sizeof(EncodeInfo));
    EncodeInfo *info = (EncodeInfo *) &bitstream[0];
    info->numFeatures = nCodedLeft;
    info->fixedLengthSize = nThreads;



    for( int threadId = 0; threadId < nThreads; threadId++)
    {
        int offset = bitstream.size();
        bitstream.resize(offset + sizeof(ThreadInfo));
        ThreadInfo *info = (ThreadInfo *) &bitstream[offset];
        info->fixedLengthSize = vEncodeContext[threadId].bitstream.size();
        info->variableLengthSize = vGlobalACCoderContext[threadId].bitstream.size();
    }

    for( int threadId = 0; threadId < nThreads; threadId++)
    {
        bitstream.insert(bitstream.end(), vEncodeContext[threadId].bitstream.begin(), vEncodeContext[threadId].bitstream.end());
        bitstream.insert(bitstream.end(), vGlobalACCoderContext[threadId].bitstream.begin(), vGlobalACCoderContext[threadId].bitstream.end());
    }

    for( int threadId = 0; threadId < nThreads; threadId++)
    {
        vEncodeContext[threadId].clear();
        vGlobalACCoderContext[threadId].clear();
    }
//    cout<<"每帧描述符的差别量："<<des_d<<endl;
    //实验
    des_d_all.push_back(des_d);
//    cout<<"汇总每帧描述符的差别量：";
//    for(int a=0,b=des_d_all.size();a<b;a++){
//        cout<<des_d_all[a]<<" , ";
//    }
//    cout<<endl<<endl;
    des_d_sum+=des_d;
//    cout<<"数据库累积描述符的差别量："<<des_d_sum<<endl;

//    mCurrentImageId++;
}



void FeatureCoder::decodeImageMono( const vector<uchar> &bitstream, std::vector<cv::KeyPoint> &kptsLeft, std::vector<BRIEF::bitset> &descriptorsLeft, std::vector<unsigned int> &visualWords , std::vector<int> &bowIndex_ljl)
{
    EncodeInfo *info = (EncodeInfo *) &bitstream[0];
    const unsigned int nCodedLeft = info->numFeatures;
    const unsigned int numThreads = info->fixedLengthSize;

    vector< vector<int> > bowIndex_allAllocate;
    size_t offset = sizeof(EncodeInfo);

    std::vector<ThreadInfo> vInfo;
    for( unsigned int threadId = 0; threadId < numThreads; threadId++)
    {
        ThreadInfo *info = (ThreadInfo *) &bitstream[offset];
        offset += sizeof(ThreadInfo);
        vInfo.push_back(*info);
        
        vector<int> single_bowIndex;
        bowIndex_allAllocate.push_back(single_bowIndex);
    }


    vector<vector<uchar> > vvFLBitstream(numThreads);
    vector<list<uchar> > vAcBitstream(numThreads);


    std::vector<size_t> vOffsets = {offset};
    for( unsigned int threadId = 0; threadId < numThreads; threadId++)
    {
        size_t fixedLengthOffset = vInfo[threadId].fixedLengthSize;
        size_t variableLengthOffset = vInfo[threadId].variableLengthSize;
        offset += fixedLengthOffset + variableLengthOffset;
        vOffsets.push_back(offset);
    }

#pragma omp parallel for
    for( unsigned int threadId = 0; threadId < numThreads; threadId++)
    {
        const size_t offset = vOffsets[threadId];
        size_t fixedLengthOffset = vInfo[threadId].fixedLengthSize;
        size_t variableLengthOffset = vInfo[threadId].variableLengthSize;
        vvFLBitstream[threadId].assign(bitstream.begin() + offset, bitstream.begin() + offset + fixedLengthOffset);
        vAcBitstream[threadId].assign(bitstream.begin() + offset + fixedLengthOffset, bitstream.begin() + offset + fixedLengthOffset + variableLengthOffset);
    }


    for(unsigned int i = 0; i < numThreads; i++ )
    {
        vGlobalACDecoderContext[i].setBitstream(vAcBitstream[i]);
        vDecodeContext[i].clear();
        vDecodeContext[i].bitstream = vvFLBitstream[i];
    }


    // Left view
//    mbCurrentViewRight = false;
//    mCurrentImageBuffer = ImgBufferEntry(mImWidth, mImHeight, mLevels );
    //TODO
    mCurrentImageBuffer = ImgBufferEntry(mImWidth, mImHeight, 1 );
    mCurrentImageBuffer.allocateSpace(nCodedLeft);

//    cout<<"解码线程的数量："<<numThreads <<endl;
//    cout<<"得到待解码的 描述符数量："<<nCodedLeft<<endl;
    int featPerThreadLeft = ceil(((float) nCodedLeft) / numThreads);
    std::vector<std::vector<unsigned int> > vLeftFeatureIndex(numThreads);
    for( unsigned int i = 0; i < nCodedLeft; i++ )
    {
        int threadId = floor(((float) i) / featPerThreadLeft);
        vLeftFeatureIndex[threadId].push_back(i);
        
        
        bowIndex_allAllocate[threadId].push_back(bowIndex_ljl[i]);
    }


#pragma omp parallel for
    for( unsigned int threadId = 0; threadId < numThreads; threadId++)
    {
//        cout<<"现在是解码的线程id:"<<threadId<< " , "<<vLeftFeatureIndex[threadId].size()<<endl;
        // Let's go
        DecodeContext &globalDecoderContext = vDecodeContext[threadId];
        ACDecodeContext &globalACDecoderContext = vGlobalACDecoderContext[threadId];
        for( size_t j = 0; j < vLeftFeatureIndex[threadId].size(); j++ )
        {
            size_t i = vLeftFeatureIndex[threadId][j];
            unsigned int visualWord = 0;
            cv::KeyPoint keypoint;
            BRIEF::bitset descriptor;
            decodeFeature(keypoint, descriptor, visualWord, globalDecoderContext, globalACDecoderContext , bowIndex_allAllocate[threadId][j] );
            
            mCurrentImageBuffer.addFeature(i, keypoint, descriptor, visualWord);
        }
    }

//    kptsLeft = mCurrentImageBuffer.mvKeypoints;
    descriptorsLeft = mCurrentImageBuffer.mDescriptors;

//    mCurrentImageBuffer.AssignFeatures();
//    mLeftImageBuffer.push_back(mCurrentImageBuffer);
//
//
//
//
//    if( mLeftImageBuffer.size() > mBufferSize)
//        mLeftImageBuffer.pop_front();


}



unsigned int FeatureCoder::encodeFeature(const ModeDecision &decision, const cv::KeyPoint &keypoint,
        const BRIEF::bitset &descriptor, const float &fDepth, EncodeContext &globalCoderContext,
        ACEncodeContext &globalACCoderContext )
{
    const unsigned int bits_start = globalCoderContext.bits() + globalACCoderContext.bits();


    // Encode mode
    encodeMode(decision.mode, globalCoderContext);


    // Intra coding
    if( decision.mode == CodingMode::INTRA)
    {
//        {
//        我们的方案测试
            //暂时所有数据的 存在这个里面
//            std::vector<unsigned char> bitstream(20);//1个字节 8位
            //发送匹配的bow下标 根据字典的长度，确定需要多少位 20位(最大20位能表达了)
//            IntraEncodeBowIndex(decision.visualWord, bitstream);
            //发送描述符差异
//            std::vector<unsigned char> bitstream;
//        IntraEncodeBowAC(decision.visualWord, globalACCoderContext);//会出现 bad_alloc 怀疑是无限增长内存
        IntraEncodeResidual(decision.residual, globalACCoderContext);//标记 现在暂时是这个
//            IntraEncodeBowResidual(decision.residual,bitstream);
            //发送特征点(这个暂时没压缩) x y response class_id(后面两个需要测试 是不是不变的 不变不发送)
//        }
        
//        哈夫曼编码
//        IntraEncodeResidual_haffman(decision.residual, globalACCoderContext);
        
        
//        {
            // Encode Global
//别人的方案测试
//            cout<<"IntraEncodeBowAC 481"<<endl;
//            IntraEncodeBowAC(decision.visualWord, globalACCoderContext);
//            cout<<"IntraEncodeKeyPointAC 483"<<endl;
//            IntraEncodeKeyPointAC(keypoint, globalACCoderContext);

//            压缩描述符残差
//            IntraEncodeResidual(decision.residual, globalACCoderContext);


            // Add depth information
//暂时注释
//                if( mbMonoDepth )
//                {
//#ifdef NO_DEPTH_Q
//                    IntraEncodeDepth(fDepth, globalCoderContext);
//#else
//                    cout<<"IntraEncodeQuantizedDepth 498"<<endl;
//                    IntraEncodeQuantizedDepth(fDepth, globalCoderContext);
//#endif
//                }
//            const cv::KeyPoint &decKeypoint = fakeCode(keypoint);
//            const float qfDepth = fakeCodeDepth(fDepth);
//            mCurrentImageBuffer.addFeature(decision.bufferIndex, decKeypoint, descriptor, qfDepth);
//        }
    }
//    else if( decision.mode == CodingMode::INTER )
//    {
//        {
//            // Encode global
//            const int &referenceId = decision.candidate.candidateId;
//
//
//            // Keypoint
//            std::list<ImgBufferEntry>::const_iterator it;
////            if( !mbCurrentViewRight )
//                it = mLeftImageBuffer.begin();
////            else
////                it = mRightImageBuffer.begin();
//
//            std::advance(it, decision.candidate.imageId);
//            const cv::KeyPoint &refKeypoint = it->mvKeypoints[decision.candidate.keypointId];
//
////#ifdef ANDROID
////            stats.interEncStats.bitsReference += encodeReference(referenceId, mInterReferenceImages, globalCoderContext);
////#else
//            InterEncodeReferenceAC(referenceId, globalACCoderContext);
////#endif
//            InterEncodeKeypoint(refKeypoint, keypoint, globalACCoderContext, globalCoderContext);
//            InterEncodeResidual(decision.residual, globalACCoderContext);
//
////            if( !mbCurrentViewRight )
////            {
//                // Add depth information, if required - intra-only for now
//                if( mbMonoDepth )
//                {
//#ifdef NO_DEPTH_Q
//                    IntraEncodeDepth(fDepth, globalCoderContext);
//#else
//                    IntraEncodeQuantizedDepth(fDepth, globalCoderContext);
//#endif
//                }
////            }
//
//            const cv::KeyPoint &decKeypoint = fakeCode(keypoint);
//            const float qfDepth = fakeCodeDepth(fDepth);
//            mCurrentImageBuffer.addFeature(decision.bufferIndex, decKeypoint, descriptor, qfDepth);
//        }
//    }
    

    const unsigned int bits_end = globalCoderContext.bits() + globalACCoderContext.bits();


    return bits_end - bits_start;
}



void FeatureCoder::decodeFeature(cv::KeyPoint &decKeypoint, BRIEF::bitset &recDescriptor, unsigned int &visualWord,
        DecodeContext &globalDecoderContext, ACDecodeContext &globalACDecoderContext,int  visualWord_ljl)
{

    CodingMode mode;
    decodeMode(globalDecoderContext, mode);



    if( mode == CodingMode::INTRA)
    {
        // Decode
        BRIEF::bitset residual;
//#ifdef ANDROID
//        IntraDecodeBow(globalDecoderContext, visualWord);
//        IntraDecodeKeyPoint(globalDecoderContext, decKeypoint);
//#else
//        IntraDecodeBowAC(globalACDecoderContext, visualWord);//这个是下标 暂时注释
//        IntraDecodeKeyPointAC(globalACDecoderContext, decKeypoint);
//#endif
        IntraDecodeResidual(globalACDecoderContext, residual);
        recDescriptor = IntraReconstructDescriptor(visualWord_ljl, residual);
//        recDescriptor =residual;
//        暂时注释，因为 并没有把下标压缩进去
//        recDescriptor = IntraReconstructDescriptor(visualWord, residual);

        // Depth data !mbCurrentViewRight &&
//        if( mbMonoDepth )
//        {
//            decKeypoint.size = -1.0;
//#ifdef NO_DEPTH_Q
//            IntraDecodeDepth(globalDecoderContext, decKeypoint.size);
//#else
//            IntraDecodeQuantizedDepth(globalDecoderContext, decKeypoint.size);
//#endif
//        }


        decKeypoint.class_id = CodingMode::INTRA;
    }
//    else if( mode == CodingMode::INTER )
//    {
//        // Decode
//        BRIEF::bitset residual;
//#ifdef ANDROID
//        const int recReferenceId = decodeReference(globalDecoderContext, mInterReferenceImages);
//#else
//        const int recReferenceId = InterDecodeReferenceAC(globalACDecoderContext);
//#endif
//        // Has to  be N-Sync with  the encoder
//        std::list<ImgBufferEntry>::const_iterator it, itEnd;
////        if( mbCurrentViewRight )
////        {
////            it = mRightImageBuffer.begin();
////            itEnd = mRightImageBuffer.end();
////        }
////        else
////        {
//            it = mLeftImageBuffer.begin();
//            itEnd = mLeftImageBuffer.end();
////        }
//
//        int keypointId = -1;
//        int numKeypoints = 0;
//        for(; it != itEnd; it++ )
//        {
//            if( recReferenceId >= numKeypoints && recReferenceId < numKeypoints + (int) it->mvKeypoints.size())
//            {
//                keypointId = recReferenceId - numKeypoints;
//                break;
//            }
//
//            numKeypoints += it->mvKeypoints.size();
//        }
//
//
//        const cv::KeyPoint &recRefKeypoint = it->mvKeypoints[keypointId];
//        const BRIEF::bitset &recRefDescriptor =  it->mDescriptors[keypointId];
//
//
//        InterDecodeKeypoint(globalACDecoderContext, globalDecoderContext, recRefKeypoint, decKeypoint);
//        InterDecodeResidual(globalACDecoderContext, residual);
//        recDescriptor = InterReconstructDescriptor(recRefDescriptor, residual);
//
//
//        // Depth data !mbCurrentViewRight &&
//        if(  mbMonoDepth )
//        {
//            decKeypoint.size = -1.0;
//#ifdef NO_DEPTH_Q
//            IntraDecodeDepth(globalDecoderContext, decKeypoint.size);
//#else
//            IntraDecodeQuantizedDepth(globalDecoderContext, decKeypoint.size);
//#endif
//        }
//
//        decKeypoint.class_id = CodingMode::INTER;
//        mDemo.voc.transform(recDescriptor, visualWord);
//    }
    

}



void FeatureCoder::initEncoderModels( ACEncodeContext &accontext )
{
    // Init models
    
    // INTRA
    ac_model_init (&accontext.acm_bow, mDemo.voc.size(), &mFreqIntraBow[0], 0);
    
    ac_model_init (&accontext.acm_intra_desc, 2, &mFreqIntraRes[0], 0);
//    cout<<"mFreqIntraRes[0]="<<mFreqIntraRes[0]<<endl;
//    cout<<"mFreqIntraRes[1]="<<mFreqIntraRes[1]<<endl;
    //ljl 2
//    ac_model_init (&accontext.acm_intra_desc, 13, &mFreqIntraRes[0], 0);
    ac_model_init (&accontext.acm_intra_angle, mAngleBins, &mFreqIntraAngle[0], 0);
//    ac_model_init (&accontext.acm_intra_octave, mLevels, &mFreqIntraOctave[0], 0);
    //TODO
    ac_model_init (&accontext.acm_intra_octave, 1, &mFreqIntraOctave[0], 0);

//    accontext.v_acm_intra_kpt_x.resize(mLevels);
//    accontext.v_acm_intra_kpt_y.resize(mLevels);
    //TODO
    accontext.v_acm_intra_kpt_x.resize(1);
    accontext.v_acm_intra_kpt_y.resize(1);

    //TODO
//    for( int octave = 0; octave < mLevels; octave++ )
    for( int octave = 0; octave < 1; octave++ )
    {
        ac_model_init (&accontext.v_acm_intra_kpt_x[octave], mFreqIntraPosX[octave].cols, (int *) mFreqIntraPosX[octave].data, 0);
        ac_model_init (&accontext.v_acm_intra_kpt_y[octave], mFreqIntraPosY[octave].cols, (int *) mFreqIntraPosY[octave].data, 0);
    }
    

    // INTER
    ac_model_init (&accontext.acm_inter_desc, 2, &mFreqInterRes[0], 0);
    ac_model_init (&accontext.acm_inter_angle, mFreqInterAngleDiff.size(), (int *) &mFreqInterAngleDiff[0], 0);
    ac_model_init (&accontext.acm_inter_octave, mFreqInterOctaveDiff.size(), (int *) &mFreqInterOctaveDiff[0], 0);

    //TODO
//    accontext.v_acm_inter_kpt.resize(mLevels);
//    for( int octave = 0; octave < mLevels; octave++ )
        accontext.v_acm_inter_kpt.resize(1);
        for( int octave = 0; octave < 1; octave++ )
    {
        const int inter_range = mFreqInterKeyPoint[octave].rows*mFreqInterKeyPoint[octave].cols;
        ac_model_init (&accontext.v_acm_inter_kpt[octave], inter_range, (int *) mFreqInterKeyPoint[octave].data, 0);
    }

    
}



void FeatureCoder::initDecoderModels( ACDecodeContext &accontext )
{
//    cout<<"初始解码器"<<endl;
    // INTRA
    ac_model_init (&accontext.acm_bow, mDemo.voc.size(), &mFreqIntraBow[0], 0);
    ac_model_init (&accontext.acm_intra_desc, 2, &mFreqIntraRes[0], 0);
    ac_model_init (&accontext.acm_intra_angle, mAngleBins, &mFreqIntraAngle[0], 0);
//    ac_model_init (&accontext.acm_intra_octave, mLevels, &mFreqIntraOctave[0], 0);
//
//    accontext.v_acm_intra_kpt_x.resize(mLevels);
//    accontext.v_acm_intra_kpt_y.resize(mLevels);
//
//    for( int octave = 0; octave < mLevels; octave++ )
    //TODO
    ac_model_init (&accontext.acm_intra_octave, 1, &mFreqIntraOctave[0], 0);

    accontext.v_acm_intra_kpt_x.resize(1);
    accontext.v_acm_intra_kpt_y.resize(1);

    for( int octave = 0; octave < 1; octave++ )
    {
        ac_model_init (&accontext.v_acm_intra_kpt_x[octave], mFreqIntraPosX[octave].cols, (int *) mFreqIntraPosX[octave].data, 0);
        ac_model_init (&accontext.v_acm_intra_kpt_y[octave], mFreqIntraPosY[octave].cols, (int *) mFreqIntraPosY[octave].data, 0);
    }

//    cout<<"初始解码器INTRA"<<endl;
    // INTER
    ac_model_init (&accontext.acm_inter_desc, 2, &mFreqInterRes[0], 0);
    ac_model_init (&accontext.acm_inter_angle, mFreqInterAngleDiff.size(), (int *) &mFreqInterAngleDiff[0], 0);
    ac_model_init (&accontext.acm_inter_octave, mFreqInterOctaveDiff.size(), (int *) &mFreqInterOctaveDiff[0], 0);

//    accontext.v_acm_inter_kpt.resize(mLevels);
//    for( int octave = 0; octave < mLevels; octave++ )
    //TODO
        accontext.v_acm_inter_kpt.resize(1);
        for( int octave = 0; octave < 1; octave++ )
    {
        const int inter_range = mFreqInterKeyPoint[octave].rows*mFreqInterKeyPoint[octave].cols;
        ac_model_init (&accontext.v_acm_inter_kpt[octave], inter_range, (int *) mFreqInterKeyPoint[octave].data, 0);
    }
//    cout<<"初始解码器INTER"<<endl;

}


//与bow区别的intra编码 编码一个特征点所需要的花销
float FeatureCoder::intraCosts( const cv::KeyPoint &currentKpt, const BRIEF::bitset &descriptor, unsigned int &visualWord, BRIEF::bitset &intraResidualMat)
{
//    cout<<"单词总数量="<<mDemo.voc.size()<<endl;//977232
    mDemo.voc.transform(descriptor, visualWord);
    const BRIEF::bitset &visualWordDesc = mDemo.voc.getWord(visualWord);

    //TODO 这里测试用 后期删一删
    intraResidualMat  =visualWordDesc^descriptor;
    
//    cout<<"汉明距离为："<<intraResidualMat.count()<<endl;
    //    cv::bitwise_xor(visualWordDesc, descriptor, intraResidualMat);
//    const int d = cv::norm(intraResidualMat, cv::NORM_HAMMING);
    
    const int d =intraResidualMat.count();//汉明距离
//    cout<<"intraResidualMat.size()="<<intraResidualMat.size()<<endl;
    des_d+=d;
//    float R_intra_res = mLutRIntra[d];//描述符的区别
//    const int &octave = currentKpt.octave;//这里是永为0
//    //这里应该会越界
//    assert(octave==0);
//    const float &nbits_x = mvBitsPyramidWidth[octave];
//    const float &nbits_y = mvBitsPyramidHeight[octave];
//    const float R_intra_kpt = mnBitsAngle + mnBitsOctave + nbits_x + nbits_y;
//
//    return mnBitsBow + R_intra_res + R_intra_kpt;
    
    return d;
}



float FeatureCoder::interCandidateSelection( const cv::KeyPoint &currentKpt, const BRIEF::bitset &descriptor, Candidate &cand)
{
    // Search best reference keypoint
    std::list<ImgBufferEntry>::iterator it;
    std::list<ImgBufferEntry>::iterator itEnd;

    //这个是已有的所有图片
//        it = mLeftImageBuffer.begin();
//        itEnd = mLeftImageBuffer.end();


    std::list<ImgBufferEntry>::iterator bestIt;
    float bestR = std::numeric_limits<float>::max();
    int bestIdx = -1;
    int bestImg = -1;
    int bestReference = -1;

    int img = 0;
    const float R_inter_ref = ceil(log2(MAX_NUM_FEATURES));

    unsigned int candidatesCount = 0;
//    for( ; it != itEnd; it++ )
//    {
        //这里应该 1. 把带匹配的描述符转为单词，2. 根据检索树能得到该单词 匹配的其它特征点
//    int visualWord=-1;
//    mDemo.voc.transform(descriptor, visualWord);
//    mDemo.detector.
//
//        const std::vector<unsigned int> &vIndices = it->GetFeaturesInArea(currentKpt.pt.x, currentKpt.pt.y, mModel.mSearchRange, currentKpt.octave-1, currentKpt.octave+1);
//
//        //这里应该遍历那些特征点 暴力or 其它策略 待考虑
//        for( size_t p = 0; p < vIndices.size(); p++ )
//        {
//            BRIEF::bitset residual;
//
////            cv::bitwise_xor(it->mDescriptors[ vIndices[p] ], descriptor, residual);
//            residual  =it->mDescriptors[ vIndices[p] ]^descriptor;
//            const int dist = residual.count();
//
//
//
//            const cv::KeyPoint &refKeypoint = it->mvKeypoints[vIndices[p]];
//            const int &octave = currentKpt.octave;
//
//            int refAngleBin = floor(refKeypoint.angle / mAngleBinSize);
//            int curAngleBin = floor(currentKpt.angle / mAngleBinSize);
//
//
//            int angleDiff = curAngleBin - refAngleBin + mnAngleOffset;
//            assert( angleDiff >= 0 && angleDiff< mModel.pAngleDelta_.cols);
//
//            float R_angleDiff = -log2(mModel.pAngleDelta_.at<float>(angleDiff));
//
//
//            // Octave coding
//            int octaveDiff = currentKpt.octave - refKeypoint.octave + mnOctaveOffset;
//            float R_inter_octave = -log2(mModel.pOctaveDelta_.at<float>(octaveDiff));
//
//            // When octave same use relative coding
//            const int sRefx = round(refKeypoint.pt.x / mScaleFactors[octave]);
//            const int sRefy = round(refKeypoint.pt.y / mScaleFactors[octave]);
//
//            const int sCurX = round(currentKpt.pt.x / mScaleFactors[octave]);
//            const int sCurY = round(currentKpt.pt.y / mScaleFactors[octave]);
//
//            const int dx = sCurX - sRefx;
//            const int dy = sCurY - sRefy;
//
//            const int tdx = dx + (mFreqInterKeyPoint[octave].cols-1)/2;
//            const int tdy = dy + (mFreqInterKeyPoint[octave].rows-1)/2;
//
//            int index;
//            KeyPointDiffToIndex(tdx, tdy, octave, index);
//
//            const float R_inter_res = mLutRInter[dist];
//
//            const float R_inter_xy = -log2(mPInterKeyPoint[octave].at<float>(index));
//            const float R_inter_kpt = R_angleDiff + R_inter_octave + R_inter_xy;
//
//            // Skip mode
//            float R_inter = R_inter_kpt + R_inter_res + R_inter_ref;
//            if( (dist < 5) && (dx == 0) && (dy == 0) && (curAngleBin == refAngleBin) && (currentKpt.octave == refKeypoint.octave) )
//            {
//                cand.skipMode = true;
//                R_inter = R_inter_ref;
//            }
//
//
//            if( R_inter < bestR )
//            {
//                bestR = R_inter;
//                bestIdx = vIndices[p];
//                bestImg = img;
//                bestReference = candidatesCount + vIndices[p];
//                cand.residual = residual;
//            }
//        }
//
//        candidatesCount +=  it->mvKeypoints.size();
//        img++;
////    }
//
//    cand.imageId = bestImg;
//    cand.keypointId = bestIdx;
//    cand.candidateId = bestReference;
//    cand.numCandidates = MAX_NUM_FEATURES;
//
//    if( cand.keypointId == -1 )
//        return std::numeric_limits<float>::max();

    return bestR;
}




int FeatureCoder::matchStereoFeatures(const std::vector<cv::KeyPoint> &kptsLeft, const std::vector<BRIEF::bitset> &descriptorsLeft,std::vector<float> &vfDepthValues)
{
//    ImgBufferEntry leftView(mImWidth, mImHeight, mLevels);
    //TODO
    ImgBufferEntry leftView(mImWidth, mImHeight, 1);
    //这里相当于 把原来尺寸，像神经网络一样 缩放到小的尺寸
    //然后在mGrid存储 特征点的总数量
    leftView.addFeatures(kptsLeft, descriptorsLeft, vfDepthValues);
    //在mGrid存储 特征点索引
    //mvRowIndices 在周围几个区域都存储特征点索引
    leftView.AssignFeatures();


//    int matches = 0;
    
    return 0;
}



ModeDecision FeatureCoder::modeDecision( const cv::KeyPoint &currentKpt, const BRIEF::bitset &descriptor, int stereoBufferId )
{
    unsigned int visualWord;
   
    BRIEF::bitset intraResidualMat;

    float R_intra = intraCosts(currentKpt, descriptor, visualWord, intraResidualMat);
    float R_inter = std::numeric_limits<float>::max();

//    Candidate interCandidate;
//
//    //第一次mbInterPred应该等于false 因为没有前一个帧 可以做比较。但是这里是直接表示可以采用这种方式
//    if( mbInterPred )
//        R_inter = interCandidateSelection(currentKpt, descriptor, interCandidate);
    

    ModeDecision decision;
    if( R_intra <= R_inter  )
    {
        decision.visualWord = visualWord;//bow下标 index
        decision.mode = CodingMode::INTRA;
        decision.residual = intraResidualMat;//描述符的差别 与bow
        decision.rate = R_intra;//这个是发送这个特征点 所需要的字节数
    }
//    if( R_inter <= R_intra  )
//    {
//        decision.mode = CodingMode::INTER;
//        if( interCandidate.skipMode )
//            decision.mode = CodingMode::INTER_SKIP;
//
//        decision.residual = interCandidate.residual;
//        decision.candidate = interCandidate;
//        decision.rate = R_inter;//这个是发送这个特征点 所需要的字节数
//    }

    return decision;
}



size_t FeatureCoder::encodeMode(const CodingMode &mode,  EncodeContext &ctxt)
{
    int nMode = 0;//intra
    if( mode == CodingMode::INTER )
        nMode = 1;
    if( mode == CodingMode::INTER_SKIP )
        nMode = 2;
    if( mode == CodingMode::STEREO_PRED )
        nMode = 3;

    const int nBitsMode = 2;
    for( int i = 0; i < nBitsMode; i++)
    {
        ctxt.cur_bit = ( nMode >> (nBitsMode - i - 1) ) & 0x0001;
        // update the 8-bits buffer 这里其实就是改buffer
        ctxt.buffer |= ctxt.cur_bit << ctxt.bit_idx;
        ctxt.bit_idx--;

        // when the buffer is full, append it to the vector; then reset the buffer
        if (ctxt.bit_idx<0){
            ctxt.bit_idx = 7;
            ctxt.bitstream.push_back(ctxt.buffer);
            ctxt.buffer = 0;
        }
    }

    return nBitsMode;
}



size_t FeatureCoder::decodeMode(DecodeContext &ctxt, CodingMode &mode)
{
    int nMode = 0;
    const int nBitsMode = 2;
    for( int i = 0; i < nBitsMode; i++ )
    {
        // reset bit counter
        if(ctxt.bit_idx<0){
            ctxt.bit_idx = 7;
            ctxt.cur_byte = ctxt.bitstream[ctxt.byte_idx];
            ctxt.byte_idx++;
        }
        // read the current bit
        ctxt.cur_bit = (ctxt.cur_byte >> ctxt.bit_idx) & 0x01;
        ctxt.bit_idx--;

        nMode |= (ctxt.cur_bit << (nBitsMode - i - 1) );
    }

    if( nMode == 0)
        mode = CodingMode::INTRA;
    else if (nMode == 1 )
        mode = CodingMode::INTER;
    else if (nMode == 2 )
        mode = CodingMode::INTER_SKIP;
    else if( nMode == 3 )
        mode = CodingMode::STEREO_PRED;

    return 1;
}



size_t FeatureCoder::IntraEncodeDepth(const float &fDepth, EncodeContext &ctxt)
{
    int nDepth = (*((int*) &fDepth));
    const int nBitsDepth = sizeof(float)*8;
    for( int i = 0; i < nBitsDepth; i++)
    {
        ctxt.cur_bit = ( nDepth >> (nBitsDepth - i - 1) ) & 0x0001;
        // update the 8-bits buffer
        ctxt.buffer |= ctxt.cur_bit << ctxt.bit_idx;
        ctxt.bit_idx--;

        // when the buffer is full, append it to the vector; then reset the buffer
        if (ctxt.bit_idx<0){
            ctxt.bit_idx = 7;
            ctxt.bitstream.push_back(ctxt.buffer);
            ctxt.buffer = 0;
        }
    }

    return nBitsDepth;
}



void FeatureCoder::IntraDecodeDepth(DecodeContext &ctxt, float &fDepth)
{
    int *depth = (int *) &fDepth;
    (*depth) = 0;

    const int nBitsDepth = sizeof(float)*8;
    for( int i = 0; i < nBitsDepth; i++ )
    {
        // reset bit counter
        if(ctxt.bit_idx<0){
            ctxt.bit_idx = 7;
            ctxt.cur_byte = ctxt.bitstream[ctxt.byte_idx];
            ctxt.byte_idx++;
        }
        // read the current bit
        ctxt.cur_bit = (ctxt.cur_byte >> ctxt.bit_idx) & 0x01;
        ctxt.bit_idx--;

        (*depth) |= (ctxt.cur_bit << (nBitsDepth - i - 1) );
    }
}



size_t FeatureCoder::IntraEncodeQuantizedDepth(const float &fDepth, EncodeContext &ctxt)
{
    // Signal if depth is comming
    ctxt.cur_bit = 0;
    if( (fDepth > 0) && !isinf(fDepth) )
        ctxt.cur_bit = 1;

    // update the 8-bits buffer
    ctxt.buffer |= ctxt.cur_bit << ctxt.bit_idx;
    ctxt.bit_idx--;

    // when the buffer is full, append it to the vector; then reset the buffer
    if (ctxt.bit_idx<0){
        ctxt.bit_idx = 7;
        ctxt.bitstream.push_back(ctxt.buffer);
        ctxt.buffer = 0;
    }


    if( ctxt.cur_bit == 0 )
        return 1;


    // Log search
    size_t idx = Utils::findNearestNeighbourIndex(fDepth, mvfDepthCodeBook);

    for( int i = 0; i < mnDepthBits; i++)
    {
        ctxt.cur_bit = ( idx >> (mnDepthBits - i - 1) ) & 0x0001;
        // update the 8-bits buffer
        ctxt.buffer |= ctxt.cur_bit << ctxt.bit_idx;
        ctxt.bit_idx--;

        // when the buffer is full, append it to the vector; then reset the buffer
        if (ctxt.bit_idx<0){
            ctxt.bit_idx = 7;
            ctxt.bitstream.push_back(ctxt.buffer);
            ctxt.buffer = 0;
        }
    }

    return mnDepthBits + 1;
}



void FeatureCoder::IntraDecodeQuantizedDepth(DecodeContext &ctxt, float &fDepth)
{
    // reset bit counter
    if(ctxt.bit_idx<0){
        ctxt.bit_idx = 7;
        ctxt.cur_byte = ctxt.bitstream[ctxt.byte_idx];
        ctxt.byte_idx++;
    }
    // read the current bit
    ctxt.cur_bit = (ctxt.cur_byte >> ctxt.bit_idx) & 0x01;
    ctxt.bit_idx--;

    // Check if deph available
    if( ctxt.cur_bit == 0 )
        return;


    size_t idx = 0;
    for( int i = 0; i < mnDepthBits; i++ )
    {
        // reset bit counter
        if(ctxt.bit_idx<0){
            ctxt.bit_idx = 7;
            ctxt.cur_byte = ctxt.bitstream[ctxt.byte_idx];
            ctxt.byte_idx++;
        }
        // read the current bit
        ctxt.cur_bit = (ctxt.cur_byte >> ctxt.bit_idx) & 0x01;
        ctxt.bit_idx--;

        idx |= (ctxt.cur_bit << (mnDepthBits - i - 1) );
    }

    // Lookup depth
    fDepth = mvfDepthCodeBook[idx];
}



size_t FeatureCoder::IntraEncodeBow(unsigned int visualWord,  EncodeContext &bowCtxt)
{
    const int bitsBow = ceil(mnBitsBow);
    for( int i = 0; i < bitsBow; i++)
    {
        bowCtxt.cur_bit = ( visualWord >> (bitsBow - i - 1) ) & 0x0001;        // update the 8-bits buffer
        bowCtxt.buffer |= bowCtxt.cur_bit << bowCtxt.bit_idx;
        bowCtxt.bit_idx--;

        // when the buffer is full, append it to the vector; then reset the buffer
        if (bowCtxt.bit_idx<0){
            bowCtxt.bit_idx = 7;
            bowCtxt.bitstream.push_back(bowCtxt.buffer);
            bowCtxt.buffer = 0;
        }
    }

    return mnBitsBow;
}



void FeatureCoder::IntraDecodeBow(DecodeContext &bowCtxt, unsigned int &visualWord )
{
    const int bitsBow = ceil(mnBitsBow);
    visualWord = 0;
    for( int i = 0; i < bitsBow; i++ )
    {
        // reset bit counter
        if(bowCtxt.bit_idx<0){
            bowCtxt.bit_idx = 7;
            bowCtxt.cur_byte = bowCtxt.bitstream[bowCtxt.byte_idx];
            bowCtxt.byte_idx++;
        }
        // read the current bit
        bowCtxt.cur_bit = (bowCtxt.cur_byte >> bowCtxt.bit_idx) & 0x01;
        bowCtxt.bit_idx--;

        visualWord |= (bowCtxt.cur_bit << (bitsBow - i - 1) );
    }

    assert( visualWord <= mDemo.voc.size() );
}



size_t FeatureCoder::IntraEncodeBowAC(unsigned int visualWord,  ACEncodeContext &accontext)
{
    cout<<"visualWord="<<visualWord<<endl;
    const size_t bits_start = accontext.bits();
    ac_encode_symbol(&accontext.ace, &accontext.acm_bow, visualWord);
    
//    vector<unsigned char> v;
//    for(;;){
//        unsigned int a_single=visualWord/10;
//        if(a_single==0){
//            if(visualWord%10==0){
//                break;
//            }
//        }
//        v.push_back(visualWord%10);
//        visualWord=a_single;
//    }
//    accontext.bitstream.push_back(v.size());
//    accontext.bitstream.push_back(v);
//    accontext.ace.total_bits+=(v.size()+1)*8;
    
    return accontext.bits() - bits_start;
}


void FeatureCoder::IntraEncodeBowIndex(unsigned int visualWord,  std::vector<unsigned char> &bitstream)
{
    int length = 19;
  
    //循环除2，把余数存储在数组中
    while (visualWord / 2) {
        bitstream[length] =  (unsigned char)visualWord % 2;
        length--;
        visualWord = visualWord / 2;
    }
    //存储最后一个余数
    bitstream[length] =  (unsigned char)visualWord;
  
}


    
void FeatureCoder::IntraEncodeBowResidual(const BRIEF::bitset &residual, std::vector<unsigned char> &bitstream)
{
    //不同为1 告知了多少残差
    //总共256位，9层 8位。 29位不同，可以用dbow表示 肯定比256要少
    
    
//    const size_t bits_start = accontext.bits();

    //暂时注释
//    for( int d = 0; d < residual.size(); d++ )
//    {
//        const int &current_bit =residual[d];
//        ac_encode_symbol(&accontext.ace, &accontext.acm_intra_desc, current_bit);
//    }


//    double p_last[2]={0,1};
//    int d_last=-1;
//    for( int d = 0; d < residual.size(); d++ )
//    {
//        if(residual[d]==1){
//            double *p=ac_encode_context(d,d_last,p_last);
//            p_last[0]=p[0];
//            p_last[1]=p[1];
//            delete p;
//            d_last=d;
//        }
//    }
//    cout<<"p_last测试:"<<p_last[0]<<endl;

    return ;
}


void FeatureCoder::IntraDecodeBowAC(ACDecodeContext &accontext, unsigned int &visualWord )
{
    // Setup decoder for descriptor
    visualWord = ac_decode_symbol(&accontext.acd, &accontext.acm_bow);
    
}



size_t FeatureCoder::IntraEncodeKeyPoint(const cv::KeyPoint &keypoint, EncodeContext &context)
{
    const int &octave = keypoint.octave;

    int angleBin = floor(keypoint.angle / mAngleBinSize);

    const int bitsAngle = ceil(mnBitsAngle);
    for( int i = 0; i < bitsAngle; i++)
    {
        context.cur_bit = ( angleBin >> (bitsAngle - i - 1) ) & 0x0001;
        // update the 8-bits buffer
        context.buffer |= context.cur_bit << context.bit_idx;
        context.bit_idx--;

        // when the buffer is full, append it to the vector; then reset the buffer
        if (context.bit_idx<0){
            context.bit_idx = 7;
            context.bitstream.push_back(context.buffer);
            context.buffer = 0;
        }
    }

    const int bitsOctave = ceil(mnBitsOctave);
    for( int i = 0; i < bitsOctave; i++)
    {
        context.cur_bit = ( octave >> (bitsOctave - i - 1) ) & 0x0001;
        // update the 8-bits buffer
        context.buffer |= context.cur_bit << context.bit_idx;
        context.bit_idx--;

        // when the buffer is full, append it to the vector; then reset the buffer
        if (context.bit_idx<0){
            context.bit_idx = 7;
            context.bitstream.push_back(context.buffer);
            context.buffer = 0;
        }
    }

    // Resize Keypoints to integer resolution
    const int nbits_x = ceil(mvBitsPyramidWidth[octave]);
    const int nbits_y = ceil(mvBitsPyramidHeight[octave]);

    int qx = round(keypoint.pt.x / mScaleFactors[octave]);
    int qy = round(keypoint.pt.y / mScaleFactors[octave]);

    for( int i = 0; i < nbits_x; i++)
    {
        context.cur_bit = ( qx >> (nbits_x - i - 1) ) & 0x0001;
        // update the 8-bits buffer
        context.buffer |= context.cur_bit << context.bit_idx;
        context.bit_idx--;

        // when the buffer is full, append it to the vector; then reset the buffer
        if (context.bit_idx<0){
            context.bit_idx = 7;
            context.bitstream.push_back(context.buffer);
            context.buffer = 0;
        }
    }


    for( int i = 0; i < nbits_y; i++)
    {
        context.cur_bit = ( qy >> (nbits_y - i - 1) ) & 0x0001;
        // update the 8-bits buffer
        context.buffer |= context.cur_bit << context.bit_idx;
        context.bit_idx--;

        // when the buffer is full, append it to the vector; then reset the buffer
        if (context.bit_idx<0){
            context.bit_idx = 7;
            context.bitstream.push_back(context.buffer);
            context.buffer = 0;
        }
    }


    return bitsAngle + bitsOctave + nbits_x + nbits_y;
}



void FeatureCoder::IntraDecodeKeyPoint(DecodeContext &context, cv::KeyPoint &keypoint)
{
    const int bitsAngle = ceil(mnBitsAngle);
    int qangle = 0, qoctave = 0, qx = 0, qy = 0;
    for( int i = 0; i < bitsAngle; i++ )
    {
        // reset bit counter
        if(context.bit_idx<0){
            context.bit_idx = 7;
            context.cur_byte = context.bitstream[context.byte_idx];
            context.byte_idx++;
        }
        // read the current bit
        context.cur_bit = (context.cur_byte >> context.bit_idx) & 0x01;
        context.bit_idx--;

        qangle |= (context.cur_bit << (bitsAngle - i - 1) );
    }

    const int bitsOctave = ceil(mnBitsOctave);
    for( int i = 0; i < bitsOctave; i++){
        // reset bit counter
        if(context.bit_idx<0){
            context.bit_idx = 7;
            context.cur_byte = context.bitstream[context.byte_idx];
            context.byte_idx++;
        }
        // read the current bit
        context.cur_bit = (context.cur_byte >> context.bit_idx) & 0x01;
        context.bit_idx--;

        qoctave |= (context.cur_bit << (bitsOctave - i - 1) );
    }

    const int nbits_x = ceil(mvBitsPyramidWidth[qoctave]);
    const int nbits_y = ceil(mvBitsPyramidHeight[qoctave]);

    for( int i = 0; i <  nbits_x; i++)
    {
        // reset bit counter
        if(context.bit_idx<0){
            context.bit_idx = 7;
            context.cur_byte = context.bitstream[context.byte_idx];
            context.byte_idx++;
        }
        // read the current bit
        context.cur_bit = (context.cur_byte >> context.bit_idx) & 0x01;
        context.bit_idx--;

        qx |= (context.cur_bit << (nbits_x - i - 1) );
    }


    for( int i = 0; i < nbits_y; i++ )
    {
        // reset bit counter
        if(context.bit_idx<0){
            context.bit_idx = 7;
            context.cur_byte = context.bitstream[context.byte_idx];
            context.byte_idx++;
        }
        // read the current bit
        context.cur_bit = (context.cur_byte >> context.bit_idx) & 0x01;
        context.bit_idx--;

        qy |= (context.cur_bit << (nbits_y- i - 1) );
    }


    keypoint.pt.x = (float) qx * mScaleFactors[qoctave];
    keypoint.pt.y = (float) qy * mScaleFactors[qoctave];
    keypoint.octave = qoctave;
    keypoint.angle = (float) qangle * mAngleBinSize + mAngleBinSize / 2;
    assert(keypoint.angle >= 0 && keypoint.angle < 360.0);
};



size_t FeatureCoder::IntraEncodeKeyPointAC(const cv::KeyPoint &keypoint, ACEncodeContext &accontext)
{
    const size_t bits_start = accontext.bits();

    const int &octave = keypoint.octave;//0
    const int angleBin = floor(keypoint.angle / mAngleBinSize);//keypoint.angle -1
//    assert( angleBin >= 0 && angleBin <= mAngleBins);

//    cout<<"octave="<<octave<<endl;
    ac_encode_symbol(&accontext.ace, &accontext.acm_intra_octave, octave);
//    ac_encode_symbol(&accontext.ace, &accontext.acm_intra_angle, angleBin);

    // Resize Keypoints to integer resolution
    int qx = round(keypoint.pt.x / mScaleFactors[octave]);
    int qy = round(keypoint.pt.y / mScaleFactors[octave]);
//    cout<<"qx="<<qx<<endl;
    ac_encode_symbol(&accontext.ace, &accontext.v_acm_intra_kpt_x[octave], qx);
//    cout<<"qy="<<qy<<endl;
    //特征点去畸变后 存在负数了
    ac_encode_symbol(&accontext.ace, &accontext.v_acm_intra_kpt_y[octave], qy);//qy=-39

    return accontext.bits() - bits_start;
}



void FeatureCoder::IntraDecodeKeyPointAC(ACDecodeContext &accontext, cv::KeyPoint &keypoint)
{
    const int octave = ac_decode_symbol(&accontext.acd, &accontext.acm_intra_octave);
    const int angleBin = ac_decode_symbol(&accontext.acd, &accontext.acm_intra_angle);

    const int qx = ac_decode_symbol(&accontext.acd, &accontext.v_acm_intra_kpt_x[octave]);
    const int qy = ac_decode_symbol(&accontext.acd, &accontext.v_acm_intra_kpt_y[octave]);

    keypoint.pt.x = (float) qx * mScaleFactors[octave];
    keypoint.pt.y = (float) qy * mScaleFactors[octave];
    keypoint.octave = octave;
    keypoint.angle = (float) angleBin * mAngleBinSize + mAngleBinSize / 2;
    assert(keypoint.angle >= 0 && keypoint.angle < 360.0);
}



size_t FeatureCoder::IntraEncodeResidual(const BRIEF::bitset &residual, ACEncodeContext &accontext)
{
    const size_t bits_start = accontext.bits();


    //暂时注释
    for( int d = 0; d < residual.size(); d++ )
    {
        const int &current_bit =residual[d];
        ac_encode_symbol(&accontext.ace, &accontext.acm_intra_desc, current_bit);
        //ljl 这里是直接发为1的下标，不太行
//        if(current_bit==1){
//            ac_encode_symbol(&accontext.ace, &accontext.acm_intra_desc, d);
//        }
    }
//    cout<<"第一次大小："<<(accontext.ace.bitstream->size())*8<<endl;
//    cout<<"最终编码的样子：";
    
//    const int zu_sum=4;//表示分组分数
//    int flag=2, same_sum=0;//连续编码的个数 实际选项会有1 2 4，初始值为2
//    int encode_sum=0;//记录已经编码个数了
//    for( int d = 0; d <= residual.size(); d++ )
//    {
//        bool isZero=true;
//        for(int j=0;j<flag;j++){
//            if(encode_sum+j<256){
//                //说明没有越界
//                const int &current_bit =residual[encode_sum+j];
//                if(isZero){
//                    //ljl
//                    if(current_bit==1){
//                        isZero=false;
//                        //告知别人这一组发生了变化 但是此时如果就是编码1个，那么无需提前告知
//                        if(flag!=1){
//                            //提前告知
//                            ac_encode_symbol(&accontext.ace, &accontext.acm_intra_desc, 1);
//
//                            //补偿前面未发的0
//                            for(int a=0;a<j;a++){
//                                ac_encode_symbol(&accontext.ace, &accontext.acm_intra_desc, 0);
//                                encode_sum++;
//    //                            cout<<0;
//                            }
//                            //                        cout<<1;
//                        }
//
//                        //把此次的1发出去
//                        ac_encode_symbol(&accontext.ace, &accontext.acm_intra_desc, current_bit);
//                        encode_sum++;
////                        cout<<current_bit;
//                    }
//                }else{
//                    //无论如何都发 表示这一组前面已经出现过1了
//                    ac_encode_symbol(&accontext.ace, &accontext.acm_intra_desc, current_bit);
//                    encode_sum++;
////                    cout<<current_bit;
//                }
//
//
//            }else{
//                break;
//            }
//        }
//        if(encode_sum>=256){
//            break;
//        }
//        if(isZero){
//            //意味着这一组都为0
//            ac_encode_symbol(&accontext.ace, &accontext.acm_intra_desc, 0);
//            encode_sum+=flag;
//
//            if(same_sum==0){
//                same_sum++;
//                //flag再稳一次
//            }else if(same_sum==1){
//                if(flag!=4){
//                    flag*=2;
//                    same_sum=0;
//                }
//            }
////            cout<<0;
//        }else{
//            //刚刚这里出现1了
//            same_sum=0;
//            if(flag!=1){
//                flag/=2;
//            }
//        }
//
//    }
//    cout<<endl;

    return accontext.bits() - bits_start;
}

struct Node {
    char ch;
    int freq;
    Node *left, *right;

    Node(char ch, int freq, Node *left = nullptr, Node *right = nullptr) {
        this->ch = ch;
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

void encode(Node *root, string code, unordered_map<char, string> &table) {
    if (!root) return;
    if (!root->left && !root->right) {
        table[root->ch] = code;
        return;
    }
    encode(root->left, code + "0", table);
    encode(root->right, code + "1", table);
}

string compress(string text) {
    unordered_map<char, float> freq;

    freq['0'] = 0.78;
    freq['1'] = 0.22;
    priority_queue<Node *, vector<Node *>, Compare> pq;
    for (auto &p : freq) {
        int freq = p.second * text.size();
        pq.push(new Node(p.first, freq));
    }
    while (pq.size() > 1) {
        Node *a = pq.top(); pq.pop();
        Node *b = pq.top(); pq.pop();
        pq.push(new Node('\0', a->freq + b->freq, a, b));
    }
    Node *root = pq.top();
    unordered_map<char, string> table;
    encode(root, "", table);
    string compressed;
    for (char c : text) {
        compressed += table[c];
    }
    delete root;
    return compressed;
}

size_t FeatureCoder::IntraEncodeResidual_haffman(const BRIEF::bitset &residual, ACEncodeContext &accontext)
{
    const size_t bits_start = accontext.bits();


    //暂时注释
//    for( int d = 0; d < residual.size(); d++ )
//    {
//        const int &current_bit =residual[d];
//        ac_encode_symbol(&accontext.ace, &accontext.acm_intra_desc, current_bit);
//    }
    string text;
    to_string(residual,text);
    
    string compressed = compress(text);
    cout << "Original text: " << text.size() <<" , "<<text<< endl;
    cout << "Compressed text: " << compressed.size() <<" , "<<compressed<< endl;
   
    return accontext.bits() - bits_start;
}

void FeatureCoder::IntraDecodeResidual(ACDecodeContext &resCtxt, BRIEF::bitset &residual)
{
    // Setup decoder for descriptor
//    cv::Mat exp_residuals(1, mModel.mDims, CV_8U);
    BRIEF::bitset exp_residuals;
    
    for( size_t d = 0; d < mModel.mDims; d++ )
        exp_residuals.push_back( ac_decode_symbol(&resCtxt.acd, &resCtxt.acm_intra_desc));
    
    //ljl
//    bool isZero=true;
//    int flag=0;
//    const int zu_sum=4;//表示分组分数
//    int des_num=0;//总共256个
//
////    cout<<"解析时的样子：";
//    for( size_t d = 0; d < mModel.mDims+64; d++ ){
//
////        cout<<"ac_decode_symbol start "<<des_num<<endl;
//        int sym=ac_decode_symbol(&resCtxt.acd, &resCtxt.acm_intra_desc);
////        cout<<"ac_decode_symbol endl "<<des_num<<endl;
////        cout<<sym;
//        if(sym==0){
//            //此时判断 是全为0， 还是全部重发的里面的0
//            if(isZero){
//                for(int i=0;i<zu_sum;i++){
//                    exp_residuals.push_back(0);
////                    cout<<0;
//                    des_num++;
//                    if(des_num>=256){
//                        break;
//                    }
//                }
//            }else{
//                exp_residuals.push_back(sym);
////                cout<<sym;
//                des_num++;
////                if(des_num>=256){
////                    break;
////                }
//                flag++;
//                if(flag==zu_sum){
//                    flag=0;
//                    isZero=true;
//                }
//            }
//        }else{
//            //此时返回的是1 要么是告知后面要全部重发 要么是全部重发里面的1
//            if(isZero){
//                //告知
//                isZero=false;
//            }else{
//                //全部重发里面的
//                exp_residuals.push_back(sym);
////                cout<<sym;
//                des_num++;
////                if(des_num>=256){
////                    break;
////                }
//                flag++;
//                if(flag==zu_sum){
//                    flag=0;
//                    isZero=true;
//                }
//            }
//        }
//        if(des_num>=256){
//            break;
//        }
//
//    }
//    cout<<endl;
    
    
    residual=exp_residuals;
   
    
//    cout<<"测试描述符："<<residual.size()<<endl;
//    cout<<residual<<endl<<endl;
}


BRIEF::bitset FeatureCoder::IntraReconstructDescriptor(const unsigned int &visualWord, BRIEF::bitset &residual)
{
    // Reconstruct the descriptor
    const BRIEF::bitset &visualCluster = mDemo.voc.getWord(visualWord);

    BRIEF::bitset descriptor;
//    cv::bitwise_xor(residual, visualCluster, descriptor);
    descriptor  =residual^visualCluster;
    return descriptor;
}



size_t FeatureCoder::InterEncodeReferenceAC(int reference, ACEncodeContext &accontext)
{
    const size_t bits_start = accontext.bits();
    ac_encode_symbol(&accontext.ace, &accontext.acm_inter_candidate, reference);
    return accontext.bits() - bits_start;
}



int FeatureCoder::InterDecodeReferenceAC(ACDecodeContext &accontext)
{
    // First we need the octave and angle.
    const int reference = ac_decode_symbol(&accontext.acd, &accontext.acm_inter_candidate);
    return reference;
}



size_t FeatureCoder::InterEncodeKeypoint(const cv::KeyPoint &refKeypoint, const cv::KeyPoint &currentKeypoint,
        ACEncodeContext &accontext, EncodeContext &context)
{
    const size_t bits_start = accontext.bits();
    const size_t flbits = 0;


    const int refAngleBin = floor(refKeypoint.angle / mAngleBinSize);
    const int curAngleBin = floor(currentKeypoint.angle / mAngleBinSize);
    assert( refAngleBin >= 0 && refAngleBin < mAngleBins);
    assert( curAngleBin >= 0 && curAngleBin < mAngleBins);

    // Angle coding
    const int diff = curAngleBin - refAngleBin;
    assert( diff > -32 && diff < 32);

    const int angleDiff = diff + mnAngleOffset;
    assert(angleDiff >= 0 && angleDiff < (int) mFreqInterAngleDiff.size() );

    ac_encode_symbol(&accontext.ace, &accontext.acm_inter_angle, angleDiff);


    // Octave coding
    const int octaveDiff = currentKeypoint.octave - refKeypoint.octave + mnOctaveOffset;
    ac_encode_symbol(&accontext.ace, &accontext.acm_inter_octave, octaveDiff);


    // KEYPOINT CODING
    const int &octave = currentKeypoint.octave;
    assert( fabs(refKeypoint.pt.x - currentKeypoint.pt.x) <= mModel.mSearchRange );
    assert( fabs(refKeypoint.pt.y - currentKeypoint.pt.y) <= mModel.mSearchRange );


    const int sRefx = round(refKeypoint.pt.x / mScaleFactors[octave]);
    const int sRefy = round(refKeypoint.pt.y / mScaleFactors[octave]);

    const int sCurX = round(currentKeypoint.pt.x / mScaleFactors[octave]);
    const int sCurY = round(currentKeypoint.pt.y / mScaleFactors[octave]);

    const int dx = sCurX - sRefx;
    const int dy = sCurY - sRefy;

    const int tdx = dx + (mFreqInterKeyPoint[octave].cols-1)/2;
    const int tdy = dy + (mFreqInterKeyPoint[octave].rows-1)/2;

    int index;
    KeyPointDiffToIndex(tdx, tdy, octave, index);


    // Coding xy-value
    ac_encode_symbol(&accontext.ace, &accontext.v_acm_inter_kpt[octave], index);


    const size_t acbits = accontext.bits() - bits_start;
    return acbits + flbits;
}



void FeatureCoder::InterDecodeKeypoint(ACDecodeContext &accontext, DecodeContext &context, const cv::KeyPoint &refKeypoint, cv::KeyPoint &currentKeypoint)
{
    // Angle decoding
    const int refAngleBin = floor(refKeypoint.angle / mAngleBinSize);
    const int angleDiff = ac_decode_symbol(&accontext.acd, &accontext.acm_inter_angle);

    // Octave decoding
    const int octaveDiff =  ac_decode_symbol(&accontext.acd, &accontext.acm_inter_octave);

    const int angleBin = angleDiff - mnAngleOffset + refAngleBin;
    const int octave = octaveDiff - mnOctaveOffset + refKeypoint.octave;


    // Keypoint decoding
    int x = 0, y = 0;


    // Position decoding
    const int index = ac_decode_symbol(&accontext.acd, &accontext.v_acm_inter_kpt[octave]);

    int dx,dy;
    IndexToKeyPointDiff(index, octave, dx, dy);

    dx = dx - (mFreqInterKeyPoint[octave].cols-1)/2;
    dy = dy - (mFreqInterKeyPoint[octave].rows-1)/2;

    const int sRefx = round(refKeypoint.pt.x / mScaleFactors[octave]);
    const int sRefy = round(refKeypoint.pt.y / mScaleFactors[octave]);

    x  = (dx + sRefx);
    y  = (dy + sRefy);

    currentKeypoint.pt.x = x *  mScaleFactors[octave];
    currentKeypoint.pt.y = y *  mScaleFactors[octave];
    currentKeypoint.angle = angleBin * mAngleBinSize + mAngleBinSize / 2;
    currentKeypoint.octave = octave;
    assert(currentKeypoint.angle >= 0 && currentKeypoint.angle < 360.0);
}



size_t FeatureCoder::InterEncodeResidual(const BRIEF::bitset &residual, ACEncodeContext &accontext)
{
    const size_t bits_start = accontext.bits();
//    BRIEF::bitset expResidual;
//    Utils::bin2mat(residual, expResidual);
//
//    for( int d = 0; d < expResidual.cols; d++ )
//    {
//        const uchar &current_bit = expResidual.at<uchar>(d);
//        ac_encode_symbol(&accontext.ace, &accontext.acm_inter_desc, current_bit);
//    }
    
//    cout<<"测试描述符："<<residual.size()<<endl;
//    cout<<residual<<endl<<endl;

    return accontext.bits() - bits_start;
}



void FeatureCoder::InterDecodeResidual(ACDecodeContext &accontext, BRIEF::bitset &residual)
{
    // We can get the residual vector
//    cv::Mat exp_residuals(1, mModel.mDims, CV_8U);
//    for( unsigned int d = 0; d < mModel.mDims; d++ )
//        exp_residuals.at<uchar>(d) = ac_decode_symbol(&accontext.acd, &accontext.acm_inter_desc);
//
//    Utils::mat2bin(exp_residuals, residual);
    
//    cout<<"测试描述符："<<residual.size()<<endl;
//    cout<<residual<<endl<<endl;
}



BRIEF::bitset FeatureCoder::InterReconstructDescriptor(const BRIEF::bitset &referenceDescriptor, const BRIEF::bitset &residual)
{
    BRIEF::bitset descriptor;
//    cv::bitwise_xor(referenceDescriptor, residual, descriptor);
    descriptor  =referenceDescriptor^residual;
    return descriptor;
}

void FeatureCoder::KeyPointDiffToIndex(int dx, int dy, int octave, int &index)
{
    index= dy * mFreqInterKeyPoint[octave].cols + dx;
}

void FeatureCoder::IndexToKeyPointDiff(int index, int octave, int &x, int &y)
{
    x = index % mFreqInterKeyPoint[octave].cols;
    y = index / mFreqInterKeyPoint[octave].cols;
}



size_t FeatureCoder::encodeReference(int reference, int numCandidates, EncodeContext &ctxt)
{
    assert( reference < numCandidates );
    const int nBitsReference = ceil(log2(numCandidates));
    for( int i = 0; i < nBitsReference; i++)
    {
        ctxt.cur_bit = ( reference >> (nBitsReference - i - 1) ) & 0x0001;
        // update the 8-bits buffer
        ctxt.buffer |= ctxt.cur_bit << ctxt.bit_idx;
        ctxt.bit_idx--;

        // when the buffer is full, append it to the vector; then reset the buffer
        if (ctxt.bit_idx<0){
            ctxt.bit_idx= 7;
            ctxt.bitstream.push_back(ctxt.buffer);
            ctxt.buffer = 0;
        }
    }

    return nBitsReference;
}



int FeatureCoder::decodeReference(DecodeContext &ctxt, int numCandidates)
{
    // First we need the octave and angle.
    const int nBitsReference = ceil(log2(numCandidates));
    int referenceId = 0;
    for( int i = 0; i < nBitsReference; i++ )
    {
        // reset bit counter
        if(ctxt.bit_idx<0){
            ctxt.bit_idx = 7;
            ctxt.cur_byte = ctxt.bitstream[ctxt.byte_idx];
            ctxt.byte_idx++;
        }
        // read the current bit
        ctxt.cur_bit = (ctxt.cur_byte >> ctxt.bit_idx) & 0x01;
        ctxt.bit_idx--;

        referenceId |= (ctxt.cur_bit << (nBitsReference - i - 1) );
    }

    return referenceId;
}



float FeatureCoder::fakeCodeDepth(const float &fDepth)
{
    size_t idx = Utils::findNearestNeighbourIndex(fDepth, mvfDepthCodeBook);
    return mvfDepthCodeBook[idx];
}



cv::KeyPoint FeatureCoder::fakeCode(const cv::KeyPoint &keyPoint)
{
    // Angle decoding
    const int angleBin = floor(keyPoint.angle / mAngleBinSize);
//    cout<<"angleBin="<<angleBin<<" , "<<mAngleBins<<endl;
    assert(angleBin >= 0 && angleBin < mAngleBins);

    cv::KeyPoint decodedKeyPoint;
    decodedKeyPoint.pt = keyPoint.pt;
    decodedKeyPoint.angle = angleBin * mAngleBinSize + mAngleBinSize/2;
    decodedKeyPoint.octave = keyPoint.octave;
    assert( decodedKeyPoint.angle >= 0 && decodedKeyPoint.angle  < 360.0);
    return decodedKeyPoint;
}

} // END NAMESPACE


