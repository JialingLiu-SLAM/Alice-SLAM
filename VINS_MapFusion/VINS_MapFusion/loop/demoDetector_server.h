//
//  demoDetector_server.h
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/6/27.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef demoDetector_server_h
#define demoDetector_server_h


#include <iostream>
#include <vector>
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// DLoopDetector and DBoW2
#include "DBoW2.h"
#include "DLoopDetector_server.h"

//for time
#include <cctype>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "dirent.h"
#include <unistd.h>
#include <string>
#include <ctime>
#include <cstdlib>
#include <chrono>

using namespace DLoopDetector_server;
using namespace DBoW2;
using namespace std;


/// Generic class to create functors to extract features
template<class TDescriptor>
class FeatureExtractor_server
{
public:
  /**
   * Extracts features
   * @param im image
   * @param keys keypoints extracted
   * @param descriptors descriptors extracted
   */
  virtual void operator()(const cv::Mat &im, const std::vector<cv::Point2f> window_pts,
    vector<cv::KeyPoint> &keys, vector<TDescriptor> &descriptors) const = 0;
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

/// @param TVocabulary vocabulary class (e.g: Surf64Vocabulary)
/// @param TDetector detector class (e.g: Surf64LoopDetector)
/// @param TDescriptor descriptor class (e.g: vector<float> for SURF)
template<class TVocabulary, class TDetector, class TDescriptor>
/// Class to run the demo
class demoDetector_server
{
public:

  /**
   * @param vocfile vocabulary file to load
   * @param imagedir directory to read images from
   * @param posefile pose file
   * @param width image width
   * @param height image height
   */
  demoDetector_server(const std::string &vocfile, int width, int height);
    
  ~demoDetector_server(){}


  /**
   * Runs the demo
   * @param name demo name
   * @param extractor functor to extract features
   */
    bool run(const std::string &name,
             const std::vector<cv::KeyPoint> &keys,
             const std::vector<TDescriptor> &descriptors,
             std::vector<cv::Point2f> &cur_pts,
             std::vector<cv::Point2f> &old_pts,
             int &old_index,int &min_startDetect_id,int cur_kf_global_index, int clientId);
    
    bool run2(const std::string &name,
             const std::vector<cv::KeyPoint> &keys,
             const std::vector<TDescriptor> &descriptors,
             std::vector<cv::Point2f> &cur_pts,
             std::vector<cv::Point2f> &old_pts,
             int &old_index,int &min_startDetect_id,int cur_kf_global_index, int clientId, int keyPoint_num_main,std::vector<cv::Point2f> &cur_pts_3d, std::vector<cv::Point2f> &old_pts_2d);
    
    void eraseIndex(std::vector<int> &erase_index);
    
    void addToDatabase_demo(const std::vector<cv::KeyPoint> &keys,const std::vector<TDescriptor> &descriptors);
    
    void addToDatabase_demo_2(const std::vector<cv::KeyPoint> &keys, const std::vector<TDescriptor> &descriptors, const BowVector &bowvec, const FeatureVector &featvec);
    
    int getDatabase_size();
    
  /*Data*/
  typename TDetector::Parameters params;
  TVocabulary voc;
  TDetector detector;
  int m_width;
  int m_height;

protected:

  /**
   * Reads the robot poses from a file
   * @param filename file
   * @param xs
   * @param ys
   */
  void readPoseFile(const char *filename, std::vector<double> &xs,
    std::vector<double> &ys) const;

protected:

  std::string m_vocfile;
  //std::string m_imagedir;
  //std::string m_posefile;
};

// ---------------------------------------------------------------------------

template<class TVocabulary, class TDetector, class TDescriptor>
demoDetector_server<TVocabulary, TDetector, TDescriptor>::demoDetector_server
  (const std::string &vocfile, int width, int height)
  : m_vocfile(vocfile), m_width(width), m_height(height),
    params(height, width), voc(vocfile),detector(voc, params)
{
    //params.use_nss = true; // use normalized similarity score instead of raw score
    //params.alpha = 0.3; // nss threshold
    //params.k = 1; // a loop must be consistent with 1 previous matches
     // use direct index for geometrical checking
    //params.di_levels = 2; // use two direct index levels
    //printf("load vocfile %s finish\n", vocfile);
    
    voc.setScoringType(L1_NORM_SERVER);
    
    
    printf("my loop image size width: %d height: %d\n", params.image_cols,params.image_rows);
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
template<class TVocabulary, class TDetector, class TDescriptor>
void demoDetector_server<TVocabulary, TDetector, TDescriptor>::eraseIndex
(std::vector<int> &erase_index)
{
    detector.eraseIndex(erase_index);
}
// ---------------------------------------------------------------------------
template<class TVocabulary, class TDetector, class TDescriptor>
void demoDetector_server<TVocabulary, TDetector, TDescriptor>::addToDatabase_demo
(const std::vector<cv::KeyPoint> &keys,
 const std::vector<TDescriptor> &descriptors)
{
    detector.addToDatabase(keys,descriptors);
}

template<class TVocabulary, class TDetector, class TDescriptor>
void demoDetector_server<TVocabulary, TDetector, TDescriptor>::addToDatabase_demo_2(const std::vector<cv::KeyPoint> &keys, const std::vector<TDescriptor> &descriptors, const BowVector &bowvec, const FeatureVector &featvec){
    detector.addToDatabase_2(keys, descriptors, bowvec, featvec);
}


template<class TVocabulary, class TDetector, class TDescriptor>
int demoDetector_server<TVocabulary, TDetector, TDescriptor>::getDatabase_size()
{
    return detector.getDatabase_size();
}
// ---------------------------------------------------------------------------

template<class TVocabulary, class TDetector, class TDescriptor>
bool demoDetector_server<TVocabulary, TDetector, TDescriptor>::run
(const std::string &name, const std::vector<cv::KeyPoint> &keys,
 const std::vector<TDescriptor> &descriptors,
 std::vector<cv::Point2f> &cur_pts,
 std::vector<cv::Point2f> &old_pts,
 int &old_index,int &min_startDetect_id,int cur_kf_global_index, int clientId)
{
  int count = 0;

  DetectionResult_server result;
    
  detector.detectLoop(keys, descriptors, result, cur_pts, old_pts,min_startDetect_id,cur_kf_global_index, clientId);
    

    
  if(result.detection())
  {
      cout <<endl<< "loopClosureRun_global - loop found with image " << result.match << "!"
        << endl;
      ++count;
      old_index = result.match;
      return true;
  }
  else
  {
//      cout << "loopClosureRun_global - No loop: ";
//      switch(result.status)
//      {
//        case CLOSE_MATCHES_ONLY_server:
//          cout << "real-my All the images in the database are very recent" << endl;
//          break;
//          
//        case NO_DB_RESULTS_server:
//          cout << "real-my There are no matches against the database (few features in"
//            " the image?)" << endl;
//          break;
//          
//        case LOW_NSS_FACTOR_server:
//          cout << "real-my Little overlap between this image and the previous one"
//            << endl;
//          break;
//            
//        case LOW_SCORES_server:
//          cout << "real-my No match reaches the score threshold (alpha: " <<
//            params.alpha << ")" << endl;
//          break;
//          
//        case NO_GROUPS_server:
//          cout << "real-my Not enough close matches to create groups. "
//            << "Best candidate: " << result.match << endl;
//          break;
//          
//        case NO_TEMPORAL_CONSISTENCY_server:
//          cout << "real-my No temporal consistency (k: " << params.k << "). "
//            << "Best candidate: " << result.match << endl;
//          break;
//          
//        case NO_GEOMETRICAL_CONSISTENCY_server:
//          cout << "real-my No geometrical consistency. Best candidate: "
//            << result.match << endl;
//          break;
//          
//        default:
//          break;
//      }
//      cout << endl;
      return false;
  }
}

template<class TVocabulary, class TDetector, class TDescriptor>
bool demoDetector_server<TVocabulary, TDetector, TDescriptor>::run2(const std::string &name,
         const std::vector<cv::KeyPoint> &keys,
         const std::vector<TDescriptor> &descriptors,
         std::vector<cv::Point2f> &cur_pts,
         std::vector<cv::Point2f> &old_pts,
          int &old_index,int &min_startDetect_id,int cur_kf_global_index, int clientId, int keyPoint_num_main,std::vector<cv::Point2f> &cur_pts_3d, std::vector<cv::Point2f> &old_pts_2d){
    int count = 0;

    DetectionResult_server result;
      
    detector.detectLoop2(keys, descriptors, result, cur_pts, old_pts,min_startDetect_id,cur_kf_global_index, clientId,keyPoint_num_main,cur_pts_3d,old_pts_2d);
      

      
    if(result.detection())
    {
        cout <<endl<< "loopClosureRun_global - loop found with image " << result.match << "!"
          << endl;
        ++count;
        old_index = result.match;
        return true;
    }
    else
    {
        return false;
    }
}
// ---------------------------------------------------------------------------

template<class TVocabulary, class TDetector, class TDescriptor>
void demoDetector_server<TVocabulary, TDetector, TDescriptor>::readPoseFile
  (const char *filename, std::vector<double> &xs, std::vector<double> &ys)
  const
{
  xs.clear();
  ys.clear();
  
  fstream f(filename, ios::in);
  
  string s;
  double ts, x, y, t;
  while(!f.eof())
  {
    getline(f, s);
    if(!f.eof() && !s.empty())
    {
      sscanf(s.c_str(), "%lf, %lf, %lf, %lf", &ts, &x, &y, &t);
      xs.push_back(x);
      ys.push_back(y);
    }
  }
  
  f.close();
}


#endif /* demoDetector_server_h */
