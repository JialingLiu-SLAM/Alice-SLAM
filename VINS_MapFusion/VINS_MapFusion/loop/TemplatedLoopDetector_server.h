//
//  TemplatedLoopDetector_server.h
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/6/26.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef TemplatedLoopDetector_server_h
#define TemplatedLoopDetector_server_h



#include <vector>
#include <numeric>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>

#include "TemplatedVocabulary.h"
#include "TemplatedDatabase.h"
#include "QueryResults.h"
#include "BowVector.h"


using namespace std;
using namespace DBoW2;

namespace DLoopDetector_server {


/// Geometrical checking methods
enum GeometricalCheck
{
  /// Exhaustive search
  GEOM_EXHAUSTIVE,
  /// Use direct index
  GEOM_DI,
  /// Use a Flann structure
  GEOM_FLANN,
  /// Do not perform geometrical checking
  GEOM_NONE
};

/// Reasons for dismissing loops
enum DetectionStatus
{
  /// Loop correctly detected
  LOOP_DETECTED_server,
  /// All the matches are very recent
  CLOSE_MATCHES_ONLY_server,
  /// No matches against the database
  NO_DB_RESULTS_server,
  /// Score of current image against previous one too low
  LOW_NSS_FACTOR_server,
  /// Scores (or NS Scores) were below the alpha threshold
  LOW_SCORES_server,
  /// Not enough matches to create groups
  NO_GROUPS_server,
  /// Not enough temporary consistent matches (k)
  NO_TEMPORAL_CONSISTENCY_server,
  /// The geometrical consistency failed
  NO_GEOMETRICAL_CONSISTENCY_server
};

/// Result of a detection
struct DetectionResult_server
{
  /// Detection status. LOOP_DETECTED iff loop detected
  DetectionStatus status;
  /// Query id
  EntryId query;
  /// Matched id if loop detected, otherwise, best candidate
  EntryId match;
  
  /**
   * Checks if the loop was detected
   * @return true iff a loop was detected
   */
  inline bool detection() const
  {
    return status == LOOP_DETECTED_server;
  }
};

/// TDescriptor: class of descriptor
/// F: class of descriptor functions
template<class TDescriptor, class F>
/// Generic Loop detector
class TemplatedLoopDetector_server
{
public:
  
  /// Parameters to create a loop detector
  struct Parameters
  {
    /// Height of expected images
    int image_rows;
    /// Width of expected images
    int image_cols;
    
    // Main loop detector parameters
    
    /// Use normalized similarity score?
    bool use_nss;
    /// Alpha threshold
    float alpha;
    /// Min consistent matches to pass the temporal check
    int k;
    /// Geometrical check
    GeometricalCheck geom_check;
    /// If using direct index for geometrical checking, direct index levels
    int di_levels;
    
    // These are less deciding parameters of the system
    
    /// Distance between entries to be consider a match
    int dislocal;
    /// Max number of results from db queries to consider
    int max_db_results;
    /// Min raw score between current entry and previous one to consider a match
    float min_nss_factor;
    /// Min number of close matches to consider some of them
    int min_matches_per_group;
    /// Max separation between matches to consider them of the same group
    int max_intragroup_gap;
    /// Max separation between groups of matches to consider them consistent
    int max_distance_between_groups;
    /// Max separation between two queries to consider them consistent
    int max_distance_between_queries;
  
    // These are for the RANSAC to compute the F
    
    /// Min number of inliers when computing a fundamental matrix
    int min_Fpoints;
    /// Max number of iterations of RANSAC
    int max_ransac_iterations;
    /// Success probability of RANSAC
    double ransac_probability;
    /// Max reprojection error of fundamental matrices
    double max_reprojection_error;
    
    // This is to compute correspondences
    
    /// Max value of the neighbour-ratio of accepted correspondences
    double max_neighbor_ratio;
  
    /**
     * Creates parameters by default
     */
    Parameters();
    
    /**
     * Creates parameters by default
     * @param height image height
     * @param width image width
     * @param frequency set the value of some parameters according to the
     *   expected working frequency (Hz)
     * @param nss use normalized similarity score
     * @param _alpha alpha parameter
     * @param _k k parameter (number of temporary consistent matches)
     * @param geom type of geometrical check
     * @param dilevels direct index levels when geom == GEOM_DI
     */
    Parameters(int height, int width, float frequency = 1, bool nss = true,
      float _alpha = 0.3, int _k = 1,
      GeometricalCheck geom = GEOM_DI, int dilevels = 2);
      
  private:
    /**
     * Sets some parameters according to the frequency
     * @param frequency
     */
    void set(float frequency);
  };
  
public:

  /**
   * Empty constructor
   */
  TemplatedLoopDetector_server(const Parameters &params = Parameters());

  /**
   * Creates a loop detector with the given parameters and with a BoW2 database
   * with the given vocabulary
   * @param voc vocabulary
   * @param params loop detector parameters
   */
  TemplatedLoopDetector_server( TemplatedVocabulary<TDescriptor, F> &voc,
    const Parameters &params = Parameters());
  
  /**
   * Creates a loop detector with a copy of the given database, but clearing
   * its contents
   * @param db database to copy
   * @param params loop detector parameters
   */
  TemplatedLoopDetector_server(const TemplatedDatabase<TDescriptor, F> &db,
    const Parameters &params = Parameters());

  /**
   * Creates a loop detector with a copy of the given database, but clearing
   * its contents
   * @param T class derived from TemplatedDatabase
   * @param db database to copy
   * @param params loop detector parameters
   */
  template<class T>
  TemplatedLoopDetector_server(const T &db, const Parameters &params = Parameters());

  /**
   * Destructor
   */
  virtual ~TemplatedLoopDetector_server(void);
  
  /**
   * Retrieves a reference to the database used by the loop detector
   * @return const reference to database
   */
  inline const TemplatedDatabase<TDescriptor, F>& getDatabase() const;
  
  /**
   * Retrieves a reference to the vocabulary used by the loop detector
   * @return const reference to vocabulary
   */
  inline const TemplatedVocabulary<TDescriptor, F>& getVocabulary() const;
  
  /**
   * Sets the database to use. The contents of the database and the detector
   * entries are cleared
   * @param T class derived from TemplatedDatabase
   * @param db database to copy
   */
  template<class T>
  void setDatabase(const T &db);
  
  /**
   * Sets a new DBoW2 database created from the given vocabulary
   * @param voc vocabulary to copy
   */
  void setVocabulary(const TemplatedVocabulary<TDescriptor, F>& voc);
  
  /**
   * Allocates some memory for the first entries
   * @param nentries number of expected entries
   * @param nkeys number of keypoints per image expected
   */
  void allocate(int nentries, int nkeys = 0);

    
  void addToDatabase(const std::vector<cv::KeyPoint> &keys,
    const std::vector<TDescriptor> &descriptors);
    
    void addToDatabase_2(const std::vector<cv::KeyPoint> &keys, const std::vector<TDescriptor> &descriptors, const BowVector &bowvec, const FeatureVector &featvec);
    
    int getDatabase_size();
  /**
   * Adds the given tuple <keys, descriptors, current_t> to the database
   * and returns the match if any
   * @param keys keypoints of the image
   * @param descriptors descriptors associated to the given keypoints
   * @param match (out) match or failing information
   * @return true iff there was match
   */
  bool detectLoop(const std::vector<cv::KeyPoint> &keys,
    const std::vector<TDescriptor> &descriptors,
    DetectionResult_server &match,std::vector<cv::Point2f> &cur_pts,
                          std::vector<cv::Point2f> &old_pts,int &min_startDetect_id,int cur_kf_global_index, int clientId);
    
    bool detectLoop2(const std::vector<cv::KeyPoint> &keys,
      const std::vector<TDescriptor> &descriptors,
      DetectionResult_server &match,std::vector<cv::Point2f> &cur_pts,
                            std::vector<cv::Point2f> &old_pts,int &min_startDetect_id,int cur_kf_global_index, int clientId, int keyPoint_num_main,std::vector<cv::Point2f> &cur_pts_3d, std::vector<cv::Point2f> &old_pts_2d);

  /**
   * Resets the detector and clears the database, such that the next entry
   * will be 0 again
   */
  inline void clear(int clientId);

  void eraseIndex(std::vector<int> &erase_index);
    
protected:
  
  /// Matching island
  struct tIsland
  {
    /// Island starting entry
    EntryId first;
    /// Island ending entry
    EntryId last;
    /// Island score
    double score; // score of island
    
    /// Entry of the island with the highest score
    EntryId best_entry; // id and score of the entry with the highest score
    /// Highest single score in the island
    double best_score;  // in the island
    
    /**
     * Creates an empty island
     */
    tIsland(){}
    
    /**
     * Creates an island
     * @param f first entry
     * @param l last entry
     */
    tIsland(EntryId f, EntryId l): first(f), last(l){}
    
    /**
     * Creates an island
     * @param f first entry
     * @param l last entry
     * @param s island score
     */
    tIsland(EntryId f, EntryId l, double s): first(f), last(l), score(s){}
    
    /**
     * Says whether this score is less than the score of another island
     * @param b
     * @return true iff this score < b.score
     */
    inline bool operator < (const tIsland &b) const
    {
      return this->score < b.score;
    }
    
    /**
     * Says whether this score is greater than the score of another island
     * @param b
     * @return true iff this score > b.score
     */
    inline bool operator > (const tIsland &b) const
    {
      return this->score > b.score;
    }
    
    /**
     * Returns true iff a > b
     * This function is used to sort in descending order
     * @param a
     * @param b
     * @return a > b
     */
    static inline bool gt(const tIsland &a, const tIsland &b)
    {
      return a.score > b.score;
    }
        
    /**
     * Returns true iff entry ids of a are less then those of b.
     * Assumes there is no overlap between the islands
     * @param a
     * @param b
     * @return a.first < b.first
     */
    static inline bool ltId(const tIsland &a, const tIsland &b)
    {
      return a.first < b.first;
    }
    
    /**
     * Returns the length of the island
     * @return length of island
     */
    inline int length() const { return last - first + 1; }
    
    /**
     * Returns a printable version of the island
     * @return printable island
     */
    std::string toString() const
    {
      stringstream ss;
      ss << "[" << first << "-" << last << ": " << score << " | best: <"
        << best_entry << ": " << best_score << "> ] ";
      return ss.str();
    }
  };
  
  /// Temporal consistency window
  struct tTemporalWindow
  {
    /// Island matched in the last query
    tIsland last_matched_island;
    /// Last query id
    EntryId last_query_id;
    /// Number of consistent entries in the window
    int nentries;
    
    /**
     * Creates an empty temporal window
     */
    tTemporalWindow(): nentries(0) {}
  };
  
  
protected:
  
  /**
   * Removes from q those results whose score is lower than threshold
   * (that should be alpha * ns_factor)
   * @param q results from query
   * @param threshold min value of the resting results
   */
  void removeLowScores(QueryResults &q, double threshold) const;
  
  /**
   * Returns the islands of the given matches in ascending order of entry ids
   * @param q
   * @param islands (out) computed islands
   */
  void computeIslands(QueryResults &q, vector<tIsland> &islands) const;
  
  /**
   * Returns the score of the island composed of the entries of q whose indices
   * are in [i_first, i_last] (both included)
   * @param q
   * @param i_first first index of q of the island
   * @param i_last last index of q of the island
   * @return island score
   */
  double calculateIslandScore(const QueryResults &q, unsigned int i_first,
    unsigned int i_last) const;

  /**
   * Updates the temporal window by adding the given match <island, id>, such
   * that the window will contain only those islands which are consistent
   * @param matched_island
   * @param entry_id
   */
  void updateTemporalWindow(const tIsland &matched_island, EntryId entry_id, int clientId);
  
  /**
   * Returns the number of consistent islands in the temporal window
   * @return number of temporal consistent islands
   */
  inline int getConsistentEntries(int clientId) const
  {

    return m_window[clientId].nentries;
  }
  
  void reduceInputToOutput(vector<cv::Point2f> v_in, vector<cv::Point2f> &v_out, vector<uchar> status);
    void reduceInputToOutput(vector<cv::Point2f> v_in, vector<cv::Point2f> &v_out, vector<uchar> status, vector<uchar> status2, vector<cv::Point2f> &v_out_3d);
  bool checkFoundamental(vector<cv::Point2f> cur_input_pts,
                   vector<cv::Point2f> old_input_pts,
                   vector<cv::Point2f> &cur_output_pts,
                   vector<cv::Point2f> &old_output_pts);
    bool checkFoundamental2(vector<cv::Point2f> cur_input_pts, vector<cv::Point2f> old_input_pts,   vector<cv::Point2f> &cur_output_pts,
         vector<cv::Point2f> &old_output_pts,vector<uchar> &status2,   vector<cv::Point2f> &cur_output_pts_3d, vector<cv::Point2f> &old_output_pts_3d);
  /**
   * Check if an old entry is geometrically consistent (by calculating a
   * fundamental matrix) with the given set of keys and descriptors
   * @param old_entry entry id of the stored image to check
   * @param keys current keypoints
   * @param descriptors current descriptors associated to the given keypoints
   * @param curvec feature vector of the current entry
   */
  bool isGeometricallyConsistent_DI(EntryId old_entry,
    const std::vector<cv::KeyPoint> &keys,
    const std::vector<TDescriptor> &descriptors,
    const FeatureVector &curvec,
    std::vector<cv::Point2f> &cur_pts,
    std::vector<cv::Point2f> &old_pts);
    
    bool isGeometricallyConsistent_DI2(EntryId old_entry,
      const std::vector<cv::KeyPoint> &keys,
      const std::vector<TDescriptor> &descriptors,
      const FeatureVector &curvec,
      std::vector<cv::Point2f> &cur_pts,
      std::vector<cv::Point2f> &old_pts,int keyPoint_num_main,std::vector<cv::Point2f> &cur_pts_3d,std::vector<cv::Point2f> &old_pts_2d);

  /**
   * Creates a flann structure from a set of descriptors to perform queries
   * @param descriptors
   * @param flann_structure (out) flann matcher
   */
  void getFlannStructure(const std::vector<TDescriptor> &descriptors,
    cv::FlannBasedMatcher &flann_structure) const;

  /**
   * Calculate the matches between the descriptors A[i_A] and the descriptors
   * B[i_B]. Applies a left-right matching without neighbour ratio
   * @param A set A of descriptors
   * @param i_A only descriptors A[i_A] will be checked
   * @param B set B of descriptors
   * @param i_B only descriptors B[i_B] will be checked
   * @param i_match_A (out) indices of descriptors matched (s.t. A[i_match_A])
   * @param i_match_B (out) indices of descriptors matched (s.t. B[i_match_B])
   */
  void getMatches_neighratio(const std::vector<TDescriptor> &A,
    const vector<unsigned int> &i_A, const vector<TDescriptor> &B,
    const vector<unsigned int> &i_B,
    vector<unsigned int> &i_match_A, vector<unsigned int> &i_match_B) const;

protected:

  /// Database
  // The loop detector stores its own copy of the database
  TemplatedDatabase<TDescriptor,F> *m_database;
  
  /// KeyPoints of images
  vector<vector<cv::KeyPoint> > m_image_keys;
  
  /// Descriptors of images
  vector<vector<TDescriptor> > m_image_descriptors;
  
  /// Last bow vector added to database
  vector<BowVector> m_last_bowvec;
//    int last_id=-1;
  
  /// Temporal consistency window
  vector<tTemporalWindow> m_window;
  
  /// Parameters of loop detector
  Parameters m_params;
  
  
};

// --------------------------------------------------------------------------

template <class TDescriptor, class F>
TemplatedLoopDetector_server<TDescriptor,F>::Parameters::Parameters():
  use_nss(true), alpha(0.3), k(4), geom_check(GEOM_DI), di_levels(0)
{
  set(1);
    
}

// --------------------------------------------------------------------------

template <class TDescriptor, class F>
TemplatedLoopDetector_server<TDescriptor,F>::Parameters::Parameters
  (int height, int width, float frequency, bool nss, float _alpha,
  int _k, GeometricalCheck geom, int dilevels)
  : image_rows(height), image_cols(width), use_nss(nss), alpha(_alpha), k(_k),
    geom_check(geom), di_levels(dilevels)
{
  set(frequency);
}

// --------------------------------------------------------------------------

template <class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor,F>::Parameters::set(float f)
{
  dislocal = 20 * f;
  max_db_results = 50 * f;
  min_nss_factor = 0.005;
  min_matches_per_group = f;
  max_intragroup_gap = 3 * f;
  max_distance_between_groups = 3 * f;
  max_distance_between_queries = 2 * f;

  min_Fpoints = 12;
  max_ransac_iterations = 500;
  ransac_probability = 0.99;
  max_reprojection_error = 2.0;
  
  max_neighbor_ratio = 0.6;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedLoopDetector_server<TDescriptor,F>::TemplatedLoopDetector_server
  (const Parameters &params)
  : m_database(NULL), m_params(params)
{
    m_last_bowvec.resize(10);
    m_window.resize(10);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedLoopDetector_server<TDescriptor,F>::TemplatedLoopDetector_server
  ( TemplatedVocabulary<TDescriptor, F> &voc, const Parameters &params)
  : m_params(params)
{
//    cout<<"TemplatedLoopDetector_server m_voc->getScoringType()="<<voc.getScoringType()<<endl;
    voc.setScoringType(L1_NORM_SERVER);
//    cout<<"TemplatedLoopDetector_server m_voc->getScoringType()="<<voc.getScoringType()<<endl;
    
  m_database = new TemplatedDatabase<TDescriptor, F>(voc,
    params.geom_check == GEOM_DI, params.di_levels);
    
    m_last_bowvec.resize(10);
    m_window.resize(10);
  //TODO change param trans method
// printf("loop set camera model finish\n");
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor,F>::setVocabulary
  (const TemplatedVocabulary<TDescriptor, F>& voc)
{
  delete m_database;
  m_database = new TemplatedDatabase<TDescriptor, F>(voc,
    m_params.geom_check == GEOM_DI, m_params.di_levels);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedLoopDetector_server<TDescriptor, F>::TemplatedLoopDetector_server
  (const TemplatedDatabase<TDescriptor, F> &db, const Parameters &params)
  : m_params(params)
{
  m_database = new TemplatedDatabase<TDescriptor, F>(db.getVocabulary(),
    params.geom_check == GEOM_DI, params.di_levels);
  m_last_bowvec.resize(10);
    m_window.resize(10);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
template<class T>
TemplatedLoopDetector_server<TDescriptor, F>::TemplatedLoopDetector_server
  (const T &db, const Parameters &params)
  : m_params(params)
{
  m_database = new T(db);
  m_database->clear();
  m_last_bowvec.resize(10);
    m_window.resize(10);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
template<class T>
void TemplatedLoopDetector_server<TDescriptor, F>::setDatabase(const T &db)
{
  delete m_database;
  m_database = new T(db);
  clear();
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedLoopDetector_server<TDescriptor, F>::~TemplatedLoopDetector_server(void)
{
  delete m_database;
  m_database = NULL;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor,F>::allocate
  (int nentries, int nkeys)
{
  const int sz = (const int)m_image_keys.size();
  
  if(sz < nentries)
  {
    m_image_keys.resize(nentries);
    m_image_descriptors.resize(nentries);
  }
  
  if(nkeys > 0)
  {
    for(int i = sz; i < nentries; ++i)
    {
      m_image_keys[i].reserve(nkeys);
      m_image_descriptors[i].reserve(nkeys);
    }
  }
  
  m_database->allocate(nentries, nkeys);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
inline const TemplatedDatabase<TDescriptor, F>&
TemplatedLoopDetector_server<TDescriptor, F>::getDatabase() const
{
  return *m_database;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
inline const TemplatedVocabulary<TDescriptor, F>&
TemplatedLoopDetector_server<TDescriptor, F>::getVocabulary() const
{
  return m_database->getVocabulary();
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor, F>::addToDatabase(
  const std::vector<cv::KeyPoint> &keys,
  const std::vector<TDescriptor> &descriptors){
    
    
    BowVector bowvec;
    FeatureVector featvec;
    
   
        EntryId entry_id = m_database->size();
//   std::cout<<"entry_id "<<entry_id<<" "<<m_database<<std::endl;
        
    if(m_params.geom_check == GEOM_DI){
//        const TemplatedVocabulary<TDescriptor,F>* m_voc_test=m_database->getVocabulary();
//        std::cout<<"getScoringType "<<m_voc_test->getScoringType()<<std::endl;
          m_database->getVocabulary()->transform(descriptors, bowvec, featvec,
            m_params.di_levels);
    }else{
          m_database->getVocabulary()->transform(descriptors, bowvec);
    }
        
        m_database->add(bowvec, featvec);
        
        
        //keys的存储
        if(m_image_keys.size() == entry_id)
        {
          //printf("image size = entry_id %d\n", entry_id);
          m_image_keys.push_back(keys);
          m_image_descriptors.push_back(descriptors);
        }
        else
        {
          //printf("image size != entry_id %d\n", entry_id);
          m_image_keys[entry_id] = keys;
          m_image_descriptors[entry_id] = descriptors;
        }
        
//        if(m_params.use_nss)
//        {
//            m_last_bowvec = bowvec;//不确定是不是这样
//        }
//        std::cout<<"entry_id "<<entry_id<<" "<<m_database<<std::endl;

}


template<class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor, F>::addToDatabase_2(
  const std::vector<cv::KeyPoint> &keys,
  const std::vector<TDescriptor> &descriptors, const BowVector &bowvec, const FeatureVector &featvec){
    
//    BowVector bowvec;
//    FeatureVector featvec;
    
    
   
        EntryId entry_id = m_database->size();
   
        
//        if(m_params.geom_check == GEOM_DI)
//          m_database->getVocabulary()->transform(descriptors, bowvec, featvec,
//            m_params.di_levels);
//        else
//          m_database->getVocabulary()->transform(descriptors, bowvec);
        
        m_database->add(bowvec, featvec);
        
        
        //keys的存储
        if(m_image_keys.size() == entry_id)
        {
          //printf("image size = entry_id %d\n", entry_id);
          m_image_keys.push_back(keys);
          m_image_descriptors.push_back(descriptors);
        }
        else
        {
          //printf("image size != entry_id %d\n", entry_id);
          m_image_keys[entry_id] = keys;
          m_image_descriptors[entry_id] = descriptors;
        }
        
//        if(m_params.use_nss)
//        {
//            m_last_bowvec = bowvec;//不确定是不是这样
//        }
        

}


template<class TDescriptor, class F>
int TemplatedLoopDetector_server<TDescriptor, F>::getDatabase_size(){
    return m_database->size();
}


template<class TDescriptor, class F>
bool TemplatedLoopDetector_server<TDescriptor, F>::detectLoop(
  const std::vector<cv::KeyPoint> &keys,
  const std::vector<TDescriptor> &descriptors,
  DetectionResult_server &match,
  std::vector<cv::Point2f> &cur_pts,
  std::vector<cv::Point2f> &old_pts,int &min_startDetect_id,int cur_kf_global_index, int clientId)
{
  EntryId entry_id = m_database->size();
  match.query = entry_id;
    
  BowVector bowvec;
  FeatureVector featvec;
  
//    cout<<"m_params.di_levels="<<m_params.di_levels<<endl;//这个分层 弄清楚 下标有什么用 这次变成2了，没弄清楚这个做什么用的
  if(m_params.geom_check == GEOM_DI)
    m_database->getVocabulary()->transform(descriptors, bowvec, featvec,
      m_params.di_levels);
  else
    m_database->getVocabulary()->transform(descriptors, bowvec);

//  if((int)entry_id <= m_params.dislocal)
//  {
//    // only add the entry to the database and finish
//    m_database->add(bowvec, featvec);
//    match.status = CLOSE_MATCHES_ONLY;
//    //printf("entry_id %d <= m_params.dislocal %d\n",entry_id,m_params.dislocal);
//  }
    
//    if((int)entry_id<=min_startDetect_id){
//        match.status = CLOSE_MATCHES_ONLY_server;//还没有新的帧 说明检测的太频繁了
//        cout<<"回环检测太频繁了"<<endl;
////        cout<<"min_startDetect_id="<<min_startDetect_id<<" entry_id="<<entry_id<<endl;
//    }
// else
  {
    //printf("entry_id %d > m_params.dislocal %d\n",entry_id,m_params.dislocal);
//    int max_id = (int)entry_id - m_params.dislocal;
      int num=0;
      for(auto iter=featvec.begin(),iter_end=featvec.end();iter!=iter_end;iter++){
          std::vector<unsigned int> h=iter->second;
          num+=h.size();
          
      }
//      std::cout<<"descriptors="<<descriptors.size()<<" , "<<keys.size()<<" , "<<bowvec.size()<<" , "<<featvec.size()<<num <<std::endl;
    
    QueryResults qret;
    m_database->query(bowvec, qret, m_params.max_db_results, min_startDetect_id);// 这里应该是max_id entry_id-min_startDetect_id 是对的

    // update database
//    m_database->add(bowvec, featvec); // returns entry_id 因为只和主地图做匹配 所以这里并不添加该帧
    
    if(!qret.empty())
    {
      //printf("qret not empty\n");
      // factor to compute normalized similarity score, if necessary
      double ns_factor = 1.0;
      
      if(m_params.use_nss)
      {
        ns_factor = m_database->getVocabulary()->score(bowvec, m_last_bowvec[clientId]);
//        printf("当前帧序号，%d loop use_nss, ns_factor : %lf\n",cur_kf_global_index, ns_factor);
      }
      
      if(!m_params.use_nss || ns_factor >= m_params.min_nss_factor)
      {
        // scores in qret must be divided by ns_factor to obtain the
        // normalized similarity score, but we can
        // speed this up by moving ns_factor to alpha's
        
        // remove those scores whose nss is lower than alpha
        // (ret is sorted in descending score order now)
        //printf("ns_factor %lf >= m_params.min_nss_factor %lf\n",ns_factor,m_params.min_nss_factor);
          /**0.0042 0.014
                            0.0069 0.023
           0.0075 0.025
           */
        removeLowScores(qret, m_params.alpha * ns_factor);//最终0.0069 0.023
        
        if(!qret.empty())
        {
          //printf("qret not empty\n");
          // the best candidate is the one with highest score by now
          match.match = qret[0].Id;
          
          // compute islands
          vector<tIsland> islands;
          computeIslands(qret, islands);
          // this modifies qret and changes the score order
          
          // get best island
          if(!islands.empty())
          {
            
            const tIsland& island = *std::max_element(islands.begin(), islands.end());//最大值
            //printf("loop chose id:%d socre:%f------------\n",island.best_entry, island.best_score);
            // check temporal consistency of this island
            updateTemporalWindow(island, cur_kf_global_index, clientId);//第二个参数 其实是去做匹配的帧的id
            
            // get the best candidate (maybe match)
            match.match = island.best_entry;
//              cout<<" 同样的回环检测处 island.best_entry:"<<island.best_entry<<endl;
            
//            if(getConsistentEntries() > m_params.k)
            {
              //printf("temporal consistent entries:%d > n_params.k:%d\n", getConsistentEntries(),m_params.k);
              // candidate loop detected
              // check geometry
              bool detection;

              if(m_params.geom_check == GEOM_DI)
              {
                //printf("loop use direct index for geometrical checking\n");
                // all the DI stuff is implicit in the database
                detection = isGeometricallyConsistent_DI(island.best_entry,
                  keys, descriptors, featvec, cur_pts, old_pts);
              }
              else // GEOM_NONE, accept the match
              {
                detection = true;
                //printf("don't check detec true\n");
              }
              
              if(detection)
              {
                match.status = LOOP_DETECTED_server;
                printf("LOOP_DETECTED\n");
              }
              else
              {
                match.status = NO_GEOMETRICAL_CONSISTENCY_server;
                
              }
              
            } // if enough temporal matches
//            else
//            {
//              match.status = NO_TEMPORAL_CONSISTENCY_server;
//              //printf("no temporal consistentcy\n");
//            }
            
          } // if there is some island
          else
          {
            match.status = NO_GROUPS_server;
            //printf("in island\n");
          }
        } // if !qret empty after removing low scores
        else
        {
          match.status = LOW_SCORES_server;
          //printf("query empty after remove low score\n");
        }
      } // if (ns_factor > min normal score)
      else
      {
        //printf("low nss factor may be rotation\n");
        match.status = LOW_NSS_FACTOR_server;
      }
    } // if(!qret.empty())
    else
    {
      match.status = NO_DB_RESULTS_server;
      //printf("data base no result\n");
    }
      
      min_startDetect_id=entry_id-1;
  }

  // update record
  // m_image_keys and m_image_descriptors have the same length
//  if(m_image_keys.size() == entry_id)
//  {
//    //printf("image size = entry_id %d\n", entry_id);
//    m_image_keys.push_back(keys);
//    m_image_descriptors.push_back(descriptors);
//  }
//  else
//  {
//    //printf("image size != entry_id %d\n", entry_id);
//    m_image_keys[entry_id] = keys;
//    m_image_descriptors[entry_id] = descriptors;
//  }
//
//  // store this bowvec if we are going to use it in next iteratons
  if(m_params.use_nss && (int)entry_id + 1 > m_params.dislocal)
  {
    m_last_bowvec[clientId] = bowvec;
//      last_id=cur_kf_global_index;
  }

  return match.detection();
}

template<class TDescriptor, class F>
bool TemplatedLoopDetector_server<TDescriptor, F>::detectLoop2(
  const std::vector<cv::KeyPoint> &keys,
  const std::vector<TDescriptor> &descriptors,
  DetectionResult_server &match,
  std::vector<cv::Point2f> &cur_pts,
  std::vector<cv::Point2f> &old_pts,int &min_startDetect_id,int cur_kf_global_index, int clientId,int keyPoint_num_main,std::vector<cv::Point2f> &cur_pts_3d, std::vector<cv::Point2f> &old_pts_2d)
{
  EntryId entry_id = m_database->size();
  match.query = entry_id;
    
  BowVector bowvec;
  FeatureVector featvec;
  
//    cout<<"m_params.di_levels="<<m_params.di_levels<<endl;//这个分层 弄清楚 下标有什么用 这次变成2了，没弄清楚这个做什么用的
  if(m_params.geom_check == GEOM_DI)
    m_database->getVocabulary()->transform(descriptors, bowvec, featvec,
      m_params.di_levels);
  else
    m_database->getVocabulary()->transform(descriptors, bowvec);

//  if((int)entry_id <= m_params.dislocal)
//  {
//    // only add the entry to the database and finish
//    m_database->add(bowvec, featvec);
//    match.status = CLOSE_MATCHES_ONLY;
//    //printf("entry_id %d <= m_params.dislocal %d\n",entry_id,m_params.dislocal);
//  }
    
//    if((int)entry_id<=min_startDetect_id){
//        match.status = CLOSE_MATCHES_ONLY_server;//还没有新的帧 说明检测的太频繁了
//        cout<<"回环检测太频繁了"<<endl;
////        cout<<"min_startDetect_id="<<min_startDetect_id<<" entry_id="<<entry_id<<endl;
//    }
// else
  {
    //printf("entry_id %d > m_params.dislocal %d\n",entry_id,m_params.dislocal);
//    int max_id = (int)entry_id - m_params.dislocal;
      int num=0;
      for(auto iter=featvec.begin(),iter_end=featvec.end();iter!=iter_end;iter++){
          std::vector<unsigned int> h=iter->second;
          num+=h.size();
          
      }
//      std::cout<<"descriptors="<<descriptors.size()<<" , "<<keys.size()<<" , "<<bowvec.size()<<" , "<<featvec.size()<<num <<std::endl;
    
    QueryResults qret;
    m_database->query(bowvec, qret, m_params.max_db_results, min_startDetect_id);// 这里应该是max_id entry_id-min_startDetect_id 是对的

    // update database
//    m_database->add(bowvec, featvec); // returns entry_id 因为只和主地图做匹配 所以这里并不添加该帧
    
    if(!qret.empty())
    {
      //printf("qret not empty\n");
      // factor to compute normalized similarity score, if necessary
      double ns_factor = 1.0;
      
      if(m_params.use_nss)
      {
        ns_factor = m_database->getVocabulary()->score(bowvec, m_last_bowvec[clientId]);
//        printf("当前帧序号，%d loop use_nss, ns_factor : %lf\n",cur_kf_global_index, ns_factor);
      }
      
      if(!m_params.use_nss || ns_factor >= m_params.min_nss_factor)
      {
        // scores in qret must be divided by ns_factor to obtain the
        // normalized similarity score, but we can
        // speed this up by moving ns_factor to alpha's
        
        // remove those scores whose nss is lower than alpha
        // (ret is sorted in descending score order now)
        //printf("ns_factor %lf >= m_params.min_nss_factor %lf\n",ns_factor,m_params.min_nss_factor);
          /**0.0042 0.014
                            0.0069 0.023
           0.0075 0.025
           */
        removeLowScores(qret, m_params.alpha * ns_factor);//最终0.0069 0.023
        
        if(!qret.empty())
        {
          //printf("qret not empty\n");
          // the best candidate is the one with highest score by now
          match.match = qret[0].Id;
          
          // compute islands
          vector<tIsland> islands;
          computeIslands(qret, islands);
          // this modifies qret and changes the score order
          
          // get best island
          if(!islands.empty())
          {
            
            const tIsland& island = *std::max_element(islands.begin(), islands.end());//最大值
            //printf("loop chose id:%d socre:%f------------\n",island.best_entry, island.best_score);
            // check temporal consistency of this island
            updateTemporalWindow(island, cur_kf_global_index, clientId);//第二个参数 其实是去做匹配的帧的id
            
            // get the best candidate (maybe match)
            match.match = island.best_entry;
//              cout<<" 同样的回环检测处 island.best_entry:"<<island.best_entry<<endl;
            
//            if(getConsistentEntries() > m_params.k)
            {
              //printf("temporal consistent entries:%d > n_params.k:%d\n", getConsistentEntries(),m_params.k);
              // candidate loop detected
              // check geometry
              bool detection;

              if(m_params.geom_check == GEOM_DI)
              {
                //printf("loop use direct index for geometrical checking\n");
                // all the DI stuff is implicit in the database
                  
                detection = isGeometricallyConsistent_DI2(island.best_entry,
                  keys, descriptors, featvec, cur_pts, old_pts,keyPoint_num_main,cur_pts_3d,
                                                          old_pts_2d);
              }
              else // GEOM_NONE, accept the match
              {
                detection = true;
                //printf("don't check detec true\n");
              }
              
              if(detection)
              {
                match.status = LOOP_DETECTED_server;
                printf("LOOP_DETECTED\n");
              }
              else
              {
                match.status = NO_GEOMETRICAL_CONSISTENCY_server;
                
              }
              
            } // if enough temporal matches
//            else
//            {
//              match.status = NO_TEMPORAL_CONSISTENCY_server;
//              //printf("no temporal consistentcy\n");
//            }
            
          } // if there is some island
          else
          {
            match.status = NO_GROUPS_server;
            //printf("in island\n");
          }
        } // if !qret empty after removing low scores
        else
        {
          match.status = LOW_SCORES_server;
          //printf("query empty after remove low score\n");
        }
      } // if (ns_factor > min normal score)
      else
      {
        //printf("low nss factor may be rotation\n");
        match.status = LOW_NSS_FACTOR_server;
      }
    } // if(!qret.empty())
    else
    {
      match.status = NO_DB_RESULTS_server;
      //printf("data base no result\n");
    }
      
      min_startDetect_id=entry_id-1;
  }

  // update record
  // m_image_keys and m_image_descriptors have the same length
//  if(m_image_keys.size() == entry_id)
//  {
//    //printf("image size = entry_id %d\n", entry_id);
//    m_image_keys.push_back(keys);
//    m_image_descriptors.push_back(descriptors);
//  }
//  else
//  {
//    //printf("image size != entry_id %d\n", entry_id);
//    m_image_keys[entry_id] = keys;
//    m_image_descriptors[entry_id] = descriptors;
//  }
//
//  // store this bowvec if we are going to use it in next iteratons
  if(m_params.use_nss && (int)entry_id + 1 > m_params.dislocal)
  {
    m_last_bowvec[clientId] = bowvec;
//      last_id=cur_kf_global_index;
  }

  return match.detection();
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
inline void TemplatedLoopDetector_server<TDescriptor, F>::clear(int clientId)
{
  m_database->clear();
  m_window[clientId].nentries = 0;
    std::cout<<"TemplatedLoopDetector_server clear"<<std::endl;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor, F>::computeIslands
  (QueryResults &q, vector<tIsland> &islands) const
{
  islands.clear();
  
  if(q.size() == 1)
  {
    islands.push_back(tIsland(q[0].Id, q[0].Id, calculateIslandScore(q, 0, 0)));
    islands.back().best_entry = q[0].Id;
    islands.back().best_score = q[0].Score;
  }
  else if(!q.empty())
  {
    // sort query results in ascending order of ids
    std::sort(q.begin(), q.end(), Result::ltId);
    
    // create long enough islands
    QueryResults::const_iterator dit = q.begin();
    int first_island_entry = dit->Id;
    int last_island_entry = dit->Id;
    
    // these are indices of q
    unsigned int i_first = 0;
    unsigned int i_last = 0;
    
    double best_score = dit->Score;
    EntryId best_entry = dit->Id;

    ++dit;
    for(unsigned int idx = 1; dit != q.end(); ++dit, ++idx)
    {
      if((int)dit->Id - last_island_entry < m_params.max_intragroup_gap)
      {
        // go on until find the end of the island
        last_island_entry = dit->Id;
        i_last = idx;
        if(dit->Score > best_score)
        {
          best_score = dit->Score;
          best_entry = dit->Id;
        }
      }
      else
      {
        // end of island reached
        int length = last_island_entry - first_island_entry + 1;
        if(length >= m_params.min_matches_per_group)
        {
          islands.push_back( tIsland(first_island_entry, last_island_entry,
            calculateIslandScore(q, i_first, i_last)) );
          
          islands.back().best_score = best_score;
          islands.back().best_entry = best_entry;
        }
        
        // prepare next island
        first_island_entry = last_island_entry = dit->Id;
        i_first = i_last = idx;
        best_score = dit->Score;
        best_entry = dit->Id;
      }
    }
    // add last island
    if(last_island_entry - first_island_entry + 1 >=
      m_params.min_matches_per_group)
    {
      islands.push_back( tIsland(first_island_entry, last_island_entry,
        calculateIslandScore(q, i_first, i_last)) );
        
      islands.back().best_score = best_score;
      islands.back().best_entry = best_entry;
    }
  }
  
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
double TemplatedLoopDetector_server<TDescriptor, F>::calculateIslandScore(
  const QueryResults &q, unsigned int i_first, unsigned int i_last) const
{
  // get the sum of the scores
  double sum = 0;
  while(i_first <= i_last) sum += q[i_first++].Score;
  return sum;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor, F>::updateTemporalWindow
  (const tIsland &matched_island, EntryId entry_id, int clientId)
{
  // if m_window.nentries > 0, island > m_window.last_matched_island and
  // entry_id > m_window.last_query_id hold
//    cout<<"max_distance_between_queries="<<m_params.max_distance_between_queries<<endl;
    //max_distance_between_queries=2
  if(m_window[clientId].nentries == 0 || int(entry_id - m_window[clientId].last_query_id)
    > m_params.max_distance_between_queries)
  {
    m_window[clientId].nentries = 1;//窗口中满足一致性约束的帧的数量
  }
  else
  {
    EntryId a1 = m_window[clientId].last_matched_island.first;//孤岛的开始位置
    EntryId a2 = m_window[clientId].last_matched_island.last;//孤岛的结束位置
    EntryId b1 = matched_island.first;
    EntryId b2 = matched_island.last;
    
    bool fit = (b1 <= a1 && a1 <= b2) || (a1 <= b1 && b1 <= a2);

    if(!fit)
    {
      int d1 = (int)a1 - (int)b2;
      int d2 = (int)b1 - (int)a2;
      int gap = (d1 > d2 ? d1 : d2);
      
      fit = (gap <= m_params.max_distance_between_groups);
    }
    
    if(fit) m_window[clientId].nentries++;
    else m_window[clientId].nentries = 1;
  }
  
  m_window[clientId].last_matched_island = matched_island;
  m_window[clientId].last_query_id = entry_id;
}

// --------------------------------------------------------------------------
template<class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor, F>::reduceInputToOutput(vector<cv::Point2f> v_in, vector<cv::Point2f> &v_out, vector<uchar> status)
{
    v_out.clear();
    //v_out.reserve(v_in.size());
    for (int i = 0; i < int(v_in.size()); i++)
        if (status[i])
            v_out.push_back(v_in[i]);
}

template<class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor, F>::reduceInputToOutput(vector<cv::Point2f> v_in, vector<cv::Point2f> &v_out, vector<uchar> status, vector<uchar> status2, vector<cv::Point2f> &v_out_3d)
{
    v_out.clear();
    v_out_3d.clear();
    //v_out.reserve(v_in.size());
    for (int i = 0; i < int(v_in.size()); i++){
        if (status[i]){
            v_out.push_back(v_in[i]);
            if (status2[i]){
                v_out_3d.push_back(v_in[i]);
            }
        }
    }
        
}

template<class TDescriptor, class F>
bool TemplatedLoopDetector_server<TDescriptor, F>::checkFoundamental(vector<cv::Point2f> cur_input_pts,
                                                              vector<cv::Point2f> old_input_pts,
                                                              vector<cv::Point2f> &cur_output_pts,
                                                              vector<cv::Point2f> &old_output_pts)
{
    if (cur_input_pts.size() >= 8)
    {
        vector<uchar> status;
        cv::findFundamentalMat(old_input_pts, cur_input_pts, cv::FM_RANSAC, 1.0, 0.99, status);
        reduceInputToOutput( cur_input_pts, cur_output_pts, status);
        reduceInputToOutput( old_input_pts, old_output_pts, status);
        if(cur_output_pts.size()>20)
        {
          return true;
        }
        else{
//            cout<<"cur_output_pts.size()<20  :"<<cur_output_pts.size()<<endl;
          return false;
        }
    }
    else
    {
      return false;
    }
}

template<class TDescriptor, class F>
bool TemplatedLoopDetector_server<TDescriptor, F>::checkFoundamental2(vector<cv::Point2f> cur_input_pts, vector<cv::Point2f> old_input_pts,   vector<cv::Point2f> &cur_output_pts,
       vector<cv::Point2f> &old_output_pts,vector<uchar> &status2,   vector<cv::Point2f> &cur_output_pts_3d, vector<cv::Point2f> &old_output_pts_3d)
{
    if (cur_input_pts.size() >= 8)
    {
        vector<uchar> status;
        cv::findFundamentalMat(old_input_pts, cur_input_pts, cv::FM_RANSAC, 1.0, 0.99, status);
        reduceInputToOutput( cur_input_pts, cur_output_pts, status,status2,cur_output_pts_3d);
        reduceInputToOutput( old_input_pts, old_output_pts, status,status2,old_output_pts_3d);
        if(cur_output_pts.size()>20)
        {
            
          return true;
        }
        else{
//            cout<<"cur_output_pts.size()<20  :"<<cur_output_pts.size()<<endl;
          return false;
        }
    }
    else
    {
      return false;
    }
}

template<class TDescriptor, class F>
bool TemplatedLoopDetector_server<TDescriptor, F>::isGeometricallyConsistent_DI(
  EntryId old_entry, const std::vector<cv::KeyPoint> &keys,
  const std::vector<TDescriptor> &descriptors,
  const FeatureVector &bowvec,
  std::vector<cv::Point2f> &cur_pts,
  std::vector<cv::Point2f> &old_pts)
{
  const FeatureVector &oldvec = m_database->retrieveFeatures(old_entry);
  
  // for each word in common, get the closest descriptors
  
  vector<unsigned int> i_old, i_cur;
  
  FeatureVector::const_iterator old_it, cur_it;
  const FeatureVector::const_iterator old_end = oldvec.end();
  const FeatureVector::const_iterator cur_end = bowvec.end();
  
  old_it = oldvec.begin();
  cur_it = bowvec.begin();
  
  while(old_it != old_end && cur_it != cur_end)
  {
    if(old_it->first == cur_it->first)
    {
      // compute matches between
      // features old_it->second of m_image_keys[old_entry] and
      // features cur_it->second of keys
      vector<unsigned int> i_old_now, i_cur_now;
      
      getMatches_neighratio(
        m_image_descriptors[old_entry], old_it->second,
        descriptors, cur_it->second,
        i_old_now, i_cur_now);
      
      i_old.insert(i_old.end(), i_old_now.begin(), i_old_now.end());
      i_cur.insert(i_cur.end(), i_cur_now.begin(), i_cur_now.end());
      
      // move old_it and cur_it forward
      ++old_it;
      ++cur_it;
    }
    else if(old_it->first < cur_it->first)
    {
      // move old_it forward
      old_it = oldvec.lower_bound(cur_it->first);
      // old_it = (first element >= cur_it.id)
    }
    else
    {
      // move cur_it forward
      cur_it = bowvec.lower_bound(old_it->first);
      // cur_it = (first element >= old_it.id)
    }
  }
  
  // calculate now the fundamental matrix
  if((int)i_old.size() >= m_params.min_Fpoints)
  {
    vector<cv::Point2f> old_points, cur_points;
    
    // add matches to the vectors to calculate the fundamental matrix
    vector<unsigned int>::const_iterator oit, cit;
    oit = i_old.begin();
    cit = i_cur.begin();
    
    for(; oit != i_old.end(); ++oit, ++cit)
    {
      const cv::KeyPoint &old_k = m_image_keys[old_entry][*oit];
      const cv::KeyPoint &cur_k = keys[*cit];
      
      old_points.push_back(old_k.pt);
      cur_points.push_back(cur_k.pt);
    }
  
    cv::Mat oldMat(old_points.size(), 2, CV_32F, &old_points[0]);
    cv::Mat curMat(cur_points.size(), 2, CV_32F, &cur_points[0]);
    //printf("-old size %d current size %d\n",old_points.size(),cur_points.size());
    bool find_fundamental_suss = false;
    find_fundamental_suss =  checkFoundamental(cur_points, old_points, cur_pts, old_pts);
    if(find_fundamental_suss)
    {
      //cur_pts = cur_points;
      //old_pts = old_points;
      return true;
    }
  }
//   cout<<"(int)i_old.size()="<< (int)i_old.size() <<" min_Fpoints="<<m_params.min_Fpoints<<endl;
  
  return false;
}


template<class TDescriptor, class F>
bool TemplatedLoopDetector_server<TDescriptor, F>::isGeometricallyConsistent_DI2(
  EntryId old_entry, const std::vector<cv::KeyPoint> &keys,
  const std::vector<TDescriptor> &descriptors,
  const FeatureVector &bowvec,
  std::vector<cv::Point2f> &cur_pts,
  std::vector<cv::Point2f> &old_pts,int keyPoint_num_main,std::vector<cv::Point2f> &cur_pts_3d,
  std::vector<cv::Point2f> &old_pts_2d)
{
  const FeatureVector &oldvec = m_database->retrieveFeatures(old_entry);
  
  // for each word in common, get the closest descriptors
  
  vector<unsigned int> i_old, i_cur;
  
  FeatureVector::const_iterator old_it, cur_it;
  const FeatureVector::const_iterator old_end = oldvec.end();
  const FeatureVector::const_iterator cur_end = bowvec.end();
  
  old_it = oldvec.begin();
  cur_it = bowvec.begin();
  
  while(old_it != old_end && cur_it != cur_end)
  {
    if(old_it->first == cur_it->first)
    {
      // compute matches between
      // features old_it->second of m_image_keys[old_entry] and
      // features cur_it->second of keys
      vector<unsigned int> i_old_now, i_cur_now;
      
      getMatches_neighratio(
        m_image_descriptors[old_entry], old_it->second,
        descriptors, cur_it->second,
        i_old_now, i_cur_now);
      
      i_old.insert(i_old.end(), i_old_now.begin(), i_old_now.end());
      i_cur.insert(i_cur.end(), i_cur_now.begin(), i_cur_now.end());
      
      // move old_it and cur_it forward
      ++old_it;
      ++cur_it;
    }
    else if(old_it->first < cur_it->first)
    {
      // move old_it forward
      old_it = oldvec.lower_bound(cur_it->first);
      // old_it = (first element >= cur_it.id)
    }
    else
    {
      // move cur_it forward
      cur_it = bowvec.lower_bound(old_it->first);
      // cur_it = (first element >= old_it.id)
    }
  }
    vector<uchar> status2;
  // calculate now the fundamental matrix
  if((int)i_old.size() >= m_params.min_Fpoints)
  {
    vector<cv::Point2f> old_points, cur_points;
      
      vector<cv::Point2f> old_points_test, cur_points_test;
    
    // add matches to the vectors to calculate the fundamental matrix
    vector<unsigned int>::const_iterator oit, cit;
    oit = i_old.begin();
    cit = i_cur.begin();
    
      int num_test=0;
    for(; oit != i_old.end(); ++oit, ++cit)
    {
      const cv::KeyPoint &old_k = m_image_keys[old_entry][*oit];
      const cv::KeyPoint &cur_k = keys[*cit];
      
      old_points.push_back(old_k.pt);
      cur_points.push_back(cur_k.pt);
        
        if((*cit)>=keyPoint_num_main){
            status2.push_back(1);
            cur_points_test.push_back(cur_k.pt);
            old_points_test.push_back(old_k.pt);
            num_test++;
        }else{
            status2.push_back(0);
        }
    }
//      std::cout<<"找到的点数："<<num_test<<std::endl;
//      checkFoundamental(cur_points_test, old_points_test, cur_pts_3d, old_pts_2d);
//      std::cout<<"匹配的点数1："<<cur_pts_3d.size()<<std::endl;
//    cv::Mat oldMat(old_points.size(), 2, CV_32F, &old_points[0]);
//    cv::Mat curMat(cur_points.size(), 2, CV_32F, &cur_points[0]);
    //printf("-old size %d current size %d\n",old_points.size(),cur_points.size());
    bool find_fundamental_suss = false;
    find_fundamental_suss =  checkFoundamental2(cur_points, old_points, cur_pts, old_pts,status2,cur_pts_3d,old_pts_2d);
//      std::cout<<"匹配的点数2："<<cur_pts_3d.size()<<std::endl;
    if(find_fundamental_suss)
    {
      //cur_pts = cur_points;
      //old_pts = old_points;
      return true;
    }
  }
//   cout<<"(int)i_old.size()="<< (int)i_old.size() <<" min_Fpoints="<<m_params.min_Fpoints<<endl;
  
  return false;
}
// --------------------------------------------------------------------------
template<class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor, F>::getFlannStructure(
  const std::vector<TDescriptor> &descriptors,
  cv::FlannBasedMatcher &flann_structure) const
{
  vector<cv::Mat> features(1);
  F::toMat32F(descriptors, features[0]);
  
  flann_structure.clear();
  flann_structure.add(features);
  flann_structure.train();
}


// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor, F>::getMatches_neighratio(
  const vector<TDescriptor> &A, const vector<unsigned int> &i_A,
  const vector<TDescriptor> &B, const vector<unsigned int> &i_B,
  vector<unsigned int> &i_match_A, vector<unsigned int> &i_match_B) const
{
  i_match_A.resize(0);
  i_match_B.resize(0);
  i_match_A.reserve( min(i_A.size(), i_B.size()) );
  i_match_B.reserve( min(i_A.size(), i_B.size()) );
  
  vector<unsigned int>::const_iterator ait, bit;
  unsigned int i, j;
  i = 0;
  for(ait = i_A.begin(); ait != i_A.end(); ++ait, ++i)
  {
    int best_j_now = -1;
    double best_dist_1 = 1e9;
    double best_dist_2 = 1e9;
    
    j = 0;
    for(bit = i_B.begin(); bit != i_B.end(); ++bit, ++j)
    {
      double d = F::distance(A[*ait], B[*bit]);
            
      // in i
      if(d < best_dist_1)
      {
        best_j_now = j;
        best_dist_2 = best_dist_1;
        best_dist_1 = d;
      }
      else if(d < best_dist_2)
      {
        best_dist_2 = d;
      }
    }
    
    if(best_dist_1 / best_dist_2 <= m_params.max_neighbor_ratio)//0.6
    {
      unsigned int idx_B = i_B[best_j_now];
      bit = find(i_match_B.begin(), i_match_B.end(), idx_B);
      
      if(bit == i_match_B.end())
      {
        i_match_B.push_back(idx_B);
        i_match_A.push_back(*ait);
      }
      else
      {
        unsigned int idx_A = i_match_A[ bit - i_match_B.begin() ];
        double d = F::distance(A[idx_A], B[idx_B]);
        if(best_dist_1 < d)
        {
          i_match_A[ bit - i_match_B.begin() ] = *ait;
        }
      }
        
    }
  }
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor, F>::removeLowScores(QueryResults &q,
  double threshold) const
{
  // remember scores in q are in descending order now
  //QueryResults::iterator qit =
  //  lower_bound(q.begin(), q.end(), threshold, Result::geqv);
//    cout<<"removeLowScores before ";
//    for(auto iter=q.begin(),iter_end=q.end();iter!=iter_end;iter++){
//        cout<<(*iter).Score<<" ";
//    }
//    cout<<endl;
//    cout<<"threshold "<<threshold<<endl;
    
  Result aux(0, threshold);
  QueryResults::iterator qit =
    lower_bound(q.begin(), q.end(), aux, Result::geq);
  
  // qit = first element < m_alpha_minus || end
  
  if(qit != q.end())
  {
    int valid_entries = qit - q.begin();
    q.resize(valid_entries);
  }
}

// --------------------------------------------------------------------------
template<class TDescriptor, class F>
void TemplatedLoopDetector_server<TDescriptor, F>::eraseIndex
(std::vector<int> &erase_index)
{
    cout<<"eraseIndex"<<endl;
        for (int i = 0; i < (int)erase_index.size(); i++)
        {
            DBoW2::EntryId entry;
            entry = (unsigned int)erase_index[i];
            m_database->delete_entry(entry);
        }
}
}



#endif /* TemplatedLoopDetector_server_h */
