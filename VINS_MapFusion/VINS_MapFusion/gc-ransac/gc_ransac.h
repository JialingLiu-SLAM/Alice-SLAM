//
//  gc_ransac.h
//  VINS_MapFusion
//
//  Created by 张剑华 on 2022/8/7.
//  Copyright © 2022 zx. All rights reserved.
//
#include <opencv2/core.hpp>
#ifndef gc_ransac_h
#define gc_ransac_h


#include <vector>
#include <thread>
#include "utils.h"

#include <opencv2/calib3d.hpp>
#include <Eigen/Eigen>

#include "GCRANSAC.h"
//neighborhood
#include "flann_neighborhood_graph.h"
#include "grid_neighborhood_graph.h"
//samplers
#include "uniform_sampler.h"
#include "prosac_sampler.h"
#include "napsac_sampler.h"
#include "progressive_napsac_sampler.h"


//preemption
#include "preemption_sprt.h"
//inlier_selectors
#include "empty_inlier_selector.h"
#include "space_partitioning_ransac.h"

//estimators
#include "solver_p3p.h"


#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#endif

struct stat info;

enum Problem {
    LineFitting,
    PerspectiveNPointFitting,
    FundamentalMatrixFitting,
    EssentialMatrixFitting,
    HomographyFitting,
    RigidTransformationFitting
};



// An example function showing how to fit 6D pose to 2D-3D correspondences by Graph-Cut RANSAC
void test6DPoseFitting(
    const std::string& intrinsics_path_, // The path where the intrinsic camera matrix can be found
    const std::string& ground_truth_pose_path_, // The path where the ground truth pose can be found
    const std::string& points_path_, // The path of the points
    const std::string& inliers_point_path_, // The path where the inlier correspondences are saved
    const double confidence_, // The RANSAC confidence value
    const double inlier_outlier_threshold_, // The used inlier-outlier threshold in GC-RANSAC.
    const double spatial_coherence_weight_, // The weight of the spatial coherence term in the graph-cut energy minimization.
    const double sphere_radius_, // The radius of the sphere used for determining the neighborhood-graph
    const int fps_, // The required FPS limit. If it is set to -1, the algorithm will not be interrupted before finishing.
    const bool numerical_optimization_ = true, // A flag to decide if numerical optimization should be applied as a post-processing step
    const double minimum_inlier_ratio_for_sprt_ = 0.00001); // An assumption about the minimum inlier ratio used for the SPRT test



std::vector<std::string> getAvailableTestScenes(Problem problem_);



// Setting the paths for the data used in 6D pose fitting
bool initializeScenePnP(
    const std::string& scene_name_, // The name of the scene
    std::string& intrinsics_path_, // The path where the intrinsic parameters of the camera can be found
    std::string& ground_truth_pose_path_, // The path of the ground truth pose used for evaluating the results
    std::string& points_path_, // The path where the 2D-3D correspondences can be found
    std::string& inlier_points_path_, // The path where the inlier correspondences should be saved
    const std::string root_directory_ = "/Users/zhangjianhua/Desktop/gc-ransac/gc-ransac/"); // The root directory where the "results" and "data" folder are



using namespace gcransac;

void getInlier()
{
    srand(static_cast<int>(time(NULL)));

    const std::string data_directory = "/Users/zhangjianhua/Desktop/gc-ransac/gc-ransac/";
    const double confidence = 0.99; // The RANSAC confidence value
    const int fps = -1; // The required FPS limit. If it is set to -1, the algorithm will not be interrupted before finishing.
    const double inlier_outlier_threshold_essential_matrix = 3.00; // The used inlier-outlier threshold in GC-RANSAC for essential matrix estimation.
    const double inlier_outlier_threshold_fundamental_matrix = 0.0003; // The used adaptive (i.e., it is the percentage of the maximum image diagonal) inlier-outlier threshold in GC-RANSAC for fundamental matrix estimation.
    const double inlier_outlier_threshold_rigid_pose = 10.0; // The used adaptive (i.e., it is the percentage of the maximum image diagonal) inlier-outlier threshold in GC-RANSAC for fundamental matrix estimation.
    const double inlier_outlier_threshold_2d_line = 2.0; // The used adaptive (i.e., it is the percentage of the maximum image diagonal) inlier-outlier threshold in GC-RANSAC for fundamental matrix estimation.
    const double inlier_outlier_threshold_homography = 2.00; // The used inlier-outlier threshold in GC-RANSAC for homography estimation.
    const double inlier_outlier_threshold_pnp = 5.50; // The used inlier-outlier threshold in GC-RANSAC for homography estimation.
    const double spatial_coherence_weight = 0.975; // The weigd_t of the spatial coherence term in the graph-cut energy minimization.
    const size_t cell_number_in_neighborhood_graph = 4; // The number of cells along each axis in the neighborhood graph.

   


    printf("------------------------------------------------------------\n6D pose fitting by the PnP algorithm\n------------------------------------------------------------\n");
    for (const std::string& scene : getAvailableTestScenes(Problem::PerspectiveNPointFitting))
    {
        printf("Processed scene = '%s'\n", scene.c_str());
        std::string points_path, // Path of the image and world points
            intrinsics_path, // Path where the intrinsics camera matrix
            ground_truth_pose_path, // Path where the ground truth pose is found
            inlier_points_path; // Path where the inlier points are saved

        // Initializing the paths
        initializeScenePnP(scene,
            intrinsics_path,
            ground_truth_pose_path,
            points_path,
            inlier_points_path,
            data_directory);
        

        // Estimating the fundamental matrix by the Graph-Cut RANSAC algorithm
        test6DPoseFitting(
            intrinsics_path, // The path where the intrinsic camera matrix can be found
            ground_truth_pose_path, // Path where the ground truth pose is found
            points_path, // The path where the image and world points can be found
            inlier_points_path, // The path where the inlier points should be saved
            confidence, // The RANSAC confidence value
            inlier_outlier_threshold_pnp, // The used inlier-outlier threshold in GC-RANSAC.
            spatial_coherence_weight, // The weight of the spatial coherence term in the graph-cut energy minimization.
            20.0, // The radius of the neighborhood ball for determining the neighborhoods.
            fps); // The required FPS limit. If it is set to -1, the algorithm will not be interrupted before finishing.
        printf("\n------------------------------------------------------------\n");
    }

    
}

std::vector<std::string> getAvailableTestScenes(Problem problem_)
{
    switch (problem_)
    {
    case Problem::PerspectiveNPointFitting:
        return { "pose6dscene" };
    case Problem::LineFitting:
        return { "adam" };
    case Problem::FundamentalMatrixFitting:
        return { "head", "johnssona", "Kyoto" };
    case Problem::HomographyFitting:
        return { "graf", "Eiffel", "adam" };
    case Problem::RigidTransformationFitting:
        return { "kitchen" };
    default:
        return { "fountain" };
    }
}



bool initializeScenePnP(
    const std::string& scene_name_,
    std::string& intrinsics_path_,
    std::string& ground_truth_pose_path_,
    std::string& points_path_,
    std::string& inlier_image_points_path_,
    const std::string root_directory_) // The root directory where the "results" and "data" folder are
{
    // The directory to which the results will be saved
    std::string results_dir = root_directory_ + "results";

    // Create the task directory if it doesn't exist
    if (stat(results_dir.c_str(), &info) != 0) // Check if exists
    {
#ifdef _WIN32 // Create a directory on Windows
        if (_mkdir(results_dir.c_str()) != 0) // Create it, if not
        {
            fprintf(stderr, "Error while creating folder 'results'\n");
            return false;
        }
#else // Create a directory on Linux
        if (mkdir(results_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
        {
            fprintf(stderr, "Error while creating a new folder in 'results'\n");
            return false;
        }
#endif
    }

    // The directory to which the results will be saved
    std::string dir = root_directory_ + "results/" + scene_name_;

    // Create the task directory if it doesn't exist
    if (stat(dir.c_str(), &info) != 0) // Check if exists
    {
#ifdef _WIN32 // Create a directory on Windows
        if (_mkdir(dir.c_str()) != 0) // Create it, if not
        {
            fprintf(stderr, "Error while creating a new folder in 'results'\n");
            return false;
        }
#else // Create a directory on Linux
        if (mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
        {
            fprintf(stderr, "Error while creating a new folder in 'results'\n");
            return false;
        }
#endif
    }

    // The path where the intrinsics camera matrix of the source camera can be found
    intrinsics_path_ =
        root_directory_ + "data/" + scene_name_ + "/" + scene_name_ + ".K";
    // The path where the ground truth pose can be found
    ground_truth_pose_path_ =
        root_directory_ + "data/" + scene_name_ + "/" + scene_name_ + "_gt.txt";
    // The path where the 3D point cloud can be found
    points_path_ =
        root_directory_ + "data/" + scene_name_ + "/" + scene_name_ + "_points.txt";
    // The path where the inliers of the estimated fundamental matrices will be saved
    inlier_image_points_path_ =
        root_directory_ + "results/" + scene_name_ + "/result_" + scene_name_ + ".txt";

    return true;
}




void test6DPoseFitting(
    const std::string& intrinsics_path_,
    const std::string& ground_truth_pose_path_,
    const std::string& points_path_,
    const std::string& inlier_image_points_path_,
    const double confidence_,
    const double inlier_outlier_threshold_,
    const double spatial_coherence_weight_,
    const double sphere_radius_,
    const int fps_,
    const bool numerical_optimization_, // A flag to decide if numerical optimization should be applied as a post-processing step)
    const double minimum_inlier_ratio_for_sprt_) // An assumption about the minimum inlier ratio used for the SPRT test
{
    // The image and world points stored as rows in the matrix. Each row is of format 'u v x y z'.
    cv::Mat points;

    // Loading the image and world points
    gcransac::utils::loadPointsFromFile<5>(points, // The points in the image
        points_path_.c_str()); // The path where the image points are stored

    // Load the intrinsic camera matrix
    Eigen::Matrix3d intrinsics;

    if (!utils::loadMatrix<double, 3, 3>(intrinsics_path_,
        intrinsics))
    {
        printf("An error occured when loading the intrinsics camera matrix from '%s'\n",
            intrinsics_path_.c_str());
        return;
    }

    // Load the ground truth pose
    Eigen::Matrix<double, 3, 4> reference_pose;
    if (!utils::loadMatrix<double, 3, 4>(ground_truth_pose_path_,
        reference_pose))
    {
        printf("An error occured when loading the reference camera pose from '%s'\n",
            ground_truth_pose_path_.c_str());
        return;
    }

    // The ground truth rotation matrix
    Eigen::Matrix3d gt_rotation =
        reference_pose.leftCols<3>();

    // The ground truth translation matrix
    Eigen::Vector3d gt_translation =
        reference_pose.rightCols<1>();

    // Normalize the point coordinate by the intrinsic matrix
    cv::Rect roi(0, 0, 2, points.rows); // A rectangle covering the 2D image points in the matrix
    cv::Mat normalized_points = points.clone(); // The 2D image points normalized by the intrinsic camera matrix
    // Normalizing the image points by the camera matrix
    utils::normalizeImagePoints(
        points(roi), // The loaded image points
        intrinsics, // The intrinsic camera matrix
        normalized_points); // The normalized points

    // Normalize the threshold by the average of the focal lengths
    const double avg_focal_length =
        (intrinsics(0, 0) + intrinsics(1, 1)) / 2.0;
    const double normalized_threshold =
        inlier_outlier_threshold_ / avg_focal_length;

    // Initialize the neighborhood used in Graph-cut RANSAC and, perhaps,
    // in the sampler if NAPSAC or Progressive-NAPSAC sampling is applied.
    std::chrono::time_point<std::chrono::system_clock> start, end; // Variables for time measurement
    start = std::chrono::system_clock::now(); // The starting time of the neighborhood calculation
    // TODO: generalize the grid class to handle not only point correspondences
    neighborhood::FlannNeighborhoodGraph neighborhood(&points, // The data points
        sphere_radius_); // The radius of the neighborhood sphere used for determining the neighborhood structure
    end = std::chrono::system_clock::now(); // The end time of the neighborhood calculation
    std::chrono::duration<double> elapsed_seconds = end - start; // The elapsed time in seconds
    printf("Neighborhood calculation time = %f secs\n", elapsed_seconds.count());

    // Apply Graph-cut RANSAC
    utils::DefaultPnPEstimator estimator; // The estimator used for the pose fitting
    Pose6D model; // The estimated model parameters

    // Initialize the samplers
    // The main sampler is used inside the local optimization
    sampler::ProsacSampler main_sampler(&points, // The data points
        estimator.sampleSize()); // The size of a minimal sample
    sampler::UniformSampler local_optimization_sampler(&points); // The local optimization sampler is used inside the local optimization

    // Checking if the samplers are initialized successfully.
    if (!main_sampler.isInitialized() ||
        !local_optimization_sampler.isInitialized())
    {
        fprintf(stderr, "One of the samplers is not initialized successfully.\n");
        return;
    }

    // Initializing SPRT test
    preemption::SPRTPreemptiveVerfication<utils::DefaultPnPEstimator> preemptive_verification(
        points,
        estimator,
        minimum_inlier_ratio_for_sprt_);
    
    // Initializing the fast inlier selector object
    inlier_selector::EmptyInlierSelector<utils::DefaultPnPEstimator,
        neighborhood::FlannNeighborhoodGraph> inlier_selector(&neighborhood);

    GCRANSAC<utils::DefaultPnPEstimator,
        neighborhood::FlannNeighborhoodGraph,
        MSACScoringFunction<utils::DefaultPnPEstimator>,
        preemption::SPRTPreemptiveVerfication<utils::DefaultPnPEstimator>> gcransac;
    gcransac.settings.threshold = normalized_threshold; // The inlier-outlier threshold
    gcransac.settings.spatial_coherence_weight = spatial_coherence_weight_; // The weight of the spatial coherence term
    gcransac.settings.confidence = confidence_; // The required confidence in the results
    gcransac.settings.max_iteration_number = 5000; // The maximum number of iterations
    gcransac.settings.min_iteration_number = 20; // The minimum number of iterations
    gcransac.settings.neighborhood_sphere_radius = sphere_radius_; // The radius of the neighborhood ball

    // Start GC-RANSAC
    gcransac.run(normalized_points, // The normalized points
        estimator,  // The estimator
        &main_sampler, // The sample used for selecting minimal samples in the main iteration
        &local_optimization_sampler, // The sampler used for selecting a minimal sample when doing the local optimization
        &neighborhood, // The neighborhood-graph
        model, // The obtained model parameters
        preemptive_verification,
        inlier_selector);

    // Get the statistics of the results
    const utils::RANSACStatistics& statistics = gcransac.getRansacStatistics();

    printf("Elapsed time = %f secs\n", statistics.processing_time);
    printf("Inlier number before = %d\n", static_cast<int>(statistics.inliers.size()));
    printf("Applied number of local optimizations = %d\n", static_cast<int>(statistics.local_optimization_number));
    printf("Applied number of graph-cuts = %d\n", static_cast<int>(statistics.graph_cut_number));
    printf("Number of iterations = %d\n\n", static_cast<int>(statistics.iteration_number));

    // Save the inliers to file
    utils::savePointsToFile(points, // The loaded data points
        inlier_image_points_path_.c_str(), // The path where the results should be saved
        &statistics.inliers); // The set of inlier found

    // The estimated rotation matrix
    Eigen::Matrix3d rotation =
        model.descriptor.leftCols<3>();
    // The estimated translation
    Eigen::Vector3d translation =
        model.descriptor.rightCols<1>();
    
    Eigen::Quaterniond q_old( 0.568527,-0.151869   , -0.802613   , -0.097713 );
    Eigen::Matrix3d rotation_old =q_old.normalized().toRotationMatrix();
    Eigen::Vector3d translation_old ;
    translation_old<<4.703599,    -1.781153 ,   0.859616;
    
    gt_translation=gt_rotation*(-rotation_old.transpose()*translation_old)+gt_translation;
    gt_rotation=gt_rotation*rotation_old.transpose();
    
    translation=rotation*(-rotation_old.transpose()*translation_old)+translation;
    rotation=rotation*rotation_old.transpose();
    
    
   

    
    Eigen::Quaterniond q_old0(    -0.41657450795, 0.57345306873, 0.57021617889, 0.41529604793 );
    Eigen::Matrix3d rotation_old0 =q_old0.normalized().toRotationMatrix();
    Eigen::Vector3d translation_old0 ;
    translation_old0<<0.01580299065, 0.00637514656, 0.06530746818 ;
    
    Eigen::Quaterniond q_old20(  -0.31390127540,   0.65969997644 ,0.62579083443 ,0.27321726084 );
    Eigen::Matrix3d rotation_old20 =q_old20.normalized().toRotationMatrix();
    Eigen::Vector3d translation_old20 ;
    translation_old20<<0.07157045603 ,-0.02330756560, 0.14762423933;
    
    Eigen::Vector3d  translation_our=rotation_old20.transpose()*(translation_old0-translation_old20);
    Eigen::Matrix3d rotation_our=rotation_old20.transpose()*rotation_old0;

    // The number of inliers found
    const size_t inlier_number = statistics.inliers.size();

    // Calculate the estimation error
    constexpr double radian_to_degree_multiplier = 180.0 / M_PI;
    const double angular_error = radian_to_degree_multiplier * (Eigen::Quaterniond(
        rotation).angularDistance(Eigen::Quaterniond(gt_rotation)));
    const double translation_error = (gt_translation - translation).norm();

    printf("The error in the rotation matrix = %f degrees\n", angular_error);
    printf("The error in the translation = %f cm\n", translation_error / 10.0);
    
    // Calculate the estimation error
    constexpr double radian_to_degree_multiplier_our = 180.0 / M_PI;
    const double angular_error_our = radian_to_degree_multiplier_our * (Eigen::Quaterniond(
                                                                                   rotation_our).angularDistance(Eigen::Quaterniond(gt_rotation)));
    const double translation_error_our = (gt_translation - translation_our).norm();

    printf("The error in the rotation matrix = %f degrees\n", angular_error_our);
    printf("The error in the translation = %f cm\n", translation_error_our / 10.0);

    // If numerical optimization is needed, apply the Levenberg-Marquardt
    // implementation of OpenCV.
//    if (numerical_optimization_ && inlier_number >= 3)
//    {
//        // Copy the data into two matrices containing the image and object points.
//        // This would not be necessary, but selecting the submatrices by cv::Rect
//        // leads to an error in cv::solvePnP().
//        cv::Mat inlier_image_points(inlier_number, 2, CV_64F),
//            inlier_object_points(inlier_number, 3, CV_64F);
//
//        for (size_t i = 0; i < inlier_number; ++i)
//        {
//            const size_t& idx = statistics.inliers[i];
//            inlier_image_points.at<double>(i, 0) = normalized_points.at<double>(idx, 0);
//            inlier_image_points.at<double>(i, 1) = normalized_points.at<double>(idx, 1);
//            inlier_object_points.at<double>(i, 0) = points.at<double>(idx, 2);
//            inlier_object_points.at<double>(i, 1) = points.at<double>(idx, 3);
//            inlier_object_points.at<double>(i, 2) = points.at<double>(idx, 4);
//        }
//
//        // Converting the estimated pose parameters OpenCV format
//        cv::Mat cv_rotation(3, 3, CV_64F, rotation.data()), // The estimated rotation matrix converted to OpenCV format
//            cv_translation(3, 1, CV_64F, translation.data()); // The estimated translation converted to OpenCV format
//
//        // Convert the rotation matrix by the rodrigues formula
//        cv::Mat cv_rodrigues(3, 1, CV_64F);
//        cv::Rodrigues(cv_rotation.t(), cv_rodrigues);
//
//        // Applying numerical optimization to the estimated pose parameters
//        cv::solvePnP(inlier_object_points, // The object points
//            inlier_image_points, // The image points
//            cv::Mat::eye(3, 3, CV_64F), // The camera's intrinsic parameters
//            cv::Mat(), // An empty vector since the radial distortion is not known
//            cv_rodrigues, // The initial rotation
//            cv_translation, // The initial translation
//            true, // Use the initial values
//            cv::SOLVEPNP_ITERATIVE); // Apply numerical refinement
//
//        // Convert the rotation vector back to a rotation matrix
//        cv::Rodrigues(cv_rodrigues, cv_rotation);
//
//        // Transpose the rotation matrix back
//        cv_rotation = cv_rotation.t();
//
//        // Calculate the error of the refined pose
//        const double angular_error_refined = radian_to_degree_multiplier * (Eigen::Quaterniond(
//            rotation).angularDistance(Eigen::Quaterniond(gt_rotation)));
//        const double translation_error_refined = (gt_translation - translation).norm();
//
//        printf("The error in the rotation matrix after numerical refinement = %f degrees\n", angular_error_refined);
//        printf("The error in the translation after numerical refinement = %f cm\n", translation_error_refined / 10.0);
//    }
}

std::vector<size_t>  test6DPoseFitting2(
                        Eigen::Matrix3d intrinsics,
    const std::string& ground_truth_pose_path_,
                        cv::Mat points,
    const std::string& inlier_image_points_path_,
    const double confidence_,
    const double inlier_outlier_threshold_,
    const double spatial_coherence_weight_,
    const double sphere_radius_,
    const int fps_,
    Eigen::Matrix3d &r,
    Eigen::Vector3d &t,
    const bool numerical_optimization_=true, // A flag to decide if numerical optimization should be applied as a post-processing step)
    const double minimum_inlier_ratio_for_sprt_= 0.00001) // An assumption about the minimum inlier ratio used for the SPRT test
{
   
    // The ground truth rotation matrix
//    Eigen::Matrix3d gt_rotation =
//        reference_pose.leftCols<3>();
//
//    // The ground truth translation matrix
//    Eigen::Vector3d gt_translation = reference_pose.rightCols<1>();

    // Normalize the point coordinate by the intrinsic matrix
    cv::Rect roi(0, 0, 2, points.rows); // A rectangle covering the 2D image points in the matrix
    cv::Mat normalized_points = points.clone(); // The 2D image points normalized by the intrinsic camera matrix
    // Normalizing the image points by the camera matrix
    utils::normalizeImagePoints(
        points(roi), // The loaded image points
        intrinsics, // The intrinsic camera matrix
        normalized_points); // The normalized points

    // Normalize the threshold by the average of the focal lengths
    const double avg_focal_length =
        (intrinsics(0, 0) + intrinsics(1, 1)) / 2.0;
    const double normalized_threshold =
        inlier_outlier_threshold_ / avg_focal_length;

    // Initialize the neighborhood used in Graph-cut RANSAC and, perhaps,
    // in the sampler if NAPSAC or Progressive-NAPSAC sampling is applied.
    std::chrono::time_point<std::chrono::system_clock> start, end; // Variables for time measurement
    start = std::chrono::system_clock::now(); // The starting time of the neighborhood calculation
    // TODO: generalize the grid class to handle not only point correspondences
    neighborhood::FlannNeighborhoodGraph neighborhood(&points, // The data points
        sphere_radius_); // The radius of the neighborhood sphere used for determining the neighborhood structure
    end = std::chrono::system_clock::now(); // The end time of the neighborhood calculation
    std::chrono::duration<double> elapsed_seconds = end - start; // The elapsed time in seconds
    printf("Neighborhood calculation time = %f secs\n", elapsed_seconds.count());

    // Apply Graph-cut RANSAC
    utils::DefaultPnPEstimator estimator; // The estimator used for the pose fitting
    Pose6D model; // The estimated model parameters

    // Initialize the samplers
    // The main sampler is used inside the local optimization
    sampler::ProsacSampler main_sampler(&points, // The data points
        estimator.sampleSize()); // The size of a minimal sample
    sampler::UniformSampler local_optimization_sampler(&points); // The local optimization sampler is used inside the local optimization

    // Checking if the samplers are initialized successfully.
    if (!main_sampler.isInitialized() ||
        !local_optimization_sampler.isInitialized())
    {
        fprintf(stderr, "One of the samplers is not initialized successfully.\n");
        std::vector<size_t> h;
        return h;
    }

    // Initializing SPRT test
    preemption::SPRTPreemptiveVerfication<utils::DefaultPnPEstimator> preemptive_verification(
        points,
        estimator,
        minimum_inlier_ratio_for_sprt_);
    
    // Initializing the fast inlier selector object
    inlier_selector::EmptyInlierSelector<utils::DefaultPnPEstimator,
        neighborhood::FlannNeighborhoodGraph> inlier_selector(&neighborhood);

    GCRANSAC<utils::DefaultPnPEstimator,
        neighborhood::FlannNeighborhoodGraph,
        MSACScoringFunction<utils::DefaultPnPEstimator>,
        preemption::SPRTPreemptiveVerfication<utils::DefaultPnPEstimator>> gcransac;
    gcransac.settings.threshold = normalized_threshold; // The inlier-outlier threshold
    gcransac.settings.spatial_coherence_weight = spatial_coherence_weight_; // The weight of the spatial coherence term
    gcransac.settings.confidence = confidence_; // The required confidence in the results
    gcransac.settings.max_iteration_number = 5000; // The maximum number of iterations
    gcransac.settings.min_iteration_number = 20; // The minimum number of iterations
    gcransac.settings.neighborhood_sphere_radius = sphere_radius_; // The radius of the neighborhood ball

    // Start GC-RANSAC
    gcransac.run(normalized_points, // The normalized points
        estimator,  // The estimator
        &main_sampler, // The sample used for selecting minimal samples in the main iteration
        &local_optimization_sampler, // The sampler used for selecting a minimal sample when doing the local optimization
        &neighborhood, // The neighborhood-graph
        model, // The obtained model parameters
        preemptive_verification,
        inlier_selector);

    // Get the statistics of the results
    const utils::RANSACStatistics& statistics = gcransac.getRansacStatistics();

    printf("Elapsed time = %f secs\n", statistics.processing_time);
    printf("Inlier number before = %d\n", static_cast<int>(statistics.inliers.size()));
    printf("Applied number of local optimizations = %d\n", static_cast<int>(statistics.local_optimization_number));
    printf("Applied number of graph-cuts = %d\n", static_cast<int>(statistics.graph_cut_number));
    printf("Number of iterations = %d\n\n", static_cast<int>(statistics.iteration_number));

    // Save the inliers to file
//    utils::savePointsToFile(points, // The loaded data points
//        inlier_image_points_path_.c_str(), // The path where the results should be saved
//        &statistics.inliers); // The set of inlier found

    // The estimated rotation matrix
//    Eigen::Matrix3d rotation =
      r=  model.descriptor.leftCols<3>();
    // The estimated translation
//    Eigen::Vector3d translation =
   t=     model.descriptor.rightCols<1>();
//
//    Eigen::Quaterniond q_old( 0.568527,-0.151869   , -0.802613   , -0.097713 );
//    Eigen::Matrix3d rotation_old =q_old.normalized().toRotationMatrix();
//    Eigen::Vector3d translation_old ;
//    translation_old<<4.703599,    -1.781153 ,   0.859616;
//
//    gt_translation=gt_rotation*(-rotation_old.transpose()*translation_old)+gt_translation;
//    gt_rotation=gt_rotation*rotation_old.transpose();
//
//    translation=rotation*(-rotation_old.transpose()*translation_old)+translation;
//    rotation=rotation*rotation_old.transpose();
//
//
//
//
//
//    Eigen::Quaterniond q_old0(    -0.41657450795, 0.57345306873, 0.57021617889, 0.41529604793 );
//    Eigen::Matrix3d rotation_old0 =q_old0.normalized().toRotationMatrix();
//    Eigen::Vector3d translation_old0 ;
//    translation_old0<<0.01580299065, 0.00637514656, 0.06530746818 ;
//
//    Eigen::Quaterniond q_old20(  -0.31390127540,   0.65969997644 ,0.62579083443 ,0.27321726084 );
//    Eigen::Matrix3d rotation_old20 =q_old20.normalized().toRotationMatrix();
//    Eigen::Vector3d translation_old20 ;
//    translation_old20<<0.07157045603 ,-0.02330756560, 0.14762423933;
//
//    Eigen::Vector3d  translation_our=rotation_old20.transpose()*(translation_old0-translation_old20);
//    Eigen::Matrix3d rotation_our=rotation_old20.transpose()*rotation_old0;

    // The number of inliers found
    const size_t inlier_number = statistics.inliers.size();
    return statistics.inliers;

//    // Calculate the estimation error
//    constexpr double radian_to_degree_multiplier = 180.0 / M_PI;
//    const double angular_error = radian_to_degree_multiplier * (Eigen::Quaterniond(
//        rotation).angularDistance(Eigen::Quaterniond(gt_rotation)));
//    const double translation_error = (gt_translation - translation).norm();
//
//    printf("The error in the rotation matrix = %f degrees\n", angular_error);
//    printf("The error in the translation = %f cm\n", translation_error / 10.0);
//
//    // Calculate the estimation error
//    constexpr double radian_to_degree_multiplier_our = 180.0 / M_PI;
//    const double angular_error_our = radian_to_degree_multiplier_our * (Eigen::Quaterniond(
//                                                                                   rotation_our).angularDistance(Eigen::Quaterniond(gt_rotation)));
//    const double translation_error_our = (gt_translation - translation_our).norm();
//
//    printf("The error in the rotation matrix = %f degrees\n", angular_error_our);
//    printf("The error in the translation = %f cm\n", translation_error_our / 10.0);

    // If numerical optimization is needed, apply the Levenberg-Marquardt
    // implementation of OpenCV.
//    if (numerical_optimization_ && inlier_number >= 3)
//    {
//        // Copy the data into two matrices containing the image and object points.
//        // This would not be necessary, but selecting the submatrices by cv::Rect
//        // leads to an error in cv::solvePnP().
//        cv::Mat inlier_image_points(inlier_number, 2, CV_64F),
//            inlier_object_points(inlier_number, 3, CV_64F);
//
//        for (size_t i = 0; i < inlier_number; ++i)
//        {
//            const size_t& idx = statistics.inliers[i];
//            inlier_image_points.at<double>(i, 0) = normalized_points.at<double>(idx, 0);
//            inlier_image_points.at<double>(i, 1) = normalized_points.at<double>(idx, 1);
//            inlier_object_points.at<double>(i, 0) = points.at<double>(idx, 2);
//            inlier_object_points.at<double>(i, 1) = points.at<double>(idx, 3);
//            inlier_object_points.at<double>(i, 2) = points.at<double>(idx, 4);
//        }
//
//        // Converting the estimated pose parameters OpenCV format
//        cv::Mat cv_rotation(3, 3, CV_64F, rotation.data()), // The estimated rotation matrix converted to OpenCV format
//            cv_translation(3, 1, CV_64F, translation.data()); // The estimated translation converted to OpenCV format
//
//        // Convert the rotation matrix by the rodrigues formula
//        cv::Mat cv_rodrigues(3, 1, CV_64F);
//        cv::Rodrigues(cv_rotation.t(), cv_rodrigues);
//
//        // Applying numerical optimization to the estimated pose parameters
//        cv::solvePnP(inlier_object_points, // The object points
//            inlier_image_points, // The image points
//            cv::Mat::eye(3, 3, CV_64F), // The camera's intrinsic parameters
//            cv::Mat(), // An empty vector since the radial distortion is not known
//            cv_rodrigues, // The initial rotation
//            cv_translation, // The initial translation
//            true, // Use the initial values
//            cv::SOLVEPNP_ITERATIVE); // Apply numerical refinement
//
//        // Convert the rotation vector back to a rotation matrix
//        cv::Rodrigues(cv_rodrigues, cv_rotation);
//
//        // Transpose the rotation matrix back
//        cv_rotation = cv_rotation.t();
//
//        // Calculate the error of the refined pose
//        const double angular_error_refined = radian_to_degree_multiplier * (Eigen::Quaterniond(
//            rotation).angularDistance(Eigen::Quaterniond(gt_rotation)));
//        const double translation_error_refined = (gt_translation - translation).norm();
//
//        printf("The error in the rotation matrix after numerical refinement = %f degrees\n", angular_error_refined);
//        printf("The error in the translation after numerical refinement = %f cm\n", translation_error_refined / 10.0);
//    }
}


#endif /* gc_ransac_h */
