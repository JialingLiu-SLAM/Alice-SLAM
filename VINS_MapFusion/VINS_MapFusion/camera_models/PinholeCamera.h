//
//  PinholeCamera.h
//  VINS_ios
//
//  Created by 张剑华 on 2020/8/23.
//  Copyright © 2020 栗大人. All rights reserved.
//
#include "feature_tracker.hpp"
#ifndef PinholeCamera_h
#define PinholeCamera_h



#include <opencv2/core/core.hpp>
#include <string>

#include "ceres/rotation.h"
#include "Camera.h"


namespace camodocal
{

class PinholeCamera: public Camera
{
public:
    class Parameters: public Camera::Parameters
    {
    public:
        Parameters();
        Parameters(const std::string& cameraName,
                   int w, int h,
                   double k1, double k2, double p1, double p2,
                   double fx, double fy, double cx, double cy);

        double& k1(void);
        double& k2(void);
        double& p1(void);
        double& p2(void);
        double& fx(void);
        double& fy(void);
        double& cx(void);
        double& cy(void);

//        double xi(void) const;
        double k1(void) const;
        double k2(void) const;
        double p1(void) const;
        double p2(void) const;
        double fx(void) const;
        double fy(void) const;
        double cx(void) const;
        double cy(void) const;

        void readFromYamlFile(int m_imageWidth,int m_imageHeight,double k1_global,double k2_global,double p1_global,double p2_global,double FOCUS_LENGTH_X_server,double FOCUS_LENGTH_Y_server,double PX_server,double PY_server);
        

        Parameters& operator=(const Parameters& other);
        friend std::ostream& operator<< (std::ostream& out, const Parameters& params);

    private:
        double m_k1;
        double m_k2;
        double m_p1;
        double m_p2;
        double m_fx;
        double m_fy;
        double m_cx;
        double m_cy;
    };

    PinholeCamera();

    /**
    * \brief Constructor from the projection model parameters
    */
    PinholeCamera(const std::string& cameraName,
                  int imageWidth, int imageHeight,
                  double k1, double k2, double p1, double p2,
                  double fx, double fy, double cx, double cy);
    /**
    * \brief Constructor from the projection model parameters
    */
    PinholeCamera(const Parameters& params);

//    Camera::ModelType modelType(void) const;
//    const std::string& cameraName(void) const;
   
    int imageWidth(void) const;
    int imageHeight(void) const;
     

    // Lift points from the image plane to the sphere
    virtual void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P


    void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const;
    //%output p

   

    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;
    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u,
                    Eigen::Matrix2d& J) const;

//    void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0) const;
//    cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
//                                    float fx = -1.0f, float fy = -1.0f,
//                                    cv::Size imageSize = cv::Size(0, 0),
//                                    float cx = -1.0f, float cy = -1.0f,
//                                    cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;

   

    const Parameters& getParameters(void) const;
    void setParameters(const Parameters& parameters);

  

private:
    Parameters mParameters;

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
    bool m_noDistortion;
};

typedef boost::shared_ptr<PinholeCamera> PinholeCameraPtr;
typedef boost::shared_ptr<const PinholeCamera> PinholeCameraConstPtr;


}


#endif /* PinholeCamera_h */
