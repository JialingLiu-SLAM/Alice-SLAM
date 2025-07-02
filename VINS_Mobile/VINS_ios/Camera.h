//
//  Camera.h
//  VINS_ios
//
//  Created by 张剑华 on 2020/8/23.
//  Copyright © 2020 栗大人. All rights reserved.
//

#ifndef Camera_h
#define Camera_h


#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <vector>

namespace camodocal
{

class Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum ModelType
    {
        KANNALA_BRANDT,
        MEI,
        PINHOLE,
        SCARAMUZZA
    };

    class Parameters
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Parameters(ModelType modelType);

        Parameters(ModelType modelType, const std::string& cameraName,
                   int w, int h);

        int& imageWidth(void);
        int& imageHeight(void);
//
        int imageWidth(void) const;
        int imageHeight(void) const;
        virtual void readFromYamlFile(){}
    protected:
        ModelType m_modelType;
        int m_nIntrinsics;
        std::string m_cameraName;
        int m_imageWidth;
        int m_imageHeight;
    };

//    virtual ModelType modelType(void) const = 0;
//    virtual const std::string& cameraName(void) const = 0;
    virtual int imageWidth(void) const = 0;
    virtual int imageHeight(void) const = 0;


    // Lift points from the image plane to the sphere
    virtual void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const = 0;
    //%output P

    // Lift points from the image plane to the projective space
    virtual void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const = 0;
    //%output P


    virtual void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const = 0;

protected:
    cv::Mat m_mask;
};

typedef boost::shared_ptr<Camera> CameraPtr;
typedef boost::shared_ptr<const Camera> CameraConstPtr;

}

#endif /* Camera_h */
