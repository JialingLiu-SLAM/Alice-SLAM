//
//  CameraFactory.m
//  VINS_ios
//
//  Created by 张剑华 on 2020/8/23.
//  Copyright © 2020 栗大人. All rights reserved.
//
#include "PinholeCamera.h"
//#import <Foundation/Foundation.h>

#include "CameraFactory.h"

#include <boost/algorithm/string.hpp>





#include "ceres/ceres.h"

namespace camodocal
{

boost::shared_ptr<CameraFactory> CameraFactory::m_instance;

CameraFactory::CameraFactory()
{

}

boost::shared_ptr<CameraFactory>
CameraFactory::instance(void)
{
    if (m_instance.get() == 0)
    {
        m_instance.reset(new CameraFactory);
    }

    return m_instance;
}

CameraPtr
CameraFactory::generateCamera(Camera::ModelType modelType,
                              const std::string& cameraName,
                              cv::Size imageSize) const
{
   
//    case Camera::PINHOLE:
//    {
        PinholeCameraPtr camera(new PinholeCamera);

        PinholeCamera::Parameters params = camera->getParameters();
//        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
//    }
    
}

CameraPtr
CameraFactory::generateCameraFromYamlFile()
{
    
//    case Camera::PINHOLE:
    
    PinholeCameraPtr camera(new PinholeCamera);

    PinholeCamera::Parameters params = camera->getParameters();
    params.readFromYamlFile();
    camera->setParameters(params);
    return camera;
}
    

}

