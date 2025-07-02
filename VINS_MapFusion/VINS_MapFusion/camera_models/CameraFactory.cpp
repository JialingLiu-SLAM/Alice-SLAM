//
//  CameraFactory.m
//  VINS_ios
//
//  Created by 张剑华 on 2020/8/23.
//  Copyright © 2020 栗大人. All rights reserved.
//


#include "CameraFactory.h"

#include <boost/algorithm/string.hpp>



#include "PinholeCamera.h"

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
CameraFactory::generateCameraFromYamlFile(int m_imageWidth,int m_imageHeight,double k1_global,double k2_global,double p1_global,double p2_global,double FOCUS_LENGTH_X_server,double FOCUS_LENGTH_Y_server,double PX_server,double PY_server)
{
    
//    case Camera::PINHOLE:
    
    PinholeCameraPtr camera(new PinholeCamera);

    PinholeCamera::Parameters params = camera->getParameters();
    params.readFromYamlFile( m_imageWidth, m_imageHeight, k1_global, k2_global, p1_global, p2_global, FOCUS_LENGTH_X_server, FOCUS_LENGTH_Y_server, PX_server, PY_server);
    camera->setParameters(params);
    return camera;
}
    

}

