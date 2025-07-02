//
//  CameraFactory.h
//  VINS_ios
//
//  Created by 张剑华 on 2020/8/23.
//  Copyright © 2020 栗大人. All rights reserved.
//

#ifndef CameraFactory_h
#define CameraFactory_h


#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "Camera.h"

namespace camodocal
{

class CameraFactory
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraFactory();

    static boost::shared_ptr<CameraFactory> instance(void);

    CameraPtr generateCamera(Camera::ModelType modelType,
                             const std::string& cameraName,
                             cv::Size imageSize) const;

    CameraPtr generateCameraFromYamlFile();
    

private:
    static boost::shared_ptr<CameraFactory> m_instance;
};

}

#endif /* CameraFactory_h */
