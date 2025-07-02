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

    CameraPtr generateCameraFromYamlFile(int m_imageWidth,int m_imageHeight,double k1_global,double k2_global,double p1_global,double p2_global,double FOCUS_LENGTH_X_server,double FOCUS_LENGTH_Y_server,double PX_server,double PY_server);
    

private:
    static boost::shared_ptr<CameraFactory> m_instance;
};

}

#endif /* CameraFactory_h */
