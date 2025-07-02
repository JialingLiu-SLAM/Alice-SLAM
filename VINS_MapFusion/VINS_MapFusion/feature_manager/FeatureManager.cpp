//
//  FeatureManager.cpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/27.
//  Copyright © 2020 zx. All rights reserved.
//

#include "FeatureManager.hpp"
FeatureManager::FeatureManager(Matrix3d _Rs[])
: Rs(_Rs)
{
    ric = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r));
}
