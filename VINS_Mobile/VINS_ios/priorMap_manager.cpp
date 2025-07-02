//
//  priorMap_manager.cpp
//  VINS_ios
//
//  Created by 张剑华 on 2022/10/5.
//  Copyright © 2022 栗大人. All rights reserved.
//

#include "priorMap_manager.hpp"

int FeaturePerId_localMap::endFrame()
{
    return (int)(start_frame + feature_per_frame.size() - 1);
}
Feature_localMap::Feature_localMap(Eigen::Matrix3d _Rs[])
: Rs(_Rs)
{
    ric = Utility::ypr2R(Eigen::Vector3d(RIC_y,RIC_p,RIC_r));
}

void Feature_localMap::addFeature(int frame_count, std::vector<cv::Point2f> measurements_cur_coarse_new, std::vector<Eigen::Vector3d> point_3d_old_new , std::vector<int> feature_id_cur_new)
{
    for (int i=0, j=point_3d_old_new.size();i<j;i++)//遍历这一帧里面所有的线段
    {
        cv::Point2f m_2d=measurements_cur_coarse_new[i];
        Eigen::Vector3d pt3d=point_3d_old_new[i];
        int f_id=feature_id_cur_new[i];
        
        FeaturePerFrame_localMap priorMap_f_per_fra(Eigen::Vector3d(m_2d.x,m_2d.y,1.0));//要转为相机平面坐标 也就是去完畸变。 转了的
        
//        auto it = find_if(feature.begin(), feature.end(), [f_id](const FeaturePerId_localMap &it)
//                          {
//                              return it.feature_id == f_id;
//                          });
//        if (it == feature.end())
//        {
            //没有找到该feature的id，则把特征点放入feature的list容器中
            feature.push_back(FeaturePerId_localMap(f_id,frame_count, pt3d));//这里要改一下frame_count 应该是哪一帧观测到它
            feature.back().feature_per_frame.push_back(priorMap_f_per_fra);
//        }
//        //find match with previous feature
//        else if (it->feature_id == f_id)
//        {
//            it->feature_per_frame.push_back(priorMap_f_per_fra);
//        }
        
    }
    
    
}

void Feature_localMap::clearState()
{
    feature.clear();
}

void Feature_localMap::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        
        if (it->start_frame != 0)
        {
            it->start_frame--;
        }
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
            {
                feature.erase(it);
                //printf("remove back\n");
            }
        }
    }
}

void Feature_localMap::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;
        
        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            //it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
            {
                feature.erase(it);
                //printf("remove front\n");
            }
            
        }
        //if(it->is_margin == true)
        //    feature.erase(it);
    }
}

