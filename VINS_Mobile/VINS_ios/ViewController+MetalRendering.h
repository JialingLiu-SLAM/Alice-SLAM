//
//  ViewController+MetalRendering.hpp
//  VINS_ios
//
//  Created by 张剑华 on 2023/5/8.
//  Copyright © 2023 栗大人. All rights reserved.
//

#import "Renderer.h"
#import "ViewController.h"

@interface ViewController (MetalRendering)


- (void) initRendering;
- (void) loadModel;
- (void) drawObjectWith:(const cv::Mat&)R andT:(const cv::Mat&)T andisTracking:(bool)isTracking;
- (void) drawObjectWith2:(const cv::Mat&)R andT:(const cv::Mat&)T;
- (void) drawObjectWith3:(const cv::Mat&)R andT:(const cv::Mat&)T;
@end
