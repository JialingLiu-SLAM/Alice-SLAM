//
//  MetalView.hpp
//  VINS_ios
//
//  Created by 张剑华 on 2023/5/9.
//  Copyright © 2023 栗大人. All rights reserved.
//
#import <UIKit/UIKit.h>
#import <QuartzCore/CAMetalLayer.h>
#import <Metal/Metal.h>

@interface MetalView : UIView

@property(nonatomic, strong) CAMetalLayer *metalLayer;

@end
