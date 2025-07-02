//
//  MetalView.cpp
//  VINS_ios
//
//  Created by 张剑华 on 2023/5/9.
//  Copyright © 2023 栗大人. All rights reserved.
//

#import "MetalView.h"
#include <iostream>
@implementation MetalView

+ (Class)layerClass
{
    return [CAMetalLayer class];
}

- (instancetype)initWithCoder:(NSCoder *)aDecoder
{
    if ((self = [super initWithCoder:aDecoder]))
    {
        _metalLayer = (CAMetalLayer *)[self layer];
        _metalLayer.backgroundColor = [UIColor clearColor].CGColor;
        CGFloat scale = [UIScreen mainScreen].scale;
        _metalLayer.drawableSize = CGSizeMake(self.bounds.size.width * scale,
                                              self.bounds.size.height * scale);
        
        std::cout<<"画图的大小"<<self.bounds.size.width<<" , "<<scale<<std::endl;
    }
    
    return self;
}

@end
