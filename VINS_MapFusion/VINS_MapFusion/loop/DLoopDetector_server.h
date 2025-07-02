//
//  DLoopDetector_server.h
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/6/27.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef DLoopDetector_server_h
#define DLoopDetector_server_h


/// Loop detector for sequences of monocular images
namespace DLoopDetector_server
{
}

//#include "DBoW2.h"
#include "TemplatedLoopDetector_server.h"
#include "FBrief.h"

/// BRIEF Loop Detector
typedef DLoopDetector_server::TemplatedLoopDetector_server
  <FBrief::TDescriptor, FBrief> BriefLoopDetector_server;

#endif /* DLoopDetector_server_h */
