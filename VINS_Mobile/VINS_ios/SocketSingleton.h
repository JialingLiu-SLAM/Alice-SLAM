//
//  SocketSingleton.hpp
//  VINS_ios
//
//  Created by 张剑华 on 2020/5/25.
//  Copyright © 2020 zx. All rights reserved.
//

#include "keyfame_database.h"
//#import <Foundation/Foundation.h>
#include "feature_tracker.hpp"

#include "VINS.hpp"
using namespace std;
#include <opencv2/core/core.hpp>

#import "AsyncSocket.h"

#define DEFINE_SHARED_INSTANCE_USING_BLOCK(block) \
static dispatch_once_t onceToken = 0; \
__strong static id sharedInstance = nil; \
dispatch_once(&onceToken, ^{ \
sharedInstance = block(); \
}); \
return sharedInstance; \


enum{
    SocketOfflineByServer,
    SocketOfflineByUser,
};

@interface SocketSingleton : NSObject<AsyncSocketDelegate>
{
    std::mutex mMutexConn;
    bool isConn;
    cv::Mat Tcw;
    
    NSThread *parseData_server;
    queue<NSMutableData *> all_msgData;
    std::mutex parseQueue_mutex;
}

@property (nonatomic, strong) AsyncSocket    *socket;       // socket
@property (nonatomic, copy  ) NSString       *socketHost;   // socket的Host
@property (nonatomic, assign) UInt16         socketPort;    // socket的prot

@property (nonatomic, retain) NSTimer        *connectTimer; // 计时器
@property (nonatomic, assign) UInt16         packIdx;
@property (nonatomic, retain) NSMutableData  *responseData;


@property (nonatomic, assign) KeyFrameDatabase *kfdb;
@property (nonatomic, assign) VINS *vins;

@property (nonatomic, assign) FeatureTracker *featureTracker;
@property (nonatomic, assign) KeyFrame *old_kf_priorMap;
@property (nonatomic, assign) KeyFrame *curLoopKf_priorMap;


//-(void)setKfdb_s:(KeyFrameDatabase *)kfdb_s;
//-(void)setVins_s:(VINS *)vins_s;

+ (SocketSingleton *)sharedInstance;


-(bool)socketConnectHost;// socket连接

-(void)cutOffSocket;// 断开socket连接

- (NSData*)hasCompleteData;
- (void)parseData:(NSData *)data;
-(void)parseData_server;

@end
