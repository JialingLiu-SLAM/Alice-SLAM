//
//  SocketSingleton.cpp
//  VINS_ios
//
//  Created by 张剑华 on 2020/5/25.
//  Copyright © 2020 zx. All rights reserved.
//

#include "SocketSingleton.h"


#import <sys/socket.h>

#import <netinet/in.h>

#import <arpa/inet.h>

#import <unistd.h>

#include "HandleData.h"

#import <UIKit/UIKit.h>

@implementation SocketSingleton
//要保持长连接,需要建一个全局的单例
+(SocketSingleton *) sharedInstance
{
    
    static SocketSingleton *sharedInstace = nil;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        
        sharedInstace = [[self alloc] init];
        sharedInstace->isConn=false;
        
        sharedInstace->_responseData=[[NSMutableData alloc] init];
        sharedInstace->parseData_server=[[NSThread alloc]initWithTarget:sharedInstace selector:@selector(parseData_server) object:nil];
        [sharedInstace->parseData_server start];
        
    });
    
    return sharedInstace;
}

-(void)parseData_server{
    while (![[NSThread currentThread] isCancelled]){
        parseQueue_mutex.lock();
        if(!all_msgData.empty()){
//            cout<<"all_msgData qian"<<all_msgData.size()<<endl;
            NSData *data=all_msgData.front();
            all_msgData.pop();
//            cout<<"all_msgData hou"<<all_msgData.size()<<endl;
            parseQueue_mutex.unlock();
            
            [self parseData:data];
            
            [NSThread sleepForTimeInterval:0.01];
        }else{
            parseQueue_mutex.unlock();
            [NSThread sleepForTimeInterval:0.03];
        }
            
    }
}

// socket连接
-(bool)socketConnectHost{
    self.socket    = [[AsyncSocket alloc] initWithDelegate:self];
    
    NSError *error = nil;
    
    if([self.socket connectToHost:self.socketHost onPort:self.socketPort withTimeout:3 error:&error]){
        isConn=true;
        
//        NSLog(@"socket连接成功");
    }else{
        isConn=false;
        NSLog(@"socket连接失败");
    }
    
    return isConn;

}
// 连接成功回调
#pragma mark  - 连接成功回调
-(void)onSocket:(AsyncSocket *)sock didConnectToHost:(NSString *)host port:(UInt16)port
{
    NSLog(@"socket连接成功");
    _packIdx = 50;
    
    //需要每隔特定时间想服务器发送心跳包
//    self.connectTimer = [NSTimer scheduledTimerWithTimeInterval:30 target:self selector:@selector(longConnectToSocket) userInfo:nil repeats:YES];
//    [self.connectTimer fire];//启动定时器
    [self.socket readDataWithTimeout:-1 tag:0];
    
}


const int robot_id=0;
// 心跳连接 发送消息
-(void)longConnectToSocket{
//    cout<<"longConnectToSocket  ing"<<endl;
    
    // 根据服务器要求发送固定格式的数据，假设为指令@"longConnect"，但是一般不会是这么简单的指令
    if(isConn){
        
        struct PackHead packhead;
        sprintf(packhead.PackName, "%s", "longConnect");
        packhead.PackIdx = robot_id;
        packhead.PackLen = 0;
        
        char* bytearr=new char[sizeof(struct PackHead)];
        memcpy(bytearr,reinterpret_cast<char*>(&packhead), sizeof(struct PackHead));
        
        NSData *data = [NSData dataWithBytes:bytearr length:strlen(bytearr)];
//        NSString *longConnect = @"longConnect";
//
//        NSData   *dataStream  = [longConnect dataUsingEncoding:NSUTF8StringEncoding];
        //withTimeout -1 :无穷大
        //tag： 消息标记
        [self.socket writeData:data withTimeout:1 tag:1];
    }else{
        NSLog(@"socket not connected!");
        if([UIApplication sharedApplication].applicationState==UIApplicationStateActive){
            [self socketConnectHost];
        }
    }
    
    
}
// 切断socket
-(void)cutOffSocket{
    isConn=false;
    
    self.socket.userData = SocketOfflineByUser;
    
    [self.connectTimer invalidate];
    
    [self.socket disconnect];
}
-(void)onSocketDidDisconnect:(AsyncSocket *)sock
{
    NSLog(@"sorry the connect is failure %ld",sock.userData);
    isConn=false;
    if (sock.userData == SocketOfflineByServer) {
        // 服务器掉线，重连
//        [self socketConnectHost];
        NSLog(@"lianjie fail by server");
    }
    else if (sock.userData == SocketOfflineByUser) {
        // 如果由用户断开，不进行重连
        return;
    }
    
}

- (NSUInteger)headLength:(NSData*)data
{
    NSData *lengthData = [data subdataWithRange:NSMakeRange(0, 4)];
    
    NSUInteger headLength;
    [lengthData getBytes:&headLength length:sizeof(NSUInteger)];
//    NSLog(@"headLength:(NSData*)data headLength==%d",headLength);
    
//    headLength = CFSwapInt32HostToBig(headLength);
//    NSLog(@"headLength:(NSData*)data headLength CFSwapInt32HostToBig==%d",headLength);
    
    return headLength;
}

- (NSData*)hasCompleteData
{
    NSData *completeData = nil;
    if ([_responseData length] > 4) {
        NSUInteger lengthBytes = 4;//存放正文长度
        NSUInteger headLength = [self headLength:_responseData];//正文长度
//        cout<<"headLength="<<headLength<<endl;
        NSUInteger leftLength = [_responseData length] - lengthBytes;
        if (leftLength >= headLength) {//有完整的包了
//            cout<<"head+data Length len="<<headLength+4<<endl;
            completeData = [_responseData subdataWithRange:NSMakeRange(lengthBytes, headLength)];//从哪开始截取 截取多长
            NSData *leftData = [_responseData subdataWithRange:NSMakeRange(headLength + lengthBytes, leftLength - headLength)];
            _responseData = [[NSMutableData alloc] initWithData:leftData];
//            cout<<"_responseData len="<<[_responseData length]<<endl;
        }
        
       
        
        
    }
    return completeData;
}

//Terminating app due to uncaught exception 'NSRangeException', reason: '*** -[NSConcreteMutableData subdataWithRange:]: range {4, 9503697766216380391} exceeds data length 5792'
//*** First throw call stack:
//(0x19efd180c 0x19ecf9fa4 0x19f2f2e04 0x1027c275c 0x1027c2dbc 0x1027c9c0c 0x1027c9a18 0x19f3a6688 0x19ef4fe00 0x19ef4fb3c 0x19ef4f20c 0x19ef4a348 0x19ef498a0 0x1a8ea1328 0x1a303a740 0x1027bafec 0x19edd4360)

- (void)parseData:(NSData *)data{
//    cout<<"parseData ing"<<endl;
    Byte *recvData = (Byte *)[data bytes];
    // 对得到的data值进行解析与转换即可

    long unsigned int offset=0;
        

        
    const char *buffer_all=(char*)recvData;
    
    int name_len;
    memcpy(&name_len, buffer_all, sizeof(int));
    offset+=sizeof(int);
    
    string name="";
//    char name_single;
    for(int i=0;i<name_len;i++){
        name+=buffer_all[offset];
        offset+=sizeof(char);
    }
    
//    std::cout<<"name:  "<<name<<"&&"<<std::endl;
    if (!(std::strcmp(name.c_str(), "Global")==0 || strcmp(name.c_str(),"loop")==0 || strcmp(name.c_str(),"loop_ano")==0 || strcmp(name.c_str(),"reject")==0 || strcmp(name.c_str(),"Fusion")==0 || strcmp(name.c_str(),"ar")==0 || strcmp(name.c_str(),"loop_an2")==0 ) )
    {
        return;
    }
    
    
    if (strcmp(name.c_str(), "Global")==0){
        NSLog(@"This message is returned by a global!");
        receiveGlobalData(buffer_all,offset,self.kfdb);
        self.kfdb->start_global_optimization=true;
//        cout<<"global oldIndex="<<self.kfdb->loopKF_index.back()<<" curIndex= "<<self.kfdb->curKF_loop_index.back()<<endl;
    }else if(strcmp(name.c_str(), "reject")==0){
        NSLog(@"This message is returned by a reject!");
        receiveRejectWithF(buffer_all,offset,self.vins);
        //执行拿到的keyFrame的外点拒绝
        int cur_reject_index=self.vins->send_status_index.front();
        self.vins->send_status_index.pop();
        vector<uchar> status=self.vins->send_status.front();
        self.vins->send_status.pop();
        
        KeyFrame* cur_kf = self.kfdb->getKeyframe(cur_reject_index);
        cur_kf->rejectWithF_server(status);
    }
    else if(strcmp(name.c_str(), "loop")==0){
//        NSLog(@"This message is returned by a loop!");
        receiveLoopData(buffer_all,offset,self.vins);
//        NSLog(@"receive a loop end");
        KeyFrame* cur_kf = self.kfdb->getKeyframe(self.vins->retrive_pose_data_server.back().cur_index);//这个没拿到值
        self.vins->retrive_pose_data_server.back().header=cur_kf->header;
 
        int old_index=self.vins->retrive_pose_data_server.back().old_index;
        KeyFrame* old_kf = self.kfdb->getKeyframe(old_index);
        Eigen::Vector3d T_w_i_old;
        Eigen::Matrix3d R_w_i_old;
        old_kf->getPose(T_w_i_old, R_w_i_old);
        self.vins->retrive_pose_data_server.back().P_old=T_w_i_old;
        self.vins->retrive_pose_data_server.back().Q_old=R_w_i_old;
        
        cur_kf->detectLoop(old_index);
//        cout<<fixed<<setprecision(1)<<"cur_kf hasloop:"<<cur_kf->global_index<<" "<<cur_kf->header<<" "<<self.vins->Headers[0]<<" "<<self.vins->retrive_pose_data.cur_index<<endl;
        
        self.kfdb->addLoop(old_index);
        
        old_kf->is_looped=1;
       
    }
    else if(strcmp(name.c_str(), "loop_ano")==0){
        receiveLoopData_another(buffer_all,offset,self.vins);
//        NSLog(@"receive a loop end");
        KeyFrame* cur_kf = self.kfdb->getKeyframe(self.vins->retrive_pose_data_server.back().cur_index);//这个没拿到值
        self.vins->retrive_pose_data_server.back().header=cur_kf->header;
 
        int old_index=self.vins->retrive_pose_data_server.back().old_index;
        KeyFrame* old_kf = self.kfdb->getKeyframe(old_index);
        Eigen::Vector3d T_w_i_old;
        Eigen::Matrix3d R_w_i_old;
//        新帧3d 原来的
        old_kf->getPose(T_w_i_old, R_w_i_old);
        self.vins->retrive_pose_data_server.back().P_old=T_w_i_old;
        self.vins->retrive_pose_data_server.back().Q_old=R_w_i_old;
        
        cur_kf->detectLoop(old_index);
        cout<<"cur_kf hasloop:"<<cur_kf->global_index<<" "<<old_index<<endl;
        
        self.kfdb->addLoop(old_index);
        
        old_kf->is_looped=1;
        
        
    }
    else if(strcmp(name.c_str(), "loop_an2")==0){
        double loop_pose_forFeatureTracker[7];
        receiveLoopData_another2(buffer_all,offset,self.vins,loop_pose_forFeatureTracker);
//        NSLog(@"receive a loop end");
        KeyFrame* cur_kf = self.kfdb->getKeyframe(self.vins->retrive_pose_data_server.back().cur_index);//这个没拿到值
        self.vins->retrive_pose_data_server.back().header=cur_kf->header;
 
        int old_index=self.vins->retrive_pose_data_server.back().old_index;
        KeyFrame* old_kf = self.kfdb->getKeyframe(old_index);
        Eigen::Vector3d T_w_i_old;
        Eigen::Matrix3d R_w_i_old;

//        老帧3d的
//        old_kf->getOriginPose(T_w_i_old, R_w_i_old);
//        self.vins->retrive_pose_data_server.back().P_old=T_w_i_old;
//        self.vins->retrive_pose_data_server.back().Q_old=R_w_i_old;
        
        cur_kf->detectLoop(old_index);
//        cout<<"cur_kf hasloop loop_an2:"<<cur_kf->global_index<<" "<<old_index<<endl;
        
        self.kfdb->addLoop(old_index);
        
        old_kf->is_looped=1;
        
        self.featureTracker->priorMap_mutex.lock();
        
//        self.old_kf_priorMap=old_kf;
//        self.curLoopKf_priorMap=cur_kf;
        self.featureTracker->old_kf_priorMap.push(old_kf);
        self.featureTracker->curLoopKf_priorMap.push(cur_kf);
        self.featureTracker->priorMapFeature->isUpdate=true;
        self.featureTracker->setLoopKf(loop_pose_forFeatureTracker);
        self.featureTracker->priorMap_mutex.unlock();
    }
    else if(strcmp(name.c_str(), "Fusion")==0){
        NSLog(@"This message is returned by a Fusion!");
        receiveGlobalData_multiClient(buffer_all,offset,self.kfdb);
        self.kfdb->start_global_optimization_multiClient=true;
//        cout<<"global Fusion oldIndex="<<self.kfdb->loopKF_index_multiClient.back()<<" curIndex= "<<self.kfdb->curKF_loop_index_multiClient.back()<<endl;
    }
    else if(strcmp(name.c_str(), "ar")==0){
        //接收到服务器端发送过来ar的位置 这里不需要设置标志变量，是因为显示的时候 会遍历所有的
        NSLog(@"This message is returned by a AR!");
        receiveAr(buffer_all,offset,self.vins);
    }
    else {
        NSLog(@"This message is returned by UFO!😱");
    }
}


//收到消息
//-(void)onSocket:(AsyncSocket *)sock didReadData:(NSData *)data withTag:(long)tag
//{
//    [_responseData appendData:data];
//    @synchronized(_responseData) {
//        NSData *completeData = nil;
//        completeData = [self hasCompleteData];
//        if (completeData!=nil) {   //一直从缓冲区里读获取完整的数据包，进行解析
//            [self parseData:completeData];
////            completeData = [self hasCompleteData];
//        }
//        [sock readDataWithTimeout:-1 tag:0];
//    }
//}



-(void)onSocket:(AsyncSocket *)sock didReadData:(NSData *)data withTag:(long)tag
{
    //将数据存入缓存区
   [_responseData appendData:data];
   
   //数据中前面有4个字节的头信息，其中前两位是固定的头长度(用处不大),后两位才是数据的长度。
   //如果大于4个字节证明有消息，因为服务器只要发送数据，必定包含头
   while (_responseData.length > 4) {
       
       //将消息转化成byte，计算总长度 = 数据的内容长度 + 前面4个字节的头长度
       NSData *lengthData = [_responseData subdataWithRange:NSMakeRange(0, 4)];
       
       int allLength;
       [lengthData getBytes:&allLength length:sizeof(allLength)];
       
       //缓存区的长度大于总长度，证明有完整的数据包在缓存区，然后进行处理
       if (_responseData.length >= allLength) {
           printf("allLength=%d,%d\n",allLength,_responseData.length);
           NSMutableData *msgData = [[_responseData subdataWithRange:NSMakeRange(4, allLength)] mutableCopy];//偶尔错误
           
           parseQueue_mutex.lock();
           all_msgData.push(msgData);
           parseQueue_mutex.unlock();
           
           //处理完数据后将处理过的数据移出缓存区
           _responseData = [NSMutableData dataWithData:[_responseData subdataWithRange:NSMakeRange(allLength+4, _responseData.length - allLength-4)]];
//           cout<<"_responseData.length="<<_responseData.length<<endl;
       }
       else{
           //缓存区内数据包不是完整的，再次从服务器获取数据，中断while循环
           break;
       }
       
   }
    [sock readDataWithTimeout:-1 tag:0];//这个必须执行 否则就这个函数只会被调用一次
}

@end


