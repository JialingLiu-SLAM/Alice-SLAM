//
//  Server.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/25.
//  Copyright © 2020 zx. All rights reserved.
//
#include "Client.hpp"
#ifndef Server_hpp
#define Server_hpp

#include <stdio.h>


#include <iostream>
#include <vector>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <sys/socket.h>
#include <netinet/in.h>

#include "MyThread.hpp"

#include "HandleData.hpp"
#include "VINS.hpp"
#import "feature_tracker.hpp"
#include "PoseGraph.hpp"
#include "LoopClosure.hpp"
#include "DrawResult.hpp"

#include "PoseGraphGlobal.hpp"
#include <unordered_map>
#include <map>

#include <sys/stat.h>
#include <sys/types.h>


#define PORT 8080


using namespace std;

typedef struct {
    void* arg1;//
    void* arg2;//
} ClientArg;



class Server {

  private:
    //Socket stuff
    int serverSock, clientSock;
    struct sockaddr_in serverAddr, clientAddr;
    char buff[256];
    

  public:
    static std::vector<Client*> clients;//
    static std::vector<PoseGraph*> poseGraphs;//
    
    Server();
    void AcceptAndDispatch();
    
    static void *PoseGraphGlobalRun_inServer_2(void *args);
     static size_t ReceiveWithSize(int sock, char* data, size_t dataLen);//
     static void * HandleClient(void *args);//
    
    
   static  void dataSendRun(Client *c,PoseGraph *poseGraph,VINS* vins);//
   static  void sendStatus_rejectWithF(Client *c,VINS* vins);//
    static void sendGlobalLoop(Client *c,PoseGraph *poseGraph);//
   static  void sendLoop(Client *c,VINS* vins);//
    static void sendLoop_another(Client *c,VINS* vins);
    static void sendLoop_another2(Client *c,VINS* vins);
    static void sendAr(Client *c, int ground_ar_idx);
    static void sendGlobalLoop_multiClient(Client *c,PoseGraph *poseGraph);
    
  static   void handleKFDataRun(Client *c,PoseGraph *poseGraph,VINS* vins);//
    static   void * globalLoopRun(void *args);//

//    static bool startHandleKf=false;
    
    static DrawResult * drawResult;//
    
    static string getTime();//
    static unordered_map<string, bool> myHashMap;//
    
   static PoseGraphGlobal* poseGraphGlobal;//还没初始化
    static FeatureMap* global_featureMap;
    
  private:
    static void ListClients();//
    static void SendToAll(char *message);//
    static int FindClientIndex(Client *c);//

};


#endif /* Server_hpp */
