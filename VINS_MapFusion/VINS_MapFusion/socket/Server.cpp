//
//  Server.cpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/25.
//  Copyright © 2020 zx. All rights reserved.
//

#include "Server.hpp"

#include <boost/thread.hpp>
#include <thread>
#include <chrono>
#include <opencv2/core/core.hpp>

#include "HandleData.hpp"

 
//Actually allocate clients
std::vector<Client*> Server::clients;
std::vector<PoseGraph*> Server::poseGraphs;
unordered_map<string, bool> Server::myHashMap;



//extern ORB_SLAM2::ORBVocabulary *_vocabulary;

Server::Server() {
    
    
    
    //Initialize static mutex from MyThread
    MyThread::InitMutex();
    
    //For setsock opt (REUSEADDR)
    int yes = 1;
    
    //Init serverSock and start listen()'ing
    serverSock = socket(AF_INET, SOCK_STREAM, 0);//套接字，面向网络的 tcp，可靠
    memset(&serverAddr, 0, sizeof(sockaddr_in));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(PORT);
    
    //server程序总是应该在调用bind()之前设置SO_REUSEADDR套接字选项。
    //Avoid bind error if the socket was not close()'d last time;
//默认情况下，两个独立的套接字不可与同一本地接口（在TCP/IP情况下，则是端口）绑定在一起。但是少数情况下，还是需要使用这种方式，来实现对一个地址的重复使用。设置了这个套接字，服务器便可在重新启动之后，在相同的本地接口以端口上进行监听。
    setsockopt(serverSock,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(int));
    //SO_REUSEADDR让端口释放后立即就可以被再次使用 用于对TCP套接字处于TIME_WAIT状态下的socket，允许重复绑定使用
    int nRecvBuf = 210*1024;
    setsockopt(serverSock,SOL_SOCKET,SO_RCVBUF,(const char*)&nRecvBuf,sizeof(int));
    
    if(::bind(serverSock, (struct sockaddr *) &serverAddr, sizeof(sockaddr_in)) < 0)
        std::cerr << "Failed to bind";
//    MapPoint::InitRand();
    
    listen(serverSock, 5);
//系统队列有5个空位可以放未决请求，客户机来的连接都是系统先收到然后应用层accept处理，如果队列满了就connect不上了
    
    
    
}

/*
    AcceptAndDispatch();
 
    Main loop:
     Blocks at accept(), until a new connection arrives.
     When it happens, create a new thread to handle the new client.
 */
void Server::AcceptAndDispatch() {
    
    Client *c;
    MyThread *t;
    
    MyThread *globalOpti;
    MyThread *globalLoop;
    
//    socklen_t cliSize = sizeof(sockaddr_in);
    socklen_t cliSize = sizeof(struct sockaddr);
    
    //服务端接收连接的逻辑不一定很快处理完这瞬间过来的每一个请求，没有被处理的就会放到系统队列里
    //上层accept调用实际上就是从系统队列取走一个未决的连接请求，一般这个系统队列是满不了的，
    //因为你的accept函数很快就从系统队列取走未决的连接请求
    while(1) {

        c = new Client();
        t = new MyThread();
        
        globalLoop = new MyThread();
        globalOpti= new MyThread();
        
        int nSendBuf = 200*1024;
        setsockopt(c->sock, SOL_SOCKET, SO_SNDBUF, (const char*)&nSendBuf, sizeof(int));

        //Blocks here;
        c->sock = accept(serverSock, (struct sockaddr *) &clientAddr, &cliSize);

        if(c->sock < 0) {
            std::cerr << "Error on accept";
        }
        else {
            std::cerr<< "one new client accepted"<<std::endl;
            t->Create((void *) Server::HandleClient, c);
            
             //开一个线程
            globalLoop->Create((void *) Server::PoseGraphGlobalRun_inServer_2, c);
                    
            globalOpti->Create((void *) Server::globalLoopRun, c);
        }
    }
}

size_t Server::ReceiveWithSize(int sock, char* data, size_t dataLen)
{
    if (data==NULL) {
        return -1;
    }
    char* p = data;
    size_t len=dataLen;
    size_t retLen = 0, ret = 0;
    
    while (len>0) {
        ret = recv(sock, p+(dataLen-len), dataLen-retLen, 0);
        if (ret<=0) {
            return ret;
        }
        len -= ret;
        retLen += ret;
    }
    return retLen;
}

string Server::getTime()
{
    time_t timep;
    time (&timep); //获取time_t类型的当前时间
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );//对日期和时间进行格式化
    return tmp;
}

bool oneEnd_server=true;

//Static 未完 思考，每有一个客户端，哪些类 会新产生一个对象
void *Server::HandleClient(void *args) {
//多个用户则需要考虑 buffer队列 不能做全局的
    
    
    cout<<"网络连接成功 HandleClient"<<endl;
    //Pointer to accept()'ed Client
    Client *c = (Client *) args;
    char name[20], packHead[30];
    char *buffer;
    int n, t,mapFusionIndex_real=1,mapFusionIndex=1,lastMapKeyFrameId=0;
    struct PackHead * packhead;
    vector<bool> curMapIsEnd;
    
    //定义VINS
    VINS* vins=new VINS();//这样就初始化了
const char *voc_file ="/Users/zhangjianhua/Desktop/VINS_MapFusion/Resources/brief_k10L6.bin";
    //全局优化
//    PoseGraph* poseGraph=new PoseGraph();
    
    PoseGraph* poseGraph=new PoseGraph(voc_file, COL, ROW);
    poseGraph->setVINS(vins);
//        boost::thread globalLoopThread(&PoseGraph::globalLoopRun, poseGraph);
    
    Server::poseGraphs.push_back(poseGraph);
    
    

    
    //Add client in Static clients <vector> (Critical section!)
    MyThread::LockMutex((const char *) c->name);
    
    //Before adding the new client, calculate its id. (Now we have the lock)
    c->SetId(Server::clients.size());
    sprintf(name, "Client n.%d\0", c->id);
    c->SetName(name);
    //    std::cout << "Adding client with id: " << c->id << std::endl;
    Server::clients.push_back(c);
    poseGraph->setClient(c);
    
    MyThread::UnlockMutex((const char *) c->name);


    //回环检测
    
    LoopClosure *loop_closure=new LoopClosure(voc_file, COL, ROW);
    loop_closure->setVINS(vins);
    loop_closure->setPoseGraph(poseGraph);
    loop_closure->setFeatureMap(Server::global_featureMap);
    boost::thread loopClosureThread(&LoopClosure::loopClosureRun_3, loop_closure);
    //这个线程是接力用的
//    boost::thread loopClosureThread_continue(&LoopClosure::loopClosureRun_9, loop_closure);
    

//    //这个线程是接力用的
    boost::thread loopClosureThread_continue(&LoopClosure::loopClosureRun_12, loop_closure);

    if(drawResult==nullptr){
        cout<<"null"<<endl;
    }
    if(Server::clients.size()==1){
        drawResult->mpVins=vins;
        drawResult->mpPoseGraph=poseGraph;
    }
    
//    if(c->is_add==false){
        
        Server::poseGraphGlobal->addPoseGraph(c->id, Server::poseGraphs[c->id]);
        c->is_add=true;
        
//    }
    
//    初始化一个解压缩的
    const char *settings_path = "/Users/zhangjianhua/Desktop/hh/VINS_Mobile/Resources/stats_8b.vstats";
//    LBFC2::CodingStats codingModel;
    c->codingModel.load(settings_path );

    // Setup encoder
    int imgWidth = 752;
    int imgHeight = 480;
    int bufferSize = 1;
    bool inter = true;
    bool stereo = false;
    bool depth = false;
    float mFocalLength=458.654;//这里是fx 好像是没什么用的
    //析构函数 有问题
    c->encoder=new  LBFC2::FeatureCoder(loop_closure->demo, c->codingModel,imgWidth, imgHeight, 32, bufferSize, inter, stereo, depth, mFocalLength);
    c->decoder=new LBFC2::FeatureCoder(loop_closure->demo, c->codingModel,imgWidth, imgHeight, 32, bufferSize, inter, stereo, depth, mFocalLength);

    
//开一个线程 专门发送数据
    std::thread* pDataSendThread= new thread(&Server::dataSendRun,c,poseGraph,vins);
    //开一个线程专门处理接收的数据
    std::thread* pHandleKFDataThread= new thread(&Server::handleKFDataRun,c,poseGraph,vins);
    
//FeatureTracker featuretracker;
    
    bool isKf=false;
    int seg_index_cur=0;

//    int kf_des_index=0,kf_keys_index=0;
//    cv::Mat R, T;

    fd_set fds;
    struct timeval timeout;
    while(true) { //this  is the main loop for slam, just like the original slam to capture image and process it
                    
        FD_ZERO(&fds);
        FD_SET(c->sock, &fds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 10;
        
        int nr=select(int(c->sock+1), &fds, NULL, NULL, &timeout);
        if (nr>0)
        {
            memset(packHead, 0, sizeof(struct PackHead));
            n = ReceiveWithSize(c->sock, packHead, sizeof(struct PackHead));
            if (n<=0) {
                c->status=false;
                //连接断开时，程序中断 不应该中断
                cout<<"n<=0 不再接收数据"<<endl;
                break;
            }
//            cout<<"收到数据 并开始解析："<<getTime()<<endl;
//            TS(handleData);
//            TS(GetBuffer);
            packhead = (struct PackHead*)packHead;
//            cout<<endl<<endl;
//            cout<<"receive packhead packlen"<<packhead->PackLen<<endl;
//            cout<<"receive packhead PackName"<<packhead->PackName<<endl;
            
//            cout<<"n="<<n<<endl;
//            cout<<"packhead->PackName  "<<packhead->PackName<<endl;
            if(strcmp(packhead->PackName, "longConnect")==0){
                cout<<"longConnect if"<<endl;
                //此时 packhead->PackIdx 是用户的id 后续可以用上
                continue;//应该没有数据包需要删除
            }
            
//            TS(isRight);
//            if (!(strcmp(packhead->PackName, "device")==0 || strcmp(packhead->PackName, "g_imu")==0 || strcmp(packhead->PackName, "KF_T")==0 || strcmp(packhead->PackName, "KF_Origin")==0 || strcmp(packhead->PackName, "ErrorLoop")==0 || strcmp(packhead->PackName, "CorrIndex")==0 || strcmp(packhead->PackName, "FrontPose")==0|| strcmp(packhead->PackName, "KF_T_add")==0 || strcmp(packhead->PackName, "KF_key")==0))
//            {
//
//                break;
//            }
            auto iterator = myHashMap.find(packhead->PackName);//find()返回一个指向2的迭代器
            if (iterator == myHashMap.end()){
                cout<<"发送的消息头 不认识"<<endl;
                break;
            }
//            TE(isRight);
            
            if (packhead->PackLen<0)
            {
                break;
            }
//            cout<<"packhead->PackName "<<packhead->PackName<<endl;
            buffer = new char[packhead->PackLen];
//            memset(buffer, 0, sizeof buffer);
            
//            TS(ReceiveWithSize);
            n = ReceiveWithSize(c->sock, buffer, packhead->PackLen);
//            TE(ReceiveWithSize);
            long unsigned int frameId;
//            cout<<"n="<<n<<endl;
//            TE(GetBuffer);
            if(n <= 0) {
                std::cerr << "Error while receiving message from client: " << c->name << " with error code: "<<errno<< std::endl;
            }
            else
            {
                c->buffer_kindsAndLen_mutex->lock();
             
                c->buffer_all_split.push(std::make_pair(buffer,packhead->PackIdx));
 
                char *pushName = new char();//char pushName [10];
                memcpy(pushName,packhead->PackName,sizeof(packhead->PackName));
                std::pair<char *,int> buffer_pair=std::make_pair(pushName,packhead->PackLen);
                
                
                c->buffer_kindsAndLen.push(buffer_pair);

                c->buffer_kindsAndLen_mutex->unlock();
                
                /**
                if((strcmp(packhead->PackName, "device")==0))
                {
                    receiveCameraParam(buffer);
                    vins->setExtrinsic();
                    vins->setIMUModel();
                    featuretracker.vins_pnp.setExtrinsic();
                    featuretracker.vins_pnp.setIMUModel();
                }
                else if((strcmp(packhead->PackName, "g_imu")==0))
                {
                    vins->g=receiveInit_aligmentParam(buffer);
                    cout<<"g   "<<vins->g.norm()<<"   "<<vins->g.transpose()<<endl;
                }
                else if((strcmp(packhead->PackName, "KF_T")==0))
                {
                    kf_des_index=-50;
                    kf_keys_index=-50;
                    KeyFrame* curKf=receiveKF_1_1(buffer);
                    
                    if(c->id!=0){
                       
                        curKf->recordLoopStartIndex = std::make_pair(mainClientID,-1);//前面那个是clientId 0，后面是匹配的其实位置 -1+1
                    }
                    
                    poseGraph->add(curKf);
    //                cout<<"KF global_index=  "<<curKf->global_index<<endl;
                    int segment_test=curKf->segment_index;
                    if(seg_index_cur!=segment_test){
                        seg_index_cur=segment_test;
                        if(poseGraph->max_seg_index<segment_test){
                            poseGraph->max_seg_index=segment_test;
                        }
                        poseGraph->cur_seg_index=segment_test;
                    }
                    
                    //把header放在vins下的header里
                    vins->Headers.push_back(curKf->header);


                }
                else if((strcmp(packhead->PackName, "KF_key")==0))
                {
                    int keys_index=packhead->PackIdx;
                    if(keys_index-kf_keys_index!=1){
                        cout<<"拼接顺序有问题"<<endl;
                    }
                    kf_keys_index++;
                    receiveKF_1_2(buffer,poseGraph);
                }
                else if((strcmp(packhead->PackName, "KF_T_add")==0))
                {
                    int des_index=packhead->PackIdx;
                    if(des_index-kf_des_index!=1){
                        cout<<"拼接顺序有问题"<<endl;
                    }
                    kf_des_index++;
                    receiveKF_add(buffer,poseGraph);
                }
                else if((strcmp(packhead->PackName, "KF_Origin")==0))
                {
                    receiveKF_OriginPose(buffer,poseGraph);
                    vins->Headers.erase(vins->Headers.begin());//删除最老的元素
                }else if((strcmp(packhead->PackName, "ErrorLoop")==0))
                {
                    receiveErrorLoop(buffer,poseGraph);
                }else if((strcmp(packhead->PackName, "CorrIndex")==0))
                {
                    receiveCorrectLoop(buffer,poseGraph);
                }else if(strcmp(packhead->PackName, "FrontPose")==0){
                    receiveFrontPoseRelative(buffer, poseGraph);
                }
                else
                    cerr<<"header error kfVariables, shouldn't"<<endl;
                 
                 */
                
                if(pushName!=NULL){
                    pushName=NULL;
                }
            }
            
            if (buffer!=NULL) {
//                cout<<"buffer!=NULL"<<endl;
//                if(isKf==false){
//                    delete [] buffer;
//                }else{
                    buffer=NULL;
//                }
                
            }
            

            /**
            TS(Optimize);
            int search_cnt=0;
            for(int i=0,j=poseGraph->size();i<j;i++){
                search_cnt++;
                KeyFrame* kf = poseGraph->getLastKeyframe(i);
               
                if(kf->IsOriginUpdate==true){
                    cout<<"IsOriginUpdate true"<<endl;
                //update edge
                // if loop happens in this frame, update pose graph;
                    if (kf->has_loop && kf->sendLoop==false)
                    {
                                                
                        kf->sendLoop=true;
                        vins->kf_global_index.push(kf->global_index);
                        vins->start_global_optimization = true;
                        cout<<"检测到回环 并通过错误回环的检测："<<getTime()<<endl;
                    }
                    break;
                }else{
                    if(search_cnt > 2 * WINDOW_SIZE)
                        break;
                }
                
            }
            TE(Optimize);
            
            cout<<"收到数据 并解析完成："<<packhead->PackName<<" "<<getTime()<<endl; */
//            TE(handleData);
        }
        
        
        
        
    }
    
    //此处是连接断开处，休眠3s，等回环检测和全局优化处理完
    usleep(3000);
    std::ofstream outFile;
    string path= "/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/pose_"+to_string(c->getId())+".txt";
    
    
//    if(!outFile){
//        cout<<"打开文件失败"<<endl;
//    }
    vector<Vector3f> refine_path_test=poseGraph->refine_path;
    vector<double> path_time_test=poseGraph->path_time;//实验用
    vector<Matrix3f> refine_r_test=poseGraph->refine_path_r;//实验用
    
    Quaternion<float> Q;
    int len=refine_r_test.size();
    for(int i=0;i<len;i++){
        Q=refine_r_test[i];
        outFile.open(path, ios::binary | ios::app | ios::in | ios::out);
        //mvsec下面需要乘1e3 *1e3
        outFile<<fixed<<setprecision(0)<<path_time_test[i]<<" ";
        outFile<<fixed<<setprecision(11)<<refine_path_test[i].x()<<" "<<refine_path_test[i].y()<<" "<<refine_path_test[i].z()<<" "<<Q.x()<<" "<<Q.y()<<" "<<Q.z()<<" "<<Q.w()<<"\n";
        outFile.close();
    }
    
    
    {
        list<KeyFrame*> keyFrameList=poseGraph->keyFrameList;
//        记录有哪些关键帧
        string kfNum_tree_dir="/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/1/";
        std::ofstream outFile;
        outFile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
        outFile.precision(16);
        //ios::binary|
//        int agentId=cur_kf->c->getId();
//        kfNum_tree_dir+=to_string(agentId);
        outFile.open(kfNum_tree_dir+"0_pgKF.txt",ios::out|ios::app);
//                    cout<<"kfNum_tree_dir="<<kfNum_tree_dir+"intra_loop.txt" <<endl;
        for(auto iter=keyFrameList.begin();iter!=keyFrameList.end();iter++){
            outFile<< (*iter)->global_index<<"\n";
        }
        
        
        //关闭文件
        outFile.close();
    }
    printf("record pose end ");
//    usleep(3000);
    poseGraph->isEnd=true;
    
    bool isEnd=true;
    
//    临时实验用
    bool isOne=true;
    //实验用
    while(true){
        isEnd=true;
        usleep(3000);
//        Server::poseGraphGlobal->poseGraphGlobal_mutex.lock();
        
        if(Server::poseGraphGlobal->PoseGraphGloabl_map.size()>=2&&oneEnd_server){
            for(auto iter=Server::poseGraphGlobal->PoseGraphGloabl_map.begin() , iter_end=Server::poseGraphGlobal->PoseGraphGloabl_map.end();iter!=iter_end;iter++){
                PoseGraph* poseGraph_other=iter->second;
                if(poseGraph_other->isEnd==false){
                    isEnd=false;
//                    Server::poseGraphGlobal->poseGraphGlobal_mutex.unlock();
                    break;
                }
            }
            
            if(isEnd)
            {
                oneEnd_server=false;
                //记录位姿
                
                //此处是连接断开处，休眠3s，等回环检测和全局优化处理完
                    
                    std::ofstream outFile;
                    string path= "/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/pose_end.txt";
                    
                
                for(auto iter_start_record=Server::poseGraphGlobal->PoseGraphGloabl_map.begin(),iter_end_record=Server::poseGraphGlobal->PoseGraphGloabl_map.end();iter_start_record!=iter_end_record;iter_start_record++){
                    PoseGraph* poseGraph_all=iter_start_record->second;
                    //refine_path_draw_mutex
                    unique_lock<mutex> lock_5(poseGraph_all->refine_path_draw_mutex);
                    vector<Vector3f> refine_path_test=poseGraph_all->refine_path_draw;
                    lock_5.unlock();
                    
                    unique_lock<mutex> lock_6(poseGraph_all->mMutexkeyFrameList);
                    vector<double> path_time_test=poseGraph_all->path_time;//实验用
                    vector<Matrix3f> refine_r_test=poseGraph_all->refine_path_r_draw;//实验用
                    lock_6.unlock();
                    
                    Quaternion<float> Q;
                    int len=refine_r_test.size();
                    for(int i=0;i<len;i++){
                        Q=refine_r_test[i];
                        outFile.open(path, ios::binary | ios::app | ios::in | ios::out);
                        outFile<<fixed <<setprecision(0)<<path_time_test[i]<<" ";
                        outFile<<fixed <<setprecision(11)<<refine_path_test[i].x()<<" "<<refine_path_test[i].y()<<" "<<refine_path_test[i].z()<<" "<<Q.x()<<" "<<Q.y()<<" "<<Q.z()<<" "<<Q.w()<<"\n";
                        outFile.close();
                    }
                }
                
                    printf("record pose end ");
//                Server::poseGraphGlobal->poseGraphGlobal_mutex.unlock();
                
                
//                std::ofstream outFile;
//                string path_header= "/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/kf_agent";
                
            
           
            
//                ----------------记录子地图的划分--------------
                
                string subMap_dir="/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/subMap/";
//                所有地图
                for(int x=0,y=global_featureMap->agent_id.size();x<y;x++){
                    int agent_id_cur=global_featureMap->agent_id[x];
                    string subMap_dir_cur=subMap_dir+to_string(agent_id_cur)+"_agent/";
                    mkdir(subMap_dir_cur.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
//                    每个地图 的所有子地图
                    list<FeatureSubMap*> ml_submap_cur=global_featureMap->ml_submap[x];
                    for(auto iter_submap_start=ml_submap_cur.begin(),iter_submap_end=ml_submap_cur.end();iter_submap_start!=iter_submap_end;iter_submap_start++){
                        FeatureSubMap *featureSubMap_cur=(*iter_submap_start);
                        string submap_allId=subMap_dir_cur;
                        int submap_global_index=featureSubMap_cur->global_index;
                        submap_allId+=to_string(submap_global_index);
                        
                        
                        int submap_size=featureSubMap_cur->ml_keyframe.size()+featureSubMap_cur->ml2_kf.size()+featureSubMap_cur->noRepresent_kfId.size()+featureSubMap_cur->noRepresent_kfId_l2.size();
                        submap_allId+="_";
                        submap_allId+=to_string(submap_size);
                        submap_allId+="_subMap";
                        for(int d=0,e=featureSubMap_cur->global_index_moreEarly.size();
                            d<e;d++){
                            submap_allId+="&";
                            submap_allId+=to_string(featureSubMap_cur->global_agent_id_moreEarly[d]);
                            submap_allId+="_";
                            submap_allId+=to_string(featureSubMap_cur->global_index_moreEarly[d]);
                        }
                        
                        std::ofstream outFile;
                        outFile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
                        outFile.precision(16);
                        outFile.open(submap_allId+".txt");
                                //写入数据
                        outFile<<featureSubMap_cur->ml_keyframe.size();
                        outFile<<" ml_keyframe client_id、kf_id、kf_header、ml_kf_id\n";
                        int d=0;
                        for(auto iter_ml_start=featureSubMap_cur->ml_keyframe.begin(), iter_ml_end=featureSubMap_cur->ml_keyframe.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                            outFile << (*iter_ml_start)->c->getId()<<" "<<(*iter_ml_start)->global_index<<" "<<(*iter_ml_start)->header<<" " <<featureSubMap_cur->ml_kfId[d]<<"\n";
                            d++;
                        }
                        outFile<<"\n";
                        outFile<<"\n";
                        
                        outFile<<featureSubMap_cur->ml2_kf.size();
                        outFile<<" ml2_kf client_id、kf_id、kf_header、ml2_kf_id\n";
                        d=0;
                        for(auto iter_ml_start=featureSubMap_cur->ml2_kf.begin(), iter_ml_end=featureSubMap_cur->ml2_kf.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                            outFile << (*iter_ml_start)->c->getId()<<" "<<(*iter_ml_start)->global_index<<" "<<(*iter_ml_start)->header<<" " <<featureSubMap_cur->ml2_kfId[d]<<"\n";
                            d++;
                        }
                        outFile<<"\n";
                        outFile<<"\n";
                        
                        outFile<<featureSubMap_cur->noRepresent_kfId.size();
                        outFile<<" noRepresent_kfId noRekf_id、noRekf_header\n";
                        d=0;
                        for(auto iter_ml_start=featureSubMap_cur->noRepresent_kfId.begin(), iter_ml_end=featureSubMap_cur->noRepresent_kfId.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                            outFile << (*iter_ml_start)<<" " <<featureSubMap_cur->noRepresent_kfHeader[d]<<"\n";
                            d++;
                        }
                        outFile<<"\n";
                        outFile<<"\n";
                        
                        outFile<<featureSubMap_cur->noRepresent_kfId_l2.size();
                        outFile<<" noRepresent_kfHeader_l2 noRekf_id\n";
                        d=0;
                        for(auto iter_ml_start=featureSubMap_cur->noRepresent_kfId_l2.begin(), iter_ml_end=featureSubMap_cur->noRepresent_kfId_l2.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                            outFile << (*iter_ml_start)<<" " <<featureSubMap_cur->noRepresent_kfHeader_l2[d]<<"\n";
                            d++;
                        }
                        outFile<<"\n";
                        outFile<<"\n";
                        
                        outFile<<featureSubMap_cur->relatedSubMapIndex.size();
                        outFile<<" relatedSubMapIndex SubMap_id\n";
                        d=0;
                        for(auto iter_ml_start=featureSubMap_cur->relatedSubMapIndex.begin(), iter_ml_end=featureSubMap_cur->relatedSubMapIndex.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                            outFile << (*iter_ml_start)<<"\n";
                            d++;
                        }
                        outFile<<"\n";
                        outFile<<"\n";
                        
                        outFile<<featureSubMap_cur->related_otherAgentSubMapIndex.size();
                        outFile<<" related_otherAgentSubMapIndex AgentSubMapIndex\n";
                        d=0;
                        for(auto iter_ml_start=featureSubMap_cur->related_otherAgentSubMapIndex.begin(), iter_ml_end=featureSubMap_cur->related_otherAgentSubMapIndex.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                            outFile << (*iter_ml_start)<<"\n";
                            d++;
                        }
                        outFile<<"\n";
                        outFile<<"\n";
                        
                        outFile<<featureSubMap_cur->related_otherAgentId.size();
                        outFile<<" related_otherAgentId AgentId\n";
                        d=0;
                        for(auto iter_ml_start=featureSubMap_cur->related_otherAgentId.begin(), iter_ml_end=featureSubMap_cur->related_otherAgentId.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                            outFile << (*iter_ml_start)<<"\n";
                            d++;
                        }
                        outFile<<"\n";
                        outFile<<"\n";
                        
                        
                        outFile<<"global_index SubMap_id\n";
                        outFile<<featureSubMap_cur->global_index<<"\n";
                        d=0;
                        for(auto iter_ml_start=featureSubMap_cur->global_index_moreEarly.begin(), iter_ml_end=featureSubMap_cur->global_index_moreEarly.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                            outFile << (*iter_ml_start)<<"\n";
                            d++;
                        }
                        
                        outFile<<"\n";
                        outFile<<"\n";
                        
                        
                        outFile<<"global_agent_id SubMap_agent_id\n";
                        outFile<<featureSubMap_cur->global_agent_id<<"\n";
                        d=0;
                        for(auto iter_ml_start=featureSubMap_cur->global_agent_id_moreEarly.begin(), iter_ml_end=featureSubMap_cur->global_agent_id_moreEarly.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                            outFile << (*iter_ml_start)<<"\n";
                            d++;
                        }
                        
                        //关闭文件
                        outFile.close();
                    }
                    
              
                }
                 
            
                 
//                 ------
                
            }
        }
//        else{
//            Server::poseGraphGlobal->poseGraphGlobal_mutex.unlock();
//        }
        
//        先验证一个地图保存下来的地图分配结果
        
        /**
        if(isOne)
        {
            isOne=false;
//                ----------------记录子地图的划分--------------
            string subMap_dir="/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/subMap/";
//                所有地图
            for(int x=0,y=global_featureMap->agent_id.size();x<y;x++){
                int agent_id_cur=global_featureMap->agent_id[x];
                string subMap_dir_cur=subMap_dir+to_string(agent_id_cur)+"_agent/";
                mkdir(subMap_dir_cur.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
//                    每个地图 的所有子地图
                list<FeatureSubMap*> ml_submap_cur=global_featureMap->ml_submap[x];
                for(auto iter_submap_start=ml_submap_cur.begin(),iter_submap_end=ml_submap_cur.end();iter_submap_start!=iter_submap_end;iter_submap_start++){
                    FeatureSubMap *featureSubMap_cur=(*iter_submap_start);
                    string submap_allId=subMap_dir_cur;
                    int submap_global_index=featureSubMap_cur->global_index;
                    submap_allId+=to_string(submap_global_index);
                    
                    int submap_size=featureSubMap_cur->ml_keyframe.size()+featureSubMap_cur->ml2_kf.size()+featureSubMap_cur->noRepresent_kfId.size()+featureSubMap_cur->noRepresent_kfId_l2.size();
                    submap_allId+="_";
                    submap_allId+=to_string(submap_size);
                    
                    submap_allId+="_subMap";
                    for(int d=0,e=featureSubMap_cur->global_index_moreEarly.size(); d<e;d++){
                        submap_allId+="&";
                        submap_allId+=to_string(featureSubMap_cur->global_agent_id_moreEarly[d]);
                        submap_allId+="_";
                        submap_allId+=to_string(featureSubMap_cur->global_index_moreEarly[d]);
                    }

                    std::ofstream outFile;
                    outFile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
                    outFile.precision(16);
                    outFile.open(submap_allId+".txt");
                            //写入数据
                    outFile<<featureSubMap_cur->ml_keyframe.size();
                    outFile<<" ml_keyframe client_id、kf_id、kf_header、ml_kf_id\n";
                    int d=0;
                    for(auto iter_ml_start=featureSubMap_cur->ml_keyframe.begin(), iter_ml_end=featureSubMap_cur->ml_keyframe.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                        outFile << (*iter_ml_start)->c->getId()<<" "<<(*iter_ml_start)->global_index<<" "<<(*iter_ml_start)->header<<" " <<featureSubMap_cur->ml_kfId[d]<<"\n";
                        d++;
                    }
                    outFile<<"\n";
                    outFile<<"\n";

                    outFile<<featureSubMap_cur->ml2_kf.size();
                    outFile<<" ml2_kf client_id、kf_id、kf_header、ml2_kf_id\n";
                    d=0;
                    for(auto iter_ml_start=featureSubMap_cur->ml2_kf.begin(), iter_ml_end=featureSubMap_cur->ml2_kf.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                        outFile << (*iter_ml_start)->c->getId()<<" "<<(*iter_ml_start)->global_index<<" "<<(*iter_ml_start)->header<<" " <<featureSubMap_cur->ml2_kfId[d]<<"\n";
                        d++;
                    }
                    outFile<<"\n";
                    outFile<<"\n";

                    outFile<<featureSubMap_cur->noRepresent_kfId.size();
                    outFile<<" noRepresent_kfId noRekf_id、noRekf_header\n";
                    d=0;
                    for(auto iter_ml_start=featureSubMap_cur->noRepresent_kfId.begin(), iter_ml_end=featureSubMap_cur->noRepresent_kfId.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                        outFile << (*iter_ml_start)<<" " <<featureSubMap_cur->noRepresent_kfHeader[d]<<"\n";
                        d++;
                    }
                    outFile<<"\n";
                    outFile<<"\n";

                    outFile<<featureSubMap_cur->noRepresent_kfId_l2.size();
                    outFile<<" noRepresent_kfHeader_l2 noRekf_id\n";
                    d=0;
                    for(auto iter_ml_start=featureSubMap_cur->noRepresent_kfId_l2.begin(), iter_ml_end=featureSubMap_cur->noRepresent_kfId_l2.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                        outFile << (*iter_ml_start)<<" " <<featureSubMap_cur->noRepresent_kfHeader_l2[d]<<"\n";
                        d++;
                    }
                    outFile<<"\n";
                    outFile<<"\n";
                    
                    outFile<<featureSubMap_cur->relatedSubMapIndex.size();
                    outFile<<" relatedSubMapIndex SubMap_id\n";
                    d=0;
                    for(auto iter_ml_start=featureSubMap_cur->relatedSubMapIndex.begin(), iter_ml_end=featureSubMap_cur->relatedSubMapIndex.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                        outFile << (*iter_ml_start)<<"\n";
                        d++;
                    }
                    
                    outFile<<featureSubMap_cur->related_otherAgentSubMapIndex.size();
                    outFile<<" related_otherAgentSubMapIndex AgentSubMapIndex\n";
                    d=0;
                    for(auto iter_ml_start=featureSubMap_cur->related_otherAgentSubMapIndex.begin(), iter_ml_end=featureSubMap_cur->related_otherAgentSubMapIndex.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                        outFile << (*iter_ml_start)<<"\n";
                        d++;
                    }
                    outFile<<"\n";
                    outFile<<"\n";
                    
                    outFile<<featureSubMap_cur->related_otherAgentId.size();
                    outFile<<" related_otherAgentId AgentId\n";
                    d=0;
                    for(auto iter_ml_start=featureSubMap_cur->related_otherAgentId.begin(), iter_ml_end=featureSubMap_cur->related_otherAgentId.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                        outFile << (*iter_ml_start)<<"\n";
                        d++;
                    }
                    outFile<<"\n";
                    outFile<<"\n";
                    
                    
                    outFile<<"global_index SubMap_id\n";
                    outFile<<featureSubMap_cur->global_index<<"\n";
                    d=0;
                    for(auto iter_ml_start=featureSubMap_cur->global_index_moreEarly.begin(), iter_ml_end=featureSubMap_cur->global_index_moreEarly.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                        outFile << (*iter_ml_start)<<"\n";
                        d++;
                    }
                    
                    outFile<<"\n";
                    outFile<<"\n";
                    
                    
                    outFile<<"global_agent_id SubMap_agent_id\n";
                    outFile<<featureSubMap_cur->global_agent_id<<"\n";
                    d=0;
                    for(auto iter_ml_start=featureSubMap_cur->global_agent_id_moreEarly.begin(), iter_ml_end=featureSubMap_cur->global_agent_id_moreEarly.end(); iter_ml_start!=iter_ml_end;iter_ml_start++){
                        outFile << (*iter_ml_start)<<"\n";
                        d++;
                    }
                    //关闭文件
                    outFile.close();
                }
            }
//
            
            string kfNum_tree_dir="/Users/zhangjianhua/Desktop/VINS_MapFusion/VINS_MapFusion/data/kfNum_tree_dir/";
            std::ofstream outFile;
            outFile.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
            outFile.precision(16);
            int treeId_len=loop_closure->treeId_kf.size();
            kfNum_tree_dir+=to_string(treeId_len);
            kfNum_tree_dir+="_agent";
            kfNum_tree_dir+=to_string(c->getId());
            outFile.open(kfNum_tree_dir+"_treeId_kf.txt");
            
            
            for(auto iter=loop_closure->treeId_kf.begin();iter!=loop_closure->treeId_kf.end();iter++){
                outFile <<iter->second<<"\n";
               
            }
            //关闭文件
            outFile.close();
//            cout<<"子地图保存结束"<<endl;
//                 ------
            
        }
         */
    }
    return NULL;
}

void Server::dataSendRun(Client *c,PoseGraph *poseGraph,VINS* vins){
    int clientId_send=c->id;
    //检测到哪个用户端数据需要发送
    while(true){
//        if(!vins->send_status_index.empty()){
//            sendStatus_rejectWithF(c,vins);
//        }
        
        if(!c->status){
            usleep(3000);
            continue;
        }
        //回环不再发送 在服务器端计算
        if(vins->isSendLoopData){
            vins->isSendLoopData=false;
            sendLoop(c,vins);
            usleep(10);
        }
        if(vins->isSendLoop_another){
            vins->isSendLoop_another=false;
            sendLoop_another(c,vins);
            usleep(10);
        }
        if(vins->isSendLoop_another2){
            vins->isSendLoop_another2=false;
            sendLoop_another2(c,vins);
            usleep(10);
        }
        if(poseGraph->isSendGlobalData){
            poseGraph->isSendGlobalData=false;
            sendGlobalLoop(c,poseGraph);
            usleep(1170);
        }
        if(poseGraph->isSendGloablData_multiClient){
            poseGraph->isSendGloablData_multiClient=false;
            
//            cout<<"测试发送全局优化： 用户id="<<poseGraph->keyFrameList.front()->c->id<<endl;
            //这里更新一下Client那边的位姿
            sendGlobalLoop_multiClient(c,poseGraph);
            //这里更新一下Ar
            //发送AR 检查有没有第一次融合成功的 这里暂时是默认 主地图创建的AR
            for(int a=0, b=Server::poseGraphGlobal->grounds.size();a<b;a++){
                GroundPoint gp_single=Server::poseGraphGlobal->grounds[a];
                if(gp_single.isSendAr[clientId_send]==0){
                    sendAr(c, a);
                    Server::poseGraphGlobal->grounds[a].isSendAr[clientId_send]=1;
                }
            }
            usleep(1170);
        }
        usleep(10);
    }
}



void Server::handleKFDataRun(Client *c,PoseGraph *poseGraph,VINS* vins){
    int seg_index_cur=0,packLen=-1;
    int kf_des_index=0,kf_keys_index=0,kf_des_ComIndex=0,kf_des_compress=0;
//    char packName[10];
    char * buffer;
    FeatureTracker featuretracker;
    while(true){
        c->buffer_kindsAndLen_mutex->lock();
//
        if(!c->buffer_kindsAndLen.empty()){
//            cout<<endl;
            
            
            std::pair<char *, int> pair_kindsAndLen=c->buffer_kindsAndLen.front();
            c->buffer_kindsAndLen.pop();
            
//            strcpy(packName, pair_kindsAndLen.first);
//            memcpy(packName,pair_kindsAndLen.first,strlen(pair_kindsAndLen.first));
            
            
            std::pair<char *, int> pair_bufferAndIdx=c->buffer_all_split.front();
            c->buffer_all_split.pop();
            c->buffer_kindsAndLen_mutex->unlock();
            
            packLen=pair_kindsAndLen.second;
//            char *buffer_head=pair_bufferAndIdx.first;
//            cout<<"buffer_head="<<strlen(buffer_head)<<endl;
            
            buffer = new char[packLen];
//            memset(buffer, '/0', packLen);
//            strcpy(buffer, buffer_head);
            memcpy(buffer,pair_bufferAndIdx.first,packLen);
//            cout<<"packLen"<<packLen<<endl;
//            cout<<"buffer="<<pair_kindsAndLen.first<<" , "<<packLen<<endl;
            
            
//            cout<<"packName="<<pair_kindsAndLen.first<<endl;
            
            if((strcmp(pair_kindsAndLen.first, "device")==0))
            {
                DeviceType deviceType=receiveCameraParam(buffer);
                
                c->setCam_intrinsic(deviceType);
                featuretracker.setCam_intrinsic(deviceType);
                
                double  FOCUS_LENGTH_X_server=c->FOCUS_LENGTH_X_server;
//                vins->setExtrinsic();
                vins->setIMUModel(FOCUS_LENGTH_X_server);
//                featuretracker.vins_pnp.setExtrinsic();
                featuretracker.vins_pnp.setIMUModel(FOCUS_LENGTH_X_server);
                
            }
            else if((strcmp(pair_kindsAndLen.first, "g_imu")==0))
            {
                vins->g=receiveInit_aligmentParam(buffer);
                cout<<"g   "<<vins->g.norm()<<"   "<<vins->g.transpose()<<endl;
            }
            else if((strcmp(pair_kindsAndLen.first, "KF_T")==0))
            {
                
                kf_des_index=-50;
                kf_keys_index=-50;
                kf_des_ComIndex=-50;
                kf_des_compress=-50;
                KeyFrame* curKf=receiveKF_1_1(buffer);
                curKf->c=c;
//                cout<<"接收到关键帧"<<curKf->global_index<<endl;
                
                if(c->id!=0){
                   
                    curKf->recordLoopStartIndex = std::make_pair(mainClientID,-1);//前面那个是clientId 0，后面是匹配的其实位置 -1+1
                }
                 
                
                poseGraph->add(curKf);
                
                //更新共视关系
                int conn_i=0;
                curKf->mMutexConnections.lock();
                for(vector<int>::iterator i=curKf->mvOrderedConnectedKeyFrames_global_index.begin(), j=curKf->mvOrderedConnectedKeyFrames_global_index.end(); i!=j; i++){
                    KeyFrame* kf =poseGraph->getLastKeyframe_index(*i);
                    if(kf!=NULL){
                        curKf->mvpOrderedConnectedKeyFrames.push_back(kf);
                        kf->AddConnection_weight(curKf, curKf->mvOrderedWeights[conn_i]);
                    }else{
                        //找不到该关键帧 不应该  的确存在找不到的时候
                        assert(false);
                    }
                    conn_i++;
                }
                //因为本来只是临时变量，所以用完清空掉
                curKf->mvOrderedConnectedKeyFrames_global_index.clear();
                curKf->mMutexConnections.unlock();
                
                
//                cout<<"receive KF global_index=  "<<curKf->global_index<<endl;
                int segment_test=curKf->segment_index;
                if(seg_index_cur!=segment_test){
                    seg_index_cur=segment_test;
                    if(poseGraph->max_seg_index<segment_test){
                        poseGraph->max_seg_index=segment_test;
                    }
                    poseGraph->cur_seg_index=segment_test;
                }
                
                //把header放在vins下的header里
                vins->Headers.push_back(curKf->header);
                
                if(c->isUndistorted){
                    curKf->isUndistorted_kf=true;
                }
            }
            else if((strcmp(pair_kindsAndLen.first, "KF_key")==0))
            {
                int keys_index=pair_bufferAndIdx.second;
                if(keys_index-kf_keys_index!=1){
                    cout<<"KF_key 拼接顺序有问题"<<endl;
                }
                kf_keys_index++;
                
                receiveKF_1_2(buffer,poseGraph);
            }
            else if((strcmp(pair_kindsAndLen.first, "KF_T_add")==0))
            {
                int des_index=pair_bufferAndIdx.second;
                if(des_index-kf_des_index!=1){
                    cout<<"KF_T_add 拼接顺序有问题"<<endl;
                }
                kf_des_index++;
                receiveKF_add(buffer,poseGraph);
            }
            else if((strcmp(pair_kindsAndLen.first, "KF_bowI")==0))
            {
                int des_index=pair_bufferAndIdx.second;
                if(des_index-kf_des_ComIndex!=1){
                    cout<<"KF_bowI 拼接顺序有问题"<<endl;
                }
                kf_des_ComIndex++;
                receiveKF_add_compressIndex(buffer,poseGraph,packLen);
            }
            else if((strcmp(pair_kindsAndLen.first, "KF_bowC")==0))
            {
                int des_index=pair_bufferAndIdx.second;
                if(des_index-kf_des_compress!=1){
                    cout<<"KF_bowC 拼接顺序有问题"<<endl;
                }
                kf_des_compress++;
                receiveKF_add_compress(buffer,poseGraph,c,packLen);
            }
            else if((strcmp(pair_kindsAndLen.first, "KF_Origin")==0))
            {
                //这里得到更新原始位姿的帧
                KeyFrame* cur_kf=receiveKF_OriginPose(buffer,poseGraph);
                vins->Headers.erase(vins->Headers.begin());//删除最老的元素
//                暂时注释 感觉多余
                if(cur_kf!=nullptr){
                    if (cur_kf->has_loop && cur_kf->sendLoop==false && cur_kf->is_get_loop_info)
                    {

                        cur_kf->sendLoop=true;
                        vins->globalOpti_index_mutex.lock();
                        vins->kf_global_index.push(cur_kf->global_index);
                        vins->start_kf_global_index.push(cur_kf->loop_index);
                        poseGraph->latest_loop_index=cur_kf->global_index;
                        vins->start_global_optimization = true;//先暂停回环 不启动

                        vins->globalOpti_index_mutex.unlock();
                        cout<<"检测到回环 并通过错误回环的检测 这里是server端唤醒："<<cur_kf->global_index<<" , "<<getTime()<<endl;
                    }
                }
            }else if((strcmp(pair_kindsAndLen.first, "ErrorLoop")==0))
            {
                cout<<"移除错误的回环"<<endl;
                receiveErrorLoop(buffer,poseGraph);
            }else if((strcmp(pair_kindsAndLen.first, "CorrIndex")==0))
            {
                cout<<"*** CorrectLoop ***"<<endl;
                receiveCorrectLoop(buffer,poseGraph);
            }else if(strcmp(pair_kindsAndLen.first, "FrontPose")==0){
                receiveFrontPoseRelative(buffer, poseGraph);
            }
            else if(strcmp(pair_kindsAndLen.first, "FrontPos2")==0){
                cout<<"回环优化"<<endl;
                receiveFrontPoseRelative_add_pitch_roll(buffer, poseGraph);
            }else if(strcmp(pair_kindsAndLen.first, "FrontPos3")==0){
                cout<<"FrontPos3"<<endl;
                receiveFrontPoseRelative_add_pitch_roll_forRemove(buffer, poseGraph);
            }else if(strcmp(pair_kindsAndLen.first, "Ar")==0){
                int ground_ar_idx=receiveAr(buffer, poseGraphGlobal, c->id);
                //发送给其它client，另外新建一个client的时候，要检查是否已经存在AR了  不需要检查，因为还没有融合成功
                //现在这里默认 是主地图创建的AR物体
                for(int i=1;i<10;i++){
                    //这里是因为ar物体是刚创建的 所以 其它人肯定都没有，所以直接发送，但是要标记这些人已经发送过了
                    
                    if(poseGraphGlobal->isFusion[i]==1){//判断i和主地图是否融合成功
                        Client *client_other_sendAr=Server::clients[i];
                        if(client_other_sendAr->id==i){
                            sendAr(client_other_sendAr,ground_ar_idx);
                            poseGraphGlobal->grounds[ground_ar_idx].isSendAr[i]=1;
                        }else{
                            assert(false);//按理说是按顺序放到 应该没有放错
                        }
                        
                        
                    }else
                        continue;
                }
            }
            else
                cerr<<"header error kfVariables, shouldn't"<<endl;
            
                        
            if(buffer!=NULL){
                delete [] buffer;
            }
            if(pair_bufferAndIdx.first!=NULL){
                delete pair_bufferAndIdx.first;
            }
            if(pair_kindsAndLen.first!=NULL){
                delete pair_kindsAndLen.first;
            }
            
            //待改 这里可以记录 上次已经遍历到哪个位置 接着那个位置遍历
//            TS(Optimize);
            //这里暂时注释，回环如果在客户端，则取消注释
//           int search_cnt=0;
//           for(int i=0,j=poseGraph->keyFrameList.size();i<j;i++){
//               search_cnt++;
//               KeyFrame* kf = poseGraph->getLastKeyframe(i);
//
//               if(kf->IsOriginUpdate==true){
////                   cout<<"IsOriginUpdate true"<<endl;
//               //update edge
//               // if loop happens in this frame, update pose graph;
//                   if (kf->has_loop && kf->sendLoop==false && kf->is_get_loop_info)
//                   {
//
//                       kf->sendLoop=true;
//                       vins->globalOpti_index_mutex.lock();
//                       vins->kf_global_index.push(kf->global_index);
//                       vins->start_kf_global_index.push(kf->loop_index);
//                       poseGraph->latest_loop_index=kf->global_index;
//                       vins->start_global_optimization = true;//先暂停回环 不启动
//
//                       vins->globalOpti_index_mutex.unlock();
//                       cout<<"检测到回环 并通过错误回环的检测："<<kf->global_index<<endl;
//                   }
//
//                   break;
//               }else{
//                   if(search_cnt > 2 * WINDOW_SIZE)
//                       break;
//               }
//
////               if(kf->isUndistorted_kf==false && kf->isAllKeypoint){
////                   c->undistortedPoints(kf->keypoints);
////                   kf->setWin_keyPoint_des();
////               }
//
//           }
//           TE(Optimize);
        }
        else{
            c->buffer_kindsAndLen_mutex->unlock();
           
        }
        
        usleep(10);
    }
}

void Server::sendGlobalLoop_multiClient(Client *c,PoseGraph *poseGraph){
    int len;
    
    char* bytearr=sendGlobalLoopData_multiClient(len,poseGraph);
//    cout<<"c->id"<<c->id<<" len="<<len<<endl;
    MyThread::LockMutex("sendDataToClient()");
    int n = send(c->sock, bytearr, len, 0);
    
    while(n==-1){
         n = send(c->sock, bytearr, len, 0);
        std::cout <<n << " bytes of sendGlobalLoop_multiClient changed data sent." << std::endl;
    }
    MyThread::UnlockMutex("sendDataToClient()");
    std::cout <<n << " bytes of sendGlobalLoop_multiClient changed data sent." << std::endl;
}


void Server::sendAr(Client *c, int ground_ar_idx){
    int len;
    
    char* bytearr=sendArData(len,poseGraphGlobal,ground_ar_idx,c->id);
    MyThread::LockMutex("sendDataToClient()");
    int n = send(c->sock, bytearr, len, 0);
    MyThread::UnlockMutex("sendDataToClient()");
    std::cout <<n << " bytes of sendAr changed data sent." << std::endl;
}

void Server::sendStatus_rejectWithF(Client *c,VINS* vins){
    int len;
    
    char* bytearr=sendRejectWithF(len,vins);
    MyThread::LockMutex("sendDataToClient()");
    int n = send(c->sock, bytearr, len, 0);
    MyThread::UnlockMutex("sendDataToClient()");
    std::cout <<n << " bytes of sendStatus_rejectWithF changed data sent." << std::endl;
}

void Server::sendLoop(Client *c,VINS* vins){
    int len;
    
    char* bytearr=sendLoopData(len,vins);
    MyThread::LockMutex("sendDataToClient()");
    int n = send(c->sock, bytearr, len, 0);
    MyThread::UnlockMutex("sendDataToClient()");
    std::cout <<n << " bytes of loop closing changed data sent." << std::endl;
}

//这里是只是告诉客户端有回环了，不用它去做优化算相对位姿
void Server::sendLoop_another(Client *c,VINS* vins){
    int len;
    
    char* bytearr=sendLoopData_another(len,vins);
    MyThread::LockMutex("sendDataToClient()");
    int n = send(c->sock, bytearr, len, 0);
    MyThread::UnlockMutex("sendDataToClient()");
//    std::cout <<n << " bytes of loop_another closing changed data sent." << std::endl;
}
//这里是只是告诉客户端有回环了，不用它去做优化算相对位姿
void Server::sendLoop_another2(Client *c,VINS* vins){
    int len;
    
    char* bytearr=sendLoopData_another2(len,vins);
    MyThread::LockMutex("sendDataToClient()");
    int n = send(c->sock, bytearr, len, 0);
    MyThread::UnlockMutex("sendDataToClient()");
//    std::cout <<n << " bytes of loop_another closing changed data sent." << std::endl;
}

void Server::sendGlobalLoop(Client *c,PoseGraph *poseGraph){
    int len;
    
    char* bytearr=sendGlobalLoopData(len,poseGraph);
    
    MyThread::LockMutex("sendDataToClient()");
    int n = send(c->sock, bytearr, len, 0);
    MyThread::UnlockMutex("sendDataToClient()");
//    std::cout <<n << " bytes of global loop changed data sent." << std::endl<< std::endl;
}


/**

void Server::SendToAll(char *message) {
    int n;
    
    //Acquire the lock
    MyThread::LockMutex("'SendToAll()'");
    
    for(size_t i=0; i<clients.size(); i++) {
        n = send(Server::clients[i].sock, message, strlen(message), 0);
        std::cout << n << " bytes sent." << std::endl;
    }
    
    //Release the lock
    MyThread::UnlockMutex("'SendToAll()'");
}
*/
void Server::ListClients() {
    for(size_t i=0; i<clients.size(); i++) {
        std::cout << clients.at(i)->name << std::endl;
    }
}
 

/*
 Should be called when vector<Client> clients is locked!
 */
int Server::FindClientIndex(Client *c) {
    for(size_t i=0; i<clients.size(); i++) {
        if((Server::clients[i]->id) == c->id) return (int) i;
    }
    std::cerr << "Client id not found." << std::endl;
    return -1;
}


//这个写一下同时跑 每个代理都有这个线程
void * Server::PoseGraphGlobalRun_inServer_2(void *args){
    const char *voc_file ="/Users/zhangjianhua/Desktop/VINS_MapFusion/Resources/brief_k10L6.bin";
    
    cout<<"PoseGraphGlobalRun_inServer_2 进来几次"<<endl;
//    ----------拿到数据 client posegraph vins------------------
    Client *c = (Client *) args;
    usleep(1000);
    
    PoseGraph* poseGraph;
    VINS* vins;
    int client_id=c->id;
    
    while(client_id==-1){
        usleep(1000);
        client_id=c->id;
    }
     while(poseGraphGlobal==nullptr)
         usleep(2000);
    
//    Server::poseGraphGlobal->poseGraphGlobal_mutex.lock();
    Server::poseGraphGlobal->readWriteLock.readLock();
    map<int,PoseGraph*>::iterator iter_map = poseGraphGlobal->PoseGraphGloabl_map.find(client_id);
    while(iter_map==poseGraphGlobal->PoseGraphGloabl_map.end()){
//        Server::poseGraphGlobal->poseGraphGlobal_mutex.unlock();
        Server::poseGraphGlobal->readWriteLock.readUnLock();
        usleep(1000);
//        Server::poseGraphGlobal->poseGraphGlobal_mutex.lock();
        Server::poseGraphGlobal->readWriteLock.readLock();
        iter_map = poseGraphGlobal->PoseGraphGloabl_map.find(client_id);
    }
    
    if(iter_map!=poseGraphGlobal->PoseGraphGloabl_map.end())
    {
        poseGraph = iter_map->second;
        vins = poseGraph->vins;

    }else{
        assert(false);
    }
//    Server::poseGraphGlobal->poseGraphGlobal_mutex.unlock();
    Server::poseGraphGlobal->readWriteLock.readUnLock();
    
    
//   const demoDetector_server<BriefVocabulary, BriefLoopDetector_server, FBrief::TDescriptor> demo_global(voc_file,COL,ROW);
//    Server::poseGraphGlobal->mv_demo_global.push_back(demo_global);
    
//    --------------插入数据到关键帧数据库
    while(true){
        //做个测试 先改成0
        if(Server::clients.size()>=1){
//            这里应该要加锁 后续换成读写锁 这里是读锁
            unique_lock<mutex> lock(poseGraph->mMutexkeyFrameList);
            list<KeyFrame*> keyFrameList_all=poseGraph->keyFrameList;
            lock.unlock();
            
            //---------每次位置识别检测之前 先把主地图的元素加入到database---------------
            int kf_all_num=keyFrameList_all.size();//这里的keyframe长度是不对的，因为keyframe是拆开发的
            int kf_reverse_num=kf_all_num-Server::poseGraphGlobal->mv_pushDatabase_num[client_id];
            if(kf_reverse_num>0){
                int kf_reverseSum=0, real_kf_reverse_sum=0;
                list<KeyFrame*> kf_test_list;
                if(Server::poseGraphGlobal->mv_pushDatabase_num[client_id]==0){
                    for(auto kf_iter=keyFrameList_all.begin(),kf_iter_end=keyFrameList_all.end() ;kf_iter!=kf_iter_end &&(*kf_iter)->is_des_end; ++real_kf_reverse_sum,kf_iter++){
                        kf_test_list.push_back((*kf_iter));
                    }
                }else{
                    for(auto kf_iter=keyFrameList_all.rbegin(),kf_iter_end=keyFrameList_all.rend() ;kf_reverseSum<kf_reverse_num &&kf_iter!=kf_iter_end ;kf_iter++){
                        if((*kf_iter)->is_des_end){
                            kf_test_list.push_front((*kf_iter));
                            real_kf_reverse_sum++;
                        }
                        ++kf_reverseSum;
                    }
                }
                
                //在这个下面判断一下 有没有check_loop
                poseGraph->isAddKF2Database_mutex.lock();
                for(auto iter_start=kf_test_list.begin() ,iter_end=kf_test_list.end();iter_start!=iter_end ;iter_start++){
                    if(*iter_start!=nullptr){//验证个数是否相等 kf是否接收完成
                        assert((*iter_start)->keypoints.size()==(*iter_start)->descriptors.size());
                    }
//                    cout<<"895 size=: "<<(*iter_start)->global_index<<endl;
//                    demoDetector_server<BriefVocabulary, BriefLoopDetector_server, FBrief::TDescriptor> demo_current=Server::poseGraphGlobal->mv_demo_global[client_id];
//                    demo_current.addToDatabase_demo_2((*iter_start)->keypoints, (*iter_start)->descriptors, (*iter_start)->m_bowvec, (*iter_start)->m_featvec);
//                    demo_current.addToDatabase_demo((*iter_start)->keypoints, (*iter_start)->descriptors);
//                    if((*iter_start)->check_loop){
//                        poseGraph->demo_global_in_poseGraph.addToDatabase_demo_2((*iter_start)->keypoints, (*iter_start)->descriptors, (*iter_start)->m_bowvec, (*iter_start)->m_featvec);
//                    }else{
                        poseGraph->demo_global_in_poseGraph.addToDatabase_demo((*iter_start)->keypoints, (*iter_start)->descriptors);//TODO 报错了
                        poseGraph->treeId_kf[poseGraph->kfNum_tree]= (*iter_start)->global_index;
                        poseGraph->kfNum_tree++;
//                    }
                }
                poseGraph->isAddKF2Database_mutex.unlock();
                
                Server::poseGraphGlobal->mv_pushDatabase_num[client_id]+=real_kf_reverse_sum;//实际加的个数
//                cout<<"mv_pushDatabase_num:"<<Server::poseGraphGlobal->mv_pushDatabase_num[client_id]<<endl;
            }
            
//            Server::poseGraphGlobal->poseGraphGlobal_mutex.lock();
            Server::poseGraphGlobal->readWriteLock.readLock();
            //有多个地图存在 ， 并且该地图有新的关键帧插入
            if(Server::poseGraphGlobal->PoseGraphGloabl_map.size()>=2 && kf_reverse_num>0){
//                Server::poseGraphGlobal->poseGraphGlobal_mutex.unlock();
                Server::poseGraphGlobal->readWriteLock.readUnLock();
                
                //                    第三篇论文测试
                Server::poseGraphGlobal->loopClosureRun_global_17_all(client_id);
                
//                if(client_id==mainClientID){
////                    Server::poseGraphGlobal->loopClosureRun_global_11_2_2(client_id);
//                    //应该是论文最终的样子
////                    Server::poseGraphGlobal->loopClosureRun_global_11_3(client_id);
//                    //测试第二篇论文的方案
//                    Server::poseGraphGlobal->loopClosureRun_global_14_1(client_id);
////                    Server::poseGraphGlobal->loopClosureRun_global_13_1(client_id);
//
//                }else{
////                    Server::poseGraphGlobal->loopClosureRun_global_11_2(client_id);//这里是和主地图去做匹配
//                    //应该是论文最终的样子
////                    Server::poseGraphGlobal->loopClosureRun_global_11_3_2(client_id);
//                    //测试第二篇论文的方案
//                    Server::poseGraphGlobal->loopClosureRun_global_14_1_2(client_id);
////                    Server::poseGraphGlobal->loopClosureRun_global_13_1_2(client_id);
//
//                }
            }else{
//                Server::poseGraphGlobal->poseGraphGlobal_mutex.unlock();
                Server::poseGraphGlobal->readWriteLock.readUnLock();
            }
            
            
        }else{
            usleep(2000);
        }
        usleep(30);
    }
}



//这里是判断进行地图内部的回环优化 还是进行多个地图之间的融合优化
//每个代理都有这个线程
void * Server::globalLoopRun(void *args){
    cout<<"globalLoopRun 进来几次"<<endl;
    Client *c = (Client *) args;
    usleep(1000);
    
    PoseGraph* poseGraph;
    VINS* vins;
    int client_id=c->id;
    
    while(client_id==-1){
        usleep(1000);
        client_id=c->id;
    }
    
    cout<<"client_id: "<<client_id<<endl;
    
    
//    Server::poseGraphGlobal->poseGraphGlobal_mutex.lock();
    Server::poseGraphGlobal->readWriteLock.readLock();
    map<int,PoseGraph*>::iterator iter_map = Server::poseGraphGlobal->PoseGraphGloabl_map.find(client_id);
    cout<<"找到了 PoseGraphGloabl_map="<<Server::poseGraphGlobal->PoseGraphGloabl_map.size()<<endl;
    while(iter_map==Server::poseGraphGlobal->PoseGraphGloabl_map.end()){
//        Server::poseGraphGlobal->poseGraphGlobal_mutex.unlock();
        Server::poseGraphGlobal->readWriteLock.readUnLock();
        usleep(1000);
//        Server::poseGraphGlobal->poseGraphGlobal_mutex.lock();
        Server::poseGraphGlobal->readWriteLock.readLock();
        iter_map = Server::poseGraphGlobal->PoseGraphGloabl_map.find(client_id);
//        cout<<"找到了 PoseGraphGloabl_map="<<Server::poseGraphGlobal->PoseGraphGloabl_map.size()<<endl;
    }
//    cout<<"找到了 PoseGraphGloabl_map"<<endl;
    if(iter_map!=Server::poseGraphGlobal->PoseGraphGloabl_map.end())
    {
        poseGraph = iter_map->second;
        vins = poseGraph->vins;

    }else{
        assert(false);
    }
//    Server::poseGraphGlobal->poseGraphGlobal_mutex.unlock();
    Server::poseGraphGlobal->readWriteLock.readUnLock();
//    cout<<"global loop run 066"<<endl;
    while(true){
        
//        if (poseGraph->keyFrameList_global_fusion_mutex.try_lock()) {
//            cout <<"锁定"<< poseGraph->c->id<<endl;
        bool start_wait_optimization=false;
//        if(poseGraph->waitOpti!=-1){
//            start_wait_optimization=true;
//        }
    
        vins->globalOpti_index_mutex.lock();
        if(vins->start_global_optimization || start_wait_optimization)
        {
            int cur_global_index=0;
//                cout<<"测试 是谁在优化 "<<poseGraph->keyFrameList.front()->c->id<<endl;
            if(vins->start_global_optimization){
                vins->start_global_optimization = false;
                
                //这里是因为可能多个全局优化挤一起了，所以找到当时 最小的回环下标和最大的匹配下标
                int toGlobalOptiNum=vins->kf_global_index.size();
                int x_cur=vins->kf_global_index.front(), y_start=vins->start_kf_global_index.front();
                int  start_global_index=y_start;
                cur_global_index=x_cur;
                vins->kf_global_index.pop();
                vins->start_kf_global_index.pop();
                for(int i=1;i<toGlobalOptiNum;i++){
                    x_cur=vins->kf_global_index.front();
                    y_start=vins->start_kf_global_index.front();
                    cur_global_index=cur_global_index>x_cur?cur_global_index:x_cur;
                    start_global_index=start_global_index<y_start?start_global_index:y_start;
                    vins->kf_global_index.pop();
                    vins->start_kf_global_index.pop();
                }
                vins->globalOpti_index_mutex.unlock();
                
                //然后更新最小和最大的回环下标
                poseGraph->loop_index_mutex.lock();
                if(poseGraph->earliest_loop_index==-1 || poseGraph->earliest_loop_index>start_global_index)
                    poseGraph->earliest_loop_index=start_global_index;
                poseGraph->latest_loop_index=poseGraph->latest_loop_index>cur_global_index?poseGraph->latest_loop_index:cur_global_index;
                poseGraph->loop_index_mutex.unlock();
//                cout<<"测试 cur_global_index："<<cur_global_index<<" "<<start_global_index<<" "<<poseGraph->earliest_loop_index<<" "<<poseGraph->latest_loop_index<<endl;
            }else{
                vins->globalOpti_index_mutex.unlock();
                cur_global_index=poseGraph->waitOpti;
                poseGraph->waitNum++;
            }
            
//            poseGraph->hasDetectLoopKfId_mutex.lock();
//            int hasDetectLoopKfId= poseGraph->hasDetectLoopKfId;
//            poseGraph->hasDetectLoopKfId_mutex.unlock();
//            cout<<"检验回环优化的信息: 上一次回环id="<< poseGraph->prePoseGraphOpti<<" , 当前得到回环id="<<cur_global_index<<" , 当前检测过的帧id="<<hasDetectLoopKfId <<",前面等待的回环="<<poseGraph->waitOpti<<endl;
////                判断增强几何一致性验证 是否还有很多没计算。判断离上一次回环优化已经超过10个关键帧了 再想一下
////            if(hasDetectLoopKfId-poseGraph->prePoseGraphOpti<10){
//                if(hasDetectLoopKfId-poseGraph->prePoseGraphOpti<10 && cur_global_index-poseGraph->prePoseGraphOpti<10 && poseGraph->waitNum<3){
//                    poseGraph->waitOpti=cur_global_index;
//                    sleep(5);
//                    continue;
//                }
//                poseGraph->waitNum=0;
////            }
//
//            poseGraph->prePoseGraphOpti=cur_global_index;
//            poseGraph->waitOpti=-1;
            
//                TS(loop_thread);
            poseGraph->readWriteLock_is_fusion_mutex.readLock();
            if(poseGraph->is_fusion){
                poseGraph->readWriteLock_is_fusion_mutex.readUnLock();
                int clientId_send=poseGraph->keyFrameList.front()->c->id;
                //加一个trylock 在函数里面加 防止是两个client互相调用
//                   论文2
//                    Server::poseGraphGlobal->GlobalFuse_8(clientId_send);
//                    论文3
                Server::poseGraphGlobal->GlobalFuse_9(clientId_send);
                
                
            }else{
                poseGraph->readWriteLock_is_fusion_mutex.readUnLock();
                
                std::unique_lock<std::mutex> lock(poseGraph->keyFrameList_global_fusion_mutex, std::defer_lock);
//                    cout<<"内部加锁："<<poseGraph->c->id<<endl;
                if (lock.try_lock()) {
                    poseGraph->optimize4DoFLoopPoseGraph5(cur_global_index, poseGraph->loop_correct_t, poseGraph->loop_correct_r);
                    vins->t_drift = poseGraph->loop_correct_t;
                    vins->r_drift = poseGraph->loop_correct_r;
//                        暂时注释
                    poseGraph->isSendGlobalData=true;//暂时融合优化并没有发送过去
                    lock.unlock();
                    usleep(170);
                } else {
                    cout<<"最终的内部优化中断"<<endl;
                }
                
            }

            
//                TE(loop_thread);
            
//                cout<<"检测到回环 并完成一次全局优化：server "<<getTime()<<endl;
            
            
        }else{
            vins->globalOpti_index_mutex.unlock();
        }
        
        //判断 是不是有融合优化了
        poseGraph->start_global_fuse_opti_mutex.lock();
        if(poseGraph->start_global_fuse_opti){
            poseGraph->start_global_fuse_opti=0;
            poseGraph->start_global_fuse_opti_mutex.unlock();
//                cout<<"poseGraph->start_global_fuse_opti 测试 是谁在优化 "<<poseGraph->keyFrameList.front()->c->id<<endl;
            int clientId_send=poseGraph->keyFrameList.front()->c->id;
//                   论文2
//                Server::poseGraphGlobal->GlobalFuse_8(clientId_send);
//                    论文3
                Server::poseGraphGlobal->GlobalFuse_9(clientId_send);
            
            
        }else{
            poseGraph->start_global_fuse_opti_mutex.unlock();
        }

//            poseGraph->keyFrameList_global_fusion_mutex.unlock();
//            cout <<"解除锁定"<< poseGraph->c->id<<endl;
//        } else {
//            // 不能获取锁以修改 'job_shared'
//            // 说明此时正在参与优化 即将的优化 休眠3秒钟 TODO 后面考虑是不是会存在此次优化 其实已经参与到前面的优化里面了， 要注意这方面的数据安全
//            usleep(3000);
//        }
        

        usleep(30);
    }
}

