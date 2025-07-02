//
//  featureSubMap.m
//  dbow_test
//
//  Created by 张剑华 on 2020/8/15.
//  Copyright © 2020 zx. All rights reserved.
//
#include "FeatureSubMap.hpp"



FeatureSubMap::FeatureSubMap()
{
//    keyPoint.clear();
//    descriptor.clear();
//    featureID.clear();
//    ric = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r));
    loop_mltra_num=0;
}

void FeatureSubMap::operator=(const FeatureSubMap& old){
    ;
}
void FeatureSubMap::perfectFuse_subMap(FeatureSubMap * curSubMap){
   
    cout<<"perfectFuse_subMap加锁"<<endl;
    bool is_old=true,is_cur=true;
    is_old=mMutexSubMapList.try_lock();
    is_cur=curSubMap->mMutexSubMapList.try_lock();
    while(!(is_old && is_cur)){
        if(is_old){
            mMutexSubMapList.unlock();
        }if(is_cur){
            curSubMap->mMutexSubMapList.unlock();
        }
        usleep(10);
        is_old=mMutexSubMapList.try_lock();
        is_cur=curSubMap->mMutexSubMapList.try_lock();
    }
    
    //l1 和 l1合并
    ml_keyframe.splice(ml_keyframe.end(), curSubMap->ml_keyframe);
    ml_kfId.insert(ml_kfId.end(),curSubMap->ml_kfId.begin(),curSubMap->ml_kfId.end());
//    l2合并
    ml2_kf.splice(ml2_kf.end(), curSubMap->ml2_kf);
    ml2_kfId.insert(ml2_kfId.end(),curSubMap->ml2_kfId.begin(),curSubMap->ml2_kfId.end());
//    l1非代表帧合并
    noRepresent_kfId.insert(noRepresent_kfId.end(),curSubMap->noRepresent_kfId.begin(),curSubMap->noRepresent_kfId.end());
    noRepresent_kfHeader.insert(noRepresent_kfHeader.end(),curSubMap->noRepresent_kfHeader.begin(),curSubMap->noRepresent_kfHeader.end());
//l2非代表帧合并
    noRepresent_kfId_l2.insert(noRepresent_kfId_l2.end(),curSubMap->noRepresent_kfId_l2.begin(),curSubMap->noRepresent_kfId_l2.end());
    noRepresent_kfHeader_l2.insert(noRepresent_kfHeader_l2.end(),curSubMap->noRepresent_kfHeader_l2.begin(),curSubMap->noRepresent_kfHeader_l2.end());
//    global_index更新
    global_index_moreEarly.push_back(global_index);
    global_index_moreEarly.insert(global_index_moreEarly.end(), curSubMap->global_index_moreEarly.begin(),curSubMap->global_index_moreEarly.end());
    global_index=curSubMap->global_index;
//    global_agent_id更新
    global_agent_id_moreEarly.push_back(global_agent_id);
    global_agent_id_moreEarly.insert(global_agent_id_moreEarly.end(), curSubMap->global_agent_id_moreEarly.begin(),curSubMap->global_agent_id_moreEarly.end());
    global_agent_id=curSubMap->global_agent_id;
    
    center_kf=curSubMap->center_kf;
    loop_mltra_num+=curSubMap->loop_mltra_num;
    curSubMap->mMutexSubMapList.unlock();
    mMutexSubMapList.unlock();
    
    cout<<"perfectFuse_subMap解锁"<<endl;
}

void FeatureSubMap::perfectFuse_subMap2(FeatureSubMap * curSubMap){
   
    cout<<"perfectFuse_subMap加锁"<<endl;
    bool is_old=true,is_cur=true;
    is_old=mMutexSubMapList.try_lock();
    is_cur=curSubMap->mMutexSubMapList.try_lock();
    while(!(is_old && is_cur)){
        if(is_old){
            mMutexSubMapList.unlock();
        }if(is_cur){
            curSubMap->mMutexSubMapList.unlock();
        }
        usleep(10);
        is_old=mMutexSubMapList.try_lock();
        is_cur=curSubMap->mMutexSubMapList.try_lock();
    }
    
    //l1 和 l1合并
//    ml_keyframe.splice(ml_keyframe.end(), curSubMap->ml_keyframe);
    
//    cout<<"ml_keyframe=";
//    for(auto i=ml_keyframe.begin();i!=ml_keyframe.end();i++){
//        cout<<(*i)->global_index<<" , ";
//
//    }
//   cout<<endl<<"curSubMap->ml_keyframe=";
//    for(auto i=curSubMap->ml_keyframe.begin();i!=curSubMap->ml_keyframe.end();i++){
//       cout<<(*i)->global_index<<" , ";
//
//   }
//  cout<<endl;
              
//    ml_keyframe.insert(ml_keyframe.end(), curSubMap->ml_keyframe.begin(),curSubMap->ml_keyframe.end());
    list<KeyFrame*> v1(curSubMap->ml_keyframe);
    v1.insert(v1.end(), ml_keyframe.begin(),ml_keyframe.end());
    ml_keyframe.assign(v1.begin(), v1.end());
//    cout<<"更新后ml_keyframe=";
//    for(auto i=ml_keyframe.begin();i!=ml_keyframe.end();i++){
//        cout<<(*i)->global_index<<" , ";
//
//    }
//   cout<<endl<<"curSubMap->ml_keyframe=";
//    for(auto i=curSubMap->ml_keyframe.begin();i!=curSubMap->ml_keyframe.end();i++){
//       cout<<(*i)->global_index<<" , ";
//
//   }
//  cout<<endl;
    
    vector<int> ml_kfId_v1(curSubMap->ml_kfId);
    ml_kfId_v1.insert(ml_kfId_v1.end(), ml_kfId.begin(),ml_kfId.end());
    ml_kfId.assign(ml_kfId_v1.begin(), ml_kfId_v1.end());
    
//    ml_kfId.insert(ml_kfId.end(),curSubMap->ml_kfId.begin(),curSubMap->ml_kfId.end());
//    l2合并
//    ml2_kf.splice(ml2_kf.end(), curSubMap->ml2_kf);
    
    list<KeyFrame*> ml2_kf_v1(curSubMap->ml2_kf);
    ml2_kf_v1.insert(ml2_kf_v1.end(), ml2_kf.begin(),ml2_kf.end());
    ml2_kf.assign(ml2_kf_v1.begin(), ml2_kf_v1.end());
//    ml2_kf.insert(ml2_kf.end(), curSubMap->ml2_kf.begin(),  curSubMap->ml2_kf.end());
    
    vector<int> ml2_kfId_v1(curSubMap->ml2_kfId);
    ml2_kfId_v1.insert(ml2_kfId_v1.end(), ml2_kfId.begin(),ml2_kfId.end());
    ml2_kfId.assign(ml2_kfId_v1.begin(), ml2_kfId_v1.end());
    
    
//    ml2_kfId.insert(ml2_kfId.end(),curSubMap->ml2_kfId.begin(),curSubMap->ml2_kfId.end());
//    l1非代表帧合并
    
    vector<int> noRepresent_kfId_v1(curSubMap->noRepresent_kfId);
    noRepresent_kfId_v1.insert(noRepresent_kfId_v1.end(), noRepresent_kfId.begin(),noRepresent_kfId.end());
    noRepresent_kfId.assign(noRepresent_kfId_v1.begin(), noRepresent_kfId_v1.end());
//    noRepresent_kfId.insert(noRepresent_kfId.end(),curSubMap->noRepresent_kfId.begin(),curSubMap->noRepresent_kfId.end());
    
    vector<double> noRepresent_kfHeader_v1(curSubMap->noRepresent_kfHeader);
    noRepresent_kfHeader_v1.insert(noRepresent_kfHeader_v1.end(), noRepresent_kfHeader.begin(),noRepresent_kfHeader.end());
    noRepresent_kfHeader.assign(noRepresent_kfHeader_v1.begin(), noRepresent_kfHeader_v1.end());
//    noRepresent_kfHeader.insert(noRepresent_kfHeader.end(),curSubMap->noRepresent_kfHeader.begin(),curSubMap->noRepresent_kfHeader.end());
//l2非代表帧合并
    
    vector<int> noRepresent_kfId_l2_v1(curSubMap->noRepresent_kfId_l2);
    noRepresent_kfId_l2_v1.insert(noRepresent_kfId_l2_v1.end(), noRepresent_kfId_l2.begin(),noRepresent_kfId_l2.end());
    noRepresent_kfId_l2.assign(noRepresent_kfId_l2_v1.begin(), noRepresent_kfId_l2_v1.end());
//    noRepresent_kfId_l2.insert(noRepresent_kfId_l2.end(),curSubMap->noRepresent_kfId_l2.begin(),curSubMap->noRepresent_kfId_l2.end());
    
    vector<double> noRepresent_kfHeader_l2_v1(curSubMap->noRepresent_kfHeader_l2);
    noRepresent_kfHeader_l2_v1.insert(noRepresent_kfHeader_l2_v1.end(), noRepresent_kfHeader_l2.begin(),noRepresent_kfHeader_l2.end());
    noRepresent_kfHeader_l2.assign(noRepresent_kfHeader_l2_v1.begin(), noRepresent_kfHeader_l2_v1.end());
//    noRepresent_kfHeader_l2.insert(noRepresent_kfHeader_l2.end(),curSubMap->noRepresent_kfHeader_l2.begin(),curSubMap->noRepresent_kfHeader_l2.end());
//    global_index更新
    
    vector<int> global_index_moreEarly_v1(curSubMap->global_index_moreEarly);
    global_index_moreEarly_v1.push_back(curSubMap->global_index);
    global_index_moreEarly_v1.insert(global_index_moreEarly_v1.end(), global_index_moreEarly.begin(),global_index_moreEarly.end());
    global_index_moreEarly.assign(global_index_moreEarly_v1.begin(), global_index_moreEarly_v1.end());
//    global_index_moreEarly.insert(global_index_moreEarly.end(), curSubMap->global_index_moreEarly.begin(),curSubMap->global_index_moreEarly.end());
//    global_index_moreEarly.push_back(curSubMap->global_index);
    
//    global_index=curSubMap->global_index;
//    global_agent_id更新
    
    vector<int> global_agent_id_moreEarly_v1(curSubMap->global_agent_id_moreEarly);
    global_agent_id_moreEarly_v1.push_back(curSubMap->global_agent_id);
    global_agent_id_moreEarly_v1.insert(global_agent_id_moreEarly_v1.end(), global_agent_id_moreEarly.begin(),global_agent_id_moreEarly.end());
    global_agent_id_moreEarly.assign(global_agent_id_moreEarly_v1.begin(), global_agent_id_moreEarly_v1.end());
//    global_agent_id_moreEarly.insert(global_agent_id_moreEarly.end(), curSubMap->global_agent_id_moreEarly.begin(),curSubMap->global_agent_id_moreEarly.end());
//    global_agent_id_moreEarly.push_back(curSubMap->global_agent_id);
    
//    以防两个子地图 之前都融合过其它地图的同一子地图
    vector<int> related_otherAgentSubMapIndex_v1(curSubMap->related_otherAgentSubMapIndex);
    vector<int> related_otherAgentId_v1(curSubMap->related_otherAgentId);
    for(int i=0,j=related_otherAgentSubMapIndex.size();i<j;i++){
        int single_related_otherAgentSubMapIndex=related_otherAgentSubMapIndex[i];
        int single_related_otherAgentId=related_otherAgentId[i];
        bool isFind=false;
        for(int a=0,b=curSubMap->related_otherAgentSubMapIndex.size();a<b;a++){
            int single_related_otherAgentSubMapIndex_cur=curSubMap->related_otherAgentSubMapIndex[a];
            int single_related_otherAgentId_cur=curSubMap->related_otherAgentId[a];
            if(single_related_otherAgentSubMapIndex==single_related_otherAgentSubMapIndex_cur && single_related_otherAgentId==single_related_otherAgentId_cur){
                isFind=true;
                break;
            }
        }
        if(!isFind){
            related_otherAgentSubMapIndex_v1.push_back(single_related_otherAgentSubMapIndex);
            related_otherAgentId_v1.push_back(single_related_otherAgentId);
        }
    }
//    related_otherAgentSubMapIndex_v1.insert(related_otherAgentSubMapIndex_v1.end(), related_otherAgentSubMapIndex.begin(),related_otherAgentSubMapIndex.end());
    related_otherAgentSubMapIndex.assign(related_otherAgentSubMapIndex_v1.begin(), related_otherAgentSubMapIndex_v1.end());
//    related_otherAgentId_v1.insert(related_otherAgentId_v1.end(), related_otherAgentId.begin(),related_otherAgentId.end());
    related_otherAgentId.assign(related_otherAgentId_v1.begin(), related_otherAgentId_v1.end());
    
    
//    global_agent_id=curSubMap->global_agent_id;
//    center_kf=curSubMap->center_kf;
    loop_mltra_num+=curSubMap->loop_mltra_num;
    curSubMap->mMutexSubMapList.unlock();
    mMutexSubMapList.unlock();
    
    cout<<"perfectFuse_subMap解锁"<<endl;
    
    assert(ml_kfId.size()==ml_keyframe.size());
    assert(ml2_kfId.size()==ml2_kf.size());
}

bool FeatureSubMap::isBigView(int threshold_bigView){
    mMutexSubMapList.lock();
    int l1_len=ml_kfId.size();
    int l2_len=ml2_kfId.size();
    int view_num=l1_len+l2_len-loop_mltra_num;
    mMutexSubMapList.unlock();
    if(view_num>threshold_bigView){
        return  true;
    }else{
        return false;
    }
}

bool FeatureSubMap::isBigView_noMutex(int threshold_bigView){
//    mMutexSubMapList.lock();
    int l1_len=ml_kfId.size();
    int l2_len=ml2_kfId.size();
    int view_num=l1_len+l2_len-loop_mltra_num;
//    mMutexSubMapList.unlock();
    if(view_num>threshold_bigView){
        return  true;
    }else{
        return false;
    }
}
//void FeatureSubMap::replaceSelf(FeatureSubMap *oldSubMap){
//
//}

FeatureMap::FeatureMap(){
    ;
}

int FeatureMap::getIndex_fromAgentId(int curAgent_id){
//mMutex_ml_submap 加锁 TODO
    readWriteLock_agent_id.readLock();
    for(int i=0,j=agent_id.size();i<j;i++){
        if(agent_id[i]==curAgent_id){
            readWriteLock_agent_id.readUnLock();
            return i;
        }
    }
    assert(false);
    readWriteLock_agent_id.readUnLock();
    return -1;
}

//暂时没用 锁加的也不对
FeatureSubMap* FeatureMap::getSubMapByIndex(int index,int curAgent_id){
    int curAgent_index=getIndex_fromAgentId(curAgent_id);
    
    mMutex_ml_submap[curAgent_index]->lock();
    list<FeatureSubMap*> cur_ml_submap=ml_submap[curAgent_index];
    list<FeatureSubMap*>::iterator it = cur_ml_submap.begin();
    for (; it != cur_ml_submap.end(); it++)
    {
        {
            unique_lock<mutex> lock((*it)->mMutexSubMapList);
            if((*it)->global_index == index){
                FeatureSubMap* curFeatureSubMap=*it;
                mMutex_ml_submap[curAgent_index]->unlock();
                return curFeatureSubMap;
            }
        }
    }
    mMutex_ml_submap[curAgent_index]->unlock();
    return NULL;
}

//单个代理内的子地图划分与融合
void FeatureMap::allocateSubMap(KeyFrame* kf_cur,int old_index,int kfNum_tree,vector<int> old_vector){
//    TS(allocateSubMap);
    int cur_agent_id=kf_cur->c->getId();
    int cur_agent_index=-1;
    
    
    
    if(kf_cur->global_index==0){
        FeatureSubMap* firstSubMap=new FeatureSubMap();
        Vector3d first_t_w_i;
        Matrix3d first_r_w_i;
        kf_cur->getOriginPose(first_t_w_i, first_r_w_i);
        firstSubMap->ml_keyframe.push_back(kf_cur);
        firstSubMap->ml_kfId.push_back(kf_cur->global_index);
        firstSubMap->global_index=0;
        firstSubMap->center_kf=kf_cur;
        firstSubMap->global_agent_id=cur_agent_id;
        

        readWriteLock_agent_id.writeLock();
        agent_id.push_back(cur_agent_id);
        readWriteLock_agent_id.writeUnLock();
        cur_agent_index=getIndex_fromAgentId(cur_agent_id);
        
        
        shared_ptr<std::mutex> md=make_shared<std::mutex>();
        mMutex_ml_submap.push_back(md);
        
        mMutex_ml_submap[cur_agent_index]->lock();
        ml_submap.push_back(list<FeatureSubMap*>{firstSubMap});
        kfAttr_ForAMap.push_back(vector<int>(1));
        kfAttr_SubMapId.push_back(vector<int>{firstSubMap->global_index});
        treeId_kf.push_back(map<int,int> { std::make_pair(kf_cur->global_index,kfNum_tree)});//kfNum_tree此时为0
//        RelatedSubMap* relatedSubMap=new RelatedSubMap();
//        relatedSubMap_forAgent.push_back(relatedSubMap);
        relatedSubMap_forAgent.push_back(vector<int>{firstSubMap->global_index});
        delteSubMap_num.push_back(0);
        mMutex_ml_submap[cur_agent_index]->unlock();
    }else{
        cur_agent_index=getIndex_fromAgentId(cur_agent_id);
        
        if(kf_cur!=nullptr){
            bool isSimlar=false;
            bool isSimlar_l2=false;
            int isSimlar_index=-1;
            int isSimlar_l2_index=-1;
            
            mMutex_ml_submap[cur_agent_index]->lock();
            treeId_kf[cur_agent_index].insert(std::make_pair(kf_cur->global_index,kfNum_tree));
            FeatureSubMap *curSubMap=ml_submap[cur_agent_index].back();
            for(int i=0,j=old_vector.size();i<j;i++){
//                    只和代表帧做对比
                int kf_attr_l1l2no=kfAttr_ForAMap[cur_agent_index][old_vector[i]];
                if(kf_attr_l1l2no==1 || kf_attr_l1l2no==2){
                    for(int a=0,b=curSubMap->ml_kfId.size();a<b;a++){
                        if(old_vector[i]==treeId_kf[cur_agent_index][curSubMap->ml_kfId[a]]){
                            isSimlar=true;
                            isSimlar_index=a;
                            break;
                        }
                    }
//                    if(isSimlar){
//                        break;
//                    }
                    for(int a=0,b=curSubMap->ml2_kfId.size();a<b;a++){
                        if(old_vector[i]==treeId_kf[cur_agent_index][curSubMap->ml2_kfId[a]]){
                            isSimlar_l2=true;
                            isSimlar_l2_index=a;
                            break;
                        }
                    }
                }
                if(isSimlar || isSimlar_l2){
                    break;
                }
            }
            mMutex_ml_submap[cur_agent_index]->unlock();
            
            curSubMap->mMutexSubMapList.lock();
            KeyFrame *curSubMap_l1kf=curSubMap->center_kf;
            bool isFirstKF=false;
            if(curSubMap->ml_keyframe.size()==1){
                isFirstKF=true;
            }
            curSubMap->mMutexSubMapList.unlock();
            
//                计算当前帧 和 curSubMap_l1kf的距离
            Vector3d old_t_w_i;
            Matrix3d old_r_w_i;
            curSubMap_l1kf->getOriginPose(old_t_w_i, old_r_w_i);
            Vector3d cur_t_w_i;
            Matrix3d cur_r_w_i;
            kf_cur->getOriginPose(cur_t_w_i, cur_r_w_i);
            double distance_cur_l1kf=(old_t_w_i-cur_t_w_i).norm();
//            cout<<"distance_cur_l1kf="<< distance_cur_l1kf<<endl;
            
    //                算相似度 后面改一下 最多返回多少个 相似度过关的
            double threshold_distance_t=3,threshold_distance_t_l2=5;//以m为单位
            int threshold_bigView=10;//子地图的视角 丰富不

            bool loop_succ= (old_index==-1 ? false : true);
//                    近 比较代表帧 在不在old_vector里面，在说明相似度高。 不在说明 低
       
//                    这里是去找 是否存在相似度高，但是并没有考虑 它可能和其它地图内的帧更相似
//                   TODO 距离近的情况下，和其它子地图更相似，说明这个子地图 应该融合到其它子地图 （但是如果两个子地图时序上距离很近，说明新的子地图中心不太对）

            cout<<"基础准备工作"<<endl;
            FeatureSubMap *old_reCurSubMap;
            bool isL1=false;
            int oldSubMap_id=-1;
            int oldSubMap_agent_id=-1;
            bool isSameArea=false;
//            list<FeatureSubMap *>::iterator iter_update;
            if(loop_succ){
//                cout<<"此时此刻，发生回环，应该要做子地图融合的"<<endl;
                mMutex_ml_submap[cur_agent_index]->lock();
                oldSubMap_id=kfAttr_SubMapId[cur_agent_index][old_index];
                int levelKf=kfAttr_ForAMap[cur_agent_index][old_index];
                vector<int> relatedSubMap_curAgent=relatedSubMap_forAgent[cur_agent_index];
                mMutex_ml_submap[cur_agent_index]->unlock();
                
//                找到了帧最原始属于的submap 现在找融合到哪个map里了
                vector<int> updateRelatedIndex;
                while(relatedSubMap_curAgent[oldSubMap_id]!=oldSubMap_id){
                    updateRelatedIndex.push_back(oldSubMap_id);
                    cout<<oldSubMap_id<<" , ";
                    oldSubMap_id=relatedSubMap_curAgent[oldSubMap_id];
                    
                }
                cout<<endl;
                for(int hh=0,jj=updateRelatedIndex.size();hh<jj;hh++){
                    mMutex_ml_submap[cur_agent_index]->lock();
                    relatedSubMap_forAgent[cur_agent_index][updateRelatedIndex[hh]]=oldSubMap_id;
                    mMutex_ml_submap[cur_agent_index]->unlock();
                }
                
                //               判断是否属于两个不同的子地图，即是否融合过了
                if(oldSubMap_id!=curSubMap->global_index){
                    for(auto i=curSubMap->relatedSubMapIndex.begin(),j=curSubMap->relatedSubMapIndex.end();i!=j;i++){
                        if(oldSubMap_id==(*i)){
                            isSameArea=true;
                            break;
                        }
                        
                    }
                }else{
                    isSameArea=true;
                }
                
                cout<<"老帧所属子地图信息"<<oldSubMap_id<<" , "<<levelKf<<" , "<<old_index<<endl;
                if(!isSameArea){
                    int index=0;
                    int real_oldSubMap_id=oldSubMap_id-delteSubMap_num[cur_agent_index];
                    for(auto iter=ml_submap[cur_agent_index].begin(),iter_end=ml_submap[cur_agent_index].end();iter!=iter_end;iter++){
                        
                        if(index<real_oldSubMap_id){
                            index++;
                            continue;
                        }
                        if(oldSubMap_id==(*iter)->global_index){
                            old_reCurSubMap=*iter;
                            oldSubMap_agent_id=(*iter)->global_agent_id;
                            
                            cout<<"oldSubMap="<<oldSubMap_id<<" , ";
                            for(int hh=0,j=(*iter)->global_index_moreEarly.size();hh<j;hh++){
                                cout<<(*iter)->global_index_moreEarly[hh]<<" , ";
                            }
                            cout<<endl;
                            if(levelKf==1){
        //                                说明和对方l1层匹配上了 两个子地图应该融合
                                isL1=true;
                            }else if(levelKf==2){
        //                                说明和对方l2层匹配上了 只需要记住关联关系
                                ;
                            }else if(levelKf==0){
        //                                说明和非代表帧匹配上了 l1层
    //                            为了做优化 需要把这一帧 插入到代表帧层面（强制）
                                isL1=true;
                            }else if(levelKf==3){
        //                                说明和非代表帧匹配上了 l2层
    //                            为了做优化 需要把这一帧 插入到代表帧层面（强制）
                                ;
                            }
                            break;
                        }
                        
                    }
                }

            }
            
            cout<<"loop_succ="<<loop_succ<<" , "<<isL1<<" , "<<isSimlar<<" , "<<isSimlar_l2<<" , "<< distance_cur_l1kf<<" , "<<kf_cur->global_index<<endl;
//            cout<<"未做处理前 当前子地图"<<curSubMap->global_index<<" , "<<ml_submap.size()<<endl;
    //                短距离内 l1层
//            old_reCurSubMap=*iter_update;
            
//            old_reCurSubMap->mMutexSubMapList.lock();
            if(distance_cur_l1kf<threshold_distance_t){
                if(loop_succ && !isSameArea){
                    //两个子地图应该融合 把 && oldSubMap_agent_id!=curSubMap->global_agent_id
                    if(oldSubMap_id!=curSubMap->global_index || oldSubMap_agent_id!=curSubMap->global_agent_id){
                        cout<<"代理是否重复融合："<<oldSubMap_id<<" , "<< oldSubMap_agent_id<<" , "<<curSubMap->global_index  <<" , "<< curSubMap->global_agent_id<<endl;
    //                            这是两个子地图
                        if(isL1){
    //                            说明都是L1层的 因为l2层有个l1层的细节扩展，但是和l1层更匹配 说明两个子地图应该完美融合
    //                            第一帧 以当前子地图的为主，用来比较距离（因为当前附近误差较小）
    //                                进行融合之前 要先检测当前地图是否已经发生过融合


                            for(int a=0,b=old_reCurSubMap->global_index_moreEarly.size();a<b;a++){
                                cout<<old_reCurSubMap->global_index_moreEarly[a]<<" , "<<old_reCurSubMap->global_agent_id_moreEarly[a]<<" , ";
                            }
                            cout<<endl;
//                            cout<<"测试地址对不对 改之前"<<recurSubMap<<" , "<<old_reCurSubMap<<" , "<<curSubMap<<endl;
                            
//                            暂时注释0621
                            curSubMap->perfectFuse_subMap2(old_reCurSubMap);
//                            recurSubMap->mMutexSubMapList.lock();
//                            (*iter_update)=curSubMap;
//                            recurSubMap->mMutexSubMapList.unlock();
////                            delete recurSubMap;  list<FeatureSubMap*>
                            ///
                            cout<<"测试地址对不对 改之后"<<old_reCurSubMap<<" , "<<curSubMap<<endl;
                            cout<<old_reCurSubMap->global_index<<" , "<<curSubMap->global_index<<endl;
                            
                            int index=0;
                            int real_oldSubMap_id=oldSubMap_id-delteSubMap_num[cur_agent_index];
                            for(auto iter=ml_submap[cur_agent_index].begin(),iter_end=ml_submap[cur_agent_index].end();iter!=iter_end;iter++){
                                
                                if(index<real_oldSubMap_id){
                                    index++;
                                    continue;
                                }
                                if(oldSubMap_id==(*iter)->global_index){
                                    cout<<"测试是否删除成功："<<ml_submap[cur_agent_index].size()<<endl;
                                    ml_submap[cur_agent_index].erase(iter);
                                    relatedSubMap_forAgent[cur_agent_index][old_reCurSubMap->global_index]=curSubMap->global_index;
                                    delteSubMap_num[cur_agent_index]++;
                                    
                                    cout<<ml_submap[cur_agent_index].size()<<endl;
//                                    old_reCurSubMap->mMutexSubMapList.lock();
//                                    (*iter)=curSubMap;
//                                    old_reCurSubMap->mMutexSubMapList.unlock();
//                                    cout<<"测试老帧有没有改对"<<(*iter)->global_index<<endl;
                                    break;
                                }
                            }
                            
                            
                            
//                            mMutex_ml_submap[cur_agent_index]->lock();
//                            oldSubMap_id=kfAttr_SubMapId[cur_agent_index][old_index];
//                            int levelKf=kfAttr_ForAMap[cur_agent_index][old_index];
//                            mMutex_ml_submap[cur_agent_index]->unlock();
                            
                            
                            
                                    
//                            old_reCurSubMap->perfectFuse_subMap2(curSubMap);
//                            curSubMap=old_reCurSubMap;
//                            mMutex_ml_submap[cur_agent_index]->lock();
//                            ml_submap[cur_agent_index].pop_back();
//                            ml_submap[cur_agent_index].push_back(curSubMap);
//                            mMutex_ml_submap[cur_agent_index]->unlock();
                            
//                            cout<<"子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            
                            
                            
                        }else{
    //                            说明一个是l1层 一个l2层 此时此刻的距离不准，看当前子地图的数量和老子地图的数量（代表帧的数量 l1+l2 说明视角大不大），大于某个阈值 只记录两个子地图的关联关系（两个人的视角 都比较全了，才有一次回环，说明比较偏），否则 一个人视角大一个视角小 （因为时间上差的比较远了，尽可能融合）融合
                            if(old_reCurSubMap->isBigView(threshold_bigView) && curSubMap->isBigView(threshold_bigView)){
                                //记录强关联关系
                                old_reCurSubMap->relatedSubMapIndex.push_back(curSubMap->global_index);
                                curSubMap->relatedSubMapIndex.push_back(old_reCurSubMap->global_index);
//                                cout<<"强关联了"<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            }else{
                                
//
                                
                                //                            暂时注释0621
                                curSubMap->perfectFuse_subMap2(old_reCurSubMap);
                                
////                                delete recurSubMap;
                               
                                cout<<"测试地址对不对 改之后"<<old_reCurSubMap<<" , "<<curSubMap<<endl;
                                cout<<old_reCurSubMap->global_index<<" , "<<curSubMap->global_index<<endl;
                                
                                int index=0;
                                int real_oldSubMap_id=oldSubMap_id-delteSubMap_num[cur_agent_index];
                                for(auto iter=ml_submap[cur_agent_index].begin(),iter_end=ml_submap[cur_agent_index].end();iter!=iter_end;iter++){
                                    
                                    if(index<real_oldSubMap_id){
                                        index++;
                                        continue;
                                    }
                                    if(oldSubMap_id==(*iter)->global_index){
                                        cout<<"测试是否删除成功："<<ml_submap[cur_agent_index].size()<<endl;
                                        ml_submap[cur_agent_index].erase(iter);
                                        relatedSubMap_forAgent[cur_agent_index][old_reCurSubMap->global_index]=curSubMap->global_index;
                                        delteSubMap_num[cur_agent_index]++;
                                        cout<<ml_submap[cur_agent_index].size()<<endl;
//                                        old_reCurSubMap->mMutexSubMapList.lock();
//                                        (*iter)=curSubMap;
//                                        old_reCurSubMap->mMutexSubMapList.unlock();
//                                        cout<<"测试老帧有没有改对"<<(*iter)->global_index<<endl;
                                        break;
                                    }
                                }
                                
                                
                                
                           
//                                old_reCurSubMap->perfectFuse_subMap2(curSubMap);
//                                curSubMap=old_reCurSubMap;
//                                mMutex_ml_submap[cur_agent_index]->lock();
//                                ml_submap[cur_agent_index].pop_back();
//                                ml_submap[cur_agent_index].push_back(curSubMap);
//                                mMutex_ml_submap[cur_agent_index]->unlock();
                                
//                                cout<<"子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            }
                        }
                        
                    }else{
                        cout<<"本来就是一个地图"<<endl;
                    }
//                            由于当前帧是回环帧 所以应该加入所在的层
                    if(isSimlar || isSimlar_l2){
                        curSubMap->loop_mltra_num++;
                    }
                    mMutex_ml_submap[cur_agent_index]->lock();
                    kfAttr_ForAMap[cur_agent_index].push_back(1);
                    kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                    mMutex_ml_submap[cur_agent_index]->unlock();
                    
                    curSubMap->mMutexSubMapList.lock();
                    curSubMap->ml_keyframe.push_back(kf_cur);
                    curSubMap->ml_kfId.push_back(kf_cur->global_index);
                    curSubMap->mMutexSubMapList.unlock();
//                    cout<<"处理后 当前子地图"<<curSubMap->global_index<<" , "<<curSubMap->global_index_moreEarly.size()<<endl;
//                    for(int x=0,y=curSubMap->global_index_moreEarly.size();x<y;x++){
//                        cout<<"该子地图还包括："<<curSubMap->global_index_moreEarly[x]<<" , ";
//                    }
//                    cout<<endl;
                }else{
    //                        没有子地图需要融合 仅仅是加入当前子地图
                    if(isSimlar){
    //                            检查是否替换和自己相似的帧（如果共视的点数量大很多，那么替换。否则，考虑前面的更精确 累积误差小 不进行替换）
                        list<KeyFrame*>::iterator it = curSubMap->ml_keyframe.begin();
                        //借助 advance() 函数将 it 迭代器前进 2 个位置
                        advance(it, isSimlar_index);
                        KeyFrame* kf_old= *it;
                        assert(kf_old->global_index==curSubMap->ml_kfId[isSimlar_index]);
                        
                        if(kf_old->is_looped || kf_old->is_get_loop_info || kf_old->has_global_loop || kf_old->is_global_looped){
                            //无论如何都不替换
    //                        加入作为非代表
                            curSubMap->noRepresent_kfId.push_back(kf_cur->global_index);
                            curSubMap->noRepresent_kfHeader.push_back(kf_cur->header);
                            kfAttr_ForAMap[cur_agent_index].push_back(0);
                            kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                        }else{
                            int sum_old=0,sum_cur=0;
                            for(int x=0,y=kf_old->mvOrderedWeights.size();x<y;x++){
                                sum_old+=kf_old->mvOrderedWeights[x];
                            }
                            for(int x=0,y=kf_cur->mvOrderedWeights.size();x<y;x++){
                                sum_cur+=kf_cur->mvOrderedWeights[x];
                            }
    //                        暂时注释
                            cout<<"替换="<<sum_old<<" , "<<kf_old->mvOrderedWeights.size()<<" , "<<sum_cur*0.8<<" , "<<kf_cur->mvOrderedWeights.size()<<endl;
                            if((float)sum_old<(sum_cur*0.8)){
                                cout<<"真的替换了"<<endl;
    //                            int iter_index=0;
    //                            cout<<"更新前：";
    //                            for(auto iter_ml_start=curSubMap->ml_keyframe.begin(),iter_ml_end=curSubMap->ml_keyframe.end();iter_ml_start!=iter_ml_end;iter_ml_start++){
    //                                cout<<(*iter_ml_start)->global_index<<" , "<<curSubMap->ml_kfId[iter_index]<<endl;
    //                                iter_index++;
    //
    //                            }

        //                                当前帧替换老的关键帧 成为代表帧，老的关键帧成为非代表帧
                                mMutex_ml_submap[cur_agent_index]->lock();
                                int real_oldkf_id=treeId_kf[cur_agent_index][kf_old->global_index];
                                kfAttr_ForAMap[cur_agent_index][real_oldkf_id]=0;
                                kfAttr_SubMapId[cur_agent_index][real_oldkf_id]=(curSubMap->global_index);
                                kfAttr_ForAMap[cur_agent_index].push_back(1);
                                kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                                mMutex_ml_submap[cur_agent_index]->unlock();
                                
        //                        加入作为非代表
                                curSubMap->noRepresent_kfId.push_back(kf_old->global_index);
                                curSubMap->noRepresent_kfHeader.push_back(kf_old->header);
    //                            kfAttr_ForAMap[cur_agent_index][kf_old->global_index]=0;
    //                            kfAttr_SubMapId[cur_agent_index][kf_old->global_index]=(curSubMap->global_index);
                                curSubMap->ml_keyframe.push_back(kf_cur);
                                curSubMap->ml_kfId.push_back(kf_cur->global_index);//加入loopclosure的顺序
                                

        //                                验证这么操作对不对
                                curSubMap->ml_keyframe.erase(it);
                                for(int h=isSimlar_index,g=curSubMap->ml_kfId.size();h<(g-1);h++){
                                    curSubMap->ml_kfId[h]=curSubMap->ml_kfId[h+1];
                                }
                                curSubMap->ml_kfId.resize(curSubMap->ml_kfId.size()-1);
        //                                curSubMap->ml_kfId.erase(curSubMap->ml_kfId.rbegin() + (curSubMap->ml_kfId.size()-isSimlar_index));

    //                            cout<<"更新后：";
    //                            iter_index=0;
    //                            for(auto iter_ml_start=curSubMap->ml_keyframe.begin(),iter_ml_end=curSubMap->ml_keyframe.end();iter_ml_start!=iter_ml_end;iter_ml_start++){
    //                                cout<<(*iter_ml_start)->global_index<<" , "<<curSubMap->ml_kfId[iter_index]<<endl;
    //                                iter_index++;
    //                            }
                            }
                            else
                            {
        //                        加入作为非代表
                                curSubMap->noRepresent_kfId.push_back(kf_cur->global_index);
                                curSubMap->noRepresent_kfHeader.push_back(kf_cur->header);
                                
                                mMutex_ml_submap[cur_agent_index]->lock();
                                kfAttr_ForAMap[cur_agent_index].push_back(0);
                                kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                                mMutex_ml_submap[cur_agent_index]->unlock();
                            }
                        }
                    }else{
    //                        加入作为代表
                        if(isFirstKF){
                            isFirstKF=false;
    //                                assert(curSubMap->ml_keyframe.size()==1);//此时 只有1个代表帧在子地图里面
    //                                curSubMap->ml_keyframe.push_front(kf_cur);
    //                                curSubMap->ml_kfId.push_back(curSubMap->ml_kfId[0]);//加入loopclosure的顺序
    //                                curSubMap->ml_kfId[0]=(kf_cur->global_index);
                            
//                            curSubMap->ml_keyframe.push_back(kf_cur);
//                            curSubMap->ml_kfId.push_back(kf_cur->global_index);//加入loopclosure的顺序
                            curSubMap->center_kf=kf_cur;
    //                            curSubMap->ml_kfId[0]=representkf_id;
                            curSubMap_l1kf=kf_cur;
                        }
//                        else{
//
//    //                            curSubMap->ml_kfId.push_back(representkf_id);//加入loopclosure的顺序
//
//                        }
                        curSubMap->ml_keyframe.push_back(kf_cur);
                        curSubMap->ml_kfId.push_back(kf_cur->global_index);
//                        representkf_id++;
                        
                        mMutex_ml_submap[cur_agent_index]->lock();
                        kfAttr_ForAMap[cur_agent_index].push_back(1);
                        kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                        mMutex_ml_submap[cur_agent_index]->unlock();
    //                        loop_closure->addToDatabase_demo(kf_cur->keypoints, kf_cur->descriptors);
                    }
                }
                
            }else{
                if(loop_succ && !isSameArea){
                    //两个子地图应该融合 把
                    if(oldSubMap_id!=curSubMap->global_index || oldSubMap_agent_id!=curSubMap->global_agent_id){
                        cout<<"代理是否重复融合："<<oldSubMap_id<<" , "<< oldSubMap_agent_id<<" , "<<curSubMap->global_index  <<" , "<< curSubMap->global_agent_id<<endl;
    //                            这是两个子地图
                        if(isL1){
    //                            说明L1层 l2层的 看当前子地图的数量和老子地图的数量（代表帧的数量 l1+l2），大于某个阈值 只记录两个子地图的关联关系，否则 融合
                            if(old_reCurSubMap->isBigView(threshold_bigView) && curSubMap->isBigView(threshold_bigView)){
                                //记录强关联关系
                                old_reCurSubMap->relatedSubMapIndex.push_back(curSubMap->global_index);
                                curSubMap->relatedSubMapIndex.push_back(old_reCurSubMap->global_index);
//                                cout<<"强关联了"<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            }else{
                               
                                
                                //                            暂时注释0621
                                curSubMap->perfectFuse_subMap2(old_reCurSubMap);
                              
////                                delete recurSubMap;
                                
                                cout<<"测试地址对不对 改之后"<<old_reCurSubMap<<" , "<<curSubMap<<endl;
                                cout<<old_reCurSubMap->global_index<<" , "<<curSubMap->global_index<<endl;
                                
                                int index=0;
                                int real_oldSubMap_id=oldSubMap_id-delteSubMap_num[cur_agent_index];
                                for(auto iter=ml_submap[cur_agent_index].begin(),iter_end=ml_submap[cur_agent_index].end();iter!=iter_end;iter++){
                                    
                                    if(index<real_oldSubMap_id){
                                        index++;
                                        continue;
                                    }
                                    if(oldSubMap_id==(*iter)->global_index){
                                        cout<<"测试是否删除成功："<<ml_submap[cur_agent_index].size()<<endl;
                                        ml_submap[cur_agent_index].erase(iter);
                                        relatedSubMap_forAgent[cur_agent_index][old_reCurSubMap->global_index]=curSubMap->global_index;
                                        delteSubMap_num[cur_agent_index]++;
                                        
                                        cout<<"测试是否删除成功："<<ml_submap[cur_agent_index].size()<<endl;
//                                        old_reCurSubMap->mMutexSubMapList.lock();
//                                        (*iter)=curSubMap;
//                                        old_reCurSubMap->mMutexSubMapList.unlock();
//                                        cout<<"测试老帧有没有改对"<<(*iter)->global_index<<endl;
                                        break;
                                    }
                                }
                                
                                
                                
                            
//                                old_reCurSubMap->perfectFuse_subMap2(curSubMap);
//                                curSubMap=old_reCurSubMap;
//                                mMutex_ml_submap[cur_agent_index]->lock();
//                                ml_submap[cur_agent_index].pop_back();
//                                ml_submap[cur_agent_index].push_back(curSubMap);
//                                mMutex_ml_submap[cur_agent_index]->unlock();
                                
//                                cout<<"子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            }
                        }else{
    //                            说明都是l2层 判断有没有一个子地图数量大于某个阈值，大于 都不能和l1匹配上，说明还是有一定的差距，或者中间有个拐弯 也应该更详细记录拐弯处的详情 不融合 只记录两个子地图的关联关系。 否则 融合
                            if(old_reCurSubMap->isBigView(threshold_bigView) || curSubMap->isBigView(threshold_bigView)){
                                //记录强关联关系 暂时没写
                                old_reCurSubMap->relatedSubMapIndex.push_back(curSubMap->global_index);
                                curSubMap->relatedSubMapIndex.push_back(old_reCurSubMap->global_index);
//                                cout<<"强关联了"<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            }else{

                                
                                //                            暂时注释0621
                                curSubMap->perfectFuse_subMap2(old_reCurSubMap);
                               
////                                delete recurSubMap;
                                ///
                                cout<<"测试地址对不对 改之后"<<old_reCurSubMap<<" , "<<curSubMap<<endl;
                                cout<<old_reCurSubMap->global_index<<" , "<<curSubMap->global_index<<endl;
                                
                                
                                int index=0;
                                int real_oldSubMap_id=oldSubMap_id-delteSubMap_num[cur_agent_index];
                                for(auto iter=ml_submap[cur_agent_index].begin(),iter_end=ml_submap[cur_agent_index].end();iter!=iter_end;iter++){
                                    
                                    if(index<real_oldSubMap_id){
                                        index++;
                                        continue;
                                    }
                                    if(oldSubMap_id==(*iter)->global_index){
                                        cout<<"测试是否删除成功："<<ml_submap[cur_agent_index].size()<<endl;
                                        ml_submap[cur_agent_index].erase(iter);
                                        relatedSubMap_forAgent[cur_agent_index][old_reCurSubMap->global_index]=curSubMap->global_index;
                                        delteSubMap_num[cur_agent_index]++;
                                        cout<<ml_submap[cur_agent_index].size()<<endl;
                                        
//                                        old_reCurSubMap->mMutexSubMapList.lock();
//                                        (*iter)=curSubMap;
//                                        old_reCurSubMap->mMutexSubMapList.unlock();
//                                        cout<<"测试老帧有没有改对"<<(*iter)->global_index<<endl;
                                        break;
                                    }
                                }
                                
                               
                                
                              
//                                old_reCurSubMap->perfectFuse_subMap2(curSubMap);
//                                curSubMap=old_reCurSubMap;
//                                mMutex_ml_submap[cur_agent_index]->lock();
//                                ml_submap[cur_agent_index].pop_back();
//                                ml_submap[cur_agent_index].push_back(curSubMap);
//                                mMutex_ml_submap[cur_agent_index]->unlock();
//                                cout<<"子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            }
                        }
                        
                    }else{
                        cout<<"本来就是一个地图"<<endl;
                    }
                    //                            由于当前帧是回环帧 所以应该加入所在的层
                    if(isSimlar || isSimlar_l2){
                        curSubMap->loop_mltra_num++;
                    }
                    mMutex_ml_submap[cur_agent_index]->lock();
                    kfAttr_ForAMap[cur_agent_index].push_back(2);
                    kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                    mMutex_ml_submap[cur_agent_index]->unlock();
                    
                    curSubMap->ml2_kf.push_back(kf_cur);
                    curSubMap->ml2_kfId.push_back(kf_cur->global_index);
                    
//                    cout<<"处理后 当前子地图"<<curSubMap->global_index<<" , "<<curSubMap->global_index_moreEarly.size()<<endl;
//                    for(int x=0,y=curSubMap->global_index_moreEarly.size();x<y;x++){
//                        cout<<"该子地图还包括："<<curSubMap->global_index_moreEarly[x]<<" , ";
//                    }
//                    cout<<endl;
                }else{
    //                    远
                    if(isSimlar_l2){
    //                        和第二层相似 加入作为非代表 TODO 这里也许不应该
    //                            curSubMap->noRepresent_kfId.push_back(kf_cur->global_index);
    //                            curSubMap->noRepresent_kfHeader.push_back(kf_cur->header);
    //                            kf_attr.kfAttr_ForAMap.push_back(0);
    //                            kf_attr.kfAttr_SubMapId.push_back(curSubMap->global_index);
                        
    //                            检查是否替换和自己相似的帧（如果共视的点数量大很多，那么替换。否则，考虑前面的更精确 累积误差小 不进行替换）
                        list<KeyFrame*>::iterator it = curSubMap->ml2_kf.begin();
                        //借助 advance() 函数将 it 迭代器前进 2 个位置
                        advance(it, isSimlar_l2_index);
                        KeyFrame* kf_old= *it;
                            assert(kf_old->global_index==curSubMap->ml2_kfId[isSimlar_l2_index]);
                        
                        if(kf_old->is_looped || kf_old->is_get_loop_info || kf_old->has_global_loop || kf_old->is_global_looped){
                            //无论如何都不替换
    //                        加入作为非代表
                            curSubMap->noRepresent_kfId_l2.push_back(kf_cur->global_index);
                            curSubMap->noRepresent_kfHeader_l2.push_back(kf_cur->header);
                            
                            mMutex_ml_submap[cur_agent_index]->lock();
                            kfAttr_ForAMap[cur_agent_index].push_back(3);
                            kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                            mMutex_ml_submap[cur_agent_index]->unlock();
                        }else{
                            int sum_old=0,sum_cur=0;
                            for(int x=0,y=kf_old->mvOrderedWeights.size();x<y;x++){
                                sum_old+=kf_old->mvOrderedWeights[x];
                            }
                            for(int x=0,y=kf_cur->mvOrderedWeights.size();x<y;x++){
                                sum_cur+=kf_cur->mvOrderedWeights[x];
                            }
    //                        暂时注释
                            cout<<"替换="<<sum_old<<" , "<<kf_old->mvOrderedWeights.size()<<" , "<<sum_cur*0.8<<" , "<<kf_cur->mvOrderedWeights.size()<<endl;
                            if((float)sum_old<(sum_cur*0.8)){
                                cout<<"真的替换了"<<endl;
    //                            int iter_index=0;
    //                            cout<<"更新前：";
    //                            for(auto iter_ml_start=curSubMap->ml2_kf.begin(),iter_ml_end=curSubMap->ml2_kf.end();iter_ml_start!=iter_ml_end;iter_ml_start++){
    //                                cout<<(*iter_ml_start)->global_index<<" , "<<curSubMap->ml2_kfId[iter_index]<<endl;
    //                                iter_index++;
    //
    //                            }

        //                                当前帧替换老的关键帧 成为代表帧，老的关键帧成为非代表帧
                                mMutex_ml_submap[cur_agent_index]->lock();
                                int real_oldkf_id=treeId_kf[cur_agent_index][kf_old->global_index];
                                kfAttr_ForAMap[cur_agent_index][real_oldkf_id]=3;
                                kfAttr_SubMapId[cur_agent_index][real_oldkf_id]=(curSubMap->global_index);
                                kfAttr_ForAMap[cur_agent_index].push_back(2);
                                kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                                mMutex_ml_submap[cur_agent_index]->unlock();
                                
        //                        加入作为非代表
                                curSubMap->noRepresent_kfId_l2.push_back(kf_old->global_index);
                                curSubMap->noRepresent_kfHeader_l2.push_back(kf_old->header);
    //                            kfAttr_ForAMap[cur_agent_index][kf_old->global_index]=3;
    //                            kfAttr_SubMapId[cur_agent_index][kf_old->global_index]=(curSubMap->global_index);
                                

                                curSubMap->ml2_kf.push_back(kf_cur);
                                curSubMap->ml2_kfId.push_back(kf_cur->global_index);//加入loopclosure的顺序
                                
        //                                验证这么操作对不对
                                curSubMap->ml2_kf.erase(it);
                                for(int h=isSimlar_l2_index,g=curSubMap->ml2_kfId.size();h<(g-1);h++){
                                    curSubMap->ml2_kfId[h]=curSubMap->ml2_kfId[h+1];
                                }
                                curSubMap->ml2_kfId.resize(curSubMap->ml2_kfId.size()-1);
        //                                curSubMap->ml2_kfId.erase(curSubMap->ml2_kfId.rbegin() + (curSubMap->ml2_kfId.size()-isSimlar_l2_index));


    //                            cout<<"更新后：";
    //                            iter_index=0;
    //                            for(auto iter_ml_start=curSubMap->ml2_kf.begin(),iter_ml_end=curSubMap->ml2_kf.end();iter_ml_start!=iter_ml_end;iter_ml_start++){
    //                                cout<<(*iter_ml_start)->global_index<<" , "<<curSubMap->ml2_kfId[iter_index]<<endl;
    //                                iter_index++;
    //
    //                            }
                            }else
                            {
        //                        加入作为非代表
                                curSubMap->noRepresent_kfId_l2.push_back(kf_cur->global_index);
                                curSubMap->noRepresent_kfHeader_l2.push_back(kf_cur->header);
                                
                                mMutex_ml_submap[cur_agent_index]->lock();
                                kfAttr_ForAMap[cur_agent_index].push_back(3);
                                kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                                mMutex_ml_submap[cur_agent_index]->unlock();
                            }
                        }
                    }else{
                        if(isSimlar){
    //                        加入作为代表 更为细节
                            curSubMap->ml2_kf.push_back(kf_cur);
    //                            curSubMap->ml2_kfId.push_back(representkf_id);//加入loopclosure的顺序
                            curSubMap->ml2_kfId.push_back(kf_cur->global_index);

                            
                            mMutex_ml_submap[cur_agent_index]->lock();
                            kfAttr_ForAMap[cur_agent_index].push_back(2);
                            kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                            mMutex_ml_submap[cur_agent_index]->unlock();
                        }
                        else{
    //                            这里是既不和l1层相似，也不和l2层相似
    //                        存在两种情况， 距离超远 应该产生/检索到另一个子地图  ， 距离适中<7m 作为代表放到当前子地图 作为代表
                            if(distance_cur_l1kf<threshold_distance_t_l2){
                                curSubMap->ml2_kf.push_back(kf_cur);//TODO 思考 也许这里应该加入l1层
    //                                curSubMap->ml2_kfId.push_back(representkf_id);//加入loopclosure的顺序
                                curSubMap->ml2_kfId.push_back(kf_cur->global_index);

                                mMutex_ml_submap[cur_agent_index]->lock();
                                kfAttr_ForAMap[cur_agent_index].push_back(2);
                                kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                                mMutex_ml_submap[cur_agent_index]->unlock();
                            }else{

    //                                产生新的子地图
                                FeatureSubMap  *firstSubMap=new FeatureSubMap();
                                    firstSubMap->ml_keyframe.push_back(kf_cur);//在前面加一个 这个不要成为头
                                firstSubMap->center_kf=kf_cur;

                                firstSubMap->global_agent_id=cur_agent_id;
                                
                                mMutex_ml_submap[cur_agent_index]->lock();
                                firstSubMap->global_index=ml_submap[cur_agent_index].size()+delteSubMap_num[cur_agent_index];
                               
                                ml_submap[cur_agent_index].push_back(firstSubMap);
                                isFirstKF=true;
                                kfAttr_ForAMap[cur_agent_index].push_back(1);
                                kfAttr_SubMapId[cur_agent_index].push_back(firstSubMap->global_index);
                                mMutex_ml_submap[cur_agent_index]->unlock();

                                firstSubMap->ml_kfId.push_back(kf_cur->global_index);

                                relatedSubMap_forAgent[cur_agent_index].push_back(firstSubMap->global_index);
                                
                                curSubMap=firstSubMap;
                                curSubMap_l1kf=curSubMap->ml_keyframe.front();
                       
                            }
                        }
                    }
                }
            }
//            old_reCurSubMap->mMutexSubMapList.unlock();
             
            mMutex_ml_submap[cur_agent_index]->lock();
            int subMap_id=kfAttr_SubMapId[cur_agent_index][kfNum_tree];
            int levelKf=kfAttr_ForAMap[cur_agent_index][kfNum_tree];
            
            assert(cur_agent_index<kfAttr_SubMapId.size());
            assert(cur_agent_index<kfAttr_ForAMap.size());
            assert(kfNum_tree<kfAttr_SubMapId[cur_agent_index].size());
            assert(kfNum_tree<kfAttr_ForAMap[cur_agent_index].size());
            
            assert(curSubMap->ml_kfId.size()==curSubMap->ml_keyframe.size());
            assert(curSubMap->ml2_kfId.size()==curSubMap->ml2_kf.size());
            
            int curSubMap_id=kfAttr_SubMapId[cur_agent_index][kfNum_tree];
            assert(curSubMap_id==curSubMap->global_index);
            mMutex_ml_submap[cur_agent_index]->unlock();
            
            cout<<"当前帧所属子地图"<<subMap_id<<" , "<<levelKf<<endl;
            if(loop_succ){
                cout<<"当前帧的更新："<<curSubMap->global_index<<" , ";
                for(int hh=0,j=curSubMap->global_index_moreEarly.size();hh<j;hh++){
                    cout<<curSubMap->global_index_moreEarly[hh]<<" , ";
                }
                cout<<endl;
//                int index=0;
//                for(auto iter=ml_submap[cur_agent_index].begin(),iter_end=ml_submap[cur_agent_index].end();iter!=iter_end;iter++){
//
//                    if(index<oldSubMap_id){
//                        index++;
//                        continue;
//                    }
//                    if(index==oldSubMap_id){
//                        cout<<"老帧的更新："<<(*iter)->global_index<<" , ";
//                        for(int hh=0,j=(*iter)->global_index_moreEarly.size();hh<j;hh++){
//                            cout<<(*iter)->global_index_moreEarly[hh]<<" , ";
//                        }
//                        break;
//                    }
//                }
            }
            
            cout<<endl<<endl;

        }
    }
//    TE(allocateSubMap);
}

//不进行融合 为了方便后续的优化问题
void FeatureMap::allocateSubMap2(KeyFrame* kf_cur,int old_index,int kfNum_tree,vector<int> old_vector){
//    TS(allocateSubMap);
    int cur_agent_id=kf_cur->c->getId();
    int cur_agent_index=-1;
    
    
    
    if(kf_cur->global_index==0){
        FeatureSubMap* firstSubMap=new FeatureSubMap();
        Vector3d first_t_w_i;
        Matrix3d first_r_w_i;
        kf_cur->getOriginPose(first_t_w_i, first_r_w_i);
        firstSubMap->ml_keyframe.push_back(kf_cur);
        firstSubMap->ml_kfId.push_back(kf_cur->global_index);
        firstSubMap->global_index=0;
        firstSubMap->center_kf=kf_cur;
        firstSubMap->global_agent_id=cur_agent_id;
        
//        firstSubMap->featureID=kf_cur->features_id_origin;
//        //点太少了 把重复的点加进去
//        firstSubMap->keyPoint=kf_cur->keypoints;
//        firstSubMap->descriptor=kf_cur->descriptors;

        readWriteLock_agent_id.writeLock();
        agent_id.push_back(cur_agent_id);
        readWriteLock_agent_id.writeUnLock();
        cur_agent_index=getIndex_fromAgentId(cur_agent_id);
        
        
        shared_ptr<std::mutex> md=make_shared<std::mutex>();
        mMutex_ml_submap.push_back(md);
        
        mMutex_ml_submap[cur_agent_index]->lock();
        ml_submap.push_back(list<FeatureSubMap*>{firstSubMap});
        kfAttr_ForAMap.push_back(vector<int>(1));
        kfAttr_SubMapId.push_back(vector<int>{firstSubMap->global_index});
        treeId_kf.push_back(map<int,int> { std::make_pair(kf_cur->global_index,kfNum_tree)});//kfNum_tree此时为0
        mMutex_ml_submap[cur_agent_index]->unlock();
    }else{
        cur_agent_index=getIndex_fromAgentId(cur_agent_id);
        
        if(kf_cur!=nullptr){
            bool isSimlar=false;
            bool isSimlar_l2=false;
            int isSimlar_index=-1;
            int isSimlar_l2_index=-1;
            
            mMutex_ml_submap[cur_agent_index]->lock();
            treeId_kf[cur_agent_index].insert(std::make_pair(kf_cur->global_index,kfNum_tree));
            FeatureSubMap *curSubMap=ml_submap[cur_agent_index].back();
            for(int i=0,j=old_vector.size();i<j;i++){
//                    只和代表帧做对比
                int kf_attr_l1l2no=kfAttr_ForAMap[cur_agent_index][old_vector[i]];
                if(kf_attr_l1l2no==1 || kf_attr_l1l2no==2){
                    for(int a=0,b=curSubMap->ml_kfId.size();a<b;a++){
                        if(old_vector[i]==treeId_kf[cur_agent_index][curSubMap->ml_kfId[a]]){
                            isSimlar=true;
                            isSimlar_index=a;
                            break;
                        }
                    }
//                    if(isSimlar){
//                        break;
//                    }
                    for(int a=0,b=curSubMap->ml2_kfId.size();a<b;a++){
                        if(old_vector[i]==treeId_kf[cur_agent_index][curSubMap->ml2_kfId[a]]){
                            isSimlar_l2=true;
                            isSimlar_l2_index=a;
                            break;
                        }
                    }
                }
                
                if(isSimlar || isSimlar_l2){
                    break;
                }
            }
            mMutex_ml_submap[cur_agent_index]->unlock();
            
            curSubMap->mMutexSubMapList.lock();
            KeyFrame *curSubMap_l1kf=curSubMap->center_kf;
            bool isFirstKF=false;
            if(curSubMap->ml_keyframe.size()==1){
                isFirstKF=true;
            }
            curSubMap->mMutexSubMapList.unlock();
            
//                计算当前帧 和 curSubMap_l1kf的距离
            Vector3d old_t_w_i;
            Matrix3d old_r_w_i;
            curSubMap_l1kf->getOriginPose(old_t_w_i, old_r_w_i);
            Vector3d cur_t_w_i;
            Matrix3d cur_r_w_i;
            kf_cur->getOriginPose(cur_t_w_i, cur_r_w_i);
            double distance_cur_l1kf=(old_t_w_i-cur_t_w_i).norm();
//            cout<<"distance_cur_l1kf="<< distance_cur_l1kf<<endl;
            
    //                算相似度 后面改一下 最多返回多少个 相似度过关的
            double threshold_distance_t=3,threshold_distance_t_l2=5;//以m为单位
            int threshold_bigView=10;//子地图的视角 丰富不

            bool loop_succ= (old_index==-1 ? false : true);
//                    近 比较代表帧 在不在old_vector里面，在说明相似度高。 不在说明 低
       
//                    这里是去找 是否存在相似度高，但是并没有考虑 它可能和其它地图内的帧更相似
//                   TODO 距离近的情况下，和其它子地图更相似，说明这个子地图 应该融合到其它子地图 （但是如果两个子地图时序上距离很近，说明新的子地图中心不太对）

            FeatureSubMap *old_reCurSubMap;
            bool isL1=false;
            int oldSubMap_id=-1;
            int oldSubMap_agent_id=-1;
            if(loop_succ){
//                cout<<"此时此刻，发生回环，应该要做子地图融合的"<<endl;
                mMutex_ml_submap[cur_agent_index]->lock();
                oldSubMap_id=kfAttr_SubMapId[cur_agent_index][old_index];
                int levelKf=kfAttr_ForAMap[cur_agent_index][old_index];
                mMutex_ml_submap[cur_agent_index]->unlock();
                
                cout<<"老帧所属子地图"<<oldSubMap_id<<" , "<<levelKf<<" , "<<old_index<<endl;
                int index=0;
                for(auto iter=ml_submap[cur_agent_index].begin(),iter_end=ml_submap[cur_agent_index].end();iter!=iter_end;iter++){
                    
                    if(index<oldSubMap_id){
                        index++;
                        continue;
                    }
                    if(index==oldSubMap_id){
                        old_reCurSubMap=*iter;
                        //oldSubMap_id一开始是老帧所在子地图的id， 现在变成所在子地图也许融合了 是它当前时刻最终的id
                        oldSubMap_id=(*iter)->global_index;
                        oldSubMap_agent_id=(*iter)->global_agent_id;
                        cout<<"oldSubMap="<<oldSubMap_id<<" , "<<oldSubMap_agent_id<<endl;
                        if(levelKf==1){
    //                                说明和对方l1层匹配上了 两个子地图应该融合
                            isL1=true;
                        }else if(levelKf==2){
    //                                说明和对方l2层匹配上了 只需要记住关联关系
                            ;
                        }else if(levelKf==0){
    //                                说明和非代表帧匹配上了 l1层
//                            为了做优化 需要把这一帧 插入到代表帧层面（强制）
                            isL1=true;
                        }else if(levelKf==3){
    //                                说明和非代表帧匹配上了 l2层
//                            为了做优化 需要把这一帧 插入到代表帧层面（强制）
                            ;
                        }
                        break;
                    }
                    
                }
            }
            
            cout<<"loop_succ="<<loop_succ<<" , "<<isL1<<" , "<<isSimlar<<" , "<<isSimlar_l2<<" , "<< distance_cur_l1kf<<" , "<<kf_cur->global_index<<endl;
//            cout<<"未做处理前 当前子地图"<<curSubMap->global_index<<" , "<<ml_submap.size()<<endl;
    //                短距离内 l1层
            if(distance_cur_l1kf<threshold_distance_t){
                if(loop_succ){
                    //两个子地图应该融合 把 && oldSubMap_agent_id!=curSubMap->global_agent_id
                    if(oldSubMap_id!=curSubMap->global_index || oldSubMap_agent_id!=curSubMap->global_agent_id){
                        
    //                            这是两个子地图
                        if(isL1){
    //                            说明都是L1层的 因为l2层有个l1层的细节扩展，但是和l1层更匹配 说明两个子地图应该完美融合
    //                            第一帧 以当前子地图的为主，用来比较距离（因为当前附近误差较小）
    //                                进行融合之前 要先检测当前地图是否已经发生过融合
                           
//                            cout<<"子地图融合前="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            
                            FeatureSubMap *recurSubMap;
                            recurSubMap=old_reCurSubMap;
//                            cout<<"删除oldSubMap="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , ";
                            for(int a=0,b=old_reCurSubMap->global_index_moreEarly.size();a<b;a++){
                                cout<<old_reCurSubMap->global_index_moreEarly[a]<<" , "<<old_reCurSubMap->global_agent_id_moreEarly[a]<<" , ";
                            }
                            cout<<endl;
//                            cout<<"测试地址对不对 改之前"<<recurSubMap<<" , "<<old_reCurSubMap<<" , "<<curSubMap<<endl;
                            
//                            暂时注释0621
//                            curSubMap->perfectFuse_subMap2(old_reCurSubMap);
//                            recurSubMap->mMutexSubMapList.lock();
////                            *old_reCurSubMap=*curSubMap;
//                            old_reCurSubMap=curSubMap;
//                            recurSubMap->mMutexSubMapList.unlock();
////                            delete recurSubMap;
                            
//                            cout<<"测试地址对不对 改之后"<<recurSubMap<<" , "<<old_reCurSubMap<<" , "<<curSubMap<<endl;
//                            cout<<recurSubMap->global_index<<" , "<<old_reCurSubMap->global_index<<" , "<<curSubMap->global_index<<endl;
                            
//                            old_reCurSubMap->perfectFuse_subMap2(curSubMap);
//                            curSubMap=old_reCurSubMap;
//                            mMutex_ml_submap[cur_agent_index]->lock();
//                            ml_submap[cur_agent_index].pop_back();
//                            ml_submap[cur_agent_index].push_back(curSubMap);
//                            mMutex_ml_submap[cur_agent_index]->unlock();
                            
//                            cout<<"子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            
                            
                            
                        }else{
    //                            说明一个是l1层 一个l2层 此时此刻的距离不准，看当前子地图的数量和老子地图的数量（代表帧的数量 l1+l2 说明视角大不大），大于某个阈值 只记录两个子地图的关联关系（两个人的视角 都比较全了，才有一次回环，说明比较偏），否则 一个人视角大一个视角小 （因为时间上差的比较远了，尽可能融合）融合
                            if(old_reCurSubMap->isBigView(threshold_bigView) && curSubMap->isBigView(threshold_bigView)){
                                //记录强关联关系
                                old_reCurSubMap->relatedSubMapIndex.push_back(curSubMap->global_index);
                                curSubMap->relatedSubMapIndex.push_back(old_reCurSubMap->global_index);
//                                cout<<"强关联了"<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            }else{
                                
//                                cout<<"子地图融合前="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                                FeatureSubMap *recurSubMap;
                                recurSubMap=old_reCurSubMap;
//                                cout<<"删除oldSubMap="<<recurSubMap->global_index<<" , "<<recurSubMap->global_agent_id<<" , ";
//                                for(int a=0,b=old_reCurSubMap->global_index_moreEarly.size();a<b;a++){
//                                    cout<<old_reCurSubMap->global_index_moreEarly[a]<<" , "<<old_reCurSubMap->global_agent_id_moreEarly[a]<<" , ";
//                                }
//                                cout<<endl;
//                                cout<<"测试地址对不对 改之前"<<recurSubMap<<" , "<<old_reCurSubMap<<" , "<<curSubMap<<endl;
                                
                                //                            暂时注释0621
//                                curSubMap->perfectFuse_subMap2(old_reCurSubMap);
//                                recurSubMap->mMutexSubMapList.lock();
////                                (*old_reCurSubMap)=(*curSubMap);
//                                old_reCurSubMap=curSubMap;
//                                recurSubMap->mMutexSubMapList.unlock();
////                                delete recurSubMap;
                                
//                                cout<<"测试地址对不对 改之后"<<recurSubMap<<" , "<<old_reCurSubMap<<" , "<<curSubMap<<endl;
//                                cout<<recurSubMap->global_index<<" , "<<old_reCurSubMap->global_index<<" , "<<curSubMap->global_index<<endl;
//                                old_reCurSubMap->perfectFuse_subMap2(curSubMap);
//                                curSubMap=old_reCurSubMap;
//                                mMutex_ml_submap[cur_agent_index]->lock();
//                                ml_submap[cur_agent_index].pop_back();
//                                ml_submap[cur_agent_index].push_back(curSubMap);
//                                mMutex_ml_submap[cur_agent_index]->unlock();
                                
//                                cout<<"子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            }
                        }
                        
                    }else{
                        cout<<"本来就是一个地图"<<endl;
                    }
                    //                            由于当前帧是回环帧 所以应该加入所在的层
                    if(isSimlar || isSimlar_l2){
                        curSubMap->loop_mltra_num++;
                    }
                    mMutex_ml_submap[cur_agent_index]->lock();
                    kfAttr_ForAMap[cur_agent_index].push_back(1);
                    kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                    mMutex_ml_submap[cur_agent_index]->unlock();
                    
                    curSubMap->mMutexSubMapList.lock();
                    curSubMap->ml_keyframe.push_back(kf_cur);
                    curSubMap->ml_kfId.push_back(kf_cur->global_index);
                    curSubMap->mMutexSubMapList.unlock();
//                    cout<<"处理后 当前子地图"<<curSubMap->global_index<<" , "<<curSubMap->global_index_moreEarly.size()<<endl;
//                    for(int x=0,y=curSubMap->global_index_moreEarly.size();x<y;x++){
//                        cout<<"该子地图还包括："<<curSubMap->global_index_moreEarly[x]<<" , ";
//                    }
//                    cout<<endl;
                }else{
    //                        没有子地图需要融合 仅仅是加入当前子地图
                    if(isSimlar){
    //                            检查是否替换和自己相似的帧（如果共视的点数量大很多，那么替换。否则，考虑前面的更精确 累积误差小 不进行替换）
                        list<KeyFrame*>::iterator it = curSubMap->ml_keyframe.begin();
                        //借助 advance() 函数将 it 迭代器前进 2 个位置
                        advance(it, isSimlar_index);
                        KeyFrame* kf_old= *it;
                        assert(kf_old->global_index==curSubMap->ml_kfId[isSimlar_index]);
                        
                        if(kf_old->is_looped || kf_old->is_get_loop_info || kf_old->has_global_loop || kf_old->is_global_looped){
                            //无论如何都不替换
    //                        加入作为非代表
                            curSubMap->noRepresent_kfId.push_back(kf_cur->global_index);
                            curSubMap->noRepresent_kfHeader.push_back(kf_cur->header);
                            kfAttr_ForAMap[cur_agent_index].push_back(0);
                            kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                        }else{
                            int sum_old=0,sum_cur=0;
                            for(int x=0,y=kf_old->mvOrderedWeights.size();x<y;x++){
                                sum_old+=kf_old->mvOrderedWeights[x];
                            }
                            for(int x=0,y=kf_cur->mvOrderedWeights.size();x<y;x++){
                                sum_cur+=kf_cur->mvOrderedWeights[x];
                            }
    //                        暂时注释
                            cout<<"替换="<<sum_old<<" , "<<kf_old->mvOrderedWeights.size()<<" , "<<sum_cur*0.8<<" , "<<kf_cur->mvOrderedWeights.size()<<endl;
                            if((float)sum_old<(sum_cur*0.8)){
                                cout<<"真的替换了"<<endl;
    //                            int iter_index=0;
    //                            cout<<"更新前：";
    //                            for(auto iter_ml_start=curSubMap->ml_keyframe.begin(),iter_ml_end=curSubMap->ml_keyframe.end();iter_ml_start!=iter_ml_end;iter_ml_start++){
    //                                cout<<(*iter_ml_start)->global_index<<" , "<<curSubMap->ml_kfId[iter_index]<<endl;
    //                                iter_index++;
    //
    //                            }

        //                                当前帧替换老的关键帧 成为代表帧，老的关键帧成为非代表帧
                                mMutex_ml_submap[cur_agent_index]->lock();
                                int real_oldkf_id=treeId_kf[cur_agent_index][kf_old->global_index];
                                kfAttr_ForAMap[cur_agent_index][real_oldkf_id]=0;
                                kfAttr_SubMapId[cur_agent_index][real_oldkf_id]=(curSubMap->global_index);
                                kfAttr_ForAMap[cur_agent_index].push_back(1);
                                kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                                mMutex_ml_submap[cur_agent_index]->unlock();
                                
        //                        加入作为非代表
                                curSubMap->noRepresent_kfId.push_back(kf_old->global_index);
                                curSubMap->noRepresent_kfHeader.push_back(kf_old->header);
    //                            kfAttr_ForAMap[cur_agent_index][kf_old->global_index]=0;
    //                            kfAttr_SubMapId[cur_agent_index][kf_old->global_index]=(curSubMap->global_index);
                                curSubMap->ml_keyframe.push_back(kf_cur);
                                curSubMap->ml_kfId.push_back(kf_cur->global_index);//加入loopclosure的顺序
                                

        //                                验证这么操作对不对
                                curSubMap->ml_keyframe.erase(it);
                                for(int h=isSimlar_index,g=curSubMap->ml_kfId.size();h<(g-1);h++){
                                    curSubMap->ml_kfId[h]=curSubMap->ml_kfId[h+1];
                                }
                                curSubMap->ml_kfId.resize(curSubMap->ml_kfId.size()-1);
        //                                curSubMap->ml_kfId.erase(curSubMap->ml_kfId.rbegin() + (curSubMap->ml_kfId.size()-isSimlar_index));

    //                            cout<<"更新后：";
    //                            iter_index=0;
    //                            for(auto iter_ml_start=curSubMap->ml_keyframe.begin(),iter_ml_end=curSubMap->ml_keyframe.end();iter_ml_start!=iter_ml_end;iter_ml_start++){
    //                                cout<<(*iter_ml_start)->global_index<<" , "<<curSubMap->ml_kfId[iter_index]<<endl;
    //                                iter_index++;
    //                            }
                            }
                            else
                            {
        //                        加入作为非代表
                                curSubMap->noRepresent_kfId.push_back(kf_cur->global_index);
                                curSubMap->noRepresent_kfHeader.push_back(kf_cur->header);
                                
                                mMutex_ml_submap[cur_agent_index]->lock();
                                kfAttr_ForAMap[cur_agent_index].push_back(0);
                                kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                                mMutex_ml_submap[cur_agent_index]->unlock();
                            }
                        }
                    }else{
    //                        加入作为代表
                        if(isFirstKF){
                            isFirstKF=false;
    //                                assert(curSubMap->ml_keyframe.size()==1);//此时 只有1个代表帧在子地图里面
    //                                curSubMap->ml_keyframe.push_front(kf_cur);
    //                                curSubMap->ml_kfId.push_back(curSubMap->ml_kfId[0]);//加入loopclosure的顺序
    //                                curSubMap->ml_kfId[0]=(kf_cur->global_index);
                            
//                            curSubMap->ml_keyframe.push_back(kf_cur);
//                            curSubMap->ml_kfId.push_back(kf_cur->global_index);//加入loopclosure的顺序
                            curSubMap->center_kf=kf_cur;
    //                            curSubMap->ml_kfId[0]=representkf_id;
                            curSubMap_l1kf=kf_cur;
                        }
//                        else{
//
//    //                            curSubMap->ml_kfId.push_back(representkf_id);//加入loopclosure的顺序
//
//                        }
                        curSubMap->ml_keyframe.push_back(kf_cur);
                        curSubMap->ml_kfId.push_back(kf_cur->global_index);
//                        representkf_id++;
                        
                        mMutex_ml_submap[cur_agent_index]->lock();
                        kfAttr_ForAMap[cur_agent_index].push_back(1);
                        kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                        mMutex_ml_submap[cur_agent_index]->unlock();
    //                        loop_closure->addToDatabase_demo(kf_cur->keypoints, kf_cur->descriptors);
                    }
                }
                
            }else{
                if(loop_succ){
                    //两个子地图应该融合 把
                    if(oldSubMap_id!=curSubMap->global_index || oldSubMap_agent_id!=curSubMap->global_agent_id){
    //                            这是两个子地图
                        if(isL1){
    //                            说明L1层 l2层的 看当前子地图的数量和老子地图的数量（代表帧的数量 l1+l2），大于某个阈值 只记录两个子地图的关联关系，否则 融合
                            if(old_reCurSubMap->isBigView(threshold_bigView) && curSubMap->isBigView(threshold_bigView)){
                                //记录强关联关系
                                old_reCurSubMap->relatedSubMapIndex.push_back(curSubMap->global_index);
                                curSubMap->relatedSubMapIndex.push_back(old_reCurSubMap->global_index);
//                                cout<<"强关联了"<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            }else{
                                
//                                cout<<"子地图融合前="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                                
                                FeatureSubMap *recurSubMap;
                                recurSubMap=old_reCurSubMap;
//                                cout<<"删除oldSubMap="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , ";
//                                for(int a=0,b=old_reCurSubMap->global_index_moreEarly.size();a<b;a++){
//                                    cout<<old_reCurSubMap->global_index_moreEarly[a]<<" , "<<old_reCurSubMap->global_agent_id_moreEarly[a]<<" , ";
//                                }
//                                cout<<endl;
//                                cout<<"测试地址对不对 改之前"<<recurSubMap<<" , "<<old_reCurSubMap<<" , "<<curSubMap<<endl;
                                
                                //                            暂时注释0621
//                                curSubMap->perfectFuse_subMap2(old_reCurSubMap);
//                                recurSubMap->mMutexSubMapList.lock();
////                                *old_reCurSubMap=*curSubMap;
//                                old_reCurSubMap=curSubMap;
//                                recurSubMap->mMutexSubMapList.unlock();
////                                delete recurSubMap;
                                
//                                cout<<"测试地址对不对 改之后"<<recurSubMap<<" , "<<old_reCurSubMap<<" , "<<curSubMap<<endl;
//                                cout<<recurSubMap->global_index<<" , "<<old_reCurSubMap->global_index<<" , "<<curSubMap->global_index<<endl;
//                                old_reCurSubMap->perfectFuse_subMap2(curSubMap);
//                                curSubMap=old_reCurSubMap;
//                                mMutex_ml_submap[cur_agent_index]->lock();
//                                ml_submap[cur_agent_index].pop_back();
//                                ml_submap[cur_agent_index].push_back(curSubMap);
//                                mMutex_ml_submap[cur_agent_index]->unlock();
                                
//                                cout<<"子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            }
                        }else{
    //                            说明都是l2层 判断有没有一个子地图数量大于某个阈值，大于 都不能和l1匹配上，说明还是有一定的差距，或者中间有个拐弯 也应该更详细记录拐弯处的详情 不融合 只记录两个子地图的关联关系。 否则 融合
                            if(old_reCurSubMap->isBigView(threshold_bigView) || curSubMap->isBigView(threshold_bigView)){
                                //记录强关联关系 暂时没写
                                old_reCurSubMap->relatedSubMapIndex.push_back(curSubMap->global_index);
                                curSubMap->relatedSubMapIndex.push_back(old_reCurSubMap->global_index);
//                                cout<<"强关联了"<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            }else{
//                                cout<<"子地图融合前="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                                
                                FeatureSubMap *recurSubMap;
                                recurSubMap=old_reCurSubMap;
//                                cout<<"删除oldSubMap="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , ";
//                                for(int a=0,b=old_reCurSubMap->global_index_moreEarly.size();a<b;a++){
//                                    cout<<old_reCurSubMap->global_index_moreEarly[a]<<" , "<<old_reCurSubMap->global_agent_id_moreEarly[a]<<" , ";
//                                }
//                                cout<<endl;
//                                cout<<"测试地址对不对 改之前"<<recurSubMap<<" , "<<old_reCurSubMap<<" , "<<curSubMap<<endl;
                                
                                //                            暂时注释0621
//                                curSubMap->perfectFuse_subMap2(old_reCurSubMap);
//                                recurSubMap->mMutexSubMapList.lock();
////                                (*old_reCurSubMap)=(*curSubMap);
//                                old_reCurSubMap=curSubMap;
//                                recurSubMap->mMutexSubMapList.unlock();
////                                delete recurSubMap;
                                
//                                cout<<"测试地址对不对 改之后"<<recurSubMap<<" , "<<old_reCurSubMap<<" , "<<curSubMap<<endl;
//                                cout<<recurSubMap->global_index<<" , "<<old_reCurSubMap->global_index<<" , "<<curSubMap->global_index<<endl;
                                
//                                old_reCurSubMap->perfectFuse_subMap2(curSubMap);
//                                curSubMap=old_reCurSubMap;
//                                mMutex_ml_submap[cur_agent_index]->lock();
//                                ml_submap[cur_agent_index].pop_back();
//                                ml_submap[cur_agent_index].push_back(curSubMap);
//                                mMutex_ml_submap[cur_agent_index]->unlock();
//                                cout<<"子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                            }
                        }
                        
                    }else{
                        cout<<"本来就是一个地图"<<endl;
                    }
                    //                            由于当前帧是回环帧 所以应该加入所在的层
                    if(isSimlar || isSimlar_l2){
                        curSubMap->loop_mltra_num++;
                    }
                    mMutex_ml_submap[cur_agent_index]->lock();
                    kfAttr_ForAMap[cur_agent_index].push_back(2);
                    kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                    mMutex_ml_submap[cur_agent_index]->unlock();
                    
                    curSubMap->ml2_kf.push_back(kf_cur);
                    curSubMap->ml2_kfId.push_back(kf_cur->global_index);
                    
//                    cout<<"处理后 当前子地图"<<curSubMap->global_index<<" , "<<curSubMap->global_index_moreEarly.size()<<endl;
//                    for(int x=0,y=curSubMap->global_index_moreEarly.size();x<y;x++){
//                        cout<<"该子地图还包括："<<curSubMap->global_index_moreEarly[x]<<" , ";
//                    }
//                    cout<<endl;
                }else{
    //                    远
                    if(isSimlar_l2){
    //                        和第二层相似 加入作为非代表 TODO 这里也许不应该
    //                            curSubMap->noRepresent_kfId.push_back(kf_cur->global_index);
    //                            curSubMap->noRepresent_kfHeader.push_back(kf_cur->header);
    //                            kf_attr.kfAttr_ForAMap.push_back(0);
    //                            kf_attr.kfAttr_SubMapId.push_back(curSubMap->global_index);
                        
    //                            检查是否替换和自己相似的帧（如果共视的点数量大很多，那么替换。否则，考虑前面的更精确 累积误差小 不进行替换）
                        list<KeyFrame*>::iterator it = curSubMap->ml2_kf.begin();
                        //借助 advance() 函数将 it 迭代器前进 2 个位置
                        advance(it, isSimlar_l2_index);
                        KeyFrame* kf_old= *it;
                            assert(kf_old->global_index==curSubMap->ml2_kfId[isSimlar_l2_index]);
                        
                        if(kf_old->is_looped || kf_old->is_get_loop_info || kf_old->has_global_loop || kf_old->is_global_looped){
                            //无论如何都不替换
    //                        加入作为非代表
                            curSubMap->noRepresent_kfId_l2.push_back(kf_cur->global_index);
                            curSubMap->noRepresent_kfHeader_l2.push_back(kf_cur->header);
                            
                            mMutex_ml_submap[cur_agent_index]->lock();
                            kfAttr_ForAMap[cur_agent_index].push_back(3);
                            kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                            mMutex_ml_submap[cur_agent_index]->unlock();
                        }else{
                            int sum_old=0,sum_cur=0;
                            for(int x=0,y=kf_old->mvOrderedWeights.size();x<y;x++){
                                sum_old+=kf_old->mvOrderedWeights[x];
                            }
                            for(int x=0,y=kf_cur->mvOrderedWeights.size();x<y;x++){
                                sum_cur+=kf_cur->mvOrderedWeights[x];
                            }
    //                        暂时注释
                            cout<<"替换="<<sum_old<<" , "<<kf_old->mvOrderedWeights.size()<<" , "<<sum_cur*0.8<<" , "<<kf_cur->mvOrderedWeights.size()<<endl;
                            if((float)sum_old<(sum_cur*0.8)){
                                cout<<"真的替换了"<<endl;
    //                            int iter_index=0;
    //                            cout<<"更新前：";
    //                            for(auto iter_ml_start=curSubMap->ml2_kf.begin(),iter_ml_end=curSubMap->ml2_kf.end();iter_ml_start!=iter_ml_end;iter_ml_start++){
    //                                cout<<(*iter_ml_start)->global_index<<" , "<<curSubMap->ml2_kfId[iter_index]<<endl;
    //                                iter_index++;
    //
    //                            }

        //                                当前帧替换老的关键帧 成为代表帧，老的关键帧成为非代表帧
                                mMutex_ml_submap[cur_agent_index]->lock();
                                int real_oldkf_id=treeId_kf[cur_agent_index][kf_old->global_index];
                                kfAttr_ForAMap[cur_agent_index][real_oldkf_id]=3;
                                kfAttr_SubMapId[cur_agent_index][real_oldkf_id]=(curSubMap->global_index);
                                kfAttr_ForAMap[cur_agent_index].push_back(2);
                                kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                                mMutex_ml_submap[cur_agent_index]->unlock();
                                
        //                        加入作为非代表
                                curSubMap->noRepresent_kfId_l2.push_back(kf_old->global_index);
                                curSubMap->noRepresent_kfHeader_l2.push_back(kf_old->header);
    //                            kfAttr_ForAMap[cur_agent_index][kf_old->global_index]=3;
    //                            kfAttr_SubMapId[cur_agent_index][kf_old->global_index]=(curSubMap->global_index);
                                

                                curSubMap->ml2_kf.push_back(kf_cur);
                                curSubMap->ml2_kfId.push_back(kf_cur->global_index);//加入loopclosure的顺序
                                
        //                                验证这么操作对不对
                                curSubMap->ml2_kf.erase(it);
                                for(int h=isSimlar_l2_index,g=curSubMap->ml2_kfId.size();h<(g-1);h++){
                                    curSubMap->ml2_kfId[h]=curSubMap->ml2_kfId[h+1];
                                }
                                curSubMap->ml2_kfId.resize(curSubMap->ml2_kfId.size()-1);
        //                                curSubMap->ml2_kfId.erase(curSubMap->ml2_kfId.rbegin() + (curSubMap->ml2_kfId.size()-isSimlar_l2_index));


    //                            cout<<"更新后：";
    //                            iter_index=0;
    //                            for(auto iter_ml_start=curSubMap->ml2_kf.begin(),iter_ml_end=curSubMap->ml2_kf.end();iter_ml_start!=iter_ml_end;iter_ml_start++){
    //                                cout<<(*iter_ml_start)->global_index<<" , "<<curSubMap->ml2_kfId[iter_index]<<endl;
    //                                iter_index++;
    //
    //                            }
                            }else
                            {
        //                        加入作为非代表
                                curSubMap->noRepresent_kfId_l2.push_back(kf_cur->global_index);
                                curSubMap->noRepresent_kfHeader_l2.push_back(kf_cur->header);
                                
                                mMutex_ml_submap[cur_agent_index]->lock();
                                kfAttr_ForAMap[cur_agent_index].push_back(3);
                                kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                                mMutex_ml_submap[cur_agent_index]->unlock();
                            }
                        }
                    }else{
                        if(isSimlar){
    //                        加入作为代表 更为细节
                            curSubMap->ml2_kf.push_back(kf_cur);
    //                            curSubMap->ml2_kfId.push_back(representkf_id);//加入loopclosure的顺序
                            curSubMap->ml2_kfId.push_back(kf_cur->global_index);
//                            representkf_id++;
    //                            loop_closure->addToDatabase_demo(kf_cur->keypoints, kf_cur->descriptors);
                            
                            mMutex_ml_submap[cur_agent_index]->lock();
                            kfAttr_ForAMap[cur_agent_index].push_back(2);
                            kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                            mMutex_ml_submap[cur_agent_index]->unlock();
                        }
                        else{
    //                            这里是既不和l1层相似，也不和l2层相似
    //                        存在两种情况， 距离超远 应该产生/检索到另一个子地图  ， 距离适中<7m 作为代表放到当前子地图 作为代表
                            if(distance_cur_l1kf<threshold_distance_t_l2){
                                curSubMap->ml2_kf.push_back(kf_cur);//TODO 思考 也许这里应该加入l1层
    //                                curSubMap->ml2_kfId.push_back(representkf_id);//加入loopclosure的顺序
                                curSubMap->ml2_kfId.push_back(kf_cur->global_index);
//                                representkf_id++;
    //                                loop_closure->addToDatabase_demo(kf_cur->keypoints, kf_cur->descriptors);
                                mMutex_ml_submap[cur_agent_index]->lock();
                                kfAttr_ForAMap[cur_agent_index].push_back(2);
                                kfAttr_SubMapId[cur_agent_index].push_back(curSubMap->global_index);
                                mMutex_ml_submap[cur_agent_index]->unlock();
                            }else{

    //                                产生新的子地图
                                FeatureSubMap  *firstSubMap=new FeatureSubMap();
                                    firstSubMap->ml_keyframe.push_back(kf_cur);//在前面加一个 这个不要成为头
                                firstSubMap->center_kf=kf_cur;
//                                    firstSubMap->featureID=kf_cur->features_id_origin;
//                                    //点太少了 把重复的点加进去
//                                    firstSubMap->keyPoint=kf_cur->keypoints;
//                                    firstSubMap->descriptor=kf_cur->descriptors;
//                                    submap_size++;
                                firstSubMap->global_agent_id=cur_agent_id;
                                
                                mMutex_ml_submap[cur_agent_index]->lock();
                                firstSubMap->global_index=ml_submap[cur_agent_index].size();
                                ml_submap[cur_agent_index].push_back(firstSubMap);
                                isFirstKF=true;
                                kfAttr_ForAMap[cur_agent_index].push_back(1);
                                kfAttr_SubMapId[cur_agent_index].push_back(firstSubMap->global_index);
                                mMutex_ml_submap[cur_agent_index]->unlock();

                                firstSubMap->ml_kfId.push_back(kf_cur->global_index);

                                
                                curSubMap=firstSubMap;
                                curSubMap_l1kf=curSubMap->ml_keyframe.front();
                       
                            }
                        }
                    }
                }
            }
             
            mMutex_ml_submap[cur_agent_index]->lock();
            int subMap_id=kfAttr_SubMapId[cur_agent_index][kfNum_tree];
            int levelKf=kfAttr_ForAMap[cur_agent_index][kfNum_tree];
            
            assert(cur_agent_index<kfAttr_SubMapId.size());
            assert(cur_agent_index<kfAttr_ForAMap.size());
            assert(kfNum_tree<kfAttr_SubMapId[cur_agent_index].size());
            assert(kfNum_tree<kfAttr_ForAMap[cur_agent_index].size());
            
            assert(curSubMap->ml_kfId.size()==curSubMap->ml_keyframe.size());
            assert(curSubMap->ml2_kfId.size()==curSubMap->ml2_kf.size());
            
            int curSubMap_id=kfAttr_SubMapId[cur_agent_index][kfNum_tree];
            assert(curSubMap_id==curSubMap->global_index);
            mMutex_ml_submap[cur_agent_index]->unlock();
            
            cout<<"当前帧所属子地图"<<subMap_id<<" , "<<levelKf<<endl<<endl;

        }
    }
//    TE(allocateSubMap);
}

//多个代理内 子地图的融合
void FeatureMap::allocateSubMap_multi(KeyFrame* kf_cur,KeyFrame* kf_old){
//    判断是否为空
    if(kf_cur && kf_old){
        int cur_agent_id=kf_cur->c->getId();
        int cur_index=kf_cur->global_index;
        int cur_agent_index=getIndex_fromAgentId(cur_agent_id);
//        这里会因为实时性 出问题
        FeatureSubMap *curSubMap;
        bool isFind=false;
        mMutex_ml_submap[cur_agent_index]->lock();
        int cur_kf_saveIndex=treeId_kf[cur_agent_index][cur_index];
        int curSubMap_id=kfAttr_SubMapId[cur_agent_index][cur_kf_saveIndex];
        int level_curKf=kfAttr_ForAMap[cur_agent_index][cur_kf_saveIndex];
        for(auto iter=ml_submap[cur_agent_index].rbegin(),iter_end=ml_submap[cur_agent_index].rend();iter!=iter_end;iter++){
            if(curSubMap_id==(*iter)->global_index && cur_agent_id==(*iter)->global_agent_id){
                curSubMap=*iter;
                
                isFind=true;
                break;
            }
            for(int a=0,b=(*iter)->global_index_moreEarly.size();a<b;a++){
                if(curSubMap_id==(*iter)->global_index_moreEarly[a] && cur_agent_id==(*iter)->global_agent_id_moreEarly[a]){
                    curSubMap=*iter;
                    
                    isFind=true;
                    break;
                }
            }
            if(isFind){
                break;
            }
        }
        mMutex_ml_submap[cur_agent_index]->unlock();
    //    这里可能报错 因为一旦和其他人发生过融合
        assert(isFind);
        bool isL1_cur=false;
        if(level_curKf==0 || level_curKf==1){
            isL1_cur=true;
        }
            
        int old_agent_id=kf_old->c->getId();
        int old_index=kf_old->global_index;
        int old_agent_index=getIndex_fromAgentId(old_agent_id);
        FeatureSubMap *old_reCurSubMap;
        list<FeatureSubMap *>::iterator iter_update;
        bool isL1=false;
        int oldSubMap_agent_id=-1;
        int index=0;
        mMutex_ml_submap[old_agent_index]->lock();
        int old_kf_saveIndex=treeId_kf[old_agent_index][old_index];
        int oldSubMap_id=kfAttr_SubMapId[old_agent_index][old_kf_saveIndex];
        int level_oldKf=kfAttr_ForAMap[old_agent_index][old_kf_saveIndex];
        vector<int> relatedSubMap_curAgent=relatedSubMap_forAgent[old_agent_index];
        while(relatedSubMap_curAgent[oldSubMap_id]!=oldSubMap_id){
            oldSubMap_id=relatedSubMap_curAgent[oldSubMap_id];
        }
        int real_oldSubMap_id=oldSubMap_id-delteSubMap_num[old_agent_index];
        for(auto iter=ml_submap[old_agent_index].begin(),iter_end=ml_submap[old_agent_index].end();iter!=iter_end;iter++,index++){
            if(index<real_oldSubMap_id){
                index++;
                continue;
            }
            if((*iter)->global_index==oldSubMap_id){
                old_reCurSubMap=*iter;
                iter_update=iter;
                //oldSubMap_id一开始是老帧所在子地图的id， 现在变成所在子地图也许融合了 是它当前时刻最终的id
//               这里不能加assert 应为kfAttr没有更新
//                assert(oldSubMap_id==(*iter)->global_index);
               
                oldSubMap_agent_id=(*iter)->global_agent_id;
                if(level_oldKf==1){
//                                说明和对方l1层匹配上了 两个子地图应该融合
                    isL1=true;
                }else if(level_oldKf==2){
//                                说明和对方l2层匹配上了 只需要记住关联关系
                    ;
                }else if(level_oldKf==0){
//                                说明和非代表帧匹配上了 l1层
                    isL1=true;
                }else if(level_oldKf==3){
//                                说明和非代表帧匹配上了 l2层
                    ;
                }
                break;
            }
        }
        mMutex_ml_submap[old_agent_index]->unlock();
    

        int threshold_bigView=10;//子地图的视角 丰富不
        

        //两个子地图应该融合 这里需要重新判断 不同子地图 不同代理之间，如何融合过没？

        if(oldSubMap_id!=curSubMap->global_index || oldSubMap_agent_id!=curSubMap->global_agent_id){
//                            这是两个子地图
            if(isL1 && isL1_cur){
//                            说明都是L1层的 因为l2层有个l1层的细节扩展，但是和l1层更匹配 说明两个子地图应该完美融合
//                            第一帧 以当前子地图的为主，用来比较距离（因为当前附近误差较小）
//                                进行融合之前 要先检测当前地图是否已经发生过融合
                
//                cout<<"多地图 子地图融合前="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
//                FeatureSubMap *recurSubMap;
//                recurSubMap=old_reCurSubMap;
//                curSubMap->perfectFuse_subMap(old_reCurSubMap);
//                recurSubMap->mMutexSubMapList.lock();
////                *old_reCurSubMap=*curSubMap;
//                (*iter_update)=curSubMap;
//                recurSubMap->mMutexSubMapList.unlock();
//                delete recurSubMap;
                
//                mMutex_ml_submap[cur_agent_index]->lock();
//                ml_submap[cur_agent_index].pop_back();
//                ml_submap[cur_agent_index].push_back(curSubMap);
//                mMutex_ml_submap[cur_agent_index]->unlock();
                
                //记录强关联关系
                old_reCurSubMap->related_otherAgentSubMapIndex.push_back(curSubMap->global_index);
                old_reCurSubMap->related_otherAgentId.push_back(curSubMap->global_agent_id);
                curSubMap->related_otherAgentSubMapIndex.push_back(old_reCurSubMap->global_index);
                curSubMap->related_otherAgentId.push_back(old_reCurSubMap->global_agent_id);
                
                cout<<"多地图 子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
//                cout<<"测试 是不是同一个地址："<<curSubMap<<" , "<<(*iter)<<endl;
            }else if(isL1 || isL1_cur){
//                            说明一个是l1层 一个l2层 此时此刻的距离不准，看当前子地图的数量和老子地图的数量（代表帧的数量 l1+l2 说明视角大不大），大于某个阈值 只记录两个子地图的关联关系（两个人的视角 都比较全了，才有一次回环，说明比较偏），否则 一个人视角大一个视角小 （因为时间上差的比较远了，尽可能融合）融合
                if(old_reCurSubMap->isBigView(threshold_bigView) && curSubMap->isBigView(threshold_bigView)){
                    //记录强关联关系
//                    old_reCurSubMap->relatedSubMapIndex.push_back(curSubMap->global_index);
//                    curSubMap->relatedSubMapIndex.push_back(old_reCurSubMap->global_index);
                    
                    old_reCurSubMap->related_otherAgentSubMapIndex.push_back(curSubMap->global_index);
                    old_reCurSubMap->related_otherAgentId.push_back(curSubMap->global_agent_id);
                    
                    curSubMap->related_otherAgentSubMapIndex.push_back(old_reCurSubMap->global_index);
                    curSubMap->related_otherAgentId.push_back(old_reCurSubMap->global_agent_id);
                    
                    cout<<"多地图 强关联了"<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                }else{
//                    cout<<"多地图 子地图融合前="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
//
//                    FeatureSubMap *recurSubMap;
//                    recurSubMap=old_reCurSubMap;
//                    curSubMap->perfectFuse_subMap(old_reCurSubMap);
//                    recurSubMap->mMutexSubMapList.lock();
////                    *old_reCurSubMap=*curSubMap;
//                    (*iter_update)=curSubMap;
//                    recurSubMap->mMutexSubMapList.unlock();
//                    delete recurSubMap;


//                    mMutex_ml_submap[cur_agent_index]->lock();
//                    ml_submap[cur_agent_index].pop_back();
//                    ml_submap[cur_agent_index].push_back(curSubMap);
//                    mMutex_ml_submap[cur_agent_index]->unlock();
                    
                    old_reCurSubMap->related_otherAgentSubMapIndex.push_back(curSubMap->global_index);
                    old_reCurSubMap->related_otherAgentId.push_back(curSubMap->global_agent_id);
                    curSubMap->related_otherAgentSubMapIndex.push_back(old_reCurSubMap->global_index);
                    curSubMap->related_otherAgentId.push_back(old_reCurSubMap->global_agent_id);
                    
                    cout<<"多地图 子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                }
            }else if(!isL1 && !isL1_cur){
                //                            说明都是l2层 判断有没有一个子地图数量大于某个阈值，大于 都不能和l1匹配上，说明还是有一定的差距，或者中间有个拐弯 也应该更详细记录拐弯处的详情 不融合 只记录两个子地图的关联关系。 否则 融合
                if(old_reCurSubMap->isBigView(threshold_bigView) || curSubMap->isBigView(threshold_bigView)){
                    //记录强关联关系 暂时没写
//                    old_reCurSubMap->relatedSubMapIndex.push_back(curSubMap->global_index);
//                    curSubMap->relatedSubMapIndex.push_back(old_reCurSubMap->global_index);
                    
                    old_reCurSubMap->related_otherAgentSubMapIndex.push_back(curSubMap->global_index);
                    old_reCurSubMap->related_otherAgentId.push_back(curSubMap->global_agent_id);
                    
                    curSubMap->related_otherAgentSubMapIndex.push_back(old_reCurSubMap->global_index);
                    curSubMap->related_otherAgentId.push_back(old_reCurSubMap->global_agent_id);
                    
                    cout<<"多地图 强关联了"<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                }else{
//                    cout<<"多地图 子地图融合前="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
//                    FeatureSubMap *recurSubMap;
//                    recurSubMap=old_reCurSubMap;
//                    curSubMap->perfectFuse_subMap(old_reCurSubMap);
//                    recurSubMap->mMutexSubMapList.lock();
////                    *old_reCurSubMap=*curSubMap;
//                    (*iter_update)=curSubMap;
//                    recurSubMap->mMutexSubMapList.unlock();
//                    delete recurSubMap;

//                    mMutex_ml_submap[cur_agent_index]->lock();
//                    ml_submap[cur_agent_index].pop_back();
//                    ml_submap[cur_agent_index].push_back(curSubMap);
//                    mMutex_ml_submap[cur_agent_index]->unlock();
                    
                    old_reCurSubMap->related_otherAgentSubMapIndex.push_back(curSubMap->global_index);
                    old_reCurSubMap->related_otherAgentId.push_back(curSubMap->global_agent_id);
                    curSubMap->related_otherAgentSubMapIndex.push_back(old_reCurSubMap->global_index);
                    curSubMap->related_otherAgentId.push_back(old_reCurSubMap->global_agent_id);
                    
                    cout<<"多地图 子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                }
                
            }
        }
    }
    
}

//多个代理内 子地图的融合
//由于多代理之间 只记录关联关系，因此判断两个地图有没有融合过 要改
void FeatureMap::allocateSubMap_multi2(KeyFrame* kf_cur,KeyFrame* kf_old){
//    判断是否为空
    if(kf_cur && kf_old){
        int cur_agent_id=kf_cur->c->getId();
        int cur_index=kf_cur->global_index;
        int cur_agent_index=getIndex_fromAgentId(cur_agent_id);
//        这里会因为实时性 出问题
        FeatureSubMap *curSubMap;
        bool isFind=false;
        mMutex_ml_submap[cur_agent_index]->lock();
        int cur_kf_saveIndex=treeId_kf[cur_agent_index][cur_index];
        int curSubMap_id=kfAttr_SubMapId[cur_agent_index][cur_kf_saveIndex];
        int level_curKf=kfAttr_ForAMap[cur_agent_index][cur_kf_saveIndex];
        for(auto iter=ml_submap[cur_agent_index].rbegin(),iter_end=ml_submap[cur_agent_index].rend();iter!=iter_end;iter++){
            if(curSubMap_id==(*iter)->global_index && cur_agent_id==(*iter)->global_agent_id){
                curSubMap=*iter;
                
                isFind=true;
                break;
            }
            for(int a=0,b=(*iter)->global_index_moreEarly.size();a<b;a++){
                if(curSubMap_id==(*iter)->global_index_moreEarly[a] && cur_agent_id==(*iter)->global_agent_id_moreEarly[a]){
                    curSubMap=*iter;
                    
                    isFind=true;
                    break;
                }
            }
            if(isFind){
                break;
            }
        }
        mMutex_ml_submap[cur_agent_index]->unlock();
    //    这里可能报错 因为一旦和其他人发生过融合
        assert(isFind);
        bool isL1_cur=false;
        if(level_curKf==0 || level_curKf==1){
            isL1_cur=true;
        }
            
        int old_agent_id=kf_old->c->getId();
        int old_index=kf_old->global_index;
        int old_agent_index=getIndex_fromAgentId(old_agent_id);
        FeatureSubMap *old_reCurSubMap;
       
        bool isL1=false;
        int oldSubMap_agent_id=-1;
        int index=0;
        mMutex_ml_submap[old_agent_index]->lock();
        int old_kf_saveIndex=treeId_kf[old_agent_index][old_index];
        int oldSubMap_id=kfAttr_SubMapId[old_agent_index][old_kf_saveIndex];
        int level_oldKf=kfAttr_ForAMap[old_agent_index][old_kf_saveIndex];
        vector<int> relatedSubMap_curAgent=relatedSubMap_forAgent[old_agent_index];
        while(relatedSubMap_curAgent[oldSubMap_id]!=oldSubMap_id){
            oldSubMap_id=relatedSubMap_curAgent[oldSubMap_id];
        }
        int real_oldSubMap_id=oldSubMap_id-delteSubMap_num[old_agent_index];
        for(auto iter=ml_submap[old_agent_index].begin(),iter_end=ml_submap[old_agent_index].end();iter!=iter_end;iter++,index++){
            if(index<real_oldSubMap_id){
                index++;
                continue;
            }
            if((*iter)->global_index==oldSubMap_id){
                old_reCurSubMap=*iter;
                oldSubMap_agent_id=(*iter)->global_agent_id;
                if(level_oldKf==1){
//                                说明和对方l1层匹配上了 两个子地图应该融合
                    isL1=true;
                }else if(level_oldKf==2){
//                                说明和对方l2层匹配上了 只需要记住关联关系
                    ;
                }else if(level_oldKf==0){
//                                说明和非代表帧匹配上了 l1层
                    isL1=true;
                }else if(level_oldKf==3){
//                                说明和非代表帧匹配上了 l2层
                    ;
                }
                break;
            }
        }
        mMutex_ml_submap[old_agent_index]->unlock();
    

        int threshold_bigView=10;//子地图的视角 丰富不
        
        bool isSameSubMap=false;
        //两个子地图应该融合 这里需要重新判断 不同子地图 不同代理之间，如何融合过没？
        for(int i=0,j=curSubMap->related_otherAgentId.size();i<j;i++){
            int relatedAgentId=curSubMap->related_otherAgentId[i];
            int relatedAgentSubmapId=curSubMap->related_otherAgentSubMapIndex[i];
            if(oldSubMap_id==relatedAgentSubmapId && oldSubMap_agent_id==relatedAgentId){
                isSameSubMap=true;
                break;
            }
            for(int a=0,b=old_reCurSubMap->global_agent_id_moreEarly.size();a<b;a++){
                int old_relatedAgentId=old_reCurSubMap->global_agent_id_moreEarly[i];
                int old_relatedAgentSubmapId=old_reCurSubMap->global_index_moreEarly[i];
                if(old_relatedAgentSubmapId==relatedAgentSubmapId && old_relatedAgentId==relatedAgentId){
                    isSameSubMap=true;
                    break;
                }
            }
            if(isSameSubMap){
                break;
            }
            
        }
        

        if(!isSameSubMap){
//                            这是两个子地图
            if(isL1 && isL1_cur){
//                            说明都是L1层的 因为l2层有个l1层的细节扩展，但是和l1层更匹配 说明两个子地图应该完美融合
//                            第一帧 以当前子地图的为主，用来比较距离（因为当前附近误差较小）
//                                进行融合之前 要先检测当前地图是否已经发生过融合
                
//                cout<<"多地图 子地图融合前="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
//                FeatureSubMap *recurSubMap;
//                recurSubMap=old_reCurSubMap;
//                curSubMap->perfectFuse_subMap(old_reCurSubMap);
//                recurSubMap->mMutexSubMapList.lock();
////                *old_reCurSubMap=*curSubMap;
//                (*iter_update)=curSubMap;
//                recurSubMap->mMutexSubMapList.unlock();
//                delete recurSubMap;
                
//                mMutex_ml_submap[cur_agent_index]->lock();
//                ml_submap[cur_agent_index].pop_back();
//                ml_submap[cur_agent_index].push_back(curSubMap);
//                mMutex_ml_submap[cur_agent_index]->unlock();
                
                //记录强关联关系
                old_reCurSubMap->related_otherAgentSubMapIndex.push_back(curSubMap->global_index);
                old_reCurSubMap->related_otherAgentId.push_back(curSubMap->global_agent_id);
                curSubMap->related_otherAgentSubMapIndex.push_back(old_reCurSubMap->global_index);
                curSubMap->related_otherAgentId.push_back(old_reCurSubMap->global_agent_id);
                
                cout<<"多地图 子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
//                cout<<"测试 是不是同一个地址："<<curSubMap<<" , "<<(*iter)<<endl;
            }else if(isL1 || isL1_cur){
//                            说明一个是l1层 一个l2层 此时此刻的距离不准，看当前子地图的数量和老子地图的数量（代表帧的数量 l1+l2 说明视角大不大），大于某个阈值 只记录两个子地图的关联关系（两个人的视角 都比较全了，才有一次回环，说明比较偏），否则 一个人视角大一个视角小 （因为时间上差的比较远了，尽可能融合）融合
                if(old_reCurSubMap->isBigView(threshold_bigView) && curSubMap->isBigView(threshold_bigView)){
                    //记录强关联关系
//                    old_reCurSubMap->relatedSubMapIndex.push_back(curSubMap->global_index);
//                    curSubMap->relatedSubMapIndex.push_back(old_reCurSubMap->global_index);
                    
                    old_reCurSubMap->related_otherAgentSubMapIndex.push_back(curSubMap->global_index);
                    old_reCurSubMap->related_otherAgentId.push_back(curSubMap->global_agent_id);
                    
                    curSubMap->related_otherAgentSubMapIndex.push_back(old_reCurSubMap->global_index);
                    curSubMap->related_otherAgentId.push_back(old_reCurSubMap->global_agent_id);
                    
                    cout<<"多地图 强关联了"<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                }else{
//                    cout<<"多地图 子地图融合前="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
//
//                    FeatureSubMap *recurSubMap;
//                    recurSubMap=old_reCurSubMap;
//                    curSubMap->perfectFuse_subMap(old_reCurSubMap);
//                    recurSubMap->mMutexSubMapList.lock();
////                    *old_reCurSubMap=*curSubMap;
//                    (*iter_update)=curSubMap;
//                    recurSubMap->mMutexSubMapList.unlock();
//                    delete recurSubMap;


//                    mMutex_ml_submap[cur_agent_index]->lock();
//                    ml_submap[cur_agent_index].pop_back();
//                    ml_submap[cur_agent_index].push_back(curSubMap);
//                    mMutex_ml_submap[cur_agent_index]->unlock();
                    
                    old_reCurSubMap->related_otherAgentSubMapIndex.push_back(curSubMap->global_index);
                    old_reCurSubMap->related_otherAgentId.push_back(curSubMap->global_agent_id);
                    curSubMap->related_otherAgentSubMapIndex.push_back(old_reCurSubMap->global_index);
                    curSubMap->related_otherAgentId.push_back(old_reCurSubMap->global_agent_id);
                    
                    cout<<"多地图 子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                }
            }else if(!isL1 && !isL1_cur){
                //                            说明都是l2层 判断有没有一个子地图数量大于某个阈值，大于 都不能和l1匹配上，说明还是有一定的差距，或者中间有个拐弯 也应该更详细记录拐弯处的详情 不融合 只记录两个子地图的关联关系。 否则 融合
                if(old_reCurSubMap->isBigView(threshold_bigView) || curSubMap->isBigView(threshold_bigView)){
                    //记录强关联关系 暂时没写
//                    old_reCurSubMap->relatedSubMapIndex.push_back(curSubMap->global_index);
//                    curSubMap->relatedSubMapIndex.push_back(old_reCurSubMap->global_index);
                    
                    old_reCurSubMap->related_otherAgentSubMapIndex.push_back(curSubMap->global_index);
                    old_reCurSubMap->related_otherAgentId.push_back(curSubMap->global_agent_id);
                    
                    curSubMap->related_otherAgentSubMapIndex.push_back(old_reCurSubMap->global_index);
                    curSubMap->related_otherAgentId.push_back(old_reCurSubMap->global_agent_id);
                    
                    cout<<"多地图 强关联了"<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                }else{
//                    cout<<"多地图 子地图融合前="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
//                    FeatureSubMap *recurSubMap;
//                    recurSubMap=old_reCurSubMap;
//                    curSubMap->perfectFuse_subMap(old_reCurSubMap);
//                    recurSubMap->mMutexSubMapList.lock();
////                    *old_reCurSubMap=*curSubMap;
//                    (*iter_update)=curSubMap;
//                    recurSubMap->mMutexSubMapList.unlock();
//                    delete recurSubMap;

//                    mMutex_ml_submap[cur_agent_index]->lock();
//                    ml_submap[cur_agent_index].pop_back();
//                    ml_submap[cur_agent_index].push_back(curSubMap);
//                    mMutex_ml_submap[cur_agent_index]->unlock();
                    
                    old_reCurSubMap->related_otherAgentSubMapIndex.push_back(curSubMap->global_index);
                    old_reCurSubMap->related_otherAgentId.push_back(curSubMap->global_agent_id);
                    curSubMap->related_otherAgentSubMapIndex.push_back(old_reCurSubMap->global_index);
                    curSubMap->related_otherAgentId.push_back(old_reCurSubMap->global_agent_id);
                    
                    cout<<"多地图 子地图融合后="<<old_reCurSubMap->global_index<<" , "<<old_reCurSubMap->global_agent_id<<" , "<<curSubMap->global_index<<" , "<<curSubMap->global_agent_id<<endl;
                }
                
            }
        }
    }
    
}

