//
//  MyThread.cpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/25.
//  Copyright © 2020 zx. All rights reserved.
//

#include "MyThread.hpp"


//Need to actually "allocate" static member
pthread_mutex_t MyThread::mutex;

MyThread::MyThread() {
}

int MyThread::Create(void *Callback, void *args) {
  int tret=0;
 
  //Supercreepy typecast
  tret = pthread_create(&this->tid, NULL, (void *(*)(void *))Callback, args);

  if(tret) {
    std::cerr << "Error while creating threads." << std::endl;
    return tret;
  }
  else {
    std::cout << "Thread successfully created." << std::endl;
    return 0;
  }
}

int MyThread::Join() {
  pthread_join(this->tid, NULL);
  return 0;
}

int MyThread::InitMutex() {
  
  if(pthread_mutex_init(&MyThread::mutex, NULL) < 0) {
    std::cerr << "Error while initializing mutex" << std::endl;
    return -1;
  }
  else {
    std::cout << "Mutex initialized." << std::endl;
    return 0;
  }
}

/*
    LockMutex():
        Blocks until mutex becomes available
*/
int MyThread::LockMutex(const char *identifier) {
//  std::cout << identifier << " is trying to acquire the lock..." << std::endl;
  if(pthread_mutex_lock(&MyThread::mutex) == 0) {
//    std::cout << identifier << " acquired the lock!" << std::endl;
    return 0;
  }
  else {
   std::cerr << "Error while " << identifier << " was trying to acquire the lock" << std::endl;
   return -1;
  }
}

int MyThread::UnlockMutex(const char *identifier) {
//  std::cout << identifier << " is trying to release the lock..." << std::endl;
  if(pthread_mutex_unlock(&MyThread::mutex) == 0) {
//    std::cout << identifier << " released the lock!" << std::endl;
    return 0;
  }
  else {
   std::cerr << "Error while " << identifier << " was trying to release the lock" << std::endl;
   return -1;
  }
}

