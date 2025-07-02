//
//  MyThread.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2020/5/25.
//  Copyright © 2020 zx. All rights reserved.
//

#ifndef MyThread_hpp
#define MyThread_hpp

#include <stdio.h>

#include <iostream>
#include <string>
#include <pthread.h>
#include <unistd.h>

#include <cstdlib>



class MyThread {
  public:
    pthread_t tid;

  private:
    static pthread_mutex_t mutex;

  public:
    MyThread();
    int Create(void *Callback, void *args);
    int Join();

    static int InitMutex();
    static int LockMutex(const char *identifier);
    static int UnlockMutex(const char *identifier);
};



#endif /* MyThread_hpp */
