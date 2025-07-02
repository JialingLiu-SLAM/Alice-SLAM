//
//  UnionFind.h
//  VINS_MapFusion
//
//  Created by 张剑华 on 2022/4/1.
//  Copyright © 2022 zx. All rights reserved.
//

#ifndef UnionFind_h
#define UnionFind_h

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <stack>
#include <iomanip>
using namespace std;
 
class UnionFind {
    int father[1001]; // 0 ~ 1000  记录上级/老大
    int contain[1001]; // 包含多少小弟
    int minIndex; // 范围
    int maxIndex; // 范围
    int cnt; //连通分量的个数
    
//    int minId[1001];//每个人对应的最小id 其实只有root对应的最小id是准确的
//    double pitch[1001];//到最小id的pitch
//    double roll[1001];//到最小id的roll
 
public :
    UnionFind() {}
    UnionFind(int minIndex, int maxIndex) {
        // 1 ~ 4
        this->minIndex = minIndex;
        this->maxIndex = maxIndex;
        this->cnt = maxIndex - minIndex + 1; // 连通分量的个数
 
        for (int i = minIndex; i <= maxIndex ; i++) {
            father[i] = -1;
            contain[i] = 1;
//            minId[i]=i;
//            pitch[i]=0;
//            roll[i]=0;
        }
 
    }
 
    int getRoot(int x) {
        if (father[x] == -1) {
            return x;  // 自己就是老大
        } else {
            //自己不是老大，自己有上级
            // 上级是set老大 还是 图上的父节点？
            int d = father[x];
            int root = getRoot(d);
            if (d != root) {
                father[x] = root; // 直接跟root算了，不跟图上的父节点
                contain[d] -= contain[x]; // contain少了，投奔root了
            }
            return root;
        }
 
    }
 
    bool isConnected(int x, int y) {
        return getRoot(x) == getRoot(y);
    }
 
//    bool connect(int x, int y, double pitch_2d_3d, double roll_2d_3d) {
//        int xRoot = getRoot(x);
//        int yRoot = getRoot(y);
//        if (xRoot == yRoot) {
//            //cout << "已经在同一个set里面了" << endl;
//            return false; // 已经在同一个set里面了，已经在同一个连通分量里面了
//        } else {
//            if(contain[x] >= contain[y]) {
//                // x 人多势大， y投奔x
//                father[yRoot] = xRoot;
//                contain[xRoot] += contain[yRoot];
//                
//                //更新最小id 不可能出现等于
//                if(minId[xRoot]>minId[yRoot]){
//                    minId[xRoot]=minId[yRoot];
//                    pitch[xRoot]=pitch_2d_3d*(pitch[y]) ;
//                    roll[xRoot]= roll_2d_3d*(pitch[y]);
//                }else{
//                    minId[yRoot]=minId[xRoot];
//                    pitch[xRoot]=(pitch[y])*pitch_2d_3d ;
//                    roll[xRoot]= (pitch[y])*roll_2d_3d;
//                }
//            } else {
//                // y人多势大，x投奔y
//                father[xRoot] = yRoot;
//                contain[yRoot] += contain[xRoot];
//            }
//        }
//        cnt --; //连通分量少1
//        return true;
//    }
    
    bool connect(int x, int y) {
        int xRoot = getRoot(x);
        int yRoot = getRoot(y);
        if (xRoot == yRoot) {
            //cout << "已经在同一个set里面了" << endl;
            return false; // 已经在同一个set里面了，已经在同一个连通分量里面了
        } else {
            if(contain[x] >= contain[y]) {
                // x 人多势大， y投奔x
                father[yRoot] = xRoot;
                contain[xRoot] += contain[yRoot];
               
            } else {
                // y人多势大，x投奔y
                father[xRoot] = yRoot;
                contain[yRoot] += contain[xRoot];
            }
        }
        cnt --; //连通分量少1
        return true;
    }
 
    int getCnt() {
        return cnt;
    }
 
 
    void show() {
        cout << "---目前情况---" << endl;
        cout << "father :";
        for (int i = minIndex; i <= maxIndex; i++) {
            cout << setw(4) << father[i] << "\t";
        }
        cout << endl;
 
        cout << "index  :";
        for (int i = minIndex; i <= maxIndex; i++) {
            cout << setw(4)  << i << "\t";
        }
        cout << endl << endl;
 
 
        cout << "contain :";
        for (int i = minIndex; i <= maxIndex; i++) {
            cout << setw(4)  << contain[i] << "\t";
        }
        cout << endl;
 
        cout << "index  :";
        for (int i = minIndex; i <= maxIndex; i++) {
            cout << setw(4)  << i << "\t";
        }
        cout << endl;
 
        cout << "---打印完毕---" << endl << endl;
    }
 
 
 
};
 


#endif /* UnionFind_h */
