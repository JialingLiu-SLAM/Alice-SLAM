# Alice-SLAM

Accurate and Lite-Communication Collaborative SLAM for Resource-Constrained Multi-Agent

Related Papers

- Collaborative visual inertial slam for multiple smart phones, Jialing Liu, Ruyu Liu, Kaiqi Chen, Jianhua Zhang, Dongyan Guo, IEEE International Conference on Robotics and Automation (ICRA, 2021)
- Robust and accurate multi-agent slam with efficient communication for smart mobiles, Jialing Liu, Kaiqi Chen, Ruyu Liu, Yanhong Yang, Zhenhua Wang, Jianhua Zhang, IEEE International Conference on Robotics and Automation (ICRA, 2022)
- Alice-SLAM: Accurate and Lite-Communication Collaborative SLAM for Resource-Constrained Multi-Agent, Jialing Liu, Kaiqi Chen, Ruyu Liu, Xu Cheng, Jianhua Zhang, Shengyong Chen, Houxiang Zhang, Arash Ajoudani, IEEE Journal on Selected Areas in Communications (JSAC, 2025)

If you use Alice-SLAM for your academic research, please cite at least one of our related papers. The source code is available in the **master** branch.

## 1. Build

The code has been compiled on macOS Big Sur with Xcode and tested on iPhone6s, iPhone7p, iPhone8 or iPad Pro 2020.

Alice-SLAM consists of two systems: client and server. The client is VINS-Mobile, which is modified based on [VINS-Mobile](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile). The server is VINS-MapFusion. The compilation conditions of the client are the same as those of [VINS-Mobile](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile). The server uses a special DBoW, which has been modified by us.

The Client compilation (same with VINS-Mobile):

1.1 Install boost for macOS

```
brew install boost
```

1.2 Download specific **opencv2.framework** from [here](http://uav.ust.hk/storage/opencv2.framework.zip), then unzip it to VINS_ThirdPartyLib/opencv2.framework **(Please make sure you haven't installed opencv for your OSX)**

1.3 In your Xcode, select **Product**-> **Scheme**-> **Edit Scheme**-> **Run**-> **Info**, set **Build Configuration** to **Release** (not debug)

1.4 **Select your device** at upper left corner, then **choose your device size** at Main.storyboard, build and run

The Server compilation:

1.1 In your Xcode, build and run

## 2. Acknowledgements

We are based on [VINS-Mobile](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile), use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBow](https://github.com/dorian3d/DBoW2) for loop detection.

## 3. Licence

We are still working for improving the code readability. Welcome to contribute to Alice-SLAM or ask any issues via Github or contacting Jialing Liu <liujialing98@hotmail.com>

