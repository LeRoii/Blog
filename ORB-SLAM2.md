## ORB-SLAM2

link:[https://github.com/raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)

ORB-SLAM是一个基于特征点的实时单目SLAM系统，该系统包含了所有SLAM系统共有的模块：跟踪（Tracking）、建图（Mapping）、重定位（Relocalization）、闭环检测（Loop closing），由于ORB-SLAM系统是基于特征点的SLAM系统，故其能够实时计算出相机的轨线，并生成场景的稀疏三维重建结果。ORB-SLAM2在ORB-SLAM的基础上，还支持标定后的双目相机和RGB-D相机


### Prerequisites

#### opencv
```
sudo apt-get install libcv-dev
sudo apt-get clean
```
apt-get装的opencv版本较低，
可以下载源码自己编译，参考：[https://www.cnblogs.com/dragonyo/p/6754599.html](https://www.cnblogs.com/dragonyo/p/6754599.html)
#### Pangolin
一个用于OpenGL显示/交互以及视频输入的一个轻量级、快速开发库
```
sudo apt-get install libglew-dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
```
在虚拟机上构建orb-slam可能会出现死机，为了防止死机，需要修改Pangolin/src中CMakeLists.txt，把其中关于OPENNI和OPENNI2的内容全部用#注释掉，
参考 [http://blog.csdn.net/qq_33251186/article/details/70313227](http://blog.csdn.net/qq_33251186/article/details/70313227)


#### Eigen3
a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.<br>
是一个开源线性代数库，提供有关矩阵的线性代数运算。

```
sudo apt-get install libeigen3-dev
```

### Build ORB-SLAM2 library and examples

防止死机，build.sh中的所有`make -j`改为`make`

```
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

build可能遇到的问题，参考[http://blog.csdn.net/wangshuailpp/article/details/70226534](http://blog.csdn.net/wangshuailpp/article/details/70226534)

### Monocular Examples
#### KITTI Dataset
1.Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php

2.execute
```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER

```
Change `KITTIX.yaml`by `KITTI00-02.yaml`, `KITTI03.yaml` or `KITTI04-12.yaml` for sequence 0 to 2, 3, and 4 to 12 respectively
Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. 
Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11.


### Source Code
#### mono_kitti.cc
`std::chrono`:
chrono是一个time library, 源于boost，现在已经是C++11标准，要使用chrono库，需要`#include<chrono>`，其所有实现均在std::chrono namespace下。

http://www.cplusplus.com/reference/chrono/<br>
http://blog.csdn.net/u010977122/article/details/53258859