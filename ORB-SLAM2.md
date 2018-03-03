## ORB-SLAM2

link:[https://github.com/raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)

ORB-SLAM是一个基于特征点的实时单目SLAM系统，该系统包含了所有SLAM系统共有的模块：跟踪（Tracking）、建图（Mapping）、重定位（Relocalization）、闭环检测（Loop closing），由于ORB-SLAM系统是基于特征点的SLAM系统，故其能够实时计算出相机的轨线，并生成场景的稀疏三维重建结果。ORB-SLAM2在ORB-SLAM的基础上，还支持标定后的双目相机和RGB-D相机


### Prerequisites

#### opencv
```
sudo apt-get install libcv-dev
sudo apt-get clean
```
使用apt-get只安装库文件，无法选择版本，
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
#####`std::chrono`:
chrono是一个time library, 源于boost，现在已经是C++11标准，要使用chrono库，需要`#include<chrono>`，其所有实现均在std::chrono namespace下。

[http://www.cplusplus.com/reference/chrono/](http://www.cplusplus.com/reference/chrono/)<br>
[http://blog.csdn.net/u010977122/article/details/53258859](http://blog.csdn.net/u010977122/article/details/53258859)

`std::chrono::steady_clock`:<br>
为了表示稳定的时间间隔，后一次调用now()得到的时间总是比前一次的值大（这句话的意思其实是，如果中途修改了系统时间，也不影响now()的结果），每次tick都保证过了稳定的时间间隔

`std::chrono::monotonic_clock`:<br>
is provided for backwards compatibility only; new programs should use the `std::chrono::steady_clock` class or `std::chrono::high_resolution_clock` class instead.

#####`usleep()`
与`sleep()`类似，用于延迟挂起进程。进程被挂起放到reday queue。
是一般情况下，延迟时间数量级是秒的时候，尽可能使用sleep()函数。
如果延迟时间为几十毫秒（1ms = 1000us），或者更小，尽可能使用usleep()函数。这样才能最佳的利用CPU时间
```
void usleep(int micro_seconds);
unsigned sleep(unsigned seconds);
```

line100:<br>
why `if(ni>)`

line111:<br>
why use `sort`

##### unique_lock

C++多线程编程中通常会对共享的数据进行写保护，以防止多线程在对共享数据成员进行读写时造成资源争抢导致程序出现未定义的行为。通常的做法是在修改共享数据成员的时候进行加锁--mutex。在使用锁的时候通常是在对共享数据进行修改之前进行lock操作，在写完之后再进行unlock操作，进场会出现由于疏忽导致由于lock之后在离开共享成员操作区域时忘记unlock，导致死锁。

针对以上的问题，C++11中引入了std::unique_lock与std::lock_guard两种数据结构。通过对lock和unlock进行一次薄的封装，实现自动unlock的功能。

[http://www.cplusplus.com/reference/mutex/unique_lock/?kw=unique_lock](http://www.cplusplus.com/reference/mutex/unique_lock/?kw=unique_lock)
[http://blog.csdn.net/tgxallen/article/details/73522233](http://blog.csdn.net/tgxallen/article/details/73522233)