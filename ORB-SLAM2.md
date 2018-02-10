## ORB-SLAM2

link:[https://github.com/raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)

ORB-SLAM是一个基于特征点的实时单目SLAM系统，该系统包含了所有SLAM系统共有的模块：跟踪（Tracking）、建图（Mapping）、重定位（Relocalization）、闭环检测（Loop closing），由于ORB-SLAM系统是基于特征点的SLAM系统，故其能够实时计算出相机的轨线，并生成场景的稀疏三维重建结果。ORB-SLAM2在ORB-SLAM的基础上，还支持标定后的双目相机和RGB-D相机


### Prerequisites

#### opencv
```
sudo apt-get install libcv-dev
sudo apt-get clean
```

#### Pangolin
```
sudo apt-get install libglew-dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
```

#### Eigen3
a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

下载[source file](http://eigen.tuxfamily.org/index.php?title=Main_Page)
解压后进入文件夹
```
mkdir build_dir
cd build_dir
cmake ..
make
sudo make install
```