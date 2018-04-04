Eigen::Quaternion::inversse():
return the quaternion describing the inverse rotation






Estimator:

`Vector3d Ps[(WINDOW_SIZE + 1)]:`
Position

`Vector3d Vs[(WINDOW_SIZE + 1)]:`
Velocity

`Matrix3d Rs[(WINDOW_SIZE + 1)]:`
Rotation

`Vector3d Bas[(WINDOW_SIZE + 1)]:`
accelerometer Bias,对应linear_acceleration在x,y,z的bias
   
`Vector3d Bgs[(WINDOW_SIZE + 1)]:`
gyroscope Bias,对应angular velocity在x,y,z的bias

`vector<double> dt_buf[(WINDOW_SIZE + 1)]：`
time interval between two frames

`vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)]:`
linear_acceleration

`vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)]:`
angular velocity

`IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)]:`
integration for every frame in slide window

`Vector3d acc_0:`
raw linear_acceleration measurment IMU加速度观测值

`Vector3d gry_0:`
raw angular velocity measurment 角速度观测值

Matrix3d ric[NUM_OF_CAM]:


Vector3d tic[NUM_OF_CAM]:


#### estimator_node.cpp:

`Eigen::Vector3d tmp_P`
linear_acceleration 预测值

`Eigen::Quaterniond tmp_Q:`
temporal quaternion???
angular velocity 预测值

`Eigen::Vector3d tmp_V`
velocity 预测值

`Eigen::Vector3d tmp_Ba:`
linear_acceleration bias

`Eigen::Vector3d tmp_Bg:`
angular velocity bias

`Eigen::Vector3d acc_0：`
linear_acceleration上一帧的观测值

`Eigen::Vector3d gyr_0：`
angular velocity上一帧的观测值

```cpp
void predict(const sensor_msgs::ImuConstPtr &imu_msg):
```

测未考虑观测噪声的p、v、q值，这里计算得到的pvq是估计值，注意是没有观测噪声和偏置的结果，作用是与下面预积分计算得到的pvq（考虑了观测噪声和偏置）做差得到残差

```cpp
Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba - tmp_Q.inverse() * estimator.g);
```
acc_0-bias-重力加速度g得到真实值，结果和tmp_Q相乘，得到一个旋转后的向量，旋转轴和角度由tmp_Q给出

`Eigen::Vector3d un_acc_0：`
？？？？？

```cpp
Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
```
gyr_0和angular_velocity求平均？？？然后减去Bias

`Eigen::Vector3d un_gyr:`
angular_velocity真实值？？？

```cpp
tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);
```
两帧之间旋转的变化量(un_gyr * dt)，除2，用quaternion表示，实部为1，赋给tmp_Q

```cpp
Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba - tmp_Q.inverse() * estimator.g);
```
linear_acceleration测量值减bias，减重力加速度，乘tmp_Q（考虑两帧间发生的旋转）

Eigen::Vector3d un_acc_1：
linear_acceleration真实值？？？？

```cpp
Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
```
un_acc_0和un_acc_1取平均得到最后的真实值？？？

```cpp
tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
tmp_V = tmp_V + dt * un_acc;
```
由刚得到的加速度更新位置和速度

```
acc_0 = linear_acceleration;
gyr_0 = angular_velocity;
```
更新观测值？？？？

#### estimator.cpp

```cpp
void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
```
处理观测值数据

`map<double, ImageFrame> all_image_frame;`
时间戳+Img信息，ImageFrame中有所有特征点和预积分信息


#### integration_base.h
`ACC_N`
accelerometer measurement noise standard deviation(标准差)

`GYR_N`
gyroscope measurement noise standard deviation

`ACC_W`
accelerometer bias random work noise standard deviation

`GYR_W`
gyroscope bias random work noise standard deviation

`Eigen::Vector3d acc_0, gyr_0;`
前一帧IMU观测值

`Eigen::Vector3d acc_1, gyr_1;`
当前帧IMU观测值

`const Eigen::Vector3d linearized_acc, linearized_gyr;`
？？？？？

`Eigen::Vector3d linearized_ba, linearized_bg;`
前一帧加速度bias，gyro bias

`Eigen::Vector3d delta_p;`
前一帧位置变化量


`Eigen::Quaterniond delta_q;`
前一帧角度变化量

`Eigen::Vector3d delta_v;`
前一帧速度变化量


```cpp
void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
```
积分计算两个关键帧之间IMU测量的变化量： 旋转delta_q 速度delta_v 位移delta_p，加速度的bias linearized_ba 陀螺仪的Bias linearized_bg
同时维护更新预积分的Jacobian和Covariance,计算优化时必要的参数
```cpp
dt = _dt;
acc_1 = _acc_1;
gyr_1 = _gyr_1;
```
更新当前帧的$\delta$t，加速度和角度的测量值

```cpp
Vector3d result_delta_p;
Quaterniond result_delta_q;
Vector3d result_delta_v;
Vector3d result_linearized_ba;
Vector3d result_linearized_bg;
```
临时变量，存放中值积分结果

**这里的delta_p等是累积的变化量，也就是说是从i时刻到当前时刻的变化量，这个才是最终要求的结果（为修正偏置一阶项），result_delta_p等只是一个暂时的变量**

中值积分
```cpp
void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
```

```cpp
Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
```
上一帧加速度减bias，旋转delta_q(上一帧的旋转变化量)

```cpp
Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
```
两帧角速度求均值减bias

```cpp
result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
```
由角速度求得当前帧delta_q(角度变化量)

```cpp
Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
```
当前帧加速度测量值减bias，旋转当前帧delta_q，和上一帧加速度求均值

```cpp
result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
result_delta_v = delta_v + un_acc * _dt;
```
当前帧位置速度的变化量

```cpp
F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt + 
-0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() -R_w_x * _dt) * _dt * _dt;
```

`delta_q`:<br>
$q_k$：前一帧orientation变化量


`R_a_0_x`:<br>
$[a_k-b_{a_k}]_\times$<br>
$a_k$：前一帧加速度测量值<br>
$b_{a_k}$：前一帧加速度bias

`_dt`:<br>
$\delta$t：两帧时间差<br>

`result_delta_q`:<br>
$q_{k+1}$：当前帧orientation变化量

`R_a_1_x`:<br>
$[a_{k+1}-b_{a_k}]_\times$<br>
$a_{k+1}$：当前帧加速度测量值<br>
$b_{a_k}$：前一帧加速度bias<br>

  
`R_w_x`:<br>
$[\frac{ω_{k+1}+ω_k}{2}-b_{g_k}]_\times$<br>
$\omega_{k+1}$：当前帧orientation测量值<br>
$\omega_k$：前一帧orientation测量值<br>
$b_{g_k}$：前一帧orientation bias<br>


$\delta$t：两帧时间差 `_dt`<br>
$q_k$：前一帧orientation变化量 `delta_q`<br>
$q_{k+1}$：当前帧orientation变化量 `result_delta_q`<br>
$a_k$：前一帧加速度测量值 `_acc_0`<br>
$a_{k+1}$：当前帧加速度测量值 `_acc_1`<br>
$b_{a_k}$：前一帧加速度bias `linearized_ba`<br>
$\omega_k$：前一帧orientation测量值 `_gyr_0`<br> 
$\omega_{k+1}$：当前帧orientation测量值 `_gyr_1`<br>
$b_{g_k}$：前一帧gyro bias `linearized_bg`<br>
`delta_p` 前一帧position变化量<br>


#### initial_ex_rotation.cpp
calibrate camera and IMU

`vector< Matrix3d > Rc;`
`vector< Matrix3d > Rimu;`
`vector< Matrix3d > Rc_g;`
`Matrix3d ric;`


#### initial_sfm.cpp
`Matrix3d c_Rotation[frame_num];`
`Vector3d c_Translation[frame_num];`
`Quaterniond c_Quat[frame_num];`

大小为frame_num的数组但是内容从l开始，l之前为0

BA的目的，最小化重投影误差


松耦合和紧耦合，松耦合分别单独计算出IMU测量得到的状态和视觉里程计得到的状态然后融合，紧耦合则将IMU测量和视觉约束信息放在一个非线性优化函数中去优化．紧耦合的框架使得IMU数据可以对视觉里程计进行矫正，同时视觉里程计信息也可以矫正IMU的零偏，因此一般认为紧耦合的定位精度较高．个人认为松耦合和滤波融合的方法类似，紧耦合则主要基于非线性优化．

## VINS paper Overview

### Notations and frame definitions
$(\cdot)^b$: body(IMU) frame<br>
$(\cdot)^c$: camera frame<br>
$(\cdot)_k$: discrete time k<br>
$(\cdot)_t$: continuous time t<br>

$p$: position<br>
$v$: velocity<br>
$q$: rotation<br>
$a$: acceleration<br>
$\omega$: angular velocity<br>
$b_a$: accelerometer bias<br>
$b_\omega$: gyroscope bias<br>
$n$: noise<br>

$p^w_b$: translation from the body frame to the world frame<br>
$q^w_b$: rotation from the body frame to the world frame<br>
$b_k$: the body frame while taking the $k^th$ image<br>
$c_k$: the camera frame while taking the $k^th$ image<br>

$\otimes$: the mulitiplication between two quaternions<br>
$\hat{(\cdot)}$: noisy measurement or estimate of a certain quantity<br>
$s$: scaling parameter




### Measurement Preprocessing
For visual measurements

* track features between consecutive frames
* detect new features in the latest frame

For IMU measurements

* pre-integrate between two consecutive frames<br>
IMU measurements are affected by both bias and noise<br>
take bias into account in the pre-integration process

##### *A. Vision Processing Front-end*
* Features are tracked by the KLT sparse optical flow algorithm
* 100-300 features in each image
* 2D features are firstly undistorted
* outlier rejection using RANSAN with fundamental matrix model
* projected to a unit sphere

Two criteria for keyframe selection

* average parallax of tracked feature between the current frame and the latest keyframe is beyond a certain threshold
* tracking quality, number of tracked features below a certain threshold

##### *B. IMU Pre-integration*

State Estimation其实是想估计任意时刻的全局坐标系下的位置和姿态, 也就是论文中的$p^\omega_{b_k}$,$v^\omega_{b_k}$,$q^\omega_{b_k}$ 分别对应位置,速度,旋转四元数

即当前时刻的状态可以通过上一时刻状态来迭代求得. 但这里有个问题在于积分中有$R^\omega_t$,$q^{b_k}_t$, 它是t时刻在world系下的旋转, 是一个要估计的量, 后面估计的过程中, 这个量会被不断的更新改变, 那么每变一次就得重新积分. 把上次都乘一个旋转$R^{b_k}_\omega$ 转到$b_k$系下.<br>
把$R^\omega_t$从积分中抽出来, 这样不管$R^\omega_{b_k}$怎么变, 我们只需要一些简单的旋转矩阵相乘就能得到世界坐标系下的状态量. <br>

直观的理解是: 位置,姿态,速度等, 它们的绝对量是跟参考坐标系的选取有关的, 如果参考坐标系移动了, 那么这些量也会跟着改变, 但某两个时刻之间的相对量($p^{b_k}_{b_k+1}$,$v^{b_k}_{b_k+1}$,$q^{b_k}_{b_k+1}$)是跟坐标选取无关的, 不管在哪个坐标系下看, 相对变化是不变的, 所以可以事先把这些不变的量求出来. $p^\omega_{b_k}$,$q^\omega_{b_k}$,这些状态发生改变后, 可以通过矩阵乘法很容易的把相对量转成绝对量. 

更加简单的理解: 把相邻两帧图像之间的一段时间的IMU预先积分起来, 得到两帧之间的IMU相对运动, 也就是把多个IMU观测变成一个运动观测, 它是不随某一时刻的绝对位姿改变而发生改变的.

三个结果：<br>
$\hat{\alpha}^{b_k}_{b_k+1}$: body frame下，k帧到k+1帧position变化量的估计值(noisy measurement or estimate of a certain quantity)<br>
$\hat{\beta}^{b_k}_{b_k+1}$: body frame下，k帧到k+1帧velocity变化量的估计值<br>
$\hat{\gamma}^{b_k}_{b_k+1}$: body frame下，k帧到k+1帧orientation变化量的估计值<br>
其实还有两个bias<br>
IMU预积分的结果在后面优化的时候是作为约束？？？<br>

### Estimator Initialization
to solve the scaling parameter

* Highly nonlinear system<br>
Scale is not directly observable<br>
Stationary initial condition? inapplicable in real-world application<br>
Loosely-coupled sensor fusion method to get initial values<br>
Bootstrap the vision-only SLAM by 8-pts or 5-pts algorithms, or estimating Homogeneous matrices. Then align metric IMU pre-integration with the Visual-only SFM results. To recover scale,gravity velocity even bias.<br>
Ignore accelerometer bias terms in the initial step.<br> 
Due to large magnitude of the gravity vector and short during of initialization, the bias terms are hard to be observed 

##### *A. Sliding Window Vision-only SFM*
to estimate a graph of up-to-scale camera pose and feature positions

maintain a sliding window<br>

1.check feature correspondences between the latest frame and all previous frames<br>

if find stable feature tracking and sufficient parallax between the latest frame and any other frames in the window<br>

Recover the relative rotation and up-to-scale translation between these two frames using 5-pts algorithm.<br>

Otherwise, keep the latest frame in the window and wait for new frame<br>

2.If 5-pts algorithm success, arbitrarily set the scale and triangulate all features in these two frames.<br>

3.Based on these triangulated features, performe a PnP method to estimate poses of all other frames in the window.<br>

4.A global full bundle adjustment to minimize the total reprojection error of  all feature observations

##### *B. Visual-Inertial Alignment*<br>

1) Gyroscope Bias Calibration<br>
将预积分的结果作为约束，由cost function(15)求出新的gyro bias $b_\omega$，再用新的$b_\omega$ re-propagate 预积分项<br>
$$\min_{\delta b_\omega}\sum_{k\in \mathcal B}\left\|{q^{c_o}_{b_k+1}}^{-1}\otimes q^{c_o}_{b_k}\otimes \gamma^{b_k}_{b_k+1}\right\|^2$$
$$\gamma^{b_k}_{b_k+1}\approx \hat\gamma^{b_k}_{b_k+1}\otimes \begin{bmatrix} 1 \\ \frac{1}{2}J^\gamma_{b_\omega}\delta b_\omega \end{bmatrix}$$

2) Veloity, Gravit Vector and Metric Scale Initialization<br>
由运动方程，IMU和Camera reference之间的转换关系，确定观测方程，得到最小二乘问题，求解<br>

3) Gravity Refinement <br>
为了求世界坐标和camera坐标的关系？？？？<br>

4) Completing Initialization<br>
Translational components from the visual SFM will be scaled to metric units and fed for a tightly-coupled monocular VIO

##### Tightly-Coupled Monocular VIO
###### *A) Formulation*<br>
visual-inertial bundle adjustment formulation
minimize the sum of prior and the Mahalanobis norm of all measurement residuals to obtain a maximum posteriori estimation<br>

###### *B) IMU Measurement Residual*<br>

###### *C) Visual Measurement Residual*<br>
Define the camera measurement residual on a unit sphere<br>

###### *D) Marginalization*<br>
To bound the computational complexity<br>
Selectively marginalize out IMU states and features from the sliding window<br>
Convert measurements corresponding to marginalized states into a prior????<br>
把丢弃的帧的IMU数据保存下来当作先验信息？？？？<br>

###### *E) Motion-only Visual-Iertial Bundle Adjustment for Camera-Rate State Estimation*<br>
Only optimize the poses and velocities of a fixed number of latest IMU states<br>
Treat feature depth, extrinsic parameters, bias, and old IMU states as constant values<br>

###### *F) IMU forward Propagation for IMU-Rate State Estimation*
Propagate the latest VIO estimate with the set of most recent IMU measurements to achieve IMU-rate performance<br>
The high-frequency state estimates can be utilized as state feedback for closed loop closure<br>

###### *G) Failure Detection and Recovery*<br>

* The number of features being tracking in the latest frame is less than a certain threshold<br>
* Large discontinuity in position or rotation between last two estimator outputs<br>
* Large change in bias or extrinsic parameters estimation<br>


##### Relocalization
Drifts occur in global 3D position(x,y,z) and the rotation around the gravity direction(yaw)

###### *A. Loop Detection*
DBoW2<br>
Descriptors are treated as the visual word to query the visual database<br>
Keep all BRIEF descriptors for feature retrieving<br>
Discard raw image

###### *B. Feature Retrieval*
The connection between the local sliding window and the loop closure candidate is established by retrieving feature correspondences<br>
Correspondences are found by BRIEF descriptor matching<br>
Two-step geometric outlier rejection

###### *C. Tightly-Coupled Relocalization*
relocalization将滑窗和位姿图对应？？？？<br>
Optimize the sliding window

##### Global Pose Graph Optimization
Utilizing the relocalization results, to ensure the set of past poses are registered into a globally consistent configuration<br>
Visual-inertial setup redeners roll and pitch angles fully observable<br>
Only perform 4-DOF pose graph optimization(x,y,z,yaw)

###### *A. Adding Keyframes into the Pose Graph*
When a keyframe is marginalized out from the sliding window, it will be added to pose graph.<br>
This keyframe serves as a vertex in the pose graph and it connects with other vertexs by two types of edges<br>

1) Sequential Edge<br>
represents the relative transformation between two keyframes in the local sliding window, which value is taken directly from VIO<br>

2) Loop Closure Edge<br>
connects the marginalized keyframe with the loop closure frame

The value of the loop closure edge is obtained using results from relocalization

###### *B. 4-DOF Pose Graph Optimization*
The whole graph of sequential edges and loop closure edges are optimized by minimizing the cost function

Pose graph optimization and relocalization runs asynchronously in two separate threads

###### *C. Pose Graph Management*
Downsample process to maintain the pose graph database to a limited size

All keyframes with loop closure constraints will be kept,while other keyframes that are either too close or have very similar orientations to its neighbors may be removed

### Least squares
A standard approach to approximate the solution of overdetermined system(sets of equations in which there are more equationss than unkowns),"Least squares" means that the overall sloution minimize the sum of the squares of the residuals made in the results of every single equation
Two categories:
* Linear
* Nonlinear
solved by iterative refinement,at each iteration the system is approximated by a linear one<br>
depending on whether or not the residuals are linear 
