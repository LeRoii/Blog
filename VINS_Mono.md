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


estimator_node.cpp:

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

void predict(const sensor_msgs::ImuConstPtr &imu_msg):
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

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
处理观测值数据


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
上一帧IMU数据

`Eigen::Vector3d acc_1, gyr_1;`
当前帧IMU数据

void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
积分计算两个关键帧之间IMU测量的变化量： 旋转delta_q 速度delta_v 位移delta_p，加速度的bias linearized_ba 陀螺仪的Bias linearized_bg
同时维护更新预积分的Jacobian和Covariance,计算优化时必要的参数


中值积分
void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)

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
由角速度求得当前帧delta_q，角度变化量

```cpp
Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
```
当前帧加速度减bias，旋转当前帧delta_q，和上一帧加速度求均值

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
q<sub>k</sub>

`R_a_0_x`:<br>
[a<sub>k</sub>-b<sub>a<sub>k</sub></sub>]<sub>x</sub>

`_dt`:<br>
δt

`result_delta_q`:<br>
q<sub>k+1</sub>

`R_a_1_x`:<br>
[a<sub>k+1</sub>-b<sub>a<sub>k</sub></sub>]<sub>x</sub>

`R_w_x`:<br>
\frac{ω<sub>k+1</sub>+ω<sub>k</sub>}{2}
