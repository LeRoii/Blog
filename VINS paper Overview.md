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
* outlier rejection using RANSAC with fundamental matrix model
* projected to a unit sphere

Two criteria for keyframe selection

* average parallax of tracked feature between the current frame and the latest keyframe is beyond a certain threshold
* tracking quality, number of tracked features below a certain threshold

##### *B. IMU Pre-integration*

State Estimation其实是想估计任意时刻的全局坐标系下的位置和姿态, 也就是论文中的$p^\omega_{b_k}$,$v^\omega_{b_k}$,$q^\omega_{b_k}$，分别对应位置,速度,旋转四元数

即当前时刻的状态可以通过上一时刻状态来迭代求得. 但这里有个问题在于积分中有$R^\omega_t$,$q^{b_k}_t$, 它是t时刻在world系下的旋转, 是一个要估计的量, 后面估计的过程中, 这个量会被不断的更新改变, 那么每变一次就得重新积分. 把上次都乘一个旋转$R^{b_k}_\omega$ 转到$b_k$系下.<br>
把$R^\omega_t$从积分中抽出来, 这样不管$R^\omega_{b_k}$怎么变, 我们只需要一些简单的旋转矩阵相乘就能得到世界坐标系下的状态量. <br>

直观的理解是: 位置,姿态,速度等, 它们的绝对量是跟参考坐标系的选取有关的, 如果参考坐标系移动了, 那么这些量也会跟着改变, 但某两个时刻之间的相对量($p^{b_k}_{b_k+1}$,$v^{b_k}_{b_k+1}$,$q^{b_k}_{b_k+1}$)是跟坐标选取无关的, 不管在哪个坐标系下看, 相对变化是不变的, 所以可以事先把这些不变的量求出来. $p^\omega_{b_k}$,$q^\omega_{b_k}$,这些状态发生改变后, 可以通过矩阵乘法很容易的把相对量转成绝对量. 

更加简单的理解: 把相邻两帧图像之间的一段时间的IMU预先积分起来, 得到两帧之间的IMU相对运动, 也就是把多个IMU观测变成一个运动观测, 它是不随某一时刻的绝对位姿改变而发生改变的.

三个结果：<br>
$\hat{\alpha}^{b_k}_{b_k+1}$: body frame下，k帧到k+1帧position变化量的估计值(noisy measurement or estimate of a certain quantity)<br>
$\hat{\beta}^{b_k}_{b_k+1}$: body frame下，k帧到k+1帧velocity变化量的估计值<br>
$\hat{\gamma}^{b_k}_{b_k+1}$: body frame下，k帧到k+1帧orientation变化量的估计值<br>
其实还有两个bias<br>
IMU预积分的结果在后面优化的时候是作为约束<br>

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

$$q^{c_0}_{b_k} = q^{c_0}_{c_k} \otimes (q^b_c)^{-1}$$
$$s\bar P^{c_0}_{b_k} = s\bar P^{c_0}_{c_k} - R^{c_0}_{b_k}p^b_c$$

$(\cdot)^{c_0}$: the first camera frame，相机第一帧的坐标系，也作为SFM的参考坐标系<br>
下面的frame指的是坐标系，不是帧<br>
$q^{c_0}_{b_k}$: 相机第一帧frame 到 第k帧body(IMU)frame 的旋转<br>
$q^{c_0}_{c_k}$: 相机第一帧frame 到 第k帧camera frame 的旋转<br>
$q^b_c$: 按之前的说明应该是下标到上标的旋转（即camera frame 到 IMU frame的旋转），但是如果这样理解再对$q^b_c$取逆就解释不通了，所以感觉应该是上标到下标的旋转，即IMU frame 到 camera frame的旋转？？？？


##### *B. Visual-Inertial Alignment*<br>

1) Gyroscope Bias Calibration<br>
将预积分的结果作为约束，由下面的cost function求出新的gyro bias $b_\omega$，再用新的$b_\omega$ re-propagate 预积分项<br>
$$\min_{\delta b_\omega}\sum_{k\in \mathcal B}\left\|{q^{c_o}_{b_k+1}}^{-1}\otimes q^{c_o}_{b_k}\otimes \gamma^{b_k}_{b_k+1}\right\|^2$$
$$\gamma^{b_k}_{b_k+1}\approx \hat\gamma^{b_k}_{b_k+1}\otimes \begin{bmatrix} 1 \\ \frac{1}{2}J^\gamma_{b_\omega}\delta b_\omega \end{bmatrix}$$

$\mathcal B$ indexs all frames in the window

2) Veloity, Gravit Vector and Metric Scale Initialization<br>
由运动方程，IMU和Camera reference之间的转换关系，确定观测方程，得到最小二乘问题，求解<br>

3) Gravity Refinement <br>
refine teh gravity vector by constraining the magnitude
为了求世界坐标和camera坐标的关系？？？？<br>

4) Completing Initialization<br>
Translational components from the visual SFM will be scaled to metric units and fed for a tightly-coupled monocular VIO

### Tightly-Coupled Monocular VIO
##### *A) Formulation*<br>
visual-inertial bundle adjustment formulation
minimize the sum of prior and the Mahalanobis norm of all measurement residuals to obtain a maximum posteriori estimation<br>
$$\min_\chi \lbrace\|r_p-H_p\chi\|^2 + \sum_{k\in \mathcal B}\|r_{\mathcal B}(\hat z^{b_k}_{b_k+1},\chi)\|^2_{P^{b_k}_{b_k+1}} + \sum_{(l,j)\in \mathcal C}\rho(\|r_{\mathcal C}(\hat z^{c_j}_l,\chi)\|^2_{P^{c_j}_l})\rbrace$$

Huber norm is defined as:<br>
$$\rho (s) = \begin{cases} 1 & \text{s$\ge$1} \\ 2\sqrt s - 1 & \text {s$\lt$1}\end{cases}$$

由于误匹配等原因，某个误差项会给出错误的数据，结果就是把一条不该出现的边加到图里去了，而优化算法并不能识别这是错误的数据，它会把所有数据当作误差来处理，也会优化这条误差很大的边，并试图调整这条边所连接节点的估计值，由于这条错误的边误差很大，会抹平其他正确的边的影响，所以用核函数来确保误差不会大的太多


$r_{\mathcal B}(\hat z^{b_k}_{b_k+1},\chi)$: residuals for IMU measurements<br>
$r_{\mathcal C}(\hat z^{c_j}_l,\chi)$: residual for visual measurements<br>
$\lbrace r_\rho,H_\rho\rbrace$: prior information from marginalization<br>

##### *B) IMU Measurement Residual*<br>

##### *C) Visual Measurement Residual*<br>
Define the camera measurement residual on a unit sphere<br>

##### *D) Marginalization*<br>
To bound the computational complexity<br>
Selectively marginalize out IMU states and features from the sliding window<br>
Convert measurements corresponding to marginalized states into a prior????<br>
把丢弃的帧的IMU数据保存下来当作先验信息？？？？<br>

##### *E) Motion-only Visual-Inertial Bundle Adjustment for Camera-Rate State Estimation*<br>
Only optimize the poses and velocities of a fixed number of latest IMU states<br>
Treat feature depth, extrinsic parameters, bias, and old IMU states as constant values<br>

##### *F) IMU forward Propagation for IMU-Rate State Estimation*
Propagate the latest VIO estimate with the set of most recent IMU measurements to achieve IMU-rate performance<br>
The high-frequency state estimates can be utilized as state feedback for closed loop closure<br>

##### *G) Failure Detection and Recovery*<br>

* The number of features being tracked in the latest frame is less than a certain threshold<br>
* Large discontinuity in position or rotation between last two estimator outputs<br>
* Large change in bias or extrinsic parameters estimation<br>


### Relocalization
Drifts occur in global 3D position(x,y,z) and the rotation around the gravity direction(yaw)

##### *A. Loop Detection*
DBoW2<br>
Descriptors are treated as the visual word to query the visual database<br>
Keep all BRIEF descriptors for feature retrieving<br>
Discard raw image

##### *B. Feature Retrieval*
The connection between the local sliding window and the loop closure candidate is established by retrieving feature correspondences<br>
Correspondences are found by BRIEF descriptor matching<br>
Two-step geometric outlier rejection

##### *C. Tightly-Coupled Relocalization*
relocalization将滑窗和位姿图对应？？？？<br>
Optimize the sliding window

### Global Pose Graph Optimization
Utilize the relocalization results, to ensure the set of past poses are registered into a globally consistent configuration<br>
Visual-inertial setup redeners roll and pitch angles fully observable<br>
Only perform 4-DOF pose graph optimization(x,y,z,yaw)

##### *A. Adding Keyframes into the Pose Graph*
When a keyframe is marginalized out from the sliding window, it will be added to pose graph.<br>
This keyframe serves as a vertex in the pose graph and it connects with other vertexs by two types of edges<br>

1) Sequential Edge<br>
represents the relative transformation between two keyframes in the local sliding window, which value is taken directly from VIO<br>

2) Loop Closure Edge<br>
connects the marginalized keyframe with the loop closure frame

The value of the loop closure edge is obtained using results from relocalization

##### *B. 4-DOF Pose Graph Optimization*
The whole graph of sequential edges and loop closure edges are optimized by minimizing the cost function

Pose graph optimization and relocalization runs asynchronously in two separate threads

##### *C. Pose Graph Management*
Downsample process to maintain the pose graph database to a limited size

All keyframes with loop closure constraints will be kept,while other keyframes that are either too close or have very similar orientations to its neighbors may be removed