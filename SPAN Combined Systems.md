## SPAN Combined Systems
Synchronous Position, Attitude and Navigation

**includes:**

### GNSS
Global Navigation Satellite System<br>
全球导航卫星系统，它是泛指所有的卫星导航系统，包括美国的GPS、俄罗斯的Glonass、欧洲的Galileo、中国的北斗

#### 特点
* 全球覆盖
* 精度高，误差不积累，输出不连续

### INS
Inertial Navigation System, 惯性导航

**includes:**
#### [Gyroscope(陀螺仪)](https://en.wikipedia.org/wiki/Gyroscope)
measuring or maintaining orientation and angular velocity

* 机械陀螺仪
* 震动陀螺仪
* 光学陀螺仪
* MEMS陀螺仪

#### [Accelerometer(加速计)](https://en.wikipedia.org/wiki/Accelerometer)
measure [proper acceleration](https://en.wikipedia.org/wiki/Proper_acceleration)

#### 特点
* 自主的，不依赖外部信息，不受外界电磁干扰
* 可以提供连续的位置，速度，航向信息，短期精度高
* 导航信息经过积分产生，误差随时间积累，没有时间信息

#### 重要参数
* Bias Stability(零偏稳定性)<br>
	当角速率为零时，输出量围绕其均值的离散程度，以规定时间内输出量的标准方差相应的等效输入角速率表示
* Bias(零偏)
	静止时输出量的平均值相应的等效输入角速率，理想状态下为地球自转角速度的分量


## RTK
Real-Time Kinematic，实时动态差分

a technique used to enhance the precision of position data derived from satellite-based positioning systems (global navigation satellite systems, GNSS) such as GPS, GLONASS, Galileo, and BeiDou. It uses measurements of the phase of the signal's carrier wave, rather than the information content of the signal, and relies on a single reference station or interpolated virtual station to provide real-time corrections, providing up to centimetre-level accuracy.<br>
由基准站通过数据链实时将其载波观测量及站坐标信息一同传送给用户站。用户站接收GPS卫星的载波相位 与来自基准站的载波相位，并组成相位差分观测值进行实时处理，能实时给出厘米级的定位结果。