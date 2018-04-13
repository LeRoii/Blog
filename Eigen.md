Eigen 是一个基于 C++ 模板的线性代数库，提供了快速的有关矩阵的线性代数运算，还包括解方程等功能

这里主要通过Eigen来介绍一下四元数，包括四元数乘法，四元数表示旋转，用四元数旋转向量

Eigen API manual
http://eigen.tuxfamily.org/dox/

一篇对四元数解释的比较清楚的文档
https://www.3dgep.com/understanding-quaternions/

Eigen中四元数由一个模板类定义
```
template<typename _Scalar, int _Options>
class Eigen::Quaternion<_Scalar, _Options>
```

Quaternion constructor

Quaternion(const Scalar & w,
            const Scalar & x,
            const Scalar & y,
            const Scalar & z)

Constructs and initializes the quaternion  w+xi+yj+zk

Eigen自己typedef了两个常用的类型，Quaternionf 和Quaterniond ，分别对应float和double类的四元数，所以一般不会直接使用上面的模板类

下面的函数用来显示四元数的四个分量
coeffs()<br>
a read-only vector expression of the coefficients (x,y,z,w)

构造的时候实部在前(wxyz)，coeffs显示的时候实部在后(xyzw)

用四元数表示旋转

单位四元数可以表示空间中任意旋转，模值(wxyz平方和开根号)为1的四元数称为单位四元数

```cpp
//rotate 45deg around axis z
Eigen::AngleAxisd rotation_vector_q(M_PI/4,Eigen::Vector3d(0,0,1)); 
//rotate -45deg around axis z   
Eigen::AngleAxisd rotation_vector_p(-M_PI/4,Eigen::Vector3d(0,0,1));
//rotation expressed by Quaternion    
Eigen::Quaterniond q(rotation_vector_q);                                
Eigen::Quaterniond p(rotation_vector_p);
//vector v along axis x
Eigen::Vector3d v{1,0,0}; 
//vector v expressed by Quaternion                                        
Eigen::Quaterniond Quater_v(0,1,0,0);                                   
```

先定义两个旋转，分别是绕z轴转45和-45度，<br>
把两个旋转写成四元数形式(q,p)，<br>
定义一个三维空间向量，沿x轴长度为1<br>
三维向量v用四元数表示<br>


上面定义的两个旋转用四元数表示出来如下，可以验证p，q的模值都为1
```
Quaterniond q:
       0
       0
0.382683
 0.92388
Quaterniond p:
       -0
       -0
-0.382683
  0.92388
```


用四元数旋转向量
```cpp
cout<<"v rotated by q:"<<(q*v).transpose()<<endl;
cout<<"Quater_v rotated by q:"<<endl<<(q*Quater_v*q.inverse()).coeffs()<<endl;
```

result：
```
v rotated by q:0.707107 0.707107        0
Quater_v rotated by q:
0.707107
0.707107
       0
       0
```

三维空间任意点或向量可以用一个虚四元数(实部为0)表示，xyz分别对应三个虚部ijk

四元数q描述了一个旋转，
v是一个三维空间向量，

如果v用普通三维空间坐标表示(xyz)，
想要得到向量v经过q的旋转之后的结果，就用q乘v，类似于旋转矩阵乘向量，不可交换，必须是四元数乘向量，交换会出错
乘出的结果是一个三维向量

如果v用四元数表示(wxyz)，即上面代码中的Quater_v，
v经过q的旋转之后的结果由q* Quater_v * q.inverse()得到
乘出的结果是一个虚四元数，结果一样，只是把上面的向量用四元数的形式表示出来



四元数相乘表示旋转

```cpp
//first rotate p(-45deg along z) then rotate q(45deg along z) 
Eigen::Quaterniond q_multi_p = q*p; 
//first rotate q(45deg along z) then rotate p(-45deg along z)            
Eigen::Quaterniond p_multi_q = p*q;                                     

cout<<"v rotated by q_multi_p:"<<(q_multi_p*v).transpose()<<endl;       
cout<<"v rotated by p_multi_q:"<<(p_multi_q*v).transpose()<<endl;       

//rotate -90deg around axis z
Eigen::Quaterniond p_minus_q = q.inverse()*p; 
//rotate 90deg around axis z                          
Eigen::Quaterniond q_minus_p = p.inverse()*q;                           

cout<<"v rotated by p_minus_q:"<<(p_minus_q*v).transpose()<<endl;       
cout<<"v rotated by q_minus_p:"<<(q_minus_p*v).transpose()<<endl;       
```

result：
```
v rotated by q_multi_p:1 0 0
v rotated by p_multi_q:1 0 0
v rotated by p_minus_q:-2.22045e-16           -1            0
v rotated by q_minus_p:-2.22045e-16            1            0
```

两个四元数相乘，结果就是wijk多项式相乘，可以交换，但是结果不同，

q\*p表示两个旋转的结合，先转p再转q，结果还是一个旋转，结合上面的例子就是先绕z轴转-45，再绕z轴转45，结果就是没转，如果用(q\*p)乘一个向量，乘出来的还是原向量<br>
p*q和上面一样，只是先绕z轴转45，再转-45

q.inverse()\*p可以理解为两个旋转的差(p-q)，<br>
可以类比三维空间中向量的减法，可以想象成q到p的距离，结果也是个旋转，<br>
p是转-45，q是转45，p-q就是转-90，<br>
用(q.inverse()*p)的结果乘一个向量就等于让向量绕z轴转-90

p.inverse()*q和上面一样，等于q-p，就是p到q的距离，<br>
结果是绕z轴转正90

总结一下<br>
对于$p\otimes q$的理解，
如果q是单位四元数，等于做了两次旋转，得到的结果还是一个旋转
如果q是虚四元数，等于用p旋转q表示的向量，得到的结果是一个向量

