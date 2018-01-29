#### * vector和list分别怎么用

vector在内存中连续存储，随机存储效率高，插入删除或重新申请空间需要拷贝内存效率低
list和链表类似，在内存中不连续，随机存取效率低，插入删除效率高<br>
使用原则：<br>

vector适用：对象数量变化少，结构简单，随机访问频繁<br>
list适用：对象数量变化大，结构复杂，插入删除频繁<br>

#### * static_cast,dynamic_cast,reinterpret_cast,const_cast的区别

`static_cast<typeid>(expression)`<br>
把expression转换为typeid类型<br>
1. 用于基类派生类之间指针或引用的转换，派生类转为基类是安全的，基类转为派生类是不安全的<br>
2. 基本数据类型的转换（int，char，double，float）<br>
3. 空指针转为目标类型的空指针<br>
4. 任意类型的表达式转换成void类型<br>

一般的转换，用的最多，在编译时强制转换<br>

`dynamic_cast`<br>
用于类层次间的转换，还有类之间的交叉转换，基类转为派生类时比static_cast安全，因为有类型检查的功能，根据RTTI<br>

一般用于基类和派生类之间的转换，在运行时转换<br>

`reinterpret_cast`<br>
用于进行没有任何关联之间的转换，如一个字符转为一个整形，指针和整数之间的转换<br>

`const_cast`<br>
用来移除const，volatile属性，常量转为非常量<br>


#### *volatile有什么用<br>


使用volatile声明变量，说明该变量会在程序外被改变，系统总是重新从它所在的内存读取数据，而不是使用寄存器中的备份.<br>


#### *extern C用法

为了能够正确实现C++代码调用其他C语言代码, 它会指示编译器这部分代码按C语言进行编译和链接,cpp支持重载而c不支持,编译函数时会加入函数的参数信息,编译c语言时不会带入参数信息<br>
用g++编译cpp程序时,编译器会定义宏__cplusplus,可以根据是否定义了这个宏决定是否需要extern “C”<br>


为了能够正确实现C++代码调用其他C语言代码。加上extern "C"后，会指示编译器这部分代码按C语言的进行编译，而不是C++的。由于C++支持函数重载，因此编译器编译函数的过程中会将函数的参数类型也加到编译后的代码中，而不仅仅是函数名；而C语言并不支持函数重载，因此编译C语言代码的函数时不会带上函数的参数类型，一般之包括函数名。



场景：<br>
1.写一个c模块供以后使用,源文件事先编译好<br>
```c
#ifdef __cplusplus
extern “C”{
#endif
//some code
#ifdef __cplusplus
}
#endif
```

2.模块已经存在,在cpp代码中<br>
```cpp
extern “C”{
#include “module_written_in_c.h”
int funcInC(int);
}
```

#### * const用法<br>
```cpp
char greeting[ ] = “hello”;
char const *p = greeting;   //non-const pointer const data
const char *p = greeting;   //non-const pointer const data
char * const p = greeting;  //const pointer non-const data
const char * const p = greeting;    //const pointer const data;
```

**const出现在星号左边，指针指的数据是变量**<br>
**const出现在星号右边，指针本身是常量**<br>


STL迭代器类似指针,作用如同 T*<br>
声明迭代器为const如同声明指针为const, 即指针为常量,不可指向其他数据, <br>但指向的数据可以改变,即 T* const p<br>
如果希望迭代器指向的数据不可变,即 const T* p,需要用 const_iterator<br>
```cpp
std::vector<int> vec;
const std::vector<int>::iterator iter = vec.begin( );       //T* const p
*iter = 10;     //ok
iter++;         //error
std::vector<int>::const_iterator cIter;     //const T* p
*cIter = 10;        //error
cIter++;        //ok
```

const可以修饰：<br>
1. 局部和全局变量：变量值不会变<br>
2. 函数参数：参数不会被函数改变<br>
3. 函数返回值：返回值不能做为左值，多用于操作符重载，避免出现如下错误<br>
```cpp
class Rational{…};
const Rational operator* (const Rational& lhs, const Rational& rhs);
Rational a,b,c;
(a*b) = c;
```
4. 类的成员函数本身：只有被const修饰的成员函数才可以被const对象调用<br>
const成员函数中不允许对成员进行修改<br>
如果成员变量被mutable修饰，就可以被const成员函数修改<br>
如果成员是指针，const成员函数可以修改指针指向的内容，如<br>
```cpp
har* m_sName;
void setName(const string &s) const
{
    m_sName = s.c_str();    //error
    m_sName[1] = s[1];      //ok
}
```

#### Http中Get和Post的区别

* GET - 从指定的服务器中获取数据
* POST - 提交数据给指定的服务器处理

GET方法：
使用GET方法时，查询字符串（键值对）被附加在URL地址后面一起发送到服务器：

POST方法：
使用POST方法时，查询字符串在POST信息中单独存在，和HTTP请求一起发送到服务器<br>
**POST可能会修改服务器上的资源**

* 用一句话总结GET和POST的区别，GET的安全性较POST方式要差些，包含机密信息的话，建议用POST数据提交方式。在做数据查询时，建议用GET方式；而在做数据添加、修改、删除时，建议用POST方式。


#### Tcp中的粘包如何处理

1.什么是粘包现象

TCP粘包是指发送方发送的若干包数据到接收方接收时粘成一包，从接收缓冲区看，后一包数据的头紧接着前一包数据的尾。

2.为什么出现粘包现象

　　（1）发送方原因

　　我们知道，TCP默认会使用Nagle算法。而Nagle算法主要做两件事：

	1）只有上一个分组得到确认，才会发送下一个分组；
	2）收集多个小分组，在一个确认到来时一起发送。

　　所以，正是Nagle算法造成了发送方有可能造成粘包现象。

　　（2）接收方原因

　　TCP接收到分组时，并不会立刻送至应用层处理，或者说，应用层并不一定会立即处理；实际上，TCP将收到的分组保存至接收缓存里，然后应用程序主动从缓存里读收到的分组。这样一来，如果TCP接收分组的速度大于应用程序读分组的速度，多个包就会被存至缓存，应用程序读时，就会读到多个首尾相接粘到一起的包。

3.什么时候需要处理粘包现象


　　（1）如果发送方发送的多个分组本来就是同一个数据的不同部分，比如一个很大的文件被分成多个分组发送，这时，当然不需要处理粘包的现象；

　　（2）但如果多个分组本毫不相干，甚至是并列的关系，我们就一定要处理粘包问题了。比如，我当时要接收的每个分组都是一个有固定格式的商品信息，如果不处理粘包问题，每个读进来的分组我只会处理最前边的那个商品，后边的就会被丢弃。这显然不是我要的结果。

4.如何处理粘包现象

　　（1）发送方

　　对于发送方造成的粘包现象，我们可以通过关闭Nagle算法来解决，使用TCP_NODELAY选项来关闭Nagle算法。

　　（2）接收方

　　遗憾的是TCP并没有处理接收方粘包现象的机制，我们只能在应用层进行处理。

　　（3）应用层处理

　　应用层的处理简单易行！并且不仅可以解决接收方造成的粘包问题，还能解决发送方造成的粘包问题。

　　解决方法就是循环处理：应用程序在处理从缓存读来的分组时，读完一条数据时，就应该循环读下一条数据，直到所有的数据都被处理；但是如何判断每条数据的长度呢？

　　两种途径：

　　　　1）格式化数据：每条数据有固定的格式（开始符、结束符），这种方法简单易行，但选择开始符和结束符的时候一定要注意每条数据的内部一定不能出现开始符或结束符；

　　　　2）发送长度：发送每条数据的时候，将数据的长度一并发送，比如可以选择每条数据的前4位是数据的长度，应用层处理时可以根据长度来判断每条数据的开始和结束。

