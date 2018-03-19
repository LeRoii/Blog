[toc]


### mv

移动文件夹
```
mv [-fiu] source dest
```

##### 参数说明：<br>
-f: force，强制直接移动而不询问<br>
-i: 若目标文件(destination)已经存在，就会询问是否覆盖<br>
-u: 若目标文件已经存在，且源文件比较新，才会更新<br>

##### eg:<br>
 A文件夹下的blog文件夹移到B文件夹下<br>
如果当前路径在B文件夹下<br>
```
mv A/* ./
```

### [wget](http://man.linuxde.net/wget)

用来从指定的URL下载文件

##### eg:<br>
下载单个文件
```
wget http://download.redis.io/releases/redis-4.0.8.tar.gz
```

下载并以指定文件名保存
```
wget -O wordpress.zip http://www.linuxde.net/download.aspx?id=1080
```


`$()`：这个小括号里放的是命令，和``反引号作用一样，执行这个命令

`${}`：这里面放的是变量，用来引用的

`#`: 代表 root权限

`$`: 代表普通用户

### [chown](http://man.linuxde.net/chown)
改变某个文件或目录的所有者和所属的组，该命令可以向某个用户授权，使该用户变成指定文件的所有者或者改变文件所属的组。

##### eg:<br>
```
sudo chown -R $(whoami) /usr/local
```

### [tar](https://www.cnblogs.com/xiaochina/p/5801959.html)
可以为linux的文件和目录创建档案。利用tar，可以为某一特定文件创建档案（备份文件），也可以在档案中改变文件，或者向档案中加入新的文件。tar最初被用来在磁带上创建档案，现在，用户可以在任何设备上创建档案。利用tar命令，可以把一大堆的文件和目录全部打包成一个文件，这对于备份文件或将几个文件组合成为一个文件以便于网络传输是非常有用的。

#### 参数
`-c`: 建立压缩档案<br>
`-x`：解压<br>
`-t`：查看内容<br>
`-r`：向压缩归档文件末尾追加文件<br>
`-u`：更新原压缩包中的文件<br>

以上五个是独立的命令参数，压缩解压都要用到其中一个，可以和别的命令连用但**只能用其中一个**。下面的参数是根据需要在压缩或解压档案时可选的。

`-z`：有gzip属性的  gz<br>
`-j`：有bz2属性的   bz2<br>

`-J` ：有xz属性的   xz<br>
`-Z`：有compress属性的<br>
`-v`：显示所有过程<br>
`-O`：将文件解开到标准输出<br>

下面的参数-f是必须的

-f: 使用档案名字，切记，这个参数是最后一个参数，后面只能接档案名。

##### eg:<br>
```
tar -xzf redis-4.0.8.tar.gz
```

### [ln](http://man.linuxde.net/ln)
是为某一个文件在另外一个位置建立一个同步的链接。
当我们需要在不同的目录，用到相同的文件时，我们不需要在每一个需要的目录下都放一个必须相同的文件，我们只要在某个固定的目录，放上该文件，然后在其它的目录下用ln命令链接（link）它就可以，不必重复的占用磁盘空间。
连接类型分为硬连接和符号连接两种，默认的连接类型是硬连接。如果要创建符号连接必须使用"-s"选项。
```
ln [-option] Src Dest
```

##### eg:<br>
```
ln -f /usr/local/Cellar/redis/3.0.4/homebrew.mxcl.redis.plist ~/Library/LaunchAgents/  
```

### [launchctl](https://www.jianshu.com/p/4addd9b455f2)(mac command)
是一个统一的服务管理框架，可以启动、停止和管理守护进程、应用程序、进程和脚本等。
launchctl是通过配置文件来指定执行周期和任务的


### [source](http://blog.csdn.net/violet_echo_0908/article/details/52056071)
```
source FileName
```

在当前bash环境下读取并执行FileName中的命令
source命令的作用就是用来执行一个脚本，那么：
source a.sh 同直接执行 ./a.sh 有什么不同呢，比如你在一个脚本里export $KKK=111 ,如果你用./a.sh执行该脚本，执行完毕后，你运行 echo $KKK ,发现没有值，如果你用source来执行 ，然后再echo ,就会发现KKK=111。因为调用./a.sh来执行shell是在一个子shell里运行的，所以执行后，结构并没有反应到父shell里，但是source不同它就是在本shell中执行的，所以可以看到结果

### [export](http://blog.csdn.net/damontive/article/details/50352722)
设置或显示环境变量。（比如我们要用一个命令，但这个命令的执行文件不在当前目录，这样我们每次用的时候必须指定执行文件的目录，麻烦，在代码中先执行export，这个相当于告诉程序，执行某某东西时，需要的文件或什么东东在这些目录里）

```
export [-fnp][变量名称]=[变量设置值]
```

#### 参数
-f 　代表[变量名称]中为函数名称。 
-n 　删除指定的变量。变量实际上并未删除，只是不会输出到后续指令的执行环境中。 
-p 　列出所有的shell赋予程序的环境变量。

export $PATH="$PATH:路径1:路径2:...:路径n” （或“PATH=$PATH:路径1:路径2:...:路径n"　），意思是可执行文件的路径包括原先设定的路径，也包括从“路径1”到“路径n”的所有路径。当用户输入一个一串字符并按回车后，shell会依次在这些路径里找对应的可执行文件并交给系统核心执行。那个“$PATH”表示原先设定的路径仍然有效，注意不要漏掉。某些软件可能还有“PATH”以外类型的环境变量需要添加，但方法与此相同，并且也需要注意“$”

### [mkdir -p](http://blog.csdn.net/caoeryingzi/article/details/7979329)

-p, --parents     需要时创建上层目录，如目录早已存在则不当作错误

##### eg:<br>
```
mkdir -p ~/catkin_ws/src
```

### [ls -lh]
-h 便于human看





