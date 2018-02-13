[TOC]

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
















