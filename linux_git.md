本地git关联到github

安装git
```
apt-get install git
```
配置用户信息
```
git config --global user.name "wangzp"
git config --global user.email "wzhpanging@163.com"
```

开启ssh服务，一般默认都会有ssh服务，可跳过

生成ssh key
```
ssh-keygen -t rsa -C"wzhpanging@163.com"
```
回车*3

复制ssh key
ssh key存在.ssh/id_rsa.pub中，复制其中的内容并在github的setting中添加


在本地建立git仓库
```
cd localdir
git init
git add .
git commit -m "first commit"
```

关联到github
```
git remote add origin git@github.com:PatricWang/everything.git
git push -u origin master
```

修改已经提交的注释
```
git commit --amend
```