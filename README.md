## BirlModuleRobot

**BIRL 爬壁机器人&双手爪攀爬机器人控制**

##### 1. 安装通信驱动及相关依赖
自动安装脚本：[./install.sh](/src/BirlModuleRobot/install.sh)
```
sudo ./install.sh
```
##### 2. ROS安装
>参考网址：
> <http://wiki.ros.org/kinetic/Installation/Ubuntu>

#### 3. 软件使用

##### 3.1 文件内容:
|文件夹名|解释|
|----|-----|
|*birl_modular_robot*| *正逆运动学ros服务*|
|*canopen_communication*|*canopen底层通信*|
|*kinematics*| *正逆运动学源码*|
|*ui*| *界面逻辑文件*|

##### 3.1 连接通信启动软件
```
rosrun canopen_communication can_prepare.sh
roslaunch ui ui_start.launch
```