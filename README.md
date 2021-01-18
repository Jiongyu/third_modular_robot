## BIRL 爬壁机器人&双手爪攀爬机器人控制
>**功能简介：**  
>**1. 关节空间位置控制**  
>**2. 关节空间速度控制**  
>**3. 笛卡尔空间位置控制**  
>**4. 离线轨迹控制**  
>**5. 示教记录**  
>**6. 零点设置**  
>**7. 夹持器控制**  
>**8. ROS控制**  
>**9. 自主抓夹**

##### 1. 安装通信驱动及相关依

###### 1.1 软件下载与环境配置
``` bash
# ~/ros/BirlModuleRobot/ 为用户自定义文件夹
mkdir -p ~/ros/BirlModuleRobot/src/ && cd ~/ros/BirlModuleRobot/src/  
git clone https://github.com/Jiongyu/third_modular_robot.git   
```

自动安装脚本：[./install.sh](./install.sh)
``` bash
cd  ~/ros/BirlModuleRobot/src/third_modular_robot  
chmod 777 ./install.sh  
sudo ./install.sh  
```

###### 1.2 编译
``` bash
cd  ~/ros/BirlModuleRobot/  
catkin_make  
sudo echo "source ~/ros/BirlModuleRobot/devel/setup.bash">> ~/.bashrc  
source ~/.bashrc 
```



##### 2. ROS安装
>参考网址：
> <http://wiki.ros.org/kinetic/Installation/Ubuntu>

#### 3. 软件使用
##### 3.1 使用手册
[使用手册](./manual/manual.pdf)

##### 3.2 文件内容:
|文件夹名|解释|
|----|-----|
|**birl_modular_robot**| **正逆运动学ros服务**|
|**canopen_communication**|**canopen底层通信**|
|**kinematics**| **正逆运动学源码**|
|**ui**| **界面逻辑文件**|

##### 3.3 连接通信启动软件
```
rosrun canopen_communication can_prepare.sh  
roslaunch ui ui_start.launch  
```
