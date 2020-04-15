## BirlModuleRobot

**BIRL 爬壁机器人&双手爪攀爬机器人控制**

### 1. 安装通信驱动及相关依赖
```
cd
mkdir -r ros/biped5d_robot/src/ && cd ~/ros/biped5d_robot/src/
wget http://www.ixxat.com/docs/librariesprovider8/default-document-library/downloads/other-drivers/socketcan_1-1-92-0_20150508.zip?sfvrsn=10 
unzip socketcan_1-1-92-0_20150508.zip?sfvrsn=10
gedit README &
cd usb-to-can_v2_socketcan
sudo modprobe can-dev
sudo apt-get install module-assistant
sudo module-assistant prepare
make
sudo make install
```

```
sudo touch /etc/can.conf
sudo chmod a+w /etc/can.conf

echo "[default]  
interface = socketcan  
channel = can0" >> /etc/can.conf
```

```
sudo apt-get install can-utils
sudo pip install canopen
```

### 2. ROS安装

  参考网址：


   <http://wiki.ros.org/kinetic/Installation/Ubuntu>

### 3. 软件使用

#### 3.1 软件下载与环境配置
```
cd ~/ros/biped5d_robot/src/ 
git clone https://github.com/Jiongyu/biped5d.git
sudo echo "source ~/ros/biped5d_robot/devel/setup.bash">> ~/.bashrc
source ~/.bashrc

```

#### 3.2 连接通信启动软件
##### 3.2.1. ui control
```
rosrun canopen_communication can_prepare.sh

roslaunch ui ui_start.launch
```