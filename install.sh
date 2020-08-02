#! /bin/bash
# 安装通信驱动及相关依赖
cd
mkdir -r ros/BirlModuleRobot/src/ && cd ~/ros/BirlModuleRobot/src/
wget http://www.ixxat.com/docs/librariesprovider8/default-document-library/downloads/other-drivers/socketcan_1-1-92-0_20150508.zip?sfvrsn=10 
unzip socketcan_1-1-92-0_20150508.zip?sfvrsn=10
gedit README &
cd usb-to-can_v2_socketcan
sudo modprobe can-dev
sudo apt-get install module-assistant
sudo module-assistant prepare
# 内核版本过高可能编译报错
make
sudo make install

sudo touch /etc/can.conf
sudo chmod a+w /etc/can.conf

echo "[default]  
interface = socketcan  
channel = can0" >> /etc/can.conf

sudo apt-get install can-utils
sudo pip install python-can==3.1.1
sudo pip install canopen==1.0.0

# 软件下载与环境配置
cd ~/ros/BirlModuleRobot/src/ 
git clone https://github.com/Jiongyu/third_modular_robot.git
sudo echo "source ~/ros/BirlModuleRobot/devel/setup.bash">> ~/.bashrc
source ~/.bashrc