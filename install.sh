#! /bin/bash

# 安装通信驱动及相关依赖
sudo modprobe can-dev
sudo apt-get install module-assistant
sudo module-assistant prepare
# 内核版本过高可能编译报错(4.15.0-76-generic 测试通过)
cd ./usb-to-can_socketcan
make
sudo make install

canConfFile='/etc/can.conf'

if [ -f ${canConfFile} ];then
  echo ${canConfFile} "文件存在"
  echo "删除文件" ${canConfFile}
  sudo rm -rf ${canConfFile}
else
  echo ${canConfFile} "文件不存在"
fi

echo "新建文件" ${canConfFile}
sudo touch /etc/can.conf
sudo chmod a+w /etc/can.conf

echo "[default]  
interface = socketcan  
channel = can0" >> /etc/can.conf

sudo apt-get install can-utils

sudo apt install python-pip
pip --version

sudo pip install python-can==3.1.1
sudo pip install canopen==1.0.0
