#! /bin/bash

# 安装通信驱动及相关依赖
sudo modprobe can-dev
apt list module-assistant | grep installed || sudo apt install module-assistant
sudo module-assistant prepare

echo -e "\e[1;32m \n编译usbtocan驱动:\n \e[0m"
# 内核版本过高可能编译报错(4.15.0-76-generic 测试通过)
cd ./usb-to-can_socketcan
make clean
make
sudo make install

# 创建/etc/can.conf文件
canConfFile='/etc/can.conf'
if [ -f ${canConfFile} ];then
  # can.conf 若存在，删除
  echo -e "\e[1;32m ${canConfFile} 文件存在 \e[0m"
  echo -e "\e[1;32m 删除文件 ${canConfFile} \e[0m"
  sudo rm -rf ${canConfFile}
else
  echo -e "\e[1;32m ${canConfFile} 文件不存在 \e[0m"
fi
echo -e "\e[1;32m 新建文件 ${canConfFile} \e[0m"
sudo touch /etc/can.conf
sudo chmod a+w /etc/can.conf

echo "[default]  
interface = socketcan  
channel = can0" >> /etc/can.conf

echo -e "\e[1;32m \n判断 can-utils 是否安装:\n \e[0m"
apt list can-utils | grep installed || sudo apt install can-utils

echo -e "\e[1;32m \n判断 python-pip 是否安装:\n \e[0m"
apt list python-pip | grep installed || sudo apt install python-pip
pip --version

echo -e "\e[1;32m \n判断pip中是否存在 3.1.1 python-can:\n \e[0m"
pip show python-can | grep 3.1.1 || sudo pip install python-can==3.1.1

echo -e "\e[1;32m \n判断pip中是否存在 1.0.0 canopen:\n \e[0m"
pip show canopen | grep 1.0.0 || sudo pip install canopen==1.0.0
