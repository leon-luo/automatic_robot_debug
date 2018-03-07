#! /bin/sh
###
### 挂载WIN10的目录（cleaner_robot_debug）到本地指定目录（~/catkin_ws/src/cleaner_robot）上
### 作者：leon
###


DEFAULT_IP="227"
HOST_IP="192.168.31."

if [ $# -eq 1 ]; then
	if [ $1 -gt 1 ] && [ $1 -lt 255 ]; then
		USER_IP=$1
		WIN_IP=$HOST_IP$USER_IP
	else
		echo "s1="$1
	fi
else
	WIN_IP=$HOST_IP$DEFAULT_IP
fi

SRC_DIR="cleaner_robot_debug"
DES_DIR="/home/leon/catkin_ws/src/cleaner_robot"

echo "para sum="$#
echo "para0="$0
echo "para1="$1
echo "win10ip="$WIN_IP

sudo mount //$WIN_IP/$SRC_DIR $DES_DIR -o user=leon,pass=leon
