#! /bin/sh
###
### 挂载WIN10的目录（cleaner_robot_debug）到本地指定目录（~/catkin_ws/src/cleaner_robot）上
### 作者：leon
###


DEFAULT_IP="227"
HOST_IP="192.168.31."


if [ $# -eq 1 ]; then

	IP_ADDR=$1
	regex="\b(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9][0-9]|[1-9])\.(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9][0-9]|[0-9])\.(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9][0-9]|[0-9])\.(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9][0-9]|[1-9])\b"
	ret=`echo $1 | egrep $regex | wc -l`
	#echo "ret=\"$ret\""
	if [ $ret -eq 0 ]
	then
		if [ $1 -gt 1 ] && [ $1 -lt 255 ]; then
			USER_IP=$1
			WIN_IP=$HOST_IP$USER_IP
		else
			echo "The string $IPADDR is not a correct IP address ! s1=\"$1\""
			echo "please input the full IP address or [0 ~ 255] ."
			exit 0
		fi
	       
	else
		WIN_IP=$IP_ADDR
	fi
else
	WIN_IP=$HOST_IP$DEFAULT_IP
fi

SRC_DIR="cleaner_robot_debug"
DES_DIR="$HOME/catkin_ws/src/cleaner_robot"

#echo "para sum="$#
#echo "para0="$0
#echo "para1="$1

LOCAL_NAME=`hostname`
#echo "LOCAL_NAME=\"$LOCAL_NAME\""


#CURR_IP=`/sbin/ifconfig -a|grep inet|grep -v 127.0.0.1|grep -v inet6|awk '{print $2}'|tr -d "addr:"`
CURR_IP=`hostname -i|awk '{print $2}'`

echo "$LOCAL_NAME IP=\"$CURR_IP\""
echo "Window IP=\"$WIN_IP\""

sudo mount //$WIN_IP/$SRC_DIR $DES_DIR -o user=leon,pass=leon

df -h | grep $DES_DIR

ls -lh --time-style=long-iso $DES_DIR

