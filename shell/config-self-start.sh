#! /bin/bash
#
#################################################################
# describe :该文件用与启动扫地机器人应用程序
# author   : Leon
# date     : 2018/01/25
#################################################################
PATH="/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin"

echo "==========================[Begin $0]=========================="

INIT_DIR=/etc/init.d
FILE_NAME=ros-daemon.bash
LINK_NAME=ros-daemon
CURR_DIR=`pwd`
echo "Current direction:\"$CURR_DIR\""
#删除自启动
sudo update-rc.d -f $LINK_NAME remove

if [ -f $CURR_DIR/$FILE_NAME ];then
	sudo cp $CURR_DIR/$FILE_NAME $INIT_DIR
	if [ -f $INIT_DIR/$LINK_NAME ];then
		sudo rm $INIT_DIR/$LINK_NAME
	fi
	sudo ln -s $INIT_DIR/$FILE_NAME $INIT_DIR/$LINK_NAME
	sudo chmod a+x $INIT_DIR/$LINK_NAME
fi

tree /etc/rc2.d/
sudo update-rc.d ros-daemon defaults 90 10
if [ $? == 0 ];then
	ls -lh --time-style=full-iso /etc/rc2.d/*$LINK_NAME
fi

SH_DIR=/home/firefly/shell
if [ ! -d $SH_DIR ];then
	sudo mkdir $SH_DIR
fi

FILE_NAME=start_prj.sh
if [ -f $PRJ_PATH ];then
	sudo cp $CURR_DIR/$FILE_NAME $SH_DIR
	ls -lh --time-style=full-iso $SH_DIR/$FILE_NAME
fi

echo "==========================[End   $0]=========================="

exit 0


