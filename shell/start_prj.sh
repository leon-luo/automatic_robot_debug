#! /bin/bash
#
#################################################################
# describe :该文件用与启动扫地机器人应用程序
# author   : Leon
# date     : 2018/01/23
#################################################################
PATH="/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:/opt/ros/indigo/bin"

echo "==========================[Begin $0]=========================="

source /home/firefly/.bashrc

# initialized variables
#如果要开机自动执行清扫程序测设置为1。
SELF_STARTING=0

#调试用的可执行文件
PRJ_NAME=cleaner_robot
#PRJ_NAME=main.exe

PRJ_DIRECTORY=${HOME}/catkin_ws/devel/lib/cleaner_robot
PRJ_PATH=$PRJ_DIRECTORY"/"$PRJ_NAME

if [ ! -f $PRJ_PATH ];then
	#局部清扫测试OK的可执行文件
	BACKUP_EXE_NAME=PRJ_NAME=cleaner_robot.partial-clean-ok-20180124
	PRJ_PATH=$PRJ_DIRECTORY"/"$BACKUP_EXE_NAME
fi

echo "-----------------------------------------------------------------------"
echo "| Project name is \"$PRJ_NAME\""
echo "| Project directory is \"$PRJ_DIRECTORY\""
echo "| Project path is \"$PRJ_PATH\""
echo "| The program $PRJ_NAME will be running!"
echo "| Current directory is:`pwd`"
echo "| Current user is :`whoami`"
echo "-----------------------------------------------------------------------"

run_project()
{
	if [ -d $PRJ_DIRECTORY ];then
		if [ -f $PRJ_PATH ];then
			if [ -x $PRJ_PATH ];then
				ls -lh $PRJ_PATH
			else
				#ls -lh $PRJ_PATH
				echo "[info] Non-executable permissions! \"${PRJ_NAME}\""
				chmod a+x $PRJ_PATH
				ls -lh $PRJ_PATH
			fi
			
			if [ $SELF_STARTING == 1  ];then
				echo "[info] Run \"${PRJ_PATH}\""
				$PRJ_PATH &
			fi
		else
			echo "[info] Can't find the project application program! \"${PRJ_NAME}\""
		fi

	else
		echo "[info] Can't find the project directory! \"${PRJ_DIRECTORY}\""
	fi
}

run_project

echo "==========================[End   $0]=========================="


exit 0


