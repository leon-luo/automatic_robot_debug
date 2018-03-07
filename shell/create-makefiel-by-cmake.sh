#!/bin/bash
##! /bin/sh
###

#******************************************************************************
# File name   : create-makefiel-by-cmake.sh
# Description : 创建生成Makefile
#	1.先使用～/mount-cleaner-robot.sh脚本挂载win10 PC 上的代码到目录指定目录
#	（~/catkin_ws/src/cleaner_robot）上
#	2.进入目录 ~/Desktop/cross_compile_prj
#	3.删除原先生成的旧Makefile文件、目标文件、应用程序等。
#	4.在当前目录下使用cmake生成Makefile文件
#	5.使用make命令编译生成对应的应用程序（cleaner-robot或main.app）
#****************************************************************************** 
# History        :
#  1.Data        :2018/3/6
#    Author      : Leon
#    Modification: Created file.
#******************************************************************************

SOURCE_DIR="$HOME/catkin_ws/src/cleaner_robot"
BUILD_DIR="$HOME/Desktop/cross_compile_prj"

SAVE_APP_DIR=${SOURCE_DIR}"/app"

CLEANER_ROBOT_APP_NAME="cleaner_robot"
TEST_APP_NAME="main.app"
DEBUG_APP_NAME="${CLEANER_ROBOT_APP_NAME}-new"

SH_FILE_INFO="run \"$0 \""
BEGINE_INFO="Begin ${SH_FILE_INFO}"
STOP_INFO="STOP ${SH_FILE_INFO}"



function print_info()
{
	echo "*************************[$1]*************************"
}

#拷贝新生成的应用程序出来
function copy_application_program()
{
	SRC_FILE=${BUILD_DIR}/${CLEANER_ROBOT_APP_NAME}
	DES_FILE=${SAVE_APP_DIR}/${DEBUG_APP_NAME}

	if [ -f "$SRC_FILE" ]; then
		echo
		cp ${SRC_FILE} ${DES_FILE}
		echo "[INFO] The new app is:"
		ls -lh --full-time ${SRC_FILE}
		echo "[INFO] The debug app is:"
		ls -lh --full-time ${DES_FILE}
		echo
	fi
}

#生成Makefile
function creat_makefile()
{
	if [ -d ${SOURCE_DIR} ]; then
		ls -lh ${SOURCE_DIR}
	fi

	if [ ! -f ${SOURCE_DIR}/win10-direction ]; then
		echo "[WARNING] Please mount the win10 source direction to ubuntu \"${SOURCE_DIR}\" by running the script \"～/mount-cleaner-robot.sh\"."
		exit 0
	fi

	if [ ! -d ${BUILD_DIR} ]; then
		mkdir ${BUILD_DIR}
	fi

	cd ${BUILD_DIR}
	CURR_DIR=`pwd`
	echo "[INFO] Current dir is \"${CURR_DIR}\"."


	cmake ${SOURCE_DIR}
	echo
	echo "[INFO] The new Makefile is:"
	ls -lh --time-style=locale ${BUILD_DIR}/Makefile
	echo

  	if [ "make" == "$1" ]; then
		make
	elif [ "cp" == "$1" ]; then
		copy_application_program $1
	else
		make $1
	fi

	echo
	ls -l --full-time ${BUILD_DIR}
	echo

	return 1
}


print_info ${BEGINE_INFO}

echo "para sum = "$#
echo "para0 = "$0
echo "para1 = "$1
echo "SOURCE_DIR = "${SOURCE_DIR}
echo "BUILD_DIR = \"${BUILD_DIR}\""
echo "SAVE_APP_DIR = \"${SAVE_APP_DIR}\""

creat_makefile $1

print_info ${STOP_INFO}

