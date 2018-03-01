#! /bin/sh
###
### 通过生成Makefile 
### 作者：leon
###

cd ~/Desktop/cross_compile_prj
CURR_PATH=`pwd`

cmake_file_path="/home/leon/catkin_ws/src/cleaner_robot"

echo "Current path: $CURR_PATH"
echo "$0()++++++++++++++++++++++++++"
sudo cmake $cmake_file_path
echo "$0()--------------------------"
