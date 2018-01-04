##CMAKE_MINIMUM_REQUIRED(VERSION 2.5 FATAL_ERROR) 
##如果 cmake 版本小与 2.5，则出现严重错误，整个过程中止。
cmake_minimum_required(VERSION 2.8.3)

#项目程序名称
project(cleaner_robot)


##判断当前操作系统平台
MESSAGE(STATUS "|#############################################################")

MESSAGE(STATUS "operation system is ${CMAKE_SYSTEM}")

IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
	MESSAGE(STATUS "current platform: Linux ")
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Windows")
	MESSAGE(STATUS "current platform: Windows")
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
	MESSAGE(STATUS "current platform: FreeBSD")
ELSE ()
	MESSAGE(STATUS "other platform: ${CMAKE_SYSTEM_NAME}")
ENDIF (CMAKE_SYSTEM_NAME MATCHES "Linux")

MESSAGE(STATUS "-------------------------------------------------------------")

IF (WIN32)
	MESSAGE(STATUS "Now is windows")
ELSEIF (APPLE)
	MESSAGE(STATUS "Now is Apple systens.")
ELSEIF (UNIX)
	MESSAGE(STATUS "Now is UNIX-like OS's. Including aPPLE os x and CygWin")
ENDIF ()
MESSAGE(STATUS "#############################################################|")

##BUILD_SHARED_LIBS这个开关用来控制默认的库编译方式，如果不进行设置，使用 ADD_LIBRARY 并没有指定库
##类型的情况下，默认编译生成的库都是静态库。
##如果 SET(BUILD_SHARED_LIBS ON)后，默认生成的为动态库。

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
message(STATUS "|------------------------------------------------------------")
message(STATUS " CMAKE_SYSTEM_PROCESSOR is \"${CMAKE_SYSTEM_PROCESSOR}\"")

#告知当前使用的是交叉编译方式，必须配置
set(CMAKE_SYSTEM_NAME Linux)
message(STATUS "CMAKE_SYSTEM_NAME is \"${CMAKE_SYSTEM_NAME}\"")

#指定C交叉编译器,必须配置
#或交叉编译器使用绝对地址
#SET(CMAKE_C_COMPILER "arm-linux-gcc")
#SET(CMAKE_C_COMPILER E:\my-studio-tool\Linaro\gcc-linaro-arm-linux-gnueabihf-4.8-2014.04\bin\arm-linux-gnueabihf-gcc.exe)
set(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
#SET(CMAKE_C_COMPILER /home/compile/rk_repo/px3-se/buildroot/output/host/usr/bin/arm-rockchip-linux-gnueabihf-gcc)

#指定C++交叉编译器
#SET(CMAKE_CXX_COMPILER "arm-linux-g++")
#SET(CMAKE_CXX_COMPILER E:\my-studio-tool\Linaro\gcc-linaro-arm-linux-gnueabihf-4.8-2014.04\bin\arm-linux-gnueabihf-g++.exe)
set(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)
#SET(CMAKE_CXX_COMPILER /home/compile/rk_repo/px3-se/buildroot/output/host/usr/bin/arm-rockchip-linux-gnueabihf-g++)
message(STATUS "CMAKE_C_COMPILER is \"${CMAKE_C_COMPILER}\"")
message(STATUS "CMAKE_CXX_COMPILER is \"${CMAKE_CXX_COMPILER}\"")

#不一定需要设置
#指定交叉编译环境安装目录
#set(CMAKE_FIND_ROOT_PATH /path/to/cross/compile/build/environment)

# Have to set this one to BOTH, to allow CMake to find rospack
#从来不在指定目录下查找工具程序
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
message(STATUS "CMAKE_FIND_ROOT_PATH_MODE_PROGRAM is \"${CMAKE_FIND_ROOT_PATH_MODE_PROGRAM}\"")

#只在指定目录下查找库文件
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
message(STATUS "CMAKE_FIND_ROOT_PATH_MODE_LIBRARY is \"${CMAKE_FIND_ROOT_PATH_MODE_LIBRARY}\"")

#只在指定目录下查找头文件
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
message(STATUS "CMAKE_FIND_ROOT_PATH_MODE_INCLUDE is \"${CMAKE_FIND_ROOT_PATH_MODE_INCLUDE}\"")

set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
message(STATUS "CMAKE_FIND_ROOT_PATH_MODE_PACKAGE is \"${CMAKE_FIND_ROOT_PATH_MODE_INCLUDE}\"")
message(STATUS "------------------------------------------------------------|")

endif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")

##CMAKE_BUILD_TYPE:build类型(Debug,Release,…),CMAKE_BUILD_TYPE=Debug;
#set(CMAKE_BUILD_TYPE Debug)
#但是，这样的设置却是没有效果的。必须改成如下的才行：
set(CMAKE_BUILD_TYPE Debug CACHE STRING "set build type to debug")  
#还可以在命令行设置：
#cmake  -DCMAKE_BUILD_TYPE=Debug .. 

## CMAKE_MAJOR_VERSION，CMAKE 主版本号，比如 2.4.6 中的 2 
## CMAKE_MINOR_VERSION，CMAKE 次版本号，比如 2.4.6 中的 4 
## CMAKE_PATCH_VERSION，CMAKE 补丁等级，比如 2.4.6 中的 6 
## CMAKE_SYSTEM，系统名称，比如 Linux-2.6.22 
## CMAKE_SYSTEM_NAME，不包含版本的系统名，比如 Linux 
## CMAKE_SYSTEM_VERSION，系统版本，比如 2.6.22 
## CMAKE_SYSTEM_PROCESSOR，处理器名称，比如 i686. armv7l
## UNIX，在所有的类 UNIX 平台为 TRUE，包括 OS X 和 cygwin 
## WIN32，在所有的 win32 平台为 TRUE，包括 cygwin
## CMAKE_BINARY_DIR 工程顶层目录
## CMAKE_SOURCE_DIR 工程顶层目录
## PROJECT_BINARY_DIR 应用程序存放的位置
## PROJECT_NAME 返回通过 PROJECT 指令定义的项目名称。
## 使用$ENV{NAME}指令就可以调用系统的环境变量了。比如MESSAGE(STATUS “HOME dir: $ENV{HOME}”) 
## 设置环境变量的方式是：SET(ENV{变量名} 值) 
## EXECUTABLE_OUTPUT_PATH：可执行文件的存放路径
## LIBRARY_OUTPUT_PATH：库文件路径
## CMAKE_BUILD_TYPE:：build 类型(Debug, Release, ...)，CMAKE_BUILD_TYPE=Debug
## BUILD_SHARED_LIBS：Switch between shared and static libraries

## message([STATUS | SEND_ERROR |  FATAL_ERROR] "message to display" ...)
message(STATUS "|==============================================================")
message(STATUS "UNIX is \"${UNIX}\"")
message(STATUS "WIN32 is \"${WIN32}\"")
message(STATUS "Application name is \"${PROJECT_NAME}\"")
message(STATUS "PROJECT_SOURCE_DIR is \"${PROJECT_SOURCE_DIR}\"")
message(STATUS "PROJECT_BINARY_DIR is \"${PROJECT_BINARY_DIR}\"")
message(STATUS "<projectname>_BINARY_DIR is \"${${PROJECT_NAME}_SOURCE_DIR}\"")
message(STATUS "<projectname>_BINARY_DIR is \"${${PROJECT_NAME}_BINARY_DIR}\"")
message(STATUS "CMAKE VERSION is \"cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}\"")
message(STATUS "CMAKE_SYSTEM is \"${CMAKE_SYSTEM}\"")
message(STATUS "CMAKE_SYSTEM_NAME is \"${CMAKE_SYSTEM_NAME}\"")
message(STATUS "CMAKE_SYSTEM_VERSION is \"${CMAKE_SYSTEM_VERSION}\"")
message(STATUS "CMAKE_SYSTEM_PROCESSOR is \"${CMAKE_SYSTEM_PROCESSOR}\"")
message(STATUS "CMAKE_C_FLAGS is \"${CMAKE_C_FLAGS}\"")
message(STATUS "CMAKE_CXX_FLAGS is \"${CMAKE_CXX_FLAGS}\"")
message(STATUS "Build Type CMAKE_BUILD_TYPE is \"${CMAKE_BUILD_TYPE}\"")
message(STATUS "CMAKE_FIND_ROOT_PATH is \"${CMAKE_FIND_ROOT_PATH}\"")
message(STATUS "==============================================================|")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
##find_package(catkin REQUIRED COMPONENTS
##  roscpp
##  std_msgs
##)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs

  dynamixel_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
#  joy
#  pcl_conversions
#  pcl_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cleaner_robot
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
#	include
#	${catkin_INCLUDE_DIRS}
#	#/opt/ros/indigo/include/ros/
#)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/cleaner_robot.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/cleaner_robot_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cleaner_robot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

include_directories(
	include
	include/${PROJECT_NAME}
	include/${PROJECT_NAME}/debug
	include/${PROJECT_NAME}/type
	include/${PROJECT_NAME}/lib
)

include_directories(
	include/${PROJECT_NAME}/lib/common
	include/${PROJECT_NAME}/lib/common/angle
	include/${PROJECT_NAME}/lib/common/algorithm
	include/${PROJECT_NAME}/lib/common/algorithm/pid
	include/${PROJECT_NAME}/lib/common/version
)

include_directories(
	include/${PROJECT_NAME}/lib/cfg_layer
	include/${PROJECT_NAME}/lib/cfg_layer/data
	include/${PROJECT_NAME}/lib/cfg_layer/mobile_robot
	include/${PROJECT_NAME}/lib/cfg_layer/walk_plan
	include/${PROJECT_NAME}/lib/cfg_layer/modulate
)

include_directories(
	include/${PROJECT_NAME}/lib/bll_layer
)

include_directories(
	include/${PROJECT_NAME}/lib/drv_layer
)

include_directories(
	include/${PROJECT_NAME}/lib/ui_layer
)

include_directories(
	src/common/angle
	src/common/algorithm/pid
	src/common/line
	src/common/version
)

include_directories(
	src/business_logic_layer/timer
	src/business_logic_layer/rotate
	src/business_logic_layer/trajectory
	src/business_logic_layer/motion_control
	src/business_logic_layer/partial_cleaning
	src/business_logic_layer/processing_function
	src/business_logic_layer/straight_line_moving
	src/business_logic_layer/sensor/cliff_sensor
	src/business_logic_layer/sensor/bumper_sensor
	src/business_logic_layer/sensor/ultrasonic_sensor
	src/business_logic_layer/sensor/wheel_drop_sensor
	src/business_logic_layer/sensor/wall_following_sensor
	
)

include_directories(
	src/cfg_layer/data
	src/cfg_layer/mobile_robot
	src/cfg_layer/modulate
	src/cfg_layer/walk_plan
)

include_directories(
	src/drv_layer/sensor
)

add_executable(${PROJECT_NAME}  src/main/mobile_robot_main.cpp

								src/cfg_layer/mobile_robot/cfg_mobile_robot.cpp
								src/cfg_layer/data/cfg_robot_data.cpp
								src/cfg_layer/walk_plan/cfg_walk_plan.cpp
								src/cfg_layer/modulate/cfg_modulate.cpp
								
								src/common/algorithm/pid/pid.cpp
								src/common/angle/angle_base.cpp
								src/common/line/line_base.cpp
								src/common/version/version.cpp
								
								src/business_logic_layer/rotate/bll_rotate.cpp
								src/business_logic_layer/motion_control/bll_motion_control.cpp
								src/business_logic_layer/straight_line_moving/bll_traight_line_moving.cpp
								src/business_logic_layer/partial_cleaning/bll_partial_cleaning.cpp
								src/business_logic_layer/processing_function/bll_processing_function.cpp
								
								src/business_logic_layer/sensor/bumper_sensor/bll_bumper.cpp
								src/business_logic_layer/sensor/cliff_sensor/bll_cliff.cpp
								src/business_logic_layer/sensor/ultrasonic_sensor/bll_ultrasonic.cpp
								src/business_logic_layer/sensor/wall_following_sensor/bll_wall_following.cpp
								src/business_logic_layer/sensor/wheel_drop_sensor/bll_wheel_drop.cpp
								
								src/business_logic_layer/timer/bll_timer.cpp
								src/business_logic_layer/trajectory/bll_trajectory.cpp
								src/business_logic_layer/processing_function/bll_processing_function.cpp
								
								src/drv_layer/sensor/drv_sensor.cpp

								include/${PROJECT_NAME}/lib/cfg_layer/mobile_robot/cfg_if_mobile_robot.cpp
								include/${PROJECT_NAME}/lib/cfg_layer/modulate/cfg_if_modulate.cpp
								include/${PROJECT_NAME}/lib/common/version/cfg_if_version.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

