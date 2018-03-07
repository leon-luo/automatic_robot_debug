#/******************************************************************************
# 文件名称: system-platform.cmake
# 版本编号: 初稿
# 作     者: Leon
# 生成日期: 2018年01月30日
# 最近修改:
# 功能描述: 定义系统与开发编译平台相关功能宏与函数
# 函数列表:
# 
# 修改历史:
# 1.日     期: 208年01月30日
#   作     者: Leon
#   修改内容: 创建文件
#*****************************************************************************/

#包含文件
include(system-platform.cmake)

##CMAKE_MINIMUM_REQUIRED(VERSION 2.5 FATAL_ERROR) 
##如果 cmake 版本小与 2.5，则出现严重错误，整个过程中止。
cmake_minimum_required(VERSION 2.8.3)

#项目程序名称
project(cleaner_robot)

#===================================================================
parallel_lines()
message(STATUS "| CMAKE VERSION is \"cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}\"")
print_variate(PROJECT_NAME)
parallel_lines()

blank_line()
print_operation_system_info()
blank_line()
config_compilers()
blank_line()
config_link_lib_and_include_directories()
blank_line()
config_compile_options()
blank_line()
print_cmake_info()
blank_line()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
##find_package(catkin REQUIRED COMPONENTS
##  roscpp
##  std_msgs
##)

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
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cleaner_robot
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
#)

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

#===================================================================
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
	include/${PROJECT_NAME}/lib/common/key
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
	src/common/bit
	src/common/key/key_data
	src/common/key/key_unit
	src/common/line
	src/common/time
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

set(${PROJECT_NAME}_SRC_LIST
	src/main/mobile_robot_main.cpp

	src/cfg_layer/mobile_robot/cfg_mobile_robot.cpp
	src/cfg_layer/data/cfg_robot_data.cpp
	src/cfg_layer/walk_plan/cfg_walk_plan.cpp
	src/cfg_layer/modulate/cfg_modulate.cpp

	src/common/algorithm/pid/pid.cpp
	src/common/angle/angle_base.cpp
	src/common/key/key_data/key_data.cpp
	src/common/key/key_unit/key_unit.cpp
	src/common/line/line_base.cpp
	src/common/time/time_base.c
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

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRC_LIST} )

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES} ${OTHER_LINK_LIST}
)

#===================================================================
set(MAIN_TEST_EXE_NAME main.app)

set(MAIN_TEST_SRC_LIST src/main/main.cpp)

add_executable(${MAIN_TEST_EXE_NAME} ${MAIN_TEST_SRC_LIST})

target_link_libraries(${MAIN_TEST_EXE_NAME}
	${catkin_LIBRARIES} ${OTHER_LINK_LIST}
)

#===================================================================
debug_info()

