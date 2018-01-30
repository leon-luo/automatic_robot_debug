##CMAKE_MINIMUM_REQUIRED(VERSION 2.5 FATAL_ERROR) 
##如果 cmake 版本小与 2.5，则出现严重错误，整个过程中止。
cmake_minimum_required(VERSION 2.8.3)

#项目程序名称
project(cleaner_robot)

macro(blank_line)
	message(STATUS "")
endmacro()


macro(cut_off_rule)
	message(STATUS "|--------------------------------------------------------------")
endmacro()


macro(parallel_lines)
	message(STATUS "|==============================================================")
endmacro()


macro(print_variate arg)
	message(STATUS "| ${arg} is \"${${arg}}\"")
endmacro()


macro(print_info info arg)
	message(STATUS "| info : \"${arg}\"")
endmacro()


function(include_sub_directories_recursively root_dir)
	parallel_lines()
    if (IS_DIRECTORY ${root_dir})               # 当前路径是一个目录吗，是的话就加入到包含目录
        message(STATUS "include dir: " ${root_dir})
        include_directories(${root_dir})
    endif()

    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*) # 获得当前目录下的所有文件，让如ALL_SUB列表中
    foreach(sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})                    
            include_sub_directories_recursively(${root_dir}/${sub}) # 对子目录递归调用，包含
        endif()
    endforeach()
    parallel_lines()
endfunction()


function(print_operation_system_info)
	parallel_lines()
	message(STATUS "print_operation_system_info()")
	message(STATUS "| operation system is ${CMAKE_SYSTEM}")
	if (CMAKE_SYSTEM_NAME MATCHES "Linux")
		message(STATUS "| Current platform: Linux ")
	elseif (CMAKE_SYSTEM_NAME MATCHES "Windows")
		message(STATUS "| Current platform: Windows")
	elseif (CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
		message(STATUS "| Current platform: FreeBSD")
	else ()
		message(STATUS "| Other platform: ${CMAKE_SYSTEM_NAME}")
	endif (CMAKE_SYSTEM_NAME MATCHES "Linux")
	cut_off_rule()
	
	if (WIN32)
		message(STATUS "| Now is windows")
	elseif (APPLE)
		message(STATUS "| Now is Apple systens.")
	elseif (UNIX)
		message(STATUS "| Now is UNIX-like OS's. Including aPPLE os x and CygWin")
	endif ()
	parallel_lines()
endfunction()

macro(config_compile_options)
	parallel_lines()
	message(STATUS "config_compile_options()")
	add_compile_options(-g -lrt -lm -lpthread)
	## Compile as C++11, supported in ROS Kinetic and newer
	#add_compile_options(-std=c++11 -g)
	if (CMAKE_COMPILER_IS_GNUCXX)
		print_variate(CMAKE_COMPILER_IS_GNUCXX)
		# c++11 required
	    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++ -std=gnu++11")
		set(CMAKE_CXX_STANDARD 11)
		set(CMAKE_CXX_STANDARD_REQUIRED ON)
		print_variate(CMAKE_CXX_FLAGS)
		print_variate(CMAKE_CXX_STANDARD)
		print_variate(CMAKE_CXX_STANDARD_REQUIRED)
	endif (CMAKE_COMPILER_IS_GNUCXX)

	##BUILD_SHARED_LIBS这个开关用来控制默认的库编译方式，如果不进行设置，使用 ADD_LIBRARY 并没有指定库
	##类型的情况下，默认编译生成的库都是静态库。
	##如果 SET(BUILD_SHARED_LIBS ON)后，默认生成的为动态库。

	##CMAKE_BUILD_TYPE:build类型(Debug,Release,…),CMAKE_BUILD_TYPE=Debug;
	##CMake 中有一个变量 CMAKE_BUILD_TYPE ,可以的取值是 Debug Release RelWithDebInfo 和 MinSizeRel。
	##当这个变量值为 Debug 的时候,CMake 会使用变量 CMAKE_CXX_FLAGS_DEBUG 和 CMAKE_C_FLAGS_DEBUG 中
	##的字符串作为编译选项生成 Makefile ,当这个变量值为 Release 的时候,工程会使用变量 
	##CMAKE_CXX_FLAGS_RELEASE 和 CMAKE_C_FLAGS_RELEASE 选项生成 Makefile。 
	#set(CMAKE_BUILD_TYPE Debug)
	#但是，这样的设置却是没有效果的。必须改成如下的才行：
	set(CMAKE_BUILD_TYPE Debug CACHE STRING "set build type to debug")  
	#还可以在命令行设置：
	#cmake  -DCMAKE_BUILD_TYPE=Debug .. 

	##要显示执行构建过程中详细的信息(比如为了得到更详细的出错信息) 
	##或者执行make时
	##$ make VERBOSE=1
	##或
	##$ export VERBOSE=1
	##$ make
	#SET(CMAKE_VERBOSE_MAKEFILE on)
	parallel_lines()
endmacro()

macro(config_cross_compiler)
	parallel_lines()
	message(STATUS "config_cross_compiler()")
	if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
		message(STATUS "|************ Current processor is \"${CMAKE_SYSTEM_PROCESSOR}\" **************|")
		#告知当前使用的是交叉编译方式，必须配置
		set(CMAKE_SYSTEM_NAME Linux)
		print_variate(CMAKE_SYSTEM_NAME)
		
		set(TOOL_COMPILING_PATH /home/compile/rk_repo/px3-se/buildroot/output/host/usr)
		set(COMPILER_DIR /home/compile/rk_repo/px3-se/buildroot/output/host/usr/bin)
		set(COMPILER_NAME_PREFIX arm-rockchip-linux-gnueabihf-)

		#set(COMPILER_DIR $ENV{HOME}/cross_compile_lib/gcc-linaro-5.5.0-2017.10-x86_64_arm-linux-gnueabihf/bin)
		#set(COMPILER_NAME_PREFIX arm-linux-gnueabihf-)
		
		#不一定需要设置
		#指定交叉编译环境安装目录
		SET(TOOLCHAIN_DIR ${COMPILER_DIR})
		SET(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_DIR})
	
		# Have to set this one to BOTH, to allow CMake to find rospack
		#从来不在指定目录下查找工具程序
		set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
		print_variate(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM)
		#只在指定目录下查找库文件
		set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
		print_variate(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY)
		#只在指定目录下查找头文件
		set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
		print_variate(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE)

		set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
		print_variate(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE)

		set(CMAKE_CROSSCOMPILING true)
		print_variate(CMAKE_CROSSCOMPILING)
	elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
		message(STATUS "|************ Current processor is \"${CMAKE_SYSTEM_PROCESSOR}\" **************|")
		set(COMPILER_DIR /usr/bin)
		set(COMPILER_NAME_PREFIX arm-linux-gnueabihf-)
	endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")

	set(C_COMPILER_NAME   ${COMPILER_NAME_PREFIX}gcc)
	set(CXX_COMPILER_NAME ${COMPILER_NAME_PREFIX}g++)

	set(C_COMPILER_PATH   ${COMPILER_DIR}/${C_COMPILER_NAME})
	set(CXX_COMPILER_PATH ${COMPILER_DIR}/${CXX_COMPILER_NAME})

	#编译器必须配置,可以使用交叉编译器绝对地址
	SET(CMAKE_C_COMPILER   ${C_COMPILER_PATH})      #指定C交叉编译器
	SET(CMAKE_CXX_COMPILER ${CXX_COMPILER_PATH})    #指定C++交叉编译器

	parallel_lines()
endmacro()

macro(config_link_lib_and_include_directories)
	parallel_lines()
	message(STATUS "config_link_lib_and_include_directories()")
	if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
		message(STATUS "|************ Current processor is \"${CMAKE_SYSTEM_PROCESSOR}\" **************|")

		SET(ROS_INSTALL_DIRECTORY $ENV{HOME}/cross_compile_lib/arm-ros/indigo) #指定ROS安装路径
		SET(ROS_LIBRARY_DIRECTORY ${ROS_INSTALL_DIRECTORY}/lib)                #指定ROS库路径
		SET(ROS_INCLUDE_DIRECTORY ${ROS_INSTALL_DIRECTORY}/include)            #指定ROS头文件路径
		include_directories(
			$ENV{HOME}/cross_compile_lib/arm-boost_1_66_0/include
		)

		link_directories(
			/usr/arm-linux-gnueabihf/lib
			$ENV{HOME}/cross_compile_lib/lib/usr/lib/arm-linux-gnueabihf
			$ENV{HOME}/cross_compile_lib/lib
			$ENV{HOME}/cross_compile_lib/arm-boost_1_66_0/lib
		)
	elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
		message(STATUS "|************ Current processor is \"${CMAKE_SYSTEM_PROCESSOR}\" **************|")

		SET(ROS_INSTALL_DIRECTORY /opt/ros/indigo)                             #指定ROS安装路径
		SET(ROS_LIBRARY_DIRECTORY ${ROS_INSTALL_DIRECTORY}/lib)                #指定ROS库路径
		SET(ROS_INCLUDE_DIRECTORY ${ROS_INSTALL_DIRECTORY}/include)            #指定ROS头文件路径
	endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")

	include_directories(
		${ROS_INCLUDE_DIRECTORY}
	)

	SET(OTHER_LINK_LIST	 
		${ROS_LIBRARY_DIRECTORY}/libroscpp.so 
		${ROS_LIBRARY_DIRECTORY}/librosconsole.so 
		${ROS_LIBRARY_DIRECTORY}/librosconsole_log4cxx.so 
		${ROS_LIBRARY_DIRECTORY}/librosconsole_backend_interface.so 
		${ROS_LIBRARY_DIRECTORY}/libroscpp_serialization.so 
		${ROS_LIBRARY_DIRECTORY}/librostime.so 
		${ROS_LIBRARY_DIRECTORY}/libxmlrpcpp.so 
		${ROS_LIBRARY_DIRECTORY}/libcpp_common.so 
		-rdynamic 
		-lpthread 
		-llog4cxx 
		-lconsole_bridge 
		-lboost_signals 
		-lboost_filesystem 
		-lboost_regex 
		-lboost_date_time 
		-lboost_system 
		-lboost_thread 
		-Wl,-rpath,${ROS_LIBRARY_DIRECTORY}
	)

	print_variate(OTHER_LINK_LIST)
	parallel_lines()
endmacro()

function(print_cmake_info)
## CMAKE_MAJOR_VERSION，CMAKE 主版本号，比如 2.4.6 中的 2 
## CMAKE_MINOR_VERSION，CMAKE 次版本号，比如 2.4.6 中的 4 
## CMAKE_PATCH_VERSION，CMAKE 补丁等级，比如 2.4.6 中的 6 
## CMAKE_SYSTEM，系统名称，比如 Linux-2.6.22 
## CMAKE_SYSTEM_NAME，不包含版本的系统名，比如 Linux 
## CMAKE_SYSTEM_VERSION，系统版本，比如 2.6.22 
## CMAKE_SYSTEM_PROCESSOR，处理器名称，比如 x86_64、i686、arm、 armv7l
## UNIX，在所有的类 UNIX 平台为 TRUE，包括 OS X 和 cygwin 
## WIN32，在所有的 win32 平台为 TRUE，包括 cygwin
## CMAKE_BINARY_DIR 工程顶层目录
## CMAKE_SOURCE_DIR 工程顶层目录
## PROJECT_BINARY_DIR 应用程序存放的位置
## PROJECT_NAME 返回通过 PROJECT 指令定义的项目名称。
## 使用$ENV{NAME}指令就可以调用系统的环境变量了。比如MESSAGE(STATUS “HOME dir: $ENV{HOME}”) 
## 设置环境变量的方式是：SET(ENV{变量名} 值) 
## EXECUTABLE_OUTPUT_PATH：可执行文件的存放路径，重新定义目标二进制可执行文件的存放位置
## LIBRARY_OUTPUT_PATH 重新定义目标链接库文件的存放位置，库文件路径
## CMAKE_BUILD_TYPE:：build 类型(Debug, Release, ...)，CMAKE_BUILD_TYPE=Debug
## BUILD_SHARED_LIBS：Switch between shared and static libraries
## CMAKE_VERBOSE_MAKEFILE：要显示执行构建过程中详细的信息(比如为了得到更详细的出错信息) 设为on
## message([STATUS | SEND_ERROR |  FATAL_ERROR] "message to display" ...)
## STATUS,输出前缀为-的信息。
## SEND_ERROR，产生错误，生成过程被跳过。
## FATAL_ERROR，立即终止所有cmake过程。
	parallel_lines()
	message(STATUS "print_cmake_info()")
	print_variate(UNIX)
	print_variate(WIN32)
	blank_line()
	print_variate(PROJECT_NAME)
	print_variate(PROJECT_SOURCE_DIR)
	print_variate(PROJECT_BINARY_DIR)
	print_variate(${PROJECT_NAME}_BINARY_DIR)
	blank_line()
	print_variate(CMAKE_SYSTEM)
	print_variate(CMAKE_SYSTEM_NAME)
	print_variate(CMAKE_SYSTEM_VERSION)
	print_variate(CMAKE_SYSTEM_PROCESSOR)
	blank_line()
	print_variate(CMAKE_C_COMPILER)
	print_variate(CMAKE_C_FLAGS)
	blank_line()
	print_variate(CMAKE_CXX_COMPILER)
	print_variate(CMAKE_CXX_FLAGS)
	print_variate(CMAKE_CXX_STANDARD)
	print_variate(CMAKE_CXX_STANDARD_REQUIRED)
	print_variate(CMAKE_COMPILER_IS_GNUCXX)
	blank_line()
	print_variate(CMAKE_BUILD_TYPE)
	print_variate(CMAKE_CROSSCOMPILING)
	print_variate(CMAKE_FIND_ROOT_PATH)
	print_variate(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM)
	print_variate(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY)
	print_variate(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE)
	print_variate(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE)
	print_variate(CMAKE_VERBOSE_MAKEFILE)
	print_variate(CMAKE_LIBRARY_PATH)
	print_variate(CMAKE_INCLUDE_PATH)
	blank_line()
	print_variate(LD_LIBRARY_PATH)
	print_variate(BUILD_SHARED_LIBS)
	print_variate(LIBRARY_OUTPUT_PATH)
	print_variate(EXECUTABLE_OUTPUT_PATH)
	print_variate(OTHER_LINK_LIST)
	parallel_lines()
endfunction()

function(debug_info)
	parallel_lines()
	print_variate(catkin_FOUND)
	print_variate(catkin_INCLUDE_DIRS)
	print_variate(catkin_LIBRARIES)
	print_variate(catkin_DEFINITIONS)
	blank_line()
	print_variate(Boost_FOUND)
	print_variate(Boost_INCLUDE_DIRS)
	print_variate(Boost_LIBRARIES)
	print_variate(Boost_DEFINITIONS)
	blank_line()
	print_variate(roscpp_FOUND)
	print_variate(roscpp_INCLUDE_DIRS)
	print_variate(roscpp_LIBRARIES)
	print_variate(roscpp_DEFINITIONS)
	parallel_lines()
endfunction()

#===================================================================
parallel_lines()
print_cmake_info()
parallel_lines()
message(STATUS "| CMAKE VERSION is \"cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}\"")
message(STATUS "|************ Current project is ${PROJECT_NAME} **************|")
print_variate(PROJECT_NAME)
blank_line()
print_operation_system_info()
blank_line()
config_cross_compiler()
blank_line()
config_link_lib_and_include_directories()
blank_line()
print_variate(CMAKE_C_COMPILER)
print_variate(CMAKE_CXX_COMPILER)
blank_line()

config_compile_options()


blank_line()
parallel_lines()
print_variate(CMAKE_C_COMPILER)
print_variate(CMAKE_CXX_COMPILER)
print_variate(CMAKE_C_FLAGS)
print_variate(CMAKE_CXX_FLAGS)
parallel_lines()

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

##find_package(catkin REQUIRED COMPONENTS
#	roscpp
#)

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

#blank_line()
#print_variate(${PROJECT_NAME}_SRC_LIST)
#blank_line()
#print_variate(PROJECT_NAME)
#blank_line()
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRC_LIST} )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${OTHER_LINK_LIST}
)

#===================================================================
parallel_lines()
set(MAIN_TEST_EXE_NAME main.app)
debug_info()
set(MAIN_TEST_SRC_LIST src/main/main.cpp)

add_executable(${MAIN_TEST_EXE_NAME}  ${MAIN_TEST_SRC_LIST})

target_link_libraries(${MAIN_TEST_EXE_NAME}
  ${catkin_LIBRARIES} ${OTHER_LINK_LIST}
)
parallel_lines()

