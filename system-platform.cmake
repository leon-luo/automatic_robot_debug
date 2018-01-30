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

##包含文件
include(print.cmake)

##支持IF(A) ELSE()的写法
#SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS ON) 

#/*****************************************************************************
# 函 数 名: include_sub_directories_recursively
# 功能描述: 递归包含目录树
# 输入参数: root_dir:顶层目录
# 输出参数: 无
# 返 回 值: 无
# 
# 修改历史:
#  1.日     期: 208年01月30日
#    作     者: Leon
#    修改内容: 新生成函数
#*****************************************************************************/
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

#/*****************************************************************************
# 函 数 名: print_operation_system_info
# 功能描述: 打印系统定义变量信息
# 输入参数: 无
# 输出参数: 无
# 返 回 值: 无
# 
# 修改历史:
#  1.日     期: 208年01月30日
#    作     者: Leon
#    修改内容: 新生成函数
#*****************************************************************************/
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

#/*****************************************************************************
# 函 数 名: config_compile_options
# 功能描述: 配置编译选项
# 输入参数: 无
# 输出参数: 无
# 返 回 值: 无
# 
# 修改历史:
#  1.日     期: 208年01月30日
#    作     者: Leon
#    修改内容: 新生成宏
#*****************************************************************************/
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

#/*****************************************************************************
# 函 数 名: config_compilers
# 功能描述: 配置编译器
# 输入参数: 无
# 输出参数: 无
# 返 回 值: 无
# 
# 修改历史:
#  1.日     期: 208年01月30日
#    作     者: Leon
#    修改内容: 新生成宏
#*****************************************************************************/
macro(config_compilers)
	parallel_lines()
	message(STATUS "config_compilers()")
	if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
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
	elseif ()
		message(STATUS "|************ Current processor is \"${CMAKE_SYSTEM_PROCESSOR}\" **************|")
		
		set(COMPILER_DIR /usr/bin)
		set(COMPILER_NAME_PREFIX arm-linux-gnueabihf-)
	endif()

	set(C_COMPILER_NAME   ${COMPILER_NAME_PREFIX}gcc)
	set(CXX_COMPILER_NAME ${COMPILER_NAME_PREFIX}g++)

	set(C_COMPILER_PATH   ${COMPILER_DIR}/${C_COMPILER_NAME})
	set(CXX_COMPILER_PATH ${COMPILER_DIR}/${CXX_COMPILER_NAME})

	#编译器必须配置,可以使用交叉编译器绝对地址
	SET(CMAKE_C_COMPILER   ${C_COMPILER_PATH})      #指定C交叉编译器
	SET(CMAKE_CXX_COMPILER ${CXX_COMPILER_PATH})    #指定C++交叉编译器

	parallel_lines()
endmacro()

#/*****************************************************************************
# 函 数 名: config_link_lib_and_include_directories
# 功能描述: 配置连接库与包含头文件路径
# 输入参数: 无
# 输出参数: 无
# 返 回 值: 无
# 
# 修改历史:
#  1.日     期: 208年01月30日
#    作     者: Leon
#    修改内容: 新生成宏
#*****************************************************************************/
macro(config_link_lib_and_include_directories)
	parallel_lines()
	message(STATUS "config_link_lib_and_include_directories()")
	if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
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
	elseif ()
		message(STATUS "|************ Current processor is \"${CMAKE_SYSTEM_PROCESSOR}\" **************|")

		SET(ROS_INSTALL_DIRECTORY /opt/ros/indigo)                             #指定ROS安装路径
		SET(ROS_LIBRARY_DIRECTORY ${ROS_INSTALL_DIRECTORY}/lib)                #指定ROS库路径
		SET(ROS_INCLUDE_DIRECTORY ${ROS_INSTALL_DIRECTORY}/include)            #指定ROS头文件路径
	endif()

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

#/*****************************************************************************
# 函 数 名: print_cmake_info
# 功能描述: 打印CMAKE平台版本系统定义变量等相关信息
# 输入参数: 无
# 输出参数: 无
# 返 回 值: 无
# 
# 修改历史:
#  1.日     期: 208年01月30日
#    作     者: Leon
#    修改内容: 新生成函数
#*****************************************************************************/
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
	print_variate(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS)
	blank_line()
	print_variate(LD_LIBRARY_PATH)
	print_variate(BUILD_SHARED_LIBS)
	print_variate(LIBRARY_OUTPUT_PATH)
	print_variate(EXECUTABLE_OUTPUT_PATH)
	print_variate(OTHER_LINK_LIST)
	parallel_lines()
endfunction()

#/*****************************************************************************
# 函 数 名: debug_info
# 功能描述: 打印调试相关信息
# 输入参数: 无
# 输出参数: 无
# 返 回 值: 无
# 
# 修改历史:
#  1.日     期: 208年01月30日
#    作     者: Leon
#    修改内容: 新生成函数
#*****************************************************************************/
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