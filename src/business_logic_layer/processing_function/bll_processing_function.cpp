/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: bll_processing_function.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月25日
  最近修改:
  功能描述   : 业务逻辑处理功能类定义
  函数列表:
  修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "bll_processing_function.h"

#include "bll_timer.h"
#include "bll_cliff.h"
#include "bll_bumper.h"
#include "bll_rotate.h"
#include "bll_wheel_drop.h"
#include "bll_ultrasonic.h"
#include "bll_trajectory.h"
#include "bll_motion_control.h"
#include "bll_partial_cleaning.h"

#include "cfg_if_mobile_robot.h"
#include "cfg_if_version.h"
#include "debug_function.h"

/******************************************************************************
 * 外部变量定义
 ******************************************************************************/

/******************************************************************************
 * 外部函数定义
 ******************************************************************************/

/******************************************************************************
 * 全局变量
 ******************************************************************************/

/******************************************************************************
 * 宏定义
 ******************************************************************************/

/******************************************************************************
 * 常量定义
 ******************************************************************************/

/******************************************************************************
 * 枚举类型
 ******************************************************************************/

/******************************************************************************
 * 结构体类型
 ******************************************************************************/

/******************************************************************************
 * 类定义
 ******************************************************************************/
pthread_mutex_t bll_processing_function::mutex_;
bll_processing_function* bll_processing_function::p_instance_ = nullptr;

/*****************************************************************************
 函 数 名: bll_processing_function.bll_processing_function
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_processing_function
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_processing_function::bll_processing_function()
{

}

/*****************************************************************************
 函 数 名: bll_processing_function.~bll_processing_function
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_processing_function
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_processing_function::~bll_processing_function()
{

}

/*****************************************************************************
 函 数 名: bll_processing_function.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: bll_processing_function*
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_processing_function* bll_processing_function::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr == p_instance_)
		{
			p_instance_ = new bll_processing_function();
		}
		pthread_mutex_unlock(&mutex_);
	}
	return p_instance_;
}

/*****************************************************************************
 函 数 名: bll_processing_function.release_instance
 功能描述  : 释放实例
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_processing_function::release_instance(void)
{
	if (nullptr != p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr != p_instance_)
		{
			delete p_instance_;
			p_instance_ = nullptr;
		}
		pthread_mutex_unlock(&mutex_);
	}
}

/*****************************************************************************
 函 数 名: bll_processing_function.initialize
 功能描述  : 初始化
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_processing_function::initialize(void)
{
	initialize_version();
	initialize_fuction();
}

/*****************************************************************************
 函 数 名: bll_processing_function.function_processor
 功能描述  : 移动机器人功能处理机
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_processing_function::function_processor ( void )
{
	functional_mode();
	sensors_deal();
	
	bll_rotate* p_rotate = bll_rotate::get_instance();
	p_rotate->rotate_operation();

	bll_partial_cleaning* p_partial_cleaning = bll_partial_cleaning::get_instance();
	p_partial_cleaning->local_cover_movement();
}

/*****************************************************************************
 函 数 名: bll_processing_function.initialize_version
 功能描述  : 初始化版本信息
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_processing_function::initialize_version(void)
{
	uint8_t i = 0;
	uint32_t fields = 0;
	uint32_t firmware_value[8] = {0,0,0,0,0,0,0,1};
	uint32_t hardware_value[8] = {0,0,0,0,0,0,0,0};
	
	fields = cfg_if_get_version_fields();
	for (i = 0; i < fields; ++i)
	{
		cfg_if_set_firmware_version_info(i, firmware_value[i]);
	}
	
	for (i = 0; i < fields; ++i)
	{
		cfg_if_set_hardware_version_info(i, hardware_value[i]);
	}

	cfg_if_set_serial_number("1234567890ABCDEFGHIJKLMNOPQRSTUVWXYZ");
	cfg_if_set_uboot_version("U-Boot 2014.10-RK3288-02 (Mar 20 2017 - 14:29:00);#Boot ver: 2017-03-20#2.17");
	cfg_if_set_kernerl_version("Linux firefly 3.10.0 #7 SMP PREEMPT Thu Apr 20 10:53:28 CST 2017 armv7l armv7l armv7l GNU/Linux;Ubuntu 14.04.1 LTS");
	
	cfg_if_print_version();
}

/*****************************************************************************
 函 数 名: bll_processing_function.initialize_fuction
 功能描述  : 初始化功能模块
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_processing_function::initialize_fuction(void)
{
	bll_trajectory* p_trajectory_instance = bll_trajectory::get_instance();
	p_trajectory_instance->register_trajectory_msgs();

	bll_timers* p_timers_instance = bll_timers::get_instance();
	p_timers_instance->register_time_callback();

	bll_rotate* p_rotate_instance = bll_rotate::get_instance();
	p_rotate_instance->clear_monitor_angle_data();
}

/*****************************************************************************
 函 数 名: bll_processing_function.sensors_deal
 功能描述  : 检测传感器处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_processing_function::sensors_deal(void)
{
	static bll_ultrasoic* p_ultrasoic_instance = bll_ultrasoic::get_instance();
	p_ultrasoic_instance->ultrasonic_sensor_respond_deal();
	
	static bll_bumper* p_bumper_instance = bll_bumper::get_instance();
	p_bumper_instance->bumper_sensor_respond_deal();
	
	static bll_wheel_drop* p_wheel_drop_instance = bll_wheel_drop::get_instance();
	p_wheel_drop_instance->wheel_drop_sensor_respond_deal();
	
	static bll_cliff* p_cliff_instance = bll_cliff::get_instance();
	p_cliff_instance->cliff_sensor_respond_deal();
}

/*****************************************************************************
 函 数 名: bll_processing_function.functional_mode
 功能描述  : 根据当前的行为模式状态运行相应的功能
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_processing_function::functional_mode ( void )
{
	ACTION_STATUS_ENUM action;
	bll_motion_control* p_motion_control = bll_motion_control::get_instance();
	
	cfg_if_print_change_action();
	action = cfg_if_get_curr_action();
	switch ( action )
	{
		case STOP:
			p_motion_control->stop();
			break;
		case GO_FORWARD:
			p_motion_control->go_forward();
			break;
		case GO_BACK:
			p_motion_control->go_back();
			break;
		case TURN_LEFT:
			p_motion_control->turn_left();
			break;
		case TURN_RIGHT:
			p_motion_control->turn_right();
			break;
		case PIVOT:
			p_motion_control->pivot();
			break;
		case TURN_BACK_CLOCKWISE :
			p_motion_control->turn_back_clockwise();
			break;
		case TURN_BACK_ANTICLOCKWISE :
			p_motion_control->turn_back_anticlockwise();
			break;
		case TURN_RIGHT_ANGLE_CLOCKWISE :
			p_motion_control->turn_right_angle_clockwise();
			break;
		case TURN_RIGHT_ANGLE_ANTICLOCKWISE :
			p_motion_control->turn_right_angle_anticlockwise();
			break;
		default:
			debug_print_warnning("p_motion_control->stop() action=%d", action);
			p_motion_control->stop();
			break;
	}
}

/******************************************************************************
 * 内部函数定义
 ******************************************************************************/


