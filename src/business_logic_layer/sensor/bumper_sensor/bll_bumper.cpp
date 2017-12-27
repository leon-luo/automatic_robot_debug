/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: bll_bumper.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月21日
  最近修改:
  功能描述   : 碰撞传感器类定义
  函数列表:
  修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "bll_bumper.h"

#include "bll_rotate.h"

#include "cfg_if_modulate.h"
#include "cfg_if_mobile_robot.h"
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
pthread_mutex_t bll_bumper::mutex_;
bll_bumper* bll_bumper::p_instance_ = nullptr;

/*****************************************************************************
 函 数 名: bll_bumper.bll_bumper
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_bumper
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_bumper::bll_bumper()
{

}

/*****************************************************************************
 函 数 名: bll_bumper.~bll_bumper
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_bumper
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_bumper::~bll_bumper()
{

}

/*****************************************************************************
 函 数 名: bll_bumper.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: bll_bumper*
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_bumper* bll_bumper::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr == p_instance_)
		{
			p_instance_ = new bll_bumper();
		}
		pthread_mutex_unlock(&mutex_);
	}
	return p_instance_;
}

/*****************************************************************************
 函 数 名: bll_bumper.release_instance
 功能描述  : 释放实例
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_bumper::release_instance(void)
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
 函 数 名: bll_bumper.upate_bumper_state
 功能描述  : 更新碰撞传感器的状态
 输入参数: uint8_t id     
           uint8_t value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_bumper::upate_bumper_state(uint8_t id, uint8_t value)
{
	bool ret = false;
	bool bumper_state = false;
	BUMPER_ID_ENUM bumper_id;

	ret = cfg_if_convert_bumper_id(id, bumper_id);
	if ( false == ret )
	{
		debug_print_warnning("id = %d", id);
		return;
	}
	
	ret = cfg_if_convert_bumper_state(value, bumper_state);
	if ( false == ret )
	{
		debug_print_warnning("value = %d", value);
		return;
	}

	std::string name_str ("< INFO > the ");
	std::string state_str ("unknow!");
	if (LEFT_BUMPER == bumper_id)
	{
		name_str += "left ";
	}
	else if (RIGHT_BUMPER == bumper_id)
	{
		name_str += "right ";
	}
	else if (CENTER_BUMPER == bumper_id)
	{
		name_str += "center ";
	}
	name_str += "bumper ";

	if (true == bumper_state)
	{
		state_str = " pressed.";
	}
	else if (false == bumper_state)
	{
		state_str = " released.";
	}
	std::cout<<name_str<<state_str<<std::endl;
	
	cfg_if_set_bumper_state(bumper_id, bumper_state);
}

/*****************************************************************************
 函 数 名: bll_bumper.monitor_angle_left_bumper_deal
 功能描述  : 左边碰撞处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_bumper::monitor_angle_left_bumper_deal(void)
{
	bool flag = true;
	double angle = 0.0;
	double curr_angle = 0.0;
	ACTION_STATUS_ENUM action = TURN_RIGHT;
	void(bll_rotate::*pf)(void) = NULL;

	bll_rotate* p_rotate_instance = bll_rotate::get_instance();
	flag = p_rotate_instance->test_angle_monitor_is_no_working();
	if (true == flag)
	{
		cfg_if_disable_angular_velocity_ajust();
		curr_angle = cfg_if_get_current_position_angle();
		angle = curr_angle - collide_adjusted_angle_;
		cfg_if_change_curr_action(action);
		pf = &bll_rotate::monitor_angle_left_bumper_respond;
		p_rotate_instance->set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: bll_bumper.monitor_angle_right_bumper_deal
 功能描述  : 右边碰撞处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_bumper::monitor_angle_right_bumper_deal(void)
{
	bool flag = true;
	double angle = 0.0;
	double curr_angle = 0.0;
	ACTION_STATUS_ENUM action = TURN_LEFT;
	void(bll_rotate::*pf)(void) = NULL;

	bll_rotate* p_rotate_instance = bll_rotate::get_instance();
	flag = p_rotate_instance->test_angle_monitor_is_no_working();
	if (true == flag)
	{
		cfg_if_disable_angular_velocity_ajust();
		
		curr_angle = cfg_if_get_current_position_angle();
		angle = curr_angle + collide_adjusted_angle_;
		//string str;
		//get_action_status_str( action , str );
		//debug_print_error("###########Change [%s]=%d; curr_angle = %lf; angle = %lf", str.c_str(), action,curr_angle, angle);
		cfg_if_change_curr_action(action);
		pf = &bll_rotate::monitor_angle_right_bumper_respond;
		p_rotate_instance->set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: bll_bumper.bumper_sensor_respond_deal
 功能描述  : 碰撞响应处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_bumper::bumper_sensor_respond_deal(void)
{
	bool left_state = false;
	bool right_state = false;
	static bool last_left_state = false;
	static bool last_right_state = false;
	BUMPER_ID_ENUM left_id = LEFT_BUMPER;
	BUMPER_ID_ENUM right_id = RIGHT_BUMPER;

	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_bumper_state(left_id, left_state);
	p_mobile_robot->get_bumper_state(right_id, right_state);

	if ((last_left_state != left_state) && (last_right_state != right_state))
	{
		if ((true == left_state) && (true == right_state))
		{
			debug_print_warnning("【left】and【right】==【center】bump！");
			cfg_if_change_curr_action(GO_BACK);
		}
		else if ((true == left_state) && (false == right_state))
		{
			debug_print_warnning("【left】bump！");
			monitor_angle_left_bumper_deal();
		}
		else if ((false == left_state) && (true == right_state))
		{
			debug_print_warnning("【right】bump！");
			monitor_angle_right_bumper_deal();
		}
	}
	else if (last_left_state != left_state)
	{
		last_left_state = left_state;
		if ((true == left_state) && (false == right_state))
		{
			debug_print_warnning("【left】bump！");
			monitor_angle_left_bumper_deal();
		}
	}
	else if (last_right_state != right_state)
	{
		last_right_state = right_state;
		if ((false == left_state) && (true == right_state))
		{
			debug_print_warnning("【right】bump！");
			monitor_angle_right_bumper_deal();
		}
	}
}

/******************************************************************************
 * 内部函数定义
 ******************************************************************************/


