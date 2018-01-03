/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: bll_rotate.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月18日
  最近修改:
  功能描述   : 旋转功能类定义
  函数列表:
  修改历史:
  1.日     期: 2017年12月18日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "bll_rotate.h"

#include "bll_motion_control.h"
#include "bll_partial_cleaning.h"
#include "bll_traight_line_moving.h"

#include "cfg_if_modulate.h"
#include "cfg_if_mobile_robot.h"
#include "debug_function.h"

#include <math.h>

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
pthread_mutex_t bll_rotate::mutex_;
bll_rotate* bll_rotate::p_instance_ = NULL;

/*****************************************************************************
 函 数 名: bll_rotate.bll_rotate
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_rotate
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_rotate::bll_rotate()
{

}

/*****************************************************************************
 函 数 名: bll_rotate.~bll_rotate
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_rotate
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_rotate::~bll_rotate()
{

}

/*****************************************************************************
 函 数 名: bll_rotate.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: bll_rotate*
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_rotate* bll_rotate::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr == p_instance_)
		{
			p_instance_ = new bll_rotate();
		}
		pthread_mutex_unlock(&mutex_);
	}
	return p_instance_;
}

/*****************************************************************************
 函 数 名: bll_rotate.release_instance
 功能描述  : 释放实例
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::release_instance(void)
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
 函 数 名: bll_rotate.set_monitor_angle_data
 功能描述  : 设置监测角度状态
 输入参数: const ANGLE_MONITOR_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::set_monitor_angle_data(const ANGLE_MONITOR_STRU data)
{
	angle_monitor_ = data;
}

/*****************************************************************************
 函 数 名: bll_rotate.get_monitor_angle_data
 功能描述  : 获取监测角度状态
 输入参数: ANGLE_MONITOR_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::get_monitor_angle_data(ANGLE_MONITOR_STRU &data)
{
	data = angle_monitor_;
}

/*****************************************************************************
 函 数 名: bll_rotate.clear_monitor_angle_data
 功能描述  : 清除角度监测状态
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::clear_monitor_angle_data(void)
{
	ANGLE_MONITOR_STRU data;

	data.angle = 0.0;
	data.stage = MONITOR_SLEEP;
	set_monitor_angle_data(data);
}

/*****************************************************************************
 函 数 名: bll_rotate.set_monitor_angle_stage
 功能描述  : 设置监测角度的阶段状态
 输入参数: const MONITOR_ENUM stage  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::set_monitor_angle_stage(const MONITOR_ENUM stage)
{
	angle_monitor_.stage = stage;
}

/*****************************************************************************
 函 数 名: bll_rotate.get_monitor_angle_stage
 功能描述  : 获取监测角度的阶段状态
 输入参数: MONITOR_ENUM &stage  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::get_monitor_angle_stage(MONITOR_ENUM &stage)
{
	stage = angle_monitor_.stage;
}

/*****************************************************************************
 函 数 名: bll_rotate.test_angle_monitor_is_no_working
 功能描述  : 检测角度旋转检测是否不在检测工作中
 输入参数: void  
 输出参数: 无
 返 回 值: bool 不正在检测则返回真，否则返回假
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_rotate::test_angle_monitor_is_no_working(void)
{
	bool ret = false;
	
	if (MONITOR_SLEEP == angle_monitor_.stage)
	{
		ret = true;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: bll_rotate.set_monitor_angle_respond_goal
 功能描述  : 设置目标响应角度
 输入参数: const double angle  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::set_monitor_angle_respond_goal(const double angle)
{
	angle_monitor_.angle = angle;
}

/*****************************************************************************
 函 数 名: bll_rotate.get_monitor_angle_respond_goal
 功能描述  : 获取目标响应角度
 输入参数: 无
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 22017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_rotate::get_monitor_angle_respond_goal(void)
{
	return angle_monitor_.angle;
}

/*****************************************************************************
 函 数 名: bll_rotate.set_monitor_angle_respond_func
 功能描述  : 设置角度到达以后的响应功能函数
 输入参数: PF_FUNC pf  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::set_monitor_angle_respond_func(void(bll_rotate::*pf)(void))
{
	pf_monitor_ = pf;
}

/*****************************************************************************
 函 数 名: bll_rotate.get_monitor_angle_respond_func
 功能描述  : 获取角度到达以后的响应功能函数指针
 输入参数: PF_FUNC &pf  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::get_monitor_angle_respond_func(void(bll_rotate::*pf)(void))
{
	pf = pf_monitor_;
}

/*****************************************************************************
 函 数 名: bll_rotate.set_monitor_angle_respond_goal_and_func
 功能描述  : 设置监测目标角度和到达目标角度以后的响应功能函数
 输入参数: double angle  
           PF_FUNC pf    
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::set_monitor_angle_respond_goal_and_func(double angle, void(bll_rotate::*pf)(void))
{
	set_monitor_angle_respond_goal(angle);
	set_monitor_angle_respond_func(pf);
}


/*****************************************************************************
 函 数 名: bll_rotate.set_monitor_angle_rotate_call_back
 功能描述  : 注册监测指定角度以后响应的功能
 输入参数: double angle       
           void*pf_call_back  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::set_monitor_angle_rotate_call_back(double angle, void(bll_rotate::*pf_call_back)(void))
{
	bool flag = false;
	void(bll_rotate::*pf)(void) = NULL;
	MONITOR_ENUM stage = MONITOR_RUNNING;
	
	flag = test_angle_monitor_is_no_working();
	if (true == flag)
	{
		set_monitor_angle_stage(stage);

		if (NULL != pf_call_back)
		{
			pf = pf_call_back;
		}
		
		set_monitor_angle_respond_goal_and_func(angle, pf);
	}
}

#ifdef BLUEWAYS_DEBUG
/*****************************************************************************
 函 数 名: bll_rotate.test_adjust_velocity
 功能描述  : 检查是否需要调整速度
 输入参数: double &linear_velocity   
           double &angular_velocity  
 输出参数: double linear_velocity调整的线速度
           double angular_velocity调整的角速度
 返 回 值: bool 需要返回真否则返回假
 
 修改历史:
  1.日     期: 2017年12月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_rotate::test_adjust_velocity(double &linear_velocity, double &angular_velocity)
{
	bool flag = true;
	double veer = 1.0;
	double diff = 0.0;
	double offset = 0.0;
	double linear_v = linear_velocity;
	double angular_v = angular_velocity;
	ACTION_STATUS_ENUM action;

	diff = get_curr_angle_difference_respond();
	offset = fabs(diff);
	action = cfg_if_get_curr_action();
	if (PIVOT == action)
	{
		if ( offset <= 0.2) {
			angular_v = 0.20;
		}else if ( offset <= 1.0) {
			angular_v = 0.20;
		}else if ( offset <= 2.5) {
			angular_v = 0.21;
		}else if ( offset <= 5.0) {
			angular_v = 0.22;
		}else if ( offset <= 10.0) {
			angular_v = 0.23;
		}else if ( offset <= 15.0) {
			angular_v = 0.4;
		}else {
			flag = false;
		}
	}
	else if((TURN_BACK_CLOCKWISE == action) || (TURN_BACK_ANTICLOCKWISE == action))
	{
		if ( offset <= 0.2) {
			angular_v = 0.21;
			linear_v = 0.0;
		}else if ( offset <= 1.0) {
			angular_v = 0.215;
			linear_v = 0.0;
		}else if ( offset <= 2.5) {
			angular_v = 0.22;
			linear_v = 0.02;
		}else if ( offset <= 5.0) {
			angular_v = 0.3;
			linear_v = 0.05;
		}else if ( offset <= 15.0) {
			angular_v = 0.4;
			linear_v = 0.07;
		}else {
			flag = false;
		}
	}
	else if((TURN_LEFT == action) 
		|| (TURN_RIGHT == action)
		|| (TURN_RIGHT_ANGLE_CLOCKWISE == action)
		|| (TURN_RIGHT_ANGLE_ANTICLOCKWISE == action))
	{
		if ( offset <= 0.2 ) {
			angular_v = 0.22;
		}else if ( offset <= 2.0 ) {
			angular_v = 0.23;
		}else if ( offset <= 5.0 ) {
			angular_v = 0.24;
		}else if ( offset <= 8.0 ) {
			angular_v = 0.25;
		}else if ( offset <= 15.0 ) {
			angular_v = 0.4;
		}else {
			angular_v = 0.8;
			flag == false;
		}
		linear_v = 0.0;
	}
	else {
		flag = false;
	}

	if (true == flag)
	{
		if (diff < 0.0)
		{
			angular_v = -fabs(angular_v);
		}
		linear_velocity = linear_v;
		angular_velocity = angular_v;
	}

	return flag;
}

#elif defined KOBUKI_DEBUG
bool bll_rotate::test_adjust_velocity(double &linear_velocity, double &angular_velocity)
{
	bool flag = true;
	double veer = 1.0;
	double diff = 0.0;
	double offset = 0.0;
	double linear_v = linear_velocity;
	double angular_v = angular_velocity;
	ACTION_STATUS_ENUM action;

	diff = get_curr_angle_difference_respond();
	offset = fabs(diff);
	action = cfg_if_get_curr_action();
	if (PIVOT == action)
	{
		if ( offset <= 0.2) {
			angular_v = 0.18;
		}else if ( offset <= 1.0) {
			angular_v = 0.18;
		}else if ( offset <= 2.5) {
			angular_v = 0.19;
		}else if ( offset <= 5.0) {
			angular_v = 0.2;
		}else if ( offset <= 10.0) {
			angular_v = 0.22;
		}else if ( offset <= 15.0) {
			angular_v = 0.4;
		}else {
			flag = false;
		}
	}
	else if((TURN_BACK_CLOCKWISE == action) || (TURN_BACK_ANTICLOCKWISE == action))
	{
		if ( offset <= 0.2) {
			angular_v = 0.20;
			linear_v = 0.0;
		}else if ( offset <= 1.0) {
			angular_v = 0.21;
			linear_v = 0.0;
		}else if ( offset <= 2.5) {
			angular_v = 0.22;
			linear_v = 0.02;
		}else if ( offset <= 5.0) {
			angular_v = 0.3;
			linear_v = 0.05;
		}else if ( offset <= 15.0) {
			angular_v = 0.4;
			linear_v = 0.07;
		}else {
			flag = false;
		}
	}
	else if((TURN_RIGHT_ANGLE_CLOCKWISE == action) || (TURN_RIGHT_ANGLE_ANTICLOCKWISE == action))
	{
		if ( offset <= 0.2 ) {
			angular_v = 0.21;
		}else if ( offset <= 2.0 ) {
			angular_v = 0.215;
		}else if ( offset <= 5.0 ) {
			angular_v = 0.21;
		}else if ( offset <= 8.0 ) {
			angular_v = 0.25;
		}else if ( offset <= 15.0 ) {
			angular_v = 0.4;
		}else {
			flag == false;
		}
	}
	else {
		flag = false;
	}

	if (true == flag)
	{
		if (diff < 0.0)
		{
			angular_v = -fabs(angular_v);
		}
		linear_velocity = linear_v;
		angular_velocity = angular_v;
	}

	return flag;
}

#elif defined KOBUKI_SIMULATOR_DEBUG
bool bll_rotate::test_adjust_velocity(double &linear_velocity, double &angular_velocity)
{
	bool flag = true;
	double veer = 1.0;
	double diff = 0.0;
	double offset = 0.0;
	double linear_v = linear_velocity;
	double angular_v = angular_velocity;
	ACTION_STATUS_ENUM action;

	diff = get_curr_angle_difference_respond();
	offset = fabs(diff);
	action = cfg_if_get_curr_action();
	if (PIVOT == action)
	{
		if ( offset <= 0.2) {
			angular_v = 0.02;//0.001;
		}else if ( offset <= 1.0) {
			angular_v = 0.02;//0.002;
		}else if ( offset <= 2.5) {
			angular_v = 0.025;
		}else if ( offset <= 5.0) {
			angular_v = 0.05;
		}else if ( offset <= 10.0) {
			angular_v = 0.1;
		}else if ( offset <= 15.0) {
			angular_v = 0.2;
		}else if ( offset <= 30.0) {
			angular_v = 0.15;
		}
		else {
			flag = false;
		}
	}
	else if((TURN_BACK_CLOCKWISE == action) || (TURN_BACK_ANTICLOCKWISE == action))
	{
		if ( offset <= 0.2) {
			angular_v = 0.015;
			linear_v = 0.0;
		}else if ( offset <= 1.0) {
			angular_v = 0.02;
			linear_v = 0.0;
		}else if ( offset <= 2.5) {
			angular_v = 0.025;
			linear_v = 0.02;
		}else if ( offset <= 6.0) {
			angular_v = 0.15;
			linear_v = 0.05;
		}else if ( offset <= 10.0) {
			angular_v = 0.2;
			linear_v = 0.07;
		}else if ( offset <= 30.0) {
			angular_v = 0.25;
			linear_v = 0.07;
		}else {
			flag = false;
		}
	}
	else if((TURN_RIGHT_ANGLE_CLOCKWISE == action) || (TURN_RIGHT_ANGLE_ANTICLOCKWISE == action))
	{
		if ( offset <= 0.2 ) {
			angular_v = 0.005;
		}else if ( offset <= 1.0 ) {
			angular_v = 0.01;
		}else if ( offset <= 2.0 ) {
			angular_v = 0.02;
		}else if ( offset <= 5.0 ) {
			angular_v = 0.05;
		}else if ( offset <= 8.0 ) {
			angular_v = 0.1;
		}else if ( offset <= 10.0 ) {
			angular_v = 0.15;
		}else if ( offset <= 30.0 ) {
			angular_v = 0.2;
		}else {
			flag == false;
		}
	}
	else
	{
		flag == false;
		debug_print_fatal("action=%d", action);
	}

	if (true == flag)
	{
		if (diff < 0.0)
		{
			angular_v = -fabs(angular_v);
		}
		linear_velocity = linear_v;
		angular_velocity = angular_v;
		
		//if (last_offset != offset)
		//{
		//	last_offset = offset;
			debug_print_error("action=%d flag= %d; angular_v=%lf; linear_v=%lf; diff=%lf; angular_velocity=%lf;", action, flag, angular_v, linear_v, diff, angular_velocity);
		//}
	}
	else
	{
		//if (last_offset != offset)
		//{
		//	last_offset = offset;
			debug_print_info("action=%d flag= %d; angular_v=%lf; linear_v=%lf; diff=%lf; angular_velocity=%lf;", action, flag, angular_v, linear_v, diff, angular_velocity);
		//}
	}

	return flag;
}

#endif /* KOBUKI_SIMULATOR_DEBUG */

/*****************************************************************************
 函 数 名: bll_rotate.monitor_angle_adjust_velocity
 功能描述  : 调整速度
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::monitor_angle_adjust_velocity(void)
{
	bool flag = false;
	double linear_velocity = 0.0;
	double angular_velocity = 0.0;

	cfg_if_get_velocity(linear_velocity, angular_velocity);
	flag = test_adjust_velocity(linear_velocity, angular_velocity);
	if ( true == flag) {
		cfg_if_set_velocity(linear_velocity, angular_velocity);
		cfg_if_set_adjust_velocity_flag(flag);

//		double curr = 0.0;
//		double respond = 0.0;
//		static double pre = 0.0;
//		curr = cfg_if_get_current_position_angle();
//		respond = get_monitor_angle_respond_goal();
//		if (pre != curr) {
//			pre = curr;
//			debug_print_info("flag = %d, ||curr(%lf) --> respond(%lf)||, linear_v(%lf); angular_v(%lf)",flag, curr, respond, linear_velocity, angular_velocity);
//		}
	}
	else
	{
		//debug_print_fatal("flag=%d", flag);
	}
}

/*****************************************************************************
 函 数 名: bll_rotate.monitor_stop_rotate
 功能描述  : 停止旋转
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::monitor_stop_rotate(void)
{
	double line_v = 0.0;
	double angular_v = 0.0;
	cfg_if_get_velocity(line_v, angular_v);
	angular_v = 0.0;//停止旋转
	bll_motion_control* p_instance = bll_motion_control::get_instance();
	p_instance->set_motion_control_velocity(line_v, angular_v);
}

/*****************************************************************************
 函 数 名: bll_rotate.monitor_angle_running
 功能描述  : 检测旋转偏移状态是否可以响应
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::monitor_angle_running(void)
{
	MONITOR_ENUM stage;
	double offset = 0.0;
	double precision = 0.08;//0.02;//旋转角度响应精度

	get_monitor_angle_stage(stage);
	if (MONITOR_RUNNING == stage)
	{
		monitor_angle_adjust_velocity();

		double curr_angle = 0.0;
		double respond_angle = 0.0;
		static double pre_offset = 0.0;
		curr_angle = cfg_if_get_current_position_angle();
		respond_angle = get_monitor_angle_respond_goal();
			
		offset = get_curr_angle_difference_respond();

		//if (offset != pre_offset)
		//{
		//	std::cout<<std::endl;
		//	pre_offset = offset;
		//	debug_print_info("[NO MONITOR_RESPOND] :offset(%lf) = [curr_angle(%lf) - respond_angle(%lf)] < precision(%lf)", offset, curr_angle, respond_angle, precision);
		//	std::cout<<std::endl;
		//}
		
		if (fabs(offset) < precision)
		{
			stage = MONITOR_RESPOND;
			monitor_stop_rotate();
			set_monitor_angle_stage(stage);
			
			std::cout<<std::endl;
			debug_print_warnning("[MONITOR_RESPOND] :offset(%lf) = [curr_angle(%lf) - respond_angle(%lf)] < precision(%lf)", offset, curr_angle, respond_angle, precision);
			std::cout<<std::endl;
		}
		else
		{
			//debug_print_info("[NO MONITOR_RESPOND] :offset(%lf) = [curr_angle(%lf) - respond_angle(%lf)] < precision(%lf)", offset, curr_angle, respond_angle, precision);
		}
	}
}

/*****************************************************************************
 函 数 名: bll_rotate.monitor_angle_respond
 功能描述  : 检测旋转角度响应
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::monitor_angle_respond(void)
{
	MONITOR_ENUM stage;
	
	get_monitor_angle_stage(stage);
	if (MONITOR_RESPOND == stage)
	{
		if (NULL != pf_monitor_)
		{
			(this->*pf_monitor_)();
		}
		
		set_monitor_angle_respond_func(NULL);
		clear_monitor_angle_data();
	}
}

/*****************************************************************************
 函 数 名: bll_rotate.rotate_operation
 功能描述  : 旋转业务处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::rotate_operation(void)
{
	monitor_angle_running();
	monitor_angle_respond();
}

/*****************************************************************************
 函 数 名: bll_rotate.left_or_right_bumper_respond
 功能描述  : 左右碰撞以后旋转响应角度以后的响应动作
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::left_or_right_bumper_respond(void)
{
	cfg_if_change_curr_action(GO_FORWARD);
}

/*****************************************************************************
 函 数 名: bll_rotate.monitor_angle_left_bumper_respond
 功能描述  : 左边碰撞旋转响应
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::monitor_angle_left_bumper_respond(void)
{
	left_or_right_bumper_respond();
}

/*****************************************************************************
 函 数 名: bll_rotate.monitor_angle_right_bumper_respond
 功能描述  : 右边碰撞旋转响应
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::monitor_angle_right_bumper_respond(void)
{
	left_or_right_bumper_respond();
}

/*****************************************************************************
 函 数 名: bll_rotate.monitor_angle_turn_back_respond
 功能描述  : 响应检测旋转角度的处理功能
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::monitor_angle_turn_back_respond(void)
{
	POSE_STRU pos;
	ACTION_STATUS_ENUM action;

	action = cfg_if_get_curr_action();
	if ((action == TURN_BACK_ANTICLOCKWISE) || (action == TURN_BACK_CLOCKWISE))
	{
		bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
		p_traight_line_moving->clear_traight_line_moving_data();
		p_traight_line_moving->set_traight_line_moving();
		
		cfg_if_switch_move_direction();
		
		bll_partial_cleaning* p_partial_cleaning = bll_partial_cleaning::get_instance();
		p_partial_cleaning->set_current_position_to_refer_start_pose();
		
		cfg_if_disable_linear_velocity_ajust();
		p_traight_line_moving->get_straight_moving_refer_target_pose(pos);
		p_traight_line_moving->set_traight_line_moving_target_pos(pos);
	}
}

/*****************************************************************************
 函 数 名: bll_rotate.monitor_angle_pivot_respond
 功能描述  : 原地向后旋转到达后的响应
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月7日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::monitor_angle_pivot_respond(void)
{
	POSE_STRU pos;
	
	bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
	p_traight_line_moving->clear_traight_line_moving_data();

	bll_partial_cleaning* p_partial_cleaning = bll_partial_cleaning::get_instance();
	p_partial_cleaning->get_refer_line_start_point_pose(pos);
	
	p_traight_line_moving->set_traight_line_moving_target_pos(pos);
	
	cfg_if_disable_linear_velocity_ajust();
	
	p_traight_line_moving->set_traight_line_moving();
	
	p_partial_cleaning->set_partial_cleaning_state(LOCAL_MOVE_FST_LINE_START);

	cfg_if_switch_move_direction();
	
	p_partial_cleaning->set_current_position_to_refer_start_pose();
}

/*****************************************************************************
 函 数 名: bll_rotate.monitor_angle_turn_to_refer_line_respond
 功能描述  :  检测旋转角度转向参考线方向处理响应
 输入参数: void  
 输出参数: 无
 返 回 值: void
 备      注：为了最快靠近参考线，垂直与参考线方向为最快最优角度
 修改历史:
  1.日     期: 2017年10月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::monitor_angle_turn_to_refer_line_respond(void)
{
	bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
	p_traight_line_moving->clear_traight_line_moving_data();

	bll_partial_cleaning* p_partial_cleaning = bll_partial_cleaning::get_instance();
	p_partial_cleaning->update_refer_line_traight_line_moving_target_pos();
	
	p_traight_line_moving->set_traight_line_moving();
	
	p_partial_cleaning->set_partial_cleaning_state(LOCAL_MOVE_RETURN_REFER_LINE);
}

/*****************************************************************************
 函 数 名: bll_rotate.monitor_angle_turn_to_center_respond
 功能描述  : 回到第二半区的拐直角的处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::monitor_angle_turn_to_center_respond(void)
{
	bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
	p_traight_line_moving->clear_traight_line_moving_data();
	
	cfg_if_disable_linear_velocity_ajust();
	p_traight_line_moving->set_traight_line_moving();
	
	cfg_if_switch_partial_cleaning_part();
	cfg_if_switch_move_direction();
	
	bll_partial_cleaning* p_partial_cleaning = bll_partial_cleaning::get_instance();
	p_partial_cleaning->set_current_position_to_refer_start_pose();
	p_partial_cleaning->update_refer_line_traight_line_moving_target_pos();
	p_partial_cleaning->set_partial_cleaning_state(LOCAL_MOVE_SEC_HALF);
}

/*****************************************************************************
 函 数 名: bll_rotate.monitor_angle_turn_to_original_pose_respond
 功能描述  : 转向原始点的角度到达以后的响应处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::monitor_angle_turn_to_original_pose_respond(void)
{
	POSE_STRU pos;
	bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
	p_traight_line_moving->clear_traight_line_moving_data();

	cfg_if_disable_linear_velocity_ajust();
	
	cfg_if_get_partial_cleaning_original_pose(pos);
	p_traight_line_moving->set_traight_line_moving_target_pos(pos);
	
	p_traight_line_moving->set_traight_line_moving();
	bll_partial_cleaning* p_partial_cleaning = bll_partial_cleaning::get_instance();
	p_partial_cleaning->set_partial_cleaning_state(LOCAL_MOVE_RETURN_CENTER_POS);
}

/*****************************************************************************
 函 数 名: bll_rotate.monitor_angle_turn_to_original_direction_respond
 功能描述  : 转向原始方向角度到达以后的响应处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_rotate::monitor_angle_turn_to_original_direction_respond(void)
{
	bll_partial_cleaning* p_partial_cleaning = bll_partial_cleaning::get_instance();
	p_partial_cleaning->set_partial_cleaning_state(LOCAL_MOVE_ALL_DONE);
	cfg_if_change_curr_action(STOP);
	cfg_if_disable_linear_velocity_ajust();
}

/*****************************************************************************
 函 数 名: bll_rotate.test_rotate_is_over_clockwise
 功能描述  : 测试顺时针旋转是否已经超过预定的检测值
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_rotate::test_rotate_is_over_clockwise(void)
{
	bool ret = false;
	double target = 0.0;
	double current = 0.0;
	
	current = cfg_if_get_current_position_angle();
	target = get_monitor_angle_respond_goal();
	angel_base angel_base_instance;
	ret = angel_base_instance.test_angle_is_over_clockwise(current, target);

	return ret;
}

/*****************************************************************************
 函 数 名: bll_rotate.test_rotate_is_over_anticlockwise
 功能描述  : 测试逆时针旋转是否已经超过预定的检测值
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_rotate::test_rotate_is_over_anticlockwise(void)
{
	bool ret = false;
	double target = 0.0;
	double current = 0.0;
	
	current = cfg_if_get_current_position_angle();
	target = get_monitor_angle_respond_goal();
	angel_base angel_base_instance;
	ret = angel_base_instance.test_angle_is_over_anticlockwise(current, target);
	
	return ret;
}

/*****************************************************************************
 函 数 名: bll_rotate.test_angle_is_over
 功能描述  : 检测角度是否越过目标角度
 输入参数: double current  
           double target   
 输出参数: double &offset
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_rotate::test_angle_is_over(double current, double target, double &offset)
{
	double diff = 0.0;
	bool over_flag = false;
	ROTATE_DIRECTION_ENUM rotate_dir;
	angel_base angel_base_instance;
	
	diff = angel_base_instance.get_angle_differences(current, target);
	rotate_dir = get_curr_rotate_direction();
	if (CLOCKWISE == rotate_dir)
	{
		over_flag = angel_base_instance.test_angle_is_over_clockwise(current, target);
		if (false == over_flag) 
		{
			offset = -fabs(diff);
		}
		else
		{
			offset = fabs(diff);
		}
	}
	else if (ANTICLOCKWISE == rotate_dir)
	{
		over_flag = angel_base_instance.test_angle_is_over_anticlockwise(current, target);
		if (true == over_flag) 
		{
			offset = -fabs(diff);
		}
		else
		{
			offset = fabs(diff);
		}
	}
	else
	{
		//debug_print_fatal("diff = %lf  over_flag=%d", diff, over_flag);
	}
	
	return over_flag;
}

/*****************************************************************************
 函 数 名: bll_rotate.get_curr_angle_difference_respond
 功能描述  : 获取当前角度与响应角度的偏差
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_rotate::get_curr_angle_difference_respond(void)
{
	double diff = 0.0;
	double current = 0.0;
	double target = 0.0;
	bool over_flag = false;
	
	current = cfg_if_get_current_position_angle();
	target = get_monitor_angle_respond_goal();
	over_flag = test_angle_is_over(current, target, diff);
	
	return diff;
}

/*****************************************************************************
 函 数 名: bll_rotate.get_curr_rotate_direction
 功能描述  : 获取当前旋转方向
 输入参数: void  
 输出参数: 无
 返 回 值: ROTATE_DIRECTION_ENUM
 
 修改历史:
  1.日     期: 2017年11月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
ROTATE_DIRECTION_ENUM bll_rotate::get_curr_rotate_direction(void)
{
	ACTION_STATUS_ENUM action;
	ROTATE_DIRECTION_ENUM rotate;
	
	action = cfg_if_get_curr_action();
	switch ( action )
	{
		case TURN_RIGHT :
		case TURN_BACK_CLOCKWISE :
		case TURN_RIGHT_ANGLE_CLOCKWISE :
			rotate = CLOCKWISE;
			break;
		case TURN_LEFT :
		case TURN_BACK_ANTICLOCKWISE :
		case TURN_RIGHT_ANGLE_ANTICLOCKWISE :
		case PIVOT :
			rotate = ANTICLOCKWISE;
			break;
		default:
			break;
	}

	return rotate;
}

/******************************************************************************
 * 内部函数定义
 ******************************************************************************/


