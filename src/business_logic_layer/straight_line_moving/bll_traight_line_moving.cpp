/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: bll_traight_line_moving.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月18日
  最近修改:
  功能描述   : 直行行驶功能类定义
  函数列表:
  修改历史:
  1.日     期: 2017年12月18日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "bll_traight_line_moving.h"

#include <math.h>
#include "line_base.h"
#include "cfg_if_modulate.h"
#include "cfg_if_mobile_robot.h"

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
pthread_mutex_t bll_traight_line_moving::mutex_;
bll_traight_line_moving* bll_traight_line_moving::p_instance_ = nullptr;

/*****************************************************************************
 函 数 名: bll_traight_line_moving.bll_traight_line_moving
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_traight_line_moving
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_traight_line_moving::bll_traight_line_moving()
{

}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.~bll_traight_line_moving
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_traight_line_moving
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_traight_line_moving::~bll_traight_line_moving()
{

}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: bll_traight_line_moving*
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_traight_line_moving* bll_traight_line_moving::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr == p_instance_)
		{
			p_instance_ = new bll_traight_line_moving();
		}
		pthread_mutex_unlock(&mutex_);
	}
	return p_instance_;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.release_instance
 功能描述  : 释放实例
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::release_instance(void)
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
 函 数 名: bll_traight_line_moving.set_traight_line_moving
 功能描述  : 设置直线运行
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_traight_line_moving::set_traight_line_moving(void)
{
	cfg_if_change_curr_action(GO_FORWARD);
	
	update_traight_line_moving_data();
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.test_is_traight_line_moving
 功能描述  : 检测当前是否为直行状态
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_traight_line_moving::test_is_traight_line_moving(void)
{
	bool flag = false;
	ACTION_STATUS_ENUM action;
	
	action = cfg_if_get_curr_action();
	if (GO_FORWARD == action)
	{
		flag = true;
	}

	return flag;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.update_traight_line_moving_data
 功能描述  : 更新直行数据
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::update_traight_line_moving_data(void)
{
	POSE_STRU pos;
	bool flag = false;
	
	flag = test_is_traight_line_moving();
	if (true == flag)
	{
		cfg_if_get_current_position(pos);
		flag = get_traight_line_moving_flag();
		if (false == flag)
		{
			set_traight_line_moving_flag(true);
			set_traight_line_moving_start_pos(pos);
			//debug_print_warnning("traight_line_moving_start_pos{(%lf, %lf) : %lf}", pos.point.x, pos.point.y, pos.angle);
		}
		else
		{
			set_traight_line_moving_current_pos(pos);
		}
	}
	else
	{
		clear_traight_line_moving_data();
	}
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_flag
 功能描述  : 设置当前是否处于直行状态
 输入参数: const bool flag  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_flag(const bool flag)
{
	straight_line_moving_.flag = flag;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_flag
 功能描述  : 获取当前是否处于直行状态
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_traight_line_moving::get_traight_line_moving_flag(void)
{
	return straight_line_moving_.flag;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_start_pos
 功能描述  : 设置直行的起始方向位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_start_pos(const POSE_STRU pos)
{
	straight_line_moving_.start_pos = pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_start_pos
 功能描述  : 获取直行的起始方向位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::get_traight_line_moving_start_pos(POSE_STRU &pos)
{
	pos = straight_line_moving_.start_pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_target_pos
 功能描述  : 设置直行方向目标位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_target_pos(const POSE_STRU pos)
{
	straight_line_moving_.target_pos = pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_target_pos
 功能描述  : 获取直行方向目标位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::get_traight_line_moving_target_pos(POSE_STRU &pos)
{
	pos = straight_line_moving_.target_pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_current_pos
 功能描述  : 设置直行当前的位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_current_pos(const POSE_STRU pos)
{
	straight_line_moving_.current_pos = pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_current_pos
 功能描述  : 获取直行当前的位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::get_traight_line_moving_current_pos(POSE_STRU &pos)
{
	pos = straight_line_moving_.current_pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_direction
 功能描述  : 设置直行行驶的方向角度
 输入参数: const double direction  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_direction(const double direction)
{
	straight_line_moving_.direction = direction;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_direction
 功能描述  : 获取直行行驶的方向角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_traight_line_moving::get_traight_line_moving_direction(void)
{
	return straight_line_moving_.direction;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_distance
 功能描述  : 设置直线行驶的行程距离
 输入参数: const double distance  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_distance(const double distance)
{
	straight_line_moving_.travel_distance = distance;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_distance
 功能描述  : 获取直线行驶的行程距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_traight_line_moving::get_traight_line_moving_distance(void)
{
	return straight_line_moving_.travel_distance;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_finish_all_route_done
 功能描述  : 设置直线行驶是否已经到达目标点完成所有行程
 输入参数: const bool flag  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_finish_all_route_done(const bool flag)
{
	straight_line_moving_.all_route_done = flag;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_finish_all_route_done
 功能描述  : 获取直线行驶是否已经到达目标点完成所有行程
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_traight_line_moving::get_finish_all_route_done(void)
{
	return straight_line_moving_.all_route_done;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_finish_part_route_done
 功能描述  : 设置直线显示完成一部分的标志
 输入参数: const bool flag  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_finish_part_route_done(const bool flag)
{
	straight_line_moving_.part_route_done = flag;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_finish_part_route_done
 功能描述  : 获取直线显示完成一部分的标志
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_traight_line_moving::get_finish_part_route_done(void)
{
	return straight_line_moving_.part_route_done;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_data
 功能描述  : 设置直行数据
 输入参数: const STRAIGHT_LINE_MOVING_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_data(const STRAIGHT_LINE_MOVING_STRU data)
{
	straight_line_moving_ = data;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_data
 功能描述  : 获取直行数据
 输入参数: STRAIGHT_LINE_MOVING_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::get_traight_line_moving_data(STRAIGHT_LINE_MOVING_STRU &data)
{
	data = straight_line_moving_;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_direction_angle
 功能描述  : 获取直线行驶的运行方向角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_traight_line_moving::get_traight_line_moving_direction_angle(void)
{
	return straight_line_moving_.start_pos.angle;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.clear_traight_line_moving_data
 功能描述  : 清除直行数据
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::clear_traight_line_moving_data(void)
{
	POSE_STRU pos;
	pos.angle = 0.0;
	pos.point.x = 0.0;
	pos.point.y = 0.0;
	straight_line_moving_.flag = false;
	straight_line_moving_.start_pos = pos;
	straight_line_moving_.target_pos = pos;
	straight_line_moving_.current_pos = pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.straight_driving_adjust_angle
 功能描述  : 直行调整角度
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::straight_driving_adjust_angle(void)
{
	double diff = 0.0;
	double curr_angle = 0.0;
	double direction_angle = 0.0;
	double angular_velocity = 0.0;
	bool is_clockwise = false;
	
	curr_angle = cfg_if_get_current_position_angle();
	direction_angle = get_traight_line_moving_direction_angle();
	angel_base angel_base_instance;
	diff = angel_base_instance.get_angle_differences(curr_angle, direction_angle);
	if (diff > 0.001)
	{
		if ( diff <= 0.1) {
			diff = 0.03;
		}
		else if ( diff <= 0.5) {
			diff = 0.04;
		}
		else if ( diff <= 1.0) {
			angular_velocity = 0.05;
		}
		else if ( diff <= 2.5) {
			angular_velocity = 0.08;
		}
		else if ( diff <= 5.0) {
			angular_velocity = 0.15;
		}
		else {
			angular_velocity = 0.4;
		}
		
		is_clockwise = angel_base_instance.test_angle_rotate_direction_is_clockwise(curr_angle, direction_angle);
		if (true == is_clockwise)
		{
			angular_velocity = -angular_velocity;
		}
		
		cfg_modulate* p_modulate = cfg_modulate::get_instance();
		p_modulate->endble_angular_velocity_ajust(angular_velocity);
	}
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.straight_driving_adjust_speed
 功能描述  : 直线调节速度
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::straight_driving_adjust_speed(void)
{
	bool flag = false;
	double temp = 0.0;
	double distance = 0.0;
	double linear_velocity = 0.0;
	double angular_velocity = 0.0;
	
	cfg_if_get_velocity(linear_velocity, angular_velocity);
	temp = fabs(linear_velocity);
	distance = get_distance_to_traight_line_moving_target_pos();
	if ((distance < 0.12) && (distance >= 0.08))
	{
		if (temp >= 0.2)
		{
			linear_velocity = 0.2;
			flag = true;
		}
	}
	else if ((distance < 0.08) && (distance >= 0.04))
	{
		if (temp >= 0.2)
		{
			linear_velocity = 0.15;
			flag = true;
		}
	}
	else if ((distance < 0.04) && (distance >= 0.02))
	{
		if (temp >= 0.15)
		{
			linear_velocity = 0.12;
			flag = true;
		}
	}
	else if ((distance < 0.02) && (distance >= 0.0))
	{
		if (temp >= 0.1)
		{
			linear_velocity = 0.1;
			flag = true;
		}
	}

	if (true == flag)
	{
		cfg_if_endble_linear_velocity_ajust(linear_velocity);
	}
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_distance_to_traight_line_moving_start_pos
 功能描述  : 获取当前位置到开始直行位置的距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_traight_line_moving::get_distance_to_traight_line_moving_start_pos(void)
{
	POSE_STRU pos;
	POSE_STRU curr_pos;
	double distance = 0.0;

	cfg_if_get_current_position(curr_pos);
	get_traight_line_moving_start_pos(pos);
	line_base line_base_instance;
	distance = line_base_instance.get_distance(curr_pos.point.x, curr_pos.point.y, pos.point.x, pos.point.y);
	//debug_print_info("distance(%lf) = |curr_pos.point=(%lf, %lf)<-->pos.point=(%lf, %lf)|", distance, curr_pos.point.x, curr_pos.point.y, pos.point.x, pos.point.y);
	return distance;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_distance_to_traight_line_moving_target_pos
 功能描述  : 获取到目标点的距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_traight_line_moving::get_distance_to_traight_line_moving_target_pos(void)
{
	POSE_STRU pos;
	POSE_STRU curr_pos;
	double distance = 0.0;

	cfg_if_get_current_position(curr_pos);
	get_traight_line_moving_target_pos(pos);
	line_base line_base_instance;
	distance = line_base_instance.get_distance(curr_pos.point.x, curr_pos.point.y, pos.point.x, pos.point.y);
	//debug_print_info("distance(%lf) = |curr_pos.point=(%lf, %lf)<-->pos.point=(%lf, %lf)|", distance, curr_pos.point.x, curr_pos.point.y, pos.point.x, pos.point.y);
	return distance;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_straight_moving_refer_start_pose
 功能描述  : 设置直线行驶参考起点位置
 输入参数: const POSE_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_straight_moving_refer_start_pose(const POSE_STRU data)
{
	start_pose_ = data;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_straight_moving_refer_start_pose
 功能描述  : 获取直线行驶参考起点位置
 输入参数: POSE_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::get_straight_moving_refer_start_pose(POSE_STRU &data)
{
	data = start_pose_;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_straight_moving_refer_target_pose
 功能描述  : 设置直线行驶参考目标位置
 输入参数: const POSE_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_straight_moving_refer_target_pose(const POSE_STRU data)
{
	target_pose_ = data;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_straight_moving_refer_target_pose
 功能描述  : 获取直线行驶参考目标位置
 输入参数: POSE_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::get_straight_moving_refer_target_pose(POSE_STRU &data)
{
	data = target_pose_;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_straight_moving_refer_pose
 功能描述  : 获取直线行驶参考起始点和目标点位置
 输入参数: POSE_STRU &start   
           POSE_STRU &target  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::get_straight_moving_refer_pose(POSE_STRU &start,POSE_STRU &target)
{
	start = start_pose_;
	target = target_pose_;
}

/******************************************************************************
 * 内部函数定义
 ******************************************************************************/


