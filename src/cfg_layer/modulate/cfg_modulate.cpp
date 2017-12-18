/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: cfg_modulate.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年11月9日
  最近修改:
  功能描述: 调节功能
  函数列表:
  修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "cfg_modulate.h"

#include <stdlib.h>
#include <math.h>
#include "debug_function.h"
#include "pid.h"

/******************************************************************************
 * 外部变量声明
 ******************************************************************************/

/******************************************************************************
 * 外部函数声明
 ******************************************************************************/

/******************************************************************************
 * 全局变量
 ******************************************************************************/

/******************************************************************************
 * 宏定义
 ******************************************************************************/

/******************************************************************************
 * 常量声明
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
cfg_modulate* volatile cfg_modulate::p_instance_ = nullptr;
pthread_mutex_t cfg_modulate::mutex_ = PTHREAD_MUTEX_INITIALIZER;

/*****************************************************************************
 函 数 名: cfg_modulate.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: cfg_modulate*
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
cfg_modulate* cfg_modulate::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr == p_instance_)
			p_instance_ = new cfg_modulate();
		pthread_mutex_unlock(&mutex_);
	}
	
	return p_instance_;
}

/*****************************************************************************
 函 数 名: cfg_modulate.release_instance
 功能描述  : 销毁实例
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::release_instance(void)
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
 函 数 名: cfg_modulate.cfg_modulate
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: cfg_modulate
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
cfg_modulate::cfg_modulate()
{
	PID_STRU pid;
	//debug_print_error("+++++++++++++");
	pid.goal_value = 0.0;               // 设定目标 desired value
	pid.proportion = 0.1;               // 比例常数 proportional const
	pid.integral = 0.0;                 // 积分常数 integral const
	pid.derivative = 0.4;               // 微分常数 derivative const
	pid.last_error = 0.0;               // error[-1]
	pid.prev_error = 0.0;               // error[-2]
	pid.sum_error = 0.0;                // sums of errors
	angular_velocity_pid_.set_pid(pid);

	//angular_velocity_pid_.print_pid_data();
	//debug_print_error("-------------");
}

/*****************************************************************************
 函 数 名: cfg_modulate.~cfg_modulate
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: cfg_modulate
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
cfg_modulate::~cfg_modulate()
{

}

/*****************************************************************************
 函 数 名: cfg_modulate.set_velocity_ajust
 功能描述  : 设置速度调节状态数据
 输入参数: const VELOCITY_AJUST_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_velocity_ajust(const VELOCITY_AJUST_STRU data)
{
	velocity_ajust_ = data;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_velocity_ajust
 功能描述  : 获取速度调节状态数据
 输入参数: VELOCITY_AJUST_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::get_velocity_ajust(VELOCITY_AJUST_STRU &data)
{
	data = velocity_ajust_;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_angular_velocity_flag
 功能描述  : 设置角速度调节是否使能
 输入参数: bool data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_angular_velocity_flag(bool data)
{
	velocity_ajust_.angular_flag = data;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_angular_velocity_flag
 功能描述  : 获取角速度调节是否使能
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_modulate::get_angular_velocity_flag(void)
{
	return velocity_ajust_.angular_flag;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_angular_velocity_value
 功能描述  : 设置角速度调整值
 输入参数: double value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_angular_velocity_value(double value)
{
	velocity_ajust_.angular_velocity = value;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_angular_velocity_value
 功能描述  : 获取角速度调整值
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_modulate::get_angular_velocity_value(void)
{
	return velocity_ajust_.angular_velocity;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_angular_velocity_ajust
 功能描述  : 设置角速度调整当前状态
 输入参数: bool flag     
           double value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_angular_velocity_ajust(bool flag, double value)
{
	velocity_ajust_.angular_flag = flag;
	velocity_ajust_.angular_velocity = value;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_angular_velocity_ajust
 功能描述  : 获取角速度调整当前状态
 输入参数: double &value  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_modulate::get_angular_velocity_ajust(double &value)
{
	bool flag = false;

	flag = velocity_ajust_.angular_flag;
	if (true == flag)
	{
		value = velocity_ajust_.angular_velocity;
	}
	
	return flag;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_linear_velocity_ajust
 功能描述  : 设置线速度调整当前状态
 输入参数: bool flag     
           double value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_linear_velocity_ajust(bool flag, double value)
{
	velocity_ajust_.linear_flag = flag;
	velocity_ajust_.linear_velocity = value;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_linear_velocity_ajust
 功能描述  : 获取线速度调整当前状态
 输入参数: double &value  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_modulate::get_linear_velocity_ajust(double &value)
{
	bool flag = false;

	flag = velocity_ajust_.linear_flag;
	if (true == flag)
	{
		value = velocity_ajust_.linear_velocity;
	}
	
	return flag;
}

/*****************************************************************************
 函 数 名: cfg_modulate.disable_angular_velocity_ajust
 功能描述  : 不启用速度调整功能
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月18日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::disable_angular_velocity_ajust(void)
{
	set_angular_velocity_ajust(false, 0.0);
}

/*****************************************************************************
 函 数 名: cfg_modulate.endble_angular_velocity_ajust
 功能描述  : 启用速度调整功能
 输入参数: double velocity  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月18日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::endble_angular_velocity_ajust(double velocity)
{
	set_angular_velocity_ajust(true, velocity);
}

/*****************************************************************************
 函 数 名: cfg_modulate.disable_linear_velocity_ajust
 功能描述  : 禁用线速度调整功能
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::disable_linear_velocity_ajust(void)
{
	set_linear_velocity_ajust(false, 0.0);
}

/*****************************************************************************
 函 数 名: cfg_modulate.endble_linear_velocity_ajust
 功能描述  : 启用线速度调整功能
 输入参数: double velocity  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::endble_linear_velocity_ajust(double velocity)
{
	set_linear_velocity_ajust(true, velocity);
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_distance
 功能描述  : 获取两点之间的距离
 输入参数: double x1  
           double y1  
           double x2  
           double y2  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_modulate::get_distance(double x1, double y1, double x2, double y2)
{
	double x = 0.0;
	double y = 0.0;
	double hypotenuse = 0.0;
	
	x = fabs(x2 - x1);
	y = fabs(y2 - y1);
	
	if (0.0 == x)
	{
		hypotenuse = y;
	}
	else if (0.0 == y)
	{
		hypotenuse = x;
	}
	else
	{
		hypotenuse = sqrt(pow(x, 2) + pow(y, 2));            //斜边长度
	}

	return hypotenuse;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_traight_line_moving_flag
 功能描述  : 设置当前是否处于直行状态
 输入参数: const bool flag  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_traight_line_moving_flag(const bool flag)
{
	straight_line_moving_.flag = flag;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_traight_line_moving_flag
 功能描述  : 获取当前是否处于直行状态
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_modulate::get_traight_line_moving_flag(void)
{
	return straight_line_moving_.flag;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_traight_line_moving_start_pos
 功能描述  : 设置直行的起始方向位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_traight_line_moving_start_pos(const POSE_STRU pos)
{
	straight_line_moving_.start_pos = pos;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_traight_line_moving_start_pos
 功能描述  : 获取直行的起始方向位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::get_traight_line_moving_start_pos(POSE_STRU &pos)
{
	pos = straight_line_moving_.start_pos;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_traight_line_moving_target_pos
 功能描述  : 设置直行方向目标位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_traight_line_moving_target_pos(const POSE_STRU pos)
{
	straight_line_moving_.target_pos = pos;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_traight_line_moving_target_pos
 功能描述  : 获取直行方向目标位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::get_traight_line_moving_target_pos(POSE_STRU &pos)
{
	pos = straight_line_moving_.target_pos;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_traight_line_moving_current_pos
 功能描述  : 设置直行当前的位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_traight_line_moving_current_pos(const POSE_STRU pos)
{
	straight_line_moving_.current_pos = pos;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_traight_line_moving_current_pos
 功能描述  : 获取直行当前的位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::get_traight_line_moving_current_pos(POSE_STRU &pos)
{
	pos = straight_line_moving_.current_pos;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_traight_line_moving_data
 功能描述  : 设置直行数据
 输入参数: const STRAIGHT_LINE_MOVING_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_traight_line_moving_data(const STRAIGHT_LINE_MOVING_STRU data)
{
	straight_line_moving_ = data;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_traight_line_moving_data
 功能描述  : 获取直行数据
 输入参数: STRAIGHT_LINE_MOVING_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::get_traight_line_moving_data(STRAIGHT_LINE_MOVING_STRU &data)
{
	data = straight_line_moving_;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_traight_line_moving_direction_angle
 功能描述  : 获取直线行驶的运行方向角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_modulate::get_traight_line_moving_direction_angle(void)
{
	return straight_line_moving_.start_pos.angle;
}

/*****************************************************************************
 函 数 名: cfg_modulate.clear_traight_line_moving_data
 功能描述  : 清除直行数据
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::clear_traight_line_moving_data(void)
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

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/




