/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: cfg_mobile_robot.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年8月1日
  最近修改:
  功能描述   : 移动机器人功能类定义
  函数列表:
  修改历史:
  1.日     期: 2017年8月1日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "cfg_mobile_robot.h"

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include "tf/LinearMath/Matrix3x3.h"
#include <vector>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>

#include <signal.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"

#include "base_type.h"
#include "version.h"
#include "cfg_walk_plan.h"
#include "bll_motion_control.h"
#include "drv_sensor.h"


#include "debug_function.h"

/******************************************************************************
 * 外部变量声明
 ******************************************************************************/
using namespace std;

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
pthread_mutex_t cfg_mobile_robot::mutex;
cfg_mobile_robot* cfg_mobile_robot::p_instance_ = NULL;

/*****************************************************************************
 函 数 名: cfg_mobile_robot.cfg_mobile_robot
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: cfg_mobile_robot

 修改历史:
  1.日     期: 2017年8月1日
    作     者: Leon
    修改内容: 新生成函数

*****************************************************************************/
cfg_mobile_robot::cfg_mobile_robot()
{
	init ();
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_instance
 功能描述  : 获取实例
 输入参数  : 无
 输出参数: 无
 返 回 值: cfg_mobile_robot*
 修改历史:
  1.日     期: 2017年8月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
cfg_mobile_robot* cfg_mobile_robot::get_instance()
{
	if ( p_instance_ == NULL )
	{
		pthread_mutex_lock ( &mutex );
		if ( p_instance_ == NULL )
		{
			p_instance_ = new cfg_mobile_robot();
		}
		pthread_mutex_unlock ( &mutex );
	}

	return p_instance_;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.init
 功能描述  : 初始化
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数

*****************************************************************************/
void cfg_mobile_robot::init ( void )
{
	odometer_horizontal = 3;
	odometer_vertical = Diameter_;

	clear_monitor_angle_data();
	
	ACTION_STATUS_STRU status = {LOCAL_AREA, STOP};
	set_action_status(status);
	
	REFERENCE_DATA_STRU ref_data = {{0.0, false}};
	set_reference_data(ref_data);

	cfg_modulate *p_modulate = cfg_modulate::get_instance();
}
#if 0
/*****************************************************************************
 函 数 名: cfg_mobile_robot.straight_and_rotate_moving
 功能描述  : 行走的同时转弯
 输入参数: double line_v
           double angular_v
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月3日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::straight_and_rotate_moving(double line_v, double angular_v)
{
	bool flag = false;

	flag = get_adjust_velocity();
	if (true == flag)
	{
		get_velocity(line_v, angular_v);
		set_adjust_velocity(false);
	}
	set_velocity(line_v, angular_v);

	drv_sensor* p_instance = drv_sensor::get_instance();
	p_instance->set_run_velocity(line_v, angular_v);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.straight_moving
 功能描述  : 以指定速度直线行走
 输入参数: double velocity
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月3日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::straight_moving( double velocity )
{
	double rad = 0.0;
	bool linear_flag = false;
	bool angular_flag = false;
//	static double last_rad = 0.0;
//	static double last_linear = 0.0;

	#ifdef ENABLE_MOUDLATE
	angular_flag = get_angular_velocity_ajust(rad);
	linear_flag = get_linear_velocity_ajust(velocity);
	#endif /* ENABLE_MOUDLATE */
	
//	if ((last_rad != rad ) && (true == angular_flag))
//	{
//		last_rad = rad;
//		debug_print_fatal("==============================angular_flag=%d, rad=%lf", angular_flag, rad);
//	}
//	
//	if ((last_linear != velocity ) && (true == linear_flag))
//	{
//		last_linear = velocity;
//		debug_print_fatal("==============================linear_flag=%d, velocity=%lf", linear_flag, velocity);
//	}

	straight_and_rotate_moving ( velocity, rad );
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.rotate_moving
 功能描述  : 以指定角速度旋转
 输入参数: double rad
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月3日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::rotate_moving ( double rad )
{
	const double velocity = 0.0;
	straight_and_rotate_moving(velocity, rad);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.shut_down
 功能描述  : 关机
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::shut_down ( void )
{

}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.power_on
 功能描述  : 上电
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数

*****************************************************************************/
void cfg_mobile_robot::power_on ( void )
{

}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.stop
 功能描述  : 停止
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::stop ( void )
{
	straight_and_rotate_moving ( 0, 0 );
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.go_forward
 功能描述  : 向前执行
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::go_forward ( void )
{
	straight_moving ( Linear_velocity_ );
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.go_back
 功能描述  : 向后执行
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::go_back ( void )
{
	straight_moving ( -Linear_velocity_ );
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.turn_left
 功能描述  : 左转弯
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::turn_left ( void )
{
	rotate_moving( Angular_velocity_ );
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.turn_right
 功能描述  : 右转弯
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::turn_right ( void )
{
	rotate_moving( -Angular_velocity_ );
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.pivot
 功能描述  : 向后旋转180度
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::pivot ( void )
{
	rotate_moving( Angular_velocity_ );
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.turn_back_clockwise
 功能描述  : 顺时针返回
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::turn_back_clockwise ( void )
{
	straight_and_rotate_moving(Linear_velocity_clockwise_, -Angular_velocity_clockwise_);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.turn_back_anticlockwise
 功能描述  : 逆时针返回
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::turn_back_anticlockwise ( void )
{
	straight_and_rotate_moving(Linear_velocity_clockwise_, Angular_velocity_clockwise_);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.turn_right_angle_clockwise
 功能描述  : 原地顺时针旋转一个直角
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::turn_right_angle_clockwise(void)
{
	rotate_moving( -Angular_velocity_clockwise_ );
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.turn_right_angle_anticlockwise
 功能描述  : 原地逆时针旋转一个直角
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::turn_right_angle_anticlockwise(void)
{
	rotate_moving( Angular_velocity_clockwise_ );
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.edge_ways
 功能描述  : 沿边运动
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::edge_ways ( void )
{

}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.auto_dock
 功能描述  : 自动返程
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::auto_dock ( void )
{

}
#endif 
/*****************************************************************************
 函 数 名: cfg_mobile_robot.local_cover_movement
 功能描述  : 区域覆盖行驶
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月31日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::local_cover_movement(void)
{
	do_local_move();
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_action_status
 功能描述  : 设置当前的行为模式状态
 输入参数: const ACTION_STATUS_STRU status
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_action_status ( const ACTION_STATUS_STRU status )
{
	action_ = status;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_action_status
 功能描述  : 获取当前的行为模式状态
 输入参数: ACTION_STATUS_STRU &status
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_action_status ( ACTION_STATUS_STRU& status )
{
	status = action_;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_curr_action
 功能描述  : 设置当前运行状态
 输入参数: ACTION_STATUS_ENUM action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_curr_action(ACTION_STATUS_ENUM action)
{
	ACTION_STATUS_STRU status;
	
	get_action_status(status);
	status.curr_action = action;
	set_action_status(status);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_curr_action
 功能描述  : 获取当前运行状态
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_curr_action(ACTION_STATUS_ENUM &action)
{
	ACTION_STATUS_STRU status;
	
	get_action_status(status);
	action = status.curr_action;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_last_action
 功能描述  : 设置上次运行状态
 输入参数: ACTION_STATUS_ENUM action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_last_action(ACTION_STATUS_ENUM action)
{
	ACTION_STATUS_STRU status;
	
	get_action_status(status);
	status.last_action = action;
	set_action_status(status);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_last_action
 功能描述  : 获取上次运行状态
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_last_action(ACTION_STATUS_ENUM &action)
{
	ACTION_STATUS_STRU status;
	
	get_action_status(status);
	action = status.last_action;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.save_running_status
 功能描述  : 保存运行状态数据
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::save_running_status(void)
{
	ACTION_STATUS_ENUM action;

	get_curr_action(action);
	set_last_action(action);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.recover_running_status
 功能描述  : 恢复运行状态数据
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::recover_running_status(void)
{
	ACTION_STATUS_ENUM action;

	get_last_action(action);
	set_curr_action(action);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_reference_data
 功能描述  : 设置参照数据
 输入参数: const REFERENCE_DATA_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月11日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_reference_data(const REFERENCE_DATA_STRU &data)
{
	ref_data_ = data;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_reference_data
 功能描述  : 获取参照数据
 输入参数: REFERENCE_DATA_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月11日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_reference_data(REFERENCE_DATA_STRU &data)
{
	data = ref_data_;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.check_reference_data_valid
 功能描述  : 检查参考数据是否已经有效保存
 输入参数: 无
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_mobile_robot::check_reference_data_valid(void)
{
	bool ret = false;
	REFERENCE_DATA_STRU ref_data;
	
	get_reference_data(ref_data);
	
	ret = ref_data.fst_dir.valid;

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_reference_data_forward_angle
 功能描述  : 设置参考数据正向的角度值
 输入参数: double data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_reference_data_forward_angle(double data)
{
	ref_data_.fst_dir.forward = data;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_reference_data_forward_angle
 功能描述  : 获取参考数据正向的角度值
 输入参数:  
 输出参数: double &data 
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_mobile_robot::get_reference_data_forward_angle(double &data)
{
	bool ret = false;
	REFERENCE_DATA_STRU ref_data;
	
	get_reference_data(ref_data);
	
	ret = ref_data.fst_dir.valid;
	if ( true == ret)
	{
		data = ref_data.fst_dir.forward;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_reference_data_reverse_angle
 功能描述  : 设置参考数据反向的角度值
 输入参数: double data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_reference_data_reverse_angle(double data)
{
	ref_data_.fst_dir.reverse = data;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_reference_data_reverse_angle
 功能描述  : 获取参考数据反向的角度值
 输入参数: double &data  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_mobile_robot::get_reference_data_reverse_angle(double &data)
{
	bool ret = false;
	REFERENCE_DATA_STRU ref_data;
	
	get_reference_data(ref_data);
	
	ret = ref_data.fst_dir.valid;
	if ( true == ret)
	{
		data = ref_data.fst_dir.reverse;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_reference_data_inversion
 功能描述  : 设置转向颠倒标志
 输入参数: bool data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月30日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_reference_data_inversion(bool data)
{
	ref_data_.fst_dir.inversion = data;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_reference_data_inversion
 功能描述  : 获取转向颠倒标志
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年10月30日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_mobile_robot::get_reference_data_inversion(void)
{
	return ref_data_.fst_dir.inversion;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_curr_rotate_direction
 功能描述  : 获取当前旋转方向
 输入参数: void  
 输出参数: 无
 返 回 值: ROTATE_DIRECTION_ENUM
 
 修改历史:
  1.日     期: 2017年11月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
ROTATE_DIRECTION_ENUM cfg_mobile_robot::get_curr_rotate_direction(void)
{
	ACTION_STATUS_ENUM action;
	ROTATE_DIRECTION_ENUM rotate;
	
	get_curr_action(action);
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

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_traight_line_moving
 功能描述  : 设置直线运行
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_mobile_robot::set_traight_line_moving(void)
{
	change_curr_action(GO_FORWARD);

	update_traight_line_moving_data();
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.test_is_traight_line_moving
 功能描述  : 检测当前是否为直行状态
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_mobile_robot::test_is_traight_line_moving(void)
{
	bool flag = false;
	ACTION_STATUS_ENUM action;
	
	get_curr_action(action);
	if (GO_FORWARD == action)
	{
		flag = true;
	}

	return flag;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_rotate_action_type
 功能描述  : 获取旋转方向
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_rotate_action_type(ACTION_STATUS_ENUM &action)
{
	bool area_part_is_left = false;
	bool direction_is_forward = false;
	const ACTION_STATUS_ENUM clockwise = TURN_RIGHT_ANGLE_CLOCKWISE;
	const ACTION_STATUS_ENUM anticlockwise = TURN_RIGHT_ANGLE_ANTICLOCKWISE;
	area_part_is_left = test_local_move_area_part_is_left();
	direction_is_forward = test_local_move_direction_is_forward();

	//debug_print_warnning("area_part_is_left = %d; direction_is_forward = %d;",area_part_is_left, direction_is_forward);
	if(true == area_part_is_left)
	{
		if(true == direction_is_forward)
		{
			action = clockwise;
			//debug_print_warnning("action = TURN_RIGHT_ANGLE_CLOCKWISE;");
		}
		else
		{
			action = anticlockwise;
			//debug_print_warnning("action = TURN_RIGHT_ANGLE_ANTICLOCKWISE;");
		}
	}
	else
	{
		if(true == direction_is_forward)
		{
			action = anticlockwise;
			//debug_print_warnning("action = TURN_RIGHT_ANGLE_ANTICLOCKWISE;");
		}
		else
		{
			action = clockwise;
			//debug_print_warnning("action = TURN_RIGHT_ANGLE_CLOCKWISE;");
		}
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_driving_direction_right_angle
 功能描述  : 获取相对与运行方向的垂直角度
 输入参数: void  
 输出参数: 无
 返 回 值: 垂直角度
 行驶方向的垂直方向，且指向初始参考线一方
 修改历史:
  1.日     期: 2017年11月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_driving_direction_right_angle(void)
{
	double angle = 0.0;
	double right_angle = 0.0;
	bool area_part_is_left = false;
	bool direction_is_forward = false;
	ROTATE_DIRECTION_ENUM direction;

	area_part_is_left = test_local_move_area_part_is_left();
	direction_is_forward = test_local_move_direction_is_forward();
	if(true == area_part_is_left)
	{
		if (true == direction_is_forward)
		{
			get_reference_data_forward_angle(angle);
			direction = ANTICLOCKWISE;
		}
		else
		{
			get_reference_data_reverse_angle(angle);
			direction = CLOCKWISE;
		}
	}
	else
	{
		if (true == direction_is_forward)
		{
			get_reference_data_forward_angle(angle);
			direction = CLOCKWISE;
		}
		else
		{
			get_reference_data_reverse_angle(angle);
			direction = ANTICLOCKWISE;
		}
	}

	right_angle = get_right_angle(angle, direction);

	return right_angle;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_turn_back_action_type
 功能描述  : 获取返回运行模式类型
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_turn_back_action_type(ACTION_STATUS_ENUM &action)
{
	bool area_part_is_left = false;
	bool direction_is_forward = false;

	area_part_is_left = test_local_move_area_part_is_left();
	direction_is_forward = test_local_move_direction_is_forward();
	if(true == area_part_is_left)
	{
		if(true == direction_is_forward)
		{
			action = TURN_BACK_ANTICLOCKWISE;
			//debug_print_info(" area_part_is_left: action = TURN_BACK_ANTICLOCKWISE;");
		}
		else
		{
			action = TURN_BACK_CLOCKWISE;
			//debug_print_info(" area_part_is_left: action = TURN_BACK_CLOCKWISE;");
		}
	}
	else
	{
		if(true == direction_is_forward)
		{
			action = TURN_BACK_CLOCKWISE;
			//debug_print_info(" area_part_is_right: action = TURN_BACK_CLOCKWISE;");
		}
		else
		{
			action = TURN_BACK_ANTICLOCKWISE;
			//debug_print_info(" area_part_is_right: action = TURN_BACK_ANTICLOCKWISE;");
		}
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_turn_right_angle_action_type
 功能描述  : 获取旋转到垂直方向的运动模式
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_turn_right_angle_action_type(ACTION_STATUS_ENUM &action)
{
	get_rotate_action_type(action);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_reference_angle
 功能描述  : 获取直行参考角
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月16日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_reference_angle(void)
{
	double angle = 0.0;
	bool direction_is_forward = false;

	direction_is_forward = test_local_move_direction_is_forward();
	if (true == direction_is_forward)
	{
		get_reference_data_forward_angle(angle);
		//debug_print_info("---------------------->>>>>>(angle=%lf)", angle);
	}
	else
	{
		get_reference_data_reverse_angle(angle);
		//debug_print_info("<<<<<<<<---------------------(angle=%lf)", angle);
	}

	return angle;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_turnt_to_reference_deriction_angle
 功能描述  : 获取转向参考方向的角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_turnt_to_reference_deriction_angle(void)
{
	double angle = 0.0;
	bool direction_is_forward = false;

	direction_is_forward = test_local_move_direction_is_forward();
	if (true == direction_is_forward)
	{
		get_reference_data_reverse_angle(angle);
	}
	else
	{
		get_reference_data_forward_angle(angle);
	}

	return angle;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_turnt_to_reference_deriction_action_type
 功能描述  : 获取转向参考方向的动作模式
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_turnt_to_reference_deriction_action_type(ACTION_STATUS_ENUM &action)
{
	get_rotate_action_type(action);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.save_first_line_refer_direction
 功能描述  : 保存第一行参考数据
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::save_first_line_refer_direction(void)
{
	double temp = 0.0;
	double forward = 0.0;
	double reverse = 0.0;
	const double inversion = 180.0;
	POSE_STRU pos;
	REFERENCE_DATA_STRU ref_data;
	
	get_reference_data(ref_data);
	if ( false == ref_data.fst_dir.valid)
	{
		get_current_position(pos);
		forward = format_angle(pos.angle);
		temp = forward + inversion;
		reverse = format_angle(temp);
		
		ref_data.fst_dir.forward = forward;
		ref_data.fst_dir.reverse = reverse;
		ref_data.fst_dir.inversion = false;
		ref_data.fst_dir.valid = true;
		set_reference_data(ref_data);
		
		std::cout<<std::endl;
		debug_print_info("current pos.angle=%lf; forward=%lf; reverse =%lf",pos.angle, forward, reverse);
		updata_district_area(pos);
		std::cout<<std::endl;
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.save_current_positions
 功能描述  : 保存当前位置状态
 输入参数: double x      
           double y      
           double theta  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::save_current_positions (double x, double y, double theta)
{
	POSE_STRU pos;
	double real_yaw = theta;
	double real_degree = 0.0;

	real_degree = convert_radians_to_degrees(real_yaw);
	pos.point.x = x;
	pos.point.y = y;
	pos.angle = real_degree;
	set_current_position(pos);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.format_angle
 功能描述  : 格式化角度在合理的范围
 输入参数: double angle  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年8月11日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
//double cfg_mobile_robot::format_angle(double angle)
//{
//	//把角度规划到-180至180之间
//	const double pi_angle = 180.0;
//	if (angle > pi_angle)
//		angle = angle - 2*pi_angle;
//	else if (angle < (-1.0*pi_angle))
//		angle = angle + 2*pi_angle;
//	
//	return angle;
//}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.convert_degrees_to_radians
 功能描述  : 角度转化为弧度
 输入参数: double degrees  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月17日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
//double cfg_mobile_robot::convert_degrees_to_radians(double degrees)
//{
//	double radians = (M_PI/180.0)*degrees;
//	return radians;
//}
//
///*****************************************************************************
// 函 数 名: cfg_mobile_robot.convert_radians_to_degrees
// 功能描述  : 弧度转化为角度
// 输入参数: double radians  
// 输出参数: 无
// 返 回 值: double
// 
// 修改历史:
//  1.日     期: 2017年11月17日
//    作     者: Leon
//    修改内容: 新生成函数
//*****************************************************************************/
//double cfg_mobile_robot::convert_radians_to_degrees(double radians)
//{
//	double degrees = (180.0/M_PI)*radians;
//	return degrees;
//}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.convert_to_acute_angle
 功能描述  : 转化为锐角
 输入参数: double angle  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月16日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
//double cfg_mobile_robot::convert_to_acute_angle(double angle)
//{
//	double ret = 0.0;
//	double temp = 0.0;
//	
//	temp = fabs(angle);
//	temp = fmod(angle, 360.0);
//	if (temp <= 90.0)
//	{
//		ret = temp;
//	}
//	else if ((90.0 < temp) && (temp <= 180.0))
//	{
//		ret = 180.0 - temp;
//	}
//	else if ((180.0 < temp) && (temp <= 270.0))
//	{
//		ret = temp - 180.0;
//	}
//	else if ((270.0 < temp) && (temp <= 360.0))
//	{
//		ret = temp - 270.0;
//	}
//	else if(temp > 360.0)
//	{
//		ret = fmod( temp, 360.0 );
//	}
//	
//	return ret;
//}


/*****************************************************************************
 函 数 名: cfg_mobile_robot.test_differences
 功能描述  : 检测偏差是不是在误差精度范围内
 输入参数: double value          检测值
           double reference  参考值
           double precision  精度值
 输出参数: 偏差在精度范围内则返回真否则返回假
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年9月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
//bool cfg_mobile_robot::test_differences(double value, double reference, double precision)
//{
//	bool ret = false;
//	double temp = 0.0;
//	
//	temp = value - reference;
//	if (fabs(temp) < precision)
//	{
//		ret = true;
//	}
//	
//	return ret;
//}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.update_goal_positions
 功能描述  : 更新目标位置
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月7日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::update_goal_positions (void)
{

}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.update_ultrasonic_sensor_data
 功能描述  : 更新超声波传感器检测到数据
 输入参数: double value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月12日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::update_ultrasonic_sensor_data (double value)
{
	ULTRASONIC_SENSOR_STRU data;
	
	get_ultrasonic_sensor_state(data);
	if ( false == data.enable )
	{
		if ( value > 0.0 )
		{
			data.enable = true;
			debug_print_warnning("value=%lf, data.enable=%d, min=%lf, max=%lf", value, data.enable,data.min,data.max);
		}
	}
	else
	{
		if ((data.min >= data.max ) || (data.min < 0.0) || (data.max < 0.0))
		{
			data.enable = false;
			debug_print_warnning("value=%lf, data.enable=%d, min=%lf, max=%lf", value, data.enable,data.min,data.max);
		}
	}
	
	if (( true == data.enable ) && (value > 0.0))
	{
		data.value = value;
		set_ultrasonic_sensor_state(data);
		if (( 4.5 < value ) && (value < 500.0))//超声波检测范围[4cm ~ 5m]
		{
			update_obstatcle_safety_level();
		}
	}
	else
	{
		debug_print_warnning("value=%lf", value);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.update_wall_following_sensor_data
 功能描述  : 更新沿墙传感器数据信息状态
 输入参数: double value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::update_wall_following_sensor_data (double value)
{
	WALL_FOLLOWING_SENSOR_STRU data;
	
	get_wall_following_sensor_state(data);

	if (( true == data.enable ) && (value > 0.0))
	{
		data.value = value;
		set_wall_following_sensor_state(data);
	}
	else
	{
		//debug_print_warnning("value=%lf", value);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.upate_cliff_state
 功能描述  : 更新悬崖传感器状态
 输入参数: uint8_t id     
           uint8_t value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::upate_cliff_state(uint8_t id, uint8_t value)
{
	bool flag = false;
	bool cliff_state = false;
	CLIFF_ID_ENUM cliff_id;
	
	flag = convert_cliff_id(id, cliff_id);
	if (false == flag)
	{
		debug_print_warnning("id = %d", id);
		return;
	}

	flag = convert_cliff_state(value, cliff_state);
	if (false == flag)
	{
		debug_print_warnning("value = %d", value);
		return;
	}

	set_cliff_state(cliff_id, cliff_state);
	
	debug_print_info("cliff_id=%d, cliff_state=%d", cliff_id, cliff_state);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.upate_bumper_state
 功能描述  : 更新碰撞传感器的状态
 输入参数: uint8_t id     
           uint8_t value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::upate_bumper_state(uint8_t id, uint8_t value)
{
	bool ret = false;
	bool bumper_state = false;
	BUMPER_ID_ENUM bumper_id;

	ret = convert_bumper_id(id, bumper_id);
	if ( false == ret )
	{
		debug_print_warnning("id = %d", id);
		return;
	}
	
	ret = convert_bumper_state(value, bumper_state);
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
	
	set_bumper_state(bumper_id, bumper_state);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.upate_wheel_drop_state
 功能描述  : 更新跌落传感器状态
 输入参数: uint8_t id     
           uint8_t value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::upate_wheel_drop_state(uint8_t id, uint8_t value)
{
	bool ret = false;
	bool wheel_state = false;
	WHEEL_ID_ENUM wheel_id;

	ret = convert_wheel_drop_id(id, wheel_id);
	if ( false == ret )
	{
		debug_print_warnning("id = %d", id);
		return;
	}
	
	ret = convert_wheel_drop_state(value, wheel_state);
	if ( false == ret )
	{
		debug_print_warnning("value = %d", value);
		return;
	}

	set_wheel_drop_state(wheel_id, wheel_state);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.update_obstatcle_safety_level
 功能描述  : 更新障碍物距离安全等级
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::update_obstatcle_safety_level(void)
{
	SAFETY_LEVEL_ENUM level;
	static SAFETY_LEVEL_ENUM last_level;
	ULTRASONIC_SENSOR_STRU data;

	get_ultrasonic_sensor_state(data);
	if ( true == data.enable )
	{
		if ( data.value > data.max )
		{
			level = SAFETY_LEVEL_SAFE;
			//debug_print_info("value=%lf, min=%lf, max=%lf,level=%d SAFETY_LEVEL_SAFE", data.value, data.min, data.max, level);
		}
		else if (( data.value > data.min ) && ( data.value < data.max ))
		{
			level = SAFETY_LEVEL_WARN;
			//debug_print_warnning("value=%lf, min=%lf, max=%lf,level=%d SAFETY_LEVEL_SAFE", data.value, data.min, data.max, level);
		}
		else if ( data.value < data.min )
		{
			level = SAFETY_LEVEL_FATAL;
			//debug_print_fatal("value=%lf, min=%lf, max=%lf,level=%d SAFETY_LEVEL_FATAL", data.value, data.min, data.max, level);
		}
		
		if ( last_level != level )
		{
			last_level = level;

			std::cout<<std::endl;
			if ( level == SAFETY_LEVEL_SAFE )
			{
				debug_print_info("SAFETY_LEVEL_SAFE=%d; distance=%lf;", data.level, data.value);
			}
			else if (level == SAFETY_LEVEL_WARN)
			{
				debug_print_warnning("SAFETY_LEVEL_WARN=%d; distance=%lf;", data.level, data.value);
			}
			else if ( level == SAFETY_LEVEL_FATAL )
			{
				debug_print_fatal("SAFETY_LEVEL_FATAL=%d; distance=%lf;", data.level, data.value);
			}
			std::cout<<std::endl;
		}
		
		data.level = level;
		set_ultrasonic_sensor_state(data);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.sensors_deal
 功能描述  : 检测传感器处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月12日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::sensors_deal(void)
{
	ultrasonic_sensor_respond_deal();
	bumper_sensor_respond_deal();
	wheel_drop_sensor_respond_deal();
	cliff_sensor_respond_deal();
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_curr_pose_angle
 功能描述  : 获取当前角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年10月24日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_curr_pose_angle(void)
{
	double ret = 0.0;
	POSE_STRU pos;
	
	get_current_position(pos);
	ret = pos.angle;

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_curr_pose_reverse_angle
 功能描述  : 获取当前位置状态下的反向角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_curr_pose_reverse_angle(void)
{
	double curr_angle = 0.0;
	double reverse_angle = 0.0;
	
	curr_angle = get_curr_pose_angle();
	reverse_angle = get_reverse_angle(curr_angle);
	
	return reverse_angle;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_curr_pose_right_angle
 功能描述  : 获取当前角度的偏移一个直角的角度
 输入参数: ROTATE_DIRECTION_ENUM direction  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_curr_pose_right_angle(ROTATE_DIRECTION_ENUM direction)
{
	double angle = 0.0;
	double ret = 0.0;
	
	angle = get_curr_pose_angle();
	if (CLOCKWISE == direction)
	{
		ret = get_right_angle_clockwise(angle);
		debug_print_warnning("ret = get_right_angle_clockwise(%lf)=%lf", angle, ret);
	}
	else
	{
		ret = get_right_angle_anticlockwise(angle);
		debug_print_warnning("ret = get_right_angle_anticlockwise(%lf)=%lf", angle, ret);
	}

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.test_rotate_is_over_clockwise
 功能描述  : 测试顺时针旋转是否已经超过预定的检测值
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_mobile_robot::test_rotate_is_over_clockwise(void)
{
	bool over_flag = false;
	double current = 0.0;
	double target = 0.0;
	
	current = get_curr_pose_angle();
	target = get_monitor_angle_respond_goal();
	over_flag = test_angle_is_over_clockwise(current, target);

	return over_flag;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.test_rotate_is_over_anticlockwise
 功能描述  : 测试逆时针旋转是否已经超过预定的检测值
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_mobile_robot::test_rotate_is_over_anticlockwise(void)
{
	bool over_flag = false;
	double current = 0.0;
	double target = 0.0;

	current = get_curr_pose_angle();
	target = get_monitor_angle_respond_goal();
	over_flag = test_angle_is_over_anticlockwise(current, target);
	
	return over_flag;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.test_angle_is_over
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
bool cfg_mobile_robot::test_angle_is_over(double current, double target, double &offset)
{
	double diff = 0.0;
	bool over_flag = false;
	ROTATE_DIRECTION_ENUM rotate_dir;
	
	diff = get_angle_differences(current, target);
	
	rotate_dir = get_curr_rotate_direction();
	if (CLOCKWISE == rotate_dir)
	{
		over_flag = test_angle_is_over_clockwise(current, target);
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
		over_flag = test_angle_is_over_anticlockwise(current, target);
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
 函 数 名: cfg_mobile_robot.get_curr_angle_difference_respond
 功能描述  : 获取当前角度与响应角度的偏差
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_curr_angle_difference_respond(void)
{
	double diff = 0.0;
	double current = 0.0;
	double target = 0.0;
	bool over_flag = false;
	
	current = get_curr_pose_angle();
	target = get_monitor_angle_respond_goal();
	over_flag = test_angle_is_over(current, target, diff);
	
	return diff;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_monitor_angle_data
 功能描述  : 设置监测角度状态
 输入参数: const ANGLE_MONITOR_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月8日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_monitor_angle_data(const ANGLE_MONITOR_STRU data)
{
	angle_monitor_ = data;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_monitor_angle_data
 功能描述  : 获取监测角度状态
 输入参数: ANGLE_MONITOR_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月8日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_monitor_angle_data(ANGLE_MONITOR_STRU &data)
{
	data = angle_monitor_;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.clear_monitor_angle_data
 功能描述  : 清除角度监测状态
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::clear_monitor_angle_data(void)
{
	ANGLE_MONITOR_STRU data;

	data.angle = 0.0;
	data.stage = MONITOR_SLEEP;
	set_monitor_angle_data(data);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_monitor_angle_stage
 功能描述  : 设置监测角度的阶段状态
 输入参数: const MONITOR_ENUM stage  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_monitor_angle_stage(const MONITOR_ENUM stage)
{
	angle_monitor_.stage = stage;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_monitor_angle_stage
 功能描述  : 获取监测角度的阶段状态
 输入参数: MONITOR_ENUM &stage  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_monitor_angle_stage(MONITOR_ENUM &stage)
{
	stage = angle_monitor_.stage;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_monitor_angle_respond_goal
 功能描述  : 设置目标响应角度
 输入参数: const double angle  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_monitor_angle_respond_goal(const double angle)
{
	angle_monitor_.angle = angle;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_monitor_angle_respond_goal
 功能描述  : 获取目标响应角度
 输入参数: 无
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_monitor_angle_respond_goal(void)
{
	return angle_monitor_.angle;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_monitor_angle_respond_func
 功能描述  : 设置角度到达以后的响应功能函数
 输入参数: PF_FUNC pf  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_monitor_angle_respond_func(void(cfg_mobile_robot::*pf)(void))
{
	pf_monitor_ = pf;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_monitor_angle_respond_func
 功能描述  : 获取角度到达以后的响应功能函数指针
 输入参数: PF_FUNC &pf  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_monitor_angle_respond_func(void(cfg_mobile_robot::*pf)(void))
{
	pf = pf_monitor_;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_monitor_angle_respond_goal_and_func
 功能描述  : 设置监测目标角度和到达目标角度以后的响应功能函数
 输入参数: double angle  
           PF_FUNC pf    
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_monitor_angle_respond_goal_and_func(double angle, void(cfg_mobile_robot::*pf)(void))
{
	set_monitor_angle_respond_goal(angle);
	set_monitor_angle_respond_func(pf);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_monitor_angle_turn_back_angle
 功能描述  : 获取拐弯返回到达响应角度
 输入参数: void
 输出参数: 无
 返 回 值: double 
 
 修改历史:
  1.日     期: 2017年8月11日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_monitor_angle_turn_back_angle(void)
{
	MONITOR_ENUM stage;
	double respond_angle = 0.0;
	double forward_angle = 0.0;
	double reverse_angle = 0.0;
	bool inversion = false;
	
	get_monitor_angle_stage(stage);
	if (MONITOR_SLEEP == stage)
	{
		if ( true == check_reference_data_valid())
		{
			get_reference_data_forward_angle(forward_angle);
			get_reference_data_reverse_angle(reverse_angle);
			inversion = get_reference_data_inversion();
			if ( false == inversion )
			{
				if(true == test_local_move_direction_is_forward())
				{
					respond_angle = reverse_angle;
				}
				else
				{
					respond_angle = forward_angle;
				}
			}
			else
			{
				if(true == test_local_move_direction_is_forward())
				{
					respond_angle = forward_angle;
				}
				else
				{
					respond_angle = reverse_angle;
				}
			}
		}
		else
		{
			respond_angle = get_curr_pose_reverse_angle();
			debug_print_warnning("get_curr_pose_reverse_angle()=%lf", respond_angle);
		}
	}
	
	return respond_angle;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_turn_back_respond
 功能描述  : 响应检测旋转角度的处理功能
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_turn_back_respond(void)
{
	POSE_STRU pos;
	ACTION_STATUS_ENUM action;
	get_curr_action(action);
	if ((action == TURN_BACK_ANTICLOCKWISE) || (action == TURN_BACK_CLOCKWISE))
	{
		clear_traight_line_moving_data();
		
		set_traight_line_moving();
		
		switch_local_move_direction();
		
		#ifdef ENABLE_MOUDLATE
		set_current_position_to_refer_start_pose();
		
		disable_linear_velocity_ajust();
		get_straight_moving_refer_target_pose(pos);
		set_traight_line_moving_target_pos(pos);
		#endif /* ENABLE_MOUDLATE */
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_set_turn_back_deal
 功能描述  : 设置检测角度旋转返回处理
 输入参数: void
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_set_turn_back_deal(void)
{
	double angle = 0.0;
	MONITOR_ENUM stage;
	void(cfg_mobile_robot::*pf)(void) = NULL;
	
	get_monitor_angle_stage(stage);
	if (MONITOR_SLEEP == stage)
	{
		angle = get_monitor_angle_turn_back_angle();
		pf = &cfg_mobile_robot::monitor_angle_turn_back_respond;
		set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_pivot_respond
 功能描述  : 原地向后旋转到达后的响应
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月7日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_pivot_respond(void)
{
	POSE_STRU pos;
	
	clear_traight_line_moving_data();

	get_refer_line_start_point_pose(pos);
	set_traight_line_moving_target_pos(pos);
	disable_linear_velocity_ajust();
	set_traight_line_moving();
	
	set_local_move_state(LOCAL_MOVE_FST_LINE_START);
	switch_local_move_direction();
	
	#ifdef ENABLE_MOUDLATE
	set_current_position_to_refer_start_pose();
	#endif /* ENABLE_MOUDLATE */
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_set_pivot_deal
 功能描述  : 设置原地旋转反向的处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_set_pivot_deal(void)
{
	double angle = 0.0;
	MONITOR_ENUM stage;
	void(cfg_mobile_robot::*pf)(void) = NULL;
	
	get_monitor_angle_stage(stage);
	if (MONITOR_SLEEP == stage)
	{
		POSE_STRU original_pose;
		get_local_move_original_pose(original_pose);
		change_curr_action (PIVOT);
		angle = get_reverse_angle(original_pose.angle);
		pf = &cfg_mobile_robot::monitor_angle_pivot_respond;
		set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_turn_to_refer_line_respond
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
void cfg_mobile_robot::monitor_angle_turn_to_refer_line_respond(void)
{
	POSE_STRU pos;
	
	clear_traight_line_moving_data();

	update_refer_line_traight_line_moving_target_pos();
	
	set_traight_line_moving();
	
	set_local_move_state(LOCAL_MOVE_RETURN_REFER_LINE);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_turn_to_refer_line_deal
 功能描述  : 转向参考线最近方向处理
 输入参数: void
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_turn_to_refer_line_deal(void)
{
	double angle = 0.0;
	MONITOR_ENUM stage;
	ACTION_STATUS_ENUM action;
	void(cfg_mobile_robot::*pf)(void) = NULL;
	
	get_monitor_angle_stage(stage);
	if (MONITOR_SLEEP == stage)
	{
		disable_linear_velocity_ajust();
		disable_angular_velocity_ajust();
		
		get_turn_right_angle_action_type(action);
		change_curr_action (action);
		
		angle = get_driving_direction_right_angle();
		pf = &cfg_mobile_robot::monitor_angle_turn_to_refer_line_respond;
		set_monitor_angle_rotate_call_back(angle, pf);
		debug_print_warnning("||||||||||||||||||||| angle=%lf; action=%d", angle, action);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_turn_to_center_respond
 功能描述  : 回到第二半区的拐直角的处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_turn_to_center_respond(void)
{
	clear_traight_line_moving_data();
	disable_linear_velocity_ajust();
	set_traight_line_moving();
	
	switch_local_move_area_part();
	switch_local_move_direction();
	
	set_current_position_to_refer_start_pose();
	update_refer_line_traight_line_moving_target_pos();
	set_local_move_state(LOCAL_MOVE_SEC_HALF);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_turn_to_center_deal
 功能描述  : 转向中心点处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_turn_to_center_deal(void)
{
	double angle = 0.0;
	MONITOR_ENUM stage;
	ACTION_STATUS_ENUM action;
	void(cfg_mobile_robot::*pf)(void) = NULL;
	
	get_monitor_angle_stage(stage);
	if (MONITOR_SLEEP == stage)
	{
		disable_angular_velocity_ajust();
		
		angle = get_turnt_to_reference_deriction_angle();
		get_turnt_to_reference_deriction_action_type(action);
		debug_print_info("action =%d   angle=%lf", action, angle);
		change_curr_action(action);
		pf = &cfg_mobile_robot::monitor_angle_turn_to_center_respond;
		set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_turnt_to_original_pose_angle
 功能描述  : 获取转向原始点的目标角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_turnt_to_original_pose_angle(void)
{
	double angle = 0.0;
	bool direction_is_forward = false;
	POSE_STRU current_pos;
	POSE_STRU original_pos;
	get_current_position(current_pos);
	get_local_move_original_pose(original_pos);

	angle = get_angle(current_pos.point.x, current_pos.point.y, original_pos.point.x, original_pos.point.y);

	//debug_print_warnning("(%lf, %lf) -> (%lf, %lf) angle=%lf", current_pos.point.x, current_pos.point.y, original_pos.point.x, original_pos.point.y, angle);
	return angle;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_turnt_to_original_pose_action_type
 功能描述  : 获取转向原始点的运动类型
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_turnt_to_original_pose_action_type(ACTION_STATUS_ENUM &action)
{
	get_rotate_action_type(action);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_turn_to_original_pose_respond
 功能描述  : 转向原始点的角度到达以后的响应处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_turn_to_original_pose_respond(void)
{
	POSE_STRU pos;
	
	clear_traight_line_moving_data();
	disable_linear_velocity_ajust();
	get_local_move_original_pose(pos);
	set_traight_line_moving_target_pos(pos);
	
	set_traight_line_moving();
	set_local_move_state(LOCAL_MOVE_RETURN_CENTER_POS);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_turn_to_original_pose_deal
 功能描述  : 转向原始出发点处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_turn_to_original_pose_deal(void)
{
	double angle = 0.0;
	MONITOR_ENUM stage;
	ACTION_STATUS_ENUM action;
	void(cfg_mobile_robot::*pf)(void) = NULL;
	
	get_monitor_angle_stage(stage);
	if (MONITOR_SLEEP == stage)
	{
		disable_angular_velocity_ajust();
		
		angle = get_turnt_to_original_pose_angle();
		get_turnt_to_original_pose_action_type(action);
		debug_print_info("action =%d   angle=%lf", action, angle);
		change_curr_action(action);
		pf = &cfg_mobile_robot::monitor_angle_turn_to_original_pose_respond;
		set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_turnt_to_original_direction_angle
 功能描述  : 获取原始方向角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_turnt_to_original_direction_angle(void)
{
	double angle = 0.0;
	get_reference_data_forward_angle(angle);
	return angle;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_turnt_to_original_direction_action_type
 功能描述  : 获取转向原始方向的运动类型
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_turnt_to_original_direction_action_type(ACTION_STATUS_ENUM &action)
{
	get_rotate_action_type(action);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_turn_to_original_direction_respond
 功能描述  : 转向原始方向角度到达以后的响应处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_turn_to_original_direction_respond(void)
{
	set_local_move_state(LOCAL_MOVE_ALL_DONE);
	smooth_decelerate_stop();
	disable_linear_velocity_ajust();
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_turn_to_original_direction_deal
 功能描述  : 转向原始方向功能处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_turn_to_original_direction_deal(void)
{
	double angle = 0.0;
	MONITOR_ENUM stage;
	ACTION_STATUS_ENUM action;
	void(cfg_mobile_robot::*pf)(void) = NULL;
	
	get_monitor_angle_stage(stage);
	if (MONITOR_SLEEP == stage)
	{
		disable_angular_velocity_ajust();
		
		angle = get_turnt_to_original_direction_angle();
		get_turnt_to_original_direction_action_type(action);
		//debug_print_info("  action = %d   angle = %lf", action, angle);
		change_curr_action(action);
		pf = &cfg_mobile_robot::monitor_angle_turn_to_original_direction_respond;
		set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.left_or_right_bumper_respond
 功能描述  : 左右碰撞以后旋转响应角度以后的响应动作
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::left_or_right_bumper_respond(void)
{
	//smooth_decelerate_stop();
	change_curr_action(GO_FORWARD);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_left_bumper_respond
 功能描述  : 左边碰撞旋转响应
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_left_bumper_respond(void)
{
	left_or_right_bumper_respond();
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_left_bumper_deal
 功能描述  : 左边碰撞处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_left_bumper_deal(void)
{
	double angle = 0.0;
	double curr_angle = 0.0;
	MONITOR_ENUM stage;
	ACTION_STATUS_ENUM action = TURN_RIGHT;
	void(cfg_mobile_robot::*pf)(void) = NULL;

	get_monitor_angle_stage(stage);
	if (MONITOR_SLEEP == stage)
	{
		disable_angular_velocity_ajust();
		
		curr_angle = get_curr_pose_angle();
		angle = curr_angle - collide_adjusted_angle_;
		//string str;
		//get_action_status_str( action , str );
		//debug_print_error("########Change [%s]=%d; curr_angle = %lf; angle = %lf", str.c_str(), action,curr_angle, angle);
		change_curr_action(action);
		pf = &cfg_mobile_robot::monitor_angle_left_bumper_respond;
		set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_right_bumper_respond
 功能描述  : 右边碰撞旋转响应
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_right_bumper_respond(void)
{
	left_or_right_bumper_respond();
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_right_bumper_deal
 功能描述  : 右边碰撞处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_right_bumper_deal(void)
{
	double angle = 0.0;
	double curr_angle = 0.0;
	MONITOR_ENUM stage;
	ACTION_STATUS_ENUM action = TURN_LEFT;
	void(cfg_mobile_robot::*pf)(void) = NULL;

	get_monitor_angle_stage(stage);
	if (MONITOR_SLEEP == stage)
	{
		disable_angular_velocity_ajust();
		
		curr_angle = get_curr_pose_angle();
		angle = curr_angle + collide_adjusted_angle_;
		//string str;
		//get_action_status_str( action , str );
		//debug_print_error("###########Change [%s]=%d; curr_angle = %lf; angle = %lf", str.c_str(), action,curr_angle, angle);
		change_curr_action(action);
		pf = &cfg_mobile_robot::monitor_angle_right_bumper_respond;
		set_monitor_angle_rotate_call_back(angle, pf);
	}
}

#ifdef BLUEWAYS_DEBUG
/*****************************************************************************
 函 数 名: cfg_mobile_robot.test_adjust_velocity
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
bool cfg_mobile_robot::test_adjust_velocity(double &linear_velocity, double &angular_velocity)
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
	get_curr_action(action);
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
bool cfg_mobile_robot::test_adjust_velocity(double &linear_velocity, double &angular_velocity)
{
	bool flag = true;
	double veer = 1.0;
	double diff = 0.0;
	double offset = 0.0;
	double linear_v = linear_velocity;
	double angular_v = angular_velocity;
	
	diff = get_curr_angle_difference_respond();
	offset = fabs(diff);
	get_curr_action(action);
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
bool cfg_mobile_robot::test_adjust_velocity(double &linear_velocity, double &angular_velocity)
{
	bool flag = true;
	double veer = 1.0;
	double diff = 0.0;
	double offset = 0.0;
	double linear_v = linear_velocity;
	double angular_v = angular_velocity;
	ACTION_STATUS_ENUM action;
	static double last_offset = 0.0;

	diff = get_curr_angle_difference_respond();
	offset = fabs(diff);
	get_curr_action(action);
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
 函 数 名: cfg_mobile_robot.monitor_angle_adjust_velocity
 功能描述  : 调整速度
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_adjust_velocity(void)
{
	bool flag = false;
	double linear_velocity = 0.0;
	double angular_velocity = 0.0;
	
	get_velocity(linear_velocity, angular_velocity);
	flag = test_adjust_velocity(linear_velocity, angular_velocity);
	if ( true == flag) {
		set_velocity(linear_velocity, angular_velocity);
		set_adjust_velocity(flag);
	
		double curr = 0.0;
		double respond = 0.0;
		static double pre = 0.0;
		curr = get_curr_pose_angle();
		respond = get_monitor_angle_respond_goal();
		if (pre != curr) {
			pre = curr;
			debug_print_info("flag = %d, ||curr(%lf) --> respond(%lf)||, linear_v(%lf); angular_v(%lf)",flag, curr, respond, linear_velocity, angular_velocity);
		}
	}
	else
	{
		//debug_print_fatal("flag=%d", flag);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.monitor_angle_running
 功能描述  : 检测旋转偏移状态是否可以响应
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_running(void)
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
		curr_angle = get_curr_pose_angle();
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
			print_curr_pose_angle();
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
 函 数 名: cfg_mobile_robot.monitor_angle_respond
 功能描述  : 检测旋转角度响应
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_angle_respond(void)
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
 函 数 名: cfg_mobile_robot.monitor_stop_rotate
 功能描述  : 停止旋转
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月20日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::monitor_stop_rotate(void)
{
	double line_v = 0.0;
	double angular_v = 0.0;
	
	get_velocity(line_v, angular_v);
	angular_v = 0.0;
	bll_motion_control* p_instance = bll_motion_control::get_instance();
	p_instance->set_motion_control_velocity(line_v, angular_v);
}

void cfg_mobile_robot::smooth_decelerate_stop(void)
{
	change_curr_action(STOP);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_monitor_angle_rotate_call_back
 功能描述  : 注册监测指定角度以后响应的功能
 输入参数: double angle       
           void*pf_call_back  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_monitor_angle_rotate_call_back(double angle, void(cfg_mobile_robot::*pf_call_back)(void))
{
	void(cfg_mobile_robot::*pf)(void) = NULL;
	MONITOR_ENUM stage;
	
	get_monitor_angle_stage(stage);
	if (MONITOR_SLEEP == stage)
	{
		stage = MONITOR_RUNNING;
		set_monitor_angle_stage(stage);

		if (NULL != pf_call_back)
		{
			pf = pf_call_back;
		}
		
		set_monitor_angle_respond_goal_and_func(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.bumper_respond_deal
 功能描述  : 碰撞响应处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::bumper_sensor_respond_deal(void)
{
	bool left_state = false;
	bool right_state = false;
	static bool last_left_state = false;
	static bool last_right_state = false;
	BUMPER_ID_ENUM left_id = LEFT_BUMPER;
	BUMPER_ID_ENUM right_id = RIGHT_BUMPER;
	
	get_bumper_state(left_id, left_state);
	get_bumper_state(right_id, right_state);

	if ((last_left_state != left_state) && (last_right_state != right_state))
	{
		if ((true == left_state) && (true == right_state))
		{
			debug_print_warnning("【left】and【right】==【center】bump！");
			do_retreat();
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

/*****************************************************************************
 函 数 名: cfg_mobile_robot.cliff_sensor_respond_deal
 功能描述  : 悬崖传感器处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::cliff_sensor_respond_deal(void)
{
	bool ret = true;
	
	ret = test_robot_is_ok();
	if (false == ret )
	{
		return;
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.wheel_drop_sensor_respond_deal
 功能描述  : 跌落传感器处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::wheel_drop_sensor_respond_deal(void)
{
	bool curr_is_normal = true;
	static bool last_is_normal = true;
	
	curr_is_normal = test_wheel_sensor_is_normal();
	if ( false == curr_is_normal )
	{
		if ( true == last_is_normal )
		{
			save_running_status();
			bll_motion_control* p_instance = bll_motion_control::get_instance();
			p_instance->stop();
		}
	}
	else
	{
		if ( false == last_is_normal )
		{
			recover_running_status();
		}
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.ultrasonic_sensor_respond_deal
 功能描述  : 超声波传感器响应处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月17日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::ultrasonic_sensor_respond_deal(void)
{
	ULTRASONIC_SENSOR_STRU data;

	get_ultrasonic_sensor_state(data);
	if ( SAFETY_LEVEL_WARN == data.level )
	{
		
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.functional_mode
 功能描述  : 根据当前的行为模式状态运行相应的功能
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::functional_mode ( void )
{
	ACTION_STATUS_ENUM action;
	bll_motion_control* p_instance = bll_motion_control::get_instance();
	
	print_change_action_status();
	get_curr_action(action);
	switch ( action )
	{
		case STOP:
			p_instance->stop();
			break;
		case GO_FORWARD:
			p_instance->go_forward();
			break;
		case GO_BACK:
			p_instance->go_back();
			break;
		case TURN_LEFT:
			p_instance->turn_left();
			break;
		case TURN_RIGHT:
			p_instance->turn_right();
			break;
		case PIVOT:
			p_instance->pivot();
			break;
		case TURN_BACK_CLOCKWISE :
			p_instance->turn_back_clockwise();
			break;
		case TURN_BACK_ANTICLOCKWISE :
			p_instance->turn_back_anticlockwise();
			break;
		case TURN_RIGHT_ANGLE_CLOCKWISE :
			p_instance->turn_right_angle_clockwise();
			break;
		case TURN_RIGHT_ANGLE_ANTICLOCKWISE :
			p_instance->turn_right_angle_anticlockwise();
			break;
		case EDGE_WAYS:
			break;
		case AUTO_DOCK:
			break;
		default:
			p_instance->stop();
			break;
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_action_status_str
 功能描述  : 获取行动状态信息ID对应的字符串
 输入参数: ACTION_STATUS_ENUM id  
           string& str            
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年9月18日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_mobile_robot::get_action_status_str(ACTION_STATUS_ENUM id , string& str)
{
	bool ret = true;
	string name ("Invalid ACTION_STATUS_ENUM ID!");
	switch ( id )
	{
		case STOP:
			name.assign ( "STOP" );
			break;
		case GO_FORWARD:
			name.assign ( "GO_FORWARD" );
			break;
		case GO_BACK:
			name.assign ( "GO_BACK" );
			break;
		case TURN_LEFT:
			name.assign ( "TURN_LEFT" );
			break;
		case TURN_RIGHT:
			name.assign ( "TURN_RIGHT" );
			break;
		case PIVOT:
			name.assign ( "PIVOT" );
			break;
		case TURN_BACK_CLOCKWISE :
			name.assign ( "TURN_BACK_CLOCKWISE" );
			break;
		case TURN_BACK_ANTICLOCKWISE :
			name.assign ( "TURN_BACK_ANTICLOCKWISE" );
			break;
		case TURN_RIGHT_ANGLE_CLOCKWISE :
			name.assign ( "TURN_RIGHT_ANGLE_CLOCKWISE" );
			break;
		case TURN_RIGHT_ANGLE_ANTICLOCKWISE :
			name.assign ( "TURN_RIGHT_ANGLE_ANTICLOCKWISE" );
			break;
		case EDGE_WAYS:
			name.assign ( "EDGE_WAYS" );
			break;
		case AUTO_DOCK:
			name.assign ( "AUTO_DOCK" );
			break;
		case LOCAL_AREA:
			name.assign ( "LOCAL_AREA" );
			break;
		default:
			ret = false;
			break;
	}

	str.assign(name);
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.print_curr_action_status_str
 功能描述  : 打印当前的运行模式信息
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月18日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::print_curr_action_status_str(void)
{
	string str;
	ACTION_STATUS_ENUM action;
	
	get_curr_action(action);
	get_action_status_str(action , str);
	debug_print_info("%s", str.c_str());
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.print_change_action_status
 功能描述  : 打印显示当前改变的运行模式
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::print_change_action_status ( void )
{
	ACTION_STATUS_ENUM curr_action;
	static ACTION_STATUS_ENUM last_action;
	
	get_curr_action(curr_action);
	if ( curr_action != last_action )
	{
		std::cout<<std::endl;
		string str;
		get_action_status_str( curr_action , str );
		debug_print_info("%s", str.c_str());
		std::cout<<std::endl;
	}

	last_action = curr_action;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.print_curr_pose_angle
 功能描述  : 打印当前角度
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::print_curr_pose_angle(void)
{
	double curr_angle = get_curr_pose_angle();
	debug_print_info("%lf", curr_angle);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.change_curr_action
 功能描述  : 更改当前的运动行为模式
 输入参数: const ACTION_STATUS_ENUM action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::change_curr_action(const ACTION_STATUS_ENUM action)
{
	ACTION_STATUS_ENUM curr_action;
	
	get_curr_action(curr_action);
	if ( action != curr_action)
	{
		set_curr_action ( action);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.change_rotate_direction
 功能描述  : 修改旋转方向
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::change_rotate_direction(void)
{
	ACTION_STATUS_ENUM action;

	monitor_angle_set_turn_back_deal();
	get_turn_back_action_type(action);
	change_curr_action(action);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.do_retreat
 功能描述  : 后退
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::do_retreat ( void )
{
	set_curr_action(GO_BACK);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_front_position
 功能描述  : 获取当前驶向的参考线两头哪一个方向的点
 输入参数: POSE_STRU &position  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月17日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::get_front_position(POSE_STRU &position)
{
	bool direction_is_forward = false;
	
	direction_is_forward = test_local_move_direction_is_forward();
	if (true == direction_is_forward)
	{
		get_refer_line_end_point_pose(position);
		//debug_print_info("{ end.pos  (%lf, %lf) : %lf}", position.point.x, position.point.y, position.angle);
	}
	else
	{
		get_refer_line_start_point_pose(position);
		//debug_print_info("{ start.pos (%lf, %lf) : %lf}", position.point.x, position.point.y, position.angle);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_vertical_distance_curr_position_to_refer_line
 功能描述  : 获取当前点到参考线的垂直距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月17日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_vertical_distance_curr_position_to_refer_line(void)
{
	POSE_STRU curr;
	POSE_STRU original;
	double temp = 0.0;
	double radian = 0.0;
	double angle = 0.0;
	double diff_angle = 0.0;
	double acute_angle = 0.0;
	double refer_angle = 0.0;
	double opposite = 0.0;    //三角形对边
	double hypotenuse = 0.0;  //三角形斜边
	
	get_current_position(curr);
	get_local_move_original_pose(original);
	//printf("{curr.pos (%lf, %lf) : %lf}\n", curr.point.x, curr.point.y, curr.angle);
	//printf("{original.pos (%lf, %lf) : %lf}\n", original.point.x, original.point.y, original.angle);
	hypotenuse = get_distance(original.point.x, original.point.y, curr.point.x, curr.point.y);
	refer_angle = get_reference_angle();
	angle = get_angle(original.point.x, original.point.y, curr.point.x, curr.point.y);
	diff_angle = get_angle_differences(angle, refer_angle);
	//printf("diff_angle = get_angle_differences(angle=%lf, refer_angle=%lf) = %lf\n", angle, refer_angle, diff_angle);
	acute_angle = convert_to_acute_angle(diff_angle);
	radian = convert_degrees_to_radians(acute_angle);
	temp = sin(radian);
	opposite = hypotenuse*temp;
	//printf("hypotenuse=%lf; acute_angle=%lf; sin(radian=%lf)=%lf; opposite=%lf\n", hypotenuse, acute_angle, radian, temp, opposite);
	return opposite;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.set_current_position_to_refer_start_pose
 功能描述  : 将当前位置保存为直线行走的起始点位置
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::set_current_position_to_refer_start_pose(void)
{
	POSE_STRU curr;
	POSE_STRU target;
	POSE_STRU front_pos;
	double radian = 0.0;
	double acute_angle = 0.0;
	double hypotenuse = 0.0;  //三角形斜边
	double x_deviation = 0.0;
	double y_deviation = 0.0;
	double refer_angle = 0.0;
	double y_trend = 1.0;
	double vertical_distance = 0.0;
	bool area_part_is_left = false;

	//std::cout<<std::endl;
	//debug_print_warnning("++++++++++++");
	get_current_position(curr);
	set_straight_moving_refer_start_pose(curr);
	//printf("set curr.pos {(%lf, %lf) : %lf}\n", curr.point.x, curr.point.y, curr.angle);
	
	vertical_distance = get_vertical_distance_curr_position_to_refer_line();
	hypotenuse = vertical_distance;
	
	refer_angle = get_reference_angle();
	acute_angle = convert_to_acute_angle(refer_angle);
	radian = convert_degrees_to_radians(acute_angle);
	x_deviation = hypotenuse*sin(radian);
	y_deviation = hypotenuse*cos(radian);

	//printf("refer_angle(%lf)  ==>  acute_angle(%lf) ==> radian(%lf)\n", refer_angle, acute_angle, radian);
	//printf("sin(%lf) = %lf;   ", radian, sin(radian));
	//printf("cos(%lf) = %lf;\n", radian, cos(radian));
	//printf("hypotenuse  = %lf\n", hypotenuse);
	//printf("x_deviation = %lf,       y_deviation = %lf\n", x_deviation, y_deviation);

	area_part_is_left = test_local_move_area_part_is_left();
	if (false == area_part_is_left)
	{
		y_trend = -y_trend;
		//debug_print_fatal("area_part_is_left =%d ;y_trend=%lf", area_part_is_left, y_trend);
	}

	get_front_position(front_pos);
	target.angle = refer_angle;
	target.point.x = front_pos.point.x - x_deviation;
	target.point.y = front_pos.point.y + y_deviation*y_trend;
	set_straight_moving_refer_target_pose(target);
	
	//printf("set target.pos{ (%lf, %lf) : %lf}\n", target.point.x, target.point.y, target.angle);
	//debug_print_warnning("------------");
	//std::cout<<std::endl;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.straight_moving_ajust_velocity
 功能描述  : 直行调整速度
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::straight_moving_ajust_velocity(void)
{
//	POSE_STRU start;
//	POSE_STRU target;
//	POSE_STRU curr_pos;
//	
//	get_current_position(curr_pos);
//	get_straight_moving_refer_pose(start, target);
//	update_velocity(curr_pos, start, target);

	traight_line_moving_dynamic_regulation();
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.straight_driving_adjust_angle
 功能描述  : 直行调整角度
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::straight_driving_adjust_angle(void)
{
	double diff = 0.0;
	double curr_angle = 0.0;
	double direction_angle = 0.0;
	double angular_velocity = 0.0;
	bool is_clockwise = false;
	
	curr_angle = get_curr_pose_angle();
	direction_angle = get_traight_line_moving_direction_angle();
	diff = get_angle_differences(curr_angle, direction_angle);
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
		
		is_clockwise = test_angle_rotate_direction_is_clockwise(curr_angle, direction_angle);
		if (true == is_clockwise)
		{
			angular_velocity = -angular_velocity;
		}
	
		endble_angular_velocity_ajust(angular_velocity);
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.straight_driving_adjust_speed
 功能描述  : 直线调节速度
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::straight_driving_adjust_speed(void)
{
	double temp = 0.0;
	double distance = 0.0;
	double linear_velocity = 0.0;
	double angular_velocity = 0.0;
	
	get_velocity(linear_velocity, angular_velocity);
	temp = fabs(linear_velocity);
	distance = get_distance_to_traight_line_moving_target_pos();
	if ((distance < 0.12) && (distance >= 0.08))
	{
		if (temp >= 0.2)
		{
			linear_velocity = 0.2;
			endble_linear_velocity_ajust(linear_velocity);
		}
	}
	else if ((distance < 0.08) && (distance >= 0.04))
	{
		if (temp >= 0.2)
		{
			linear_velocity = 0.15;
			endble_linear_velocity_ajust(linear_velocity);
		}
	}
	else if ((distance < 0.04) && (distance >= 0.02))
	{
		if (temp >= 0.15)
		{
			linear_velocity = 0.12;
			endble_linear_velocity_ajust(linear_velocity);
		}
	}
	else if ((distance < 0.02) && (distance >= 0.0))
	{
		if (temp >= 0.1)
		{
			linear_velocity = 0.1;
			endble_linear_velocity_ajust(linear_velocity);
		}
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.update_traight_line_moving_data
 功能描述  : 更新直行数据
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::update_traight_line_moving_data(void)
{
	POSE_STRU pos;
	bool flag = false;
	ACTION_STATUS_ENUM action;
	
	flag = test_is_traight_line_moving();
	if (true == flag)
	{
		get_current_position(pos);
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
 函 数 名: cfg_mobile_robot.get_distance_to_start_point_pos
 功能描述  : 获取当前位置到参考线初始点位置的距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_distance_to_start_point_pos(void)
{
	double distance = 0.0;
	POSE_STRU curr_pos;
	POSE_STRU start_pos;
	
	get_current_position(curr_pos);
	get_refer_line_start_point_pose(start_pos);
	distance = get_distance(curr_pos.point.x, curr_pos.point.y, start_pos.point.x, start_pos.point.y);
	//debug_print_info("distance(%lf) = |curr_pos.point=(%lf, %lf)<-->start_pos.point=(%lf, %lf)|", distance, curr_pos.point.x, curr_pos.point.y, start_pos.point.x, start_pos.point.y);
	return distance;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_distance_to_end_point_pos
 功能描述  : 获取当前位置到参考线末尾点位置的距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_distance_to_end_point_pos(void)
{
	double distance = 0.0;
	POSE_STRU curr_pos;
	POSE_STRU end_pos;
	
	get_current_position(curr_pos);
	get_refer_line_end_point_pose(end_pos);
	distance = get_distance(curr_pos.point.x, curr_pos.point.y, end_pos.point.x, end_pos.point.y);
	//debug_print_info("distance(%lf) = |curr_pos.point=(%lf, %lf)<-->end_pos.point=(%lf, %lf)|", distance, curr_pos.point.x, curr_pos.point.y, end_pos.point.x, end_pos.point.y);

	return distance;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_distance_to_traight_line_moving_start_pos
 功能描述  : 获取当前位置到开始直行位置的距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_distance_to_traight_line_moving_start_pos(void)
{
	double distance = 0.0;
	POSE_STRU curr_pos;
	POSE_STRU pos;
	
	get_current_position(curr_pos);
	get_traight_line_moving_start_pos(pos);
	distance = get_distance(curr_pos.point.x, curr_pos.point.y, pos.point.x, pos.point.y);
	//debug_print_info("distance(%lf) = |curr_pos.point=(%lf, %lf)<-->pos.point=(%lf, %lf)|", distance, curr_pos.point.x, curr_pos.point.y, pos.point.x, pos.point.y);
	return distance;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_distance_to_traight_line_moving_target_pos
 功能描述  : 获取到目标点的距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_distance_to_traight_line_moving_target_pos(void)
{
	double distance = 0.0;
	POSE_STRU curr_pos;
	POSE_STRU pos;
	
	get_current_position(curr_pos);
	get_traight_line_moving_target_pos(pos);
	distance = get_distance(curr_pos.point.x, curr_pos.point.y, pos.point.x, pos.point.y);
	//debug_print_info("distance(%lf) = |curr_pos.point=(%lf, %lf)<-->pos.point=(%lf, %lf)|", distance, curr_pos.point.x, curr_pos.point.y, pos.point.x, pos.point.y);
	return distance;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_vertical_dimension_curr_pos_to_refer_line
 功能描述  : 获取当前点到参考线的垂直距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_vertical_dimension_curr_pos_to_refer_line(void)
{
	POSE_STRU pos;
	POSE_STRU curr_pos;
	double radian = 0.0;
	double angle = 0.0;
	double refe_angle = 0.0;
	double acute_angle = 0.0;
	double distance = 0.0;
	double vertical_dimension = 0.0;
	bool direction_is_forward = false;

	direction_is_forward = test_local_move_direction_is_forward();
	if (true == direction_is_forward)
	{
		get_refer_line_start_point_pose(pos);
		angle = get_refer_line_start_pos_to_curr_pos_angle();
	}
	else
	{
		get_refer_line_end_point_pose(pos);
		angle = get_refer_line_end_pos_to_curr_pos_angle();
	}

	get_current_position(curr_pos);
	refe_angle = get_reference_angle();
	distance = get_distance(curr_pos.point.x, curr_pos.point.y, pos.point.x, pos.point.y);

	acute_angle = get_angle_differences(angle, refe_angle);
	radian = convert_degrees_to_radians(acute_angle);
	vertical_dimension = distance*sin(radian);
	//debug_print_warnning("direction_is_forward(%d), angle(%lf), refe_angle(%lf), acute_angle(%lf),radian(%lf), vertical_dimension(%lf)",direction_is_forward, angle, refe_angle, acute_angle,radian, vertical_dimension);

	return vertical_dimension;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_refer_line_start_pos_to_curr_pos_angle
 功能描述  : 获取参考线起始点指向当前位置点的角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_refer_line_start_pos_to_curr_pos_angle(void)
{
	double angle = 0.0;
	POSE_STRU pos;
	POSE_STRU curr_pos;
	
	get_current_position(curr_pos);
	get_refer_line_start_point_pose(pos);
	angle = get_angle(pos.point.x, pos.point.y, curr_pos.point.x, curr_pos.point.y);

	return angle;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_refer_line_end_pos_to_curr_pos_angle
 功能描述  : 获取参考线末尾的指向当前位置点的方向角度
                 
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_refer_line_end_pos_to_curr_pos_angle(void)
{
	double angle = 0.0;
	POSE_STRU pos;
	POSE_STRU curr_pos;
	
	get_current_position(curr_pos);
	get_refer_line_end_point_pose(pos);
	angle = get_angle(pos.point.x, pos.point.y, curr_pos.point.x, curr_pos.point.y);

	return angle;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.get_center_pose_to_curr_pos_angle
 功能描述  : 获取中心点指向当前点的角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_mobile_robot::get_center_pose_to_curr_pos_angle(void)
{
	double angle = 0.0;
	POSE_STRU orig_pos;
	POSE_STRU curr_pos;
	
	get_current_position(curr_pos);
	get_local_move_original_pose(orig_pos);
	angle = get_angle(orig_pos.point.x, orig_pos.point.y, curr_pos.point.x, curr_pos.point.y);

	return angle;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.test_arrive_at_refer_line
 功能描述  : 检测是否已经运行到与参考方向平行
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_mobile_robot::test_arrive_at_refer_line(void)
{
	bool flag = false;
	bool is_over = false;
	double current = 0.0;
	double refe_angle = 0.0;
	bool area_part_is_left = false;
	bool direction_is_forward = false;

	flag = test_is_traight_line_moving();
	if (true == flag)
	{
		refe_angle = get_reference_angle();
		current = get_center_pose_to_curr_pos_angle();
	
		area_part_is_left = test_local_move_area_part_is_left();
		direction_is_forward = test_local_move_direction_is_forward();
		if(true == area_part_is_left)
		{
			if(true == direction_is_forward)
			{
				is_over = test_angle_is_over_clockwise(current, refe_angle);
			}
			else
			{
				is_over = test_angle_is_over_anticlockwise(current, refe_angle);
			}
		}
		else
		{
			if(false == direction_is_forward)
			{
				is_over = test_angle_is_over_clockwise(current, refe_angle);
			}
			else
			{
				is_over = test_angle_is_over_anticlockwise(current, refe_angle);
			}
		}
		//debug_print_info("is_over=%d; is_left =%d ;direction_is_forward =%d; current= %lf, refe_angle= %lf",is_over, area_part_is_left, direction_is_forward, current, refe_angle);	
	}
	else
	{
		//debug_print_error("false == flag ==%d", flag);
	}
	
	return is_over;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.update_refer_line_traight_line_moving_target_pos
 功能描述  : 更新目标坐标点位置
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::update_refer_line_traight_line_moving_target_pos(void)
{
	POSE_STRU pos;
	bool direction_is_forward = false;
	
	direction_is_forward = test_local_move_direction_is_forward();
	if (true == direction_is_forward)
	{
		get_refer_line_end_point_pose(pos);
		debug_print_info("set target pos end {(%lf, %lf): %lf}", pos.point.x, pos.point.y, pos.angle);
	}
	else
	{
		get_refer_line_start_point_pose(pos);
		debug_print_info("set target pos start {(%lf, %lf): %lf}", pos.point.x, pos.point.y, pos.angle);
	}
	set_traight_line_moving_target_pos(pos);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.traight_line_moving_dynamic_regulation
 功能描述  : 动态调整直线运行速度
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::traight_line_moving_dynamic_regulation(void)
{
	bool flag = false;
	
	flag = test_is_traight_line_moving();
	if (true == flag)
	{
		straight_driving_adjust_angle();
		straight_driving_adjust_speed();
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.local_move_start
 功能描述  : 开始运行局部无障碍清扫
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::local_move_start(void)
{
	set_local_move_direction(FORWARD);
	set_local_move_area_part(AREA_PART_LEFT);
	set_local_move_state(LOCAL_MOVE_PIVOT);
	
	set_traight_line_moving();
	disable_linear_velocity_ajust();
	update_refer_line_traight_line_moving_target_pos();
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.local_move_pivot
 功能描述  : 局部无障碍清扫原地旋转返回
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::local_move_pivot(void)
{
	bool flag = false;
	double length = 0.0;
	double distance = 0.0;
	flag = test_is_traight_line_moving();
	if (true == flag)
	{
		length = get_local_move_edge_length();
		distance = get_distance_to_start_point_pos();
		if (distance >= length)
		{
			monitor_angle_set_pivot_deal();
		}
		else
		{
			flag = false;
			flag = test_detect_obstacle_turn_back();
			if (true == flag)
			{
				monitor_angle_set_pivot_deal();
			}
			else
			{
				traight_line_moving_dynamic_regulation();
			}
		}
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.local_move_fst_line_start
 功能描述  : 开始局部弓字行走
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::local_move_fst_line_start(void)
{
	bool flag = false;
	double length = 0.0;
	double distance = 0.0;

	length = get_local_move_edge_length();
	distance = get_distance_to_end_point_pos();
	if (distance >= length)
	{
		change_rotate_direction();
		set_local_move_state(LOCAL_MOVE_FST_HALF);
	}
	else
	{
		flag = test_detect_obstacle_turn_back();
		if (true == flag)
		{
			change_rotate_direction();
			set_local_move_state(LOCAL_MOVE_FST_HALF);
		}
		else
		{
			traight_line_moving_dynamic_regulation();
		}
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.local_move_return_reference_line
 功能描述  : 返回参考线
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::local_move_return_reference_line(void)
{
	bool flag = false;
	flag = test_arrive_at_refer_line();
	if (true == flag)
	{
		disable_linear_velocity_ajust();
		set_local_move_state(LOCAL_MOVE_SEC_HALF_START);
		smooth_decelerate_stop();
		clear_traight_line_moving_data();
	}
	else
	{
		traight_line_moving_dynamic_regulation();
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.local_move_return_center_pose
 功能描述  : 局部弓字型行走返回中心位置
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::local_move_return_center_pose(void)
{
	bool flag = false;
	
	flag = test_arrive_at_refer_line();
	if (true == flag)
	{
		set_local_move_state(LOCAL_MOVE_TURN_TO_ORIGINAL_DIR);
		smooth_decelerate_stop();
		clear_traight_line_moving_data();
	}
	else
	{
		traight_line_moving_dynamic_regulation();
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.local_move_fst_half_area
 功能描述  : 局部弓字行走第一半区往返运动
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::local_move_fst_half_area(void)
{
	bool flag = false;
	bool half_done_flag = false;
	double l_x = 0.0;
	double r_x = 0.0;
	double length = 0.0;
	double half_length = 0.0;
	double distance = 0.0;
	double ref_value = 0.0;
	double precision = 0.01;
	double vertical_dimension = 0.0;
	POSE_STRU curr_pos;
	POSE_STRU original_pose;
	
	flag = test_is_traight_line_moving();
	if (true == flag)
	{
		#ifdef ENABLE_MOUDLATE
		update_traight_line_moving_data();
		straight_moving_ajust_velocity();
		#endif /* ENABLE_MOUDLATE */
		
		distance = get_distance_to_traight_line_moving_start_pos();
		length = get_local_move_edge_length();

		half_length = length/2;
		vertical_dimension = get_vertical_dimension_curr_pos_to_refer_line();
		if (vertical_dimension > half_length)
		{
			half_done_flag = true;
		}
		
		if (distance >= length)
		{
			if (true == half_done_flag)
			{
				set_local_move_state(LOCAL_MOVE_FST_HALF_DONE);
				return;
			}
			change_rotate_direction();
		}
		else
		{
			flag = test_detect_obstacle_turn_back();
			if (true == flag)
			{
				if (true == half_done_flag)
				{
					set_local_move_state(LOCAL_MOVE_FST_HALF_DONE);
					return;
				}
				change_rotate_direction();
			}
		}
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.local_move_sec_half_area
 功能描述  : 局部弓字行走第二半区往返运动
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::local_move_sec_half_area(void)
{
	bool flag = false;
	double l_x = 0.0;
	double r_x = 0.0;
	double temp = 0.0;
	double length = 0.0;
	double half_length = 0.0;
	double distance = 0.0;
	double ref_value = 0.0;
	double precision = 0.01;
	double vertical_dimension = 0.0;
	POSE_STRU curr_pos;
	POSE_STRU original_pose;
	ACTION_STATUS_ENUM curr_action;
	
	flag = test_is_traight_line_moving();
	if (true == flag)
	{
		#ifdef ENABLE_MOUDLATE
		if (true == flag)
		{
			straight_moving_ajust_velocity();
		}
		#endif /* ENABLE_MOUDLATE */

		distance = get_distance_to_traight_line_moving_start_pos();
		length = get_local_move_edge_length();
		if (distance >= length)
		{
			half_length = length/2;
			vertical_dimension = get_vertical_dimension_curr_pos_to_refer_line();
			if (vertical_dimension > half_length)
			{
				set_local_move_state(LOCAL_MOVE_TURN_TO_CENTER_DIR);
				return;
			}
			
			change_rotate_direction();
		}
		else
		{
			flag = test_detect_obstacle_turn_back();
			if (true == flag)
			{
				change_rotate_direction();
			}
		}
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.publish_trajectory
 功能描述  : 发布轨迹数据
 输入参数: double x   
           double y   
           double th  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月30日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::publish_trajectory(double x, double y, double th)
{
	ros::Time current_time;
	static nav_msgs::Path path;
	geometry_msgs::PoseStamped pose_stamped;
	
	current_time = ros::Time::now();
	
	pose_stamped.header.stamp = current_time;
	pose_stamped.header.frame_id = "odom";
	pose_stamped.pose.position.x = x;
	pose_stamped.pose.position.y = y;

	geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
	pose_stamped.pose.orientation.x = goal_quat.x;
	pose_stamped.pose.orientation.y = goal_quat.y;
	pose_stamped.pose.orientation.z = goal_quat.z;
	pose_stamped.pose.orientation.w = goal_quat.w;
	
	path.header.stamp=current_time;
	path.header.frame_id="odom";
	path.poses.push_back(pose_stamped);
	
	pub_path_.publish(path);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.show_trajectory
 功能描述  : 显示轨迹
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::show_trajectory(void)
{
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;
	POSE_STRU pos;
	get_current_position(pos);
	
	x = pos.point.x;
	y = pos.point.y;
	th = pos.angle;
	publish_trajectory(x, y, th);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.test_detect_obstacle_turn_back
 功能描述  : 检测到障碍物返回
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_mobile_robot::test_detect_obstacle_turn_back(void)
{
	bool flag = false;
	ULTRASONIC_SENSOR_STRU state;
	
	get_ultrasonic_sensor_state(state);
	if (SAFETY_LEVEL_WARN == state.level)
	{
		flag = true;
		debug_print_warnning("flag = true;");
	}
	else if (SAFETY_LEVEL_FATAL == state.level)
	{
		flag = true;
		debug_print_fatal("flag = true;");
	}
	else
	{
		flag = false;
	}
	
	return flag;
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.do_local_move
 功能描述  : 局部无障碍弓字型遍历行驶
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::do_local_move(void)
{
	bool flag = false;
	LOCAL_MOVE_STATE_ENUM state;
	
	flag = test_local_move_is_enable();
	if (true == flag)
	{
		save_first_line_refer_direction();

		get_local_move_state(state);
		if (LOCAL_MOVE_START == state)
		{
			local_move_start();
		}
		else if (LOCAL_MOVE_PIVOT == state)
		{
			local_move_pivot();
		}
		else if (LOCAL_MOVE_FST_LINE_START == state)
		{
			local_move_fst_line_start();
		}
		else if (LOCAL_MOVE_FST_HALF == state)
		{
			local_move_fst_half_area();
		}
		else if (LOCAL_MOVE_FST_HALF_DONE == state)
		{
			monitor_angle_turn_to_refer_line_deal();
		}
		else if (LOCAL_MOVE_RETURN_REFER_LINE == state)
		{
			local_move_return_reference_line();
		}
		else if (LOCAL_MOVE_SEC_HALF_START == state)
		{
			monitor_angle_turn_to_center_deal();
		}
		else if (LOCAL_MOVE_SEC_HALF == state)
		{
			local_move_sec_half_area();
		}
		else if (LOCAL_MOVE_TURN_TO_CENTER_DIR == state)
		{
			monitor_angle_turn_to_original_pose_deal();
		}
		else if (LOCAL_MOVE_RETURN_CENTER_POS == state)
		{
			local_move_return_center_pose();
		}
		else if (LOCAL_MOVE_TURN_TO_ORIGINAL_DIR == state)
		{
			monitor_angle_turn_to_original_direction_deal();
		}
		else if (LOCAL_MOVE_ALL_DONE == state)
		{
			set_local_move_state(LOCAL_MOVE_STOP);
		}
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.timer_callback
 功能描述  : 定时器回调函数
 输入参数: const ros::TimerEvent& event  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月11日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::timer_callback(const ros::TimerEvent& event)
{
	POSE_STRU pos;
	ACTION_STATUS_STRU status;
	get_action_status ( status );
	get_current_position(pos);
	//debug_print_info("robot current postion :-------------------------------------{(%lf, %lf) : %lf}", pos.point.x, pos.point.y, pos.angle);
	MONITOR_ENUM stage;
	get_monitor_angle_stage(stage);
	
	//debug_print_info("status.curr_action=%d, status.last_action=%d, stage=%d", status.curr_action, status.last_action, stage);
	//print_curr_action_status_str();
	//print_local_move_direction();
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.retreat_callback
 功能描述  : 后退响应回调函数
 输入参数: const ros::TimerEvent& event  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::retreat_callback(const ros::TimerEvent& event)
{
	static int n = 0;
	ACTION_STATUS_ENUM curr_action;
	
	get_curr_action(curr_action);
	if ( GO_BACK == curr_action)
	{
		n++;
		if (n > 80)
		{
			debug_print_fatal("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
			monitor_angle_left_bumper_deal();
			n = 0;
		}
	}
	else
	{
		n = 0;
	}
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.register_time_callback
 功能描述  : 注册消息处理相关回调函数
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::register_time_callback ( void )
{
	ros::NodeHandle node_h;
	//定时器
	timer_ = node_h.createTimer(ros::Duration(2), &cfg_mobile_robot::timer_callback, p_instance_);
	retreat_timer_ = node_h.createTimer(ros::Duration(0.01), &cfg_mobile_robot::retreat_callback, p_instance_);
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.register_msgs_and_timers
 功能描述  : 注册消息和定时器
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::register_msgs_and_timers(void)
{
	register_time_callback();
	
	ros::NodeHandle node_h;
	pub_path_ = node_h.advertise<nav_msgs::Path>("trajectory", 1, true);
	
	drv_sensor* p_instance = drv_sensor::get_instance();
	p_instance->register_sensor_msgs_callback();
}

/*****************************************************************************
 函 数 名: cfg_mobile_robot.function_processor
 功能描述  : 移动机器人功能处理机
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_mobile_robot::function_processor ( void )
{
	functional_mode();
	
	sensors_deal();
	
	monitor_angle_running();
	monitor_angle_respond();

	local_cover_movement();

	show_trajectory();
}

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/


