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
	initialize ();
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
 函 数 名: cfg_mobile_robot.initialize
 功能描述  : 初始化
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数

*****************************************************************************/
void cfg_mobile_robot::initialize ( void )
{
	ACTION_STATUS_STRU status = {LOCAL_AREA, STOP};
	set_action_status(status);
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
	angel_base angel_base_instance;
	real_degree = angel_base_instance.convert_radians_to_degrees(real_yaw);
	pos.point.x = x;
	pos.point.y = y;
	pos.angle = real_degree;
	set_current_position(pos);
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
	angel_base angel_base_instance;
	reverse_angle = angel_base_instance.get_reverse_angle(curr_angle);
	
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
	angel_base angel_base_instance;
	angle = get_curr_pose_angle();
	if (CLOCKWISE == direction)
	{
		ret = angel_base_instance.get_right_angle_clockwise(angle);
		debug_print_warnning("ret = get_right_angle_clockwise(%lf)=%lf", angle, ret);
	}
	else
	{
		ret = angel_base_instance.get_right_angle_anticlockwise(angle);
		debug_print_warnning("ret = get_right_angle_anticlockwise(%lf)=%lf", angle, ret);
	}

	return ret;
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

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/


