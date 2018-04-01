/******************************************************************************

  版权所有 (C), 2017-2028, _ _ _ Co., Ltd.

 ******************************************************************************
  文件名称: bll_motion_control.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月19日
  最近修改:
  功能描述   : 运动控制
  函数列表:
  修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "bll_motion_control.h"

#include "bll_traight_line_moving.h"

#include "cfg_if_mobile_robot.h"
#include "cfg_if_modulate.h"

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

bll_motion_control* bll_motion_control::p_instance_ = nullptr;
pthread_mutex_t bll_motion_control::mutex_ = PTHREAD_MUTEX_INITIALIZER;

/*****************************************************************************
 函 数 名: bll_motion_control.bll_motion_control
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_motion_control
 
 修改历史:
  1.日     期: 2017年12月20日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_motion_control::bll_motion_control()
{
	go_back_linear_velocity_ = -0.2;               //线速度(m/s)
	linear_velocity_ = 0.25;               //线速度(m/s)
	angular_velocity_ = 0.8;               //角速度(rad/s)
	linear_velocity_clockwise_ = 0.08;//0.15;     //顺时针旋转返回线速度(m/s)
	angular_velocity_clockwise_ = 1.0;//0.8;     //顺时针旋转返回角速度(rad/s)
}

/*****************************************************************************
 函 数 名: bll_motion_control.~bll_motion_control
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_motion_control
 
 修改历史:
  1.日     期: 2017年12月20日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_motion_control::~bll_motion_control()
{

}

/*****************************************************************************
 函 数 名: bll_motion_control.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: bll_motion_control*
 
 修改历史:
  1.日     期: 2017年12月20日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_motion_control* bll_motion_control::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr == p_instance_)
		{
			p_instance_ = new bll_motion_control();
		}
		pthread_mutex_unlock(&mutex_);
	}
	return p_instance_;
}

/*****************************************************************************
 函 数 名: bll_motion_control.release_instance
 功能描述  : 释放实例
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月20日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::release_instance(void)
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
 函 数 名: bll_motion_control.set_motion_control_velocity
 功能描述  : 设置运动控制的速度
 输入参数: double line_v     
           double angular_v  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月20日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::set_motion_control_velocity(double line_v, double angular_v)
{
	bool flag = false;
	bool ret = false;
	bool line_v_flag = false;
	bool angular_v_flag = false;
	double line = line_v;
	double angular = angular_v;
	
	flag = cfg_if_update_velocity(line_v, angular_v);

	line_v_flag = cfg_if_get_linear_velocity_ajust(line_v);
	if ( true == line_v_flag )
	{
//		bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
//		flag = p_traight_line_moving->test_is_traight_line_moving();
//		if (true == flag)
//		{
//			debug_print_error("line_v_flag=%d; flag=%d; line=%lf; line_v=%lf",line_v_flag, flag, line, line_v);
//		}
		cfg_if_disable_linear_velocity_ajust();
		ret = true;
	}

	angular_v_flag = cfg_if_get_angular_velocity_ajust(angular_v);
	if ( true == angular_v_flag )
	{
		cfg_if_disable_angular_velocity_ajust();
		ret = true;
	}
	
	cfg_if_set_run_velocity(line_v, angular_v);
}

/*****************************************************************************
 函 数 名: bll_motion_control.straight_moving
 功能描述  : 以指定速度直线行走
 输入参数: double line_v
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月3日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::straight_moving( double line_v )
{
	double angular_v = 0.0;
	set_motion_control_velocity(line_v, angular_v);
}

/*****************************************************************************
 函 数 名: bll_motion_control.rotate_moving
 功能描述  : 以指定角速度旋转
 输入参数: double angular_v
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月3日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::rotate_moving ( double angular_v )
{
	const double line_v = 0.0;
	set_motion_control_velocity(line_v, angular_v);
}

/*****************************************************************************
 函 数 名: bll_motion_control.stop
 功能描述  : 停止
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::stop ( void )
{
	set_motion_control_velocity( 0, 0 );
}

/*****************************************************************************
 函 数 名: bll_motion_control.go_forward
 功能描述  : 向前执行
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::go_forward ( void )
{
	straight_moving ( linear_velocity_ );
}

/*****************************************************************************
 函 数 名: bll_motion_control.go_back
 功能描述  : 向后执行
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::go_back ( void )
{
	straight_moving ( go_back_linear_velocity_ );
}

/*****************************************************************************
 函 数 名: bll_motion_control.turn_left
 功能描述  : 左转弯
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::turn_left ( void )
{
	rotate_moving( angular_velocity_ );
}

/*****************************************************************************
 函 数 名: bll_motion_control.turn_right
 功能描述  : 右转弯
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::turn_right ( void )
{
	rotate_moving( -angular_velocity_ );
}

/*****************************************************************************
 函 数 名: bll_motion_control.pivot
 功能描述  : 向后旋转180度
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::pivot ( void )
{
	rotate_moving( angular_velocity_ );
}

/*****************************************************************************
 函 数 名: bll_motion_control.turn_back_clockwise
 功能描述  : 顺时针返回
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::turn_back_clockwise ( void )
{
	const double line_v = linear_velocity_clockwise_;
	const double angular_v = -angular_velocity_clockwise_;
	
	set_motion_control_velocity(line_v, angular_v);
}

/*****************************************************************************
 函 数 名: bll_motion_control.turn_back_anticlockwise
 功能描述  : 逆时针返回
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::turn_back_anticlockwise ( void )
{
	const double line_v = linear_velocity_clockwise_;
	const double angular_v = angular_velocity_clockwise_;
	
	set_motion_control_velocity(line_v, angular_v);
}

/*****************************************************************************
 函 数 名: bll_motion_control.turn_right_angle_clockwise
 功能描述  : 原地顺时针旋转一个直角
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::turn_right_angle_clockwise(void)
{
	rotate_moving( -angular_velocity_clockwise_ );
}

/*****************************************************************************
 函 数 名: bll_motion_control.turn_right_angle_anticlockwise
 功能描述  : 原地逆时针旋转一个直角
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_motion_control::turn_right_angle_anticlockwise(void)
{
	rotate_moving( angular_velocity_clockwise_ );
}

/******************************************************************************
 * 内部函数定义
 ******************************************************************************/


