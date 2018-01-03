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

#include "cfg_mobile_robot.h"

#include <stdlib.h>

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
	disable_linear_velocity_ajust();
	disable_angular_velocity_ajust();
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
 函 数 名: cfg_modulate.update_velocity
 功能描述  : 检测更新调整速度
 输入参数: double &line_v     
           double &angular_v  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月20日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
//bool cfg_modulate::update_velocity(double &line_v, double &angular_v)
//{
//	bool ret = false;
//	bool line_v_flag = false;
//	bool angular_v_flag = false;
//	cfg_mobile_robot* p_instance = cfg_mobile_robot::get_instance();
//
//	line_v_flag = get_linear_velocity_ajust(line_v);
//	if ( true == line_v_flag )
//	{
//		p_instance->set_linear_velocity(line_v, angular_v);
//		disable_linear_velocity_ajust();
//		ret = true;
//	}
//
//	angular_v_flag = get_angular_velocity_ajust(angular_v);
//	if ( true == angular_v_flag )
//	{
//		p_instance->set_angular_velocity(line_v, angular_v);
//		disable_angular_velocity_ajust();
//		ret = true;
//	}
//	
//	return ret;
//}

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/




