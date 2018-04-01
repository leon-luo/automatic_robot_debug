/******************************************************************************

  版权所有 (C), 2017-2028, _ _ _ Co., Ltd.

 ******************************************************************************
  文件名称: bll_ultrasonic.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月21日
  最近修改:
  功能描述   : 超声波传感器类定义
  函数列表:
  修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "bll_ultrasonic.h"

#include "cfg_if_mobile_robot.h"
#include "debug_function.h"

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
pthread_mutex_t bll_ultrasoic::mutex_;
bll_ultrasoic* bll_ultrasoic::p_instance_ = nullptr;

/*****************************************************************************
 函 数 名: bll_ultrasoic.bll_ultrasoic
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_ultrasoic
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_ultrasoic::bll_ultrasoic()
{

}

/*****************************************************************************
 函 数 名: bll_ultrasoic.~bll_ultrasoic
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_ultrasoic
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_ultrasoic::~bll_ultrasoic()
{

}

/*****************************************************************************
 函 数 名: bll_ultrasoic.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: bll_ultrasoic*
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_ultrasoic* bll_ultrasoic::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr == p_instance_)
		{
			p_instance_ = new bll_ultrasoic();
		}
		pthread_mutex_unlock(&mutex_);
	}
	return p_instance_;
}

/*****************************************************************************
 函 数 名: bll_ultrasoic.release_instance
 功能描述  : 释放实例
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_ultrasoic::release_instance(void)
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
 函 数 名: bll_ultrasoic.ultrasonic_sensor_respond_deal
 功能描述  : 超声波传感器响应处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_ultrasoic::ultrasonic_sensor_respond_deal(void)
{
	ULTRASONIC_SENSOR_STRU data;
	
	cfg_if_get_ultrasonic_sensor(data);
	if ( SAFETY_LEVEL_WARN == data.level )
	{
		
	}
}

/*****************************************************************************
 函 数 名: bll_ultrasoic.update_obstatcle_safety_level
 功能描述  : 更新障碍物距离安全等级
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_ultrasoic::update_obstatcle_safety_level(void)
{
	SAFETY_LEVEL_ENUM level;
	static SAFETY_LEVEL_ENUM last_level;
	ULTRASONIC_SENSOR_STRU data;
	
	cfg_if_get_ultrasonic_sensor(data);
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
		cfg_if_set_ultrasonic_sensor(data);
	}
}

/*****************************************************************************
 函 数 名: bll_ultrasoic.update_ultrasonic_sensor_data
 功能描述  : 更新超声波传感器检测到数据
 输入参数: double value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_ultrasoic::update_ultrasonic_sensor_data(double value)
{
	ULTRASONIC_SENSOR_STRU data;
	
	cfg_if_get_ultrasonic_sensor(data);
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
		cfg_if_set_ultrasonic_sensor(data);
		if (( 4.5 < value ) && (value < 500.0))//超声波检测范围[4cm ~ 5m]
		{
			update_obstatcle_safety_level();
		}
	}
	else
	{
		//debug_print_warnning("data.enable=, value=%lf", data.enable, value);
	}
}

/******************************************************************************
 * 内部函数定义
 ******************************************************************************/


