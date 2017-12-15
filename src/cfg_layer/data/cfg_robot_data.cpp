/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: cfg_robot_data.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年8月1日
  最近修改:
  功能描述   : 机器人数据类定义
  函数列表:
  修改历史:
  1.日     期: 2017年8月1日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "cfg_robot_data.h"

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
pthread_mutex_t cfg_robot_data::mutex;
cfg_robot_data* cfg_robot_data::p_instance_ = NULL;

/*****************************************************************************
 函 数 名: cfg_robot_data.cfg_robot_data
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: cfg_robot_data
 
 修改历史:
  1.日     期: 2017年8月1日
    作     者: Leon
    修改内容: 新生成函数

*****************************************************************************/
cfg_robot_data::cfg_robot_data()
{
	ULTRASONIC_SENSOR_STRU ultrasonic_sensor = {35.0, 25.0, 15.0, false, SAFETY_LEVEL_SAFE};
	set_ultrasonic_sensor_state(ultrasonic_sensor);

	set_bumper_state(LEFT_BUMPER, false);
	set_bumper_state(CENTER_BUMPER, false);
	set_bumper_state(RIGHT_BUMPER, false);

	set_cliff_state(LEFT_CLIFF, false);
	set_cliff_state(RIGHT_CLIFF, false);
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_instance
 功能描述  : 获取实例
 输入参数  : 无
 输出参数: 无
 返 回 值: cfg_robot_data*
 修改历史:
  1.日     期: 2017年8月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
cfg_robot_data* cfg_robot_data::get_instance(void)
{
	if (p_instance_ == NULL)
	{
		pthread_mutex_lock(&mutex);
		if (p_instance_ == NULL)
		{
			p_instance_ = new cfg_robot_data();
		}
		pthread_mutex_unlock(&mutex);
	}
	
	return p_instance_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_odometry_updated
 功能描述  : 设置里程计是否更新标记
 输入参数: bool flag  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_odometry_updated(bool flag)
{
	odometry_updated_ = flag;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_odometry_updated
 功能描述  : 获取程计是否更新标记
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_robot_data::get_odometry_updated(void)
{
	return odometry_updated_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_origin_position
 功能描述  : 设置原点位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_origin_position(const POSE_STRU pos)
{
	origin_pos_ = pos;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_origin_position
 功能描述  : 获取原点位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_origin_position(POSE_STRU &pos)
{
	pos = origin_pos_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_current_position
 功能描述  : 设置当前位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_current_position(const POSE_STRU pos)
{
	current_pos_ = pos;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_current_position
 功能描述  : 获取当前位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_current_position(POSE_STRU &pos)
{
	pos = current_pos_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_goal_position
 功能描述  : 设置目标位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_goal_position(const POSE_STRU pos)
{
	goal_pos_ = pos;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_goal_position
 功能描述  : 获取目标位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_goal_position(POSE_STRU &pos)
{
	pos = goal_pos_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_recover_position
 功能描述  : 设置返回继续任务的位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_recover_position(const POSE_STRU pos)
{
	recover_pos_ = pos;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_recover_position
 功能描述  : 获取返回继续任务的位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_recover_position(POSE_STRU &pos)
{
	pos = recover_pos_;
}


/*****************************************************************************
 函 数 名: cfg_robot_data.set_event_position
 功能描述  : 设置发生事件时的位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_event_position(const POSE_STRU pos)
{
	event_pos_ = pos;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_event_position
 功能描述  : 获取发生事件时的位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_event_position(POSE_STRU &pos)
{
	pos = event_pos_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_bumper_state
 功能描述  : 设置碰撞传感器状态信息
 输入参数: BUMPER_ID_ENUM id  
           const bool state   
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_bumper_state(BUMPER_ID_ENUM id, const bool state)
{
	bumper_state_[id] = state;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_bumper_state
 功能描述  : 获取悬崖传感器状态信息
 输入参数: BUMPER_ID_ENUM id  
           bool &state        
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_bumper_state(BUMPER_ID_ENUM id, bool &state)
{
	state = bumper_state_[id];
}

/*****************************************************************************
 函 数 名: cfg_robot_data.convert_bumper_id
 功能描述  : 转化碰撞传感器编号
 输入参数: uint8_t num         
 输出参数: BUMPER_ID_ENUM &id  
 返 回 值: bool 正确执行返回真否则返回假
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_robot_data::convert_bumper_id(uint8_t num, BUMPER_ID_ENUM &id)
{
	bool ret = true;
	const uint8_t left_bumper = 0;
	const uint8_t right_bumper = 2;
	const uint8_t center_bumper = 1;

	if ( left_bumper == num )
	{
		id = LEFT_BUMPER;
	}
	else if ( right_bumper == num )
	{
		id = RIGHT_BUMPER;
	}
	else if ( center_bumper == num )
	{
		id = CENTER_BUMPER;
	}
	else
	{
		ret = false;
		debug_print_warnning("num = %d; id = %d", num, id);
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.convert_bumper_state
 功能描述  : 获取碰撞传感器的状态值
 输入参数: const uint8_t value  
 输出参数: bool state 真表示按下,假表示释放
 返 回 值: bool 正确执行返回真否则返回假
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_robot_data::convert_bumper_state(const uint8_t value, bool &state)
{
	bool ret = true;
	const uint8_t pressed = 1;
	const uint8_t released = 0;

	if ( pressed == value )
	{
		state = true;
	}
	else if ( released == value )
	{
		state = false;
	}
	else
	{
		debug_print_warnning("value = %d", value);
		ret = false;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_wheel_drop_state
 功能描述  : 设置轮子跌落传感器状态
 输入参数: WHEEL_ID_ENUM id  
           const bool state  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_wheel_drop_state(WHEEL_ID_ENUM id, const bool state)
{
	wheel_drop_state_[id] = state;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_wheel_drop_state
 功能描述  : 获取轮子跌落传感器状态
 输入参数: WHEEL_ID_ENUM id  
           bool &state       
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_wheel_drop_state(WHEEL_ID_ENUM id, bool &state)
{
	state = wheel_drop_state_[id];
}

/*****************************************************************************
 函 数 名: cfg_robot_data.convert_wheel_drop_id
 功能描述  : 转换为跌落传感器编号
 输入参数: uint8_t num        
 输出参数: WHEEL_ID_ENUM &id  
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_robot_data::convert_wheel_drop_id(uint8_t num, WHEEL_ID_ENUM &id)
{
	bool ret = true;
	const uint8_t left_wheel = 0;
	const uint8_t right_wheel = 1;
	
	if (left_wheel == num )
	{
		id = LEFT_WHEEL;
	}
	else if (right_wheel == num )
	{
		id = RIGHT_WHEEL;
	}
	else
	{
		debug_print_warnning("num = %d", num);
		ret = false;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.convert_wheel_drop_state
 功能描述  : 转换跌落状态
 输入参数: uint8_t value
           bool &state  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_robot_data::convert_wheel_drop_state(uint8_t value, bool &state)
{
	bool ret = true;
	const uint8_t normal = 0;
	const uint8_t abnormal = 1;

	if ( abnormal == value )
	{
		state = true;
	}
	else if ( normal == value )
	{
		state = false;
	}
	else
	{
		debug_print_warnning("value = %d", value);
		ret = false;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.test_wheel_sensor_is_normal
 功能描述  : 检测跌落传感器是否正常
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_robot_data::test_wheel_sensor_is_normal(void)
{
	bool ret = true;
	bool state = false;
	uint8_t i = 0;
	uint8_t sum = WHEEL_SUM;

	for (i = 0; i < sum; ++i)
	{
		get_wheel_drop_state((WHEEL_ID_ENUM)i, state);
		if ( true == state )
		{
			return false;
		}
	}

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_cliff_state
 功能描述  : 设置悬崖传感器状态信息
 输入参数: CLIFF_ID_ENUM id  
           const bool state  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_cliff_state(CLIFF_ID_ENUM id, const bool state)
{
	cliff_state_[id] = state;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_cliff_state
 功能描述  : 获取悬崖传感器状态信息
 输入参数: CLIFF_ID_ENUM id  
           bool &state       
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_cliff_state(CLIFF_ID_ENUM id, bool &state)
{
	state = cliff_state_[id];
}

/*****************************************************************************
 函 数 名: cfg_robot_data.convert_cliff_id
 功能描述  : 转换为悬崖传感器编号
 输入参数: uint8_t num        
 输出参数: CLIFF_ID_ENUM &id  
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_robot_data::convert_cliff_id(uint8_t num, CLIFF_ID_ENUM &id)
{
	bool ret = true;
	const uint8_t left_bumper = 0;
	const uint8_t right_bumper = 2;
	const uint8_t left_center_bumper = 1;
	const uint8_t right_center_bumper = 3;
	
	if (left_bumper == num)
	{
		id = LEFT_CLIFF;
	}
	else if (left_center_bumper == num)
	{
		id = LEFT_CENTER_CLIFF;
	}
	else if (right_center_bumper == num)
	{
		id = RIGHT_CENTER_CLIFF;
	}
	else if (right_bumper == num)
	{
		id = RIGHT_CLIFF;
	}
	else
	{
		ret = false;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.convert_cliff_state
 功能描述  : 转换悬崖传感器状态
 输入参数: uint8_t value  
           bool &state  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_robot_data::convert_cliff_state(uint8_t value, bool &state)
{
	bool ret = true;
	const uint8_t normal = 0;
	const uint8_t abnormal = 1;

	if ( abnormal == value )
	{
		state = true;
	}
	else if ( normal == value )
	{
		state = false;
	}
	else
	{
		debug_print_warnning("value = %d", value);
		ret = false;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.test_cliff_sensor_is_normal
 功能描述  : 检测悬崖传感器是否正常
 输入参数: void  
 输出参数: 无
 返 回 值: bool 任何一个悬崖传感器检测到悬崖则返回真，否则返回假
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_robot_data::test_cliff_sensor_is_normal(void)
{
	bool ret = true;
	bool state = false;
	uint8_t i = 0;
	uint8_t sum = CLIFF_SUM;

	for (i = 0; i < sum; ++i)
	{
		get_cliff_state((CLIFF_ID_ENUM)i, state);
		if ( true == state )
		{
			return false;
		}
	}

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_battery_state
 功能描述  : 设置电池状态
 输入参数: const BATTERY_STRU state  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_battery_state(const BATTERY_STRU state)
{
	charge_state_ = state;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_battery_state
 功能描述  : 获取电池状态
 输入参数: BATTERY_STRU &state  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_battery_state(BATTERY_STRU &state)
{
	state = charge_state_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_ultrasonic_sensor_state
 功能描述  : 设置传感器状态信息
 输入参数: const ULTRASONIC_SENSOR_STRU state  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_ultrasonic_sensor_state(const ULTRASONIC_SENSOR_STRU state)
{
	ultrasonic_sensor_ = state;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_ultrasonic_sensor_state
 功能描述  : 获取传感器状态信息
 输入参数: ULTRASONIC_SENSOR_STRU &state  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_ultrasonic_sensor_state(ULTRASONIC_SENSOR_STRU &state)
{
	state = ultrasonic_sensor_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_wall_following_sensor_state
 功能描述  : 设置沿墙传感器状态信息
 输入参数: const WALL_FOLLOWING_SENSOR_STRU state  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_wall_following_sensor_state(const WALL_FOLLOWING_SENSOR_STRU state)
{
	wall_following_sensor_ = state;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_wall_following_sensor_state
 功能描述  : 获取沿墙传感器状态信息
 输入参数: WALL_FOLLOWING_SENSOR_STRU &state  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_wall_following_sensor_state(WALL_FOLLOWING_SENSOR_STRU &state)
{
	state = wall_following_sensor_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_linear_velocity
 功能描述  : 设置线速度
 输入参数: double value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_linear_velocity(double value)
{
	linear_velocity_ = value;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_linear_velocity
 功能描述  : 获取线速度
 输入参数: double &value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_linear_velocity(double &value)
{
	value = linear_velocity_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_angular_velocity
 功能描述  : 设置角速度
 输入参数: double value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_angular_velocity(double value)
{
	angular_velocity_ = value;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_angular_velocity
 功能描述  : 获取角速度
 输入参数: double &value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_angular_velocity(double &value)
{
	value = angular_velocity_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_velocity
 功能描述  : 设置线速度和角速度
 输入参数: double velocity  
           double rad       
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_velocity(double velocity, double rad)
{
	linear_velocity_ = velocity;
	angular_velocity_ = rad;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_velocity
 功能描述  : 获取线速度和角速度
 输入参数: double &velocity  
           double &rad       
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::get_velocity(double &velocity, double &rad)
{
	velocity = linear_velocity_;
	rad = angular_velocity_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.set_adjust_velocity
 功能描述  : 设置是否进行速度调整
 输入参数: bool flag  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_robot_data::set_adjust_velocity(bool flag)
{
	adjust_velocity_ = flag;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_adjust_velocity
 功能描述  : 获取调整的速度
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_robot_data::get_adjust_velocity(void)
{
	return adjust_velocity_;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_curr_x_axis_coordinate
 功能描述  : 获取当前位置的X轴坐标
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_robot_data::get_curr_x_axis_coordinate(void)
{
	return current_pos_.point.x;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_curr_y_axis_coordinate
 功能描述  : 获取当前位置的Y轴坐标
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_robot_data::get_curr_y_axis_coordinate(void)
{
	return current_pos_.point.y;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.get_curr_pos_angle
 功能描述  : 获取当前位置的方向角
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_robot_data::get_curr_pos_angle(void)
{
	return current_pos_.angle;
}

/*****************************************************************************
 函 数 名: cfg_robot_data.test_robot_is_ok
 功能描述  : 检测机器人是否处于正常运行状态
 输入参数: void  
 输出参数: 无
 返 回 值: bool 是返回真，否则返回假
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_robot_data::test_robot_is_ok(void)
{
	bool ret = true;
	ret = test_wheel_sensor_is_normal();
	if (false == ret)
	{
		return false;
	}

	return ret;
}


/******************************************************************************
 * 内部函数声明
 ******************************************************************************/


