/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: cfg_if_mobile_robot.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月26日
  最近修改:
  功能描述   :  
  函数列表:
  修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "cfg_if_mobile_robot.h"

#include "drv_sensor.h"

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

/******************************************************************************
 * 内部函数定义
 ******************************************************************************/
/*****************************************************************************
 函 数 名: cfg_if_print_change_action
 功能描述  : 打印改变的运动状态
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年1月3日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_print_change_action(void)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->print_change_action_status();
}

/*****************************************************************************
 函 数 名: cfg_if_set_current_position
 功能描述  : 设置当前位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_current_position(const POSE_STRU pos)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_current_position(pos);
}

/*****************************************************************************
 函 数 名: cfg_if_get_current_position
 功能描述  : 获取当前位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_current_position(POSE_STRU &pos)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_current_position(pos);
}

/*****************************************************************************
 函 数 名: cfg_if_save_current_position
 功能描述  : 保存当前位置状态
 输入参数: double x      
           double y      
           double theta  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_save_current_position(double x, double y, double theta)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->save_current_positions(x, y, theta);
}

/*****************************************************************************
 函 数 名: cfg_if_get_current_position_angle
 功能描述  : 获取当前角度
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_if_get_current_position_angle(void)
{
	double ret = 0.0;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->get_curr_pose_angle();

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_get_current_position_reverse_angle
 功能描述  : 获取当前位置状态下的反向角度
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_if_get_current_position_reverse_angle(void)
{
	double ret = 0.0;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_curr_pose_reverse_angle();
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_get_curr_action
 功能描述  : 获取当前运行状态
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
ACTION_STATUS_ENUM cfg_if_get_curr_action(void)
{
	ACTION_STATUS_ENUM action;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_curr_action(action);
	
	return action;
}

/*****************************************************************************
 函 数 名: cfg_if_set_curr_action
 功能描述  : 设置当前的运动行为模式
 输入参数: const ACTION_STATUS_ENUM action  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_curr_action(const ACTION_STATUS_ENUM action)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_curr_action(action);
}

/*****************************************************************************
 函 数 名: cfg_if_change_curr_action
 功能描述  : 更改当前的运动行为模式
 输入参数: const ACTION_STATUS_ENUM action  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_change_curr_action(const ACTION_STATUS_ENUM action)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->change_curr_action(action);
}

/*****************************************************************************
 函 数 名: cfg_if_save_running_status
 功能描述  : 保存运行状态数据
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_save_running_status(void)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->save_running_status();
}

/*****************************************************************************
 函 数 名: cfg_if_recover_running_status
 功能描述  : 恢复运行状态数据
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_recover_running_status(void)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->recover_running_status();
}

/*****************************************************************************
 函 数 名: cfg_if_set_bumper_state
 功能描述  : 设置碰撞传感器状态信息
 输入参数: BUMPER_ID_ENUM id  
           const bool state   
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_bumper_state(BUMPER_ID_ENUM id, const bool state)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_bumper_state(id, state);
}

/*****************************************************************************
 函 数 名: cfg_if_get_bumper_state
 功能描述  : 获取悬崖传感器状态信息
 输入参数: BUMPER_ID_ENUM id  
           bool &state        
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_bumper_state(BUMPER_ID_ENUM id, bool &state)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_bumper_state(id, state);
}

/*****************************************************************************
 函 数 名: cfg_if_convert_bumper_id
 功能描述  : 转化碰撞传感器编号
 输入参数: uint8_t num         
 输出参数: BUMPER_ID_ENUM &id  
 返 回 值: bool 正确执行返回真否则返回假
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_convert_bumper_id(uint8_t num, BUMPER_ID_ENUM &id)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	return p_mobile_robot->convert_bumper_id(num, id);
}

/*****************************************************************************
 函 数 名: cfg_if_convert_bumper_state
 功能描述  : 获取碰撞传感器的状态值
 输入参数: const uint8_t value  
 输出参数: bool state 真表示按下,假表示释放
 返 回 值: bool 正确执行返回真否则返回假
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_convert_bumper_state(const uint8_t value, bool &state)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	return p_mobile_robot->convert_bumper_state(value, state);
}

/*****************************************************************************
 函 数 名: cfg_if_set_cliff_state
 功能描述  : 设置悬崖传感器状态信息
 输入参数: CLIFF_ID_ENUM id  
           const bool state  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_cliff_state(CLIFF_ID_ENUM id, const bool state)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_cliff_state(id, state);
}

/*****************************************************************************
 函 数 名: cfg_if_get_cliff_state
 功能描述  : 获取悬崖传感器状态信息
 输入参数: CLIFF_ID_ENUM id  
           bool &state       
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_cliff_state(CLIFF_ID_ENUM id, bool &state)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_cliff_state(id, state);
}

/*****************************************************************************
 函 数 名: cfg_if_convert_cliff_id
 功能描述  : 转换为悬崖传感器编号
 输入参数: uint8_t num        
           CLIFF_ID_ENUM &id  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_convert_cliff_id(uint8_t num, CLIFF_ID_ENUM &id)
{
	bool ret = false;
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->convert_cliff_id(num, id);
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_convert_cliff_state
 功能描述  : 转换悬崖传感器状态
 输入参数: uint8_t value  
           bool &state    
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_convert_cliff_state(uint8_t value, bool &state)
{
	bool ret = false;
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->convert_cliff_state(value, state);
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_test_robot_is_ok
 功能描述  : 检测机器人是否处于正常运行状态
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_test_robot_is_ok(void)
{
	bool ret = false;
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->test_robot_is_ok();
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_set_wheel_drop_state
 功能描述  : 设置轮子跌落传感器状态
 输入参数: WHEEL_ID_ENUM id  
           const bool state  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_wheel_drop_state(WHEEL_ID_ENUM id, const bool state)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_wheel_drop_state(id, state);
}

/*****************************************************************************
 函 数 名: cfg_if_get_wheel_drop_state
 功能描述  : 获取轮子跌落传感器状态
 输入参数: WHEEL_ID_ENUM id  
           bool &state       
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_wheel_drop_state(WHEEL_ID_ENUM id, bool &state)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_wheel_drop_state(id, state);
}

/*****************************************************************************
 函 数 名: cfg_if_convert_wheel_drop_id
 功能描述  : 转换为跌落传感器编号
 输入参数: uint8_t num        
           WHEEL_ID_ENUM &id  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_convert_wheel_drop_id(uint8_t num, WHEEL_ID_ENUM &id)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->convert_wheel_drop_id(num, id);

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_convert_wheel_drop_state
 功能描述  : 转换跌落状态
 输入参数: uint8_t value  
           bool &state    
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_convert_wheel_drop_state(uint8_t value, bool &state)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->convert_wheel_drop_state(value, state);
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_test_wheel_sensor_is_normal
 功能描述  : 检测跌落传感器是否正常
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_test_wheel_sensor_is_normal(void)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->test_wheel_sensor_is_normal();

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_set_ultrasonic_sensor
 功能描述  : 设置超声波传感器状态
 输入参数: const ULTRASONIC_SENSOR_STRU data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_ultrasonic_sensor(const ULTRASONIC_SENSOR_STRU data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_ultrasonic_sensor_state(data);
}

/*****************************************************************************
 函 数 名: cfg_if_get_ultrasonic_sensor
 功能描述  : 获取超声波传感器状态
 输入参数: ULTRASONIC_SENSOR_STRU &data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_ultrasonic_sensor(ULTRASONIC_SENSOR_STRU &data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_ultrasonic_sensor_state(data);
}

/*****************************************************************************
 函 数 名: cfg_if_set_wall_following_sensor
 功能描述  : 设置沿墙传感器状态
 输入参数: const WALL_FOLLOWING_SENSOR_STRU data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_wall_following_sensor(const WALL_FOLLOWING_SENSOR_STRU data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_wall_following_sensor_state(data);
}

/*****************************************************************************
 函 数 名: cfg_if_get_wall_following_sensor
 功能描述  : 获取沿墙传感器状态
 输入参数: WALL_FOLLOWING_SENSOR_STRU &data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_wall_following_sensor(WALL_FOLLOWING_SENSOR_STRU &data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_wall_following_sensor_state(data);
}

/*****************************************************************************
 函 数 名: cfg_if_switch_move_direction
 功能描述  : 切换当前保存的运行方向
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_switch_move_direction(void)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->switch_partition_driving_direction();
}

/*****************************************************************************
 函 数 名: cfg_if_switch_partial_cleaning_part
 功能描述  : 切换局部清扫当前所在的半区局域
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_switch_partial_cleaning_part(void)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->switch_partition_driving_area_part();
}

/*****************************************************************************
 函 数 名: cfg_if_test_move_direction_is_forward
 功能描述  : 检测当前运行方向是否为向前
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_test_move_direction_is_forward(void)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->test_partition_driving_direction_is_forward();
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_test_partial_cleaning_part_is_left
 功能描述  : 检测是否在左边半区局域
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_test_partial_cleaning_part_is_left(void)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->test_partition_driving_area_part_is_left();
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_enable_partial_cleaning
 功能描述  : 使能局域清扫功能
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_enable_partial_cleaning(void)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->enable_local_move();
}

/*****************************************************************************
 函 数 名: cfg_if_disable_partial_cleaning
 功能描述  : 禁用局域清扫功能
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_disable_partial_cleaning(void)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->disable_local_move();
}

/*****************************************************************************
 函 数 名: cfg_if_test_partial_cleaning_is_enable
 功能描述  : 检测局域清扫功能是否开启
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_test_partial_cleaning_is_enable(void)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->test_partition_driving_is_enable();
}

/*****************************************************************************
 函 数 名: cfg_if_set_partial_cleaning_mode
 功能描述  : 设置局部清扫模式
 输入参数: const PARTITION_DRIVING_ENUM data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_partial_cleaning_mode(const PARTITION_DRIVING_ENUM data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_partition_driving_mode(data);
}

/*****************************************************************************
 函 数 名: cfg_if_get_partial_cleaning_mode
 功能描述  : 获取局部清扫模式
 输入参数: PARTITION_DRIVING_ENUM &data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_partial_cleaning_mode(PARTITION_DRIVING_ENUM &data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_partition_driving_mode(data);
}

/*****************************************************************************
 函 数 名: cfg_if_set_partial_cleaning_state
 功能描述  : 设置局部清扫阶段
 输入参数: const PARTITION_DRIVING_STATE_ENUM data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_partial_cleaning_state(const PARTITION_DRIVING_STATE_ENUM data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_partition_driving_state(data);
}

/*****************************************************************************
 函 数 名: cfg_if_get_partial_cleaning_state
 功能描述  : 获取局部清扫阶段
 输入参数: PARTITION_DRIVING_STATE_ENUM &data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_partial_cleaning_state(PARTITION_DRIVING_STATE_ENUM &data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_partition_driving_state(data);
}

/*****************************************************************************
 函 数 名: cfg_if_set_partial_cleaning_area_part
 功能描述  : 设置局部清除半区
 输入参数: const AREA_PART_ENUM data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_partial_cleaning_area_part(const AREA_PART_ENUM data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_partition_driving_area_part(data);
}

/*****************************************************************************
 函 数 名: cfg_if_get_partial_cleaning_area_part
 功能描述  : 获取局部清除半区
 输入参数: AREA_PART_ENUM &data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_partial_cleaning_area_part(AREA_PART_ENUM &data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_partition_driving_area_part(data);
}

/*****************************************************************************
 函 数 名: cfg_if_set_partial_cleaning_direction
 功能描述  : 设置局部清扫行走方向
 输入参数: const MOVE_DIRECTION_ENUM data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_partial_cleaning_direction(const MOVE_DIRECTION_ENUM data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_partition_driving_direction(data);
}

/*****************************************************************************
 函 数 名: cfg_if_get_partial_cleaning_direction
 功能描述  : 获取局部清扫行走方向
 输入参数: MOVE_DIRECTION_ENUM &data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_partial_cleaning_direction(MOVE_DIRECTION_ENUM &data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_partition_driving_direction(data);
}

/*****************************************************************************
 函 数 名: cfg_if_set_partial_cleaning_original_pose
 功能描述  : 设置局部清扫的起始原始点位置
 输入参数: const POSE_STRU &data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_partial_cleaning_original_pose(const POSE_STRU &data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_partition_driving_original_pose(data);
}

/*****************************************************************************
 函 数 名: cfg_if_get_partial_cleaning_original_pose
 功能描述  : 获取局部清扫的起始原始点位置
 输入参数: POSE_STRU &data  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_partial_cleaning_original_pose(POSE_STRU &data)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_partition_driving_original_pose(data);
}

/*****************************************************************************
 函 数 名: ccfg_if_set_velocity
 功能描述  : 获取线速度和角速度
 输入参数: double &line_v  
           double &angular_v 
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_velocity(double line_v, double angular_v)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_velocity(line_v, angular_v);
}

/*****************************************************************************
 函 数 名: ccfg_if_get_velocity
 功能描述  : 获取线速度和角速度
 输入参数: 
 输出参数: double &line_v  
           double &angular_v
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_velocity(double &line_v, double &angular_v)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_velocity(line_v, angular_v);
}

/*****************************************************************************
 函 数 名: cfg_if_update_velocity
 功能描述  : 更新速度
 输入参数: double &line_v     
           double &angular_v  
 输出参数: double &line_v     
           double &angular_v  
 返 回 值: bool 更新则返回真，否则返回假
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_update_velocity(double &line_v, double &angular_v)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	return p_mobile_robot->update_velocity(line_v, angular_v);
}

/*****************************************************************************
 函 数 名: cfg_if_set_run_velocity
 功能描述  : 配置运行速度
 输入参数: double &line_v     
           double &angular_v  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_set_run_velocity(double &line_v, double &angular_v)
{
	drv_sensor* p_drv_instance = drv_sensor::get_instance();
	p_drv_instance->set_run_velocity(line_v, angular_v);
}

/*****************************************************************************
 函 数 名: cfg_if_set_partition_driving_planning_pose
 功能描述  : 设置局域行驶区域方框角的位置
 输入参数: const POSE_STRU &data  
           int index              
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_partition_driving_planning_pose(const POSE_STRU &data, int index)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_partition_driving_planning_pose(data, index);
}

/*****************************************************************************
 函 数 名: cfg_if_get_partition_driving_planning_pose
 功能描述  : 获取局域行驶区域方框角的位置
 输入参数: POSE_STRU &data  
           int index        
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_partition_driving_planning_pose(POSE_STRU &data, int index)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_partition_driving_planning_pose(data, index);
}

