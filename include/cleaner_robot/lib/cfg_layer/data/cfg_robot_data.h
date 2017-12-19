/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: cfg_robot_data.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年8月1日
  最近修改:
  功能描述: cfg_robot_data.cpp 的头文件
  函数列表:
  修改历史:
  1.日     期: 2017年8月1日
	作     者: Leon
	修改内容: 创建文件
******************************************************************************/
#ifndef __CFG_ROBOT_DATA_H__
#define __CFG_ROBOT_DATA_H__

/*****************************************************************************/
#ifdef __cplusplus
#if __cplusplus
//extern "C"{
#endif
#endif /* __cplusplus */
/*****************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include <iostream>
#include <unistd.h>
#include <pthread.h>

#include "cfg_base_type.h"

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
//左右轮子编号
typedef enum WHEEL_ID
{
	LEFT_WHEEL = 0,
	RIGHT_WHEEL,
	WHEEL_SUM,
}WHEEL_ID_ENUM;

//充电状态
typedef enum CHARGE_STATE
{
	CHARGE_SLEEP,//0：不充电状态
	CHARGED,//2：对接充电状态（charged）
	CHARGING,//6：对接充电状态（charging）
	ADAPTER_CHARGED,//18：适配器充电状态（charged）
	ADAPTER_CHARGING,//22：适配器充电状态（charging）
}CHARGE_STATE_ENUM;

typedef enum BUMPER_ID
{
	LEFT_BUMPER,
	CENTER_BUMPER,
	RIGHT_BUMPER,
	BUMPER_SUM,
	
}BUMPER_ID_ENUM;

typedef enum CLIFF_ID
{
	LEFT_CLIFF = 0,
	LEFT_CENTER_CLIFF,
	RIGHT_CENTER_CLIFF,
	RIGHT_CLIFF,
	CLIFF_SUM,
}CLIFF_ID_ENUM;

typedef enum SAFETY_LEVEL
{
	SAFETY_LEVEL_SAFE,
	SAFETY_LEVEL_WARN,
	SAFETY_LEVEL_FATAL,
}SAFETY_LEVEL_ENUM;

/******************************************************************************
 * 结构体类型
 ******************************************************************************/

//电池状态
typedef struct BATTERY
{
	double value;                                            //(电池电量)
	CHARGE_STATE_ENUM charge;                                //(充电状态)
}BATTERY_STRU;


//超生波传感器信息
typedef struct ULTRASONIC_SENSOR
{
	double value;                                            //检测距离
	double max;
	double min;
	bool enable;
	SAFETY_LEVEL_ENUM level;
}ULTRASONIC_SENSOR_STRU;

//沿墙传感器信息
typedef struct WALL_FOLLOWING_SENSOR
{
	double value;                                            //检测距离
	double max;
	double min;
	bool enable;
}WALL_FOLLOWING_SENSOR_STRU;


typedef struct VELOCITY_TWIST
{
	double x;
	double y;
	double z;
}VELOCITY_TWIST_STRU;

typedef struct TWIST
{
	VELOCITY_TWIST_STRU linear;
	VELOCITY_TWIST_STRU angular;
}TWIST_STRU;


/******************************************************************************
 * 类声明
 ******************************************************************************/
//移动机器人状态数据类
class cfg_robot_data
{
public:
	static cfg_robot_data* get_instance(void);

	void set_odometry_updated(bool flag);
	bool get_odometry_updated(void);

	void set_origin_position(const POSE_STRU pos);
	void get_origin_position(POSE_STRU &pos);

	void set_current_position(const POSE_STRU pos);
	void get_current_position(POSE_STRU &pos);

	void set_goal_position(const POSE_STRU pos);
	void get_goal_position(POSE_STRU &pos);

	void set_recover_position(const POSE_STRU pos);
	void get_recover_position(POSE_STRU &pos);

	void set_event_position(const POSE_STRU pos);
	void get_event_position(POSE_STRU &pos);

	void set_bumper_state(BUMPER_ID_ENUM id, const bool state);
	void get_bumper_state(BUMPER_ID_ENUM id, bool &state);
	bool convert_bumper_id(uint8_t num, BUMPER_ID_ENUM &id);
	bool convert_bumper_state(const uint8_t value, bool &state);

	void set_wheel_drop_state(WHEEL_ID_ENUM id, const bool state);
	void get_wheel_drop_state(WHEEL_ID_ENUM id, bool &state);
	bool convert_wheel_drop_id(uint8_t num, WHEEL_ID_ENUM &id);
	bool convert_wheel_drop_state(uint8_t value, bool &state);
	bool test_wheel_sensor_is_normal(void);

	void set_cliff_state(CLIFF_ID_ENUM id, const bool state);
	void get_cliff_state(CLIFF_ID_ENUM id, bool &state);
	bool convert_cliff_id(uint8_t num, CLIFF_ID_ENUM &id);
	bool convert_cliff_state(uint8_t value, bool &state);
	bool test_cliff_sensor_is_normal(void);
	
	void set_battery_state(const BATTERY_STRU state);
	void get_battery_state(BATTERY_STRU &state);

	void set_ultrasonic_sensor_state(const ULTRASONIC_SENSOR_STRU state);
	void get_ultrasonic_sensor_state(ULTRASONIC_SENSOR_STRU &state);

	void set_wall_following_sensor_state(const WALL_FOLLOWING_SENSOR_STRU state);
	void get_wall_following_sensor_state(WALL_FOLLOWING_SENSOR_STRU &state);

	void set_linear_velocity(double value);
	void get_linear_velocity(double &value);

	void set_angular_velocity(double value);
	void get_angular_velocity(double &value);

	void set_velocity(double velocity, double rad);
	void get_velocity(double &velocity, double &rad);

	void set_adjust_velocity(bool flag);
	bool get_adjust_velocity(void);

	double get_curr_x_axis_coordinate(void);
	double get_curr_y_axis_coordinate(void);
	double get_curr_pos_angle(void);

	bool test_robot_is_ok(void);
	
	static pthread_mutex_t mutex;
	cfg_robot_data();
private:
	static cfg_robot_data* p_instance_;
	
	bool odometry_updated_;                                    //里程计是否已经更新
	
	POSE_STRU origin_pos_;                                     //起始点位置
	POSE_STRU current_pos_;                                    //当前位置
	POSE_STRU goal_pos_;                                       //目标位置
	POSE_STRU recover_pos_;                                    //电量不足回充前保存当前位置
	POSE_STRU event_pos_;                                      //发生事件时的位置
	
	bool bumper_state_[3];                                     //减震器(右侧减震器;左侧减震器;中心减震器)
	bool wheel_drop_state_[2];                                 //轮跌落
	bool cliff_state_[3];                                      //防跌落

	bool button_state_[2];                                     //按键

	bool over_current_[2];                                     //左右轮检测到过流状态

	uint32_t wheel_encoder_[2];                                //左右轮编码器
	
	BATTERY_STRU charge_state_;                                //充电状态
	ULTRASONIC_SENSOR_STRU ultrasonic_sensor_;                 //超声波传感器状态
	WALL_FOLLOWING_SENSOR_STRU wall_following_sensor_;         //沿墙传感器状态

	double linear_velocity_;                                   //当前的线速度
	double angular_velocity_;                                  //当前的角速度
	bool adjust_velocity_;
};

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/



/*****************************************************************************/
#ifdef __cplusplus
#if __cplusplus
//}
#endif
#endif /* __cplusplus */
/*****************************************************************************/

#endif /* __CFG_ROBOT_DATA_H__ */
