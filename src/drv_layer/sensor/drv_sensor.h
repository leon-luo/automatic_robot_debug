/******************************************************************************

  版权所有 (C), 2017-2028, _ _ _ Co., Ltd.

 ******************************************************************************
  文件名称: drv_sensor.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月19日
  最近修改:
  功能描述: drv_sensor.cpp 的头文件
  函数列表:
  修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/
#ifndef __DRV_SENSOR_H__
#define __DRV_SENSOR_H__

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
#include "base_type.h"

#include <pthread.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>

#include "key_data.h"

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
 * 类声明
 ******************************************************************************/

class drv_sensor
{
protected:
	drv_sensor();
	~drv_sensor();

public:
	static drv_sensor* get_instance(void);
	static void release_instance(void);

	void initialize(void);
	void set_run_velocity(double line_velocity, double angular_velocity);
	
private:
	void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
	void cliff_event_callback(const kobuki_msgs::CliffEvent &msg);
	void bumper_evnet_callback(const kobuki_msgs::BumperEvent &msg);
	void wheel_drop_event_callback(const kobuki_msgs::WheelDropEvent &msg);
	void ultrasonic_sensor_callback(const std_msgs::Int16& msg);
	void wall_following_sensor_callback(const std_msgs::Int16& msg);

	void laser_scan_callback( const sensor_msgs::LaserScan& msg );
	void velocity_callback( const geometry_msgs::Twist& msg );

	void set_key_status(KEY_ID_ENUM id, uint8_t bit, int16_t value);
	void key_callback(const std_msgs::Int16& msg);
	

	void register_sensor_msgs_callback(void);

	drv_sensor(const drv_sensor&){};
	drv_sensor& operator=(const drv_sensor&){};

	ros::Publisher pub_cmvl_;
	
	ros::Subscriber position_sub_;
	ros::Subscriber cliff_sub_;
	ros::Subscriber bumper_sub_;
	ros::Subscriber wheel_drop_sub_;
	ros::Subscriber velocity_sub_;
	ros::Subscriber ultrasonic_sensor_sub_;
	ros::Subscriber wall_following_sensor_sub_;
	ros::Subscriber laser_scan_sub_;
	ros::Subscriber home_kye_sub_;
	
	static drv_sensor* p_instance_;
	static pthread_mutex_t mutex_;
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

#endif /* __DRV_SENSOR_H__ */
