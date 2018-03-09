/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: drv_sensor.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月19日
  最近修改:
  功能描述   : 传感器相关功能类定义
  函数列表:
  修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "drv_sensor.h"

#include "tf/LinearMath/Matrix3x3.h"

#include "key_base.h"

#include "bll_cliff.h"
#include "bll_bumper.h"
#include "bll_wheel_drop.h"
#include "bll_ultrasonic.h"
#include "bll_wall_following.h"

#include "bit_base.h"
#include "time_base.h"

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
 * 类声明
 ******************************************************************************/
drv_sensor* drv_sensor::p_instance_ = nullptr;
pthread_mutex_t drv_sensor::mutex_ = PTHREAD_MUTEX_INITIALIZER;

/*****************************************************************************
 函 数 名: drv_sensor.drv_sensor
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: drv_sensor
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
drv_sensor::drv_sensor()
{

}

/*****************************************************************************
 函 数 名: drv_sensor.~drv_sensor
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: drv_sensor
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
drv_sensor::~drv_sensor()
{

}

/*****************************************************************************
 函 数 名: drv_sensor.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: drv_sensor*
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
drv_sensor* drv_sensor::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr == p_instance_)
		{
			p_instance_ = new drv_sensor();
		}
		pthread_mutex_unlock(&mutex_);
	}
	return p_instance_;
}

/*****************************************************************************
 函 数 名: drv_sensor.release_instance
 功能描述  : 释放实例
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void drv_sensor::release_instance(void)
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
 函 数 名: drv_sensor.initialize
 功能描述  : 初始化
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void drv_sensor::initialize(void)
{
	register_sensor_msgs_callback();
}

/*****************************************************************************
 函 数 名: drv_sensor.set_run_velocity
 功能描述  : 设置行驶速度
 输入参数: double line_velocity     
           double angular_velocity  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void drv_sensor::set_run_velocity(double line_velocity, double angular_velocity)
{
	geometry_msgs::Twist twist;
	twist.angular.z = angular_velocity;
	twist.linear.x = line_velocity;
	pub_cmvl_.publish(twist);
}

/*****************************************************************************
 函 数 名: drv_sensor.odometry_callback
 功能描述  : 里程计信息更新
 输入参数: const nav_msgs::Odometry::ConstPtr& msg
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void drv_sensor::odometry_callback ( const nav_msgs::Odometry::ConstPtr& msg )
{
	double yaw = 0.0;
	double roll = 0.0;
	double pitch = 0.0;
	double yaw_degree = 0.0;
	geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
	tf::Matrix3x3 mat ( tf::Quaternion ( orientation.x, orientation.y, orientation.z, orientation.w ) );
	mat.getEulerYPR ( yaw, pitch, roll );

	POSE_STRU pos;
	pos.point.x = msg->pose.pose.position.x;
	pos.point.y = msg->pose.pose.position.y;
	pos.angle = yaw;

	cfg_if_save_current_position(pos.point.x, pos.point.y, pos.angle);
}

/*****************************************************************************
 函 数 名: drv_sensor.cliff_event_callback
 功能描述  : 悬崖传感器检测回调
 输入参数: const kobuki_msgs::CliffEvent& msg  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月3日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void drv_sensor::cliff_event_callback ( const kobuki_msgs::CliffEvent& msg )
{
	/*
	# Provides a cliff sensor event.
	# This message is generated whenever a particular cliff sensor signals that the
	# robot approaches or moves away from a cliff.
	# Note that, despite cliff field on SensorState messages, state field is not a
	# bitmask, but the new state of a single sensor.

	# cliff sensor
	uint8_t LEFT   = 0
	uint8_t CENTER = 1
	uint8_t RIGHT  = 2

	# cliff sensor state
	uint8_t FLOOR = 0
	uint8_t CLIFF = 1

	uint8_t sensor
	uint8_t state

	# distance to floor when cliff was detected
	uint16 bottom
	*/
	uint8_t cliff_id = msg.sensor;
	uint8_t cliff_state = msg.state;

	bll_cliff* p_cliff = bll_cliff::get_instance();
	p_cliff->upate_cliff_state(cliff_id, cliff_state);
}

/*****************************************************************************
 函 数 名: drv_sensor.bumper_evnet_callback
 功能描述  : 碰撞回调函数
 输入参数: const kobuki_msgs::BumperEvent& msg  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月3日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void drv_sensor::bumper_evnet_callback ( const kobuki_msgs::BumperEvent& msg )
{
	/*消息类型如下
	# Provides a bumper event.
	# This message is generated whenever a particular bumper is pressed or released.
	# Note that, despite bumper field on SensorState messages, state field is not a
	# bitmask, but the new state of a single sensor.

	# bumper
	uint8_t LEFT   = 0
	uint8_t CENTER = 1
	uint8_t RIGHT  = 2

	# state
	uint8_t RELEASED = 0
	uint8_t PRESSED  = 1

	uint8_t bumper
	uint8_t state
	*/
	uint8_t bumper_id = msg.bumper;
	uint8_t state = msg.state;

	bll_bumper* p_bumper = bll_bumper::get_instance();
	p_bumper->upate_bumper_state(bumper_id, state);
}

/*****************************************************************************
 函 数 名: drv_sensor.wheel_drop_event_callback
 功能描述  : 轮子离开地面事件回调函数
 输入参数: const kobuki_msgs::WheelDropEvent &msg
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void drv_sensor::wheel_drop_event_callback ( const kobuki_msgs::WheelDropEvent& msg )
{
	/*消息类型如下：
	# Provides a wheel drop event.
	# This message is generated whenever one of the wheels is dropped (robot fell
	# or was raised) or raised (normal condition).
	# Note that, despite wheel_drop field on SensorState messages, state field is
	# not a bitmask, but the new state of a single sensor.

	# wheel
	uint8_t LEFT  = 0
	uint8_t RIGHT = 1

	# state
	uint8_t RAISED  = 0
	uint8_t DROPPED = 1

	uint8_t wheel
	uint8_t state
	*/
	uint8_t wheel_id = msg.wheel;
	uint8_t wheel_state = msg.state;

	bll_wheel_drop* p_wheel_drop = bll_wheel_drop::get_instance();
	p_wheel_drop->update_wheel_drop_state(wheel_id, wheel_state);
}

/*****************************************************************************
 函 数 名: drv_sensor.ultrasonic_sensor_callback
 功能描述  : 超声波传感器回调函数
 输入参数: const std_msgs::Int16& msg  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月3日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void drv_sensor::ultrasonic_sensor_callback ( const std_msgs::Int16& msg )
{
	double value = ( double ) msg.data;

	bll_ultrasoic* p_ultrasoic = bll_ultrasoic::get_instance();
	p_ultrasoic->update_ultrasonic_sensor_data (value);
}

/*****************************************************************************
 函 数 名: wall_following_sensor_callback
 功能描述  : 沿墙传感器回调函数
 输入参数: const std_msgs::Int16& msg  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年12月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void drv_sensor::wall_following_sensor_callback ( const std_msgs::Int16& msg )
{
	double value = ( double ) msg.data;

	bll_wall_following* p_wall_following = bll_wall_following::get_instance();
	p_wall_following->update_wall_following_sensor_data (value);
}

/*****************************************************************************
 函 数 名: laser_scan_callback
 功能描述  : 激光扫描传感器消息回调函数
 输入参数: const sensor_msgs::LaserScan& msg  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年8月18日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void drv_sensor::laser_scan_callback( const sensor_msgs::LaserScan& msg )
{
/*
leon@ubuntu:~$ rostopic type /scan 
sensor_msgs/LaserScan
leon@ubuntu:~$ rosmsg show sensor_msgs/LaserScan 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min           # 开始扫描的角度(角度)
float32 angle_max           # 结束扫描的角度(角度)
float32 angle_increment     # 每一次扫描增加的角度(角度)
float32 time_increment      # 测量的时间间隔(s)
float32 scan_time           # 扫描的时间间隔(s)
float32 range_min           # 距离最小值(m)
float32 range_max           # 距离最大值(m)
float32[] ranges            # 距离数组(长度360)
float32[] intensities       # 与设备有关,强度数组(长度360)
leon@ubuntu:~$ 
*/
	float angle_min = msg.angle_min;
	float angle_max = msg.angle_max;
	float angle_increment = msg.angle_increment;
	float time_increment = msg.time_increment;
	float scan_time = msg.scan_time;
	float range_min = msg.range_min;
	float range_max = msg.range_max;
//	uint8_t ranges_length = msg.ranges_length;
//	float st_ranges = msg.st_ranges;
	std::vector<float> ranges = msg.ranges;
//	uint8_t intensities_length = msg.intensities_length;
//	float st_intensities = msg.st_intensities;
	std::vector<float> intensities = msg.intensities;

	cout<<"angle_min:"<<angle_min<<endl;
	cout<<"angle_max:"<<angle_max<<endl;
	cout<<"angle_increment:"<<angle_increment<<endl;
	
	cout<<"time_increment:"<<time_increment<<endl;
	cout<<"scan_time:"<<scan_time<<endl;
	
	cout<<"range_min:"<<range_min<<endl;
	cout<<"range_max:"<<range_max<<endl;
//	cout<<"ranges_length:"<<ranges_length<<endl;
//	cout<<"st_ranges:"<<st_ranges<<endl;
	cout<<"ranges:"<<ranges.size()<<endl;
	
//	cout<<"intensities_length:"<<intensities_length<<endl;
//	cout<<"st_intensities:"<<st_intensities<<endl;
	cout<<"intensities:"<<intensities.size()<<endl;
}

/*****************************************************************************
 函 数 名: drv_sensor.velocity_callback
 功能描述  : 速度检测回调函数
 输入参数: const geometry_msgs::Twist& msg  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月3日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void drv_sensor::velocity_callback ( const geometry_msgs::Twist& msg )
{

}

/*****************************************************************************
 函 数 名: drv_sensor.home_key_callback
 功能描述  : home按键检测回调函数
 输入参数: const std_msgs::Int16& msg  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2018年2月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void drv_sensor::home_key_callback(const std_msgs::Int16& msg)
{
	int16_t value = ( int16_t ) msg.data;
	static uint64_t start_time = 0;
	uint64_t end_time = 0;
	uint64_t keep_time = 0;
	static bool flag = false;
	const uint8_t short_press_bit = 0;
	const uint8_t long_press_bit = 1;
	const KEY_STATUS_ENUM key_press = KEY_PRESSED;
	const KEY_STATUS_ENUM key_release = KEY_RELEASED;
	KEY_STATUS_ENUM short_press_status = key_release;
	KEY_STATUS_ENUM long_press_status = key_release;

	if (1 == GET_BIT(value, short_press_bit))
	{
		short_press_status = key_press;
	}
	
	if (1 == GET_BIT(value, long_press_bit))
	{
		long_press_status = key_press;
	}
	cfg_if_set_key_status(HOME_KEY_ID, short_press_status);
//	cout<<"home_key:"<<value<<endl;
//	cout<<"long_press_status:"<<long_press_status<<endl;
//	cout<<"short_press_status:"<<short_press_status<<endl<<endl;
//
//	if (short_press_status == key_press)
//	{
//		if(false == flag)
//		{
//			start_time = get_millisecond_time();
//			flag = true;
//		}
//	}
//	else if (short_press_status == key_release)
//	{
//		if(true == flag)
//		{
//			end_time = get_millisecond_time();
//
//			keep_time = end_time - start_time;
//			if (keep_time > 100)
//			{
//				cout<<"keep_time:"<<keep_time<<endl;
//			}
//			else if (keep_time > 5000)
//			{
//				cout<<"keep_time:"<<keep_time<<endl;
//			}
//			flag = false;
//			start_time = 0;
//		}
//	}
}

/*****************************************************************************
 函 数 名: drv_sensor.register_msgs_callback
 功能描述  : 注册消息处理相关回调函数
 输入参数: void
 输出参数: 无
 返 回 值: void

 修改历史:
  1.日     期: 2017年8月2日
    作     者: Leon
    修改内容: 新生成函数
firefly@firefly:~$ rostopic list
/capability_server/bonds
/capability_server/events
/cmd_vel_mux/active
/cmd_vel_mux/input/navi
/cmd_vel_mux/input/safety_controller
/cmd_vel_mux/input/switch
/cmd_vel_mux/input/teleop
/cmd_vel_mux/parameter_descriptions
/cmd_vel_mux/parameter_updates
/diagnostics
/diagnostics_agg
/diagnostics_toplevel_state
/gateway/force_update
/gateway/gateway_info
/info
/interactions/interactive_clients
/interactions/pairing
/joint_states
/mobile_base/commands/controller_info
/mobile_base/commands/digital_output
/mobile_base/commands/external_power
/mobile_base/commands/led1
/mobile_base/commands/led2
/mobile_base/commands/motor_power
/mobile_base/commands/reset_odometry
/mobile_base/commands/sound
/mobile_base/commands/velocity
/mobile_base/controller_info
/mobile_base/debug/raw_control_command
/mobile_base/debug/raw_data_command
/mobile_base/debug/raw_data_stream
/mobile_base/events/bumper
/mobile_base/events/button
/mobile_base/events/cliff
/mobile_base/events/digital_input
/mobile_base/events/power_system
/mobile_base/events/robot_state
/mobile_base/events/wheel_drop
/mobile_base/sensors/bumper_pointcloud
/mobile_base/sensors/core
/mobile_base/sensors/dock_ir
/mobile_base/sensors/dock_us
/mobile_base/sensors/edge_ir
/mobile_base/sensors/imu_data
/mobile_base/sensors/imu_data_raw
/mobile_base/version_info
/mobile_base_nodelet_manager/bond
/odom
/rosout
/rosout_agg
/scan
/tf
/tf_static
/trajectory
/turtlebot/incompatible_rapp_list
/turtlebot/rapp_list
/turtlebot/status
/zeroconf/lost_connections
/zeroconf/new_connections

*****************************************************************************/
void drv_sensor::register_sensor_msgs_callback(void)
{
	ros::NodeHandle node_h;
	//发布
	pub_cmvl_ = node_h.advertise<geometry_msgs::Twist> ( "/mobile_base/commands/velocity", 10 );

	//订阅
	position_sub_ = node_h.subscribe ( "/odom", 10, &drv_sensor::odometry_callback, p_instance_);
	cliff_sub_ = node_h.subscribe ( "/mobile_base/events/cliff", 10, &drv_sensor::cliff_event_callback, p_instance_);
	bumper_sub_ = node_h.subscribe ( "/mobile_base/events/bumper", 10, &drv_sensor::bumper_evnet_callback, p_instance_);
	wheel_drop_sub_ = node_h.subscribe ( "/mobile_base/events/wheel_drop", 10, &drv_sensor::wheel_drop_event_callback, p_instance_);
	velocity_sub_ = node_h.subscribe ( "/mobile_base/commands/velocity", 100, &drv_sensor::velocity_callback, p_instance_);
	ultrasonic_sensor_sub_ = node_h.subscribe ( "/mobile_base/sensors/dock_us", 100, &drv_sensor::ultrasonic_sensor_callback, p_instance_);
	wall_following_sensor_sub_ = node_h.subscribe ( "/mobile_base/sensors/edge_ir", 100, &drv_sensor::wall_following_sensor_callback, p_instance_);
	laser_scan_sub_ = node_h.subscribe ( "/scan", 100, &drv_sensor::laser_scan_callback, p_instance_);

	home_kye_sub_ = node_h.subscribe ( "/mobile_base/events/home_key", 10, &drv_sensor::home_key_callback, p_instance_);
}

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/


