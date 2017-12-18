/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: cfg_mobile_robot.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年8月10日
  最近修改:
  功能描述: 
  函数列表:
  修改历史:
  1.日     期: 2017年8月10日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/
#ifndef __CFG_MOBILE_ROBOT_H__
#define __CFG_MOBILE_ROBOT_H__

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
#include <string>

#include <unistd.h>
#include <pthread.h>
#include <inttypes.h>
#include <sys/types.h>

#include "math.h"

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

#include "version.h"

#include "cfg_robot_data.h"
#include "cfg_walk_plan.h"
#include "cfg_modulate.h"

using namespace std;

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
//只能打开其中的某一个宏，对应采用不同的机型机型调试，因为物理属性差异角度
#define BLUEWAYS_DEBUG
//#define KOBUKI_DEBUG
//#define KOBUKI_SIMULATOR_DEBUG


/******************************************************************************
 * 常量声明
 ******************************************************************************/

/******************************************************************************
 * 枚举类型
 ******************************************************************************/
//移动机器人行动状态
typedef enum ACTION_STATUS_ID
{
	STOP = 0,                                  //停止运行
	GO_FORWARD,                                //向前运行
	GO_BACK,                                   //后退运行
	TURN_LEFT,                                 //左转
	TURN_RIGHT,                                //右转
	PIVOT,                                     //原地向后旋转180度
	TURN_BACK_CLOCKWISE,                       //顺时针向后转180度
	TURN_BACK_ANTICLOCKWISE,                   //逆时针向后转180度
	TURN_RIGHT_ANGLE_CLOCKWISE,                //原地顺时针旋转90度
	TURN_RIGHT_ANGLE_ANTICLOCKWISE,            //原地逆时针旋转90度
	EDGE_WAYS,                                 //沿边运行
	AUTO_DOCK,                                 //自动返程
	LOCAL_AREA,                                //区域覆盖运行
}ACTION_STATUS_ENUM;


typedef enum MONITOR
{
	MONITOR_SLEEP = 0,
	MONITOR_RUNNING,
	MONITOR_RESPOND,
}MONITOR_ENUM;

//typedef enum ROTATE_DIRECTION
//{
//	CLOCKWISE = 0,
//	ANTICLOCKWISE,
//}ROTATE_DIRECTION_ENUM;
	
typedef void (*PF_FUNC)(void);//定义一个函数指针类型 

typedef struct ANGLE_MONITOR
{
	double angle;                                            //检测角度
	MONITOR_ENUM stage;                                      //监测阶段状态
	//PF_FUNC pf_function;                                     //监测角度达标以后执行的动作
	
	ANGLE_MONITOR()  //默认构造函数
	{
		angle = 0.0;
		stage = MONITOR_SLEEP;
		//pf_function = NULL;
	}
}ANGLE_MONITOR_STRU;

/******************************************************************************
 * 结构体类型
 ******************************************************************************/

typedef struct ACTION_STATUS
{
	ACTION_STATUS_ENUM curr_action;
	ACTION_STATUS_ENUM last_action;
}ACTION_STATUS_STRU;

typedef struct PRIME_DIRECTION
{
	double forward;                            //执行弓形行走时开始行走的正向
	double reverse;                            //执行弓形行走时开始行走的反向
	bool inversion;                            //颠倒调转
	bool valid;                                //是否已保存有效的正反向数据标志
}PRIME_DIRECTION_STRU;


typedef struct REFERENCE_DATA
{
	PRIME_DIRECTION_STRU fst_dir;
}REFERENCE_DATA_STRU;


/******************************************************************************
 * 类声明
 ******************************************************************************/

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/

//移动机器人状态数据类 
class cfg_mobile_robot : public cfg_robot_data, public version, public cfg_walk_plan, public cfg_modulate
{
public:
	static cfg_mobile_robot* get_instance();

	void init(void);

	void straight_and_rotate_moving(double velocity, double rad);
	void straight_moving(double velocity);
	void rotate_moving(double rad);
	
	void shut_down(void);
	void power_on(void);
	
	void stop(void);
	void go_forward(void);
	void go_back(void);
	void turn_left(void);
	void turn_right(void);
	void pivot(void);
	void edge_ways(void);
	void auto_dock(void);
	void local_cover_movement(void);

	void turn_back_clockwise(void);
	void turn_back_anticlockwise(void);

	void turn_right_angle_clockwise(void);
	void turn_right_angle_anticlockwise(void);

	void set_action_status(const ACTION_STATUS_STRU status);
	void get_action_status(ACTION_STATUS_STRU &status);
	
	void set_curr_action(ACTION_STATUS_ENUM action);
	void get_curr_action(ACTION_STATUS_ENUM &action);
	
	void set_last_action(ACTION_STATUS_ENUM action);
	void get_last_action(ACTION_STATUS_ENUM &action);

	void save_running_status(void);
	void recover_running_status(void);
	
	void set_reference_data(const REFERENCE_DATA_STRU &data);
	void get_reference_data(REFERENCE_DATA_STRU &data);

	bool check_reference_data_valid(void);

	void set_reference_data_forward_angle(double data);
	bool get_reference_data_forward_angle(double &data);
	
	void set_reference_data_reverse_angle(double data);
	bool get_reference_data_reverse_angle(double &data);

	void set_reference_data_inversion(bool data);
	bool get_reference_data_inversion(void);

	ROTATE_DIRECTION_ENUM get_curr_rotate_direction(void);

	bool set_traight_line_moving(void);
	bool test_is_traight_line_moving(void);

	void get_rotate_action_type(ACTION_STATUS_ENUM &action);

	double get_driving_direction_right_angle(void);
	void get_turn_back_action_type(ACTION_STATUS_ENUM &action);
	void get_turn_right_angle_action_type(ACTION_STATUS_ENUM &action);

	double get_reference_angle(void);
	
	double get_turnt_to_reference_deriction_angle(void);
	void get_turnt_to_reference_deriction_action_type(ACTION_STATUS_ENUM &action);

	void save_first_line_refer_direction(void);
	
	void save_current_positions(double x, double y, double theta);

	void update_goal_positions(void);
	void update_ultrasonic_sensor_data (double value);
	void update_wall_following_sensor_data (double value);

	void upate_cliff_state(uint8_t id, uint8_t value);
	void upate_bumper_state(uint8_t id, uint8_t value);
	void upate_wheel_drop_state(uint8_t id, uint8_t value);

	void update_obstatcle_safety_level(void);
	void sensors_deal(void);

	//double format_angle(double angle);
	//double convert_degrees_to_radians(double degrees);
	//double convert_radians_to_degrees(double radians);
	//double convert_to_acute_angle(double angle);
	
	bool test_differences(double value, double reference, double precision);
	
	//double change_angle(double angle, double change_value);
	
	//double get_right_angle_clockwise(double angle);
	//double get_right_angle_anticlockwise(double angle);
	//double get_right_angle(double angle, ROTATE_DIRECTION_ENUM direction);
	//double get_reverse_angle(double angle);
	double get_curr_pose_angle(void);
	double get_curr_pose_reverse_angle(void);
	double get_curr_pose_right_angle(ROTATE_DIRECTION_ENUM direction);

	//bool test_angle_is_over_clockwise(double current, double target);
	//bool test_angle_is_over_anticlockwise(double current, double target);
	
	bool test_rotate_is_over_clockwise(void);
	bool test_rotate_is_over_anticlockwise(void);

	bool test_angle_is_over(double current, double target, double &offset);
	double get_curr_angle_difference_respond(void);

	void set_monitor_angle_data(const ANGLE_MONITOR_STRU data);
	void get_monitor_angle_data(ANGLE_MONITOR_STRU &data);
	
	void clear_monitor_angle_data(void);
	
	void set_monitor_angle_stage(const MONITOR_ENUM stage);
	void get_monitor_angle_stage(MONITOR_ENUM &stage);
	
	void set_monitor_angle_respond_goal(const double angle);
	double get_monitor_angle_respond_goal(void);
	
	void set_monitor_angle_respond_func(void(cfg_mobile_robot::*pf)(void));
	void get_monitor_angle_respond_func(void(cfg_mobile_robot::*pf)(void));
	
	void set_monitor_angle_respond_goal_and_func(double angle, void(cfg_mobile_robot::*pf)(void));
	

	double get_monitor_angle_turn_back_angle(void);
	void monitor_angle_turn_back_respond(void);
	void monitor_angle_set_turn_back_deal(void);

	void monitor_angle_pivot_respond(void);
	void monitor_angle_set_pivot_deal(void);

	void monitor_angle_turn_to_refer_line_respond(void);
	void monitor_angle_turn_to_refer_line_deal(void);

	void monitor_angle_turn_to_center_respond(void);
	void monitor_angle_turn_to_center_deal(void);

	double get_turnt_to_original_pose_angle(void);
	void get_turnt_to_original_pose_action_type(ACTION_STATUS_ENUM &action);
	void monitor_angle_turn_to_original_pose_respond(void);
	void monitor_angle_turn_to_original_pose_deal(void);

	double get_turnt_to_original_direction_angle(void);
	void get_turnt_to_original_direction_action_type(ACTION_STATUS_ENUM &action);
	void monitor_angle_turn_to_original_direction_respond(void);
	void monitor_angle_turn_to_original_direction_deal(void);

	void left_or_right_bumper_respond(void);
	void monitor_angle_left_bumper_respond(void);
	void monitor_angle_left_bumper_deal(void);
	void monitor_angle_right_bumper_respond(void);
	void monitor_angle_right_bumper_deal(void);

	bool test_adjust_velocity(double &linear_velocity, double &angular_velocity);
	
	void monitor_angle_adjust_velocity(void);
	void monitor_angle_running(void);
	void monitor_angle_respond(void);
	
	void monitor_stop_rotate(void);
	
	void smooth_decelerate_stop(void);
	
	void set_monitor_angle_rotate_call_back(double angle, void(cfg_mobile_robot::*pf_call_back)(void));
	

	void bumper_sensor_respond_deal(void);
	void cliff_sensor_respond_deal(void);
	void wheel_drop_sensor_respond_deal(void);
	void ultrasonic_sensor_respond_deal(void);

	void functional_mode(void);

	bool get_action_status_str(ACTION_STATUS_ENUM id, string& str);
	void print_curr_action_status_str(void);
	void print_change_action_status(void);
	void print_curr_pose_angle(void);

	void change_curr_action(const ACTION_STATUS_ENUM action);
	void change_rotate_direction(void);
	
	void do_retreat(void);

	void get_local_move_planning_pose_x(double &min, double &max);
	void get_local_move_planning_pose_y(double &min, double &max);

	void get_front_position(POSE_STRU &position);
	double get_vertical_distance_curr_position_to_refer_line(void);
	
	void set_current_position_to_refer_start_pose(void);
	void straight_moving_ajust_velocity(void);

	void straight_driving_adjust_angle(void);
	void straight_driving_adjust_speed(void);
	
	void update_traight_line_moving_data(void);

	double get_distance_to_start_point_pos(void);
	double get_distance_to_end_point_pos(void);
	double get_distance_to_traight_line_moving_start_pos(void);
	double get_distance_to_traight_line_moving_target_pos(void);

	double get_vertical_dimension_curr_pos_to_refer_line(void);

	double get_refer_line_start_pos_to_curr_pos_angle(void);
	double get_refer_line_end_pos_to_curr_pos_angle(void);
	double get_center_pose_to_curr_pos_angle(void);

	bool test_arrive_at_refer_line(void);
	void update_refer_line_traight_line_moving_target_pos(void);
	void traight_line_moving_dynamic_regulation(void);
	
	void local_move_start(void);
	void local_move_pivot(void);
	void local_move_fst_line_start(void);
	void local_move_return_reference_line(void);
	void local_move_return_center_pose(void);
	void local_move_fst_half_area(void);
	void local_move_sec_half_area(void);

	void do_local_move(void);
	void publish_trajectory(double x, double y, double th);
	void show_trajectory(void);

	bool test_detect_obstacle_turn_back(void);
	
	void timer_callback(const ros::TimerEvent& event);
	void retreat_callback(const ros::TimerEvent& event);
	
	void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
	void cliff_event_callback(const kobuki_msgs::CliffEvent &msg);
	void bumper_evnet_callback(const kobuki_msgs::BumperEvent &msg);
	void wheel_drop_event_callback(const kobuki_msgs::WheelDropEvent &msg);
	void ultrasonic_sensor_callback(const std_msgs::Int16& msg);
	void wall_following_sensor_callback(const std_msgs::Int16& msg);

	void laser_scan_callback( const sensor_msgs::LaserScan& msg );
	void velocity_callback( const geometry_msgs::Twist& msg );

	void register_msgs_callback(void);
	void function_processor( void );

	static pthread_mutex_t mutex;

	double odometer_horizontal;
	double odometer_vertical;

private:
	static cfg_mobile_robot* p_instance_;

	static constexpr double Linear_velocity_ = 0.25;               //线速度(m/s)
	static constexpr double Angular_velocity_ = 0.8;               //角速度(rad/s)
	static constexpr double Linear_velocity_clockwise_ = 0.08;//0.15;     //顺时针旋转返回线速度(m/s)
	static constexpr double Angular_velocity_clockwise_ = 1.0;//0.8;     //顺时针旋转返回角速度(rad/s)
	static constexpr double Diameter_ = 0.165;                     //机器人直径

	static constexpr double distance_precision_ = 0.01;            //距离精度
	static constexpr double angle_precision_ = 0.2;                //角度精度

	static constexpr double collide_adjusted_angle_ = 30.0;        //碰撞调节角度

	ros::Publisher pub_cmvl_;
	ros::Publisher pub_path_;
	
	ros::Subscriber position_sub_;
	ros::Subscriber cliff_sub_;
	ros::Subscriber bumper_sub_;
	ros::Subscriber wheel_drop_sub_;
	ros::Subscriber velocity_sub_;
	ros::Subscriber ultrasonic_sensor_sub_;
	ros::Subscriber wall_following_sensor_sub_;
	ros::Subscriber laser_scan_sub_;

	ros::Timer timer_;
	ros::Timer retreat_timer_;
	
	ACTION_STATUS_STRU action_;                                    //当前运动状态
	REFERENCE_DATA_STRU ref_data_;

	ANGLE_MONITOR_STRU angle_monitor_;                             //角度监测数据状态
	void(cfg_mobile_robot::*pf_monitor_)();

	cfg_mobile_robot();
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

#endif /* __CFG_MOBILE_ROBOT_H__ */
