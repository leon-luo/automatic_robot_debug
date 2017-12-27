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

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include "version.h"
#include "angle_base.h"

#include "bll_traight_line_moving.h"

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
class cfg_mobile_robot : public cfg_robot_data, public version, public cfg_walk_plan, public bll_traight_line_moving
{
public:
	static cfg_mobile_robot* get_instance();

	void initialize(void);

	void set_action_status(const ACTION_STATUS_STRU status);
	void get_action_status(ACTION_STATUS_STRU &status);
	
	void set_curr_action(ACTION_STATUS_ENUM action);
	void get_curr_action(ACTION_STATUS_ENUM &action);
	
	void set_last_action(ACTION_STATUS_ENUM action);
	void get_last_action(ACTION_STATUS_ENUM &action);

	void save_running_status(void);
	void recover_running_status(void);


	void save_current_positions(double x, double y, double theta);

	double get_curr_pose_angle(void);
	double get_curr_pose_reverse_angle(void);
	double get_curr_pose_right_angle(ROTATE_DIRECTION_ENUM direction);

	bool get_action_status_str(ACTION_STATUS_ENUM id, string& str);
	void print_curr_action_status_str(void);
	void print_change_action_status(void);
	void print_curr_pose_angle(void);

	void change_curr_action(const ACTION_STATUS_ENUM action);

	void get_front_position(POSE_STRU &position);
	double get_vertical_distance_curr_position_to_refer_line(void);
	
	static pthread_mutex_t mutex;

private:
	static cfg_mobile_robot* p_instance_;

	static constexpr double Linear_velocity_ = 0.25;               //线速度(m/s)
	static constexpr double Angular_velocity_ = 0.8;               //角速度(rad/s)
	static constexpr double Linear_velocity_clockwise_ = 0.08;//0.15;     //顺时针旋转返回线速度(m/s)
	static constexpr double Angular_velocity_clockwise_ = 1.0;//0.8;     //顺时针旋转返回角速度(rad/s)
	static constexpr double Diameter_ = 0.165;                     //机器人直径

	static constexpr double distance_precision_ = 0.01;            //距离精度
	static constexpr double angle_precision_ = 0.2;                //角度精度

	ACTION_STATUS_STRU action_;                                    //当前运动状态

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
