/******************************************************************************

  版权所有 (C), 2017-2028 

 ******************************************************************************
  文件名称: bll_rotate.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月18日
  最近修改:
  功能描述: bll_rotate.cpp 的头文件
  函数列表:
  修改历史:
  1.日     期: 2017年12月18日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/
#ifndef __BLL_ROTATE_H__
#define __BLL_ROTATE_H__

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
#include <pthread.h>
#include "angle_base.h"

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
typedef enum MONITOR
{
	MONITOR_SLEEP = 0,
	MONITOR_RUNNING,
	MONITOR_RESPOND,
}MONITOR_ENUM;

/******************************************************************************
 * 结构体类型
 ******************************************************************************/
typedef struct ANGLE_MONITOR
{
	double angle;                                            //检测角度
	MONITOR_ENUM stage;                                      //监测阶段状态
	
	ANGLE_MONITOR()  //默认构造函数
	{
		angle = 0.0;
		stage = MONITOR_SLEEP;
	}
}ANGLE_MONITOR_STRU;

/******************************************************************************
 * 类声明
 ******************************************************************************/

class bll_rotate
{
protected:
	bll_rotate();
	~bll_rotate();

public:
	static bll_rotate* get_instance(void);
	static void release_instance(void);
	
	void set_monitor_angle_data(const ANGLE_MONITOR_STRU data);
	void get_monitor_angle_data(ANGLE_MONITOR_STRU &data);
	
	void clear_monitor_angle_data(void);
	
	void set_monitor_angle_stage(const MONITOR_ENUM stage);
	void get_monitor_angle_stage(MONITOR_ENUM &stage);

	bool test_angle_monitor_is_no_working(void);
	
	void set_monitor_angle_respond_goal(const double angle);
	double get_monitor_angle_respond_goal(void);
	
	void set_monitor_angle_respond_func(void(bll_rotate::*pf)(void));
	void get_monitor_angle_respond_func(void(bll_rotate::*pf)(void));
	
	void set_monitor_angle_respond_goal_and_func(double angle, void(bll_rotate::*pf)(void));
	void set_monitor_angle_rotate_call_back(double angle, void(bll_rotate::*pf_call_back)(void));
	
	bool test_adjust_velocity(double &linear_velocity, double &angular_velocity);


	void monitor_stop_rotate(void);

	void monitor_angle_adjust_velocity(void);
	void monitor_angle_running(void);
	void monitor_angle_respond(void);
	void rotate_operation(void);

	void left_or_right_bumper_respond(void);
	void monitor_angle_right_bumper_respond(void);
	void monitor_angle_left_bumper_respond(void);
	void monitor_angle_turn_back_respond(void);
	void monitor_angle_pivot_respond(void);
	void monitor_angle_turn_to_refer_line_respond(void);
	void monitor_angle_turn_to_center_respond(void);
	void monitor_angle_turn_to_original_pose_respond(void);
	void monitor_angle_turn_to_original_direction_respond(void);

	bool test_rotate_is_over_clockwise(void);
	bool test_rotate_is_over_anticlockwise(void);
	bool test_angle_is_over(double current, double target, double &offset);
	double get_curr_angle_difference_respond(void);

	ROTATE_DIRECTION_ENUM get_curr_rotate_direction(void);

private:
	bll_rotate(const bll_rotate&){};             //禁止拷贝
	bll_rotate& operator=(const bll_rotate&){};  //禁止赋值

	ANGLE_MONITOR_STRU angle_monitor_;                             //角度监测数据状态
	void(bll_rotate::*pf_monitor_)();

	static bll_rotate* p_instance_;
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

#endif /* __BLL_ROTATE_H__ */
