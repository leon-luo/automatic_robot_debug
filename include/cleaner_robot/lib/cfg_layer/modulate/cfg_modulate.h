/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: cfg_modulate.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年11月9日
  最近修改:
  功能描述: cfg_modulate.cpp 的头文件
  函数列表:
  修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/
#ifndef __CFG_MODULATE_H__
#define __CFG_MODULATE_H__

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
#include <inttypes.h>

#include "cfg_base_type.h"
#include "pid.h"

#include "angle_base.h"

//using namespace std;

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
#define ENABLE_MOUDLATE
/******************************************************************************
 * 常量声明
 ******************************************************************************/

/******************************************************************************
 * 枚举类型
 ******************************************************************************/

/******************************************************************************
 * 结构体类型
 ******************************************************************************/
typedef struct VELOCITY_AJUST
{
	bool angular_flag;
	bool linear_flag;
	double angular_velocity;
	double linear_velocity;
}VELOCITY_AJUST_STRU;

typedef struct STRAIGHT_LINE_MOVING
{
	bool flag;
	POSE_STRU start_pos;
	POSE_STRU target_pos;
	POSE_STRU current_pos;
}STRAIGHT_LINE_MOVING_STRU;

/******************************************************************************
 * 类声明
 ******************************************************************************/

class cfg_modulate : public angel_base
{
public:
	static cfg_modulate* get_instance(void);
	static void release_instance(void);

	void set_velocity_ajust(const VELOCITY_AJUST_STRU data);
	void get_velocity_ajust(VELOCITY_AJUST_STRU &data);
	
	void set_angular_velocity_flag(bool data);
	bool get_angular_velocity_flag(void);
	
	void set_angular_velocity_value(double value);
	double get_angular_velocity_value(void);
	
	void set_angular_velocity_ajust(bool flag, double value);
	bool get_angular_velocity_ajust(double &value);

	void set_linear_velocity_ajust(bool flag, double value);
	bool get_linear_velocity_ajust(double &value);
	
	void disable_angular_velocity_ajust(void);
	void endble_angular_velocity_ajust(double velocity);

	void disable_linear_velocity_ajust(void);
	void endble_linear_velocity_ajust(double velocity);

	double get_distance(double x1, double y1, double x2, double y2);

	void set_traight_line_moving_flag(const bool flag);
	bool get_traight_line_moving_flag(void);

	void set_traight_line_moving_start_pos(const POSE_STRU pos);
	void get_traight_line_moving_start_pos(POSE_STRU &pos);

	void set_traight_line_moving_target_pos(const POSE_STRU pos);
	void get_traight_line_moving_target_pos(POSE_STRU &pos);

	void set_traight_line_moving_current_pos(const POSE_STRU pos);
	void get_traight_line_moving_current_pos(POSE_STRU &pos);
	
	void set_traight_line_moving_data(const STRAIGHT_LINE_MOVING_STRU data);
	void get_traight_line_moving_data(STRAIGHT_LINE_MOVING_STRU &data);

	double get_traight_line_moving_direction_angle(void);
	void clear_traight_line_moving_data(void);
	
protected:
	cfg_modulate();
	~cfg_modulate();

private:
	cfg_modulate(const cfg_modulate&){};                              //禁止拷贝
	cfg_modulate& operator=(const cfg_modulate&){};                   //禁止赋值
	
	static cfg_modulate* volatile p_instance_;
	static pthread_mutex_t mutex_;
	
	VELOCITY_AJUST_STRU velocity_ajust_;

	STRAIGHT_LINE_MOVING_STRU straight_line_moving_;

	pid angular_velocity_pid_;
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

#endif /* __CFG_MODULATE_H__ */
