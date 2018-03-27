/******************************************************************************

  版权所有 (C), 2017-2028 

 ******************************************************************************
  文件名称: bll_traight_line_moving.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月18日
  最近修改:
  功能描述: bll_traight_line_moving.cpp 的头文件
  函数列表:
  修改历史:
  1.日     期: 2017年12月18日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/
#ifndef __BLL_TRAIGHT_LINE_MOVING_H__
#define __BLL_TRAIGHT_LINE_MOVING_H__

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
typedef struct STRAIGHT_LINE_MOVING
{
	bool flag;                                         //直行行驶标志
	POSE_STRU start_pos;                                //行驶的起始点坐标
	POSE_STRU target_pos;                               //行驶的目标点坐标
	POSE_STRU current_pos;                              //行驶过程的当前实时位置坐标
	double direction;                                   //行驶方向(角度)
	double travel_distance;                             //已行驶距离
	bool all_route_done;                                //整个目标行程行驶完成的标志
	bool part_route_done;                               //整个目标行程行驶一部分提前完成的标志
}STRAIGHT_LINE_MOVING_STRU;

/******************************************************************************
 * 类声明
 ******************************************************************************/

class bll_traight_line_moving
{
protected:
	bll_traight_line_moving();
	~bll_traight_line_moving();
	
public:
	static bll_traight_line_moving* get_instance(void);
	static void release_instance(void);

	bool set_traight_line_moving(void);
	bool test_is_traight_line_moving(void);

	void update_traight_line_moving_data(void);
	
	void set_traight_line_moving_flag(const bool flag);
	bool get_traight_line_moving_flag(void);

	void set_traight_line_moving_start_pos(const POSE_STRU pos);
	void get_traight_line_moving_start_pos(POSE_STRU &pos);

	void set_traight_line_moving_target_pos(const POSE_STRU pos);
	void get_traight_line_moving_target_pos(POSE_STRU &pos);

	void set_traight_line_moving_current_pos(const POSE_STRU pos);
	void get_traight_line_moving_current_pos(POSE_STRU &pos);

	void set_traight_line_moving_direction(const double direction);
	double get_traight_line_moving_direction(void);

	void set_traight_line_moving_distance(const double distance);
	double get_traight_line_moving_distance(void);

	void set_finish_all_route_done(const bool flag);
	bool get_finish_all_route_done(void);

	void set_finish_part_route_done(const bool flag);
	bool get_finish_part_route_done(void);

	void set_traight_line_moving_data(const STRAIGHT_LINE_MOVING_STRU data);
	void get_traight_line_moving_data(STRAIGHT_LINE_MOVING_STRU &data);

	double get_traight_line_moving_direction_angle(void);
	void clear_traight_line_moving_data(void);

	void straight_driving_adjust_angle(void);
	void straight_driving_adjust_speed(void);

	double get_current_pos_to_target_pos_angle(void);

	double get_distance_to_traight_line_moving_start_pos(void);
	double get_distance_to_traight_line_moving_target_pos(void);

	void set_straight_moving_refer_start_pose(const POSE_STRU data);
	void get_straight_moving_refer_start_pose(POSE_STRU &data);

	void set_straight_moving_refer_target_pose(const POSE_STRU data);
	void get_straight_moving_refer_target_pose(POSE_STRU &data);

	void get_straight_moving_refer_pose(POSE_STRU &start,POSE_STRU &target);

private:
	bll_traight_line_moving(const bll_traight_line_moving&){};             //禁止拷贝
	bll_traight_line_moving& operator=(const bll_traight_line_moving&){};  //禁止赋值

	STRAIGHT_LINE_MOVING_STRU straight_line_moving_;

	POSE_STRU start_pose_;
	POSE_STRU target_pose_;

	static bll_traight_line_moving* p_instance_;
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

#endif /* __BLL_TRAIGHT_LINE_MOVING_H__ */
