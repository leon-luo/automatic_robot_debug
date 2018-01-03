/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: cfg_walk_plan.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年8月22日
  最近修改:
  功能描述: cfg_walk_plan.cpp 的头文件
  函数列表:
  修改历史:
  1.日     期: 2017年8月22日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/
#ifndef __CFG_WALK_PLAN_H__
#define __CFG_WALK_PLAN_H__

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

/******************************************************************************
 * 结构体类型
 ******************************************************************************/

typedef enum LOCAL_MOVE_MODE
{
	AMBIENT_NO_OBSTACLE,
	AMBIENT_HAS_OBSTACLE,
}LOCAL_MOVE_MODE_ENUM;

typedef enum MOVE_DIRECTION
{
	FORWARD,                                 //向前行走
	BACKWARD,                                //向前行走
}MOVE_DIRECTION_ENUM;

typedef enum AREA_PART
{
	AREA_PART_LEFT,                          //左边区域
	AREA_PART_RIGHT,                         //右边区域
}AREA_PART_ENUM;

typedef struct AREA
{
	bool enable;                              //功能开启状态
	double max_x;                             //局域最大X轴向距离
	double max_y;                             //局域最大Y轴向距离
	POSE_STRU original_pose;                  //局域原始规划原始点
	POSE_STRU planning_pose[4];               //根据起始点规划出局域块四个角的点
	LOCAL_MOVE_MODE_ENUM mode;                //局域模式.以四周一定范围内有无障碍物来判断
	LOCAL_MOVE_STATE_ENUM state;              //局域行驶状态
	AREA_PART_ENUM area_part;                 //当前所在的半区
	MOVE_DIRECTION_ENUM direction;            //行驶方向
	double reference_angle;
	double precision;
}AREA_STRU;

/******************************************************************************
 * 类声明
 ******************************************************************************/
class cfg_walk_plan
{
	public:
		static cfg_walk_plan* get_instance(void);
		cfg_walk_plan();
		~cfg_walk_plan();

		void enable_local_move(void);
		void disable_local_move(void);
		bool test_local_move_is_enable(void);

		void set_local_move_mode(const LOCAL_MOVE_MODE_ENUM data);
		void get_local_move_mode(LOCAL_MOVE_MODE_ENUM &data);

		void set_local_move_state(const LOCAL_MOVE_STATE_ENUM data);
		void get_local_move_state(LOCAL_MOVE_STATE_ENUM &data);
		
		void set_local_move_area_part(const AREA_PART_ENUM data);
		void get_local_move_area_part(AREA_PART_ENUM &data);
		bool test_local_move_area_part_is_left(void);
		void switch_local_move_area_part(void);

		void set_local_move_direction(const MOVE_DIRECTION_ENUM data);
		void get_local_move_direction(MOVE_DIRECTION_ENUM &data);
		
		bool test_local_move_direction_is_forward(void);

		void switch_local_move_direction(void);
		void print_local_move_direction(void);

		void set_local_move_original_pose(const POSE_STRU data);
		void get_local_move_original_pose(POSE_STRU &data);

		void set_local_move_planning_pose(const POSE_STRU &data, int index);
		void get_local_move_planning_pose(POSE_STRU &data, int index);

		void set_district_area(const AREA_STRU data);
		void get_district_area(AREA_STRU &data);

		void set_district_area_reference_angle(const double data);
		void get_district_area_reference_angle(double &data);
		
		void get_district_area_precision(double &data);

	private:
		static pthread_mutex_t mutex_;
		static cfg_walk_plan* p_instance_;
		
		AREA_STRU district_;
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

#endif /* __CFG_WALK_PLAN_H__ */
