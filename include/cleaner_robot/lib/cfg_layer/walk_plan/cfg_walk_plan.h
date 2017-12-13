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


typedef enum LOCAL_MOVE_STATE
{
	LOCAL_MOVE_STOP,//0
	LOCAL_MOVE_START,
	LOCAL_MOVE_PIVOT,
	LOCAL_MOVE_FST_LINE_START,
	LOCAL_MOVE_FST_HALF,
	LOCAL_MOVE_FST_HALF_DONE,//5
	LOCAL_MOVE_RETURN_REFER_LINE,
	LOCAL_MOVE_SEC_HALF_START,
	LOCAL_MOVE_SEC_HALF,
	LOCAL_MOVE_SEC_HALF_DONE,
	LOCAL_MOVE_TURN_TO_CENTER_DIR,//10
	LOCAL_MOVE_RETURN_CENTER_POS,
	LOCAL_MOVE_TURN_TO_ORIGINAL_DIR,
	LOCAL_MOVE_ALL_DONE,
}LOCAL_MOVE_STATE_ENUM;

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
	double reference_x;
	double reference_y;
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
		bool test_local_move_area_part_is_right(void);
		void switch_local_move_area_part(void);

		void set_local_move_direction(const MOVE_DIRECTION_ENUM data);
		void get_local_move_direction(MOVE_DIRECTION_ENUM &data);
		bool test_local_move_direction_is_forward(void);
		bool test_local_move_direction_is_backward(void);
		void switch_local_move_direction(void);
		void print_local_move_direction(void);

		void set_local_move_original_pose(const POSE_STRU data);
		void get_local_move_original_pose(POSE_STRU &data);

		void set_local_move_planning_pose(const POSE_STRU &data, int index);
		void get_local_move_planning_pose(POSE_STRU &data, int index);
		
		double get_local_move_edge_length(void);
		double get_local_move_half_edge_length(void);
		
		void set_map_frame_area(const AREA_STRU data);
		void get_map_frame_area(AREA_STRU &data);

		void set_district_area(const AREA_STRU data);
		void get_district_area(AREA_STRU &data);

		void set_district_area_reference_x(const double data);
		void get_district_area_reference_x(double &data);

		void set_district_area_reference_y(const double data);
		void get_district_area_reference_y(double &data);

		void set_district_area_reference_angle(const double data);
		void get_district_area_reference_angle(double &data);
		
		void get_district_area_precision(double &data);

		void updata_district_area(const POSE_STRU &data);

		void set_refer_line_start_point_pose(const POSE_STRU data);
		void get_refer_line_start_point_pose(POSE_STRU &data);
		
		void set_refer_line_end_point_pose(const POSE_STRU data);
		void get_refer_line_end_point_pose(POSE_STRU &data);

		void set_straight_moving_refer_start_pose(const POSE_STRU data);
		void get_straight_moving_refer_start_pose(POSE_STRU &data);

		void set_straight_moving_refer_target_pose(const POSE_STRU data);
		void get_straight_moving_refer_target_pose(POSE_STRU &data);

		void get_straight_moving_refer_pose(POSE_STRU &start,POSE_STRU &target);

	private:
		static pthread_mutex_t mutex_;
		static cfg_walk_plan* p_instance_;
		static constexpr double local_erea_edge_length_ = 1.4;//1.6;2.0;//      //局域清扫边长(单位:m)
		
		AREA_STRU map_frame_;
		AREA_STRU district_;

		POSE_STRU ref_start_point_;
		POSE_STRU ref_end_point_;
		
		POSE_STRU start_pose_;
		POSE_STRU target_pose_;
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
