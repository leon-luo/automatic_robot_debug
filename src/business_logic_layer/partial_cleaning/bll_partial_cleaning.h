/******************************************************************************

  版权所有 (C), 2017-2028, _ _ _ Co., Ltd.

 ******************************************************************************
  文件名称: bll_partial_cleaning.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月21日
  最近修改:
  功能描述: bll_partial_cleaning.cpp 的头文件
  函数列表:
  修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/
#ifndef __BLL_PARTIAL_CLEANING_H__
#define __BLL_PARTIAL_CLEANING_H__

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include <pthread.h>

#include "bll_base_type.h"


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
class bll_partial_cleaning
{
protected:
	bll_partial_cleaning();
	~bll_partial_cleaning();
	
public:
	static bll_partial_cleaning* get_instance(void);
	static void release_instance(void);

	void set_partial_cleaning_max_length(double data);
	
	void get_front_position(POSE_STRU &position);
	double get_vertical_distance_curr_position_to_refer_line(void);
	void set_current_position_to_refer_start_pose(void);
	double get_distance_to_start_point_pos(void);
	double get_distance_to_end_point_pos(void);
	double get_vertical_dimension_curr_pos_to_refer_line(void);
	double get_refer_line_start_pos_to_curr_pos_angle(void);

	double get_refer_line_end_pos_to_curr_pos_angle(void);
	double get_center_pose_to_curr_pos_angle(void);

	bool test_start_do_partial_cleaning(void);
	bool test_arrive_at_refer_line(void);
	bool test_detect_obstacle_turn_back(void);
	bool test_reach_refer_target_position(void);
	bool test_finish_half_area(void);
	bool test_finish_traight_line_moving(void);
	
	void update_refer_line_traight_line_moving_target_pos(void);
	void traight_line_moving_dynamic_regulation(void);

	void set_partial_cleaning_state(const PARTITION_DRIVING_STATE_ENUM data);
	void partition_driving_start(void);
	void partition_driving_pivot(void);
	void partition_driving_fst_line_start(void);
	void partition_driving_return_reference_line(void);
	void partition_driving_return_center_pose(void);
	bool partition_driving_parallel_lines(void);
	void partition_driving_fst_half_area(void);
	void partition_driving_sec_half_area(void);

	void partition_driving(void);
	void switch_partial_cleaning(void);
	void clear_local_cover_movement_data(void);
	void local_cover_movement(void);

	void smooth_decelerate_stop(void);

	double get_parallel_moving_direction_angle(void);
	double get_monitor_angle_turn_back_angle(void);
	void monitor_angle_set_turn_back_deal(void);

	void monitor_angle_set_pivot_deal(void);

	void monitor_angle_turn_to_refer_line_deal(void);
	
	void monitor_angle_turn_to_center_deal(void);

	double get_turnt_to_original_pose_angle(void);
	void get_turnt_to_original_pose_action_type(ACTION_STATUS_ENUM &action);
	void monitor_angle_turn_to_original_pose_deal(void);

	double get_turnt_to_original_direction_angle(void);
	void get_turnt_to_original_direction_action_type(ACTION_STATUS_ENUM &action);
	void monitor_angle_turn_to_original_direction_deal(void);

	void save_first_line_refer_direction(void);

	double get_partition_driving_edge_length(void);
	double get_partition_driving_half_edge_length(void);
	void change_rotate_direction(void);

	void updata_district_area(const POSE_STRU &data);

	void set_refer_line_start_point_pose(const POSE_STRU data);
	void get_refer_line_start_point_pose(POSE_STRU &data);
	
	void set_refer_line_end_point_pose(const POSE_STRU data);
	void get_refer_line_end_point_pose(POSE_STRU &data);

	double get_reference_angle(void);

	void set_reference_data(const REFERENCE_DATA_STRU &data);
	void get_reference_data(REFERENCE_DATA_STRU &data);

	bool check_reference_data_valid(void);

	void set_reference_data_forward_angle(double data);
	bool get_reference_data_forward_angle(double &data);
	
	void set_reference_data_reverse_angle(double data);
	bool get_reference_data_reverse_angle(double &data);

	void set_reference_data_inversion(bool data);
	bool get_reference_data_inversion(void);

	void set_reference_data_valid(bool valid);
	bool get_reference_data_valid(void);

	void get_rotate_action_type(ACTION_STATUS_ENUM &action);

	double get_driving_direction_right_angle(void);
	void get_turn_back_action_type(ACTION_STATUS_ENUM &action);
	void get_turn_right_angle_action_type(ACTION_STATUS_ENUM &action);
	
	double get_turnt_to_reference_deriction_angle(void);
	void get_turnt_to_reference_deriction_action_type(ACTION_STATUS_ENUM &action);

	void set_partial_cleaning_enable(bool data);
	bool get_partial_cleaning_enable(void);

private:
	bll_partial_cleaning(const bll_partial_cleaning&){};
	bll_partial_cleaning& operator=(const bll_partial_cleaning&){};

	bool partial_cleaning_enable_;
	double partial_cleaning_max_length_;//局域清扫边长(单位:m)

	POSE_STRU ref_start_point_;
	POSE_STRU ref_end_point_;

	REFERENCE_DATA_STRU ref_data_;
	
	static bll_partial_cleaning* p_instance_;
	static pthread_mutex_t mutex_;
};


/******************************************************************************
 * 内部函数声明
 ******************************************************************************/



/*****************************************************************************/
#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */
/*****************************************************************************/

/*****************************************************************************/
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */
/*****************************************************************************/

#endif /* __BLL_PARTIAL_CLEANING_H__ */
