/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: cfg_if_mobile_robot.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月26日
  最近修改:
  功能描述: cfg_if_mobile_robot.h 的头文件
  函数列表:
  修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/
#ifndef __CFG_IF_MOBILE_ROBOT_H__
#define __CFG_IF_MOBILE_ROBOT_H__
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
#include "cfg_mobile_robot.h"

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

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/
void cfg_if_print_change_action(void);

void cfg_if_set_current_position(const POSE_STRU pos);
void cfg_if_get_current_position(POSE_STRU &pos);

void cfg_if_save_current_position(double x, double y, double theta);
double cfg_if_get_current_position_angle(void);
double cfg_if_get_current_position_reverse_angle(void);

ACTION_STATUS_ENUM cfg_if_get_curr_action(void);
void cfg_if_set_curr_action(const ACTION_STATUS_ENUM action);
void cfg_if_change_curr_action(const ACTION_STATUS_ENUM action);

void cfg_if_save_running_status(void);
void cfg_if_recover_running_status(void);

void cfg_if_set_bumper_state(BUMPER_ID_ENUM id, const bool state);
void cfg_if_get_bumper_state(BUMPER_ID_ENUM id, bool &state);
bool cfg_if_convert_bumper_id(uint8_t num, BUMPER_ID_ENUM &id);
bool cfg_if_convert_bumper_state(const uint8_t value, bool &state);

void cfg_if_set_cliff_state(CLIFF_ID_ENUM id, const bool state);
void cfg_if_get_cliff_state(CLIFF_ID_ENUM id, bool &state);
bool cfg_if_convert_cliff_id(uint8_t num, CLIFF_ID_ENUM &id);
bool cfg_if_convert_cliff_state(uint8_t value, bool &state);
bool cfg_if_test_robot_is_ok(void);

void cfg_if_set_wheel_drop_state(WHEEL_ID_ENUM id, const bool state);
void cfg_if_get_wheel_drop_state(WHEEL_ID_ENUM id, bool &state);
bool cfg_if_convert_wheel_drop_id(uint8_t num, WHEEL_ID_ENUM &id);
bool cfg_if_convert_wheel_drop_state(uint8_t value, bool &state);
bool cfg_if_test_wheel_sensor_is_normal(void);

void cfg_if_set_ultrasonic_sensor(const ULTRASONIC_SENSOR_STRU data);
void cfg_if_get_ultrasonic_sensor(ULTRASONIC_SENSOR_STRU &data);

void cfg_if_set_wall_following_sensor(const WALL_FOLLOWING_SENSOR_STRU data);
void cfg_if_get_wall_following_sensor(WALL_FOLLOWING_SENSOR_STRU &data);

void cfg_if_switch_move_direction(void);
void cfg_if_switch_partial_cleaning_part(void);
bool cfg_if_test_move_direction_is_forward(void);
bool cfg_if_test_partial_cleaning_part_is_left(void);

void cfg_if_enable_partial_cleaning(void);
void cfg_if_disable_partial_cleaning(void);
bool cfg_if_test_partial_cleaning_is_enable(void);

void cfg_if_set_partial_cleaning_mode(const PARTITION_DRIVING_ENUM data);
void cfg_if_get_partial_cleaning_mode(PARTITION_DRIVING_ENUM &data);

void cfg_if_set_partial_cleaning_state(const PARTITION_DRIVING_STATE_ENUM data);
void cfg_if_get_partial_cleaning_state(PARTITION_DRIVING_STATE_ENUM &data);

void cfg_if_set_partial_cleaning_area_part(const AREA_PART_ENUM data);
void cfg_if_get_partial_cleaning_area_part(AREA_PART_ENUM &data);

void cfg_if_set_partial_cleaning_direction(const MOVE_DIRECTION_ENUM data);
void cfg_if_get_partial_cleaning_direction(MOVE_DIRECTION_ENUM &data);

void cfg_if_set_partial_cleaning_original_pose(const POSE_STRU &data);
void cfg_if_get_partial_cleaning_original_pose(POSE_STRU &data);

void cfg_if_set_velocity(double line_v, double angular_v);
void cfg_if_get_velocity(double &line_v, double &angular_v);

bool cfg_if_update_velocity(double &line_v, double &angular_v);
bool cfg_if_set_run_velocity(double &line_v, double &angular_v);

void cfg_if_set_partition_driving_planning_pose(const POSE_STRU &data, int index);
void cfg_if_get_partition_driving_planning_pose(POSE_STRU &data, int index);

void cfg_if_set_key_status(KEY_ID_ENUM id, KEY_STATUS_ENUM value);
bool cfg_if_get_key_single_click(KEY_ID_ENUM id);
bool cfg_if_get_key_double_click(KEY_ID_ENUM id);
bool cfg_if_get_key_long_click(KEY_ID_ENUM id);

/*****************************************************************************/
#ifdef __cplusplus
#if __cplusplus
//}
#endif
#endif /* __cplusplus */
/*****************************************************************************/

#endif /* __CFG_IF_MOBILE_ROBOT_H__ */
