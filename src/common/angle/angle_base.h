/******************************************************************************

  版权所有 (C), 2017-2028, _ _ _ Co., Ltd.

 ******************************************************************************
  文件名称: angle_base.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月14日
  最近修改:
  功能描述: angle_base.cpp 的头文件
  函数列表:
  修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/
#ifndef __ANGLE_BASE_H__
#define __ANGLE_BASE_H__

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
typedef enum QUADRANT
{
	QUADRANT_1,
	QUADRANT_2,
	QUADRANT_3,
	QUADRANT_4,
}QUADRANT_ENUM;

typedef enum ROTATE_DIRECTION
{
	CLOCKWISE = 0,
	ANTICLOCKWISE,
}ROTATE_DIRECTION_ENUM;

/******************************************************************************
 * 结构体类型
 ******************************************************************************/


/******************************************************************************
 * 类声明
 ******************************************************************************/
class angel_base
{
public:
	angel_base();
	~angel_base();

	double convert_degrees_to_radians(double degrees);
	double convert_radians_to_degrees(double radians);
	
	double convert_to_acute_angle(double angle);
	
	double format_angle(double angle);
	
	double get_angle(double x, double y);
	double get_angle(double x1, double y1, double x2, double y2);
	
	QUADRANT_ENUM get_quadrant(double angle);
	
	double change_angle(double angle, double change_value);
	double get_reverse_angle(double angle);
	double get_right_angle_clockwise(double angle);
	double get_right_angle_anticlockwise(double angle);
	double get_right_angle(double angle, ROTATE_DIRECTION_ENUM direction);
	
	double get_angle_differences(double angle1, double angle2);

	bool is_acute_angle(double angle);
	bool is_obtuse_angle(double angle);
	
	bool test_angle_is_over_clockwise(double current, double target);
	bool test_angle_is_over_anticlockwise(double current, double target);
	bool test_angle_rotate_direction_is_clockwise(double start, double target);
	
protected:

private:
	angel_base(const angel_base&){};             //禁止拷贝
	angel_base& operator=(const angel_base&){};  //禁止赋值
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

#endif /* __ANGLE_BASE_H__ */
