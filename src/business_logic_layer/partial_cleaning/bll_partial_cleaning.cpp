/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: bll_partial_cleaning.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月21日
  最近修改:
  功能描述   : 局部清扫类定义
  函数列表:
  修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "bll_partial_cleaning.h"

#include "line_base.h"
#include "debug_function.h"
#include "cfg_if_modulate.h"
#include "cfg_if_mobile_robot.h"



/******************************************************************************
 * 外部变量定义
 ******************************************************************************/

/******************************************************************************
 * 外部函数定义
 ******************************************************************************/

/******************************************************************************
 * 全局变量
 ******************************************************************************/

/******************************************************************************
 * 宏定义
 ******************************************************************************/

/******************************************************************************
 * 常量定义
 ******************************************************************************/

/******************************************************************************
 * 枚举类型
 ******************************************************************************/

/******************************************************************************
 * 结构体类型
 ******************************************************************************/

/******************************************************************************
 * 类定义
 ******************************************************************************/
pthread_mutex_t bll_partial_cleaning::mutex_;
bll_partial_cleaning* bll_partial_cleaning::p_instance_ = nullptr;

/*****************************************************************************
 函 数 名: bll_partial_cleaning.bll_partial_cleaning
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_partial_cleaning
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_partial_cleaning::bll_partial_cleaning()
{
	REFERENCE_DATA_STRU ref_data = {{0.0, false}};
	set_reference_data(ref_data);
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.~bll_partial_cleaning
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_partial_cleaning
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_partial_cleaning::~bll_partial_cleaning()
{

}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: bll_partial_cleaning*
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_partial_cleaning* bll_partial_cleaning::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr == p_instance_)
		{
			p_instance_ = new bll_partial_cleaning();
		}
		pthread_mutex_unlock(&mutex_);
	}
	return p_instance_;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.release_instance
 功能描述  : 释放实例
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::release_instance(void)
{
	if (nullptr != p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr != p_instance_)
		{
			delete p_instance_;
			p_instance_ = nullptr;
		}
		pthread_mutex_unlock(&mutex_);
	}
}


/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_front_position
 功能描述  : 获取当前驶向的参考线两头哪一个方向的点
 输入参数: POSE_STRU &position  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月17日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::get_front_position(POSE_STRU &position)
{
	bool direction_is_forward = false;

	direction_is_forward = cfg_if_test_move_direction_is_forward();
	if (true == direction_is_forward)
	{
		get_refer_line_end_point_pose(position);
		//debug_print_info("{ end.pos  (%lf, %lf) : %lf}", position.point.x, position.point.y, position.angle);
	}
	else
	{
		get_refer_line_start_point_pose(position);
		//debug_print_info("{ start.pos (%lf, %lf) : %lf}", position.point.x, position.point.y, position.angle);
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_vertical_distance_curr_position_to_refer_line
 功能描述  : 获取当前点到参考线的垂直距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月17日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_vertical_distance_curr_position_to_refer_line(void)
{
#if 1
	POSE_STRU curr;
	POSE_STRU point1;
	POSE_STRU point2;
	double ret = 0.0;
	double x = 0.0;
	double y = 0.0;
	double x1 = 0.0;
	double y1 = 0.0;
	double x2 = 0.0;
	double y2 = 0.0;
	
	cfg_if_get_current_position(curr);
	cfg_if_get_partial_cleaning_original_pose(point1);
	get_refer_line_start_point_pose(point2);
	x = curr.point.x;
	y = curr.point.y;
	x1 = point1.point.x;
	y1 = point1.point.y;
	x2 = point2.point.x;
	y2 = point2.point.y;
	line_base line_base_instance;
	ret = line_base_instance.get_vertical_distance(x, y, x1, y1, x2, y2);
	return ret;
#else
	POSE_STRU curr;
	POSE_STRU original;
	double temp = 0.0;
	double radian = 0.0;
	double angle = 0.0;
	double diff_angle = 0.0;
	double acute_angle = 0.0;
	double refer_angle = 0.0;
	double opposite = 0.0;    //三角形对边
	double hypotenuse = 0.0;  //三角形斜边
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	cfg_if_get_current_position(curr);
	get_local_move_original_pose(original);
	//printf("{curr.pos (%lf, %lf) : %lf}\n", curr.point.x, curr.point.y, curr.angle);
	//printf("{original.pos (%lf, %lf) : %lf}\n", original.point.x, original.point.y, original.angle);
	line_base line_base_instance;
	hypotenuse = line_base_instance.get_distance(original.point.x, original.point.y, curr.point.x, curr.point.y);
	refer_angle = get_reference_angle();
	angel_base angel_base_instance;
	angle = angel_base_instance.get_angle(original.point.x, original.point.y, curr.point.x, curr.point.y);
	diff_angle = angel_base_instance.get_angle_differences(angle, refer_angle);
	//printf("diff_angle = get_angle_differences(angle=%lf, refer_angle=%lf) = %lf\n", angle, refer_angle, diff_angle);
	acute_angle = angel_base_instance.convert_to_acute_angle(diff_angle);
	radian = angel_base_instance.convert_degrees_to_radians(acute_angle);
	temp = sin(radian);
	opposite = hypotenuse*temp;
	//printf("hypotenuse=%lf; acute_angle=%lf; sin(radian=%lf)=%lf; opposite=%lf\n", hypotenuse, acute_angle, radian, temp, opposite);
	return opposite;
#endif
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.set_current_position_to_refer_start_pose
 功能描述  : 将当前位置保存为直线行走的起始点位置
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::set_current_position_to_refer_start_pose(void)
{
	POSE_STRU curr;
	POSE_STRU target;
	POSE_STRU front_pos;
	double radian = 0.0;
	double acute_angle = 0.0;
	double hypotenuse = 0.0;  //三角形斜边
	double x_deviation = 0.0;
	double y_deviation = 0.0;
	double refer_angle = 0.0;
	double y_trend = 1.0;
	double vertical_distance = 0.0;
	bool area_part_is_left = false;

	bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
	//std::cout<<std::endl;
	//debug_print_warnning("++++++++++++");
	cfg_if_get_current_position(curr);
	p_traight_line_moving->set_straight_moving_refer_start_pose(curr);
	//printf("set curr.pos {(%lf, %lf) : %lf}\n", curr.point.x, curr.point.y, curr.angle);
	
	vertical_distance = get_vertical_distance_curr_position_to_refer_line();
	hypotenuse = vertical_distance;
	
	refer_angle = get_reference_angle();
	angel_base angel_base_instance;
	acute_angle = angel_base_instance.convert_to_acute_angle(refer_angle);
	radian = angel_base_instance.convert_degrees_to_radians(acute_angle);
	x_deviation = hypotenuse*sin(radian);
	y_deviation = hypotenuse*cos(radian);

	//printf("refer_angle(%lf)  ==>  acute_angle(%lf) ==> radian(%lf)\n", refer_angle, acute_angle, radian);
	//printf("sin(%lf) = %lf;   ", radian, sin(radian));
	//printf("cos(%lf) = %lf;\n", radian, cos(radian));
	//printf("hypotenuse  = %lf\n", hypotenuse);
	//printf("x_deviation = %lf,       y_deviation = %lf\n", x_deviation, y_deviation);
	
	area_part_is_left = cfg_if_test_partial_cleaning_part_is_left();
	if (false == area_part_is_left)
	{
		y_trend = -y_trend;
		//debug_print_fatal("area_part_is_left =%d ;y_trend=%lf", area_part_is_left, y_trend);
	}

	get_front_position(front_pos);
	target.angle = refer_angle;
	target.point.x = front_pos.point.x - x_deviation;
	target.point.y = front_pos.point.y + y_deviation*y_trend;
	
	p_traight_line_moving->set_straight_moving_refer_target_pose(target);
	
	//printf("set target.pos{ (%lf, %lf) : %lf}\n", target.point.x, target.point.y, target.angle);
	//debug_print_warnning("------------");
	//std::cout<<std::endl;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_distance_to_start_point_pos
 功能描述  : 获取当前位置到参考线初始点位置的距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_distance_to_start_point_pos(void)
{
	double distance = 0.0;
	POSE_STRU curr_pos;
	POSE_STRU start_pos;
	
	cfg_if_get_current_position(curr_pos);
	get_refer_line_start_point_pose(start_pos);
	line_base line_base_instance;
	distance = line_base_instance.get_distance(curr_pos.point.x, curr_pos.point.y, start_pos.point.x, start_pos.point.y);
	//debug_print_info("distance(%lf) = |curr_pos.point=(%lf, %lf)<-->start_pos.point=(%lf, %lf)|", distance, curr_pos.point.x, curr_pos.point.y, start_pos.point.x, start_pos.point.y);
	return distance;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_distance_to_end_point_pos
 功能描述  : 获取当前位置到参考线末尾点位置的距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_distance_to_end_point_pos(void)
{
	double distance = 0.0;
	POSE_STRU curr_pos;
	POSE_STRU end_pos;
	
	cfg_if_get_current_position(curr_pos);
	get_refer_line_end_point_pose(end_pos);
	line_base line_base_instance;
	distance = line_base_instance.get_distance(curr_pos.point.x, curr_pos.point.y, end_pos.point.x, end_pos.point.y);
	//debug_print_info("distance(%lf) = |curr_pos.point=(%lf, %lf)<-->end_pos.point=(%lf, %lf)|", distance, curr_pos.point.x, curr_pos.point.y, end_pos.point.x, end_pos.point.y);

	return distance;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_vertical_dimension_curr_pos_to_refer_line
 功能描述  : 获取当前点到参考线的垂直距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_vertical_dimension_curr_pos_to_refer_line(void)
{
	POSE_STRU pos;
	POSE_STRU curr_pos;
	double radian = 0.0;
	double angle = 0.0;
	double refe_angle = 0.0;
	double acute_angle = 0.0;
	double distance = 0.0;
	double vertical_dimension = 0.0;
	bool direction_is_forward = false;
	
	direction_is_forward = cfg_if_test_move_direction_is_forward();
	if (true == direction_is_forward)
	{
		get_refer_line_start_point_pose(pos);
		angle = get_refer_line_start_pos_to_curr_pos_angle();
	}
	else
	{
		get_refer_line_end_point_pose(pos);
		angle = get_refer_line_end_pos_to_curr_pos_angle();
	}

	cfg_if_get_current_position(curr_pos);
	refe_angle = get_reference_angle();
	line_base line_base_instance;
	distance = line_base_instance.get_distance(curr_pos.point.x, curr_pos.point.y, pos.point.x, pos.point.y);
	angel_base angel_base_instance;
	acute_angle = angel_base_instance.get_angle_differences(angle, refe_angle);
	radian = angel_base_instance.convert_degrees_to_radians(acute_angle);
	vertical_dimension = distance*sin(radian);
	//debug_print_warnning("direction_is_forward(%d), angle(%lf), refe_angle(%lf), acute_angle(%lf),radian(%lf), vertical_dimension(%lf)",direction_is_forward, angle, refe_angle, acute_angle,radian, vertical_dimension);

	return vertical_dimension;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_refer_line_start_pos_to_curr_pos_angle
 功能描述  : 获取参考线起始点指向当前位置点的角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_refer_line_start_pos_to_curr_pos_angle(void)
{
	double angle = 0.0;
	POSE_STRU pos;
	POSE_STRU curr_pos;

	cfg_if_get_current_position(curr_pos);
	get_refer_line_start_point_pose(pos);
	angel_base angel_base_instance;
	angle = angel_base_instance.get_angle(pos.point.x, pos.point.y, curr_pos.point.x, curr_pos.point.y);

	return angle;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_refer_line_end_pos_to_curr_pos_angle
 功能描述  : 获取参考线末尾的指向当前位置点的方向角度
                 
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_refer_line_end_pos_to_curr_pos_angle(void)
{
	double angle = 0.0;
	POSE_STRU pos;
	POSE_STRU curr_pos;
	
	cfg_if_get_current_position(curr_pos);
	get_refer_line_end_point_pose(pos);
	angel_base angel_base_instance;
	angle = angel_base_instance.get_angle(pos.point.x, pos.point.y, curr_pos.point.x, curr_pos.point.y);

	return angle;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_center_pose_to_curr_pos_angle
 功能描述  : 获取中心点指向当前点的角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_center_pose_to_curr_pos_angle(void)
{
	double angle = 0.0;
	POSE_STRU orig_pos;
	POSE_STRU curr_pos;
	
	cfg_if_get_current_position(curr_pos);
	cfg_if_get_partial_cleaning_original_pose(orig_pos);
	angel_base angel_base_instance;
	angle = angel_base_instance.get_angle(orig_pos.point.x, orig_pos.point.y, curr_pos.point.x, curr_pos.point.y);

	return angle;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.test_arrive_at_refer_line
 功能描述  : 检测是否已经运行到与参考方向平行
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_partial_cleaning::test_arrive_at_refer_line(void)
{
	bool flag = false;
	bool is_over = false;
	double current = 0.0;
	double refe_angle = 0.0;
	bool area_part_is_left = false;
	bool direction_is_forward = false;
	bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();

	flag = p_traight_line_moving->test_is_traight_line_moving();
	if (true == flag)
	{
		refe_angle = get_reference_angle();
		current = get_center_pose_to_curr_pos_angle();
	
		area_part_is_left = cfg_if_test_partial_cleaning_part_is_left();
		direction_is_forward = cfg_if_test_move_direction_is_forward();
		angel_base angel_base_instance;
		if(true == area_part_is_left)
		{
			if(true == direction_is_forward)
			{
				is_over = angel_base_instance.test_angle_is_over_clockwise(current, refe_angle);
			}
			else
			{
				is_over = angel_base_instance.test_angle_is_over_anticlockwise(current, refe_angle);
			}
		}
		else
		{
			if(false == direction_is_forward)
			{
				is_over = angel_base_instance.test_angle_is_over_clockwise(current, refe_angle);
			}
			else
			{
				is_over = angel_base_instance.test_angle_is_over_anticlockwise(current, refe_angle);
			}
		}
		//debug_print_info("is_over=%d; is_left =%d ;direction_is_forward =%d; current= %lf, refe_angle= %lf",is_over, area_part_is_left, direction_is_forward, current, refe_angle);	
	}
	else
	{
		//debug_print_error("false == flag ==%d", flag);
	}
	
	return is_over;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.test_detect_obstacle_turn_back
 功能描述  : 检测到障碍物返回
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_partial_cleaning::test_detect_obstacle_turn_back(void)
{
	bool flag = false;
	ULTRASONIC_SENSOR_STRU state;
	
	cfg_if_get_ultrasonic_sensor(state);
	if (SAFETY_LEVEL_WARN == state.level)
	{
		flag = true;
		debug_print_warnning("flag = true;");
	}
	else if (SAFETY_LEVEL_FATAL == state.level)
	{
		flag = true;
		debug_print_fatal("flag = true;");
	}
	else
	{
		flag = false;
	}
	
	return flag;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.update_refer_line_traight_line_moving_target_pos
 功能描述  : 更新目标坐标点位置
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::update_refer_line_traight_line_moving_target_pos(void)
{
	POSE_STRU pos;
	bool direction_is_forward = false;
	bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
	direction_is_forward = cfg_if_test_move_direction_is_forward();
	if (true == direction_is_forward)
	{
		get_refer_line_end_point_pose(pos);
		debug_print_info("set target pos end {(%lf, %lf): %lf}", pos.point.x, pos.point.y, pos.angle);
	}
	else
	{
		get_refer_line_start_point_pose(pos);
		debug_print_info("set target pos start {(%lf, %lf): %lf}", pos.point.x, pos.point.y, pos.angle);
	}
	p_traight_line_moving->set_traight_line_moving_target_pos(pos);
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.traight_line_moving_dynamic_regulation
 功能描述  : 动态调整直线运行速度
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::traight_line_moving_dynamic_regulation(void)
{
	bool flag = false;
	bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
	flag = p_traight_line_moving->test_is_traight_line_moving();
	if (true == flag)
	{
		p_traight_line_moving->straight_driving_adjust_angle();
		p_traight_line_moving->straight_driving_adjust_speed();
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.set_partial_cleaning_state
 功能描述  : 设置局部清扫的阶段
 输入参数: const LOCAL_MOVE_STATE_ENUM data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::set_partial_cleaning_state(const LOCAL_MOVE_STATE_ENUM data)
{
	cfg_if_set_partial_cleaning_state(data);
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.local_move_start
 功能描述  : 开始运行局部无障碍清扫
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::local_move_start(void)
{
	cfg_if_set_partial_cleaning_direction(FORWARD);
	cfg_if_set_partial_cleaning_area_part(AREA_PART_LEFT);
	set_partial_cleaning_state(LOCAL_MOVE_PIVOT);
	
	bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
	p_traight_line_moving->set_traight_line_moving();
	cfg_if_disable_linear_velocity_ajust();
	update_refer_line_traight_line_moving_target_pos();
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.local_move_pivot
 功能描述  : 局部无障碍清扫原地旋转返回
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::local_move_pivot(void)
{
	bool flag = false;
	double length = 0.0;
	double distance = 0.0;
	bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
	
	flag = p_traight_line_moving->test_is_traight_line_moving();
	if (true == flag)
	{
		length = get_local_move_edge_length();
		distance = get_distance_to_start_point_pos();
		if (distance >= length)
		{
			monitor_angle_set_pivot_deal();
		}
		else
		{
			flag = false;
			test_detect_obstacle_turn_back();
			if (true == flag)
			{
				monitor_angle_set_pivot_deal();
			}
			else
			{
				traight_line_moving_dynamic_regulation();
			}
		}
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.local_move_fst_line_start
 功能描述  : 开始局部弓字行走
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::local_move_fst_line_start(void)
{
	bool flag = false;
	double length = 0.0;
	double distance = 0.0;

	length = get_local_move_edge_length();
	distance = get_distance_to_end_point_pos();
	if (distance >= length)
	{
		change_rotate_direction();
		set_partial_cleaning_state(LOCAL_MOVE_FST_HALF);
	}
	else
	{
		test_detect_obstacle_turn_back();
		if (true == flag)
		{
			change_rotate_direction();
			set_partial_cleaning_state(LOCAL_MOVE_FST_HALF);
		}
		else
		{
			traight_line_moving_dynamic_regulation();
		}
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.local_move_return_reference_line
 功能描述  : 返回参考线
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::local_move_return_reference_line(void)
{
	bool flag = false;
	flag = test_arrive_at_refer_line();
	if (true == flag)
	{
		cfg_if_disable_linear_velocity_ajust();
		set_partial_cleaning_state(LOCAL_MOVE_SEC_HALF_START);
		smooth_decelerate_stop();
		bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
		p_traight_line_moving->clear_traight_line_moving_data();
	}
	else
	{
		traight_line_moving_dynamic_regulation();
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.local_move_return_center_pose
 功能描述  : 局部弓字型行走返回中心位置
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::local_move_return_center_pose(void)
{
	bool flag = false;
	
	flag = test_arrive_at_refer_line();
	if (true == flag)
	{
		set_partial_cleaning_state(LOCAL_MOVE_TURN_TO_ORIGINAL_DIR);
		smooth_decelerate_stop();
		bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
		p_traight_line_moving->clear_traight_line_moving_data();
	}
	else
	{
		traight_line_moving_dynamic_regulation();
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.local_move_fst_half_area
 功能描述  : 局部弓字行走第一半区往返运动
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::local_move_fst_half_area(void)
{
	bool flag = false;
	bool half_done_flag = false;
	double l_x = 0.0;
	double r_x = 0.0;
	double length = 0.0;
	double half_length = 0.0;
	double distance = 0.0;
	double ref_value = 0.0;
	double precision = 0.01;
	double vertical_dimension = 0.0;
	POSE_STRU curr_pos;
	POSE_STRU original_pose;
	
	bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
	flag = p_traight_line_moving->test_is_traight_line_moving();
	if (true == flag)
	{
		#ifdef ENABLE_MOUDLATE
		p_traight_line_moving->update_traight_line_moving_data();
		traight_line_moving_dynamic_regulation();
		#endif /* ENABLE_MOUDLATE */
		
		distance = p_traight_line_moving->get_distance_to_traight_line_moving_start_pos();
		length = get_local_move_edge_length();

		half_length = length/2;
		vertical_dimension = get_vertical_dimension_curr_pos_to_refer_line();
		if (vertical_dimension > half_length)
		{
			half_done_flag = true;
		}
		
		if (distance >= length)
		{
			if (true == half_done_flag)
			{
				set_partial_cleaning_state(LOCAL_MOVE_FST_HALF_DONE);
				return;
			}
			change_rotate_direction();
		}
		else
		{
			test_detect_obstacle_turn_back();
			if (true == flag)
			{
				if (true == half_done_flag)
				{
					set_partial_cleaning_state(LOCAL_MOVE_FST_HALF_DONE);
					return;
				}
				change_rotate_direction();
			}
		}
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.local_move_sec_half_area
 功能描述  : 局部弓字行走第二半区往返运动
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::local_move_sec_half_area(void)
{
	bool flag = false;
	double l_x = 0.0;
	double r_x = 0.0;
	double temp = 0.0;
	double length = 0.0;
	double half_length = 0.0;
	double distance = 0.0;
	double ref_value = 0.0;
	double precision = 0.01;
	double vertical_dimension = 0.0;
	POSE_STRU curr_pos;
	POSE_STRU original_pose;
	ACTION_STATUS_ENUM curr_action;

	bll_traight_line_moving* p_traight_line_moving = bll_traight_line_moving::get_instance();
	flag = p_traight_line_moving->test_is_traight_line_moving();
	if (true == flag)
	{
		#ifdef ENABLE_MOUDLATE
		if (true == flag)
		{
			traight_line_moving_dynamic_regulation();
		}
		#endif /* ENABLE_MOUDLATE */

		distance = p_traight_line_moving->get_distance_to_traight_line_moving_start_pos();
		length = get_local_move_edge_length();
		if (distance >= length)
		{
			half_length = length/2;
			vertical_dimension = get_vertical_dimension_curr_pos_to_refer_line();
			if (vertical_dimension > half_length)
			{
				set_partial_cleaning_state(LOCAL_MOVE_TURN_TO_CENTER_DIR);
				return;
			}
			
			change_rotate_direction();
		}
		else
		{
			test_detect_obstacle_turn_back();
			if (true == flag)
			{
				change_rotate_direction();
			}
		}
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.do_local_move
 功能描述  : 局部无障碍弓字型遍历行驶
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::do_local_move(void)
{
	bool flag = false;
	LOCAL_MOVE_STATE_ENUM state;
	
	flag = cfg_if_test_partial_cleaning_is_enable();
	if (true == flag)
	{
		save_first_line_refer_direction();

		cfg_if_get_partial_cleaning_state(state);
		if (LOCAL_MOVE_START == state)
		{
			local_move_start();
		}
		else if (LOCAL_MOVE_PIVOT == state)
		{
			local_move_pivot();
		}
		else if (LOCAL_MOVE_FST_LINE_START == state)
		{
			local_move_fst_line_start();
		}
		else if (LOCAL_MOVE_FST_HALF == state)
		{
			local_move_fst_half_area();
		}
		else if (LOCAL_MOVE_FST_HALF_DONE == state)
		{
			monitor_angle_turn_to_refer_line_deal();
		}
		else if (LOCAL_MOVE_RETURN_REFER_LINE == state)
		{
			local_move_return_reference_line();
		}
		else if (LOCAL_MOVE_SEC_HALF_START == state)
		{
			monitor_angle_turn_to_center_deal();
		}
		else if (LOCAL_MOVE_SEC_HALF == state)
		{
			local_move_sec_half_area();
		}
		else if (LOCAL_MOVE_TURN_TO_CENTER_DIR == state)
		{
			monitor_angle_turn_to_original_pose_deal();
		}
		else if (LOCAL_MOVE_RETURN_CENTER_POS == state)
		{
			local_move_return_center_pose();
		}
		else if (LOCAL_MOVE_TURN_TO_ORIGINAL_DIR == state)
		{
			monitor_angle_turn_to_original_direction_deal();
		}
		else if (LOCAL_MOVE_ALL_DONE == state)
		{
			set_partial_cleaning_state(LOCAL_MOVE_STOP);
		}
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.local_cover_movement
 功能描述  : 区域覆盖行驶
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月31日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::local_cover_movement(void)
{
	do_local_move();
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.smooth_decelerate_stop
 功能描述  : 缓慢停止
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::smooth_decelerate_stop(void)
{
	//待实现....
	cfg_if_change_curr_action(STOP);
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_monitor_angle_turn_back_angle
 功能描述  : 获取拐弯返回到达响应角度
 输入参数: void
 输出参数: 无
 返 回 值: double 
 
 修改历史:
  1.日     期: 2017年8月11日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_monitor_angle_turn_back_angle(void)
{
	bool flag = false;
	MONITOR_ENUM stage;
	double respond_angle = 0.0;
	double forward_angle = 0.0;
	double reverse_angle = 0.0;
	bool inversion = false;
	bool is_forward = false;
	static bll_rotate* p_rotate = bll_rotate::get_instance();
	
	flag = p_rotate->test_angle_monitor_is_no_working();
	if (true == flag)
	{
		if ( true == check_reference_data_valid())
		{
			is_forward = cfg_if_test_move_direction_is_forward();
			get_reference_data_forward_angle(forward_angle);
			get_reference_data_reverse_angle(reverse_angle);
			inversion = get_reference_data_inversion();
			if ( false == inversion )
			{
				if(true == is_forward)
				{
					respond_angle = reverse_angle;
				}
				else
				{
					respond_angle = forward_angle;
				}
			}
			else
			{
				if(true == is_forward)
				{
					respond_angle = forward_angle;
				}
				else
				{
					respond_angle = reverse_angle;
				}
			}
		}
		else
		{
			respond_angle = cfg_if_get_current_position_reverse_angle();
			debug_print_warnning("current position reverse angle=%lf", respond_angle);
		}
	}
	
	return respond_angle;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.monitor_angle_set_turn_back_deal
 功能描述  : 设置检测角度旋转返回处理
 输入参数: void
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::monitor_angle_set_turn_back_deal(void)
{
	bool flag = false;
	double angle = 0.0;
	MONITOR_ENUM stage;
	void(bll_rotate::*pf)(void) = NULL;
	
	static bll_rotate* p_rotate = bll_rotate::get_instance();
	flag = p_rotate->test_angle_monitor_is_no_working();
	if (true == flag)
	{
		angle = get_monitor_angle_turn_back_angle();
		pf = &bll_rotate::monitor_angle_turn_back_respond;
		p_rotate->set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.monitor_angle_set_pivot_deal
 功能描述  : 设置原地旋转反向的处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::monitor_angle_set_pivot_deal(void)
{
	bool flag = false;
	double angle = 0.0;
	MONITOR_ENUM stage;
	void(bll_rotate::*pf)(void) = NULL;
	
	static bll_rotate* p_rotate = bll_rotate::get_instance();
	flag = p_rotate->test_angle_monitor_is_no_working();
	if (true == flag)
	{
		POSE_STRU original_pose;
		cfg_if_get_partial_cleaning_original_pose(original_pose);
		cfg_if_change_curr_action(PIVOT);
		angel_base angel_base_instance;
		angle = angel_base_instance.get_reverse_angle(original_pose.angle);
		pf = &bll_rotate::monitor_angle_pivot_respond;
		p_rotate->set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.monitor_angle_turn_to_refer_line_deal
 功能描述  : 转向参考线最近方向处理
 输入参数: void
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::monitor_angle_turn_to_refer_line_deal(void)
{
	bool flag = false;
	double angle = 0.0;
	ACTION_STATUS_ENUM action;
	void(bll_rotate::*pf)(void) = NULL;
	
	static bll_rotate* p_rotate = bll_rotate::get_instance();
	flag = p_rotate->test_angle_monitor_is_no_working();
	if (true == flag)
	{
		cfg_if_disable_linear_velocity_ajust();
		cfg_if_disable_angular_velocity_ajust();
		get_turn_right_angle_action_type(action);
		cfg_if_change_curr_action(action);
		
		angle = get_driving_direction_right_angle();
		pf = &bll_rotate::monitor_angle_turn_to_refer_line_respond;
		p_rotate->set_monitor_angle_rotate_call_back(angle, pf);
		debug_print_warnning("||||||||||||||||||||| angle=%lf; action=%d", angle, action);
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.monitor_angle_turn_to_center_deal
 功能描述  : 转向中心点处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::monitor_angle_turn_to_center_deal(void)
{
	bool flag = false;
	double angle = 0.0;
	ACTION_STATUS_ENUM action;
	void(bll_rotate::*pf)(void) = NULL;
	
	static bll_rotate* p_rotate = bll_rotate::get_instance();
	flag = p_rotate->test_angle_monitor_is_no_working();
	if (true == flag)
	{
		cfg_if_disable_angular_velocity_ajust();
		angle = get_turnt_to_reference_deriction_angle();
		get_turnt_to_reference_deriction_action_type(action);
		cfg_if_change_curr_action(action);
		pf = &bll_rotate::monitor_angle_turn_to_center_respond;
		p_rotate->set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_turnt_to_original_pose_angle
 功能描述  : 获取转向原始点的目标角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_turnt_to_original_pose_angle(void)
{
	double angle = 0.0;
	POSE_STRU current_pos;
	POSE_STRU original_pos;

	cfg_if_get_current_position(current_pos);
	cfg_if_get_partial_cleaning_original_pose(original_pos);

	angel_base angel_base_instance;
	angle = angel_base_instance.get_angle(current_pos.point.x, current_pos.point.y, original_pos.point.x, original_pos.point.y);

	//debug_print_warnning("(%lf, %lf) -> (%lf, %lf) angle=%lf", current_pos.point.x, current_pos.point.y, original_pos.point.x, original_pos.point.y, angle);
	return angle;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_turnt_to_original_pose_action_type
 功能描述  : 获取转向原始点的运动类型
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::get_turnt_to_original_pose_action_type(ACTION_STATUS_ENUM &action)
{
	get_rotate_action_type(action);
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.monitor_angle_turn_to_original_pose_deal
 功能描述  : 转向原始出发点处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::monitor_angle_turn_to_original_pose_deal(void)
{
	bool flag = false;
	double angle = 0.0;
	MONITOR_ENUM stage;
	ACTION_STATUS_ENUM action;
	void(bll_rotate::*pf)(void) = NULL;
	
	static bll_rotate* p_rotate = bll_rotate::get_instance();
	flag = p_rotate->test_angle_monitor_is_no_working();
	if (true == flag)
	{
		cfg_if_disable_angular_velocity_ajust();
		angle = get_turnt_to_original_pose_angle();
		get_turnt_to_original_pose_action_type(action);
		cfg_if_change_curr_action(action);
		pf = &bll_rotate::monitor_angle_turn_to_original_pose_respond;
		p_rotate->set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_turnt_to_original_direction_angle
 功能描述  : 获取原始方向角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_turnt_to_original_direction_angle(void)
{
	double angle = 0.0;
	get_reference_data_forward_angle(angle);
	return angle;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_turnt_to_original_direction_action_type
 功能描述  : 获取转向原始方向的运动类型
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::get_turnt_to_original_direction_action_type(ACTION_STATUS_ENUM &action)
{
	get_rotate_action_type(action);
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.monitor_angle_turn_to_original_direction_deal
 功能描述  : 转向原始方向功能处理
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月29日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::monitor_angle_turn_to_original_direction_deal(void)
{
	bool flag = false;
	double angle = 0.0;
	ACTION_STATUS_ENUM action;
	void(bll_rotate::*pf)(void) = NULL;
	
	static bll_rotate* p_rotate = bll_rotate::get_instance();
	flag = p_rotate->test_angle_monitor_is_no_working();
	if (true == flag)
	{
		cfg_if_disable_angular_velocity_ajust();
		
		angle = get_turnt_to_original_direction_angle();
		get_turnt_to_original_direction_action_type(action);
		//debug_print_info("  action = %d   angle = %lf", action, angle);
		cfg_if_change_curr_action(action);
		pf = &bll_rotate::monitor_angle_turn_to_original_direction_respond;
		p_rotate->set_monitor_angle_rotate_call_back(angle, pf);
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.save_first_line_refer_direction
 功能描述  : 保存第一行参考数据
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::save_first_line_refer_direction(void)
{
	double temp = 0.0;
	double forward = 0.0;
	double reverse = 0.0;
	const double inversion = 180.0;
	POSE_STRU pos;

	if ( false == check_reference_data_valid())
	{
		cfg_if_get_current_position(pos);
		angel_base angel_base_instance;
		forward = angel_base_instance.format_angle(pos.angle);
		temp = forward + inversion;
		reverse = angel_base_instance.format_angle(temp);

		set_reference_data_forward_angle(forward);
		set_reference_data_reverse_angle(reverse);
		set_reference_data_inversion(false);
		set_reference_data_valid(true);

		debug_print_info("current pos.angle=%lf; forward=%lf; reverse =%lf",pos.angle, forward, reverse);
		updata_district_area(pos);
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_local_move_edge_length
 功能描述  : 获取局域行驶区域方框边长
 输入参数: 无
 输出参数: 无
 返 回 值: 边长
 
 修改历史:
  1.日     期: 2017年10月20日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_local_move_edge_length(void)
{
	return local_erea_edge_length_;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_local_move_half_edge_length
 功能描述  : 获取局域行驶区域方框边长的一半
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年10月30日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_local_move_half_edge_length(void)
{
	return local_erea_edge_length_/2.0;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.change_rotate_direction
 功能描述  : 修改旋转方向
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::change_rotate_direction(void)
{
	ACTION_STATUS_ENUM action;

	monitor_angle_set_turn_back_deal();
	get_turn_back_action_type(action);
	cfg_if_change_curr_action(action);
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.updata_district_area
 功能描述  : 更新局域四个角的位置点
 输入参数: const POSE_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::updata_district_area(const POSE_STRU &data)
{
	double x_min = 0.0;
	double x_max = 0.0;
	double y_min = 0.0;
	double y_max = 0.0;
	double x0 = data.point.x;
	double y0 = data.point.y;
	double length = 0.0;
	double opposite = 0.0;    //三角形对边
	double adjacent = 0.0;    //三角形邻边
	double hypotenuse = 0.0;  //三角形斜边
	POSE_STRU pose[4];
	POSE_STRU start;
	POSE_STRU end;
	bool function_enable = false;
	LOCAL_MOVE_MODE_ENUM mode;
	LOCAL_MOVE_STATE_ENUM state;

	function_enable = cfg_if_test_partial_cleaning_is_enable();
	if (true == function_enable)
	{
		cfg_if_get_partial_cleaning_state(state);
		if (LOCAL_MOVE_STOP == state)
		{
			cfg_if_set_partial_cleaning_original_pose(data);
			
			cfg_if_get_partial_cleaning_mode(mode);
			if (AMBIENT_NO_OBSTACLE == mode)
			{
				length = get_local_move_half_edge_length();
				x_min = x0 - length;
				x_max = x0 + length;
				y_min = y0 - length;
				y_max = y0 + length;
				debug_print_warnning("p(x0, y0) = {(%lf, %lf) : %lf}; length=%lf", x0, y0, data.angle, length);
				pose[0].point = {x_max, y_max};
				pose[1].point = {x_min, y_max};
				pose[2].point = {x_min, y_min};
				pose[3].point = {x_max, y_min};
				for (int i = 0; i < 4; ++i)
				{
					//debug_print_warnning("pose[%d].point=(%lf, %lf)", i, pose[i].point.x, pose[i].point.y);
					cfg_if_set_local_move_planning_pose(pose[i], i); 
				}
				//debug_print_warnning("pose[%d].point=(%lf, %lf)   pose[%d].point=(%lf, %lf)", 1, pose[1].point.x, pose[1].point.y, 0, pose[0].point.x, pose[0].point.y);
				//debug_print_warnning("pose[%d].point=(%lf, %lf)   pose[%d].point=(%lf, %lf)", 2, pose[2].point.x, pose[2].point.y, 3, pose[3].point.x, pose[3].point.y);
				hypotenuse = length;                      //三角形斜边
				opposite = hypotenuse*sin((M_PI/180.0)*data.angle);    //三角形对边
				adjacent = hypotenuse*cos((M_PI/180.0)*data.angle);    //三角形邻边
				//debug_print_warnning("hypotenuse=%lf; opposite=%lf; adjacent=%lf;", hypotenuse, opposite, adjacent);
				start.point.x = x0 - adjacent;
				start.point.y = y0 - opposite;
				start.angle = data.angle;
				end.point.x = x0 + adjacent;
				end.point.y = y0 + opposite;
				end.angle = data.angle;
				//debug_print_warnning("start.point = {(%lf, %lf) : %lf}", start.point.x, start.point.y, start.angle);
				//debug_print_warnning("end.point   = {(%lf, %lf) : %lf}", end.point.x, end.point.y, end.angle);
				set_refer_line_start_point_pose(start);
				set_refer_line_end_point_pose(end);

				state = LOCAL_MOVE_START;
				cfg_if_set_partial_cleaning_state(state);
			}
			else if (AMBIENT_HAS_OBSTACLE == mode)
			{
				//按实际碰撞沿墙传感检测局域块的大小，且不能大于无障碍时的大小
			}
		}
	}
	else
	{
		
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.set_refer_line_start_point_pose
 功能描述  : 设置参考线（第一条行驶线路）的开始点
 输入参数: const POSE_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::set_refer_line_start_point_pose(const POSE_STRU data)
{
	ref_start_point_ = data;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_refer_line_start_point_pose
 功能描述  : 获取参考线（第一条行驶线路）的开始点
 输入参数: POSE_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::get_refer_line_start_point_pose(POSE_STRU &data)
{
	data = ref_start_point_;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.set_refer_line_end_point_pose
 功能描述  : 设置参考线（第一条行驶线路）的结束点
 输入参数: const POSE_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::set_refer_line_end_point_pose(const POSE_STRU data)
{
	ref_end_point_ = data;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_refer_line_end_point_pose
 功能描述  : 获取参考线（第一条行驶线路）的结束点
 输入参数: POSE_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::get_refer_line_end_point_pose(POSE_STRU &data)
{
	data = ref_end_point_;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_reference_angle
 功能描述  : 获取直行参考角
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月16日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_reference_angle(void)
{
	double angle = 0.0;
	bool direction_is_forward = false;

	direction_is_forward = cfg_if_test_move_direction_is_forward();
	if (true == direction_is_forward)
	{
		get_reference_data_forward_angle(angle);
		//debug_print_info("---------------------->>>>>>(angle=%lf)", angle);
	}
	else
	{
		get_reference_data_reverse_angle(angle);
		//debug_print_info("<<<<<<<<---------------------(angle=%lf)", angle);
	}

	return angle;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.set_reference_data
 功能描述  : 设置参照数据
 输入参数: const REFERENCE_DATA_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月11日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::set_reference_data(const REFERENCE_DATA_STRU &data)
{
	ref_data_ = data;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_reference_data
 功能描述  : 获取参照数据
 输入参数: REFERENCE_DATA_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月11日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::get_reference_data(REFERENCE_DATA_STRU &data)
{
	data = ref_data_;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.check_reference_data_valid
 功能描述  : 检查参考数据是否已经有效保存
 输入参数: 无
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_partial_cleaning::check_reference_data_valid(void)
{
	bool ret = false;
	REFERENCE_DATA_STRU ref_data;
	
	get_reference_data(ref_data);
	
	ret = ref_data.fst_dir.valid;

	return ret;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.set_reference_data_forward_angle
 功能描述  : 设置参考数据正向的角度值
 输入参数: double data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::set_reference_data_forward_angle(double data)
{
	ref_data_.fst_dir.forward = data;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_reference_data_forward_angle
 功能描述  : 获取参考数据正向的角度值
 输入参数:  
 输出参数: double &data 
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_partial_cleaning::get_reference_data_forward_angle(double &data)
{
	bool ret = false;
	REFERENCE_DATA_STRU ref_data;
	
	get_reference_data(ref_data);
	
	ret = ref_data.fst_dir.valid;
	if ( true == ret)
	{
		data = ref_data.fst_dir.forward;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.set_reference_data_reverse_angle
 功能描述  : 设置参考数据反向的角度值
 输入参数: double data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::set_reference_data_reverse_angle(double data)
{
	ref_data_.fst_dir.reverse = data;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_reference_data_reverse_angle
 功能描述  : 获取参考数据反向的角度值
 输入参数: double &data  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年10月27日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_partial_cleaning::get_reference_data_reverse_angle(double &data)
{
	bool ret = false;
	REFERENCE_DATA_STRU ref_data;
	
	get_reference_data(ref_data);
	ret = ref_data.fst_dir.valid;
	if ( true == ret)
	{
		data = ref_data.fst_dir.reverse;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.set_reference_data_inversion
 功能描述  : 设置转向颠倒标志
 输入参数: bool data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年10月30日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::set_reference_data_inversion(bool data)
{
	ref_data_.fst_dir.inversion = data;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_reference_data_inversion
 功能描述  : 获取转向颠倒标志
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年10月30日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_partial_cleaning::get_reference_data_inversion(void)
{
	return ref_data_.fst_dir.inversion;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.set_reference_data_valid
 功能描述  : 设置参考数据有效性标志
 输入参数: bool valid  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::set_reference_data_valid(bool valid)
{
	ref_data_.fst_dir.valid = valid;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_reference_data_valid
 功能描述  : 获取参考数据有效性标志
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_partial_cleaning::get_reference_data_valid(void)
{
	return ref_data_.fst_dir.valid;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_rotate_action_type
 功能描述  : 获取旋转方向
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::get_rotate_action_type(ACTION_STATUS_ENUM &action)
{
	bool area_part_is_left = false;
	bool direction_is_forward = false;
	const ACTION_STATUS_ENUM clockwise = TURN_RIGHT_ANGLE_CLOCKWISE;
	const ACTION_STATUS_ENUM anticlockwise = TURN_RIGHT_ANGLE_ANTICLOCKWISE;
	area_part_is_left = cfg_if_test_partial_cleaning_part_is_left();
	direction_is_forward = cfg_if_test_move_direction_is_forward();

	//debug_print_warnning("area_part_is_left = %d; direction_is_forward = %d;",area_part_is_left, direction_is_forward);
	if(true == area_part_is_left)
	{
		if(true == direction_is_forward)
		{
			action = clockwise;
			//debug_print_warnning("action = TURN_RIGHT_ANGLE_CLOCKWISE;");
		}
		else
		{
			action = anticlockwise;
			//debug_print_warnning("action = TURN_RIGHT_ANGLE_ANTICLOCKWISE;");
		}
	}
	else
	{
		if(true == direction_is_forward)
		{
			action = anticlockwise;
			//debug_print_warnning("action = TURN_RIGHT_ANGLE_ANTICLOCKWISE;");
		}
		else
		{
			action = clockwise;
			//debug_print_warnning("action = TURN_RIGHT_ANGLE_CLOCKWISE;");
		}
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_driving_direction_right_angle
 功能描述  : 获取相对与运行方向的垂直角度
 输入参数: void  
 输出参数: 无
 返 回 值: 垂直角度
 行驶方向的垂直方向，且指向初始参考线一方
 修改历史:
  1.日     期: 2017年11月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_driving_direction_right_angle(void)
{
	double angle = 0.0;
	double right_angle = 0.0;
	bool area_part_is_left = false;
	bool direction_is_forward = false;
	ROTATE_DIRECTION_ENUM direction;

	area_part_is_left = cfg_if_test_partial_cleaning_part_is_left();
	direction_is_forward = cfg_if_test_move_direction_is_forward();
	if(true == area_part_is_left)
	{
		if (true == direction_is_forward)
		{
			get_reference_data_forward_angle(angle);
			direction = ANTICLOCKWISE;
		}
		else
		{
			get_reference_data_reverse_angle(angle);
			direction = CLOCKWISE;
		}
	}
	else
	{
		if (true == direction_is_forward)
		{
			get_reference_data_forward_angle(angle);
			direction = CLOCKWISE;
		}
		else
		{
			get_reference_data_reverse_angle(angle);
			direction = ANTICLOCKWISE;
		}
	}
	angel_base angel_base_instance;
	right_angle = angel_base_instance.get_right_angle(angle, direction);

	return right_angle;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_turn_back_action_type
 功能描述  : 获取返回运行模式类型
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::get_turn_back_action_type(ACTION_STATUS_ENUM &action)
{
	bool area_part_is_left = false;
	bool direction_is_forward = false;

	area_part_is_left = cfg_if_test_partial_cleaning_part_is_left();
	direction_is_forward = cfg_if_test_move_direction_is_forward();
	if(true == area_part_is_left)
	{
		if(true == direction_is_forward)
		{
			action = TURN_BACK_ANTICLOCKWISE;
			//debug_print_info(" area_part_is_left: action = TURN_BACK_ANTICLOCKWISE;");
		}
		else
		{
			action = TURN_BACK_CLOCKWISE;
			//debug_print_info(" area_part_is_left: action = TURN_BACK_CLOCKWISE;");
		}
	}
	else
	{
		if(true == direction_is_forward)
		{
			action = TURN_BACK_CLOCKWISE;
			//debug_print_info(" area_part_is_right: action = TURN_BACK_CLOCKWISE;");
		}
		else
		{
			action = TURN_BACK_ANTICLOCKWISE;
			//debug_print_info(" area_part_is_right: action = TURN_BACK_ANTICLOCKWISE;");
		}
	}
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_turn_right_angle_action_type
 功能描述  : 获取旋转到垂直方向的运动模式
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::get_turn_right_angle_action_type(ACTION_STATUS_ENUM &action)
{
	get_rotate_action_type(action);
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_turnt_to_reference_deriction_angle
 功能描述  : 获取转向参考方向的角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_partial_cleaning::get_turnt_to_reference_deriction_angle(void)
{
	double angle = 0.0;
	bool direction_is_forward = false;

	direction_is_forward = cfg_if_test_move_direction_is_forward();
	if (true == direction_is_forward)
	{
		get_reference_data_reverse_angle(angle);
	}
	else
	{
		get_reference_data_forward_angle(angle);
	}

	return angle;
}

/*****************************************************************************
 函 数 名: bll_partial_cleaning.get_turnt_to_reference_deriction_action_type
 功能描述  : 获取转向参考方向的动作模式
 输入参数: ACTION_STATUS_ENUM &action  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_partial_cleaning::get_turnt_to_reference_deriction_action_type(ACTION_STATUS_ENUM &action)
{
	get_rotate_action_type(action);
}


/******************************************************************************
 * 内部函数定义
 ******************************************************************************/


