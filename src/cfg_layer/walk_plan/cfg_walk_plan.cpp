/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: cfg_walk_plan.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年8月22日
  最近修改:
  功能描述   : 行走路径规划类相关功能定义
  函数列表:
  修改历史:
  1.日     期: 2017年8月22日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "cfg_walk_plan.h"

#include <math.h>
#include "debug_function.h"

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
 * 类定义
 ******************************************************************************/
pthread_mutex_t cfg_walk_plan::mutex_;
cfg_walk_plan* cfg_walk_plan::p_instance_ = NULL;

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: cfg_walk_plan*
 
 修改历史:
  1.日     期: 2017年8月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
cfg_walk_plan* cfg_walk_plan::get_instance(void)
{
	if (p_instance_ == NULL)
	{
		pthread_mutex_lock(&mutex_);
		if (p_instance_ == NULL)
		{
			p_instance_ = new cfg_walk_plan();
		}
		pthread_mutex_unlock(&mutex_);
	}
		
	return p_instance_;

}

/*****************************************************************************
 函 数 名: cfg_walk_plan.cfg_walk_plan
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: cfg_walk_plan
 
 修改历史:
  1.日     期: 2017年8月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
cfg_walk_plan::cfg_walk_plan()
{
	AREA_STRU district;
	district.max_x = 4;
	district.max_y = 4;
	district.precision = 0.1;
	debug_print_info("district.max_x = %lf, district.max_y = %lf", district.max_x, district.max_y);
	set_district_area(district);

	disable_local_move();
	enable_local_move();
	
	set_local_move_mode(AMBIENT_NO_OBSTACLE);
	set_local_move_state(LOCAL_MOVE_STOP);
	set_local_move_area_part(AREA_PART_LEFT);
	set_local_move_direction(FORWARD);
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.cfg_walk_plan
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: ~cfg_walk_plan
 
 修改历史:
  1.日     期: 2017年8月22日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
cfg_walk_plan::~cfg_walk_plan()
{

}

/*****************************************************************************
 函 数 名: cfg_walk_plan.enable_local_move
 功能描述  : 使能局域移动功能
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::enable_local_move(void)
{
	district_.enable = true;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.disable_local_move
 功能描述  : 禁用局域移动功能
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::disable_local_move(void)
{
	district_.enable = false;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.test_local_move_is_enable
 功能描述  : 检测局部移动功能是否开启
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_walk_plan::test_local_move_is_enable(void)
{
	return district_.enable;
}


/*****************************************************************************
 函 数 名: cfg_walk_plan.set_local_move_mode
 功能描述  : 设置局域行驶模式
 输入参数: const LOCAL_MOVE_MODE_ENUM data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_local_move_mode(const LOCAL_MOVE_MODE_ENUM data)
{
	district_.mode = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_local_move_mode
 功能描述  : 获取局域行驶模式
 输入参数: LOCAL_MOVE_MODE_ENUM &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_local_move_mode(LOCAL_MOVE_MODE_ENUM &data)
{
	data = district_.mode;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_local_move_state
 功能描述  : 设置局域行驶状态
 输入参数: const LOCAL_MOVE_STATE_ENUM data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_local_move_state(const LOCAL_MOVE_STATE_ENUM data)
{
	district_.state = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_local_move_state
 功能描述  : 获取局域行驶状态
 输入参数: LOCAL_MOVE_STATE_ENUM &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_local_move_state(LOCAL_MOVE_STATE_ENUM &data)
{
	data = district_.state;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_local_move_area_part
 功能描述  : 设置当前所在的左右半区局域
 输入参数: const AREA_PART_ENUM data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_local_move_area_part(const AREA_PART_ENUM data)
{
	district_.area_part = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_local_move_area_part
 功能描述  : 获取当前所在的左右半区局域
 输入参数: AREA_PART_ENUM &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_local_move_area_part(AREA_PART_ENUM &data)
{
	data = district_.area_part;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.test_local_move_area_part_is_left
 功能描述  : 检测是否在左边半区局域
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_walk_plan::test_local_move_area_part_is_left(void)
{
	bool ret = false;
	AREA_PART_ENUM data;
	
	get_local_move_area_part(data);
	if (AREA_PART_LEFT == data)
	{
		ret = true;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.test_local_move_area_part_is_right
 功能描述  : 检测是否在右边半区局域
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_walk_plan::test_local_move_area_part_is_right(void)
{
	bool ret = false;
	AREA_PART_ENUM data;
	
	get_local_move_area_part(data);
	if (AREA_PART_RIGHT == data)
	{
		ret = true;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.switch_local_move_area_part
 功能描述  : 切换当前所在的半区局域
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::switch_local_move_area_part(void)
{
	AREA_PART_ENUM data;

	get_local_move_area_part(data);
	if (AREA_PART_LEFT == data)
	{
		data = AREA_PART_RIGHT;
		//debug_print_info(" RRR set = AREA_PART_RIGHT= %d ;", data);
	}
	else
	{
		data = AREA_PART_LEFT;
		//debug_print_info(" LLL set = AREA_PART_RIGHT= %d ;", data);
	}
	set_local_move_area_part(data);
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_local_move_direction
 功能描述  : 设置当前运行的方向向前还是返回(相对于第一次平行直线行驶的方向而言)
 输入参数: const MOVE_DIRECTION_ENUM data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_local_move_direction(const MOVE_DIRECTION_ENUM data)
{
	district_.direction = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_local_move_direction
 功能描述  :  获取当前运行的方向向前还是返回(相对于第一次平行直线行驶的方向而言)
 输入参数: MOVE_DIRECTION_ENUM &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_local_move_direction(MOVE_DIRECTION_ENUM &data)
{
	data = district_.direction;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.test_local_move_direction_is_forward
 功能描述  : 检测当前运行方向是否为向前
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_walk_plan::test_local_move_direction_is_forward(void)
{
	bool ret = false;
	MOVE_DIRECTION_ENUM data;
	
	get_local_move_direction(data);
	if (FORWARD == data)
	{
		ret = true;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.test_local_move_direction_is_backward
 功能描述  : 检测当前运行方向是否为向后
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_walk_plan::test_local_move_direction_is_backward(void)
{
	bool ret = false;
	MOVE_DIRECTION_ENUM data;
	
	get_local_move_direction(data);
	if (BACKWARD == data)
	{
		ret = true;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.switch_local_move_direction
 功能描述  : 切换当前保存的运行方向
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::switch_local_move_direction(void)
{
	MOVE_DIRECTION_ENUM data;

	get_local_move_direction(data);
	if (FORWARD == data)
	{
		data = BACKWARD;
		debug_print_info("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<set_local_move_direction(BACKWARD)");
	}
	else
	{
		data = FORWARD;
		debug_print_info(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>set_local_move_direction(FORWARD)");
	}
	set_local_move_direction(data);
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.print_local_move_direction
 功能描述  : 打印当前的运行方向
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::print_local_move_direction(void)
{
	MOVE_DIRECTION_ENUM data;

	get_local_move_direction(data);
	if (FORWARD == data)
	{
		debug_print_fatal("FORWARD");
	}
	else
	{
		debug_print_fatal("BACKWARD");
	}
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_local_move_original_pose
 功能描述  : 设置局域行驶原始初始位置状态
 输入参数: const POSE_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_local_move_original_pose(const POSE_STRU data)
{
	district_.original_pose = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_local_move_original_pose
 功能描述  : 获取局域行驶原始初始位置状态
 输入参数: POSE_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_local_move_original_pose(POSE_STRU &data)
{
	data = district_.original_pose;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_local_move_planning_pose
 功能描述  : 设置局域行驶区域方框角的位置
 输入参数: const POSE_STRU data  
           int index             
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_local_move_planning_pose(const POSE_STRU &data, int index)
{
	district_.planning_pose[index] = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_local_move_planning_pose
 功能描述  : 获取局域行驶区域方框角的位置
 输入参数: POSE_STRU &data  
           int index        
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_local_move_planning_pose(POSE_STRU &data, int index)
{
	data = district_.planning_pose[index];
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_local_move_edge_length
 功能描述  : 获取局域行驶区域方框边长
 输入参数: 无
 输出参数: 无
 返 回 值: 边长
 
 修改历史:
  1.日     期: 2017年10月20日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_walk_plan::get_local_move_edge_length(void)
{
	return local_erea_edge_length_;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_local_move_half_edge_length
 功能描述  : 获取局域行驶区域方框边长的一半
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年10月30日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_walk_plan::get_local_move_half_edge_length(void)
{
	return local_erea_edge_length_/2.0;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_map_frame_area
 功能描述  : 设置地图最大外切虚拟框的大小
 输入参数: const AREA_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_map_frame_area(const AREA_STRU data)
{
	map_frame_ = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_map_frame_area
 功能描述  : 获取地图最大外切虚拟框的大小
 输入参数: AREA_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_map_frame_area(AREA_STRU &data)
{
	data = map_frame_;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_district_area
 功能描述  : 设置局部运动局域外框的大小
 输入参数: const AREA_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_district_area(const AREA_STRU data)
{
	district_ = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_district_area
 功能描述  : 获取局部运动局域外框的大小
 输入参数: AREA_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月23日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_district_area(AREA_STRU &data)
{
	data = district_;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_district_area_reference_x
 功能描述  : 设置X轴当前的参考比较值
 输入参数: const double data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_district_area_reference_x(const double data)
{
	district_.reference_x = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_district_area_reference_x
 功能描述  : 获取X轴当前的参考比较值
 输入参数: double &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_district_area_reference_x(double &data)
{
	data = district_.reference_x;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_district_area_reference_y
 功能描述  : 设置Y轴当前的参考比较值
 输入参数: const double data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_district_area_reference_y(const double data)
{
	district_.reference_y = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_district_area_reference_y
 功能描述  : 获取Y轴当前的参考比较值
 输入参数: double &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_district_area_reference_y(double &data)
{
	data = district_.reference_y;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_district_area_reference_angle
 功能描述  : 设置参考角度
 输入参数: const double data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_district_area_reference_angle(const double data)
{
	district_.reference_angle = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_district_area_reference_angle
 功能描述  : 获取参考角度
 输入参数: double &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_district_area_reference_angle(double &data)
{
	data = district_.reference_angle;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_district_area_precision
 功能描述  : 获取比较精度
 输入参数: double &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_district_area_precision(double &data)
{
	data = district_.precision;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.updata_district_area
 功能描述  : 更新局域四个角的位置点
 输入参数: const POSE_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年8月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::updata_district_area(const POSE_STRU &data)
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

	function_enable = test_local_move_is_enable();
	if (true == function_enable)
	{
		get_local_move_state(state);
		if (LOCAL_MOVE_STOP == state)
		{
			set_local_move_original_pose(data);
			
			get_local_move_mode(mode);
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
					set_local_move_planning_pose(pose[i], i); 
				}
				debug_print_warnning("pose[%d].point=(%lf, %lf)   pose[%d].point=(%lf, %lf)", 1, pose[1].point.x, pose[1].point.y, 0, pose[0].point.x, pose[0].point.y);
				debug_print_warnning("pose[%d].point=(%lf, %lf)   pose[%d].point=(%lf, %lf)", 2, pose[2].point.x, pose[2].point.y, 3, pose[3].point.x, pose[3].point.y);
				hypotenuse = length;                      //三角形斜边
				opposite = hypotenuse*sin((M_PI/180.0)*data.angle);    //三角形对边
				adjacent = hypotenuse*cos((M_PI/180.0)*data.angle);    //三角形邻边
				debug_print_warnning("hypotenuse=%lf; opposite=%lf; adjacent=%lf;", hypotenuse, opposite, adjacent);
				start.point.x = x0 - adjacent;
				start.point.y = y0 - opposite;
				start.angle = data.angle;
				end.point.x = x0 + adjacent;
				end.point.y = y0 + opposite;
				end.angle = data.angle;
				debug_print_warnning("start.point = {(%lf, %lf) : %lf}", start.point.x, start.point.y, start.angle);
				debug_print_warnning("end.point   = {(%lf, %lf) : %lf}", end.point.x, end.point.y, end.angle);
				set_refer_line_start_point_pose(start);
				set_refer_line_end_point_pose(end);

				state = LOCAL_MOVE_START;
				set_local_move_state(state);
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
 函 数 名: cfg_walk_plan.set_refer_line_start_point_pose
 功能描述  : 设置参考线（第一条行驶线路）的开始点
 输入参数: const POSE_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_refer_line_start_point_pose(const POSE_STRU data)
{
	ref_start_point_ = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_refer_line_start_point_pose
 功能描述  : 获取参考线（第一条行驶线路）的开始点
 输入参数: POSE_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_refer_line_start_point_pose(POSE_STRU &data)
{
	data = ref_start_point_;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_refer_line_end_point_pose
 功能描述  : 设置参考线（第一条行驶线路）的结束点
 输入参数: const POSE_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_refer_line_end_point_pose(const POSE_STRU data)
{
	ref_end_point_ = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_refer_line_end_point_pose
 功能描述  : 获取参考线（第一条行驶线路）的结束点
 输入参数: POSE_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_refer_line_end_point_pose(POSE_STRU &data)
{
	data = ref_end_point_;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_straight_moving_refer_start_pose
 功能描述  : 设置直线行驶参考起点位置
 输入参数: const POSE_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_straight_moving_refer_start_pose(const POSE_STRU data)
{
	start_pose_ = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_straight_moving_refer_start_pose
 功能描述  : 获取直线行驶参考起点位置
 输入参数: POSE_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_straight_moving_refer_start_pose(POSE_STRU &data)
{
	data = start_pose_;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_straight_moving_refer_target_pose
 功能描述  : 设置直线行驶参考目标位置
 输入参数: const POSE_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_straight_moving_refer_target_pose(const POSE_STRU data)
{
	target_pose_ = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_straight_moving_refer_target_pose
 功能描述  : 获取直线行驶参考目标位置
 输入参数: POSE_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_straight_moving_refer_target_pose(POSE_STRU &data)
{
	data = target_pose_;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_straight_moving_refer_pose
 功能描述  : 获取直线行驶参考起始点和目标点位置
 输入参数: POSE_STRU &start   
           POSE_STRU &target  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_straight_moving_refer_pose(POSE_STRU &start,POSE_STRU &target)
{
	start = start_pose_;
	target = target_pose_;
}

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/


