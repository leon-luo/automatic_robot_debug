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
	set_district_area(district);

	disable_local_move();
	enable_local_move();
	
	set_partition_driving_mode(AMBIENT_NO_OBSTACLE);
	set_partition_driving_state(PARTITION_DRIVING_STOP);
	set_partition_driving_area_part(AREA_PART_LEFT);
	set_partition_driving_direction(FORWARD);
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
 函 数 名: cfg_walk_plan.test_partition_driving_is_enable
 功能描述  : 检测局部移动功能是否开启
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年9月4日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_walk_plan::test_partition_driving_is_enable(void)
{
	return district_.enable;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_partition_driving_mode
 功能描述  : 设置局域行驶模式
 输入参数: const PARTITION_DRIVING_ENUM data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_partition_driving_mode(const PARTITION_DRIVING_ENUM data)
{
	district_.mode = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_partition_driving_mode
 功能描述  : 获取局域行驶模式
 输入参数: PARTITION_DRIVING_ENUM &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_partition_driving_mode(PARTITION_DRIVING_ENUM &data)
{
	data = district_.mode;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_partition_driving_state
 功能描述  : 设置局域行驶状态
 输入参数: const PARTITION_DRIVING_STATE_ENUM data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_partition_driving_state(const PARTITION_DRIVING_STATE_ENUM data)
{
	district_.state = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_partition_driving_state
 功能描述  : 获取局域行驶状态
 输入参数: PARTITION_DRIVING_STATE_ENUM &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_partition_driving_state(PARTITION_DRIVING_STATE_ENUM &data)
{
	data = district_.state;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_partition_driving_area_part
 功能描述  : 设置当前所在的左右半区局域
 输入参数: const AREA_PART_ENUM data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_partition_driving_area_part(const AREA_PART_ENUM data)
{
	district_.area_part = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_partition_driving_area_part
 功能描述  : 获取当前所在的左右半区局域
 输入参数: AREA_PART_ENUM &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_partition_driving_area_part(AREA_PART_ENUM &data)
{
	data = district_.area_part;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.test_partition_driving_area_part_is_left
 功能描述  : 检测是否在左边半区局域
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_walk_plan::test_partition_driving_area_part_is_left(void)
{
	bool ret = false;
	AREA_PART_ENUM data;
	
	get_partition_driving_area_part(data);
	if (AREA_PART_LEFT == data)
	{
		ret = true;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.switch_partition_driving_area_part
 功能描述  : 切换当前所在的半区局域
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::switch_partition_driving_area_part(void)
{
	AREA_PART_ENUM data;

	get_partition_driving_area_part(data);
	if (AREA_PART_LEFT == data)
	{
		data = AREA_PART_RIGHT;
	}
	else
	{
		data = AREA_PART_LEFT;
	}
	set_partition_driving_area_part(data);
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_partition_driving_direction
 功能描述  : 设置当前运行的方向向前还是返回(相对于第一次平行直线行驶的方向而言)
 输入参数: const MOVE_DIRECTION_ENUM data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_partition_driving_direction(const MOVE_DIRECTION_ENUM data)
{
	district_.direction = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_partition_driving_direction
 功能描述  :  获取当前运行的方向向前还是返回(相对于第一次平行直线行驶的方向而言)
 输入参数: MOVE_DIRECTION_ENUM &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_partition_driving_direction(MOVE_DIRECTION_ENUM &data)
{
	data = district_.direction;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.test_partition_driving_direction_is_forward
 功能描述  : 检测当前运行方向是否为向前
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_walk_plan::test_partition_driving_direction_is_forward(void)
{
	bool ret = false;
	MOVE_DIRECTION_ENUM data;
	
	get_partition_driving_direction(data);
	if (FORWARD == data)
	{
		ret = true;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.switch_partition_driving_direction
 功能描述  : 切换当前保存的运行方向
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::switch_partition_driving_direction(void)
{
	MOVE_DIRECTION_ENUM data;

	get_partition_driving_direction(data);
	if (FORWARD == data)
	{
		data = BACKWARD;
	}
	else
	{
		data = FORWARD;
	}
	set_partition_driving_direction(data);
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.print_partition_driving_direction
 功能描述  : 打印当前的运行方向
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月13日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::print_partition_driving_direction(void)
{
	MOVE_DIRECTION_ENUM data;

	get_partition_driving_direction(data);
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
 函 数 名: cfg_walk_plan.set_partition_driving_original_pose
 功能描述  : 设置局域行驶原始初始位置状态
 输入参数: const POSE_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::set_partition_driving_original_pose(const POSE_STRU data)
{
	district_.original_pose = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_partition_driving_original_pose
 功能描述  : 获取局域行驶原始初始位置状态
 输入参数: POSE_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年9月1日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_walk_plan::get_partition_driving_original_pose(POSE_STRU &data)
{
	data = district_.original_pose;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.set_partition_driving_planning_pose
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
void cfg_walk_plan::set_partition_driving_planning_pose(const POSE_STRU &data, int index)
{
	district_.planning_pose[index] = data;
}

/*****************************************************************************
 函 数 名: cfg_walk_plan.get_partition_driving_planning_pose
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
void cfg_walk_plan::get_partition_driving_planning_pose(POSE_STRU &data, int index)
{
	data = district_.planning_pose[index];
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

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/


