/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: bll_traight_line_moving.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月18日
  最近修改:
  功能描述   : 直行行驶功能类定义
  函数列表:
  修改历史:
  1.日     期: 2017年12月18日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "bll_traight_line_moving.h"

#include <math.h>

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
/*****************************************************************************
 函 数 名: bll_traight_line_moving.bll_traight_line_moving
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_traight_line_moving
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_traight_line_moving::bll_traight_line_moving()
{

}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.~bll_traight_line_moving
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_traight_line_moving
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_traight_line_moving::~bll_traight_line_moving()
{

}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_flag
 功能描述  : 设置当前是否处于直行状态
 输入参数: const bool flag  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_flag(const bool flag)
{
	straight_line_moving_.flag = flag;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_flag
 功能描述  : 获取当前是否处于直行状态
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_traight_line_moving::get_traight_line_moving_flag(void)
{
	return straight_line_moving_.flag;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_start_pos
 功能描述  : 设置直行的起始方向位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_start_pos(const POSE_STRU pos)
{
	straight_line_moving_.start_pos = pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_start_pos
 功能描述  : 获取直行的起始方向位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::get_traight_line_moving_start_pos(POSE_STRU &pos)
{
	pos = straight_line_moving_.start_pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_target_pos
 功能描述  : 设置直行方向目标位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_target_pos(const POSE_STRU pos)
{
	straight_line_moving_.target_pos = pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_target_pos
 功能描述  : 获取直行方向目标位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::get_traight_line_moving_target_pos(POSE_STRU &pos)
{
	pos = straight_line_moving_.target_pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_current_pos
 功能描述  : 设置直行当前的位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_current_pos(const POSE_STRU pos)
{
	straight_line_moving_.current_pos = pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_current_pos
 功能描述  : 获取直行当前的位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::get_traight_line_moving_current_pos(POSE_STRU &pos)
{
	pos = straight_line_moving_.current_pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_direction
 功能描述  : 设置直行行驶的方向角度
 输入参数: const double direction  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_direction(const double direction)
{
	straight_line_moving_.direction = direction;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_direction
 功能描述  : 获取直行行驶的方向角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_traight_line_moving::get_traight_line_moving_direction(void)
{
	return straight_line_moving_.direction;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_distance
 功能描述  : 设置直线行驶的行程距离
 输入参数: const double distance  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_distance(const double distance)
{
	straight_line_moving_.travel_distance = distance;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_distance
 功能描述  : 获取直线行驶的行程距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_traight_line_moving::get_traight_line_moving_distance(void)
{
	return straight_line_moving_.travel_distance;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_finish_all_route_done
 功能描述  : 设置直线行驶是否已经到达目标点完成所有行程
 输入参数: const bool flag  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_finish_all_route_done(const bool flag)
{
	straight_line_moving_.all_route_done = flag;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_finish_all_route_done
 功能描述  : 获取直线行驶是否已经到达目标点完成所有行程
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_traight_line_moving::get_finish_all_route_done(void)
{
	return straight_line_moving_.all_route_done;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_finish_part_route_done
 功能描述  : 设置直线显示完成一部分的标志
 输入参数: const bool flag  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_finish_part_route_done(const bool flag)
{
	straight_line_moving_.part_route_done = flag;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_finish_part_route_done
 功能描述  : 获取直线显示完成一部分的标志
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月19日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool bll_traight_line_moving::get_finish_part_route_done(void)
{
	return straight_line_moving_.part_route_done;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.set_traight_line_moving_data
 功能描述  : 设置直行数据
 输入参数: const STRAIGHT_LINE_MOVING_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::set_traight_line_moving_data(const STRAIGHT_LINE_MOVING_STRU data)
{
	straight_line_moving_ = data;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_data
 功能描述  : 获取直行数据
 输入参数: STRAIGHT_LINE_MOVING_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::get_traight_line_moving_data(STRAIGHT_LINE_MOVING_STRU &data)
{
	data = straight_line_moving_;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_traight_line_moving_direction_angle
 功能描述  : 获取直线行驶的运行方向角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_traight_line_moving::get_traight_line_moving_direction_angle(void)
{
	return straight_line_moving_.start_pos.angle;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.clear_traight_line_moving_data
 功能描述  : 清除直行数据
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_traight_line_moving::clear_traight_line_moving_data(void)
{
	POSE_STRU pos;
	pos.angle = 0.0;
	pos.point.x = 0.0;
	pos.point.y = 0.0;
	straight_line_moving_.flag = false;
	straight_line_moving_.start_pos = pos;
	straight_line_moving_.target_pos = pos;
	straight_line_moving_.current_pos = pos;
}

/*****************************************************************************
 函 数 名: bll_traight_line_moving.get_distance
 功能描述  : 获取两点之间的距离
 输入参数: double x1  
           double y1  
           double x2  
           double y2  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double bll_traight_line_moving::get_distance(double x1, double y1, double x2, double y2)
{
	double x = 0.0;
	double y = 0.0;
	double hypotenuse = 0.0;
	
	x = fabs(x2 - x1);
	y = fabs(y2 - y1);
	
	if (0.0 == x)
	{
		hypotenuse = y;
	}
	else if (0.0 == y)
	{
		hypotenuse = x;
	}
	else
	{
		hypotenuse = sqrt(pow(x, 2) + pow(y, 2));            //斜边长度
	}

	return hypotenuse;
}


/******************************************************************************
 * 内部函数声明
 ******************************************************************************/


