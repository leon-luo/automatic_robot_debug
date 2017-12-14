/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: angle_base.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月14日
  最近修改:
  功能描述   : 角度相关基本功能类定义
  函数列表:
  修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "angle_base.h"

#include "math.h"

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
/*****************************************************************************
 函 数 名: angel_base.get_angle
 功能描述  : 获取原点到点的角度
 输入参数: double x  
           double y  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double angel_base::get_angle(double x, double y)
{

}

/*****************************************************************************
 函 数 名: angel_base.get_angle
 功能描述  : 获取点（x1,y1）指向点（x2,y2）的角度
 输入参数: double x1  
           double y1  
           double x2  
           double y2  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double angel_base::get_angle(double x1, double y1, double x2, double y2)
{
	double x = 0.0;
	double y = 0.0;
	double angle = 0.0;
	double radian = 0.0;
	
	x = x2 - x1;
	y = y2 - y1;
#if 0
	double temp = 0.0;
	double hypotenuse = 0.0;
	hypotenuse = sqrt(pow(x, 2) + pow(y, 2));            //斜边长度
	temp = x/hypotenuse;
	radian = acos(temp);                                 //求出弧度
	if (y<0)
	{
		angle = -angle;
	}
	else if ((y == 0) && (x < 0))
	{
		angle = 180.0;
	}
#else
	//atan2（y，x）求的是y/x的反正切，其返回值为[-π, +π]之间的一个数。
	radian = atan2(y, x);
	angle = 180.0/(M_PI/radian);                             //用弧度算出角度

#endif /* #if 0 */
	return angle;
}

/*****************************************************************************
 函 数 名: angel_base.get_quadrant
 功能描述  : 获取角度所在的象限
 输入参数: double angle  
 输出参数: 无
 返 回 值: QUADRANT_ENUM
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
QUADRANT_ENUM angel_base::get_quadrant(double angle)
{
	QUADRANT_ENUM quadrant;
	
	if ((0.0 <= angle) && (angle <= 90.0))
	{
		quadrant = QUADRANT_1;
	}
	else if ((90.0 < angle) && (angle <= 180.0))
	{
		quadrant = QUADRANT_2;
	}
	else if ((-90.0 <= angle) && (angle < 0.0))
	{
		quadrant = QUADRANT_4;
	}
	else if ((-180.0 < angle) && (angle < -90.0))
	{
		quadrant = QUADRANT_3;
	}
	
	return quadrant;
}


/*****************************************************************************
 函 数 名: angel_base.format_angle
 功能描述  : 格式化角度在合理的范围
 输入参数: double angle  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double angel_base::format_angle(double angle)
{
	//把角度规划到-180至180之间
	const double pi_angle = 180.0;
	if (angle > pi_angle)
		angle = angle - 2*pi_angle;
	else if (angle < (-1.0*pi_angle))
		angle = angle + 2*pi_angle;
	
	return angle;
}

/*****************************************************************************
 函 数 名: angel_base.change_angle
 功能描述  : 改变角度
 输入参数: double angle         
           double change_value  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double angel_base::change_angle(double angle, double change_value)
{
	double temp = 0.0;
	double ret = 0.0;
	
	temp = angle + change_value;
	ret = format_angle(temp);
	
	return ret;
}

/*****************************************************************************
 函 数 名: angel_base.get_reverse_angle
 功能描述  : 获取反向角度
 输入参数: double angle  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double angel_base::get_reverse_angle(double angle)
{
	double ret = 0.0;
	const double change_value = 180.0;
		
	ret = change_angle(angle, change_value);
		
	return ret;
}

/*****************************************************************************
 函 数 名: angel_base.get_right_angle_clockwise
 功能描述  : 获取顺时针旋转一个直角的角度
 输入参数: double angle  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double angel_base::get_right_angle_clockwise(double angle)
{
	double ret = 0.0;
	const double change_value = 90.0;
	
	ret = change_angle(angle, change_value);
	
	return ret;
}

/*****************************************************************************
 函 数 名: angel_base.get_right_angle_anticlockwise
 功能描述  : 获取逆时针旋转一个直角的角度
 输入参数: double angle  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double angel_base::get_right_angle_anticlockwise(double angle)
{
	double ret = 0.0;
	const double change_value = -90.0;
	
	ret = change_angle(angle, change_value);
	
	return ret;
}

/*****************************************************************************
 函 数 名: angel_base.get_right_angle
 功能描述  : 获取指定角度的垂直角度
 输入参数: double angle                        
           ROTATIONAL_MOVEMENT_ENUM direction  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double angel_base::get_right_angle(double angle, ROTATIONAL_MOVEMENT_ENUM direction)
{
	double ret = 0.0;
	
	if ( CLOCKWISE == direction )
	{
		ret = get_right_angle_clockwise(angle);
	}
	else if( ANTICLOCKWISE == direction )
	{
		ret = get_right_angle_anticlockwise(angle);
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: angel_base.test_angle_is_over_clockwise
 功能描述  : 设置顺时针旋转方向角度是否已经超过目标角度
 输入参数: double current  
           double target   
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool angel_base::test_angle_is_over_clockwise(double current, double target)
{
	double temp = 0.0;
	bool over_flag = false;
	QUADRANT_ENUM curr_quadrant;
	QUADRANT_ENUM target_quadrant;

	curr_quadrant = get_quadrant(current);
	target_quadrant = get_quadrant(target);
	if ((current >= 0.0) && (target >= 0.0))
	{
		if (current < target)
		{
			over_flag = true;
		}
	}
	else if ((current < 0.0) && (target < 0.0))
	{
		if (current < target)
		{
			over_flag = true;
		}
	}
	else
	{
		temp = fabs(current) + fabs(target);
		
		if ((QUADRANT_1 == curr_quadrant) && (QUADRANT_4 == target_quadrant))
		{
			over_flag = false;
		}
		else if ((QUADRANT_4 == curr_quadrant) && (QUADRANT_1 == target_quadrant))
		{
			over_flag = true;
		}
		else if ((QUADRANT_3 == curr_quadrant) && (QUADRANT_2 == target_quadrant))
		{
			over_flag = false;
		}
		else if ((QUADRANT_2 == curr_quadrant) && (QUADRANT_3 == target_quadrant))
		{
			over_flag = true;
		}
		
		else if ((QUADRANT_1 == curr_quadrant) && (QUADRANT_3 == target_quadrant))
		{
			if ( temp > 180.0 ) 
			{
				over_flag = true;
			}
		}
		else if ((QUADRANT_3 == curr_quadrant) && (QUADRANT_1 == target_quadrant))
		{
			if ( temp < 180.0 ) 
			{
				over_flag = true;
			}
		}
		else if ((QUADRANT_2 == curr_quadrant) && (QUADRANT_4 == target_quadrant))
		{
			if ( temp > 180.0 ) 
			{
				over_flag = true;
			}
		}
		else if ((QUADRANT_4 == curr_quadrant) && (QUADRANT_2 == target_quadrant))
		{
			if ( temp < 180.0 ) 
			{
				over_flag = true;
			}
		}
	}

	return over_flag;
}

/*****************************************************************************
 函 数 名: angel_base.test_angle_is_over_anticlockwise
 功能描述  :  设置逆时针旋转方向角度是否已经超过目标角度
 输入参数: double current  
           double target   
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool angel_base::test_angle_is_over_anticlockwise(double current, double target)
{
	double temp = 0.0;
	bool over_flag = false;
	QUADRANT_ENUM curr_quadrant;
	QUADRANT_ENUM target_quadrant;

	curr_quadrant = get_quadrant(current);
	target_quadrant = get_quadrant(target);
	if ((current >= 0.0) && (target >= 0.0))
	{
		if (current > target)
		{
			over_flag = true;
		}
	}
	else if ((current < 0.0) && (target < 0.0))
	{
		if (current > target)
		{
			over_flag = true;
		}
	}
	else
	{
		temp = fabs(current) + fabs(target);
		
		if ((QUADRANT_1 == curr_quadrant) && (QUADRANT_4 == target_quadrant))
		{
			over_flag = true;
		}
		else if ((QUADRANT_4 == curr_quadrant) && (QUADRANT_1 == target_quadrant))
		{
			over_flag = false;
		}
		else if ((QUADRANT_3 == curr_quadrant) && (QUADRANT_2 == target_quadrant))
		{
			over_flag = true;
		}
		else if ((QUADRANT_2 == curr_quadrant) && (QUADRANT_3 == target_quadrant))
		{
			over_flag = false;
		}
		
		else if ((QUADRANT_1 == curr_quadrant) && (QUADRANT_3 == target_quadrant))
		{
			if ( temp < 180.0 ) 
			{
				over_flag = true;
			}
		}
		else if ((QUADRANT_3 == curr_quadrant) && (QUADRANT_1 == target_quadrant))
		{
			if ( temp > 180.0 ) 
			{
				over_flag = true;
			}
		}
		else if ((QUADRANT_2 == curr_quadrant) && (QUADRANT_4 == target_quadrant))
		{
			if ( temp < 180.0 ) 
			{
				over_flag = true;
			}
		}
		else if ((QUADRANT_4 == curr_quadrant) && (QUADRANT_2 == target_quadrant))
		{
			if ( temp > 180.0 ) 
			{
				over_flag = true;
			}
		}
	}
	
	return over_flag;
}

/*****************************************************************************
 函 数 名: angel_base.test_angle_rotate_direction_is_clockwise
 功能描述  : 测试靠近目标角度是否为顺时针旋转最近
 输入参数: double start   
           double target  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool angel_base::test_angle_rotate_direction_is_clockwise(double start, double target)
{
	bool flag = false;
	bool temp = 0.0;
	QUADRANT_ENUM start_quadrant;
	QUADRANT_ENUM target_quadrant;

	if ((start >= 0.0) && (target >= 0.0))
	{
		if (start > target)
		{
			flag = true;
		}
	}
	else if ((start < 0.0) && (target < 0.0))
	{
		if (start > target)
		{
			flag = true;
		}
	}
	else
	{
		start_quadrant = get_quadrant(start);
		target_quadrant = get_quadrant(target);
		if ((QUADRANT_1 == start_quadrant) && (QUADRANT_4 == target_quadrant))
		{
			flag = true;
		}
		else if ((QUADRANT_4 == start_quadrant) && (QUADRANT_1 == target_quadrant))
		{
			flag = false;
		}
		else if ((QUADRANT_3 == start_quadrant) && (QUADRANT_2 == target_quadrant))
		{
			flag = true;
		}
		else if ((QUADRANT_2 == start_quadrant) && (QUADRANT_3 == target_quadrant))
		{
			flag = false;
		}
		else if ((QUADRANT_1 == start_quadrant) && (QUADRANT_3 == target_quadrant))
		{
			temp = fabs(start) + fabs(target);
			if (180.0 > temp)
			{
				flag = true;
			}
		}
		else if ((QUADRANT_3 == start_quadrant) && (QUADRANT_1 == target_quadrant))
		{
			temp = fabs(start) + fabs(target);
			if (180.0 < temp)
			{
				flag = true;
			}
		}
		else if ((QUADRANT_2 == start_quadrant) && (QUADRANT_4 == target_quadrant))
		{
			temp = fabs(start) + fabs(target);
			if (180.0 > temp)
			{
				flag = true;
			}
		}
		else if ((QUADRANT_4 == start_quadrant) && (QUADRANT_2 == target_quadrant))
		{
			temp = fabs(start) + fabs(target);
			if (180.0 < temp)
			{
				flag = true;
			}
		}
	}

	return flag;
}

/*****************************************************************************
 函 数 名: angel_base.get_angle_differences
 功能描述  : 获取连个方向的夹角（锐角）
 输入参数: double angle1  
           double angle2  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月14日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double angel_base::get_angle_differences(double angle1, double angle2)
{
	double temp = 0.0;
	double offset = 0.0;

	temp = angle1*angle2;
	if (temp > 0.0)
	{
		offset = fabs(angle1 - angle2);
		//debug_print_info("offset = fabs(angle1=%lf - angle2=%lf) = %lf", angle1, angle2, offset);
	}
	else
	{
		temp = fabs(angle1) + fabs(angle2);
		if (temp < 180.0)
		{
			offset = temp;
			//debug_print_info("angle1=%lf; angle2=%lf; offset = temp = %lf", angle1, angle2, offset);
		}
		else if ((180.0 <= temp) && (temp <= 360.0))
		{
			offset = 360.0 - temp;
			//debug_print_fatal("angle1=%lf; angle2=%lf; offset = 360.0 - temp(%lf) = %lf", angle1, angle2, temp, offset);
		}
		else if(temp > 360.0)
		{
			offset = fmod( temp, 360.0 );
			//debug_print_warnning("angle1=%lf; angle2=%lf; offset = fmod( temp(%lf), 360.0 ) = %lf", angle1, angle2, temp, offset);
		}
	}
	
	return offset;
}

