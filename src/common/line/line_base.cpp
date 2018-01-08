/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: line_base.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月25日
  最近修改:
  功能描述   : 点和线相关类定义
  函数列表:
  修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "line_base.h"

#include "angle_base.h"

#include <math.h>
#include <stdio.h>

#include "debug_function.h"

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
/*****************************************************************************
 函 数 名: line_base.line_base
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: line_base
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
line_base::line_base()
{

}

/*****************************************************************************
 函 数 名: line_base.~line_base
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: line_base
 
 修改历史:
  1.日     期: 2017年12月26日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
line_base::~line_base()
{


}

/*****************************************************************************
 函 数 名: line_base.get_distance
 功能描述  : 获取两点之间的距离
 输入参数: double x1  
           double y1  
           double x2  
           double y2  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年12月25日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double line_base::get_distance(double x1, double y1, double x2, double y2)
{
	double x = 0.0;
	double y = 0.0;
	double hypotenuse = 0.0;
	//bool flag = false;
	//static double temp = 0.0;
	//if (temp != x1)
	//{
	//	temp = x1;
	//	flag = true;
	//}
	
	x = fabs(x2 - x1);
	y = fabs(y2 - y1);
	
	if (0.0 == x)
	{
		hypotenuse = y;
	//	if (true == flag)
	//		debug_print_info("%lf=fabs(y2=%lf - y1=%lf);",hypotenuse, x2, x1);
	}
	else if (0.0 == y)
	{
		hypotenuse = x;
	//	if (true == flag)
	//		debug_print_info("%lf=fabs(x2=%lf - x1=%lf);",hypotenuse, x2, x1);
	}
	else
	{
		hypotenuse = sqrt(pow(x, 2) + pow(y, 2));            //斜边长度
	//	if (true == flag)
	//		debug_print_info("%lf=sqrt(pow(x=%lf, 2) + pow(y=%lf, 2));",hypotenuse, x, y);
	}

	return hypotenuse;
}

/*****************************************************************************
 函 数 名: line_base.get_vertical_distance_curr_position_to_refer_line
 功能描述  :  获取点(x,y)到连接点(x1,y1)和点(x2,y2)的直线的垂直距离
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月17日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double line_base::get_vertical_distance(double x, double y, double x1, double y1, double x2, double y2)
{
	double temp = 0.0;
	double len1 = 0.0;
	double len2 = 0.0;
	double radian = 0.0;
	double diff_angle = 0.0;
	double acute_angle = 0.0;
	double line_angel = 0.0;
	double hypotenuse_angel = 0.0;
	double opposite = 0.0;    //三角形对边
	double hypotenuse = 0.0;  //三角形斜边
	angel_base angel_base_instance;
	//debug_print_info("P0(%lf, %lf) to line{P1(%lf, %lf)---P2(%lf, %lf)}", x, y, x1, y1, x2, y2);
	len1 = get_distance(x, y, x1, y1);
	len2 = get_distance(x, y, x2, y2);
	if (len1 >= len2)
	{
		line_angel = angel_base_instance.get_angle(x1, y1, x2, y2);
		hypotenuse_angel = angel_base_instance.get_angle(x1, y1, x, y);
		hypotenuse = len1;
		//debug_print_warnning("len1(%lf)>=len2(%lf) --- hypotenuse(%lf)}", len1, len2, hypotenuse);
	}
	else
	{
		line_angel = angel_base_instance.get_angle(x2, y2, x1, y1);
		hypotenuse_angel = angel_base_instance.get_angle(x2, y2, x, y);
		hypotenuse = len2;
		//debug_print_warnning("len1(%lf)<len2(%lf) --- hypotenuse(%lf)}", len1, len2, hypotenuse);
	}
	
	diff_angle = angel_base_instance.get_angle_differences(line_angel, hypotenuse_angel);
	//debug_print_info("| line_angel(%lf) - hypotenuse_angel(%lf) | = diff_angle(%lf)}", line_angel, hypotenuse_angel, diff_angle);
	acute_angle = angel_base_instance.convert_to_acute_angle(diff_angle);
	radian = angel_base_instance.convert_degrees_to_radians(acute_angle);
	//debug_print_info("diff_angle(%lf) => acute_angle(%lf) => radian(%lf)", acute_angle, diff_angle, radian);
	temp = sin(radian);
	opposite = hypotenuse*temp;
	//debug_print_info("opposite = hypotenuse*sin(radian=%lf) = %lf * %lf = %lf;",radian, hypotenuse, temp, opposite);
	return opposite;
}

/******************************************************************************
 * 内部函数定义
 ******************************************************************************/


