/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: mobile_robot_main.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年8月3日
  最近修改:
  功能描述   : 扫地机器人
  函数列表:
              main
  修改历史:
  1.日     期: 2017年8月3日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include <ros/ros.h>

#include "drv_sensor.h"
#include "bll_processing_function.h"
#include "debug_function.h"
#include <string>

using namespace std;
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
 函 数 名: main
 功能描述  : 主函数入口
 输入参数: int argc     
           char** argv  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2017年8月11日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
int main(int argc, char** argv)
{
	string name_str ("cleaner_robot");
	ROS_INFO ( "[%s():%d:]Application(%s) running !", __FUNCTION__, __LINE__, name_str.c_str());
	
	ros::init ( argc, argv, name_str );
	
	drv_sensor* p_sensor_instance = drv_sensor::get_instance();
	p_sensor_instance->initialize();

	bll_processing_function* p_processing_function = bll_processing_function::get_instance();
	p_processing_function->initialize();
	while ( ros::ok() )
	{
		p_processing_function->function_processor();
		ros::spinOnce();
	}
	
	ROS_WARN ( "[%s():%d:]After while(ros::ok())", __FUNCTION__, __LINE__);
	ros::spin();
	ROS_ERROR ( "[%s():%d:]ros::spin();", __FUNCTION__, __LINE__);
	ros::shutdown();
	ROS_FATAL ( "[%s():%d:]ros::shutdown();", __FUNCTION__, __LINE__);
	
	return 0;
}

