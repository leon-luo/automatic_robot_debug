/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: bll_trajectory.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月21日
  最近修改:
  功能描述   : 显示轨迹功能类定义
  函数列表:
  修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "bll_trajectory.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>

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
pthread_mutex_t bll_trajectory::mutex_;
bll_trajectory* bll_trajectory::p_instance_ = NULL;

/*****************************************************************************
 函 数 名: bll_trajectory.bll_trajectory
 功能描述  : 构成函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_trajectory
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_trajectory::bll_trajectory()
{

}

/*****************************************************************************
 函 数 名: bll_trajectory.~bll_trajectory
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: bll_trajectory
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_trajectory::~bll_trajectory()
{

}

/*****************************************************************************
 函 数 名: bll_trajectory.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: bll_trajectory*
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bll_trajectory* bll_trajectory::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr == p_instance_)
		{
			p_instance_ = new bll_trajectory();
		}
		pthread_mutex_unlock(&mutex_);
	}
	return p_instance_;
}

/*****************************************************************************
 函 数 名: bll_trajectory.release_instance
 功能描述  : 释放实例
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_trajectory::release_instance(void)
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
 函 数 名: bll_trajectory.register_trajectory_msgs
 功能描述  : 注册发布行驶轨迹的主题消息
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_trajectory::register_trajectory_msgs(void)
{
	ros::NodeHandle node_h;
	pub_path_ = node_h.advertise<nav_msgs::Path>("trajectory", 1, true);
}

/*****************************************************************************
 函 数 名: bll_trajectory.publish_trajectory
 功能描述  : 发布轨迹数据
 输入参数: double x   
           double y   
           double th  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_trajectory::publish_trajectory(double x, double y, double th)
{
	ros::Time current_time;
	static nav_msgs::Path path;
	geometry_msgs::PoseStamped pose_stamped;
	
	current_time = ros::Time::now();
	
	pose_stamped.header.stamp = current_time;
	pose_stamped.header.frame_id = "odom";
	pose_stamped.pose.position.x = x;
	pose_stamped.pose.position.y = y;

	geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
	pose_stamped.pose.orientation.x = goal_quat.x;
	pose_stamped.pose.orientation.y = goal_quat.y;
	pose_stamped.pose.orientation.z = goal_quat.z;
	pose_stamped.pose.orientation.w = goal_quat.w;
	
	path.header.stamp=current_time;
	path.header.frame_id="odom";
	path.poses.push_back(pose_stamped);
	
	pub_path_.publish(path);
}

/*****************************************************************************
 函 数 名: bll_trajectory.show_trajectory
 功能描述  : 显示轨迹
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void bll_trajectory::show_trajectory(void)
{
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;
	POSE_STRU pos;
	
	cfg_if_get_current_position(pos);
	x = pos.point.x;
	y = pos.point.y;
	th = pos.angle;
	publish_trajectory(x, y, th);
}

/******************************************************************************
 * 内部函数定义
 ******************************************************************************/


