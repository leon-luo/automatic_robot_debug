/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: cfg_modulate.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年11月9日
  最近修改:
  功能描述: 调节功能
  函数列表:
  修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "cfg_modulate.h"

#include <stdlib.h>
#include <math.h>
#include "debug_function.h"
#include "pid.h"

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
cfg_modulate* volatile cfg_modulate::p_instance_ = nullptr;
pthread_mutex_t cfg_modulate::mutex_ = PTHREAD_MUTEX_INITIALIZER;

/*****************************************************************************
 函 数 名: cfg_modulate.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: cfg_modulate*
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
cfg_modulate* cfg_modulate::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr == p_instance_)
			p_instance_ = new cfg_modulate();
		pthread_mutex_unlock(&mutex_);
	}
	
	return p_instance_;
}

/*****************************************************************************
 函 数 名: cfg_modulate.release_instance
 功能描述  : 销毁实例
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::release_instance(void)
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
 函 数 名: cfg_modulate.cfg_modulate
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: cfg_modulate
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
cfg_modulate::cfg_modulate()
{
	PID_STRU pid;
	//debug_print_error("+++++++++++++");
	pid.goal_value = 0.0;               // 设定目标 desired value
	pid.proportion = 0.1;               // 比例常数 proportional const
	pid.integral = 0.0;                 // 积分常数 integral const
	pid.derivative = 0.4;               // 微分常数 derivative const
	pid.last_error = 0.0;               // error[-1]
	pid.prev_error = 0.0;               // error[-2]
	pid.sum_error = 0.0;                // sums of errors
	angular_velocity_pid_.set_pid(pid);

	//angular_velocity_pid_.print_pid_data();
	//debug_print_error("-------------");
}

/*****************************************************************************
 函 数 名: cfg_modulate.~cfg_modulate
 功能描述  : 析构函数
 输入参数  : 无
 输出参数: 无
 返 回 值: cfg_modulate
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
cfg_modulate::~cfg_modulate()
{

}

/*****************************************************************************
 函 数 名: cfg_modulate.set_velocity_ajust
 功能描述  : 设置速度调节状态数据
 输入参数: const VELOCITY_AJUST_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_velocity_ajust(const VELOCITY_AJUST_STRU data)
{
	velocity_ajust_ = data;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_velocity_ajust
 功能描述  : 获取速度调节状态数据
 输入参数: VELOCITY_AJUST_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::get_velocity_ajust(VELOCITY_AJUST_STRU &data)
{
	data = velocity_ajust_;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_angular_velocity_flag
 功能描述  : 设置角速度调节是否使能
 输入参数: bool data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_angular_velocity_flag(bool data)
{
	velocity_ajust_.angular_flag = data;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_angular_velocity_flag
 功能描述  : 获取角速度调节是否使能
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_modulate::get_angular_velocity_flag(void)
{
	return velocity_ajust_.angular_flag;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_angular_velocity_value
 功能描述  : 设置角速度调整值
 输入参数: double value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_angular_velocity_value(double value)
{
	velocity_ajust_.angular_velocity = value;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_angular_velocity_value
 功能描述  : 获取角速度调整值
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_modulate::get_angular_velocity_value(void)
{
	return velocity_ajust_.angular_velocity;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_angular_velocity_ajust
 功能描述  : 设置角速度调整当前状态
 输入参数: bool flag     
           double value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_angular_velocity_ajust(bool flag, double value)
{
	velocity_ajust_.angular_flag = flag;
	velocity_ajust_.angular_velocity = value;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_angular_velocity_ajust
 功能描述  : 获取角速度调整当前状态
 输入参数: double &value  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_modulate::get_angular_velocity_ajust(double &value)
{
	bool flag = false;

	flag = velocity_ajust_.angular_flag;
	if (true == flag)
	{
		value = velocity_ajust_.angular_velocity;
	}
	
	return flag;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_linear_velocity_ajust
 功能描述  : 设置线速度调整当前状态
 输入参数: bool flag     
           double value  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_linear_velocity_ajust(bool flag, double value)
{
	velocity_ajust_.linear_flag = flag;
	velocity_ajust_.linear_velocity = value;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_linear_velocity_ajust
 功能描述  : 获取线速度调整当前状态
 输入参数: double &value  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_modulate::get_linear_velocity_ajust(double &value)
{
	bool flag = false;

	flag = velocity_ajust_.linear_flag;
	if (true == flag)
	{
		value = velocity_ajust_.linear_velocity;
	}
	
	return flag;
}

/*****************************************************************************
 函 数 名: cfg_modulate.disable_angular_velocity_ajust
 功能描述  : 不启用速度调整功能
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月18日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::disable_angular_velocity_ajust(void)
{
	set_angular_velocity_ajust(false, 0.0);
}

/*****************************************************************************
 函 数 名: cfg_modulate.endble_angular_velocity_ajust
 功能描述  : 启用速度调整功能
 输入参数: double velocity  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月18日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::endble_angular_velocity_ajust(double velocity)
{
	set_angular_velocity_ajust(true, velocity);
}

/*****************************************************************************
 函 数 名: cfg_modulate.disable_linear_velocity_ajust
 功能描述  : 禁用线速度调整功能
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::disable_linear_velocity_ajust(void)
{
	set_linear_velocity_ajust(false, 0.0);
}

/*****************************************************************************
 函 数 名: cfg_modulate.endble_linear_velocity_ajust
 功能描述  : 启用线速度调整功能
 输入参数: double velocity  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月28日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::endble_linear_velocity_ajust(double velocity)
{
	set_linear_velocity_ajust(true, velocity);
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_angle_differences
 功能描述  : 获取连个方向的夹角（锐角）
 输入参数: double angle1  
           double angle2  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_modulate::get_angle_differences(double angle1, double angle2)
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

/*****************************************************************************
 函 数 名: cfg_modulate.get_quadrant
 功能描述  : 获取角度所在的象限
 输入参数: double angle  
 输出参数: 无
 返 回 值: QUADRANT_ENUM
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
QUADRANT_ENUM cfg_modulate::get_quadrant(double angle)
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
 函 数 名: cfg_modulate.test_angle_rotate_direction_is_clockwise
 功能描述  : 测试靠近目标角度是否为顺时针旋转最近
 输入参数: double start   
           double target  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_modulate::test_angle_rotate_direction_is_clockwise(double start, double target)
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
 函 数 名: cfg_modulate.ajust_angular_velocity
 功能描述  : 调节角速度
 输入参数: double real           实际当前角度
           double target     目标调整角度
           double &velocity  调整后的角速度
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月18日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_modulate::ajust_angular_velocity(double real, double target, double &velocity)
{
	double ret = false;
	double temp = 0.0;
	double rad = 0.0;
	double change = 0.0;
	double offset = 0.0;
	double effect = 0.0;
	const double max = 0.2;
	const double precision = 0.0001;
	const double step = 0.05;
	double rotate_trend = 1.0;//顺时针方向：rad>0，角度增大;  逆时针反向：rad<0，角度减小;
	
	if ((-precision <= target) && (target <= 0.0))
	{
		target = 0.0;
	}
	else if ((-180.0 <= target) && (target <= (-180.0+precision)))
	{
		target = 180.0;
	}

	offset = get_angle_differences(real, target);
	
	//PID_STRU pid;
	//angular_velocity_pid_.get_pid(pid);
	//rad = angular_velocity_pid_.pid_calc(pid, offset);

	//angular_velocity_pid_.print_pid_data();

	//printf("rad=%lf\n", rad);
	
	bool is_clockwise = test_angle_rotate_direction_is_clockwise(real, target);
	if (true == is_clockwise)
	{
		rotate_trend = -rotate_trend;
		//std::cout<<"effect="<<effect<<std::endl;
	}

	if (0.5 < offset)
	{
		ret = true;

		if(offset>=15){
			rad = offset/2.0;
		}else if (offset>=10){
			rad = offset/0.5;
		}else if (offset>=5){
			rad = offset/0.8;
		}else if (offset>=2){
			rad = offset/0.5;
		}else if (offset>=1){
			rad = offset/1;
		}else if (offset>=0.5){
			rad = offset/2;
		}else if (offset>=0.1){
			rad = offset/2;
		}
		else{
			rad = offset/2;
		}

		velocity = rotate_trend*rad ;
		
		if (velocity >= max) {
			velocity = max;
		} else if (velocity <= -max) {
			velocity = -max;
		}
		//debug_print_warnning("effect(%lf)  offset(%lf) = real(%lf) - target(%lf);  change(%lf);  velocity = %lf * %lf * %lf = %lf",effect, offset, real, target, change, rotate_trend, offset, step, velocity);
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_modulate.update_velocity
 功能描述  : 更新速度
 输入参数: POSE_STRU curr    
           POSE_STRU ref     
           POSE_STRU target  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::update_velocity(POSE_STRU curr, POSE_STRU ref, POSE_STRU target)
{
	bool flag = false;
	double rad = 0.0;
	double real = 0.0;
	double optimal = 0.0;
	static double prev_optimal = 0.0;

	real = get_angle(curr.point.x, curr.point.y, target.point.x, target.point.y);
	optimal = get_angle(ref.point.x, ref.point.y, target.point.x, target.point.y);
	//debug_print_info("angle: real    = {(%lf, %lf) ->  (%lf, %lf) } = (%lf)", curr.point.x, curr.point.y, target.point.x, target.point.y, real);

	if (prev_optimal != optimal)
	{
		prev_optimal = optimal;
		debug_print_info("angle: optimal = {(%lf, %lf) ->  (%lf, %lf) } = (%lf)", ref.point.x, ref.point.y, target.point.x, target.point.y, optimal);
	}
	
	//flag = ajust_angular_velocity(real, optimal, rad);
	//if (true == flag)
	//{
		//endble_angular_velocity_ajust(rad);
	//}
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_angle
 功能描述  : 获取点（x1,y1）指向点（x2,y2）的角度
 输入参数: double x1  
           double y1  
           double x2  
           double y2  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_modulate::get_angle(double x1, double y1, double x2, double y2)
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
 函 数 名: cfg_modulate.get_distance
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
double cfg_modulate::get_distance(double x1, double y1, double x2, double y2)
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

/*****************************************************************************
 函 数 名: cfg_modulate.set_traight_line_moving_flag
 功能描述  : 设置当前是否处于直行状态
 输入参数: const bool flag  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_traight_line_moving_flag(const bool flag)
{
	straight_line_moving_.flag = flag;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_traight_line_moving_flag
 功能描述  : 获取当前是否处于直行状态
 输入参数: void  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_modulate::get_traight_line_moving_flag(void)
{
	return straight_line_moving_.flag;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_traight_line_moving_start_pos
 功能描述  : 设置直行的起始方向位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_traight_line_moving_start_pos(const POSE_STRU pos)
{
	straight_line_moving_.start_pos = pos;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_traight_line_moving_start_pos
 功能描述  : 获取直行的起始方向位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::get_traight_line_moving_start_pos(POSE_STRU &pos)
{
	pos = straight_line_moving_.start_pos;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_traight_line_moving_target_pos
 功能描述  : 设置直行方向目标位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_traight_line_moving_target_pos(const POSE_STRU pos)
{
	straight_line_moving_.target_pos = pos;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_traight_line_moving_target_pos
 功能描述  : 获取直行方向目标位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::get_traight_line_moving_target_pos(POSE_STRU &pos)
{
	pos = straight_line_moving_.target_pos;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_traight_line_moving_current_pos
 功能描述  : 设置直行当前的位置
 输入参数: const POSE_STRU pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_traight_line_moving_current_pos(const POSE_STRU pos)
{
	straight_line_moving_.current_pos = pos;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_traight_line_moving_current_pos
 功能描述  : 获取直行当前的位置
 输入参数: POSE_STRU &pos  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::get_traight_line_moving_current_pos(POSE_STRU &pos)
{
	pos = straight_line_moving_.current_pos;
}

/*****************************************************************************
 函 数 名: cfg_modulate.set_traight_line_moving_data
 功能描述  : 设置直行数据
 输入参数: const STRAIGHT_LINE_MOVING_STRU data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::set_traight_line_moving_data(const STRAIGHT_LINE_MOVING_STRU data)
{
	straight_line_moving_ = data;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_traight_line_moving_data
 功能描述  : 获取直行数据
 输入参数: STRAIGHT_LINE_MOVING_STRU &data  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::get_traight_line_moving_data(STRAIGHT_LINE_MOVING_STRU &data)
{
	data = straight_line_moving_;
}

/*****************************************************************************
 函 数 名: cfg_modulate.get_traight_line_moving_direction_angle
 功能描述  : 获取直线行驶的运行方向角度
 输入参数: void  
 输出参数: 无
 返 回 值: double
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
double cfg_modulate::get_traight_line_moving_direction_angle(void)
{
	return straight_line_moving_.start_pos.angle;
}

/*****************************************************************************
 函 数 名: cfg_modulate.clear_traight_line_moving_data
 功能描述  : 清除直行数据
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年11月21日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_modulate::clear_traight_line_moving_data(void)
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

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/




