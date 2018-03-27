/******************************************************************************

  版权所有 (C), 2017-2028 

 ******************************************************************************
  文件名称: cfg_modulate.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年11月9日
  最近修改:
  功能描述: cfg_modulate.cpp 的头文件
  函数列表:
  修改历史:
  1.日     期: 2017年11月9日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/
#ifndef __CFG_MODULATE_H__
#define __CFG_MODULATE_H__

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
#include <pthread.h>

#include "cfg_base_type.h"
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
typedef struct VELOCITY_AJUST
{
	bool angular_flag;
	bool linear_flag;
	double angular_velocity;
	double linear_velocity;
}VELOCITY_AJUST_STRU;

/******************************************************************************
 * 类声明
 ******************************************************************************/

class cfg_modulate
{
public:
	static cfg_modulate* get_instance(void);
	static void release_instance(void);

	void set_velocity_ajust(const VELOCITY_AJUST_STRU data);
	void get_velocity_ajust(VELOCITY_AJUST_STRU &data);
	
	void set_angular_velocity_flag(bool data);
	bool get_angular_velocity_flag(void);
	
	void set_angular_velocity_value(double value);
	double get_angular_velocity_value(void);
	
	void set_angular_velocity_ajust(bool flag, double value);
	bool get_angular_velocity_ajust(double &value);

	void set_linear_velocity_ajust(bool flag, double value);
	bool get_linear_velocity_ajust(double &value);
	
	void disable_angular_velocity_ajust(void);
	void endble_angular_velocity_ajust(double velocity);

	void disable_linear_velocity_ajust(void);
	void endble_linear_velocity_ajust(double velocity);

protected:
	cfg_modulate();
	~cfg_modulate();

private:
	cfg_modulate(const cfg_modulate&){};                              //禁止拷贝
	cfg_modulate& operator=(const cfg_modulate&){};                   //禁止赋值
	
	static cfg_modulate* volatile p_instance_;
	static pthread_mutex_t mutex_;
	
	VELOCITY_AJUST_STRU velocity_ajust_;

	//pid angular_velocity_pid_;
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

#endif /* __CFG_MODULATE_H__ */
