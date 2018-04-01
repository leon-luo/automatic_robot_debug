/******************************************************************************

  版权所有 (C), 2017-2028, _ _ _ Co., Ltd.

 ******************************************************************************
  文件名称: cfg_if_version.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月27日
  最近修改:
  功能描述   : 版本相关对外接口函数IPA
  函数列表:
  修改历史:
  1.日     期: 2017年12月27日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "cfg_if_version.h"

#include "cfg_mobile_robot.h"

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
 函 数 名: cfg_if_check_valid_fields
 功能描述  : 检查域字段是否有效
 输入参数: const uint32_t fields  
 输出参数: 无
 返 回 值: bool 有效则返回真，否则返回假
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_check_valid_fields(const uint32_t fields)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->check_valid_fields(fields);
	
	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_get_version_fields
 功能描述  : 获取版本信息字段数
 输入参数: 无
 输出参数: 无
 返 回 值: uint32_t
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
uint32_t cfg_if_get_version_fields(void)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	return p_mobile_robot->get_version_fields();
}

/*****************************************************************************
 函 数 名: cfg_if_set_version_info
 功能描述  : 设置版本信息
 输入参数: const VERSION_STRU version  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_set_version_info(const VERSION_STRU version)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->set_version_info(version);
}

/*****************************************************************************
 函 数 名: cfg_if_get_version_info
 功能描述  : 获取版本信息
 输入参数: VERSION_STRU &version  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_get_version_info(VERSION_STRU &version)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->get_version_info(version);
}

/*****************************************************************************
 函 数 名: cfg_if_set_firmware_version_info
 功能描述  : 设置软件版本指定字段的信息
 输入参数: const uint32_t fields  
           const uint32_t value   
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_set_firmware_version_info(const uint32_t fields, const uint32_t value)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->set_firmware_version_info(fields, value);

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_get_firmware_version_info
 功能描述  : 获取软件版本指定字段的信息
 输入参数: const uint32_t fields  
 输出参数: uint32_t &value
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_get_firmware_version_info(const uint32_t fields, uint32_t &value)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->get_firmware_version_info(fields, value);

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_set_hardware_version_info
 功能描述  : 设置硬件版本指定字段的信息
 输入参数: const uint32_t fields  
           const uint32_t value   
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_set_hardware_version_info(const uint32_t fields, const uint32_t value)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->set_hardware_version_info(fields, value);

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_get_hardware_version_info
 功能描述  : 获取硬件版本指定字段的信息
 输入参数: const uint32_t fields  
           uint32_t &value        
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_get_hardware_version_info(const uint32_t fields, uint32_t &value)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->get_hardware_version_info(fields, value);

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_check_valid_serial_number
 功能描述  : 检查序列号的合法性
 输入参数: const string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_check_valid_serial_number(const string &str)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->check_valid_serial_number(str);

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_set_serial_number
 功能描述  : 设置序列号
 输入参数: const string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_set_serial_number(const string &str)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->set_serial_number(str);

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_get_serial_number
 功能描述  : 获取序列号
 输入参数: string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_get_serial_number(string &str)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->get_serial_number(str);

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_set_uboot_version
 功能描述  : 设置引导程序版本信息
 输入参数: const string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_set_uboot_version(const string &str)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->set_uboot_version(str);

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_get_uboot_version
 功能描述  : 获取引导程序版本信息
 输入参数: string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_get_uboot_version(string &str)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->get_uboot_version(str);

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_set_kernerl_version
 功能描述  : 设置内核版本信息
 输入参数: const string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_set_kernerl_version(const string &str)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->set_kernerl_version(str);

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_get_kernerl_version
 功能描述  : 获取内核版本信息
 输入参数: string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool cfg_if_get_kernerl_version(string &str)
{
	bool ret = false;
	
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	ret = p_mobile_robot->get_kernerl_version(str);

	return ret;
}

/*****************************************************************************
 函 数 名: cfg_if_print_version
 功能描述  : 打印当前的版本信息
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void cfg_if_print_version(void)
{
	cfg_mobile_robot* p_mobile_robot = cfg_mobile_robot::get_instance();
	p_mobile_robot->print_version();
}


