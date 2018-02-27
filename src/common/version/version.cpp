/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: version.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月15日
  最近修改:
  功能描述   : 版本信息相关
  函数列表:
  修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "version.h"

#include <iostream>

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
pthread_mutex_t version::mutex_;
version* version::p_instance_ = nullptr;

/*****************************************************************************
 函 数 名: version.get_instance
 功能描述  : 获取实例
 输入参数: void  
 输出参数: 无
 返 回 值: version*
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
version* version::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock( &mutex_ );
		if (nullptr == p_instance_)
			p_instance_ = new version();
		pthread_mutex_unlock( &mutex_ );
	}
	
	return p_instance_;
}

/*****************************************************************************
 函 数 名: version.release_instance
 功能描述  : 释放实例
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void version::release_instance(void)
{
	if (nullptr != p_instance_)
	{
		pthread_mutex_lock( &mutex_ );
		if (nullptr != p_instance_)
		{
			delete p_instance_;
			p_instance_ = nullptr;
		}
		pthread_mutex_unlock( &mutex_ );
	}
}

/*****************************************************************************
 函 数 名: version.check_valid_fields
 功能描述  : 检查域字段是否有效
 输入参数: const uint32_t fields  
 输出参数: 无
 返 回 值: bool 有效则返回真，否则返回假
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool version::check_valid_fields(const uint32_t fields)
{
	bool ret = false;
	const uint32_t min = 0;
	const uint32_t max = version_fields_ -1;
	
	if ((min <= fields) && (fields <= max))
	{
		ret = true;
		
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: version.get_version_fields
 功能描述  : 获取版本信息字段数
 输入参数: 无
 输出参数: 无
 返 回 值: uint32_t
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
uint32_t version::get_version_fields(void)
{
	return version_fields_;
}

/*****************************************************************************
 函 数 名: version.set_version_info
 功能描述  : 设置版本信息
 输入参数: const VERSION_STRU version  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void version::set_version_info(const VERSION_STRU version)
{
	version_ = version;
}

/*****************************************************************************
 函 数 名: version.get_version_info
 功能描述  : 获取版本信息
 输入参数: VERSION_STRU &version  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void version::get_version_info(VERSION_STRU &version)
{
	version = version_;
}

/*****************************************************************************
 函 数 名: version.set_firmware_version_info
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
bool version::set_firmware_version_info(const uint32_t fields, const uint32_t value)
{
	bool ret = false;
	
	ret = check_valid_fields(fields);
	if (true == ret)
	{
		version_.firmware[fields] = value;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: version.get_firmware_version_info
 功能描述  : 获取软件版本指定字段的信息
 输入参数: const uint32_t fields  
 输出参数: uint32_t &value
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool version::get_firmware_version_info(const uint32_t fields, uint32_t &value)
{
	bool ret = false;
	
	ret = check_valid_fields(fields);
	if (true == ret)
	{
		value = version_.firmware[fields];
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: version.set_hardware_version_info
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
bool version::set_hardware_version_info(const uint32_t fields, const uint32_t value)
{
	bool ret = false;
	
	ret = check_valid_fields(fields);
	if (true == ret)
	{
		version_.hardware[fields] = value;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: version.get_hardware_version_info
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
bool version::get_hardware_version_info(const uint32_t fields, uint32_t &value)
{
	bool ret = false;
	
	ret = check_valid_fields(fields);
	if (true == ret)
	{
		value = version_.hardware[fields];
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: version.check_valid_serial_number
 功能描述  : 检查序列号的合法性
 输入参数: const string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool version::check_valid_serial_number(const string &str)
{
	bool ret = false;
	//待补充：添加检查序列号合理性策略
	if (true)
	{
		ret = true;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: version.set_serial_number
 功能描述  : 设置序列号
 输入参数: const string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool version::set_serial_number(const string &str)
{
	bool ret = false;
	
	ret = check_valid_serial_number(str);
	if (true == ret)
	{
		serial_number_ = str;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: version.get_serial_number
 功能描述  : 获取序列号
 输入参数: string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool version::get_serial_number(string &str)
{
	str = serial_number_;
	return true;
}

/*****************************************************************************
 函 数 名: version.set_uboot_version
 功能描述  : 设置引导程序版本信息
 输入参数: const string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool version::set_uboot_version(const string &str)
{
	uboot_version_ = str;
	return true;
}

/*****************************************************************************
 函 数 名: version.get_uboot_version
 功能描述  : 获取引导程序版本信息
 输入参数: string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool version::get_uboot_version(string &str)
{
	str = uboot_version_;
	return true;
}

/*****************************************************************************
 函 数 名: version.set_kernerl_version
 功能描述  : 设置内核版本信息
 输入参数: const string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool version::set_kernerl_version(const string &str)
{
	kernerl_version_ = str;
	return true;
}

/*****************************************************************************
 函 数 名: version.get_kernerl_version
 功能描述  : 获取内核版本信息
 输入参数: string &str  
 输出参数: 无
 返 回 值: bool
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool version::get_kernerl_version(string &str)
{
	str = kernerl_version_;
	return true;
}

/*****************************************************************************
 函 数 名: version.print_version
 功能描述  : 打印当前的版本信息
 输入参数: void  
 输出参数: 无
 返 回 值: void
 
 修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void version::print_version(void)
{
	std::cout<<std::endl;
	int i = 0;
	int max = version_fields_ - 1;
	string space_mark ("-");
	string uboot_version   ("* U-boot Version   : ");
	string kernerl_version ("* Kernerl Version  : ");
	string firmware_version("* Firmware Version : ");
	string hardware_version("* Hardware Version : ");
	string serial_version  ("* Serial Version   : ");
	string cut_off_rule("*******************************************************");
	std::cout<<cut_off_rule<<std::endl;
	std::cout<<uboot_version<<uboot_version_<<std::endl;
	std::cout<<kernerl_version<<kernerl_version_<<std::endl;
	
	std::cout<<hardware_version;
	for ( i = 0 ; i <= max ; i++ )
	{
		std::cout<<version_.hardware[i];
		if (i < max)
		{
			std::cout<<space_mark;
		}
		else
		{
			std::cout<<std::endl;
		}
	}
	
	std::cout<<firmware_version;
	for ( i = 0 ; i <= max ; i++ )
	{
		std::cout<<version_.firmware[i];
		if (i < max)
		{
			std::cout<<space_mark;
		}
		else
		{
			std::cout<<std::endl;
		}
	}
	
	std::cout<<serial_version<<serial_number_<<std::endl;
	std::cout<<cut_off_rule<<std::endl;
	std::cout<<std::endl;
}

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/



