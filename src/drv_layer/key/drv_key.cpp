/******************************************************************************

  版权所有 (C), 2017-2028 

 ******************************************************************************
  文件名称: key_data.c
  版本编号: 初稿
  作     者: Leon
  生成日期: 2018年2月6日
  最近修改:
  功能描述   : 按键数据与功能函数定义
  函数列表:
  修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "key_data.h"

#include "sys.h"
#include "delay.h"

/******************************************************************************
 * 外部变量定义
 ******************************************************************************/

/******************************************************************************
 * 外部函数定义
 ******************************************************************************/

/******************************************************************************
 * 全局变量
 ******************************************************************************/
//按键数据保存
static KEY_DATA_STRU g_key_data;

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

/******************************************************************************
 * 内部函数定义
 ******************************************************************************/


/*****************************************************************************
 函 数 名: set_home_key_short_press
 功能描述  : 设置home按键短按状态
 输入参数: KEY_STATUS_ENUM data  
 输出参数: 无
 返 回 值: static
 
 修改历史:
  1.日     期: 2018年1月31日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void set_home_key_short_press(KEY_STATUS_ENUM data)
{
	g_key_data.home_key.short_press.status = data;
}

/*****************************************************************************
 函 数 名: get_home_key_short_press
 功能描述  : 获取home按键短按状态
 输入参数: void  
 输出参数: 无
 返 回 值: static
 
 修改历史:
  1.日     期: 2018年1月31日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
KEY_STATUS_ENUM get_home_key_short_press(void)
{
	return g_key_data.home_key.short_press.status;
}

/*****************************************************************************
 函 数 名: set_home_key_long_press
 功能描述  : 设置home按键短按状态
 输入参数: KEY_STATUS_ENUM data  
 输出参数: 无
 返 回 值: static
 
 修改历史:
  1.日     期: 2018年1月31日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void set_home_key_long_press(KEY_STATUS_ENUM data)
{
	g_key_data.home_key.long_press.status = data;
}

/*****************************************************************************
 函 数 名: get_home_key_long_press
 功能描述  : 获取home按键短按状态
 输入参数: void  
 输出参数: 无
 返 回 值: static
 
 修改历史:
  1.日     期: 2018年1月31日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
KEY_STATUS_ENUM get_home_key_long_press(void)
{
	return g_key_data.home_key.long_press.status;
}

/*****************************************************************************
 函 数 名: set_home_key_long_press_enable_clocker
 功能描述  : 设置home按键长按计时使能标志
 输入参数: bool flag  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void set_home_key_long_press_enable_clocker(bool flag)
{
	g_key_data.home_key.long_press.enable_clocker = flag;
}

/*****************************************************************************
 函 数 名: get_home_key_long_press_enable_clocker
 功能描述  : 获取home按键长按计时使能标志
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool get_home_key_long_press_enable_clocker(void)
{
	return g_key_data.home_key.long_press.enable_clocker;
}

/*****************************************************************************
 函 数 名: set_home_key_long_press_valid_time
 功能描述  : 设置home长按有效时间
 输入参数: uint16_t value  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void set_home_key_long_press_valid_time(uint16_t value)
{
	g_key_data.home_key.long_press.valid_time = value;
}

/*****************************************************************************
 函 数 名: get_home_key_long_press_valid_time
 功能描述  : 获取home长按有效时间
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
uint16_t get_home_key_long_press_valid_time(void)
{
	return g_key_data.home_key.long_press.valid_time;
}

/*****************************************************************************
 函 数 名: set_home_key_long_press_keep_time
 功能描述  : 设置home长按保持时间
 输入参数: uint16_t value  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void set_home_key_long_press_keep_time(uint16_t value)
{
	g_key_data.home_key.long_press.keep_time = value;
}

/*****************************************************************************
 函 数 名: get_home_key_long_press_keep_time
 功能描述  : 获取home长按保持时间
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
uint16_t get_home_key_long_press_keep_time(void)
{
	return g_key_data.home_key.long_press.keep_time;
}

/*****************************************************************************
 函 数 名: clear_home_key_long_press_keep_time
 功能描述  : 清除按键保持时间
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void clear_home_key_long_press_keep_time(void)
{
	set_home_key_long_press_keep_time(0);
}

/*****************************************************************************
 函 数 名: set_key_debounce_time
 功能描述  : 设置消抖时间
 输入参数: uint16_t debounce_time  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void set_key_debounce_time(uint16_t debounce_time)
{
	g_key_data.debounce_time = debounce_time;
}

/*****************************************************************************
 函 数 名: get_key_debounce_time
 功能描述  : 获取消抖时间
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
uint16_t get_key_debounce_time(void)
{
	return g_key_data.debounce_time;
}

/*****************************************************************************
 函 数 名: switch_home_key_short_press_status
 功能描述  : 切换home短按按键状态
 输入参数: void  
 输出参数: 无
 返 回 值: static
 
 修改历史:
  1.日     期: 2018年1月31日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void switch_home_key_short_press_status(void)
{
	KEY_STATUS_ENUM data;

	data = get_home_key_short_press();
	if (KEY_RELEASED == data)
	{
		data = KEY_PRESSED;
	}
	else
	{
		data = KEY_RELEASED;
	}
	
	set_home_key_short_press(data);
}

/*****************************************************************************
 函 数 名: switch_home_key_long_press_status
 功能描述  : 切换home短按按键状态
 输入参数: void  
 输出参数: 无
 返 回 值: static
 
 修改历史:
  1.日     期: 2018年1月31日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void switch_home_key_long_press_status(void)
{
	KEY_STATUS_ENUM data;

	data = get_home_key_long_press();
	if (KEY_RELEASED == data)
	{
		data = KEY_PRESSED;
	}
	else
	{
		data = KEY_RELEASED;
	}
	
	set_home_key_long_press(data);
}

/*****************************************************************************
 函 数 名: init_home_key_status
 功能描述  : 初始化home按键初始状态
 输入参数: void  
 输出参数: 无
 返 回 值: static
 
 修改历史:
  1.日     期: 2018年1月31日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void init_home_key_status(void)
{
	KEY_STATUS_ENUM default_status = KEY_RELEASED;
	uint16_t default_debounce_time = 10;
	uint16_t default_valid_time = 50;
	uint16_t default_keep_time = 0;
	
	set_home_key_short_press(default_status);
	set_home_key_short_press(default_status);
	set_key_debounce_time(default_debounce_time);
	set_home_key_long_press_valid_time(default_valid_time);
	set_home_key_long_press_keep_time(default_keep_time);
}

/*****************************************************************************
 函 数 名: home_key_long_press_ticks
 功能描述  : home按键长按计时
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void home_key_long_press_ticks(void)
{
	bool flag = false;
	uint16_t keep_time = 0;
	
	flag = get_home_key_long_press_enable_clocker();
	if (true == flag)
	{
		keep_time = get_home_key_long_press_keep_time();
		keep_time++;
		set_home_key_long_press_keep_time(keep_time);
	}
}

/*****************************************************************************
 函 数 名: hardware_get_home_key
 功能描述  : 获取home按键硬件状态
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
uint8_t hardware_get_home_key(void)
{
	uint8_t ret = 0;
	ret = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10);
	return ret;
}

/*****************************************************************************
 函 数 名: clear_home_key_long_press
 功能描述  : 清除home按键长按数据与状态
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void clear_home_key_long_press(void)
{
	const KEY_STATUS_ENUM key_released = KEY_RELEASED;
	
	clear_home_key_long_press_keep_time();
	set_home_key_long_press(key_released);
	set_home_key_long_press_enable_clocker(false);
}

/*****************************************************************************
 函 数 名: detect_home_key_long_press
 功能描述  : 检测home按键长按状态
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月6日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool detect_home_key_long_press(void)
{
	bool ret = false;
	uint16_t keep_time = 0;
	uint16_t valid_time = 0;
	const KEY_STATUS_ENUM key_pressed = KEY_PRESSED;
	ret = get_home_key_long_press_enable_clocker();
	if (true == ret)
	{
		ret = false;
		keep_time = get_home_key_long_press_keep_time();
		valid_time = get_home_key_long_press_valid_time();
		if (keep_time >= valid_time)
		{
			set_home_key_long_press(key_pressed);
			ret = true;
		}
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名: detect_home_key
 功能描述  : 探测home按键状态
 输入参数: void  
 输出参数: 无
 返 回 值: static
 
 修改历史:
  1.日     期: 2018年1月31日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void detect_home_key(void)
{
	bool flag = false;
	uint8_t curr_status = 0;
	const uint8_t pressed = 0;
	const uint8_t released = 1;
	uint16_t debounce_time = 0;
	static u8 last_status = released;
	KEY_STATUS_ENUM key_status;
	const KEY_STATUS_ENUM key_pressed = KEY_PRESSED;
	const KEY_STATUS_ENUM key_released = KEY_RELEASED;
	const KEY_STATUS_ENUM key_invalid = KEY_STATUS_INVALID;

	debounce_time = get_key_debounce_time();
	curr_status = hardware_get_home_key();
	if ((pressed == curr_status) && (released == last_status))
	{
		delay_ms(debounce_time);
		curr_status = hardware_get_home_key();
		if(pressed == curr_status)
		{
			set_home_key_long_press_enable_clocker(true);
			flag = detect_home_key_long_press();
			if (true == flag)
			{
				key_status = key_invalid;
			}
			else
			{
				key_status = key_pressed;
			}
			set_home_key_short_press(key_status);
			last_status = pressed;
		}
	}
	else if ((released == curr_status) && (pressed == last_status))
	{
		delay_ms(debounce_time);
		curr_status = hardware_get_home_key();
		if(released == curr_status)
		{
			set_home_key_short_press(key_released);
			clear_home_key_long_press();
			last_status = released;
		}
	}
}

/*****************************************************************************
 函 数 名: get_key_data_len
 功能描述  : 获取按键数据的长度
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年1月31日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
uint16_t get_key_data_len(void)
{
	uint8_t len = 0;
	KEY_STATUS_ENUM long_press = get_home_key_long_press();
	KEY_STATUS_ENUM short_press = get_home_key_short_press();
	len = sizeof(long_press) + sizeof(short_press);
	return len;
}

