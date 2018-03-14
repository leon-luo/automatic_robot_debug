/******************************************************************************

  Copyright (C), 2017-2028, HUIZHOU BLUEWAY ELECTRONICS Co., Ltd.

 ******************************************************************************
  File Name     : key_unit.cpp
  Version       : Initial Draft
  Author        : Leon
  Created       : 2018/2/10
  Last Modified :
  Description   : 按键单元
  Function List :
  History       :
  1.Date        : 2018年2月10日
    Author      : Leon
    Modification: Created file
******************************************************************************/

/******************************************************************************
 * include header files list
 ******************************************************************************/
#include "key_unit.h"

#include <iostream>

#include "time_base.h"
#include "debug_function.h"

using namespace std;

/******************************************************************************
 * external variables
 ******************************************************************************/

/******************************************************************************
 * external function prototypes
 ******************************************************************************/

/******************************************************************************
 * project-wide global variables
 ******************************************************************************/

/******************************************************************************
 * macros
 ******************************************************************************/

/******************************************************************************
 * constants
 ******************************************************************************/

/******************************************************************************
 * enum
 ******************************************************************************/

/******************************************************************************
 * struct
 ******************************************************************************/

/******************************************************************************
 * class
 ******************************************************************************/
/*****************************************************************************
 Prototype    : key_unit.key_unit
 Description  : 构造函数
 Input        : None
 Output       : None
 Return Value : key_unit
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
key_unit::key_unit()
{
	set_max_dblclick_takt_time(300);
	set_min_long_press_time(1000);
}

/*****************************************************************************
 Prototype    : key_unit.~key_unit
 Description  : 析构函数
 Input        : None
 Output       : None
 Return Value : key_unit
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
key_unit::~key_unit()
{

}

/*****************************************************************************
 Prototype    : key_unit.set_status
 Description  : 设置按键状态
 Input        : KEY_STATUS_ENUM value  
 Output       : None
 Return Value : void
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
void key_unit::set_status(KEY_STATUS_ENUM value)
{
	data_.status = value;
}

/*****************************************************************************
 Prototype    : key_unit.get_status
 Description  : 获取按键状态
 Input        : void  
 Output       : None
 Return Value : KEY_STATUS_ENUM
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
KEY_STATUS_ENUM key_unit::get_status(void)
{
	return data_.status;
}

/*****************************************************************************
 Prototype    : key_unit.set_single_click
 Description  : 设置按键单击或短按状态
 Input        : bool value  
 Output       : None
 Return Value : void
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
void key_unit::set_single_click(bool value)
{
	data_.single_click = value;
}

/*****************************************************************************
 Prototype    : key_unit.get_single_click
 Description  : 获取按键单击或短按状态
 Input        : void  
 Output       : None
 Return Value : bool
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
bool key_unit::get_single_click(void)
{
	return data_.single_click;
}

/*****************************************************************************
 Prototype    : key_unit.set_double_click
 Description  : 获取按键双击或连续两次短按状态
 Input        : bool value  
 Output       : None
 Return Value : void
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
void key_unit::set_double_click(bool value)
{
	data_.double_click = value;
}

/*****************************************************************************
 Prototype    : key_unit.get_double_click
 Description  : 获取获取按键双击或连续两次短按状态
 Input        : void  
 Output       : None
 Return Value : bool
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
bool key_unit::get_double_click(void)
{
	return data_.double_click;
}

/*****************************************************************************
 Prototype    : key_unit.set_long_click
 Description  : 设置按键长按有效状态
 Input        : bool value  
 Output       : None
 Return Value : void
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
void key_unit::set_long_click(bool value)
{
	data_.long_click = value;
}

/*****************************************************************************
 Prototype    : key_unit.get_long_click
 Description  : 获取按键长按有效状态
 Input        : void  
 Output       : None
 Return Value : bool
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
bool key_unit::get_long_click(void)
{
	return data_.long_click;
}

/*****************************************************************************
 Prototype    : key_unit.set_enable_clocker
 Description  : 设置按下使能使能计时状态
 Input        : bool value  
 Output       : None
 Return Value : void
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
void key_unit::set_enable_clocker(bool value)
{
	data_.hold.enable_clocker = value;
}

/*****************************************************************************
 Prototype    : key_unit.get_enable_clocker
 Description  : 获取按下使能使能计时状态
 Input        : void  
 Output       : None
 Return Value : bool
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
bool key_unit::get_enable_clocker(void)
{
	return data_.hold.enable_clocker;
}

/*****************************************************************************
 Prototype    : key_unit.set_valid_time
 Description  : 设置长按保持多长时间为有效
 Input        : uint32_t value  
 Output       : None
 Return Value : void
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
void key_unit::set_valid_time(uint32_t value)
{
	data_.hold.valid_time = value;
}

/*****************************************************************************
 Prototype    : key_unit.get_valid_time
 Description  : 获取长按保持多长时间为有效
 Input        : void  
 Output       : None
 Return Value : uint32_t
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
uint32_t key_unit::get_valid_time(void)
{
	return data_.hold.valid_time;
}

/*****************************************************************************
 Prototype    : key_unit.set_keep_time
 Description  : 设置当前按键保持按下了多长时间
 Input        : uint32_t value  
 Output       : None
 Return Value : void
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
void key_unit::set_keep_time(uint32_t value)
{
	data_.hold.keep_time = value;
}

/*****************************************************************************
 Prototype    : key_unit.get_keep_time
 Description  : 获取当前按键保持按下了多长时间
 Input        : void  
 Output       : None
 Return Value : uint32_t
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
uint32_t key_unit::get_keep_time(void)
{
	return data_.hold.keep_time;
}

/******************************************************************************
 Prototype   : key_unit.set_click_num
 Description : 连续短按的次数
 Input       : uint8_t value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_click_num(uint8_t value)
{
	data_.click_num = value;
}

/******************************************************************************
 Prototype   : key_unit.get_click_num
 Description : 获取连续短按的次数
 Input       : void 
 Output      : None
 Return Value: uint8_t
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint8_t key_unit::get_click_num(void)
{
	return data_.click_num;
}

/******************************************************************************
 Prototype   : key_unit.set_max_dblclick_takt_time
 Description : 设置连续两次短按的时间间隔
 Input       : uint32_t value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_max_dblclick_takt_time(uint32_t value)
{
	data_.max_dblclick_takt_time = value;
}

/******************************************************************************
 Prototype   : key_unit.get_max_dblclick_takt_time
 Description : 获取连续两次短按的时间间隔
 Input       : void 
 Output      : None
 Return Value: uint32_t
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint32_t key_unit::get_max_dblclick_takt_time(void)
{
	return data_.max_dblclick_takt_time;
}

/******************************************************************************
 Prototype   : key_unit.set_min_long_press_time
 Description : 设置最小的有效长按保持时间
 Input       : uint32_t value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/12
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_min_long_press_time(uint32_t value)
{
	data_.min_long_press_time = value;
}

/******************************************************************************
 Prototype   : key_unit.get_min_long_press_time
 Description : 获取最小的有效长按保持时间
 Input       : void 
 Output      : None
 Return Value: uint32_t
 
 History        :
  1.Data        :2018/3/12
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint32_t key_unit::get_min_long_press_time(void)
{
	return data_.min_long_press_time;
}

/******************************************************************************
 Prototype   : key_unit.set_press_tick
 Description : 设置按下的时刻
 Input       : uint64_t value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_press_tick(uint64_t value)
{
	data_.press_tick = value;
}

/******************************************************************************
 Prototype   : key_unit.get_press_tick
 Description : 获取按下的时刻
 Input       : void 
 Output      : None
 Return Value: uint64_t
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint64_t key_unit::get_press_tick(void)
{
	return data_.press_tick;
}

/******************************************************************************
 Prototype   : key_unit.set_release_tick
 Description : 设置释放的时刻
 Input       : uint64_t value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_release_tick(uint64_t value)
{
	data_.release_tick = value;
}

/******************************************************************************
 Prototype   : key_unit.get_release_tick
 Description : 获取释放的时刻
 Input       : void 
 Output      : None
 Return Value: uint64_t
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint64_t key_unit::get_release_tick(void)
{
	return data_.release_tick;
}

/******************************************************************************
 Prototype   : key_unit.set_press_long
 Description : 设置按键按下并保持的时长
 Input       : uint64_t value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_press_long(uint64_t value)
{
	data_.press_long = value;
}

/******************************************************************************
 Prototype   : key_unit.get_press_long
 Description : 获取按键按下并保持的时长
 Input       : void 
 Output      : None
 Return Value: uint64_t
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint64_t key_unit::get_press_long(void)
{
	return data_.press_long;
}

/******************************************************************************
 Prototype   : key_unit.set_release_long
 Description : 设置按键释放并保持的时长
 Input       : uint64_t value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_release_long(uint64_t value)
{
	data_.release_long = value;
}

/******************************************************************************
 Prototype   : key_unit.get_release_long
 Description : 获取按键释放并保持的时长
 Input       : void 
 Output      : None
 Return Value: uint64_t
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint64_t key_unit::get_release_long(void)
{
	return data_.release_long;
}

/*****************************************************************************
 Prototype    : update_single_click
 Description  : 更新单机或短按状态
 Input        : void  
 Output       : None
 Return Value : virtual
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
void key_unit::update_single_click(void)
{

}

/*****************************************************************************
 Prototype    : update_double_click
 Description  : 更新双击或连续短按两次有效状态
 Input        : void  
 Output       : None
 Return Value : virtual
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
void key_unit::update_double_click(void)
{
	
}

/*****************************************************************************
 Prototype    : key_unit.update_long_click
 Description  : 更新长按有效状态
 Input        : void  
 Output       : None
 Return Value : void
 
  History        :
  1.Date         : 2018/2/11
    Author       : Leon
    Modification : Created function
*****************************************************************************/
void key_unit::update_long_click(void)
{

}

/******************************************************************************
 Prototype   : key_unit.update_key_status
 Description : 更新按键状态
 Input       : KEY_STATUS_ENUM value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/12
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::update_key_status(KEY_STATUS_ENUM value)
{
	set_status(value);
	analyze_key_click_signal();
}

/******************************************************************************
 Prototype   : key_unit.update_multiple_click
 Description : 更新连按次数
 Input       : void 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/14
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::update_multiple_click(void)
{
	uint64_t ret = 0;
	uint8_t click_num = 0;
	uint32_t takt_time = 0;
	
	takt_time = get_max_dblclick_takt_time();
	if (ret < takt_time)
	{
		click_num = get_click_num();
		click_num++;
		set_click_num(click_num);
	}
}

/******************************************************************************
 Prototype   : key_unit.get_time_form_key_release
 Description : 获取当前时刻离上次按键释放的时刻之间的时-
               长
 Input       : void 
 Output      : None
 Return Value: uint64_t
 
 History        :
  1.Data        :2018/3/14
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint64_t key_unit::get_time_form_key_release(void)
{
	uint64_t ret = 0;
	uint64_t curr_time = 0;
	uint64_t release_time = 0;
	
	curr_time = get_millisecond_time();
	release_time = get_release_tick();
	if ( curr_time > release_time )
	{
		ret = curr_time - release_time;
	}

	return ret;
}

/******************************************************************************
 Prototype   : key_unit.test_valid_action_done
 Description : 检测按键动作是否已经完成
 Input       : void 
 Output      : None
 Return Value: bool
 
 History        :
  1.Data        :2018/3/14
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
bool key_unit::test_valid_action_done(void)
{
	bool ret = false;
	uint64_t temp = 0;
	uint64_t real_time = 0;
	uint32_t takt_time = 0;
	
	takt_time = get_max_dblclick_takt_time();
	temp = takt_time + 100;
	real_time = get_time_form_key_release();
	if (real_time > temp)
	{
		ret = true;
	}
	
	return ret;
}

/******************************************************************************
 Prototype   : key_unit.save_press_tick
 Description : 保存按下的时刻
 Input       : void 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::save_press_tick(void)
{
	uint64_t curr_time = 0;
	
	curr_time = get_millisecond_time();
	set_press_tick(curr_time);
}

/******************************************************************************
 Prototype   : key_unit.save_release_tick
 Description : 保存按键释放的时刻
 Input       : void 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::save_release_tick(void)
{
	uint64_t curr_time = 0;
	
	curr_time = get_millisecond_time();
	set_release_tick(curr_time);
}

/******************************************************************************
 Prototype   : key_unit.save_press_long
 Description : 保存按键按下保持的时长
 Input       : void 
 Output      : None
 Return Value: uint64_t
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint64_t key_unit::save_press_long(void)
{
	uint64_t ret = 0;
	uint64_t press_tick = 0;
	uint64_t release_tick = 0;
	
	press_tick = get_press_tick();
	release_tick = get_release_tick();
	if (release_tick >= press_tick)
	{
		ret = release_tick - press_tick;
		set_press_long(ret);
//		debug_print_info("ret = release_tick(%lld) - press_tick(%lld) = (%lld);", release_tick, press_tick, ret);
	}
	
	return ret;
}

/******************************************************************************
 Prototype   : key_unit.save_release_long
 Description : 保存按键释放保持的时长
 Input       : void 
 Output      : None
 Return Value: uint64_t
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint64_t key_unit::save_release_long(void)
{
	uint64_t ret = 0;
	uint64_t press_tick = 0;
	uint64_t release_tick = 0;

	press_tick = get_press_tick();
	release_tick = get_release_tick();
	if (press_tick >= release_tick)
	{
		ret = press_tick - release_tick;
		set_release_long(ret);
		
		update_multiple_click();
		
//		debug_print_warnning("ret = press_tick(%lld) - release_tick(%lld) = (%lld);", press_tick, release_tick, ret);
	}
	
	return ret;
}

/******************************************************************************
 Prototype   : key_unit.get_press_hold_time
 Description : 获取按键按下保持的时间
 Input       : void 
 Output      : None
 Return Value: bool
 
 History        :
  1.Data        :2018/3/9
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
bool key_unit::save_key_status_time(void)
{
	bool ret = false;
	static bool flag = false;
	const KEY_STATUS_ENUM pressed = KEY_PRESSED;
	const KEY_STATUS_ENUM released = KEY_RELEASED;
	KEY_STATUS_ENUM curr_status = KEY_STATUS_INVALID;

	curr_status = get_status();
	if (pressed == curr_status)
	{
		if(false == flag)
		{
			save_press_tick();
			save_release_long();
			flag = true;
		}
	}
	else if (released == curr_status)
	{
		if(true == flag)
		{
			save_release_tick();
			save_press_long();
			flag = false;
			ret = true;
		}
	}
	else
	{
		debug_print_error("curr_status = %d!", curr_status);
		ret = false;
	}

	return ret;
}

/******************************************************************************
 Prototype   : key_unit.analyze_key_click_signal
 Description : 分析按键动作
 Input       : void 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/10
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::analyze_key_click_signal(void)
{
	bool ret = false;
	bool action_done_flag = false;
	uint8_t click_num = 0;
	uint32_t value = 0;
	const uint32_t min_sigle_click = 100;
	uint32_t min_long_click = get_min_long_press_time();;

	ret = save_key_status_time();
	if (true == ret)
	{
		click_num = get_click_num();
		if ( 0 < click_num)
		{
			if ( 1 == click_num)
			{
				set_double_click(true);
			}
			set_click_num(0);
		}
		else
		{
			action_done_flag = test_valid_action_done();
			ret = get_double_click();
			if ((false == ret ) && (true == action_done_flag))
			{
				value = get_press_long();
				if ((min_sigle_click <= value) && (value < min_long_click))
				{
					set_single_click(true);
					debug_print_info(" set_single_click(true); value=%d, min_sigle_click=%d, min_long_click=%d", value, min_sigle_click, min_long_click);
				}
				else if (min_long_click <= value)
				{
					set_long_click(true);
					debug_print_info(" set_long_click(true); value=%d, min_sigle_click=%d, min_long_click=%d", value, min_sigle_click, min_long_click);
				}
			}
		}
	}
}

/******************************************************************************
 * internal function prototypes
 ******************************************************************************/


