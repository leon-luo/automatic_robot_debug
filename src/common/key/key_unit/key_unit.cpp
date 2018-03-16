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
	const uint32_t takt_time = 300;
	
	set_press_tick(0);
	set_release_tick(0);
	set_max_dblclick_takt_time(takt_time);
	set_min_click_time(100);
	set_min_long_press_time(1000);
	set_click_num(0);
	set_single_click(false);
	set_double_click(false);
	set_long_click(false);
	set_record_time(false);
	set_release_timer_enable(false);
	set_release_timer_time(takt_time + 100);
	set_release_duration_time(0);
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

/******************************************************************************
 Prototype   : key_unit.set_record_time
 Description : 设置标记时间标志
 Input       : bool value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/15
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_record_time(bool value)
{
	data_.record_time = value;
}

/******************************************************************************
 Prototype   : key_unit.get_record_time
 Description : 获取标记时间标志
 Input       : void 
 Output      : None
 Return Value: bool
 
 History        :
  1.Data        :2018/3/15
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
bool key_unit::get_record_time(void)
{
	return data_.record_time;
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

/******************************************************************************
 Prototype   : key_unit.set_release_timer_enable
 Description : 设置按键释放使能计时功能是否开启
 Input       : bool value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/15
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_release_timer_enable(bool value)
{
	data_.release_timer.enable = value;
}

/******************************************************************************
 Prototype   : key_unit.get_release_timer_enable
 Description : 获取按键释放使能计时状态
 Input       : void 
 Output      : None
 Return Value: bool
 
 History        :
  1.Data        :2018/3/15
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
bool key_unit::get_release_timer_enable(void)
{
	return data_.release_timer.enable;
}

/******************************************************************************
 Prototype   : key_unit.set_release_timer_time
 Description : 设置释放计时器时间
 Input       : uint32_t value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/15
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_release_timer_time(uint32_t value)
{
	data_.release_timer.time = value;
}

/******************************************************************************
 Prototype   : key_unit.get_release_timer_time
 Description : 获取释放计时器时间
 Input       : void 
 Output      : None
 Return Value: uint32_t
 
 History        :
  1.Data        :2018/3/15
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint32_t key_unit::get_release_timer_time(void)
{
	return data_.release_timer.time;
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
 Prototype   : key_unit.set_min_click_time
 Description : 设置有效按键最小的保持按下的时间阈值
 Input       : uint32_t value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/15
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_min_click_time(uint32_t value)
{
	data_.min_click_time = value;
}

/******************************************************************************
 Prototype   : key_unit.get_min_click_time
 Description : 获取有效按键最小的保持按下的时间阈值
 Input       : void 
 Output      : None
 Return Value: uint32_t
 
 History        :
  1.Data        :2018/3/15
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint32_t key_unit::get_min_click_time(void)
{
	return data_.min_click_time;
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
 Prototype   : key_unit.set_press_duration_time
 Description : 设置按键按下并保持的时长
 Input       : uint64_t value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_press_duration_time(uint64_t value)
{
	data_.press_long = value;
}

/******************************************************************************
 Prototype   : key_unit.get_press_duration_time
 Description : 获取按键按下并保持的时长
 Input       : void 
 Output      : None
 Return Value: uint64_t
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint64_t key_unit::get_press_duration_time(void)
{
	return data_.press_long;
}

/******************************************************************************
 Prototype   : key_unit.set_release_duration_time
 Description : 设置按键释放并保持的时长
 Input       : uint64_t value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_unit::set_release_duration_time(uint64_t value)
{
	data_.release_long = value;
}

/******************************************************************************
 Prototype   : key_unit.get_release_duration_time
 Description : 获取按键释放并保持的时长
 Input       : void 
 Output      : None
 Return Value: uint64_t
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint64_t key_unit::get_release_duration_time(void)
{
	return data_.release_long;
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
	uint8_t click_num = 0;
	uint32_t takt_time = 0;
	uint64_t release_time = 0;
	
	click_num = get_click_num();
	release_time = get_release_duration_time();
	takt_time = get_max_dblclick_takt_time();
	if (((0 < click_num) && (release_time < takt_time))
	|| ((0 == click_num) && (release_time > takt_time)))
	{
		click_num++;
		set_click_num(click_num);
	}
	else
	{
		debug_print_warnning("takt_time = %d; release_time = %lld;", takt_time, release_time);
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
	bool enable = false;
	uint64_t release_time = 0;
	uint64_t real_time = 0;

	enable = get_release_timer_enable();
	if (true == enable)
	{
		release_time = get_release_timer_time();
		real_time = get_time_form_key_release();
		if (real_time > release_time)
		{
			set_release_timer_enable(false);
			ret = true;
		}
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
//	debug_print_info("curr_time=%lld", curr_time);
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
//	debug_print_info("curr_time=%lld", curr_time);
}

/******************************************************************************
 Prototype   : key_unit.calculate_time_from_press_to_release
 Description : 保存按键按下保持的时长
 Input       : void 
 Output      : None
 Return Value: uint64_t
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint64_t key_unit::calculate_time_from_press_to_release(void)
{
	uint64_t ret = 0;
	uint64_t press_tick = 0;
	uint64_t release_tick = 0;
	
	press_tick = get_press_tick();
	release_tick = get_release_tick();
	if (release_tick >= press_tick)
	{
		ret = release_tick - press_tick;
		set_press_duration_time(ret);
	}
	else
	{
		debug_print_warnning("release_tick(%lld) - press_tick(%lld) = ret(%lld);", release_tick, press_tick, ret);
	}
	
	return ret;
}

/******************************************************************************
 Prototype   : key_unit.calculate_time_from_release_to_press
 Description : 保存按键释放保持的时长
 Input       : void 
 Output      : None
 Return Value: uint64_t
 
 History        :
  1.Data        :2018/3/11
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
uint64_t key_unit::calculate_time_from_release_to_press(void)
{
	uint64_t ret = 0;
	uint64_t press_tick = 0;
	uint64_t release_tick = 0;

	press_tick = get_press_tick();
	release_tick = get_release_tick();
	if (press_tick >= release_tick)
	{
		ret = press_tick - release_tick;
		set_release_duration_time(ret);
		update_multiple_click();
	}
	else
	{
		debug_print_warnning("  press_tick(%lld) - release_tick(%lld) = ret(%lld);", press_tick, release_tick, ret);
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
	bool record_time_flag = false;
	const KEY_STATUS_ENUM pressed = KEY_PRESSED;
	const KEY_STATUS_ENUM released = KEY_RELEASED;
	KEY_STATUS_ENUM curr_status = KEY_STATUS_INVALID;
	record_time_flag = get_record_time();
	curr_status = get_status();
	if (pressed == curr_status)
	{
		if(false == record_time_flag)
		{
			save_press_tick();
			calculate_time_from_release_to_press();
			set_record_time(true);
		}
	}
	else if (released == curr_status)
	{
		if(true == record_time_flag)
		{
			save_release_tick();
			calculate_time_from_press_to_release();
			set_record_time(false);
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
	uint64_t value = 0;
	uint32_t min_click_time = get_min_click_time();
	uint32_t min_long_click = get_min_long_press_time();

	ret = save_key_status_time();
	if (true == ret)
	{
		set_release_timer_enable(true);
	}
	
	action_done_flag = test_valid_action_done();
	if (true == action_done_flag)
	{
		click_num = get_click_num();
//		debug_print_info("click_num=%d", click_num);
		if ( 1 == click_num)
		{
			value = get_press_duration_time();
			if ((min_click_time <= value) && (value < min_long_click))
			{
				set_single_click(true);
			}
			else if (min_long_click <= value)
			{
				set_long_click(true);
			}
		}
		else if ( 2 == click_num)
		{
			set_double_click(true);
		}
		
		set_click_num(0);
	}
}

/******************************************************************************
 * internal function prototypes
 ******************************************************************************/


