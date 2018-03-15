/******************************************************************************

  Copyright (C), 2017-2028, HUIZHOU BLUEWAY ELECTRONICS Co., Ltd.

 ******************************************************************************
  File Name     : key_data.cpp
  Version       : Initial Draft
  Author        : Leon
  Created       : 2018/2/10
  Last Modified :
  Description   : all key data 
  Function List :
  History       :
  1.Date        : 2018年2月10日
    Author      : Leon
    Modification: Created file
******************************************************************************/

/******************************************************************************
 * include header files list
 ******************************************************************************/
#include "key_data.h"

#include <iostream>
#include <string>
using namespace std;

//using std::cin;
//using std::cout;
//using std::endl;

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
pthread_mutex_t key_data::mutex_;
key_data* key_data::p_instance_ = nullptr;

/******************************************************************************
 Prototype   : key_data.key_data
 Description : constructor
 Input       : None
 Output      : None
 Return Value: key_data
 
 History        :
  1.Data        :2018/3/7
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
key_data::key_data()
{

}

/******************************************************************************
 Prototype   : key_data.~key_data
 Description : destructor
 Input       : None
 Output      : None
 Return Value: key_data
 
 History        :
  1.Data        :2018/3/7
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
key_data::~key_data()
{

}

/******************************************************************************
 Prototype   : key_data.get_instance
 Description : get the instance object
 Input       : void 
 Output      : None
 Return Value: key_data*
 
 History        :
  1.Data        :2018/3/7
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
key_data* key_data::get_instance(void)
{
	if (nullptr == p_instance_)
	{
		pthread_mutex_lock(&mutex_);
		if (nullptr == p_instance_)
		{
			p_instance_ = new key_data();
		}
		pthread_mutex_unlock(&mutex_);
	}
	
	return p_instance_;
}

/******************************************************************************
 Prototype   : key_data.release_instance
 Description : deallocate object
 Input       : void 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/7
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_data::release_instance(void)
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

/******************************************************************************
 Prototype   : key_data.set_key_status
 Description : 设置指定按键状态
 Input       : KEY_ID_ENUM id 
               KEY_STATUS_ENUM value 
 Output      : None
 Return Value: void
 
 History        :
  1.Data        :2018/3/7
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
void key_data::set_key_status(KEY_ID_ENUM id, KEY_STATUS_ENUM value)
{
	key_unit* p_key_unit = nullptr;
	
	p_key_unit = get_key_unit_instance(id);
	if ( nullptr != p_key_unit)
	{
		p_key_unit->update_key_status(value);
	}
}

/******************************************************************************
 Prototype   : key_data.get_single_click
 Description : 获取指定按键单机动作状态
 Input       : KEY_ID_ENUM id 
 Output      : None
 Return Value: bool :有效单次短按则返回true, 否则返回false
 
 History        :
  1.Data        :2018/3/7
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
bool key_data::get_single_click(KEY_ID_ENUM id)
{
	bool ret = false;
	key_unit* p_key_unit = nullptr;
	
	p_key_unit = get_key_unit_instance(id);
	if ( nullptr != p_key_unit)
	{
		ret = p_key_unit->get_single_click();
	}
	
	return ret;
}

/******************************************************************************
 Prototype   : key_data.clear_single_click
 Description : 清除单击按键动作
 Input       : KEY_ID_ENUM id 
 Output      : None
 Return Value: bool
 
 History        :
  1.Data        :2018/3/9
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
bool key_data::clear_single_click(KEY_ID_ENUM id)
{
	bool ret = false;
	key_unit* p_key_unit = nullptr;
	
	p_key_unit = get_key_unit_instance(id);
	if ( nullptr != p_key_unit)
	{
		p_key_unit->set_single_click(false);
		ret = true;
	}
	
	return ret;
}

/******************************************************************************
 Prototype   : key_data.get_double_click
 Description : 获取指定按键双击动作状态
 Input       : KEY_ID_ENUM id 
 Output      : None
 Return Value: bool :有效连续两次短按则返回true, 否则返回false
 
 History        :
  1.Data        :2018/3/7
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
bool key_data::get_double_click(KEY_ID_ENUM id)
{
	bool ret = false;
	key_unit* p_key_unit = nullptr;
	
	p_key_unit = get_key_unit_instance(id);
	if ( nullptr != p_key_unit)
	{
		ret = p_key_unit->get_double_click();
	}
	
	return ret;
}

/******************************************************************************
 Prototype   : key_data.clear_double_click
 Description : 清除双击按键动作
 Input       : KEY_ID_ENUM id 
 Output      : None
 Return Value: bool
 
 History        :
  1.Data        :2018/3/9
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
bool key_data::clear_double_click(KEY_ID_ENUM id)
{
	bool ret = false;
	key_unit* p_key_unit = nullptr;
	
	p_key_unit = get_key_unit_instance(id);
	if ( nullptr != p_key_unit)
	{
		p_key_unit->set_double_click(false);
		ret = true;
	}
	
	return ret;
}

/******************************************************************************
 Prototype   : key_data.get_long_click
 Description : 获取指定按键长按动作状态
 Input       : KEY_ID_ENUM id 
 Output      : None
 Return Value: bool :有效长按则返回true, 否则返回false
 
 History        :
  1.Data        :2018/3/7
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
bool key_data::get_long_click(KEY_ID_ENUM id)
{
	bool ret = false;
	key_unit* p_key_unit = nullptr;
	
	p_key_unit = get_key_unit_instance(id);
	if ( nullptr != p_key_unit)
	{
		ret = p_key_unit->get_long_click();
	}
	
	return ret;
}

/******************************************************************************
 Prototype   : key_data.clear_long_click
 Description : 清除长按按键动作
 Input       : KEY_ID_ENUM id 
 Output      : None
 Return Value: bool
 
 History        :
  1.Data        :2018/3/9
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
bool key_data::clear_long_click(KEY_ID_ENUM id)
{
	bool ret = false;
	key_unit* p_key_unit = nullptr;
	
	p_key_unit = get_key_unit_instance(id);
	if ( nullptr != p_key_unit)
	{
		p_key_unit->set_long_click(false);
		ret = true;
	}
	
	return ret;
}


/******************************************************************************
 Prototype   : key_data.get_key_unit_instance
 Description : 获取指定按键单元对象指针
 Input       : KEY_ID_ENUM id 
 Output      : None
 Return Value: key_unit*
 
 History        :
  1.Data        :2018/3/7
    Author      : Leon
    Modification: Created function.
 ******************************************************************************/
key_unit* key_data::get_key_unit_instance(KEY_ID_ENUM id)
{
	key_unit* p_key_unit = nullptr;

	switch ( id )
	{
	case HOME_KEY_ID :
		p_key_unit = &home_key_;
		break;
	case POWER_KEY_ID :
		p_key_unit = &power_key_;
		break;
	default:
		break;
	}

	return p_key_unit;
}

/******************************************************************************
 * internal function prototypes
 ******************************************************************************/


