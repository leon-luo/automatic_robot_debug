/******************************************************************************

  Copyright (C), 2017-2028, HUIZHOU BLUEWAY ELECTRONICS Co., Ltd.

 ******************************************************************************
  File Name     : key_unit.h
  Version       : Initial Draft
  Author        : Leon
  Created       : 2018/2/10
  Last Modified :
  Description   : key_unit.h header file
  Function List :
  History       :
  1.Date        : 2018年2月10日
    Author      : Leon
    Modification: Created file
******************************************************************************/
#ifndef __KEY_UNIT_H__
#define __KEY_UNIT_H__

/*****************************************************************************/
#ifdef __cplusplus
#if __cplusplus
//extern "C"{
#endif
#endif /* __cplusplus */
/*****************************************************************************/

/******************************************************************************
 * include header files list
 ******************************************************************************/
#include <unistd.h>
#include <pthread.h>

#include "key_base.h"

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
class key_unit
{
public://protected:

	key_unit();
	~key_unit();
	
public:
	void set_status(KEY_STATUS_ENUM value);
	KEY_STATUS_ENUM get_status(void);

	void set_single_click(bool value);
	bool get_single_click(void);

	void set_double_click(bool value);
	bool get_double_click(void);

	void set_long_click(bool value);
	bool get_long_click(void);

	bool get_enable_clocker(void);
	void set_enable_clocker(bool value);

	void set_valid_time(uint32_t value);
	uint32_t get_valid_time(void);

	void set_keep_time(uint32_t value);
	uint32_t get_keep_time(void);

	void set_click_num(uint8_t value);
	uint8_t get_click_num(void);
	
	void set_press_tick(uint64_t value);
	uint64_t get_press_tick(void);
	
	void set_release_tick(uint64_t value);
	uint64_t get_release_tick(void);
	
	void set_press_long(uint64_t value);
	uint64_t get_press_long(void);
	
	void set_release_long(uint64_t value);
	uint64_t get_release_long(void);

	void init_hold(LONG_PRESS_STRU value);
	void init_hold(bool enable_clocker = false, uint32_t valid_time = 100, uint32_t keep_time = 0);

	virtual void update_single_click(void);
	virtual void update_double_click(void);
	virtual void update_long_click(void);

	void save_press_tick(void);
	void save_release_tick(void);

	uint64_t save_press_long(void);
	uint64_t save_release_long(void);

	bool save_key_status_time(void);
	void analyze_key_click_signal(void);

private:
	KEY_STRU data_;
	
	key_unit(const key_unit&){};
	key_unit& operator=(const key_unit&){};
};


/******************************************************************************
 * internal function prototypes
 ******************************************************************************/



/*****************************************************************************/
#ifdef __cplusplus
#if __cplusplus
//}
#endif
#endif /* __cplusplus */
/*****************************************************************************/

#endif /* __KEY_UNIT_H__ */
