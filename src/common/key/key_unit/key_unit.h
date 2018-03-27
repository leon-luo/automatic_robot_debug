/******************************************************************************

  Copyright (C), 2017-2028, 

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
public:
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

	virtual void update_key_status(KEY_STATUS_ENUM value);

private:
	bool get_record_time(void);
	void set_record_time(bool value);

	void set_release_timer_enable(bool value);
	bool get_release_timer_enable(void);

	void set_release_timer_time(uint32_t value);
	uint32_t get_release_timer_time(void);

	void set_click_num(uint8_t value);
	uint8_t get_click_num(void);

	void set_max_dblclick_takt_time(uint32_t value);
	uint32_t get_max_dblclick_takt_time(void);

	void set_min_click_time(uint32_t value);
	uint32_t get_min_click_time(void);

	void set_min_long_press_time(uint32_t value);
	uint32_t get_min_long_press_time(void);
	
	void set_press_tick(uint64_t value);
	uint64_t get_press_tick(void);
	
	void set_release_tick(uint64_t value);
	uint64_t get_release_tick(void);
	
	void set_press_duration_time(uint64_t value);
	uint64_t get_press_duration_time(void);
	
	void set_release_duration_time(uint64_t value);
	uint64_t get_release_duration_time(void);

	void update_multiple_click(void);
	uint64_t get_time_form_key_release(void);
	bool test_valid_action_done(void);
	
	void save_press_tick(void);
	void save_release_tick(void);

	uint64_t calculate_time_from_press_to_release(void);
	uint64_t calculate_time_from_release_to_press(void);

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
