/******************************************************************************

  Copyright (C), 2017-2028, 

*****************************************************************************
  File Name      : mul_timer.h
  Version        : Initial Draft
  Author         : Leon
  Created        : 2018/3/15
  Last Modified  :
  Description    : mul_timer.h header file
  Function List  :
  History        :
  1.Date         : 2018/3/15
    Author       : Leon
    Modification : Created file
 ******************************************************************************/

#ifndef __MUL_TIMER_H__
#define __MUL_TIMER_H__

/*******************************************************************************/
#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */
/*******************************************************************************/

/******************************************************************************
 * include header files list
 ******************************************************************************/

/******************************************************************************
 * external variables
 ******************************************************************************/

/******************************************************************************
 * external function  prototypes
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
 * class prototypes
 ******************************************************************************/

/******************************************************************************
 * internal function prototypes
 ******************************************************************************/
#include <sys/time.h>

#define MAX_TIMER_CNT 10
#define MUL_TIMER_RESET_SEC 10
#define TIMER_UNIT 60
#define MAX_FUNC_ARG_LEN 100
#define INVALID_TIMER_HANDLE (-1)

typedef int timer_handle_t;

typedef struct _timer_manage
{
    struct _timer_info
    {
        int state; /* on or off */
        int interval;
        int elapse; /* 0~interval */
        int (* timer_proc) (void *arg, int arg_len);
        char func_arg[MAX_FUNC_ARG_LEN];
        int arg_len;
    }timer_info[MAX_TIMER_CNT];

    void (* old_sigfunc)(int);
    void (* new_sigfunc)(int);
    struct itimerval value, ovalue;
}_timer_manage_t;

/* success, return 0; failed, return -1 */
int init_mul_timer(void);
/* success, return 0; failed, return -1 */
int destroy_mul_timer(void);
/* success, return timer handle(>=0); failed, return -1 */
timer_handle_t set_a_timer(int interval, int (* timer_proc) (void *arg, int arg_len),void *arg, int arg_len);
/* success, return 0; failed, return -1 */
int del_a_timer(timer_handle_t handle);


/*******************************************************************************/
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */
/*******************************************************************************/

#endif /* __MUL_TIMER_H__ */
