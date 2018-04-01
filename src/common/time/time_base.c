/******************************************************************************

  版权所有 (C), 2017-2028, _ _ _ Co., Ltd.

 ******************************************************************************
  文件名称: time_base.c
  版本编号: 初稿
  作     者: Leon
  生成日期: 2018年1月10日
  最近修改:
  功能描述   : 时间相关功能函数定义
  函数列表:
  修改历史:
  1.日     期: 2018年1月10日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include "time_base.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/time.h>
#include <time.h>
#include "debug_function.h"

/******************************************************************************
 * 外部变量定义
 ******************************************************************************/

/******************************************************************************
 * 外部函数定义
 ******************************************************************************/

/******************************************************************************
 * 全局变量
 ******************************************************************************/

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
/*
1)以下是各种精确度的类型转换:
1秒=1000毫秒(ms), 1毫秒=1/1000秒(s)；
1秒=1000000 微秒(μs), 1微秒=1/1000000秒(s)；
1秒=1000000000 纳秒(ns),1纳秒=1/1000000000秒(s)；

2)clock()函数的精确度是10毫秒(ms)
times()函数的精确度是10毫秒(ms)
gettimofday()函数的精确度是微秒(μs)
clock_gettime()函数的计量单位为十亿分之一，也就是纳秒(ns)
*/


/*****************************************************************************
 函 数 名: print_current_time
 功能描述  : 打印当前时间
 输入参数: void  
 输出参数: 无
 返 回 值: 
 备     注：
 localtime_r也是用来获取系统时间，
 struct tm的结构为
          int tm_sec;    秒 – 取值区间为[0,59]
          int tm_min;    分 - 取值区间为[0,59]
          int tm_hour;   时 - 取值区间为[0,23]
          int tm_mday;   一个月中的日期 - 取值区间为[1,31]
          int tm_mon;    月份（从一月开始，0代表一月） - 取值区间为[0,11]
          int tm_year;   年份，其值等于实际年份减去1900
          int tm_wday;   星期 – 取值区间为[0,6]，其中0代表星期天，1代表星期一，以此类推
          int tm_yday;   从每年的1月1日开始的天数 – 取值区间为[0,365]，其中0代表1月1日，1代表1月2日，以此类推
          int tm_isdst;  夏令时标识符，实行夏令时的时候，tm_isdst为正。不实行夏令时的进候，tm_isdst为0；不了解情况时，tm_isdst()为负。
 修改历史:
  1.日     期: 2018年1月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void print_current_time(void)
{
	time_t now;
	struct tm *p = NULL;
	struct tm result;
	
	time(&now);
	p = localtime_r(&now, &result);
	
	debug_print_info("[Current time]:{%d-%d-%d %d:%d:%d}", (1900 + result.tm_year), ( 1 + result.tm_mon), 
		result.tm_mday, (result.tm_hour + 12), result.tm_min, result.tm_sec);
}

/*****************************************************************************
 函 数 名: get_current_time
 功能描述  : 获取当前时间
 输入参数: void  
 输出参数: 无
 返 回 值: 毫秒数
 备     注:
 函数原型：
    int gettimeofday(struct timeval*tv, struct timezone *tz);
    其参数tv是保存获取时间结果的结构体，参数tz用于保存时区结果。
    struct timezone{  
        int tz_minuteswest;  // 格林威治时间往西方的时差
        int tz_dsttime;      // DST 时间的修正方式
    }
    timezone 参数若不使用则传入NULL即可。
    struct timeval{  
        long int tv_sec;     // 秒数
        long int tv_usec;    // 微秒数
    }它获得的时间精确到微秒（1e-6 s)量级
 修改历史:
  1.日     期: 2018年1月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
uint32_t get_current_time(void)
{
	uint64_t now = 0;
//	uint64_t offset = 0;
//	static uint64_t last = 0;
	struct timeval tv;
	
	gettimeofday(&tv, NULL);
	now = tv.tv_sec * 1000 + tv.tv_usec / 1000;  //单位ms
//	if (last != now)
//	{
//		if (0 != last)
//		{
//			offset = now - last;
//		}
//		last = now;
//	}
//	debug_print_info("[Current time]:now {%ld(s) + %ld(us) = %llu(ms)}; offset = %llu(ms)", tv.tv_sec, tv.tv_usec, now, offset);
	
	return now;
}

/*****************************************************************************
 函 数 名: get_microsecond_time
 功能描述  : 获取当前时间的微秒数
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月9日
    作     者: Leon
    修改内容: 新生成函数
备注:int clock_gettime(clockid_t clk_id, struct timespec *tp); clockid_t是确定哪个时钟类型.
	CLOCK_REALTIME: 标准POSIX实时时钟
	CLOCK_MONOTONIC: POSIX时钟,以恒定速率运行;不会复位和调整,它的取值和CLOCK_REALTIME是一样的.
	CLOCK_PROCESS_CPUTIME_ID和CLOCK_THREAD_CPUTIME_ID是CPU中的硬件计时器中实现的.
*****************************************************************************/
uint64_t get_microsecond_time(void)
{
	uint64_t ret = 0;
	struct timespec temp;
	clockid_t clk_id = CLOCK_MONOTONIC;

	clock_gettime(clk_id, &temp);

	ret = MILLION*(temp.tv_sec)+(temp.tv_nsec)/KILO; //microseconds
	return ret;
}

/*****************************************************************************
 函 数 名: get_millisecond_time
 功能描述  : 获取当前时间的毫秒数
 输入参数: void  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年2月9日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
uint64_t get_millisecond_time(void)
{
	uint64_t ret = 0;
	uint64_t temp1 = 0;
	uint64_t temp2 = 0;
	struct timespec temp;
	clockid_t clk_id = CLOCK_MONOTONIC;

	clock_gettime(clk_id, &temp);
//	debug_print_info("[clock_gettime]:{tv_sec= %ld  tv_nsec=%ld}", temp.tv_sec, temp.tv_nsec);
	temp1 = KILO*(temp.tv_sec); //seconds turn into milliseconds
	temp2 = (temp.tv_nsec)/MILLION;//nanoseconds turn into milliseconds
//	debug_print_info("[clock_gettime]:{temp1= %ld  temp2=%ld}", temp1, temp2);
	ret = temp1 + temp2 ; //milliseconds

	return ret;
}

/*****************************************************************************
 函 数 名: signal_handler
 功能描述  : 信号处理
 输入参数: int16_t signo  
 输出参数: 无
 返 回 值: 
 备     注:
	ITIMER_REAL：以系统真实的时间来计算,它送出SIGALRM信号。
	ITIMER_VIRTUAL：以该进程在用户态下花费的时间来计算,它送出SIGVTALRM信号.
	ITIMER_PROF：以该进程在用户态下和内核态下所费的时间来计算,它送出SIGPROF信号。
 修改历史:
  1.日     期: 2018年1月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
void signal_handler(int16_t signo)
{
	switch (signo){
		case SIGALRM:
			printf("Caught the SIGALRM signal!\n");
			break;
		case SIGVTALRM:
			printf("Caught the SIGVTALRM signal!\n");
			break;
		case SIGPROF:
			printf("Caught the SIGPROF signal!\n");
		break;
	}
}

/*****************************************************************************
 函 数 名: set_timer
 功能描述  : 设置定时器
 输入参数: uint32_t interval_sec             下次定时器响应的间隔时间秒
           uint32_t interval_usec        下次定时器响应的间隔时间微秒
           uint32_t current_sec          当前定时器的时间秒
           uint32_t current_usec         当前定时器的时间微秒
           void (*pf_handler)(int16_t)  时间到达以后执行的函数指针
 输出参数: 无
 返 回 值: 
 备     注：
	settimer工作机制：先对it_value倒计时，当it_value为零时触发信号。
	然后重置为it_interval,继续对it_value倒计时。一直这样循环下去。基
	于此机制。setitimer既能够用来延时运行，也可定时运行。假如it_value
	为0是不会触发信号的，所以要能触发信号，it_value得大于0；假设
	it_interval为零，仅仅会延时。不会定时（也就是说仅仅会触发一次信号)。
 修改历史:
  1.日     期: 2018年1月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool set_timer(uint32_t interval_sec, uint32_t interval_usec, uint32_t current_sec, uint32_t current_usec, pf_sighandler_t pf)
{
	int ret = 0;

	if (NULL == pf) {
		return false;
	}

	if (signal(SIGALRM, pf) == SIG_ERR) {
		perror("signal() error!");
		return false;
	}

	struct itimerval new_value, old_value;
	//配置延时时长
	new_value.it_value.tv_sec = current_sec;
	new_value.it_value.tv_usec = current_usec;
	
	//配置计时间隔
	new_value.it_interval.tv_sec = interval_sec;
	new_value.it_interval.tv_usec = interval_usec;

	ret = setitimer(ITIMER_REAL, &new_value, &old_value);
	if (0 != ret) {
		perror("setitimer() error!");
		return false;
	}

	return true;
}

/*****************************************************************************
 函 数 名: deferred_execute
 功能描述  : 延时执行响应指定功能函数
 输入参数: uint32_t delay_s     
           uint32_t delay_us    
           pf_sighandler_t pf  
 输出参数: 无
 返 回 值: 
 
 修改历史:
  1.日     期: 2018年1月10日
    作     者: Leon
    修改内容: 新生成函数
*****************************************************************************/
bool deferred_execute(uint32_t delay_s, uint32_t delay_us, pf_sighandler_t pf)
{
	bool ret = false;
	
	ret = set_timer(0, 0, delay_s, delay_us, pf);

	return ret;
}

