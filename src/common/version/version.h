/******************************************************************************

  版权所有 (C), 2017-2028, _ _ _ Co., Ltd.

 ******************************************************************************
  文件名称: version.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年12月15日
  最近修改:
  功能描述: version.cpp 的头文件
  函数列表:
  修改历史:
  1.日     期: 2017年12月15日
    作     者: Leon
    修改内容: 创建文件
******************************************************************************/
#ifndef __VERSION_H__
#define __VERSION_H__

/*****************************************************************************/
#ifdef __cplusplus
#if __cplusplus
//extern "C"{
#endif
#endif /* __cplusplus */
/*****************************************************************************/

/******************************************************************************
 * 包含头文件
 ******************************************************************************/
#include <string>
#include <unistd.h>
#include <pthread.h>
#include <inttypes.h>

using namespace std;
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
#define MAX_VERSION_FIELDS                    8              //版本信息字段数

/******************************************************************************
 * 常量声明
 ******************************************************************************/

/******************************************************************************
 * 枚举类型
 ******************************************************************************/

/******************************************************************************
 * 结构体类型
 ******************************************************************************/
//版本相关信息
typedef struct VERSION
{
	uint32_t hardware[MAX_VERSION_FIELDS];                     //前后各四个字段
	uint32_t firmware[MAX_VERSION_FIELDS];                     //前后各四个字段
}VERSION_STRU;

/******************************************************************************
 * 类声明
 ******************************************************************************/
class version
{
public:
	static version* get_instance(void);
	static void release_instance(void);

	bool check_valid_fields(const uint32_t fields);
	uint32_t get_version_fields(void);

	void set_version_info(const VERSION_STRU version);
	void get_version_info(VERSION_STRU &version);

	bool set_firmware_version_info(const uint32_t fields, const uint32_t value);
	bool get_firmware_version_info(const uint32_t fields, uint32_t &value);
	
	bool set_hardware_version_info(const uint32_t fields, const uint32_t value);
	bool get_hardware_version_info(const uint32_t fields, uint32_t &value);

	bool check_valid_serial_number(const string &str);
	bool set_serial_number(const string &str);
	bool get_serial_number(string &str);

	bool set_uboot_version(const string &str);
	bool get_uboot_version(string &str);
	
	bool set_kernerl_version(const string &str);
	bool get_kernerl_version(string &str);

	void print_version(void);
	
protected:
	version(){};
	~version(){};

private:
	version(const version&){};                                       //禁止拷贝
	version& operator=(const version&){};                            //禁止赋值

	static constexpr uint32_t version_fields_ = MAX_VERSION_FIELDS;  //版本信息字段数

	VERSION_STRU version_;                                           //版本信息

	string uboot_version_;                                           //u-boot版本
	string kernerl_version_;                                         //内核版本

	string serial_number_;                                           //序列号

	static version* p_instance_;
	static pthread_mutex_t mutex_;
};

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/


/*****************************************************************************/
#ifdef __cplusplus
#if __cplusplus
//}
#endif
#endif /* __cplusplus */
/*****************************************************************************/

#endif /* __VERSION_H__ */
