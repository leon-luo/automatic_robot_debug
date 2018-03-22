/******************************************************************************

  Copyright (C), 2017-2028, HUIZHOU BLUEWAY ELECTRONICS Co., Ltd.

*****************************************************************************
  File Name      : bll_laser_radar.cpp
  Version        : Initial Draft
  Author         : Leon
  Created        : 2018/3/22
  Last Modified  :
  Description    : 
  Function List  :
  History        :
  1.Date         : 2018/3/22
    Author       : Leon
    Modification : Created file
 ******************************************************************************/

/******************************************************************************
 * include header files list
 ******************************************************************************/

/******************************************************************************
 * external variables
 ******************************************************************************/

/******************************************************************************
 * external function  difinition
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
 * class difinition
 ******************************************************************************/

/******************************************************************************
 * internal function difinition
 ******************************************************************************/


double get_directional_distance(const sensor_msgs::LaserScan & scan, double direction)
{
	double angle = direction;
	double retDis = 0.0;
	double dis = 0.0;
	double minDis = 0.0;
	int count = 0;
	double angleScope = 0.0;

	//compute the valid value between angle-1.5 ~ angle+1.5 
	double begin = D2R((angle - 1.5));
	double end = D2R((angle + 1.5));

	angleScope = scan.angle_max - scan.angle_min;
	size_t ibegin = (size_t) ((begin - scan.angle_min) / angleScope * scan.ranges.size());
	size_t iend = (size_t) ((end - scan.angle_min) / angleScope * scan.ranges.size());

	minDis = scan.range_max;

	for (size_t i = ibegin; i < iend; i++)
	{
		//filter the invalid value
		if (scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max)
		{
			continue;
		}

		//get the min value
		if (minDis > scan.ranges[i])
		{
			minDis = scan.ranges[i];
		}
	}

	//the mini value as ranging value and returned
	retDis = minDis;

	return minDis;
}


