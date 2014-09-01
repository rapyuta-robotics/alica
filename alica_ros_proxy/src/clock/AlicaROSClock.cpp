/*
 * AlicaROSClock.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: Paul Panin
 */

#include "clock/AlicaROSClock.h"
#include "ros/time.h"

namespace alicaRosProxy
{

	AlicaROSClock::AlicaROSClock()
	{
		// TODO Auto-generated constructor stub

	}

	AlicaROSClock::~AlicaROSClock()
	{
		// TODO Auto-generated destructor stub
	}

	alica::alicaTime AlicaROSClock::now()
	{
		ros::Time::init();
		ros::Time t = ros::Time::now();
		alica::alicaTime ret = ((alica::alicaTime)t.sec * 1000000000L) + t.nsec;
		return ret;
	}
	void AlicaROSClock::sleep(int ms)
	{
		ros::Duration(ms/1000).sleep();
	}

} /* namespace supplementary */
