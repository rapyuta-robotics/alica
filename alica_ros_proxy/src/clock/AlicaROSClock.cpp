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
		ros::Time::init();
	}

	AlicaROSClock::~AlicaROSClock()
	{
	}

	alica::alicaTime AlicaROSClock::now()
	{
		ros::Time t = ros::Time::now();
		alica::alicaTime ret = ((alica::alicaTime)t.sec * 1000000000L) + t.nsec;
		return ret;
	}
	void AlicaROSClock::sleep(int ms)
	{
		ros::Duration(ms * 1000000).sleep();
	}

} /* namespace supplementary */
