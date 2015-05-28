/*
 * AlicaSystemClock.cpp
 *
 *  Created on: Mar 13, 2015
 *      Author: Paul Panin
 */

#include "clock/AlicaSystemClock.h"
#include <time.h>

namespace alica_dummy_proxy {

	AlicaSystemClock::AlicaSystemClock()
	{
		// TODO Auto-generated constructor stub

	}

	AlicaSystemClock::~AlicaSystemClock()
	{
		// TODO Auto-generated destructor stub
	}
	alica::AlicaTime AlicaSystemClock::now()
	{
		time_t t;
		t = time(NULL);
		alica::AlicaTime ret = (alica::AlicaTime)(t * 1000000000UL + t*1000*1000);
		return ret;



	}
	void AlicaSystemClock::sleep(long us)
	{
		int sec = us/1000000;
		int nsec = (us%1000000)*1000;
		sleep(sec*1000);

	}

} /* namespace alica */
