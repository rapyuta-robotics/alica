/*
 * AlicaSystemClock.h
 *
 *  Created on: Mar 13, 2015
 *      Author: Paul Panin
 */

#ifndef ALICA_ALICA_DUMMY_PROXY_INCLUDE_CLOCK_ALICASYSTEMCLOCK_H_
#define ALICA_ALICA_DUMMY_PROXY_INCLUDE_CLOCK_ALICASYSTEMCLOCK_H_

#include "engine/IAlicaClock.h"
namespace alica_dummy_proxy {

	class AlicaSystemClock : public virtual alica::IAlicaClock
	{
		public:
			AlicaSystemClock();
			virtual ~AlicaSystemClock();
			virtual alica::AlicaTime now();
			virtual void sleep(long us);
	};

} /* namespace alica */

#endif /* ALICA_ALICA_DUMMY_PROXY_INCLUDE_CLOCK_ALICASYSTEMCLOCK_H_ */
