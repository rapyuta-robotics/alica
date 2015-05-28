/*
 * AlicaROSClock.h
 *
 *  Created on: Jul 18, 2014
 *      Author: Paul Panin
 */

#ifndef ALICAROSCLOCK_H_
#define ALICAROSCLOCK_H_

#include <engine/IAlicaClock.h>

namespace alicaRosProxy
{

	class AlicaROSClock : public virtual alica::IAlicaClock
	{
	public:
		AlicaROSClock();
		virtual ~AlicaROSClock();
		virtual alica::AlicaTime now();
		virtual void sleep(long us);
	};

} /* namespace alicaRosProxy */

#endif /* ALICAROSCLOCK_H_ */
