/*
 * AlicaROSClock.h
 *
 *  Created on: Jul 18, 2014
 *      Author: snook
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
		virtual alica::alicaTime now();
		virtual void sleep(int availTime);
	};

} /* namespace supplementary */

#endif /* ALICAROSCLOCK_H_ */
