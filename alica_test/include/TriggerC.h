/*
 * Trigger.h
 *
 *  Created on: Apr 8, 2015
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_TEST_INCLUDE_TRIGGERC_H_
#define ALICA_ALICA_TEST_INCLUDE_TRIGGERC_H_

#include <engine/BasicBehaviour.h>
#include <ITrigger.h>
#include <SystemConfig.h>

namespace alicaTests
{

	class TriggerC : public alica::BasicBehaviour
	{
	public:
		TriggerC();
		virtual ~TriggerC();
		virtual void run(void* msg);
		int callCounter;
		int initCounter;


	protected:
		virtual void initialiseParameters();
	};

} /* namespace alicaTests */

#endif /* ALICA_ALICA_TEST_INCLUDE_TRIGGERC_H_ */
