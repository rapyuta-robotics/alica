/*
 * Trigger.cpp
 *
 *  Created on: Apr 8, 2015
 *      Author: Stefan Jakob
 */

#include <TriggerC.h>
#include <TestWorldModel.h>


namespace alicaTests
{

	TriggerC::TriggerC() : alica::BasicBehaviour("TriggerC")
	{
		this->callCounter = 0;
		this->initCounter = 0;
		this->behaviourTrigger = alicaTests::TestWorldModel::getOne()->trigger2;

	}

	TriggerC::~TriggerC()
	{
	}

	void TriggerC::run(void* msg)
	{
		callCounter++;
	}

	void TriggerC::initialiseParameters()
	{
		callCounter = 0;
		initCounter++;
	}

} /* namespace alicaTests */
