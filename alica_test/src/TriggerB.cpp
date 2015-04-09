/*
 * Trigger.cpp
 *
 *  Created on: Apr 8, 2015
 *      Author: Stefan Jakob
 */

#include <TriggerB.h>
#include <TestWorldModel.h>


namespace alicaTests
{

	TriggerB::TriggerB() : alica::BasicBehaviour("TriggerB")
	{
		this->callCounter = 0;
		this->initCounter = 0;
		this->behaviourTrigger = alicaTests::TestWorldModel::getOne()->trigger1;

	}

	TriggerB::~TriggerB()
	{
	}

	void TriggerB::run(void* msg)
	{
		callCounter++;
	}

	void TriggerB::initialiseParameters()
	{
		callCounter = 0;
		initCounter++;
	}

} /* namespace alicaTests */
