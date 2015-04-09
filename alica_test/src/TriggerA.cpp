/*
 * Trigger.cpp
 *
 *  Created on: Apr 8, 2015
 *      Author: Stefan Jakob
 */

#include <TriggerA.h>
#include <TestWorldModel.h>

namespace alicaTests
{

	TriggerA::TriggerA() : alica::BasicBehaviour("TriggerA")
	{
		this->callCounter = 0;
		this->initCounter = 0;
		this->behaviourTrigger = alicaTests::TestWorldModel::getOne()->trigger1;

	}

	TriggerA::~TriggerA()
	{
	}

	void TriggerA::run(void* msg)
	{
		callCounter++;
	}

	void TriggerA::initialiseParameters()
	{
		callCounter = 0;
		initCounter++;
	}

} /* namespace alicaTests */
