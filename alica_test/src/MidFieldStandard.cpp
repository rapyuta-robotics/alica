/*
 * MidFieldStandard.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: Stephan Opfer
 */

#include "MidFieldStandard.h"

namespace alica
{

	MidFieldStandard::MidFieldStandard() : BasicBehaviour ("MidFieldStandard")
	{
		this->callCounter = 0;
	}

	MidFieldStandard::~MidFieldStandard()
	{
	}

	void MidFieldStandard::run(void* msg)
	{
		callCounter++;
		if(callCounter>10) {
			this->success = true;
		}
	}

	void MidFieldStandard::initialiseParameters()
	{
		this->callCounter = 0;
	}

} /* namespace alica */
