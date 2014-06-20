/*
 * MidFieldStandard.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: Stephan Opfer
 */

#include "MidFieldStandard.h"

namespace alica
{

	MidFieldStandard::MidFieldStandard()
	{
		this->callCounter = 0;
	}

	MidFieldStandard::~MidFieldStandard()
	{
	}

	void MidFieldStandard::run(void* msg)
	{
		cout << "MidFieldStandard was called " << callCounter++ << " times!" << endl;
	}

} /* namespace alica */
