/*
 * Tackle.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: Stephan Opfer
 */

#include "Tackle.h"

namespace alica
{

	Tackle::Tackle()
	{
		this->callCounter = 0;
	}

	Tackle::~Tackle()
	{
	}

	void Tackle::run(void* msg)
	{
		cout << "Tackle was called " << callCounter++ << " times!" << endl;
	}

} /* namespace alica */
