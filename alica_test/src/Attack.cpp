/*
 * Attack.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: Stephan Opfer
 */

using namespace std;

#include "Attack.h"

namespace alica
{

	Attack::Attack() : BasicBehaviour("Attack")
	{
		this->callCounter = 0;
		this->initCounter = 0;
	}

	Attack::~Attack()
	{
	}

	void Attack::run(void* msg)
	{
		callCounter++;
	}

	void Attack::initialiseParameters()
	{
		initCounter++;
	}

} /* namespace alica */
