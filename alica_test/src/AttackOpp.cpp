/*
 * AttackOpp.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: Stephan Opfer
 */

#include "AttackOpp.h"

namespace alicaTests
{

	AttackOpp::AttackOpp() : alica::BasicBehaviour("AttackOpp")
	{
		this->callCounter = 0;
	}

	AttackOpp::~AttackOpp()
	{
	}

	void AttackOpp::run(void* msg)
	{
		cout << "AttackOpp was called " << callCounter++ << " times!" << endl;
	}

	void AttackOpp::initialiseParameters()
	{
	}

} /* namespace alica */
