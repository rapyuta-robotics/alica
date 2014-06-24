/*
 * AttackOpp.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: Stephan Opfer
 */

#include "AttackOpp.h"

namespace alica
{

	AttackOpp::AttackOpp()
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

} /* namespace alica */
