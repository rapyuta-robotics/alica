/*
 * DefendMid.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: Stephan Opfer
 */

using namespace std;

#include "DefendMid.h"

namespace alica
{

	DefendMid::DefendMid() : BasicBehaviour ("DefendMid")
	{
		this->callCounter = 0;
	}

	DefendMid::~DefendMid()
	{
	}

	void DefendMid::run(void* msg)
	{
		cout << "DefendMid was called " << callCounter++ << " times!" << endl;
	}
} /* namespace alica */
