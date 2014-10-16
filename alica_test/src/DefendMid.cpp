/*
 * DefendMid.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: Stephan Opfer
 */

using namespace std;

#include "DefendMid.h"

namespace alicaTests
{

	DefendMid::DefendMid() : alica::BasicBehaviour ("DefendMid")
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

	void DefendMid::initialiseParameters()
	{
	}

} /* namespace alica */
