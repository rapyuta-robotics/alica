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

	Attack::Attack()
	{
		this->callCounter = 0;
	}

	Attack::~Attack()
	{
	}

	void Attack::run(void* msg)
	{
		cout << "Attack was called " << callCounter++ << " times!" << endl;
	}

} /* namespace alica */
