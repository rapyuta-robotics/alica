/*
 * TestConstraintCreator.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#include "TestConstraintCreator.h"
#include <iostream>

using namespace std;

namespace alicaTests
{

	TestConstraintCreator::TestConstraintCreator()
	{
		// TODO Auto-generated constructor stub

	}

	TestConstraintCreator::~TestConstraintCreator()
	{
		// TODO Auto-generated destructor stub
	}

	shared_ptr<alica::BasicConstraint> TestConstraintCreator::createConstraint(long constraintConfId)
	{
		switch (constraintConfId)
		{
			case 1412781693884:
				//return make_shared<Constraint1412781707952>();
				break;
			default:
			cerr << "TestConstraintCreator: Unknown constraint id requested: " << constraintConfId << endl;
			throw new exception();
			break;
		}
	}

} /* namespace alica */
