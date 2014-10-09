/*
 * TestConstraintCreator.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#include "TestConstraintCreator.h"

namespace alica
{

	TestConstraintCreator::TestConstraintCreator()
	{
		// TODO Auto-generated constructor stub

	}

	TestConstraintCreator::~TestConstraintCreator()
	{
		// TODO Auto-generated destructor stub
	}

	shared_ptr<BasicConstraint> TestConstraintCreator::createConstraint(long constraintConfId)
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
