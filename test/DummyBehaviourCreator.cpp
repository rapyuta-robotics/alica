/*
 * DummyBehaviourCreator.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: emmeda
 */

#include "DummyBehaviourCreator.h"
#include "engine/BasicBehaviour.h"

namespace alica
{

	DummyBehaviourCreator::DummyBehaviourCreator()
	{
	}

	DummyBehaviourCreator::~DummyBehaviourCreator()
	{
	}

	unique_ptr<BasicBehaviour> DummyBehaviourCreator::createBehaviour(string behaviourName)
	{
		unique_ptr<alica::BasicBehaviour> beh;

		if ("TestBehaviour" == behaviourName)
		{
			//beh.reset(new TestBehaviour());
			cout << "TestBehaviour matched" << endl;
		}
		else
		{
			cerr << "Unknown behaviour requested: " << behaviourName << endl;
			throw new exception();
		}

		return beh;
	}

} /* namespace alica */
