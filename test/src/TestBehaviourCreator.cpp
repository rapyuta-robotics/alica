/*
 * TestBehaviourCreator.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Stephan Opfer
 */

#include "TestBehaviourCreator.h"

#include "engine/BasicBehaviour.h"
#include "MidFieldStandard.h"
#include "DefendMid.h"
#include "Attack.h"
#include "Tackle.h"
#include "AttackOpp.h"

namespace alica
{

	TestBehaviourCreator::TestBehaviourCreator()
	{
	}

	TestBehaviourCreator::~TestBehaviourCreator()
	{
	}

	unique_ptr<BasicBehaviour> TestBehaviourCreator::createBehaviour(string behaviourName)
	{
		unique_ptr<alica::BasicBehaviour> beh;

		if ("MidFieldStandard" == behaviourName)
		{
			beh.reset(new MidFieldStandard());
		}
		else if ("DefendMid" == behaviourName)
		{
			beh.reset(new DefendMid());
		}
		else if ("Attack" == behaviourName)
		{
			beh.reset(new Attack());
		}
		else if ("Tackle" == behaviourName)
		{
			beh.reset(new Tackle());
		}
		else if ("AttackOpp" == behaviourName)
		{
			beh.reset(new AttackOpp());
		}
		else
		{
			cerr << "Unknown behaviour requested: " << behaviourName << endl;
			throw new exception();
		}

		return beh;
	}

} /* namespace alica */
