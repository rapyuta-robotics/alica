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

	unique_ptr<BasicBehaviour> TestBehaviourCreator::createBehaviour(long behaviourConfId)
	{
		unique_ptr<alica::BasicBehaviour> beh;

		switch (behaviourConfId)
		{
			case 1402488712657:
				beh.reset(new MidFieldStandard());
				break;
			case 1402488763903:
				beh.reset(new DefendMid());
				break;
			case 1402488956661:
				beh.reset(new Tackle());
				break;
			case 1402488866727:
				beh.reset(new Attack());
				break;
			case 1402489366699:
				beh.reset(new AttackOpp());
				break;
			default:
			cerr << "TestBehaviourCreator: Unknown behaviour configuration id requested: " << behaviourConfId << endl;
			throw new exception();
			break;
		}
		return beh;
	}

} /* namespace alica */
