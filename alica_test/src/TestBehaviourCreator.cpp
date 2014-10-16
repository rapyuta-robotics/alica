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

namespace alicaTests
{

	TestBehaviourCreator::TestBehaviourCreator()
	{
	}

	TestBehaviourCreator::~TestBehaviourCreator()
	{
	}

	shared_ptr<alica::BasicBehaviour> TestBehaviourCreator::createBehaviour(long behaviourConfId)
	{
		//shared_ptr<alica::BasicBehaviour> beh;

		switch (behaviourConfId)
		{
			case 1402488712657:
				return make_shared<MidFieldStandard>();
				break;
			case 1402488763903:
				return make_shared<DefendMid>();
				break;
			case 1402488956661:
				return make_shared<Tackle>();
				break;
			case 1402488866727:
				return make_shared<Attack>();
				break;
			case 1402489366699:
				return make_shared<AttackOpp>();
				break;
			default:
			cerr << "TestBehaviourCreator: Unknown behaviour configuration id requested: " << behaviourConfId << endl;
			throw new exception();
			break;
		}
		//return beh;
	}

} /* namespace alica */
