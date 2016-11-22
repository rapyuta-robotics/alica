#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>
#include "engine/IAlicaCommunication.h"
#include "engine/model/State.h"
#include "engine/model/Behaviour.h"
#include "engine/PlanRepository.h"
#include "engine/BasicBehaviour.h"
#include "engine/IBehaviourPool.h"
#include "engine/PlanBase.h"
#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>
#include <engine/constraintmodul/Query.h>
#include  "engine/DefaultUtilityFunction.h"
#include  "engine/ITeamObserver.h"
#include "engine/model/Plan.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/Assignment.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/collections/StateCollection.h"
#include <thread>
#include <iostream>
#include "SolverType.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include "Plans/ProblemModule/QueryBehaviour1.h"

class AlicaProblemCompositionTest : public ::testing::Test
{
protected:
	supplementary::SystemConfig* sc;
	alica::AlicaEngine* ae;
	alica::BehaviourCreator* bc;
	alica::ConditionCreator* cc;
	alica::UtilityFunctionCreator* uc;
	alica::ConstraintCreator* crc;

	virtual void SetUp()
	{
	}

	virtual void TearDown()
	{
	}
};

/**
 * Tests if static variables and binded correctly.
 */
TEST_F(AlicaProblemCompositionTest, SimpleStaticComposition)
{
	sc->setHostname("nase");
	ae = new alica::AlicaEngine();
	ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
	ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
	ASSERT_TRUE(ae->init(bc, cc, uc, crc, "Roleset", "ProblemBuildingMaster", ".", false))<< "Unable to initialise the Alica Engine!";

	ae->start();

	this_thread::sleep_for(chrono::milliseconds (100));

	auto queryBehaviour1 = dynamic_pointer_cast<QueryBehaviour1>( ae->getPlanBase()->getDeepestNode()->getBasicBehaviour());
	queryBehaviour1->query->getRelevantStaticVariables();
	auto allReps = queryBehaviour1->query->getUniqueVariableStore()->getAllRep();
	for (auto& rep : allReps)
	{
		cout << "Test: '" << rep->getName() << "'" << endl;
	}
}

