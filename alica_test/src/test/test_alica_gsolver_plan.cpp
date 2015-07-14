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
#include  "engine/DefaultUtilityFunction.h"
#include  "engine/ITeamObserver.h"
#include "engine/model/Plan.h"
#include "engine/model/RuntimeCondition.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include "engine/Assignment.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/collections/StateCollection.h"
#include "Plans/GSolver/SolverTestBehaviour.h"
#include <thread>
#include <iostream>
#include "SolverType.h"
#include <engine/constraintmodul/ConstraintQuery.h>
#include <CGSolver.h>
#include "ConstraintTestPlanDummySolver.h"


class AlicaGSolverPlan : public ::testing::Test
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
		// determine the path to the test config
		string path = supplementary::FileSystem::getSelfPath();
		int place = path.rfind("devel");
		path = path.substr(0, place);
		path = path + "src/alica/alica_test/src/test";

		// bring up the SystemConfig with the corresponding path
		sc = supplementary::SystemConfig::getInstance();
		sc->setRootPath(path);
		sc->setConfigPath(path + "/etc");
		sc->setHostname("nase");

		// setup the engine
		ae = new alica::AlicaEngine();
		bc = new alica::BehaviourCreator();
		cc = new alica::ConditionCreator();
		uc = new alica::UtilityFunctionCreator();
		crc = new alica::ConstraintCreator();
		ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
		ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
		ae->addSolver(SolverType::DUMMYSOLVER, new alica::reasoner::ConstraintTestPlanDummySolver(ae));
		ae->addSolver(SolverType::GRADIENTSOLVER, new alica::reasoner::CGSolver(ae));
	}

	virtual void TearDown()
	{

		ae->shutdown();
		sc->shutdown();
		delete ae->getIAlicaClock();
		delete ae->getCommunicator();
		delete ae->getSolver(SolverType::DUMMYSOLVER);
		delete ae->getSolver(SolverType::GRADIENTSOLVER);
		delete cc;
		delete bc;
		delete uc;
		delete crc;
	}
};
/**
 * Tests if Behaviour with Constraints are called
 */
TEST_F(AlicaGSolverPlan, solverTest)
{
	ae->init(bc, cc, uc, crc, "Roleset", "GSolverMaster", ".", false);
	cout << "Starting engine..." << endl;
	ae->start();

	chrono::milliseconds sleepTime(33);
	this_thread::sleep_for(sleepTime);

	ASSERT_EQ(alica::SolverTestBehaviour::result.size(), 2) << "Wrong result size";
	EXPECT_GT(alica::SolverTestBehaviour::result[0], 4000);
	EXPECT_LT(alica::SolverTestBehaviour::result[0], 5000);
	EXPECT_GT(alica::SolverTestBehaviour::result[1], 7000);
	EXPECT_LT(alica::SolverTestBehaviour::result[1], 8000);
}

