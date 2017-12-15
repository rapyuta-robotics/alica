#include <BehaviourCreator.h>
#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>
#include <CGSolver.h>
#include <ConditionCreator.h>
#include <ConstraintCreator.h>
#include <ConstraintTestPlanDummySolver.h>
#include <engine/AlicaEngine.h>
#include <engine/constraintmodul/Query.h>
#include <engine/model/Variable.h>
#include <engine/PlanBase.h>
#include <engine/RunningPlan.h>
#include <FileSystem.h>
#include <gtest/gtest.h>
#include <gtest/internal/gtest-internal.h>
#include <Plans/ProblemModule/QueryBehaviour1.h>
#include <SolverType.h>
#include <SystemConfig.h>
#include <UtilityFunctionCreator.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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
		// determine the path to the test config
		string path = supplementary::FileSystem::getSelfPath();
		int place = path.rfind("devel");
		path = path.substr(0, place);
		path = path + "src/alica/alica_test/src/test";

		// bring up the SystemConfig with the corresponding path
		sc = supplementary::SystemConfig::getInstance();
		sc->setRootPath(path);
		sc->setConfigPath(path + "/etc");

		// setup the engine
		bc = new alica::BehaviourCreator();
		cc = new alica::ConditionCreator();
		uc = new alica::UtilityFunctionCreator();
		crc = new alica::ConstraintCreator();

		sc->setHostname("nase");
		ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "Roleset", "ProblemBuildingMaster", ".", false);
		ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
		ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
		ae->addSolver(SolverType::DUMMYSOLVER, new alica::reasoner::ConstraintTestPlanDummySolver(ae));
		ae->addSolver(SolverType::GRADIENTSOLVER, new alica::reasoner::CGSolver(ae));
		ae->init(bc, cc, uc, crc);
	}

	virtual void TearDown()
	{
		ae->shutdown();
		delete ae->getCommunicator();
		delete ae->getIAlicaClock();
		sc->shutdown();
		delete cc;
		delete bc;
		delete uc;
		delete crc;
	}
};

/**
 * Tests if static variables and binded correctly.
 */
TEST_F(AlicaProblemCompositionTest, SimpleStaticComposition)
{
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

