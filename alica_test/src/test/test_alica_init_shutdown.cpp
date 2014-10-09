#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>
#include "TestBehaviourCreator.h"
#include "TestConstraintCreator.h"
#include "TestUtilityFunctionCreator.h"
#include <clock/AlicaROSClock.h>
#include "engine/PlanRepository.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/model/Plan.h"
#include <communication/AlicaRosCommunication.h>
#include "TestConditionCreator.h"

class AlicaEngineTestInit : public ::testing::Test
{
protected:
	supplementary::SystemConfig* sc;
	alica::AlicaEngine* ae;
	alica::TestBehaviourCreator* bc;
	alica::TestConditionCreator* cc;
	alica::TestUtilityFunctionCreator* uc;
	alica::TestConstraintCreator* crc;

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
		ae = alica::AlicaEngine::getInstance();
		bc = new alica::TestBehaviourCreator();
		cc = new alica::TestConditionCreator();
		uc = new alica::TestUtilityFunctionCreator();
		crc = new alica::TestConstraintCreator();
		ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
		ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
		ae->init(bc, cc, uc, crc, "Roleset", "MasterPlan", ".", false);
	}

	virtual void TearDown()
	{
		ae->shutdown();
		sc->shutdown();
		delete ae->getIAlicaClock();
		delete ae->getCommunicator();
		delete cc;
		delete uc;
		delete crc;
		delete bc;
	}
};

/**
 * Initialises an instance of the AlicaEngine and shuts it down again. This test is nice for basic memory leak testing.
 */
TEST_F(AlicaEngineTestInit, initAndShutdown)
{

	EXPECT_TRUE(ae->init(bc, cc, uc, crc, "Roleset", "MasterPlan", ".", false)) << "Unable to initialise the Alica Engine!";

}

