#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>
#include "TestBehaviourCreator.h"
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
		ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
		ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
	}

	virtual void TearDown()
	{
		//cout << "Before Shutdown" << endl;
		ae->shutdown();
		sc->shutdown();
		delete ae->getCommunicator();
		delete ae->getIAlicaClock();
		delete cc;
		delete bc;
		//cout << "After Shutdown" << endl;
	}
};

/**
 * Initialises an instance of the AlicaEngine and shuts it down again. This test is nice for basic memory leak testing.
 */
TEST_F(AlicaEngineTestInit, initAndShutdown)
{

	EXPECT_TRUE(ae->init(bc, cc, "Roleset", "MasterPlan", ".", false)) << "Unable to initialise the Alica Engine!";

}

