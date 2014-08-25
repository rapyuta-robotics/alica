#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>
#include "TestBehaviourCreator.h"
#include <clock/AlicaROSClock.h>

class AlicaEngineTestInit : public ::testing::Test
{
protected:
	supplementary::SystemConfig* sc;
	alica::AlicaEngine* ae;
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
	}

	virtual void TearDown()
	{
		ae->shutdown();
		sc->shutdown();
	}
};

/**
 * Initialises an instance of the AlicaEngine and shuts it down again. This test is nice for basic memory leak testing.
 */
TEST_F(AlicaEngineTestInit, initAndShutdown)
{
	alica::AlicaEngine* ae = alica::AlicaEngine::getInstance();
	alica::TestBehaviourCreator* bc = new alica::TestBehaviourCreator();
	ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
	EXPECT_TRUE(ae->init(bc, "Roleset", "MasterPlan", ".", false)) << "Unable to initialise the Alica Engine!";
}

