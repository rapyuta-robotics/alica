#include <iostream>
#include <typeinfo>
#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>
#include "TestBehaviourCreator.h"
#include <clock/AlicaROSClock.h>
#include "engine/PlanRepository.h"
#include "engine/model/Plan.h"
#include "engine/DefaultUtilityFunction.h"

class PlanBase : public ::testing::Test
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
		sc->setHostname("nase");
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
// Declare a test
TEST_F(PlanBase, planBaseTest)
{
	alica::AlicaEngine* ae = alica::AlicaEngine::getInstance();
	alica::TestBehaviourCreator* bc = new alica::TestBehaviourCreator();
	ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
	ae->init(bc, "Roleset", "MasterPlan", ".", false);
	ae->start();
	sleep(3);
}


