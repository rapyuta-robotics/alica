#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>
#include "engine/IAlicaCommunication.h"
#include "TestBehaviourCreator.h"
#include "engine/model/Behaviour.h"
#include "engine/PlanRepository.h"
#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>
#include  "engine/DefaultUtilityFunction.h"
#include "engine/model/Plan.h"

class AlicaEngineTestBehPool : public ::testing::Test
{
protected:
	supplementary::SystemConfig* sc;
	alica::AlicaEngine* ae;
	alica::TestBehaviourCreator* bc;

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
	}

	virtual void TearDown()
	{
		ae->shutdown();
		sc->shutdown();
		delete ae->getCommunicator();
		delete ae->getIAlicaClock();
		delete bc;
	}
};
/**
 * Tests the initialisation of the behaviourPool
 */
TEST_F(AlicaEngineTestBehPool, behaviourPoolInit)
{
	ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
	EXPECT_TRUE(ae->init(bc, "Roleset", "MasterPlan", ".", false))
			<< "Unable to initialise the Alica Engine!";
	ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
	auto behaviours = ae->getPlanRepository()->getBehaviours();
	alica::IBehaviourPool* bp = ae->getBehaviourPool();
	for (auto behaviourPair : behaviours)
	{
		cout << "Behaviour: " << behaviourPair.second->getName() << endl;
	}

}

