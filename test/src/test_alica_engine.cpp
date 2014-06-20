using namespace std;

#include "SystemConfig.h"
#include <gtest/gtest.h>

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/model/Plan.h"
#include "engine/IBehaviourPool.h"
#include "engine/model/Behaviour.h"
#include "engine/BasicBehaviour.h"
#include "TestBehaviourCreator.h"

class AlicaEngineTest : public ::testing::Test
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
		path = path + "src/alica/test";

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
TEST_F(AlicaEngineTest, initAndShutdown)
{
	alica::TestBehaviourCreator* bc = new alica::TestBehaviourCreator();
	EXPECT_TRUE(ae->init(bc, "Roleset", "MasterPlan", ".", false)) << "Unable to initialise the Alica Engine!";
	EXPECT_TRUE(ae->shutdown()) << "Unable to shutdown the Alica Engine!";
	delete bc;
}

/**
 * Tests the plan parser with some nice plans
 */
TEST_F(AlicaEngineTest, planParser)
{
	ae->init(new alica::TestBehaviourCreator(), "Roleset", "MasterPlan", ".", false);
	auto plans = ae->getPlanRepository()->getPlans();

	cout << "Printing plans from Repository: " << endl;
	for (auto iter : plans)
	{
		cout << "--------- Next Plan: -------------" << endl;
		cout << "ID: " << iter.first << endl;
		cout << "Plan: " << iter.second->toString() << endl;
	}
}

/**
 * Tests the initialisation of the behaviourPool
 */
TEST_F(AlicaEngineTest, behaviourPoolInit)
{
	EXPECT_TRUE(ae->init(new alica::TestBehaviourCreator(), "Roleset", "MasterPlan", ".", false))
			<< "Unable to initialise the Alica Engine!";

	map<long int, alica::Behaviour*> behaviours = ae->getPlanRepository()->getBehaviours();
	alica::IBehaviourPool* bp = ae->getBehaviourPool();
	for (auto behaviourPair : behaviours)
	{
		cout << "Behaviour: " << behaviourPair.second->getName() << endl;
		EXPECT_TRUE(bp->isBehaviourAvailable(behaviourPair.second)) << "Did not find the Behaviour "
				<< behaviourPair.second->getName();
	}
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
