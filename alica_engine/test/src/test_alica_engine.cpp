using namespace std;

#include "SystemConfig.h"
#include <gtest/gtest.h>
#include <map>
#include <list>

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/model/Plan.h"
#include "engine/IBehaviourPool.h"
#include "engine/model/Behaviour.h"
#include "engine/BasicBehaviour.h"
#include "engine/model/EntryPoint.h"
#include "TestBehaviourCreator.h"
#include "engine/model/State.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/Task.h"
#include "engine/model/AlicaElement.h"

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

	static void checkAlicaElement(alica::AlicaElement* ae, long id, string name, string comment)
	{
		EXPECT_EQ(ae->getId(), id) << "Wrong ID!" << endl;
		EXPECT_STREQ(ae->getName().c_str(), name.c_str()) << "Wrong Name!" << endl;
		EXPECT_STREQ(ae->getComment().c_str(), comment.c_str()) << "Wrong Comment!" << endl;
	}

	static void checkState(alica::State* s, long id, string name, string comment, initializer_list<long> absPlanIDs,
							long entryPointID = 0)
	{
		checkAlicaElement(s, id, name, comment);
		if (entryPointID != 0)
		{
			EXPECT_EQ(s->getEntryPoint()->getId(), entryPointID) << "Wrong EntryPoint for state!" << endl;
		}
		EXPECT_EQ(s->getPlans().size(),absPlanIDs.size()) << "Number of abstractPlans didnt fit Tackle.pml plans size."
				<< endl;
		for (alica::AbstractPlan* p : s->getPlans())
		{
			EXPECT_TRUE(find(absPlanIDs.begin(),absPlanIDs.end(),p->getId()) != absPlanIDs.end())
					<< "Unknown id for AbstractPlan!" << endl;
		}

	}
	static void checkEntryPoint(alica::EntryPoint* ep, long id, string name, string comment, bool successRequired,
								int minCardinality, int maxCardinality, long stateID, long taskID, string taskName)
	{
		checkAlicaElement(ep, id, name, comment);

		EXPECT_EQ(ep->getSuccessRequired(), successRequired) << "SuccesRequired true instead of false!" << endl;
		EXPECT_EQ(ep->getMinCardinality(), minCardinality) << "Wrong minCardinality ID!" << endl;
		EXPECT_EQ(ep->getMaxCardinality(), maxCardinality) << "Wrong maxCardinality ID!" << endl;
		EXPECT_EQ(ep->getState()->getId(), stateID) << "Wrong stateId for EntryPoint!" << endl;
		EXPECT_EQ(ep->getTask()->getId(), taskID) << "Wrong TaskId for EntryPoint!" << endl;
		EXPECT_STREQ(ep->getTask()->getName().c_str(), taskName.c_str()) << "Wrong taskName!" << endl;
	}

	static void checkPlan(alica::Plan* plan, long id, string name, string comment, bool masterPlan, double utilityThreshold, int minCardinality, int maxCardinality)
	{
		checkAlicaElement(plan,id, name, comment);
		EXPECT_EQ(plan->isMasterPlan(), masterPlan) << "MasterPlan true instead of false!" << endl;
		EXPECT_EQ(plan->getUtilityThreshold(), utilityThreshold) << "Wrong utilityThreshold!" << endl;
		EXPECT_EQ(plan->getMinCardinality(),minCardinality) << "Wrong minCardinality!" << endl;
		EXPECT_EQ(plan->getMaxCardinality(),maxCardinality) << "Wrong maxCardinality!" << endl;
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
		EXPECT_TRUE(iter.first == 1402488634525 || iter.first == 1402488893641
				|| iter.first == 1402488870347 || iter.first == 1402488437260
				||iter.first == 1402488770050 || iter.first == 1402489318663) << "ID not part of testplans!" << endl;
		switch (iter.first)
		{
			case 1402488634525:
				checkPlan(iter.second, 1402488634525, "AttackPlan", "", false, 0.1, 0, 2147483647);
				break;
			case 1402488893641:
				checkPlan(iter.second, 1402488893641, "Defend", "", false, 0.1, 0, 2147483647);
				break;
			case 1402488870347:
				checkPlan(iter.second, 1402488870347, "GoalPlan", "", false, 0.1, 0, 2147483647);
				break;
			case 1402488437260:
				checkPlan(iter.second, 1402488437260, "MasterPlan", "comment", true, 0.1, 0, 2147483647);
				break;
			case 1402488770050:
				checkPlan(iter.second, 1402488770050, "MidFieldPlayPlan", "", false, 0.1, 0, 2147483647);
				break;
			case 1402489318663:
				checkPlan(iter.second, 1402489318663, "Tackle", "", false, 0.1, 0, 2147483647);
				cout << "States: " << endl;
				EXPECT_EQ(iter.second->getStates().size(),1)
						<< "Number of states didnt fit Tackle.pml state size. Found " << iter.second->getStates().size()
						<< "but exspected 1!" << endl;
				for (alica::State* s : iter.second->getStates())
				{
					cout << "\t" << s->getName() << " ID: " << s->getId() << endl;
					checkState(s, 1402489329141, "AttackOpp", "", {1402489366699}, 1402489329142);
				}
				cout << "EntryPoints: " << endl;
				EXPECT_EQ(iter.second->getEntryPoints().size(),1)
						<< "Number of EntryPoints didnt fit Tackle.pml EntryPoints size. Found "
						<< iter.second->getStates().size() << "but exspected 1!" << endl;
				for (map<long, alica::EntryPoint*>::const_iterator epIterator = iter.second->getEntryPoints().begin();
						epIterator != iter.second->getEntryPoints().end(); epIterator++)
				{
					cout << "\t" << epIterator->second->getName() << " ID: " << epIterator->second->getId() << endl;
					checkEntryPoint(epIterator->second, 1402489329142, "MISSING_NAME", "", false, 0, 2147483647,
									1402489329141, 1225112227903, "DefaultTask");
				}
				break;
			default:
				cerr
						<< "TEST_F(AlicaEngineTest, planParser) found an id not part of testplans but expect_true for all ids failed!"
						<< endl;
				break;

		}
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
