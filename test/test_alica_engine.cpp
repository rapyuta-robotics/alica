using namespace std;

#include "SystemConfig.h"
#include <gtest/gtest.h>

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/model/Plan.h"

/**
 * Tests the plan parser with some nice plans
 */
TEST(Alica, planParser)
{
	// determine the path to the test config
	string path = supplementary::FileSystem::getSelfPath();
	int place = path.rfind("devel");
	path = path.substr(0, place);
	path = path + "src/alica/test";

	// bring up the SystemConfig with the corresponding path
	supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
	sc->setRootPath(path);
	sc->setConfigPath(path + "/etc");

	// setup the engine
	alica::AlicaEngine* ae = alica::AlicaEngine::getInstance();
	ae->init("WM09", "WM09", "WM09", false);
	const std::map<long int, alica::Plan*, std::less<long int>, std::allocator<std::pair<const long int, alica::Plan*> > > plans =
			ae->getPlanRepository()->getPlans();

	cout << "Printing plans from Repository: " << endl;
	for(map<long, alica::Plan*>::const_iterator iter = plans.begin(); iter != plans.end(); iter++ )
	{
		cout << "--------- Next Plan: -------------" << endl;
		cout << "ID: " << iter->first << endl;
		cout << "Plan: " << iter->second->toString() << endl;
	}
	//ae->start();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
