#include <iostream>
#include <typeinfo>
#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>
#include "../../alica_engine/test/include/TestBehaviourCreator.h"
#include "../include/clock/AlicaROSClock.h"


// Declare a test
TEST(PlanBase, planBaseTest)
{
	supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
	sc->setHostname("nase");
	alica::AlicaEngine* ae = alica::AlicaEngine::getInstance();
	alica::TestBehaviourCreator* bc = new alica::TestBehaviourCreator();
	ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
	ae->init(bc, "Roleset", "MasterPlan", ".", false);
	ae->start();
	sleep(3);
}
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

