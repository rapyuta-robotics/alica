#include <iostream>
#include <typeinfo>
#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>
#include "TestBehaviourCreator.h"
#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>
#include "engine/IAlicaCommunication.h"
#include "engine/PlanRepository.h"
#include "engine/model/Plan.h"
#include "engine/DefaultUtilityFunction.h"
#include "TestConditionCreator.h"
#include "TestConstraintCreator.h"
#include "TestUtilityFunctionCreator.h"

class PlanBaseTest : public ::testing::Test
{
protected:
	supplementary::SystemConfig* sc;
	alica::AlicaEngine* ae;
	alicaTests::TestBehaviourCreator* bc;
	alicaTests::TestConditionCreator* cc;
	alicaTests::TestUtilityFunctionCreator* uc;
	alicaTests::TestConstraintCreator* crc;

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
		sc->setHostname("nase");


		// setup the engine
		ae = new alica::AlicaEngine();
		bc = new alicaTests::TestBehaviourCreator();
		cc = new alicaTests::TestConditionCreator();
		uc = new alicaTests::TestUtilityFunctionCreator();
		crc = new alicaTests::TestConstraintCreator();
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
		delete bc;
		delete uc;
		delete crc;
	}
};
// Declare a test
TEST_F(PlanBaseTest, planBaseTest)
{
	//TODO test something
	ae->start();
	sleep(1);
}


