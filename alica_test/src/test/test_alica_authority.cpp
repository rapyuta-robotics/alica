/*
 * test_alica_authority.cpp
 *
 *  Created on: Oct 27, 2014
 *      Author: Stefan Jakob
 */

#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>
#include "engine/IAlicaCommunication.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>
#include "TestWorldModel.h"

class AlicaEngineAthorityManager : public ::testing::Test
{
protected:
	supplementary::SystemConfig* sc;
	alica::AlicaEngine* ae;
	alica::AlicaEngine* ae2;
	alica::BehaviourCreator* bc;
	alica::ConditionCreator* cc;
	alica::UtilityFunctionCreator* uc;
	alica::ConstraintCreator* crc;

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
		sc->setHostname("nase");

		// setup the engine
		ae = new alica::AlicaEngine();
		bc = new alica::BehaviourCreator();
		cc = new alica::ConditionCreator();
		uc = new alica::UtilityFunctionCreator();
		crc = new alica::ConstraintCreator();
		ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
		ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
	}

	virtual void TearDown()
	{
		ae->shutdown();
		sc->shutdown();
		ae2->shutdown();
		delete ae->getCommunicator();
		delete ae2->getCommunicator();
		delete ae->getIAlicaClock();
		delete ae2->getIAlicaClock();
		delete cc;
		delete bc;
		delete uc;
		delete crc;
	}

};

TEST_F(AlicaEngineAthorityManager, authority)
{
	sc->setHostname("nase");
	ae = new alica::AlicaEngine();
	ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
	ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
	EXPECT_TRUE(ae->init(bc, cc, uc, crc, "RolesetTA", "AuthorityTestMaster", ".", true))
			<< "Unable to initialise the Alica Engine!";

	sc->setHostname("hairy");
	ae2 = new alica::AlicaEngine();
	ae2->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
	ae2->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae2));
	EXPECT_TRUE(ae2->init(bc, cc, uc, crc, "RolesetTA", "AuthorityTestMaster", ".", true))
			<< "Unable to initialise the Alica Engine!";

	ae->start();
	ae2->start();

}
