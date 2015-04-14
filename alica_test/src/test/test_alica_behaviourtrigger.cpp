/*
 * testalicabehaviourtrigger.cpp
 *
 *  Created on: Apr 8, 2015
 *      Author: Stefan Jakob
 */

#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>
#include "engine/IAlicaCommunication.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include "BehaviourCreator.h"
#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>
#include <mutex>
#include <condition_variable>
#include "EventTrigger.h"
#include <SystemConfig.h>
#include <engine/IBehaviourPool.h>
#include <engine/BasicBehaviour.h>
#include <engine/PlanBase.h>
#include <engine/model/BehaviourConfiguration.h>
#include <TestWorldModel.h>
#include <Plans/Behaviour/TriggerA.h>
#include <Plans/Behaviour/TriggerB.h>
#include <Plans/Behaviour/TriggerC.h>
#include <Plans/Behaviour/NotToTrigger.h>

using namespace std;

class AlicaBehaviourTrigger : public ::testing::Test
{
protected:
	supplementary::SystemConfig* sc;
	alica::AlicaEngine* ae;
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
		delete ae->getIAlicaClock();
		delete ae->getCommunicator();
		delete cc;
		delete bc;
		delete uc;
		delete crc;
	}
};

TEST_F(AlicaBehaviourTrigger, triggerTest)
{
	alicaTests::TestWorldModel::getOne()->trigger1 = new supplementary::EventTrigger();
	alicaTests::TestWorldModel::getOne()->trigger2 = new supplementary::EventTrigger();
	ae->init(bc, cc, uc, crc, "Roleset", "BehaviourTriggerTestPlan", ".", false);
	ae->start();
	chrono::milliseconds duration(33);
	this_thread::sleep_for(duration);
	for (auto iter : *ae->getBehaviourPool()->getAvailableBehaviours())
	{
		if (iter.first->getName() == "TriggerA")
		{
			iter.second->setTrigger(alicaTests::TestWorldModel::getOne()->trigger1);
			continue;
		}
		else if (iter.first->getName() == "TriggerB")
		{
			iter.second->setTrigger(alicaTests::TestWorldModel::getOne()->trigger1);
			continue;
		}
		else if (iter.first->getName() == "TriggerC")
		{
			iter.second->setTrigger(alicaTests::TestWorldModel::getOne()->trigger2);
			continue;
		}
		else
		{
			cout << "BehName: " << iter.first->getName() << endl;
			continue;
		}
	}

	for (auto iter : *ae->getBehaviourPool()->getAvailableBehaviours())
	{
		if (iter.first->getName() == "TriggerA")
		{
			EXPECT_EQ(((alica::TriggerA*)(&*iter.second))->callCounter ,0);
			continue;
		}
		else if (iter.first->getName() == "TriggerB")
		{
			EXPECT_EQ(((alica::TriggerB*)(&*iter.second))->callCounter ,0);
			continue;
		}
		else if (iter.first->getName() == "TriggerC")
		{
			EXPECT_EQ(((alica::TriggerC*)(&*iter.second))->callCounter ,0);
			continue;
		}
		else if (iter.first->getName() == "NotToTriggerDefault")
		{
			EXPECT_EQ(((alica::NotToTrigger*)(&*iter.second))->callCounter ,0);
			continue;
		}
		else
		{
			cout << iter.first->getName() << endl;
			EXPECT_TRUE(false);
		}
	}
	alicaTests::TestWorldModel::getOne()->trigger1->run();
	alicaTests::TestWorldModel::getOne()->trigger2->run();
	this_thread::sleep_for(duration);
	alicaTests::TestWorldModel::getOne()->trigger1->run();
	alicaTests::TestWorldModel::getOne()->trigger2->run();
	this_thread::sleep_for(duration);
	alicaTests::TestWorldModel::getOne()->trigger1->run();
	alicaTests::TestWorldModel::getOne()->trigger2->run();
	this_thread::sleep_for(duration);
	alicaTests::TestWorldModel::getOne()->trigger2->run();
	this_thread::sleep_for(duration);

	for (auto iter : *ae->getBehaviourPool()->getAvailableBehaviours())
	{
		if (iter.first->getName() == "TriggerA")
		{
			EXPECT_EQ(((alica::TriggerA*)(&*iter.second))->callCounter ,3);
			continue;
		}
		else if (iter.first->getName() == "TriggerB")
		{
			EXPECT_EQ(((alica::TriggerB*)(&*iter.second))->callCounter ,3);
			continue;
		}
		else if (iter.first->getName() == "TriggerC")
		{
			EXPECT_EQ(((alica::TriggerC*)(&*iter.second))->callCounter ,4);
			continue;
		}
		else if (iter.first->getName() == "NotToTriggerDefault")
		{
			EXPECT_EQ(((alica::NotToTrigger*)(&*iter.second))->callCounter ,0);
			continue;
		}
		else
		{
			EXPECT_TRUE(false);
		}
	}
	cout << "Finished" << endl;
}
