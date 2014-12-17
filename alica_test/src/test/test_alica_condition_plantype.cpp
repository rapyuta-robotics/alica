/*
 * test_alica_condition_plantype.cpp
 *
 *  Created on: Dec 10, 2014
 *      Author: Paul Panin
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
#include "engine/PlanRepository.h"
#include "engine/UtilityFunction.h"
#include "engine/model/Plan.h"
#include "TestConstantValueSummand.h"
#include "engine/teamobserver/TeamObserver.h"
#include "engine/PlanBase.h"
#include "engine/model/State.h"
#include "TestWorldModel.h"

class AlicaConditionPlanType: public ::testing::Test { /* namespace alicaTests */
protected:
	supplementary::SystemConfig* sc;
	alica::AlicaEngine* ae;
	alica::BehaviourCreator* bc;
	alica::ConditionCreator* cc;
	alica::UtilityFunctionCreator* uc;
	alica::ConstraintCreator* crc;

	virtual void SetUp() {
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
	}

	virtual void TearDown() {

		ae->shutdown();
		sc->shutdown();
		delete ae->getIAlicaClock();
		delete cc;
		delete bc;
		delete uc;
		delete crc;
	}

};
/**
 * Test for Runtime or PreCondition are false with plantypes
 */
TEST_F(AlicaConditionPlanType, conditionPlanTypeTest)
{
	ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
	ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
	EXPECT_TRUE(ae->init(bc, cc, uc, crc, "Roleset", "MasterPlanTestConditionPlanType", ".", true)) << "Unable to initialise the Alica Engine!";

	auto uSummandPreConditionPlan = *((ae->getPlanRepository()->getPlans().find(1418042796751))->second->getUtilityFunction()->getUtilSummands().begin());
	TestConstantValueSummand* dbrPre = dynamic_cast<TestConstantValueSummand*>(uSummandPreConditionPlan);
	dbrPre->robotId = ae->getTeamObserver()->getOwnId();

	auto uSummandOtherPlan = *((ae->getPlanRepository()->getPlans().find(1418042819203))->second->getUtilityFunction()->getUtilSummands().begin());
	TestConstantValueSummand* dbrOther = dynamic_cast<TestConstantValueSummand*>(uSummandOtherPlan);
	dbrOther->robotId = ae->getTeamObserver()->getOwnId();

	auto uSummandRunPlan = *((ae->getPlanRepository()->getPlans().find(1418042806575))->second->getUtilityFunction()->getUtilSummands().begin());
	TestConstantValueSummand* dbrRun = dynamic_cast<TestConstantValueSummand*>(uSummandRunPlan);
	dbrRun->robotId = ae->getTeamObserver()->getOwnId();

	ae->start();

	for (int i = 0; i < 21; i++)
	{
		ae->stepNotify();
		chrono::milliseconds duration(33);
		this_thread::sleep_for(duration);

//		if(i > 1)
//		{
//			long id =  (*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId();
//			string name =  (*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getName();
//			cout << name << " : " << id  << " Iteration : " << i << endl;
//		}
		if(i == 2)
		{
			//Should be OtherPlan --> State
			EXPECT_EQ((*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId(), 1418042819204);
		}
		if(i == 5)
		{
			alicaTests::TestWorldModel::getOne()->setRuntimeCondition1418042967134(true);
		}
		if(i == 6)
		{
			//Should be RunTimeCondition --> State
			EXPECT_EQ((*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId(), 1418042806576);
		}
		if(i == 10)
		{
			alicaTests::TestWorldModel::getOne()->setRuntimeCondition1418042967134(false);
		}
		if(i == 12)
		{
			//Should be OtherPlan --> State
			EXPECT_EQ((*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId(), 1418042819204);
		}
		if(i == 13)
		{
			alicaTests::TestWorldModel::getOne()->setPreCondition1418042929966(true);
		}
		if(i > 14)
		{
			//Should be PreCondition --> State
			EXPECT_EQ((*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId(), 1418042796752);
		}
	}
}
