/*
 * test_task_assignment.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: Stefan Jakob
 */

using namespace std;

#include <gtest/gtest.h>
#include <list>
#include <vector>
#include <memory>

#include "engine/AlicaEngine.h"
#include "engine/planselector/TaskAssignment.h"
#include "engine/planselector/PlanSelector.h"
#include "engine/IPlanSelector.h"
#include "TestBehaviourCreator.h"
#include "engine/model/AbstractPlan.h"
#include "engine/RunningPlan.h"
#include "engine/PlanRepository.h"
#include "engine/model/Plan.h"
#include <clock/AlicaROSClock.h>
#include "TestConditionCreator.h"
#include "TestConstraintCreator.h"
#include "TestUtilityFunctionCreator.h"

class TaskAssignmentTest : public ::testing::Test
{
protected:
	alica::AlicaEngine* ae;
	supplementary::SystemConfig* sc;
	alicaTests::TestBehaviourCreator* bc;
	alicaTests::TestConditionCreator* cc;
	alicaTests::TestUtilityFunctionCreator* uc;
	alicaTests::TestConstraintCreator* crc;

	virtual void SetUp()
	{
		sc = supplementary::SystemConfig::getInstance();
		sc->setHostname("zwerg");
		ae = new alica::AlicaEngine();
		bc = new alicaTests::TestBehaviourCreator();
		cc = new alicaTests::TestConditionCreator();
		uc = new alicaTests::TestUtilityFunctionCreator();
		crc = new alicaTests::TestConstraintCreator();
		ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
		ae->init(bc, cc, uc, crc, "RolesetTA", "MasterPlanTaskAssignment", ".", false);
	}

	virtual void TearDown()
	{
		ae->shutdown();
		sc->shutdown();
		delete bc;
		delete cc;
		delete uc;
		delete crc;
		delete ae->getIAlicaClock();
	}
};

TEST_F(TaskAssignmentTest, constructTaskAssignment)
{

	alica::IPlanSelector* ps = ae->getPlanSelector();
	auto robots = make_shared<vector<int> >();
	for(int i = 1 ; i <= 5; i++)
	{
		robots->push_back(i);
	}
	auto planMap = ae->getPlanRepository()->getPlans();
	auto rp = make_shared<alica::RunningPlan>(ae, (*planMap.find(1407152758497)).second);
	list<alica::AbstractPlan*>* planList = new list<alica::AbstractPlan*>();
	planList->push_back((*planMap.find(1407152758497)).second);
	auto plans = ps->getPlansForState(rp, planList, robots);
}

