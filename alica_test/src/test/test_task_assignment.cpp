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

class TaskAssignmentTest : public ::testing::Test
{
protected:
	alica::AlicaEngine* ae;
	supplementary::SystemConfig* sc;

	virtual void SetUp()
	{
		sc = supplementary::SystemConfig::getInstance();
		sc->setHostname("zwerg");
		ae = alica::AlicaEngine::getInstance();
		//TODO extend TestBehaviourCreator
		alica::TestBehaviourCreator* bc = new alica::TestBehaviourCreator();
		ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
		ae->init(bc,"Roleset", "MasterPlan", ".", false);
	}

	virtual void TearDown()
	{
		ae->shutdown();
		sc->shutdown();
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
	auto rp = new alica::RunningPlan((*planMap.find(1402488437260)).second);
	list<alica::AbstractPlan*> planList;
	planList.push_back((*planMap.find(1402488437260)).second);
	auto plans = ps->getPlansForState(rp, planList, robots);
}

