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
#include "../test/include/TestBehaviourCreator.h"
#include "engine/model/AbstractPlan.h"
#include "engine/RunningPlan.h"

class TaskAssignmentTest : public ::testing::Test
{
protected:
	alica::AlicaEngine* ae;

	virtual void SetUp()
	{
		ae = alica::AlicaEngine::getInstance();
		//TODO extend TestBehaviourCreator
		alica::TestBehaviourCreator* bc = new alica::TestBehaviourCreator();
		ae->init(bc,"Roleset", "MasterPlan", ".", true);
	}

	virtual void TearDown()
	{
		ae->shutdown();
	}
};

TEST_F(TaskAssignmentTest, constructTaskAssignment)
{
	alica::IPlanSelector* ps = ae->getPlanSelector();
	auto robots = make_shared<vector<int> >(vector<int>());
	for(int i = 1 ; i <= 5; i++)
	{
		robots->push_back(i);
	}
	list<alica::AbstractPlan*> planList = list<alica::AbstractPlan*>();
	cout << "###" << endl;
	auto rp = new alica::RunningPlan();
	cout << "aaaa" << endl;
 	auto plans = ps->getPlansForState(rp, planList, robots);
}

