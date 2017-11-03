/*
 * test_task_assignment.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: Stefan Jakob
 */

using namespace std;


#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include "clock/AlicaROSClock.h"
#include "engine/AlicaEngine.h"
#include "engine/IPlanSelector.h"
#include "engine/ITeamManager.h"
#include "engine/RunningPlan.h"
#include "engine/PlanRepository.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/RobotProperties.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Plan.h"
//#include "engine/planselector/TaskAssignment.h"
#include "engine/planselector/PlanSelector.h"
#include "engine/teammanager/Agent.h"
#include "supplementary/IAgentID.h"

#include <gtest/gtest.h>
#include <list>
#include <vector>
#include <memory>


class TaskAssignmentTest : public ::testing::Test
{
protected:
	alica::AlicaEngine* ae;
	supplementary::SystemConfig* sc;
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
		cout << sc->getConfigPath() << endl;

		sc->setHostname("nase");
		ae = new alica::AlicaEngine();
		bc = new alica::BehaviourCreator();
		cc = new alica::ConditionCreator();
		uc = new alica::UtilityFunctionCreator();
		crc = new alica::ConstraintCreator();
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
	// fake a list of existing robots
	auto robots = make_shared<vector<const supplementary::IAgentID* > >();
	for (int number = 8; number <= 11; number++)
	{
		const supplementary::IAgentID * agentID =  ae->getID<int>(number);
		robots->push_back(agentID);
	}

	// fake inform the team observer about roles of none existing robots
	auto& roles = ae->getPlanRepository()->getRoles();
	int i = 0;
	for (auto& role : roles)
	{
//		ae->getTeamManager()->getAgentByID(robots->at(i))->getEngineData()->setLastRole(role.second);
		i++;
		if (i > 4)
			break;
	}

	auto planMap = ae->getPlanRepository()->getPlans();
	auto rp = make_shared<alica::RunningPlan>(ae, (*planMap.find(1407152758497)).second);
	list<alica::AbstractPlan*>* planList = new list<alica::AbstractPlan*>();
	planList->push_back((*planMap.find(1407152758497)).second);
	alica::IPlanSelector* ps = ae->getPlanSelector();
	auto plans = ps->getPlansForState(rp, planList, robots);
}

