#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include "engine/AgentIDConstPtr.h"
#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/IRoleAssignment.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/TeamObserver.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/RobotProperties.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Plan.h"
#include "engine/planselector/PlanSelector.h"
#include "engine/teammanager/Agent.h"
#include "engine/teammanager/TeamManager.h"

#include <test_alica.h>

#include <gtest/gtest.h>
#include <list>
#include <memory>
#include <ros/ros.h>
#include <vector>

namespace alica
{
namespace
{
// TODO: What to do with this clock?
/*
class StillClock : public alica::AlicaClock
{
    alica::AlicaTime now() const override { return alica::AlicaTime::milliseconds(555); }
};*/

class TaskAssignmentTest : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "RolesetTA"; }
    const char* getMasterPlanName() const override { return "MasterPlanTaskAssignment"; }
    bool stepEngine() const override { return false; }
};

TEST_F(TaskAssignmentTest, constructTaskAssignment)
{
    ASSERT_NO_SIGNAL

    // fake a list of existing robots
    alica::AgentGrp robots;
    for (int number = 8; number <= 11; number++) {
        alica::AgentIDConstPtr agentID = ae->getId<int>(number);
        robots.push_back(agentID);
        ae->getTeamManager().setTimeLastMsgReceived(agentID, ae->getAlicaClock().now());
    }
    ae->getTeamObserver().tick(nullptr);

    ae->getRoleAssignment().tick();
    // fake inform the team observer about roles of none existing robots

    const alica::PlanRepository::Accessor<alica::Plan>& planMap = ae->getPlanRepository().getPlans();
    alica::RunningPlan* rp = ae->getPlanBase().makeRunningPlan(planMap.find(1407152758497));
    alica::AbstractPlanGrp inputPlans;
    inputPlans.push_back(planMap.find(1407152758497));
    alica::PlanSelector* ps = ae->getPlanBase().getPlanSelector();

    std::vector<alica::RunningPlan*> o_plans;
    bool ok = ps->getPlansForState(rp, inputPlans, robots, o_plans);
    EXPECT_TRUE(ok);
    EXPECT_EQ(o_plans.size(), 1);
}
}
}