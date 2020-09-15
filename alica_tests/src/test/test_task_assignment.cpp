#include "test_alica.h"

#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"

#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/IRoleAssignment.h>
#include <engine/PlanBase.h>
#include <engine/PlanRepository.h>
#include <engine/RunningPlan.h>
#include <engine/TeamObserver.h>
#include <engine/collections/RobotEngineData.h>
#include <engine/collections/RobotProperties.h>
#include <engine/containers/AgentAnnouncement.h>
#include <engine/model/AbstractPlan.h>
#include <engine/model/ConfAbstractPlanWrapper.h>
#include <engine/model/Plan.h>
#include <engine/planselector/PlanSelector.h>
#include <engine/teammanager/Agent.h>
#include <engine/teammanager/TeamManager.h>
#include <essentials/IdentifierConstPtr.h>

#include <gtest/gtest.h>

#include <list>
#include <vector>

namespace alica
{
namespace
{

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

    alica::AgentAnnouncement aa;
    aa.planHash = 0;
    aa.senderSdk = ac->getVersion();
    aa.token = 55;
    for (int agentId = 8; agentId <= 11; ++agentId) {
        if (agentId == 9) {
            continue;
        }

        aa.senderID = ac->getID<int>(agentId);
        if (agentId == 8) {
            aa.roleId = 1222973297047; // Attacker
            aa.senderName = "hairy";
        } else if (agentId == 10) {
            aa.roleId = 1222973297054; // AttackSupporter
            aa.senderName = "savvy";
        } else if (agentId == 11) {
            aa.roleId = 1222973297056; // Supporter;
            aa.senderName = "myo";
        }

        ae->editTeamManager().handleAgentAnnouncement(aa);
        robots.push_back(aa.senderID);
    }

    ae->editTeamManager().tick();
    ae->editTeamObserver().tick(nullptr);
    ae->editRoleAssignment().tick();

    // fake inform the team observer about roles of none existing robots
    alica::RunningPlan* rp = new RunningPlan(ae, ae->getPlanRepository().getPlans().find(1407152758497), nullptr);
    alica::ConfAbstractPlanWrapperGrp inputWrappers;
    ConfAbstractPlanWrapper* wrapper = new ConfAbstractPlanWrapper();
    wrapper->setAbstractPlan(ae->getPlanRepository().getPlans().find(1407152758497));
    inputWrappers.push_back(wrapper);
    alica::PlanSelector* ps = ae->getPlanBase().getPlanSelector();

    std::vector<alica::RunningPlan*> o_plans;
    bool ok = ps->getPlansForState(rp, inputWrappers, robots, o_plans);
    EXPECT_TRUE(ok);
    EXPECT_EQ(o_plans.size(), 1u);
}
} // namespace
} // namespace alica