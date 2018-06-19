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

using alica::AlicaTime;

class StillClock : public alica::AlicaClock
{
    virtual alica::AlicaTime now() const override { return AlicaTime::milliseconds(555); }
};

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
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");

        // bring up the SystemConfig with the corresponding path
        sc = supplementary::SystemConfig::getInstance();
        sc->setRootPath(path);
        sc->setConfigPath(path + "/etc");
        cout << sc->getConfigPath() << endl;

        sc->setHostname("nase");
        ae = new alica::AlicaEngine(
                new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "RolesetTA", "MasterPlanTaskAssignment", ".", false);
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
        ae->setAlicaClock(new StillClock());
        ae->init(bc, cc, uc, crc);
    }

    virtual void TearDown()
    {
        ae->shutdown();
        sc->shutdown();
        delete bc;
        delete cc;
        delete uc;
        delete crc;
    }
};

TEST_F(TaskAssignmentTest, constructTaskAssignment)
{
    ASSERT_NO_SIGNAL

    // fake a list of existing robots
    alica::AgentGrp robots;
    for (int number = 8; number <= 11; number++) {
        alica::AgentIDConstPtr agentID = ae->getID<int>(number);
        robots.push_back(agentID);
        ae->getTeamManager()->setTimeLastMsgReceived(agentID, ae->getAlicaClock()->now());
    }
    ae->getTeamObserver()->tick(nullptr);
    ae->getRoleAssignment()->tick();
    // fake inform the team observer about roles of none existing robots

    const alica::PlanRepository::Accessor<alica::Plan>& planMap = ae->getPlanRepository()->getPlans();
    alica::RunningPlan* rp = ae->getPlanBase()->makeRunningPlan(planMap.find(1407152758497));
    alica::AbstractPlanGrp inputPlans;
    inputPlans.push_back(planMap.find(1407152758497));
    alica::PlanSelector* ps = ae->getPlanBase()->getPlanSelector();

    std::vector<alica::RunningPlan*> o_plans;
    bool ok = ps->getPlansForState(rp, inputPlans, robots, o_plans);
    EXPECT_TRUE(ok);
    EXPECT_EQ(o_plans.size(), 1);
}
