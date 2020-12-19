#include <engine/Assignment.h>
#include <engine/PlanRepository.h>
#include <engine/model/State.h>
#include <engine/model/Transition.h>
#include <engine/modelmanagement/ModelManager.h>
#include <engine/AlicaContext.h>

#include <essentials/IDManager.h>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>

#include "test_alica.h"

using alica::Assignment;
using alica::EntryPoint;
using alica::ModelManager;
using alica::Plan;
using alica::PlanRepository;
using alica::State;

TEST(Assignment, RobotsInserted)
{
    // determine the path to the test config
    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");

    essentials::IDManager idManager;
    int b1 = 2;
    int b2 = 1;
    int b3 = 3;
    essentials::IdentifierConstPtr robot1 = idManager.getID<int>(b1);
    essentials::IdentifierConstPtr robot2 = idManager.getID<int>(b2);
    essentials::IdentifierConstPtr robot3 = idManager.getID<int>(b3);

    ASSERT_EQ(robot2->getRaw()[0], 0x1);
    ASSERT_EQ(robot1->getRaw()[0], 0x2);

    ASSERT_TRUE(*robot1 > *robot2);
    ASSERT_TRUE(*robot1 < *robot3);

    alica::AlicaContext *ac = new alica::AlicaContext("nase", path + "/etc/", "Roleset", "MasterPlan", true);
    ac->setLocalAgentName("nase");

    PlanRepository repo;
    alica::AlicaEngine *ae = alica::AlicaTestsEngineGetter::getEngine(ac);
    ModelManager modelManager(repo, ae);

    const Plan* stp = modelManager.loadPlanTree("SimpleTestPlan");

    Assignment as1(stp);
    const EntryPoint* ep = stp->getEntryPoints()[0];
    const State* s1 = ep->getState();
    const State* s2 = s1->getOutTransitions()[0]->getOutState();

    ASSERT_EQ(as1.size(), 0);

    as1.addAgent(robot1, ep, s1);

    ASSERT_EQ(as1.size(), 1);

    as1.addAgent(robot2, ep, s1);

    ASSERT_EQ(as1.size(), 2);

    ASSERT_EQ(as1.getStateOfAgent(robot1), as1.getStateOfAgent(robot2));

    as1.addAgent(robot3, ep, s1);

    ASSERT_EQ(as1.size(), 3);

    alica::AgentGrp robots;
    as1.getAgentsInState(s2, robots);
    ASSERT_TRUE(robots.empty());

    as1.getAgentsInState(s1, robots);
    ASSERT_EQ(robots.size(), 3u);

    ASSERT_EQ(robots[0], robot1);
    ASSERT_EQ(robots[1], robot2);
    ASSERT_EQ(robots[2], robot3);

    robots.clear();

    as1.updateAgent(robot1, ep, s2);
    ASSERT_EQ(as1.getStateOfAgent(robot1), s2);

    int i = 0;

    for (essentials::IdentifierConstPtr id : as1.getAgentsInState(s1)) {
        EXPECT_TRUE(bool(id));
        ++i;
    }
    ASSERT_EQ(i, 2);
}