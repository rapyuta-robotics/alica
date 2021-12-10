#include <engine/Assignment.h>
#include <engine/PlanRepository.h>
#include <engine/model/State.h>
#include <engine/model/Transition.h>
#include <engine/modelmanagement/ModelManager.h>
#include <engine/AlicaContext.h>
#include <engine/util/HashFunctions.h>

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
    // TODO: fix up this test case
    /*
    // determine the path to the test config
    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");

    // Create 3 robots with out-of-order IDs
    alica::AgentId robot1 = 2;
    alica::AgentId robot2 = 1;
    alica::AgentId robot3 = 3;

    // Prepare ALICA for SimpleTestPlan
    alica::AlicaContext *ac = new alica::AlicaContext(
            alica::AlicaContextParams("nase", path + "/etc/", "Roleset", "MasterPlan", true));
    PlanRepository repo;
    alica::AlicaEngine *ae = alica::AlicaTestsEngineGetter::getEngine(ac);
    ModelManager modelManager(repo, ae, path + "/etc/");
    const Plan* stp = modelManager.loadPlanTree("SimpleTestPlan");
    Assignment as1(alica::contextHash(0), stp);
    ASSERT_EQ(stp->getEntryPoints().size(), 1);
    const EntryPoint* ep = stp->getEntryPoints()[0];
    const State* s1 = ep->getState();
    const State* s2 = s1->getOutTransitions()[0]->getOutState();

    // Add agents to Assignment
    ASSERT_EQ(as1.size(), 0);
    as1.addAgent(robot1, ep, s1);
    ASSERT_EQ(as1.size(), 1);
    as1.addAgent(robot2, ep, s1);
    ASSERT_EQ(as1.size(), 2);
    ASSERT_EQ(as1.getStateOfAgent(robot1), as1.getStateOfAgent(robot2));
    as1.addAgent(robot3, ep, s1);
    ASSERT_EQ(as1.size(), 3);

    // Check which agents are in which states (all three in State1)
    alica::AgentGrp robots;
    as1.getAgentsInState(s2, robots);
    ASSERT_TRUE(robots.empty());
    as1.getAgentsInState(s1, robots);
    ASSERT_EQ(robots.size(), 3u);
    ASSERT_EQ(robots[0], robot1);
    ASSERT_EQ(robots[1], robot2);
    ASSERT_EQ(robots[2], robot3);

    robots.clear();

    // Move robot1 to State2
    as1.updateAgent(robot1, ep, s2);
    ASSERT_EQ(as1.getStateOfAgent(robot1), s2);

    // Check that only 2 agents are in State1 now
    int i = 0;
    for (alica::AgentId id : as1.getAgentsInState(s1)) {
        EXPECT_TRUE(bool(id));
        ++i;
    }
    ASSERT_EQ(i, 2);
    */
}
