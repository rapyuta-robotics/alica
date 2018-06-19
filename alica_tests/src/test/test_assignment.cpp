#include <SystemConfig.h>
#include <engine/Assignment.h>
#include <engine/PlanRepository.h>
#include <engine/model/State.h>
#include <engine/model/Transition.h>
#include <engine/parser/PlanParser.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <supplementary/AgentIDFactory.h>
#include <vector>

using alica::Assignment;
using alica::EntryPoint;
using alica::Plan;
using alica::PlanParser;
using alica::PlanRepository;
using alica::State;
using supplementary::AgentID;
using supplementary::AgentIDFactory;

TEST(Assignment, RobotsInserted)
{
    // determine the path to the test config
    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");

    // bring up the SystemConfig with the corresponding path
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    sc->setRootPath(path);
    sc->setConfigPath(path + "/etc");
    sc->setHostname("nase");

    AgentIDFactory idfac;
    std::vector<uint8_t> b1 = {0x2, 0, 0, 0};
    std::vector<uint8_t> b2 = {0x1, 0, 0, 0};
    std::vector<uint8_t> b3 = {0x3, 0, 0, 0};
    const AgentID* robot1 = idfac.create(b1);
    const AgentID* robot2 = idfac.create(b2);
    const AgentID* robot3 = idfac.create(b3);

    ASSERT_EQ(robot2->getRaw()[0], 0x1);
    ASSERT_EQ(robot1->getRaw()[0], 0x2);

    ASSERT_TRUE(*robot1 > *robot2);
    ASSERT_TRUE(*robot1 < *robot3);

    PlanRepository repo;
    PlanParser parser(&repo);

    const Plan* stp = parser.parsePlanTree("SimpleTestPlan");

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
    ASSERT_EQ(robots.size(), 3);

    ASSERT_EQ(robots[0], robot1);
    ASSERT_EQ(robots[1], robot2);
    ASSERT_EQ(robots[2], robot3);

    robots.clear();

    as1.updateAgent(robot1, ep, s2);
    ASSERT_EQ(as1.getStateOfAgent(robot1), s2);

    int i = 0;

    for (alica::AgentIDConstPtr id : as1.getAgentsInState(s1)) {
        ++i;
    }
    ASSERT_EQ(i, 2);
}