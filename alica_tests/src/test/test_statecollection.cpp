
#include <gtest/gtest.h>
#include <engine/collections/StateCollection.h>
#include <engine/model/State.h>
#include <supplementary/AgentIDFactory.h>
#include <vector>

using alica::StateCollection;
using alica::State;
using supplementary::AgentID;
using supplementary::AgentIDFactory;

TEST(StateCollection, RobotsSorted) {
    
    AgentIDFactory idfac;
    std::vector<uint8_t> b1 = {0x2,0,0,0};
    std::vector<uint8_t> b2 = {0x1,0,0,0};
    std::vector<uint8_t> b3 = {0x3,0,0,0};
    const AgentID* robot1 = idfac.create(b1);
    const AgentID* robot2 = idfac.create(b2);
    const AgentID* robot3 = idfac.create(b3);

    EXPECT_EQ(robot2->getRaw()[0],0x1);
    EXPECT_EQ(robot1->getRaw()[0],0x2);

    EXPECT_TRUE(*robot1 > *robot2);
    EXPECT_TRUE(*robot1 < *robot3);
    
    StateCollection col;
    State s1(1);
    State s2(2);
    
    EXPECT_EQ(col.getCount(),0);

    col.setState(robot1,&s1);

    EXPECT_EQ(col.getCount(),1);

    col.setState(robot2,&s1);

    EXPECT_EQ(col.getCount(),2);

    col.setState(robot3,&s1);

    EXPECT_EQ(col.getCount(),3);

    alica::AgentSet robots;
    col.getRobotsInState(&s2,robots);
    EXPECT_TRUE(robots.empty());

    col.getRobotsInState(&s1,robots);
    EXPECT_EQ(robots.size(),3);

    EXPECT_EQ(robots[0],robot1);
    EXPECT_EQ(robots[1],robot2);
    EXPECT_EQ(robots[2],robot3);

    robots.clear();
    
    col.getRobotsInStateSorted(&s1,robots);

    EXPECT_EQ(robots[0],robot2);
    EXPECT_EQ(robots[1],robot1);
    EXPECT_EQ(robots[2],robot3);

}