#include <DynamicBehaviourCreator.h>
#include <engine/AlicaContext.h>
#include <engine/Assignment.h>
#include <engine/PlanRepository.h>
#include <engine/model/State.h>
#include <engine/model/Transition.h>
#include <engine/modelmanagement/ModelManager.h>

#include <gtest/gtest.h>
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
    // Path to test configs set by CMake
    std::string path;
#if defined(PLANS)
    path = PLANS;
    path += "/src/test";
#endif

    alica::AgentId robot1 = 2;
    alica::AgentId robot2 = 1;
    alica::AgentId robot3 = 3;

    ASSERT_TRUE(robot1 > robot2);
    ASSERT_TRUE(robot1 < robot3);

    auto ac = std::make_unique<alica::AlicaContext>(alica::AlicaContextParams("nase", {path + "/etc/"}, "Roleset", "MasterPlan", true));

    ASSERT_TRUE(ac->isValid());
    ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
    ac->setTimerFactory<alica::AlicaSystemTimerFactory>();

    alica::AlicaCreators creators = {std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
            std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
            std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>()};

    EXPECT_EQ(0, ac->init(std::move(creators)));
    alica::LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<alicaTests::TestWorldModel>());

    PlanRepository repo;
    alica::AlicaEngine* ae = alica::AlicaTestsEngineGetter::getEngine(ac.get());
    ModelManager modelManager(ae->getConfigChangeListener(), {path + "/etc/"}, repo);

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

    for (alica::AgentId id : as1.getAgentsInState(s1)) {
        EXPECT_TRUE(bool(id));
        ++i;
    }
    ASSERT_EQ(i, 2);
    ac->terminate();
}
