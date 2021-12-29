#include "test_alica.h"

#include "DynamicTaskBehavior.h"

#include <alica/test/Util.h>
#include <engine/AlicaContext.h>
#include <engine/RunningPlan.h>
#include <engine/model/AlicaElement.h>
#include <engine/model/Behaviour.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <engine/model/State.h>
#include <engine/model/Task.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{
// GTEST_FILTER=AlicaDynamicTaskPlanTest.* catkin run_tests -i alica_tests
class AlicaDynamicTaskPlanTest : public AlicaTestMultiAgentFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "DynamicTaskAssignmentTestMaster"; }

    const int agentCount = 2;
    int getAgentCount() const override { return agentCount; }
    const char* getHostName(int agentNumber) const override
    {
        if (agentNumber) {
            return "hairy";
        } else {
            return "nase";
        }
    }

    void startAgents()
    {
        for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
            aes[agent_index]->start();
        }
    }

    void stepAgents()
    {
        for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
            acs[agent_index]->stepEngine();
        }
    }

    void enableTransitionCondition()
    {
        // do we need to GetOne and GetTwo? :O
        alicaTests::TestWorldModel::getOne()->setTransitionCondition4496654201854254411(true);
        alicaTests::TestWorldModel::getTwo()->setTransitionCondition4496654201854254411(true);
    }

    static void checkAlicaElement(const alica::AlicaElement* alicaElement, long id, std::string name)
    {
        EXPECT_EQ(id, alicaElement->getId());
        EXPECT_STREQ(name.c_str(), alicaElement->getName().c_str());
    }

    static void checkEntryPoint(const alica::EntryPoint* ep, long id, std::string name, bool successRequired, int minCardinality, int maxCardinality,
            long stateID, long taskID, std::string taskName)
    {
        checkAlicaElement(ep, id, name);
        EXPECT_EQ(successRequired, ep->isSuccessRequired());
        EXPECT_EQ(minCardinality, ep->getMinCardinality());
        EXPECT_EQ(maxCardinality, ep->getMaxCardinality());
        EXPECT_EQ(stateID, ep->getState()->getId());
        EXPECT_EQ(taskID, ep->getTask()->getId());
        EXPECT_STREQ(taskName.c_str(), ep->getTask()->getName().c_str());
    }

    static constexpr int64_t kMasterPlanInitStateId = 4467904887554008050;
    static constexpr int64_t kMasterPlanStartStateId = 751302000461175045;
    static constexpr int64_t kDynamicState1Id = 2800951832651805821;
    static constexpr int64_t kDynamicTaskAssignmentPlanId = 2252865124432942907;
    static constexpr int64_t kDynamicTaskBehaviorId = 4044546549214673470;
    static constexpr int64_t kDynamicTaskEntrypointId = 3150793708487666867;
    static constexpr int64_t kDynamicTaskId = 1163169622598227531;

    static constexpr int64_t kDynamicTaskLCEntrypointId = 3626583666892196532;
    static constexpr int64_t kDynamicTaskBehaviourLDStateId = 3534468625273851172;

    static constexpr const char* kDynamicTaskName = "DynamicTask";
};

TEST_F(AlicaDynamicTaskPlanTest, testMultipleEntrypoints)
{
    // Plan initially has no EP
    const Plan* plan[2];
    plan[0] = aes[0]->getPlanRepository().getPlans().find(kDynamicTaskAssignmentPlanId);
    plan[1] = aes[1]->getPlanRepository().getPlans().find(kDynamicTaskAssignmentPlanId);
    EXPECT_EQ(0u, plan[0]->getEntryPoints().size()) << "Number of dynamic EntryPoints should still be empty.";
    EXPECT_EQ(0u, plan[1]->getEntryPoints().size()) << "Number of dynamic EntryPoints should still be empty.";

    // Make agents enter dynamic tasks
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();
    enableTransitionCondition();
    stepAgents();

    // Check that plan now has a dynamic EP
    for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kMasterPlanStartStateId));
        ASSERT_TRUE(alica::test::Util::isPlanActive(aes[agent_index], kDynamicTaskAssignmentPlanId));
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kDynamicState1Id));

        EXPECT_EQ(1u, plan[agent_index]->getEntryPoints().size()) << "A single dynamic EntryPoint should have been created by this point";

        const alica::EntryPoint* ep = plan[agent_index]->getEntryPoints().front();
        checkEntryPoint(ep, kDynamicTaskEntrypointId, "", false, 2, INT_MAX, kDynamicState1Id, kDynamicTaskId, kDynamicTaskName);
        EXPECT_EQ(ep->getDynamicId(), 1);
    }
}

/**
 * Tests whether it is possible to instantiate multiple tasks, dynamically, at runtime
 */
TEST_F(AlicaDynamicTaskPlanTest, runDynamicTasks)
{
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    std::cout << "AE1 (" << acs[0]->getLocalAgentId() << ")" << std::endl;
    std::cout << "AE2 (" << acs[1]->getLocalAgentId() << ")" << std::endl;

    stepAgents();
    for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kMasterPlanInitStateId));
    }

    enableTransitionCondition();
    stepAgents();
    for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kMasterPlanStartStateId));
        ASSERT_TRUE(alica::test::Util::isPlanActive(aes[agent_index], kDynamicTaskAssignmentPlanId));
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kDynamicState1Id));

        ASSERT_EQ(aes[agent_index]->getPlanBase().getDeepestNode()->getActiveEntryPoint()->getId(), kDynamicTaskLCEntrypointId);
        ASSERT_EQ(std::dynamic_pointer_cast<alica::DynamicTaskBehavior>(alica::test::Util::getBasicBehaviour(aes[agent_index], kDynamicTaskBehaviorId, 0))
                          ->callCounter,
                1);
    }
}

/**
 * Tests whether plan serialization works fine
 */
TEST_F(AlicaDynamicTaskPlanTest, serialize)
{
    // Make agents enter dynamic tasks
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();
    enableTransitionCondition();
    stepAgents();

    // Verify the simple plan tree serialization
    IdGrp plan_trees[getAgentCount()];
    for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kDynamicState1Id));
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kDynamicTaskBehaviourLDStateId));

        const RunningPlan* rpRoot = aes[agent_index]->getPlanBase().getRootNode();
        ASSERT_TRUE(rpRoot);

        // Build current Plan Tree
        IdGrp msg_to_send;
        int deepestNode = 0;
        int treeDepth = 0;
        rpRoot->toMessage(msg_to_send, rpRoot, deepestNode, treeDepth);
        plan_trees[agent_index] = msg_to_send;
    }
    std::vector<int64_t> expectedTree = {
            0, 751302000461175045, 1, 2800951832651805821, -1, 2, 1633421497783210879, 3, 2765772942388464345, 0, 3534468625273851172, -1, -1, -1, -1};
    ASSERT_EQ(plan_trees[0], expectedTree);
    ASSERT_EQ(plan_trees[1], expectedTree);
}

/**
 * Tests whether plan deserialization works fine
 */
TEST_F(AlicaDynamicTaskPlanTest, deserialize)
{
    // Make agents enter dynamic tasks
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();
    enableTransitionCondition();
    stepAgents();

    auto ep0 = aes[1]->getPlanBase().getRootNode()->getChildren()[0]->getAssignment().getEntryPointOfAgent(acs[0]->getLocalAgentId());
    ASSERT_TRUE(ep0);
    checkEntryPoint(ep0, kDynamicTaskEntrypointId, "", false, 2, INT_MAX, kDynamicState1Id, kDynamicTaskId, kDynamicTaskName);
    ASSERT_TRUE(ep0->isDynamic());
    ASSERT_EQ(ep0->getDynamicId(), 1);

    auto ep1 = aes[0]->getPlanBase().getRootNode()->getChildren()[0]->getAssignment().getEntryPointOfAgent(acs[1]->getLocalAgentId());
    ASSERT_TRUE(ep1);
    checkEntryPoint(ep1, kDynamicTaskEntrypointId, "", false, 2, INT_MAX, kDynamicState1Id, kDynamicTaskId, kDynamicTaskName);
    ASSERT_TRUE(ep1->isDynamic());
    ASSERT_EQ(ep1->getDynamicId(), 1);
}

} // namespace
} // namespace alica
