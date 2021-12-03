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
// GTEST_FILTER=AlicaDynamicTaskPlan.* catkin run_tests -i alica_tests
class AlicaDynamicTaskPlan : public AlicaTestMultiAgentFixture
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

    static void checkAlicaElement(const alica::AlicaElement* ae, long id, std::string name)
    {
        EXPECT_EQ(id, ae->getId()) << "Wrong ID!" << std::endl;
        EXPECT_STREQ(name.c_str(), ae->getName().c_str()) << "Wrong Name for AlicaElement" << std::endl;
    }

    static void checkEntryPoint(const alica::EntryPoint* ep, long id, std::string name, bool successRequired, int minCardinality, int maxCardinality,
            long stateID, long taskID, std::string taskName)
    {
        checkAlicaElement(ep, id, name);
        EXPECT_EQ(successRequired, ep->isSuccessRequired()) << "SuccesRequired true instead of false!" << std::endl;
        EXPECT_EQ(minCardinality, ep->getMinCardinality()) << "Wrong minCardinality ID!" << std::endl;
        EXPECT_EQ(maxCardinality, ep->getMaxCardinality()) << "Wrong maxCardinality ID!" << std::endl;
        EXPECT_EQ(stateID, ep->getState()->getId()) << "Wrong stateId for EntryPoint!" << std::endl;
        EXPECT_EQ(taskID, ep->getTask()->getId()) << "Wrong TaskId for EntryPoint!" << std::endl;
        EXPECT_STREQ(taskName.c_str(), ep->getTask()->getName().c_str()) << "Wrong taskName!" << std::endl;
    }

    static constexpr long kMasterPlanInitStateId = 4467904887554008050;
    static constexpr long kMasterPlanStartStateId = 751302000461175045;
    static constexpr long kDynamicState1Id = 2800951832651805821;
    static constexpr long kDynamicTaskAssignmentPlanId = 2252865124432942907;
    static constexpr long kDynamicTaskBehaviorId = 4044546549214673470;
    static constexpr long kDynamicTaskEntrypointId = 3150793708487666867;
    static constexpr long kDynamicTaskId = 1163169622598227531;
    static constexpr const char* kDynamicTaskName = "DynamicTask";
};

TEST_F(AlicaDynamicTaskPlan, testMultipleEntrypoints)
{
    // Plan initially has a single EP
    const Plan* plan_untouched = aes[0]->getPlanRepository().getPlans().find(kDynamicTaskAssignmentPlanId);
    EXPECT_EQ(1u, plan_untouched->getEntryPoints().size())
            << "Number of EntryPoints doesn't match DynamicTaskAssignmentTest.pml EntryPoints size." << std::endl;

    const alica::EntryPoint* ep = plan_untouched->getEntryPoints().front();
    checkEntryPoint(ep, kDynamicTaskEntrypointId, "", false, 2, INT_MAX, kDynamicState1Id, kDynamicTaskId, kDynamicTaskName);

    // Make agents enter dynamic tasks
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();
    enableTransitionCondition();
    stepAgents();
    for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kMasterPlanStartStateId));
        ASSERT_TRUE(alica::test::Util::isPlanActive(aes[agent_index], kDynamicTaskAssignmentPlanId));
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kDynamicState1Id));

        // TODO Get active plan, ensure it now has multiple EPs
        const Plan* dynamic_plan = aes[agent_index]->getPlanBase().getDeepestNode()->getActivePlanAsPlan();
        // EXPECT_EQ(1 + getAgentCount(), plan->getEntryPoints().size()) << "Number of EntryPoints doesn't match amount of agents!" << std::endl;
    }
}

/**
 * Tests whether it is possible to instantiate multiple tasks, dynamically, at runtime
 */
TEST_F(AlicaDynamicTaskPlan, runDynamicTasks)
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

        ASSERT_EQ(aes[agent_index]->getPlanBase().getDeepestNode()->getActiveEntryPoint()->getId(), kDynamicTaskEntrypointId);
        // TODO ensure new dynamic EP is different from 0
        // ASSERT_NEQ(aes[agent_index]->getPlanBase().getDeepestNode()->getActiveEntryPoint()->getDynamicId(), 0);
        ASSERT_EQ(std::dynamic_pointer_cast<alica::DynamicTaskBehavior>(alica::test::Util::getBasicBehaviour(aes[agent_index], kDynamicTaskBehaviorId, 0))
                          ->callCounter,
                1);
    }
}
} // namespace
} // namespace alica