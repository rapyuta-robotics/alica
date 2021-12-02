#include "test_alica.h"

#include "Behaviour/Attack.h"
#include "Behaviour/MidFieldStandard.h"
#include <alica_tests/CounterClass.h>

#include <alica/test/Util.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/Assignment.h>
#include <engine/BasicBehaviour.h>
#include <engine/BehaviourPool.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/PlanRepository.h>
#include <engine/TeamObserver.h>
#include <engine/model/Behaviour.h>
#include <engine/model/Plan.h>
#include <engine/model/RuntimeCondition.h>
#include <engine/model/State.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaDynamicTaskPlan : public AlicaTestMultiAgentFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "DynamicTaskAssignmentTestMaster"; }
    bool stepEngine() const override { return false; }

    const int agentCount = 2;
    int getAgentCount() const override { return agentCount; }
    const char* getHostName(int agentNumber) const override
    {
        return "agent"+std::to_string(agentNumber);
    }

    static void stepBothAgents(std::vector<alica::AlicaContext*> acs) {
        acs[0]->stepEngine();
        acs[1]->stepEngine();
    }
};

TEST_F(AlicaDynamicTaskPlan, testMultipleEntrypoints)
{
    // check that multiple EPs are returned
    // eval is called after cacheEvalData, so cacheEvalData needs to ensure there are already dynamic EPs in the internal variables
    // check tha "eval"'s assignments have the dynamic EPs
}

/**
 * Tests whether it is possible to instantiate multiple tasks, dynamically, at runtime
 */
TEST_F(AlicaDynamicTaskPlan, runDynamicTasks)
{
    ASSERT_NO_SIGNAL
    constexpr long MasterPlanInitStateId = 4467904887554008050;
    constexpr long MasterPlanStartStateId = 751302000461175045;
    constexpr long DynamicTaskAssignmentPlanId = 2252865124432942907;
    constexpr long DynamicTaskBehaviorId = 4044546549214673470;

    aes[0]->start();
    aes[1]->start();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepBothAgents();

    // what's happening here? prev state? before transition?

    std::cout << "AE1 step " << i << "(" << acs[0]->getLocalAgentId() << ")" << std::endl;
    std::cout << "AE2 step " << i << "(" << acs[1]->getLocalAgentId() << ")" << std::endl;

    stepBothAgents();
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[0], MasterPlanInitStateId));
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], MasterPlanInitStateId));

    stepBothAgents();
    // 4496654201854254411
    alicaTests::TestWorldModel::getOne()->setTransitionCondition4496654201854254411(true);
    alicaTests::TestWorldModel::getTwo()->setTransitionCondition4496654201854254411(true);

    stepBothAgents();
    // what's happening here? prev state? before transition?

    stepBothAgents();
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[0], MasterPlanStartStateId));
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], MasterPlanStartStateId));
    ASSERT_TRUE(alica::test::Util::isPlanActive(aes[0], DynamicTaskAssignmentPlanId));
    ASSERT_TRUE(alica::test::Util::isPlanActive(aes[1], DynamicTaskAssignmentPlanId));

    stepBothAgents();
    ASSERT_GT(std::dynamic_pointer_cast<alica::Attack>(alica::test::Util::getBasicBehaviour(aes[0], DynamicTaskBehaviorId, 0))->callCounter, 2);
    ASSERT_GT(std::dynamic_pointer_cast<alica::Attack>(alica::test::Util::getBasicBehaviour(aes[1], DynamicTaskBehaviorId, 0))->callCounter, 2);
}
} // namespace
} // namespace alica