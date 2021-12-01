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
        if (agentNumber) {
            return "hairy";
        } else {
            return "nase";
        }
    }

    static void stepBothAgents(std::vector<alica::AlicaContext*> acs)
    {
        acs[0]->stepEngine();
        acs[1]->stepEngine();
    }

    static constexpr long MasterPlanInitStateId = 4467904887554008050;
    static constexpr long MasterPlanStartStateId = 751302000461175045;
    static constexpr long DynamicTaskAssignmentPlanId = 2252865124432942907;
    static constexpr long DynamicTaskBehaviorId = 4044546549214673470;
};

TEST_F(AlicaDynamicTaskPlan, testMultipleEntrypoints)
{
    // check that multiple EPs are returned
    // eval is called after cacheEvalData, so cacheEvalData needs to ensure there are already dynamic EPs in the internal variables
    // check tha "eval"'s assignments have the dynamic EPs
    /*EXPECT_NE(nullptr, s->getEntryPoint()) << "Isolated state found!";
    checkEntryPoint(const alica::EntryPoint* ep); // test_alica_engine_plan_parser
    for (const alica::EntryPoint* ep : plan->getEntryPoints()) {
                cout << "\t" << ep->getName() << " ID: " << ep->getId() << endl;
                checkEntryPoint(ep, 1402488646221, "MISSING_NAME", "", false, 0, 2147483647, 1402488646220, 1225112227903, "DefaultTask");
            }*/

    const Plan* plan_untouched = aes[0]->getPlanRepository().getPlans().find(DynamicTaskAssignmentPlanId);
    // iterate, find DyanmicTaskAssignemnt
    EXPECT_EQ(1u, plan_untouched->getEntryPoints().size()) << "Number of EntryPoints doesn't match Tackle.pml EntryPoints size." << std::endl;

    // on every tick of the alica engine, before task assignment, the dynamic entry points will be generated for the plan by calling
    //  getApplicationEntrypointContext. So for the first time this method will return just the entrypoints defined in the designer.
    //  Subsequently it will return whatever was the previously computed value.
    // eventually run plan
    const Plan* plan = aes[0]->getPlanBase().getDeepestNode()->getActivePlanAsPlan();
    EXPECT_EQ(2u, plan->getEntryPoints().size()) << "Number of EntryPoints doesn't match Tackle.pml EntryPoints size." << std::endl;
}

/**
 * Tests whether it is possible to instantiate multiple tasks, dynamically, at runtime
 */
TEST_F(AlicaDynamicTaskPlan, runDynamicTasks)
{
    ASSERT_NO_SIGNAL

    aes[0]->start();
    aes[1]->start();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepBothAgents(acs);

    // what's happening here? prev state? before transition?

    std::cout << "AE1 (" << acs[0]->getLocalAgentId() << ")" << std::endl;
    std::cout << "AE2 (" << acs[1]->getLocalAgentId() << ")" << std::endl;

    stepBothAgents(acs);
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[0], MasterPlanInitStateId));
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], MasterPlanInitStateId));

    stepBothAgents(acs);
    alicaTests::TestWorldModel::getOne()->setTransitionCondition4496654201854254411(true);
    alicaTests::TestWorldModel::getTwo()->setTransitionCondition4496654201854254411(true);

    stepBothAgents(acs);
    // what's happening here? prev state? before transition?

    stepBothAgents(acs);
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[0], MasterPlanStartStateId));
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], MasterPlanStartStateId));
    ASSERT_TRUE(alica::test::Util::isPlanActive(aes[0], DynamicTaskAssignmentPlanId));
    ASSERT_TRUE(alica::test::Util::isPlanActive(aes[1], DynamicTaskAssignmentPlanId));

    stepBothAgents(acs);
    ASSERT_GT(std::dynamic_pointer_cast<alica::Attack>(alica::test::Util::getBasicBehaviour(aes[0], DynamicTaskBehaviorId, 0))->callCounter, 2);
    ASSERT_GT(std::dynamic_pointer_cast<alica::Attack>(alica::test::Util::getBasicBehaviour(aes[1], DynamicTaskBehaviorId, 0))->callCounter, 2);
}
} // namespace
} // namespace alica