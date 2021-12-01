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
};

/**
 * Tests whether it is possible to instantiate multiple tasks, dynamically, at runtime
 */
TEST_F(AlicaDynamicTaskPlan, runDynamicTasks)
{
    ASSERT_NO_SIGNAL
    aes[0]->start();
    aes[1]->start();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    acs[0]->stepEngine();
    acs[1]->stepEngine();

    for (int i = 0; i < 20; i++) {
        std::cout << "AE1 step " << i << "(" << acs[0]->getLocalAgentId() << ")" << std::endl;
        acs[0]->stepEngine();

        std::cout << "AE2 step " << i << "(" << acs[1]->getLocalAgentId() << ")" << std::endl;
        acs[1]->stepEngine();

        if (i < 10) {
            ASSERT_TRUE(alica::test::Util::isStateActive(aes[0], 4467904887554008050));
            ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 4467904887554008050));
        }
        if (i == 10) {
            std::cout << "1--------- Initial State passed ---------" << std::endl;
            //TODO
            alicaTests::TestWorldModel::getOne()->setTransitionConditionInitDone(true);
            alicaTests::TestWorldModel::getTwo()->setTransitionConditionInitDone(true);
        }
        if (i > 11 && i < 15) {
            ASSERT_TRUE(alica::test::Util::isStateActive(aes[0], 751302000461175045));
            ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 751302000461175045));
            ASSERT_TRUE(alica::test::Util::isPlanActive(aes[0], 2252865124432942907));
            ASSERT_TRUE(alica::test::Util::isPlanActive(aes[1], 2252865124432942907));
        }
        if (i == 15) {
            ASSERT_GT(std::dynamic_pointer_cast<alica::Attack>(alica::test::Util::getBasicBehaviour(aes[0], 4044546549214673470, 0))->callCounter, 5);
            ASSERT_GT(std::dynamic_pointer_cast<alica::Attack>(alica::test::Util::getBasicBehaviour(aes[1], 4044546549214673470, 0))->callCounter, 5);
        }
    }
}
} // namespace
} // namespace alica