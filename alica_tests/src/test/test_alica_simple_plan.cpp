
#include "Behaviour/Attack.h"
#include "Behaviour/MidFieldStandard.h"
#include "CounterClass.h"

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
#include <test_alica.h>

namespace alica
{
namespace
{

class AlicaSimplePlan : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "SimpleTestPlan"; }
    bool stepEngine() const override { return false; }
};

/**
 * Tests whether it is possible to run a behaviour in a primitive plan.
 */
TEST_F(AlicaSimplePlan, runBehaviourInSimplePlan)
{
    ASSERT_NO_SIGNAL
    ae->start();
    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    do {
        ae->getAlicaClock().sleep(sleepTime);
    } while (!alica::test::Util::isPlanActive(ae, 1412252439925));

    // Check whether RC can be called
    EXPECT_TRUE(ae->getPlanBase().getRootNode()->isRuntimeConditionValid());
    // Check whether RC has been called
    EXPECT_GE(CounterClass::called, 1);
    // Check final state
    EXPECT_TRUE(ac->isStateActive(1412761855746));
    // Check execution of final state behaviour
    EXPECT_TRUE(alica::test::Util::isPlanActive(ae, 1402488848841));
    // Assuming 30 Hz were 11 iterations are executed by MidFieldStandard, we expect at least 29*sleeptime-15 calls on
    // Attack
    EXPECT_GT(std::dynamic_pointer_cast<alica::Attack>(alica::test::Util::getBasicBehaviour(ae, 1402488848841, 0))->callCounter,
            (sleepTime.inSeconds()) * 29 - 15);
    EXPECT_GT(std::dynamic_pointer_cast<alica::Attack>(alica::test::Util::getBasicBehaviour(ae, 1402488848841, 0))->initCounter, 0);

    // Check whether we have been in state1 to execute midfield standard
    EXPECT_GT(std::dynamic_pointer_cast<alica::MidFieldStandard>(alica::test::Util::getBasicBehaviour(ae, 1402488696205, 0))->callCounter, 10);
    CounterClass::called = 0;
}
} // namespace
} // namespace alica