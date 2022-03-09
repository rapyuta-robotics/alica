#include "test_alica.h"

#include "Behaviour/Attack.h"
#include "Behaviour/MidFieldStandard.h"
#include <alica_tests/CounterClass.h>

#include <Behaviour/ReadConfigurationBehaviour.h>
#include <alica/test/Util.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/Assignment.h>
#include <engine/BasicBehaviour.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/PlanRepository.h>
#include <engine/TeamObserver.h>
#include <engine/model/Behaviour.h>
#include <engine/model/ConfAbstractPlanWrapper.h>
#include <engine/model/Plan.h>
#include <engine/model/RuntimeCondition.h>
#include <engine/model/State.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaConfigurationPlan : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "ConfigurationTestPlan"; }
    bool stepEngine() const override { return true; }
};

/**
 * Tests whether it is possible to read configured behaviours in a primitive plan.
 */
TEST_F(AlicaConfigurationPlan, runBehaviourConfigurationTest)
{
    ASSERT_NO_SIGNAL
    CounterClass::called = 0;
    // START ENGINE
    ae->start();
    STEP_UNTIL(alica::test::Util::isStateActive(ae, 1588246134801));
    STEP_UNTIL(alica::test::Util::isStateActive(ae, 1588069261047));

    // CHECK PLANTYPE
    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1588246134801)) << "Agent is not in state 'ConfA' of the plan ReadConfInPlantype!";
    // CHECK PLAN
    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1588069261047)) << "Agent is not in state 'StateA' of the plan ReadConfigurationPlan!";
    // CHECK BEHAVIOURS - CONF A
    EXPECT_TRUE(
            dynamic_cast<alica::ReadConfigurationBehaviour*>(alica::test::Util::getBasicBehaviour(ae, 1588061129360, 1588061188681))->testValue.compare("1") ==
            0);
    // CHECK BEHAVIOURS - CONF B
    EXPECT_TRUE(
            dynamic_cast<alica::ReadConfigurationBehaviour*>(alica::test::Util::getBasicBehaviour(ae, 1588061129360, 1588061200689))->testValue.compare("2") ==
            0);

    // set counter that is checked in the transition of the master plan
    CounterClass::called = 1;
    STEP_UNTIL(alica::test::Util::isStateActive(ae, 1588246136647));
    STEP_UNTIL(alica::test::Util::isStateActive(ae, 1588069265377));

    // CHECK PLANTYPE
    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1588246136647)) << "Agent is not in state 'ConfB' of the plan ReadConfInPlantype!";

    // CHECK PLAN
    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1588069265377)) << "Agent is not in state 'StateB' of the plan ReadConfigurationPlan!";

    CounterClass::called = 0;
}
} // namespace
} // namespace alica
