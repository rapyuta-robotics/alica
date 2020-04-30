#include "Behaviour/Attack.h"
#include "Behaviour/MidFieldStandard.h"
#include "CounterClass.h"
#include "engine/Assignment.h"
#include "engine/BasicBehaviour.h"
#include "engine/BehaviourPool.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/model/Behaviour.h"
#include "engine/model/Plan.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/State.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "Behaviour/ReadConfigurationBehaviour.h"

#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>

#include <gtest/gtest.h>
#include <test_alica.h>

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
    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    step(ae);

    const std::vector<RunningPlan*> children1 = ae->getPlanBase().getRootNode()->getChildren();
    for (const RunningPlan* rp : children1) {
        std::string value;
        EXPECT_TRUE(rp->getParameter("TestValue", value)) << "Missing conf value 'TestValue' in configuration of running plan!";
        if (rp->isBehaviour()) {
            // CHECK BEHAVIOURS
            EXPECT_TRUE(((alica::ReadConfigurationBehaviour*)rp->getBasicBehaviour())->testValue.compare(value) == 0);
        } else if (rp->getPlanType()) {
            // CHECK PLANTYPE
            EXPECT_EQ(rp->getActiveState()->getId(),1588246134801) << "Agent is not in state 'ConfA' of the plan ReadConfInPlantype!";
        } else {
            // CHECK PLAN
            EXPECT_EQ(rp->getActiveState()->getId(),1588069261047) << "Agent is not in state 'StateA' of the plan ReadConfigurationPlan!";
        }
    }

    // set counter that is checked in the transition of the master plan
    CounterClass::called = 1;
    step(ae);

    const std::vector<RunningPlan*> children2 = ae->getPlanBase().getRootNode()->getChildren();
    for (const RunningPlan* rp : children2) {
        std::string value;
        if (rp->getPlanType()) {
            // CHECK PLANTYPE
            EXPECT_EQ(rp->getActiveState()->getId(),1588246136647) << "Agent is not in state 'ConfB' of the plan ReadConfInPlantype!";
        } else {
            // CHECK PLAN
            EXPECT_EQ(rp->getActiveState()->getId(),1588069265377) << "Agent is not in state 'StateB' of the plan ReadConfigurationPlan!";
        }
    }

    CounterClass::called = 0;
}
} // namespace
} // namespace alica