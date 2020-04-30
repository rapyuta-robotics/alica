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
    bool stepEngine() const override { return false; }
};

/**
 * Tests whether it is possible to read configured behaviours in a primitive plan.
 */
TEST_F(AlicaConfigurationPlan, runBehaviourConfigurationTest)
{
    ASSERT_NO_SIGNAL
    // START ENGINE
    ae->start();
    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    do {
        ae->getAlicaClock().sleep(sleepTime);
    } while (ae->getPlanBase().getRootNode() == nullptr);

    // CHECK BEHAVIOURS
    const std::vector<RunningPlan*> children = ae->getPlanBase().getRootNode()->getChildren();
    for (const RunningPlan* rp : children) {
        if (!rp->isBehaviour()) {
            continue;
        }
        std::string value;
        EXPECT_TRUE(rp->getParameter("TestValue", value));
        EXPECT_TRUE(((alica::ReadConfigurationBehaviour*)rp->getBasicBehaviour())->testValue.compare(value) == 0);
    }
}
} // namespace
} // namespace alica