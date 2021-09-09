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

class AlicaSimplePlan : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "FrequencyTestPlan"; }
    bool stepEngine() const override { return false; }
};

/**
 * Tests whether it is possible to run a behaviour in a primitive plan.
 */
TEST_F(AlicaSimplePlan, behFrequency)
{
    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    ae->start();

    while (CounterClass::called == 0) {
        ae->getAlicaClock().sleep(sleepTime * 1);
    }

    ae->getAlicaClock().sleep(sleepTime * 5);

    EXPECT_EQ(CounterClass::called, 150);
}
} // namespace
} // namespace alica