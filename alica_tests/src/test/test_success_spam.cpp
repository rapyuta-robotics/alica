#include "test_alica.h"

#include <alica_tests/BehaviourCreator.h>
#include <alica_tests/ConditionCreator.h>
#include <alica_tests/ConstraintCreator.h>
#include <alica_tests/CounterClass.h>
#include <alica_tests/PlanCreator.h>
#include <alica_tests/UtilityFunctionCreator.h>

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
#include <engine/model/Plan.h>
#include <engine/model/RuntimeCondition.h>
#include <engine/model/State.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaSpamSuccess : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "BehaviorSuccessSpamMaster"; }
};

/**
 * Tests whether it is possible to run a behaviour in a primitive plan.
 */
TEST_F(AlicaSpamSuccess, runBehaviour)
{
    ASSERT_NO_SIGNAL
    ae->start();
    for (int i = 0; i < 30 * 6; ++i) {
        ac->stepEngine();
    }
    EXPECT_TRUE(alica::test::Util::isPlanActive(ae, 1522377375148));
}
} // namespace
} // namespace alica
