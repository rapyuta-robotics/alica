#include "test_alica.h"

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
    STEP_UNTIL(alica::test::Util::isPlanActive(ae, 1522377375148));
    EXPECT_TRUE(alica::test::Util::isPlanActive(ae, 1522377375148));
}
} // namespace
} // namespace alica
