#include "test_alica.h"
#include <alica/test/Util.h>
#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaFailureHandlingEnabledFixture : public AlicaTestFixture
{
protected:
    const char* getMasterPlanName() const override { return "FailureHandlingMaster"; }
    const char* getHostName() const override { return "nase"; }
};

class AlicaFailureHandlingDisabledFixture : public AlicaTestFixture
{
protected:
    const char* getMasterPlanName() const override { return "FailureHandlingMaster"; }
    const char* getHostName() const override { return "hairy"; }
};

TEST_F(AlicaFailureHandlingEnabledFixture, redoPlanOnFailure)
{
    // Checks if the FailurePlan is restarted on failure i.e. the redo plan rule should be applied if a plan fails once
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    const uint64_t FAILURE_PLAN_INIT_STATE = 1171453089016322268;
    const uint64_t FAILURE_PLAN_FAIL_STATE = 3487518754011112127;

    ae->start();
    STEP_UNTIL(test::Util::isStateActive(ae, FAILURE_PLAN_INIT_STATE));
    ASSERT_TRUE(test::Util::isStateActive(ae, FAILURE_PLAN_INIT_STATE));

    wm->setTransitionCondition1446293122737278544(true);
    STEP_UNTIL(test::Util::isStateActive(ae, FAILURE_PLAN_FAIL_STATE));
    ASSERT_TRUE(test::Util::isStateActive(ae, FAILURE_PLAN_FAIL_STATE));

    // Transition to plan failure state, which will redo the plan & we should end up in the init state again
    wm->setTransitionCondition1023566846009251524(true);
    wm->setTransitionCondition1446293122737278544(false);
    STEP_UNTIL(test::Util::isStateActive(ae, FAILURE_PLAN_INIT_STATE));
    ASSERT_TRUE(test::Util::isStateActive(ae, FAILURE_PLAN_INIT_STATE));
}

} // namespace
} // namespace alica