#include "test_alica.h"
#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/Behaviour/NotToTrigger.h>
#include <alica_tests/Behaviour/TriggerA.h>
#include <alica_tests/Behaviour/TriggerB.h>
#include <alica_tests/Behaviour/TriggerC.h>
#include <alica_tests/TestFixture.h>
#include <gtest/gtest.h>
namespace alica::test
{

TEST_F(TestFixture, triggerTest)
{

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "BehaviourTriggerTestPlanState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("BehaviourTriggerTestPlan"));
    ASSERT_NE(nullptr, _tc->getActivePlan("BehaviourTriggerTestPlan")) << _tc->getLastFailure();

    ASSERT_EQ(dynamic_cast<alica::TriggerA*>(_tc->getActiveBehaviour("TriggerA"))->callCounter, 0);
    ASSERT_EQ(dynamic_cast<alica::TriggerB*>(_tc->getActiveBehaviour("TriggerB"))->callCounter, 0);
    ASSERT_EQ(dynamic_cast<alica::TriggerC*>(_tc->getActiveBehaviour("TriggerC"))->callCounter, 0);
    ASSERT_EQ(dynamic_cast<alica::NotToTrigger*>(_tc->getActiveBehaviour("NotToTrigger"))->callCounter, 0);

    dynamic_cast<alica::TriggerA*>(_tc->getActiveBehaviour("TriggerA"))->doTrigger();
    dynamic_cast<alica::TriggerB*>(_tc->getActiveBehaviour("TriggerB"))->doTrigger();
    dynamic_cast<alica::TriggerC*>(_tc->getActiveBehaviour("TriggerC"))->doTrigger();

    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::TriggerA*>(_tc->getActiveBehaviour("TriggerA"))->callCounter == 1);
    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::TriggerB*>(_tc->getActiveBehaviour("TriggerB"))->callCounter == 1);
    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::TriggerC*>(_tc->getActiveBehaviour("TriggerC"))->callCounter == 1);

    dynamic_cast<alica::TriggerA*>(_tc->getActiveBehaviour("TriggerA"))->doTrigger();
    dynamic_cast<alica::TriggerB*>(_tc->getActiveBehaviour("TriggerB"))->doTrigger();
    dynamic_cast<alica::TriggerC*>(_tc->getActiveBehaviour("TriggerC"))->doTrigger();

    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::TriggerA*>(_tc->getActiveBehaviour("TriggerA"))->callCounter == 2);
    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::TriggerB*>(_tc->getActiveBehaviour("TriggerB"))->callCounter == 2);
    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::TriggerC*>(_tc->getActiveBehaviour("TriggerC"))->callCounter == 2);

    dynamic_cast<alica::TriggerA*>(_tc->getActiveBehaviour("TriggerA"))->doTrigger();
    dynamic_cast<alica::TriggerB*>(_tc->getActiveBehaviour("TriggerB"))->doTrigger();
    dynamic_cast<alica::TriggerC*>(_tc->getActiveBehaviour("TriggerC"))->doTrigger();

    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::TriggerA*>(_tc->getActiveBehaviour("TriggerA"))->callCounter == 3);
    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::TriggerB*>(_tc->getActiveBehaviour("TriggerB"))->callCounter == 3);
    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::TriggerC*>(_tc->getActiveBehaviour("TriggerC"))->callCounter == 3);

    dynamic_cast<alica::TriggerC*>(_tc->getActiveBehaviour("TriggerC"))->doTrigger();

    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::TriggerA*>(_tc->getActiveBehaviour("TriggerA"))->callCounter == 3);
    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::TriggerB*>(_tc->getActiveBehaviour("TriggerB"))->callCounter == 3);
    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::TriggerC*>(_tc->getActiveBehaviour("TriggerC"))->callCounter == 4);
    STEP_UNTIL_ASSERT_TRUE(_tc, dynamic_cast<alica::NotToTrigger*>(_tc->getActiveBehaviour("NotToTrigger"))->callCounter == 0);
}
} // namespace alica::test
