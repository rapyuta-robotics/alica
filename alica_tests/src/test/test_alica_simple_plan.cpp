#include "CounterClass.h"
#include "Behaviour/Attack.h"
#include "Behaviour/MidFieldStandard.h"
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
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
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
    tc->startEngine();
    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    do {
        tc->getAlicaClock().sleep(sleepTime);
    } while (tc->getRootNode() == nullptr);

    // Check whether RC can be called
    EXPECT_TRUE(tc->getRootNode()->isRuntimeConditionValid());
    // Check whether RC has been called

    //	BEFORE
    //	EXPECT_GE(((RunTimeCondition1412781693884*)&*ae->getPlanBase().getRootNode()->getPlan()->getRuntimeCondition()->getBasicCondition())->called,
    // 1);

    EXPECT_GE(CounterClass::called, 1);
    // Check final state
    EXPECT_EQ(tc->getRootNode()->getActiveState()->getId(), 1412761855746);
    // Check execution of final state behaviour
    EXPECT_EQ(tc->getRootNode()->getChildren()[0]->getBasicBehaviour()->getName(), std::string("Attack"));
    // Assuming 30 Hz were 11 iterations are executed by MidFieldStandard, we expect at least 29*sleeptime-15 calls on
    // Attack
    EXPECT_GT(((alica::Attack*) tc->getRootNode()->getChildren()[0]->getBasicBehaviour())->callCounter, (sleepTime.inSeconds()) * 29 - 15);
    EXPECT_GT(((alica::Attack*) tc->getRootNode()->getChildren()[0]->getBasicBehaviour())->initCounter, 0);

    // Check whether we have been in state1 to execute midfield standard
    EXPECT_GT(std::dynamic_pointer_cast<alica::MidFieldStandard>(tc->getBasicBehaviour(1402488696205, 0))->callCounter, 10);
    CounterClass::called = 0;
}
}
}