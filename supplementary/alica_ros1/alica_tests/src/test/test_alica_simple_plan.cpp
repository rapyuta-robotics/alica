#include "test_alica.h"

#include <alica_tests/Behaviour/Attack.h>
#include <alica_tests/Behaviour/MidFieldStandard.h>
#include <alica_tests/CounterClass.h>

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

#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/BehaviourCreator.h>
#include <alica_tests/ConditionCreator.h>
#include <alica_tests/ConstraintCreator.h>
#include <alica_tests/PlanCreator.h>
#include <alica_tests/TestWorldModel.h>
#include <alica_tests/TransitionConditionCreator.h>
#include <alica_tests/UtilityFunctionCreator.h>
#include <clock/AlicaRosTimer.h>
#include <communication/AlicaDummyCommunication.h>
#include <logger/AlicaRosLogger.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

namespace alica::test
{

class TestSimplePlanFixture : public ::testing::Test
{

public:
    virtual void SetUp() override
    {
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");
        _tc = std::make_unique<TestContext>("hairy", path + "/etc/", "Roleset", "SimpleTestPlan", false, 1);
        ASSERT_TRUE(_tc->isValid());
        const YAML::Node& config = _tc->getConfig();
        _spinner = std::make_unique<ros::AsyncSpinner>(config["Alica"]["ThreadPoolSize"].as<int>(4));
        _tc->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        _tc->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
        _tc->setLogger<alicaRosLogger::AlicaRosLogger>(config["Local"]["ID"].as<int>());
        _tc->setWorldModel<alicaTests::TestWorldModelNew>(_tc.get());
        _spinner->start();
        AlicaCreators creators{std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                std::make_unique<alica::TransitionConditionCreator>()};
        _tc->init(std::move(creators));
    }

    void TearDown() override
    {
        _spinner->stop();
        _tc->terminate();
    }

protected:
    std::unique_ptr<TestContext> _tc;
    std::unique_ptr<ros::AsyncSpinner> _spinner;
};

/**
 * Tests whether it is possible to run a behaviour in a primitive plan.
 */
TEST_F(TestSimplePlanFixture, runBehaviourInSimplePlan)
{
    ASSERT_NO_SIGNAL
    _tc->startEngine();

    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    uint8_t timeoutCount = 0;
    do {
        _tc->sleep(sleepTime);
    } while (!_tc->isPlanActive(1412252439925));

    ASSERT_TRUE(_tc->getActivePlan("SimpleTestPlan"));
    EXPECT_TRUE(_tc->isPlanActive(1412252439925));  // Plan: SimpleTestPlan
    EXPECT_TRUE(_tc->isStateActive(1412761855746)); // State: TestState2

    // Check whether RC can be called
    EXPECT_TRUE(_tc->getRunningPlan("SimpleTestPlan")->isRuntimeConditionValid());
    // Check whether RC has been called
    EXPECT_GE(CounterClass::called, 1);

    timeoutCount = 0;
    while (!_tc->isStateActive(1412761855746) && timeoutCount < 5) {
        _tc->sleep(sleepTime);
        timeoutCount++;
    }
    timeoutCount = 0;

    // Check final state
    EXPECT_TRUE(_tc->isStateActive(1412761855746)); // State: TestState2
    // Check execution of final state behaviour
    EXPECT_TRUE(_tc->isPlanActive(1402488848841)); // Behaviour: Attack

    // We assume at least 30 calls to Attack in (3 * sleepTime) seconds.
    return;
    std::string name = _tc->getName<BasicBehaviour>(1402488848841); // Behaviour: Attack
    while (dynamic_cast<alica::Attack*>(_tc->getActiveBehaviour(name))->callCounter < 30 && timeoutCount < 3) {
        _tc->sleep(sleepTime);
        timeoutCount++;
    }
    timeoutCount = 0;

    EXPECT_GE(dynamic_cast<alica::Attack*>(_tc->getActiveBehaviour(name))->callCounter, 30);
    EXPECT_GT(dynamic_cast<alica::Attack*>(_tc->getActiveBehaviour(name))->initCounter, 0);
    CounterClass::called = 0;
}
} // namespace alica::test
