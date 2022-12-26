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

class TestSuccessFixture : public ::testing::Test
{
public:
    virtual void SetUp() override
    {
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");
        _tc = std::make_unique<TestContext>("hairy", path + "/etc/", "Roleset", "TestMasterPlan", true, 1);
        ASSERT_TRUE(_tc->isValid());
        const YAML::Node& config = _tc->getConfig();
        _spinner = std::make_unique<ros::AsyncSpinner>(config["Alica"]["ThreadPoolSize"].as<int>(4));
        _tc->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        _tc->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
        _tc->setLogger<alicaRosLogger::AlicaRosLogger>(config["Local"]["ID"].as<int>());
        _tc->setWorldModel<alicaTests::TestWorldModelNew>(_tc.get());
        AlicaCreators creators{std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                std::make_unique<alica::TransitionConditionCreator>()};
        _tc->init(std::move(creators));
        _tc->startEngine();
        _spinner->start();
        STEP_UNTIL(_tc, _tc->getActivePlan("TestMasterPlan"));
        ASSERT_TRUE(_tc->getActivePlan("TestMasterPlan")) << _tc->getLastFailure();
    }

    virtual void TearDown() override
    {
        _spinner->stop();
        _tc->terminate();
    }

protected:
    std::unique_ptr<TestContext> _tc;
    std::unique_ptr<ros::AsyncSpinner> _spinner;
};

TEST_F(TestSuccessFixture, behSuccessTest)
{
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "BehSuccessTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActiveBehaviour("SuccessOnInitBeh"));
    auto beh = _tc->getActiveBehaviour("SuccessOnInitBeh");
    ASSERT_NE(beh, nullptr) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->isSuccess(beh));
    ASSERT_TRUE(_tc->isSuccess(beh));
}

TEST_F(TestSuccessFixture, planSuccessTest)
{
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "PlanSuccessTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SuccessOnInitPlan"));
    auto plan = _tc->getActivePlan("SuccessOnInitPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->isSuccess(plan));
    ASSERT_TRUE(_tc->isSuccess(plan));
}

TEST_F(TestSuccessFixture, multiPlanInstanceSuccessTest)
{
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "MultiPlanInstanceSuccessTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("ParallelSuccessOnCondPlan"));
    auto parallelPlan = _tc->getActivePlan("ParallelSuccessOnCondPlan");
    ASSERT_NE(parallelPlan, nullptr) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->setTransitionCond("ParallelSuccessOnCondPlan", "WaitForTriggerState", "ParallelExecState")) << _tc->getLastFailure();
    std::string fqnA = "/MultiPlanInstanceSuccessTestState/ParallelSuccessOnCondState/<ParallelExecState,SuccessOnCondWrapperAPlan>/SuccessOnCondState";
    STEP_UNTIL(_tc, _tc->getActivePlan(fqnA));
    auto planInstanceA = _tc->getActivePlan(fqnA);
    ASSERT_NE(planInstanceA, nullptr) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->setTransitionCond(fqnA, "WaitForCondState", "CondSuccessState"));
    STEP_UNTIL(_tc, false);
    ASSERT_TRUE(_tc->isSuccess(planInstanceA));
    ASSERT_TRUE(_tc->isStateActive("ParallelSuccessOnCondPlan", "ParallelExecState")) << _tc->getLastFailure();
    std::string fqnB = "/MultiPlanInstanceSuccessTestState/ParallelSuccessOnCondState/<ParallelExecState,SuccessOnCondWrapperBPlan>/SuccessOnCondState";
    STEP_UNTIL(_tc, _tc->getActivePlan(fqnB));
    auto planInstanceB = _tc->getActivePlan(fqnB);
    ASSERT_NE(planInstanceB, nullptr) << _tc->getLastFailure();
    ASSERT_FALSE(_tc->isSuccess(planInstanceB));
    ASSERT_TRUE(_tc->setTransitionCond(fqnB, "WaitForCondState", "CondSuccessState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->isSuccess(planInstanceB));
    ASSERT_TRUE(_tc->isSuccess(planInstanceB));
    auto wrapperA = _tc->getActivePlan("SuccessOnCondWrapperAPlan");
    auto wrapperB = _tc->getActivePlan("SuccessOnCondWrapperBPlan");
    ASSERT_NE(wrapperA, nullptr) << _tc->getLastFailure();
    ASSERT_NE(wrapperB, nullptr) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->setTransitionCond("SuccessOnCondWrapperAPlan", "SuccessOnCondState", "WrapperASuccessState")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->setTransitionCond("SuccessOnCondWrapperBPlan", "SuccessOnCondState", "WrapperBSuccessState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->isSuccess(parallelPlan));
    ASSERT_TRUE(_tc->isSuccess(parallelPlan));
}

} // namespace alica::test
