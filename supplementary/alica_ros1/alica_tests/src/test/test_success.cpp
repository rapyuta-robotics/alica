#include <alica/test/TestContext.h>
#include <alica_tests/BehaviourCreator.h>
#include <alica_tests/ConditionCreator.h>
#include <alica_tests/PlanCreator.h>
#include <alica_tests/ConstraintCreator.h>
#include <alica_tests/TransitionConditionCreator.h>
#include <alica_tests/UtilityFunctionCreator.h>
#include <communication/AlicaDummyCommunication.h>
#include <clock/AlicaRosTimer.h>
#include <logger/AlicaRosLogger.h>

#include <ros/ros.h>
#include <gtest/gtest.h>

namespace alica::test {

class TestSuccessFixture : public ::testing::Test {
public:
    virtual void SetUp() override {
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
        AlicaCreators creators{std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                std::make_unique<alica::TransitionConditionCreator>()};
        _tc->init(std::move(creators));
    }

    virtual void TearDown() override {
        _spinner->stop();
        _tc->terminate();
    }
private:
    std::unique_ptr<TestContext> _tc;
    std::unique_ptr<ros::AsyncSpinner> _spinner;
};

TEST_F(TestSuccessFixture, behSuccessTest)
{

}

}