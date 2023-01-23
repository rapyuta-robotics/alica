#include "test_alica.h"

#include <alica/test/CounterClass.h>
#include <alica_tests/Behaviour/Attack.h>
#include <alica_tests/Behaviour/MidFieldStandard.h>

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
#include <alica_tests/TestFixture.h>
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

void TestFixture::SetUp()
{
    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");
    _tc = std::make_unique<TestContext>("hairy", path + "/etc/", "Roleset", "BehaviourTriggerTestPlan", true, 1);
    ASSERT_TRUE(_tc->isValid());
    const YAML::Node& config = _tc->getConfig();
    _spinner = std::make_unique<ros::AsyncSpinner>(config["Alica"]["ThreadPoolSize"].as<int>(4));
    _tc->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
    _tc->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
    _tc->setLogger<alicaRosLogger::AlicaRosLogger>(config["Local"]["ID"].as<int>());
    LockedBlackboardRW(_tc->editGlobalBlackboard()).set("worldmodel", std::make_shared<alicaTests::TestWorldModelNew>(_tc.get()));
    _spinner->start();
    AlicaCreators creators{std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
            std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
            std::make_unique<alica::TransitionConditionCreator>()};
    _tc->init(std::move(creators));
    _tc->startEngine();
    _spinner->start();

    STEP_UNTIL(_tc, _tc->getActivePlan("BehaviourTriggerTestPlan"));
    ASSERT_TRUE(_tc->getActivePlan("BehaviourTriggerTestPlan")) << _tc->getLastFailure();
}

void TestFixture::TearDown()
{
    _spinner->stop();
    _tc->terminate();
}

}; // namespace alica::test
