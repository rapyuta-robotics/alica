#include <test_supplementary.h>

#include <clock/AlicaRosTimer.h>
#include <communication/AlicaRosCommunication.h>
#include <constraintsolver/CGSolver.h>
#include <engine/AlicaContext.h>
#include <engine/AlicaEngine.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <string>

namespace supplementary
{
namespace
{

// AlicaRosTimerFactory that counts how often the engine timer's callback runs
class CountingRosTimerFactory : public alicaRosTimer::AlicaRosTimerFactory
{
public:
    CountingRosTimerFactory(ros::CallbackQueue& queue, bool dedicatedThread, std::atomic<int>& calls)
            : alicaRosTimer::AlicaRosTimerFactory(queue, dedicatedThread)
            , _calls(calls)
    {
    }

    std::unique_ptr<alica::IAlicaTimer> createTimer(alica::IAlicaTimer::TimerCb timerCb, alica::AlicaTime period) const override
    {
        return alicaRosTimer::AlicaRosTimerFactory::createTimer(
                [&calls = _calls, timerCb = std::move(timerCb)]() {
                    ++calls;
                    timerCb();
                },
                period);
    }

private:
    std::atomic<int>& _calls;
};

class EngineTimerFactoryRosTest : public ::testing::Test
{
protected:
    // dedicatedThread is the second argument of AlicaRosTimerFactory for the engine queue
    void startEngine(bool dedicatedThread)
    {
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("rootPath", path, ".");
        ac = std::make_unique<alica::AlicaContext>(alica::AlicaContextParams("nase", {path + "/etc"}, "Roleset", "GSolverMaster", false));
        ASSERT_TRUE(ac->isValid());
        // Behaviours and communication use a queue of their own, so the solver work in GSolverMaster's behaviour stays off the engine queue
        ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>(nodeQueue);
        ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>(nodeQueue);
        ac->setEngineTimerFactory<CountingRosTimerFactory>(engineQueue, dedicatedThread, engineCalls);
        alica::AlicaCreators creators = {std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
                std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
                std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>()};
        // Start later: the behaviour needs the solver, which can only be added after init()
        ASSERT_EQ(0, ac->init(std::move(creators), true));
        ac->addSolver<alica::reasoner::CGSolver>(ac->getConfig());
        nodeSpinner.start();
        alica::AlicaTestsEngineGetter::getEngine(ac.get())->start();
    }

    // Lets the plan start, then returns how often each counter went up during one second
    std::pair<int, int> countForOneSecond()
    {
        ros::WallDuration(0.5).sleep();
        engineCalls = 0;
        otherCalls = 0;
        ros::WallDuration(1.0).sleep();
        return {engineCalls.load(), otherCalls.load()};
    }

    void TearDown() override
    {
        nodeSpinner.stop();
        engineSpinner.stop();
        // Destroys the engine timer while its queue still exists
        ac.reset();
    }

    // Destroyed in reverse order: the context, then the spinners, then the queues they serve
    std::atomic<int> engineCalls{0};
    std::atomic<int> otherCalls{0};
    ros::CallbackQueue nodeQueue;
    ros::CallbackQueue engineQueue;
    ros::AsyncSpinner nodeSpinner{2, &nodeQueue};
    // One thread, as with ros::spin()
    ros::AsyncSpinner engineSpinner{1, &engineQueue};
    std::unique_ptr<alica::AlicaContext> ac;
};

// The engine shares its queue and thread with another callback, so the factory gets the default, dedicatedThread = false.
// Before PR 553, each engine call held the thread for about one engine period (33 ms at 30 Hz), and the 10 ms timer got
// about one call per engine call.
TEST_F(EngineTimerFactoryRosTest, sharedQueueLeavesTimeForOtherCallbacks)
{
    ros::NodeHandle nh;
    nh.setCallbackQueue(&engineQueue);
    ros::WallTimer otherTimer = nh.createWallTimer(ros::WallDuration(0.01), [this](const ros::WallTimerEvent&) { ++otherCalls; });

    ASSERT_NO_FATAL_FAILURE(startEngine(false));
    engineSpinner.start();
    const auto [engine, other] = countForOneSecond();

    EXPECT_GE(engine, 20) << "engine calls in one second, at 30 Hz";
    EXPECT_GE(other, 60) << "calls of the 10 ms timer in one second, with " << engine << " engine calls";
}

// The engine queue has a spinner of its own that nothing else uses, so the factory may pass dedicatedThread = true, and the
// engine waits for fast-path events inside its callback. It still runs at its own rate.
TEST_F(EngineTimerFactoryRosTest, dedicatedQueueKeepsEngineRate)
{
    ASSERT_NO_FATAL_FAILURE(startEngine(true));
    engineSpinner.start();
    const int engine = countForOneSecond().first;

    EXPECT_GE(engine, 20) << "engine calls in one second, at 30 Hz";
}

} // namespace
} // namespace supplementary
