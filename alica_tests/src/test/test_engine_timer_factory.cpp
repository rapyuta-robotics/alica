#include "test_alica.h"

#include <alica/test/Util.h>
#include <engine/IAlicaTimer.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace alica
{
namespace
{

// The engine timer in these tests never fires on its own. The test runs its callback with fire(), on the test thread,
// the way an application main loop or a single-threaded ROS callback queue runs an engine timer on a shared thread.
struct ManualTimerState
{
    std::mutex mutex;
    IAlicaTimer::TimerCb callback;
};

class ManualTimer : public IAlicaTimer
{
public:
    ManualTimer(std::shared_ptr<ManualTimerState> state, TimerCb callback)
            : _state(std::move(state))
    {
        std::lock_guard<std::mutex> lock(_state->mutex);
        _state->callback = std::move(callback);
    }

    // Waits for a callback that fire() is running, as IAlicaTimer requires
    ~ManualTimer() override
    {
        std::lock_guard<std::mutex> lock(_state->mutex);
        _state->callback = nullptr;
    }

private:
    std::shared_ptr<ManualTimerState> _state;
};

class ManualTimerFactory : public IAlicaTimerFactory
{
public:
    ManualTimerFactory(std::shared_ptr<ManualTimerState> state, bool dedicatedThread)
            : _state(std::move(state))
            , _dedicatedThread(dedicatedThread)
    {
    }

    std::unique_ptr<IAlicaTimer> createTimer(IAlicaTimer::TimerCb timerCb, AlicaTime) const override
    {
        return std::make_unique<ManualTimer>(_state, std::move(timerCb));
    }

    bool dedicatedThread() const override { return _dedicatedThread; }

private:
    std::shared_ptr<ManualTimerState> _state;
    const bool _dedicatedThread;
};

constexpr int64_t TEST_STATE_2 = 1412761855746; // SimpleTestPlan's second state, reached once MidFieldStandard succeeds; runs Attack, which never finishes

class EngineTimerFactoryTest : public ::testing::Test, public AlicaTestBase
{
protected:
    void startEngine(bool dedicatedThread)
    {
        ac = std::make_unique<AlicaContext>(AlicaContextParams("nase", getTestFolderPaths(), "Roleset", "SimpleTestPlan", false));
        ASSERT_TRUE(ac->isValid());
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setTimerFactory<AlicaSystemTimerFactory>();
        ac->setEngineTimerFactory<ManualTimerFactory>(_timerState, dedicatedThread);
        ac->setLogger<AlicaDefaultLogger>();
        AlicaCreators creators = {std::make_unique<DynamicConditionCreator>(), std::make_unique<DynamicUtilityFunctionCreator>(),
                std::make_unique<DynamicConstraintCreator>(), std::make_unique<DynamicBehaviourCreator>(), std::make_unique<DynamicPlanCreator>(),
                std::make_unique<DynamicTransitionConditionCreator>()};
        ASSERT_EQ(0, ac->init(std::move(creators), false));
        ae = AlicaTestsEngineGetter::getEngine(ac.get());
        loopTimeUs = AlicaTime::seconds(1.0 / ac->getConfig()["Alica"]["EngineFrequency"].as<double>()).inMicroseconds();
    }

    // Runs the engine timer's callback once on this thread and returns how long it took, in microseconds
    int64_t fire()
    {
        std::lock_guard<std::mutex> lock(_timerState->mutex);
        const auto start = std::chrono::steady_clock::now();
        if (_timerState->callback) {
            _timerState->callback();
        }
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
    }

    // Fires the engine every few milliseconds until TestState2 is active, and returns how long each call took
    std::vector<int64_t> fireUntilTestState2()
    {
        std::vector<int64_t> durations;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!test::Util::isStateActive(ae, TEST_STATE_2) && std::chrono::steady_clock::now() < deadline) {
            durations.push_back(fire());
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return durations;
    }

    static int64_t median(std::vector<int64_t> durations)
    {
        std::nth_element(durations.begin(), durations.begin() + durations.size() / 2, durations.end());
        return durations[durations.size() / 2];
    }

    std::unique_ptr<AlicaContext> ac;
    AlicaEngine* ae{nullptr};
    int64_t loopTimeUs{0};

private:
    std::shared_ptr<ManualTimerState> _timerState = std::make_shared<ManualTimerState>();
};

// Before PR 553, every call waited out the rest of the engine period (33 ms at 30 Hz), which held a shared thread
TEST_F(EngineTimerFactoryTest, sharedThreadCallbackDoesNotWait)
{
    ASSERT_NO_FATAL_FAILURE(startEngine(false));

    const auto durations = fireUntilTestState2();

    EXPECT_TRUE(test::Util::isStateActive(ae, TEST_STATE_2)) << "the engine made no progress in " << durations.size() << " calls";
    ASSERT_FALSE(durations.empty());
    EXPECT_LT(median(durations), loopTimeUs / 3) << "median call time in us over " << durations.size() << " calls, engine period " << loopTimeUs << " us";
}

// A factory with a dedicated thread keeps the wait for fast-path events, as before PR 553
TEST_F(EngineTimerFactoryTest, dedicatedThreadCallbackWaitsForFastPathEvents)
{
    ASSERT_NO_FATAL_FAILURE(startEngine(true));
    fireUntilTestState2();
    ASSERT_TRUE(test::Util::isStateActive(ae, TEST_STATE_2));

    // Attack runs and never finishes, so no fast-path event ends the wait early
    std::vector<int64_t> durations;
    for (int i = 0; i < 10; ++i) {
        durations.push_back(fire());
    }
    EXPECT_GT(median(durations), loopTimeUs / 2) << "median call time in us, engine period " << loopTimeUs << " us";
}

} // namespace
} // namespace alica
