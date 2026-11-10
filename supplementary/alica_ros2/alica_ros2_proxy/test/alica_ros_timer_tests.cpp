#include <atomic>
#include <clock/AlicaRosTimer.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <sstream>
#include <thread>

namespace
{
using SteadyTime = std::chrono::steady_clock::time_point;
using std::chrono::duration;

SteadyTime now()
{
    return std::chrono::steady_clock::now();
}

void sleepFor(double seconds)
{
    std::this_thread::sleep_for(duration<double>(seconds));
}

double secondsBetween(SteadyTime from, SteadyTime to)
{
    return duration<double>(to - from).count();
}
} // namespace

TEST(SyncStopTimerRosTest, FireImmediatelyOnStart)
{
    alicaRosTimer::AlicaRosTimerFactory timerFactory;
    std::atomic<bool> called(false);
    SteadyTime timerCbTime;
    SteadyTime timerStartTime = now();
    auto timer = timerFactory.createTimer(
            [&timerCbTime, &called]() {
                if (!called) {
                    timerCbTime = now();
                    called = true;
                }
            },
            alica::AlicaTime::seconds(1));
    sleepFor(0.1);
    timer.reset();
    ASSERT_TRUE(called);
    ASSERT_LT(secondsBetween(timerStartTime, timerCbTime), 0.1);
}

TEST(SyncStopTimerRosTest, SequentialCbExecution)
{
    alicaRosTimer::AlicaRosTimerFactory timerFactory;
    std::atomic<bool> cbActive(false);
    std::atomic<bool> multipleCbsWereActive(false);
    std::atomic<bool> firstCall(true);
    auto timer = timerFactory.createTimer(
            [&cbActive, &multipleCbsWereActive, &firstCall]() {
                if (cbActive) {
                    multipleCbsWereActive = true;
                }
                cbActive = true;
                // The ROS1 test set firstCall only inside the if, so it never slept. Here every call after the first sleeps,
                // so that the timer fires while a callback is in progress.
                if (!firstCall) {
                    sleepFor(1.0);
                }
                firstCall = false;
                cbActive = false;
            },
            alica::AlicaTime::milliseconds(10));
    sleepFor(2);
    timer.reset();
    ASSERT_FALSE(multipleCbsWereActive);
}

TEST(SyncStopTimerRosTest, CbFrequencyCheck)
{
    alicaRosTimer::AlicaRosTimerFactory timerFactory;

    // Times are not accurate for <30milliseconds.
    // The check uses the number of callbacks in the full duration, not single intervals: one late callback on a loaded
    // machine must not fail the test, and rclcpp makes the interval after a late callback shorter.
    for (int periodMs = 30; periodMs <= 130; periodMs += 10) {
        const double period = periodMs / 1000.0, totDuration = 1.1, sleepBuffer = 0.1 * totDuration;
        const int minCbCnt = static_cast<int>(0.8 * totDuration / period);

        std::atomic<int> cnt(0);
        auto timer = timerFactory.createTimer([&]() { ++cnt; }, alica::AlicaTime::milliseconds(periodMs));

        const SteadyTime start = now();
        sleepFor(totDuration + sleepBuffer);
        timer.reset();
        const double elapsed = secondsBetween(start, now());

        std::ostringstream oss;
        // From the measured time, because this thread can sleep longer than requested.
        // + 1 for the call at start, + 1 for a call at the end of the last period
        const int maxCbCnt = static_cast<int>(elapsed / period) + 2;
        oss << "period: " << period << ", elapsed: " << elapsed << ", minCbCnt: " << minCbCnt << ", maxCbCnt: " << maxCbCnt << ", cnt: " << cnt << std::endl;

        ASSERT_GT(cnt, minCbCnt) << oss.str();
        ASSERT_LE(cnt, maxCbCnt) << oss.str();
    }
}

TEST(SyncStopTimerRosTest, StopChecks)
{
    alicaRosTimer::AlicaRosTimerFactory timerFactory;
    SteadyTime beforeStopTime = now();
    auto timer = timerFactory.createTimer([]() { sleepFor(1); }, alica::AlicaTime::milliseconds(10));
    sleepFor(0.15);
    timer.reset();
    ASSERT_GT(secondsBetween(beforeStopTime, now()), 0.7);

    bool firstCall = true;
    timer = timerFactory.createTimer(
            [&firstCall]() {
                if (firstCall) {
                    firstCall = false;
                    return;
                }
                sleepFor(1);
            },
            alica::AlicaTime::milliseconds(5));
    sleepFor(0.15);
    beforeStopTime = now();
    timer.reset();
    ASSERT_GT(secondsBetween(beforeStopTime, now()), 0.7);
}

TEST(SyncStopTimerRosTest, SpamStartStopTest)
{
    alicaRosTimer::AlicaRosTimerFactory timerFactory;
    std::atomic<int> cnt(0);
    for (int i = 0; i < 10000; ++i) {
        auto timer = timerFactory.createTimer(
                [&cnt]() {
                    ++cnt;
                    if (cnt & 1) {
                        std::this_thread::sleep_for(std::chrono::nanoseconds(10));
                    }
                },
                alica::AlicaTime::nanoseconds(10));
        if (i & 1) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(10));
        }
    }
    ASSERT_GT(cnt, 1000);
}

TEST(SyncStopTimerRosTest, TimersOfOneFactoryRunInParallel)
{
    // In ROS1 the timers ran on a multi threaded spinner. A slow timer must not block the other timers.
    alicaRosTimer::AlicaRosTimerFactory timerFactory;
    std::atomic<bool> firstCall(true);
    std::atomic<int> fastCnt(0);
    auto slowTimer = timerFactory.createTimer(
            [&firstCall]() {
                if (!firstCall) {
                    sleepFor(1);
                }
                firstCall = false;
            },
            alica::AlicaTime::milliseconds(10));
    auto fastTimer = timerFactory.createTimer([&fastCnt]() { ++fastCnt; }, alica::AlicaTime::milliseconds(10));
    sleepFor(0.5);
    const int cnt = fastCnt;
    fastTimer.reset();
    slowTimer.reset();
    ASSERT_GT(cnt, 20);
}

TEST(SyncStopTimerRosTest, InjectedNodeUsesCallerExecutor)
{
    auto node = rclcpp::Node::make_shared("alica_ros_timer_tests_injected");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    alicaRosTimer::AlicaRosTimerFactory timerFactory(node);
    std::atomic<int> cnt(0);
    auto timer = timerFactory.createTimer([&cnt]() { ++cnt; }, alica::AlicaTime::milliseconds(10));

    // Only the immediate call on start, since nothing spins the node yet
    sleepFor(0.2);
    ASSERT_EQ(cnt, 1);

    std::thread spinThread([&executor]() { executor.spin(); });
    sleepFor(0.5);
    timer.reset();
    executor.cancel();
    spinThread.join();
    ASSERT_GT(cnt, 20);
}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
