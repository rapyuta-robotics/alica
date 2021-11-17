#include <gtest/gtest.h>
#include <ros/ros.h>
#include <clock/AlicaRosTimer.h>
#include <mutex>
#include <condition_variable>
#include <atomic>

TEST(SyncStopTimerRosTest, FireImmediatelyOnStart)
{
    alicaRosTimer::AlicaRosTimerFactory timerFactory(2);
    ros::Time timerCbTime;
    ros::Time timerStartTime = ros::Time::now();
    auto timer = timerFactory.createTimer([&timerCbTime]() {
        if (timerCbTime.isZero()) {
            timerCbTime = ros::Time::now();
        }
    }, alica::AlicaTime::seconds(1));
    ros::Duration(0.1).sleep();
    timer.reset();
    ASSERT_TRUE(!timerCbTime.isZero());
    ASSERT_TRUE(timerCbTime - timerStartTime < ros::Duration(0.1));
}

TEST(SyncStopTimerRosTest, SequentialCbExecution)
{
    alicaRosTimer::AlicaRosTimerFactory timerFactory(2);
    std::atomic<bool> cbActive(false);
    std::atomic<bool> multipleCbsWereActive(false);
    std::atomic<bool> firstCall(true);
    auto timer = timerFactory.createTimer([&cbActive, &multipleCbsWereActive, &firstCall]() {
        if (cbActive) {
            multipleCbsWereActive = true;
        }
        cbActive = true;
        if (!firstCall) {
            ros::Duration(1.0).sleep();
            firstCall = false;
        }
        cbActive = false;
    }, alica::AlicaTime::milliseconds(10));
    ros::Duration(2).sleep();
    timer.reset();
    ASSERT_FALSE(multipleCbsWereActive);
}

TEST(SyncStopTimerRosTest, CbFrequencyCheck)
{
    alicaRosTimer::AlicaRosTimerFactory timerFactory(2);
    const int NUM = 100;

    // Times are not accurate for <30milliseconds
    for (int i = 30; i <= NUM + 30; ++i) {
        double period = i / 1000.0, totDuration = 1.1, sleepBuffer = 0.1 * totDuration;
        int expCbCnt = totDuration / period;
        expCbCnt = expCbCnt - 0.2 * expCbCnt;
        double periodUpperLimit = 0.3 * period + period, periodLowerLimit = period - 0.1 * period;

        ros::Time lastCbTime;
        bool freqLess = false, freqMore = false;
        int cnt = 0;
        auto timer = timerFactory.createTimer([&]() {
            ++cnt;
            auto now = ros::Time::now();
            if (!lastCbTime.isZero()) {
                freqMore = now - lastCbTime > ros::Duration(periodUpperLimit);
                freqLess = now - lastCbTime < ros::Duration(periodLowerLimit);
            }
            lastCbTime = ros::Time::now();
        }, alica::AlicaTime::seconds(period));

        ros::Duration(totDuration + sleepBuffer).sleep();
        timer.reset();

        std::ostringstream oss;
        oss << "period: " << period << ", totDuration: " << totDuration << ", sleepBuffer: " << sleepBuffer << ", expCbCnt: " << expCbCnt << ", periodUpperLimit: " << periodUpperLimit << ", periodLowerLimit: " << periodLowerLimit << ", freqMore: " << freqMore << ", freqLess: " << freqLess << ", cnt: " << cnt << std::endl;

        ASSERT_FALSE(freqLess) << oss.str();
        ASSERT_FALSE(freqMore) << oss.str();
        ASSERT_GT(cnt, expCbCnt) << oss.str();
    }
}

TEST(SyncStopTimerRosTest, StopChecks)
{
    alicaRosTimer::AlicaRosTimerFactory timerFactory(2);
    ros::Time beforeStopTime = ros::Time::now();
    auto timer = timerFactory.createTimer([]() {
        ros::Duration(1).sleep();
    }, alica::AlicaTime::milliseconds(10));
    timer.reset();
    ASSERT_TRUE(ros::Time::now() - beforeStopTime > ros::Duration(0.7));

    bool firstCall = true;
    timer = timerFactory.createTimer([&firstCall]() {
        if (firstCall) {
            firstCall = false;
            return;
        }
        ros::Duration(1).sleep();
    }, alica::AlicaTime::milliseconds(5));
    ros::Duration(0.15).sleep();
    beforeStopTime = ros::Time::now();
    timer.reset();
    ASSERT_TRUE(ros::Time::now() - beforeStopTime > ros::Duration(0.7));
}

TEST(SyncStopTimerRosTest, SpamStartStopTest)
{
    alicaRosTimer::AlicaRosTimerFactory timerFactory(2);
    int cnt = 0;
    for (int i = 0; i < 10000; ++i) {
        auto timer = timerFactory.createTimer([&cnt]() {
            ++cnt;
            if (cnt & 1) {
                ros::Duration{0, 10}.sleep();
            }
        }, alica::AlicaTime::nanoseconds(10));
        if (i & 1) {
            ros::Duration{0, 10}.sleep();
        }
    }
    ASSERT_GT(cnt, 1000);
}

TEST(DISABLED_SyncStopTimerRosTest, DISABLED_RosTimerAsyncDestruction)
{
    // A test case that demonstrates that ros::Timer's destructor does not wait for the timerCb to finish execution
    // Remove the DISABLED_ prefix's above to run this test case
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    std::atomic<bool> cbCalled = false, cbActive = true;
    ros::Time timeBeforeDestruction;
    {
        auto timer = nh.createTimer(ros::Duration(0.01), [&cbCalled, &cbActive](auto&&... args) {
            cbCalled = true;
            ros::Duration(1).sleep();
            cbActive = false;
        });
        ros::Duration(0.1).sleep();
        timeBeforeDestruction = ros::Time::now();
    }
    auto timeAfterDestruction = ros::Time::now();
    ASSERT_TRUE(cbCalled);
    ASSERT_TRUE(cbActive);
    ASSERT_TRUE(timeAfterDestruction - timeBeforeDestruction < ros::Duration(0.01));
}

TEST(DISABLED_SyncStopTimerRosTest, DISABLED_RosTimerAsyncStop)
{
    // A test case that demonstrates that ros::Timer's stop() does not wait for the timerCb to finish execution
    // Remove the DISABLED_ prefix's above to run this test case
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    std::atomic<bool> cbCalled = false, cbActive = true;
    ros::Time timeBeforeStop;
    auto timer = nh.createTimer(ros::Duration(0.01), [&cbCalled, &cbActive](auto&&... args) {
        cbCalled = true;
        ros::Duration(1).sleep();
        cbActive = false;
    });
    ros::Duration(0.1).sleep();
    timeBeforeStop = ros::Time::now();
    timer.stop();
    auto timeAfterStop = ros::Time::now();
    ASSERT_TRUE(cbCalled);
    ASSERT_TRUE(cbActive);
    ASSERT_TRUE(timeAfterStop - timeBeforeStop < ros::Duration(0.01));
}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "alica_ros_timer_tests");
    ros::NodeHandle nh;
    int result = RUN_ALL_TESTS();
    ros::shutdown();
    return result;
}