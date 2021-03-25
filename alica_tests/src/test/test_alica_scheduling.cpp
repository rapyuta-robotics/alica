#include "CounterClass.h"
#include "test_alica.h"

#include <alica/test/Util.h>
#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaSchedulingPlan : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "SchedulingTestMasterPlan"; }
    bool stepEngine() const override { return true; }
};

TEST_F(AlicaSchedulingPlan, scheduling)
{
    ae->start();
    CounterClass::called = 0;

    ac->stepEngine();
    // init of scheduling plan 1
    ASSERT_EQ(1, CounterClass::called);
    CounterClass::called += 1; // allow transition

    ac->stepEngine();
    // init of scheduling plan 2 and 3
    ASSERT_EQ(4, CounterClass::called);
    CounterClass::called += 1; // allow transition

    ac->stepEngine();
    // onTermination of scheduling plan 2 and 3 called
    ASSERT_EQ(7, CounterClass::called);
    CounterClass::called += 1; // allow transition

    ac->stepEngine();
    // onTermination of scheduling plan 1 called
    ASSERT_EQ(9, CounterClass::called);
}

TEST_F(AlicaSchedulingPlan, schedulingSkipState)
{
    ae->start();
    alica::scheduler::Scheduler& scheduler = ae->editScheduler();
    scheduler.terminate();

    CounterClass::called = 1;
    ac->stepEngine();

    CounterClass::called = 2;
    ac->stepEngine();

    CounterClass::called = 3;
    ac->stepEngine();

    bool isPrerequisite = false;
    for (std::weak_ptr<alica::scheduler::Job> weakJob : scheduler.getPrerequisites(5)) {
        std::shared_ptr<alica::scheduler::Job> sharedJob = weakJob.lock();
        if (sharedJob && sharedJob->id == 4) {
            isPrerequisite = true;
            break;
        }
    }
    ASSERT_TRUE(isPrerequisite);

    CounterClass::called = 4;
    ac->stepEngine();
}

TEST(AlicaScheduling, jobOrderingTest)
{
    alica::AlicaTime::seconds(1);
    std::vector<alica::scheduler::Job> jobs;
    std::vector<std::weak_ptr<alica::scheduler::Job>> prerequisites;

    alica::scheduler::Job job1(0, nullptr, prerequisites);
    job1.scheduledTime = alica::AlicaTime::seconds(1);

    alica::scheduler::Job job2(1, nullptr, prerequisites);
    job2.scheduledTime = alica::AlicaTime::seconds(2);

    alica::scheduler::Job job3(2, nullptr, prerequisites);
    job3.scheduledTime = alica::AlicaTime::seconds(3);

    jobs.insert(std::upper_bound(jobs.begin(), jobs.end(), job1), job1);
    jobs.insert(std::upper_bound(jobs.begin(), jobs.end(), job3), job3);
    jobs.insert(std::upper_bound(jobs.begin(), jobs.end(), job2), job2);

    ASSERT_EQ(0, jobs.at(0).id);
    ASSERT_EQ(1, jobs.at(1).id);
    ASSERT_EQ(2, jobs.at(2).id);

    alica::scheduler::Job job4(2, nullptr, prerequisites);
    job3.scheduledTime = alica::AlicaTime::seconds(4);

    ASSERT_EQ(job3, job4);
}

} // namespace
} // namespace alica
