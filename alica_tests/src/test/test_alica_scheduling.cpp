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

TEST(AlicaScheduling, jobIsPrerequisiteFree)
{
    std::function<void()> cb;
    std::vector<std::weak_ptr<alica::scheduler::Job>> prerequisites1;
    std::shared_ptr<alica::scheduler::Job> job1 = std::make_shared<alica::scheduler::Job>(0, cb, prerequisites1);

    std::vector<std::weak_ptr<alica::scheduler::Job>> prerequisites2;
    std::weak_ptr<alica::scheduler::Job> job1WeakPtr = job1;
    prerequisites2.push_back(job1WeakPtr);
    std::shared_ptr<alica::scheduler::Job> job2 = std::make_shared<alica::scheduler::Job>(1, cb, prerequisites2);

    ASSERT_TRUE(job1->isPrerequisiteFree());
    ASSERT_FALSE(job2->isPrerequisiteFree());

    job1.reset();

    ASSERT_TRUE(job1 == nullptr);
    ASSERT_TRUE(job1WeakPtr.lock() == nullptr);
    ASSERT_TRUE(job2->isPrerequisiteFree());
}

TEST_F(AlicaSchedulingPlan, schedulerGetNextJobID)
{
    ae->start();
    alica::scheduler::Scheduler& scheduler = ae->editScheduler();

    for(int i = 1; i <= 10; i++) {
        ASSERT_EQ(i, scheduler.getNextJobID());
    }
}

TEST_F(AlicaSchedulingPlan, schedulerGetPrerequisites)
{
    ae->start();
    alica::scheduler::Scheduler& scheduler = ae->editScheduler();
    scheduler.terminate();

    std::function<void()> cb;
    std::vector<std::weak_ptr<alica::scheduler::Job>> prerequisites1;
    std::shared_ptr<alica::scheduler::Job> job1 = std::make_shared<alica::scheduler::Job>(0, cb, prerequisites1);

    std::vector<std::weak_ptr<alica::scheduler::Job>> prerequisites2;
    std::weak_ptr<alica::scheduler::Job> job1WeakPtr = job1;
    prerequisites2.push_back(job1WeakPtr);
    std::shared_ptr<alica::scheduler::Job> job2 = std::make_shared<alica::scheduler::Job>(1, cb, prerequisites2);

    scheduler.schedule(job1);
    scheduler.schedule(job2);

    ASSERT_EQ(0, scheduler.getPrerequisites(0).size());
    ASSERT_EQ(1, scheduler.getPrerequisites(1).size());
    ASSERT_EQ(0, scheduler.getPrerequisites(2).size());

    ASSERT_EQ(job1, scheduler.getPrerequisites(1).front().lock());
}

TEST_F(AlicaSchedulingPlan, schedulerSchedule)
{
    ae->start();
    alica::scheduler::Scheduler& scheduler = ae->editScheduler();
    scheduler.terminate();

    std::function<void()> cb;
    std::vector<std::weak_ptr<alica::scheduler::Job>> prerequisites1;
    std::shared_ptr<alica::scheduler::Job> job1 = std::make_shared<alica::scheduler::Job>(0, cb, prerequisites1);

    std::vector<std::weak_ptr<alica::scheduler::Job>> prerequisites2;
    std::weak_ptr<alica::scheduler::Job> job1WeakPtr = job1;
    prerequisites2.push_back(job1WeakPtr);
    std::shared_ptr<alica::scheduler::Job> job2 = std::make_shared<alica::scheduler::Job>(1, cb, prerequisites2);

    scheduler.schedule(job1);
    scheduler.schedule(job2);
    ASSERT_TRUE(job1->scheduledTime <= job2->scheduledTime);
}

TEST(AlicaScheduling, jobQueue)
{
    alica::AlicaClock clock;

    std::function<void()> cb;
    std::vector<std::weak_ptr<alica::scheduler::Job>> prerequisites1;
    std::shared_ptr<alica::scheduler::Job> job1 = std::make_shared<alica::scheduler::Job>(0, cb, prerequisites1);
    job1->scheduledTime = clock.now() + alica::AlicaTime::minutes(2);

    std::vector<std::weak_ptr<alica::scheduler::Job>> prerequisites2;
    std::weak_ptr<alica::scheduler::Job> job1WeakPtr = job1;
    prerequisites2.push_back(job1WeakPtr);
    std::shared_ptr<alica::scheduler::Job> job2 = std::make_shared<alica::scheduler::Job>(1, cb, prerequisites2);
    job2->scheduledTime = clock.now() + alica::AlicaTime::minutes(1);

    alica::scheduler::JobQueue jobQueue;
    jobQueue.insert(std::move(job1));
    jobQueue.insert(std::move(job2));

    ASSERT_TRUE(jobQueue.getAvailableJob(clock.now() + alica::AlicaTime::minutes(1)) == nullptr);
    ASSERT_EQ(0, jobQueue.getAvailableJob(clock.now() + alica::AlicaTime::minutes(2))->id);
    ASSERT_EQ(1, jobQueue.getAvailableJob(clock.now() + alica::AlicaTime::minutes(1))->id);
}

} // namespace
} // namespace alica
