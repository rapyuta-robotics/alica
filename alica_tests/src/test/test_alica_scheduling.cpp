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
    std::cerr << "scheduling test 1 ------------------------------" << std::endl;
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
    std::cerr << "scheduling test 2 ------------------------------" << std::endl;
    ae->start();
    alica::scheduler::Scheduler& scheduler = ae->editScheduler();
    scheduler.terminate();

    CounterClass::called = 1;
    ac->stepEngine();

    CounterClass::called = 2;
    ac->stepEngine();

    CounterClass::called = 3;
    ac->stepEngine();

    std::shared_ptr<alica::scheduler::Job> firstStateTerminateJob;
    std::shared_ptr<alica::scheduler::Job> thirdStateInitJob;

    std::shared_ptr<alica::scheduler::Job> job = scheduler.popNext();
    std::vector<std::shared_ptr<alica::scheduler::Job>> jobs;
    while (job) {
        if (job->id == 5) {
            thirdStateInitJob = job;
        }
        if (job->id == 4) {
            firstStateTerminateJob = job;
        }
        jobs.push_back(std::move(job));
        job = scheduler.popNext();
    }
    ASSERT_TRUE(thirdStateInitJob->isPrerequisite(*firstStateTerminateJob));
    CounterClass::called = 4;
    ac->stepEngine();
}

} // namespace
} // namespace alica
