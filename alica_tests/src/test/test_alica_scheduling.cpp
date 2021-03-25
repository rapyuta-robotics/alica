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
        }
    }
    ASSERT_TRUE(isPrerequisite);

    CounterClass::called = 4;
    ac->stepEngine();
}

} // namespace
} // namespace alica
