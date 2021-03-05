#include "test_alica.h"
#include "CounterClass.h"

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
    ASSERT_EQ(CounterClass::called, 0);

    ac->stepEngine();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(33));
    // init of scheduling plan 1, 2 and 3 called
    ASSERT_EQ(CounterClass::called, 3);

    ac->stepEngine();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(33));
    // onTermination of scheduling plan 2 and 3 called
    ASSERT_EQ(CounterClass::called, 5);

    ac->stepEngine();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(33));
    // onTermination of scheduling plan 1 called
    ASSERT_EQ(CounterClass::called, 6);
}

} //namespace
} //namespace alica




