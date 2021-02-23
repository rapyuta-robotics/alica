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
    bool stepEngine() const override { return false; }
};

TEST_F(AlicaSchedulingPlan, scheduling)
{
    std::cerr << "RUN SCHEDULING TEST" << std::endl;
    ae->start();

    // asserts are within the plan inits and onTerminate methods to prevent skipping steps
    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    while (CounterClass::called != 6) {
        ae->getAlicaClock().sleep(sleepTime);
    }
}

} //namespace
} //namespace alica




