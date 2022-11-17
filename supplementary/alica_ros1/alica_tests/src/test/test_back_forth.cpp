#include <test_alica.h>

#include <alica/test/CounterClass.h>
#include <alica_tests/SimpleSwitches.h>

#include <alica/test/Util.h>
#include <engine/AlicaEngine.h>
#include <engine/PlanBase.h>

#include <gtest/gtest.h>

#include <iostream>
#include <thread>

namespace alica
{
namespace
{

class BackForthTest : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "BackForth"; }
};

/**
 * Tests if Behaviour with Constraints are called
 */
TEST_F(BackForthTest, testing)
{
    ASSERT_NO_SIGNAL
    CounterClass::called = 0;
    SimpleSwitches::reset();

    ASSERT_EQ(CounterClass::called, 0);
    ae->start();
    ac->stepEngine();

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int curCount = CounterClass::called;
    ASSERT_GT(curCount, 0);
    ASSERT_TRUE(alica::test::Util::isStateActive(ae, 1529456584983));

    SimpleSwitches::set(0, true);
    ac->stepEngine();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ASSERT_GT(CounterClass::called, curCount);
    curCount = CounterClass::called;
    ASSERT_TRUE(alica::test::Util::isStateActive(ae, 1529456591410));

    SimpleSwitches::set(1, true);
    for (int i = 0; i < 10; ++i) {
        ac->stepEngine();
        ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
        ASSERT_GT(CounterClass::called, curCount);
        curCount = CounterClass::called;
    }
}
} // namespace
} // namespace alica
