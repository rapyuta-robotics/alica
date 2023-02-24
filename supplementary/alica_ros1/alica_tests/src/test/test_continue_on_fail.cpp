#include "test_alica.h"

#include <alica_tests/SimpleSwitches.h>

#include <alica/test/Util.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>

namespace alica
{
namespace
{

class FailureHandling : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "HandleFailExplicitMaster"; }
    bool stepEngine() const override { return true; }
};

TEST_F(FailureHandling, continueOnFailure)
{
    ASSERT_NO_SIGNAL
    SimpleSwitches::reset();
    ae->start();
    ac->stepEngine();

    ASSERT_TRUE(alica::test::Util::isStateActive(ae, 1530004915641));

    SimpleSwitches::set(0, true);
    ac->stepEngine();

    ASSERT_TRUE(alica::test::Util::isStateActive(ae, 1530069246104));

    SimpleSwitches::set(1, true);
    ac->stepEngine();

    ASSERT_TRUE(alica::test::Util::isStateActive(ae, 1530004975275));

    SimpleSwitches::set(2, true);
    ac->stepEngine(); // Behavior should be triggered and fail immediately, causing a fast path event before the next line.

    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(33));

    // in rare cases the fast path event may not be reacted on in this iteration and one more step is needed.
    // This happens if the fp-event comes while the engine is still processing.
    ac->stepEngine();

    ASSERT_TRUE(alica::test::Util::isStateActive(ae, 1532424097662));
}
} // namespace
} // namespace alica
