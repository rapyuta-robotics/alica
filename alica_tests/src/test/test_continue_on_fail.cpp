#include "SimpleSwitches.h"
#include "engine/IAlicaCommunication.h"
#include "test_alica.h"

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
    step(ae);

    ASSERT_EQ(ae->getPlanBase().getDeepestNode()->getActiveState()->getId(), 1530004915641);

    SimpleSwitches::set(0, true);
    step(ae);

    ASSERT_EQ(ae->getPlanBase().getDeepestNode()->getActiveState()->getId(), 1530069246104);

    SimpleSwitches::set(1, true);
    step(ae);

    ASSERT_EQ(ae->getPlanBase().getDeepestNode()->getActiveState()->getId(), 1530004975275);

    SimpleSwitches::set(2, true);
    step(ae); // Behavior should be triggered and fail immediately, causing a fast path event before the next line.

    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(33));

    // in rare cases the fast path event may not be reacted on in this iteration and one more step is needed.
    // This happens if the fp-event comes while the engine is still processing.
    step(ae);

    ASSERT_EQ(ae->getPlanBase().getDeepestNode()->getActiveState()->getId(), 1532424097662);
}
}
