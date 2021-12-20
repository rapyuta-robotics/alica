#include "test_alica.h"
#include <engine/PlanInterface.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{
class PlanContextTest : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "SimpleTestPlan"; }
    bool stepEngine() const override { return false; }
};

TEST_F(PlanContextTest, testPlanContextIsValid)
{
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100)); // wait for planBase to start

    auto rp = ae->getPlanBase().getRootNode();
    EXPECT_TRUE(rp != nullptr);
    
    ThreadSafePlanInterface invalid = ThreadSafePlanInterface(nullptr);
    ThreadSafePlanInterface valid = ThreadSafePlanInterface(rp);
    EXPECT_FALSE(invalid.isValid());
    EXPECT_TRUE(valid.isValid());
}
}
}