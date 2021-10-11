#include <alica/test/Util.h>
#include <gtest/gtest.h>
#include <test_alica.h>

#include "DisabledTracing.h"
#include "EnabledTracing.h"

namespace alica
{
namespace
{

class AlicaEngineTraceTest : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "TracingTestMasterPlan"; }
    bool stepEngine() const override { return false; }
};

TEST_F(AlicaEngineTraceTest, testTracingDisabled)
{
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    EXPECT_TRUE(std::dynamic_pointer_cast<alica::DisabledTracing>(alica::test::Util::getBasicBehaviour(ae, 863651328966767832, 0))->isTracingDisabled());
    EXPECT_FALSE(std::dynamic_pointer_cast<alica::EnabledTracing>(alica::test::Util::getBasicBehaviour(ae, 3606794787493300754, 0))->isTracingDisabled());
}
} // namespace
} // namespace alica