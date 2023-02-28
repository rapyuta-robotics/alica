#include "test_alica.h"

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaDefaultLoggerTest : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "SimpleTestPlan"; }
    bool stepEngine() const override { return false; }
};

TEST_F(AlicaDefaultLoggerTest, testSettingDefaultLogger)
{
    ASSERT_NO_SIGNAL
    // check that AlicaRosLogger is set.
    alica::AlicaDefaultLogger* logger = dynamic_cast<alica::AlicaDefaultLogger*>(AlicaLogger::instance());
    EXPECT_NE(logger, nullptr);
}
} // namespace
} // namespace alica
