#include "test_alica.h"

#include <engine/logging/AlicaDefaultLogger.h>
#include <logger/AlicaRosLogger.h>

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

TEST_F(AlicaDefaultLoggerTest, testSettingRosLogger)
{
    ASSERT_NO_SIGNAL
    // check that AlicaRosLogger is set.
    alicaRosLogger::AlicaRosLogger* logger = dynamic_cast<alicaRosLogger::AlicaRosLogger*>(AlicaLogger::instance());
    EXPECT_NE(logger, nullptr);
}
} // namespace
} // namespace alica
