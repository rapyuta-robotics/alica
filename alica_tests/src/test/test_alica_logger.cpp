#include "test_alica.h"

#include <alica_tests/TestLogger.h>
#include <engine/logging/AlicaDefaultLogger.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaLoggerTest : public AlicaTestFixtureWithLogger
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "SimpleTestPlan"; }
    bool stepEngine() const override { return false; }
};

class AlicaDefaultLoggerTest : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "SimpleTestPlan"; }
    bool stepEngine() const override { return false; }
};

TEST_F(AlicaLoggerTest, testSettingCustomLogger)
{
    ASSERT_NO_SIGNAL
    // check that TestLogger is set as the logger used by the engine.
    alicaTests::TestLogger* logger = dynamic_cast<alicaTests::TestLogger*>(AlicaLogger::instance());
    EXPECT_NE(logger, nullptr);
}

TEST_F(AlicaDefaultLoggerTest, testSettingDefaultLogger)
{
    ASSERT_NO_SIGNAL
    std::cerr << "###################" << std::endl;
    // check that AlicaDefaultLogger is set when no logger is set.
    AlicaDefaultLogger* logger = dynamic_cast<AlicaDefaultLogger*>(AlicaLogger::instance());
    EXPECT_NE(logger, nullptr);
}
} // namespace
} // namespace alica
