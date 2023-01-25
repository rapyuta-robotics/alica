#include <alica/test/TestContext.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

namespace alica::test
{

class TestFixture : public ::testing::Test
{

protected:
    void SetUp() override;
    void TearDown() override;

    std::unique_ptr<TestContext> _tc;
    std::unique_ptr<ros::AsyncSpinner> _spinner;
};
} // namespace alica::test
