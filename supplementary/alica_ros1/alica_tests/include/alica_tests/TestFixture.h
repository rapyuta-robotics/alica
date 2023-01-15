#include <gtest/gtest.h>
#include <ros/ros.h>
#include <alica/test/TestContext.h>

namespace alica::test
{

class TestFixture : public ::testing::Test
{

public:
    void SetUp() override;
    void TearDown() override;

protected:
    std::unique_ptr<TestContext> _tc;
    std::unique_ptr<ros::AsyncSpinner> _spinner;
};
} // namespace alicaTests
