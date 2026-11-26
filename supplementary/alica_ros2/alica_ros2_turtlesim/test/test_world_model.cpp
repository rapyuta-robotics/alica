#include <world_model.hpp>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace
{

// Master::onTerminate() deletes the world model this way when the engine terminates. This used to crash: the destructor
// recursed until the stack overflowed, and destroying ALICATurtle's still running spin thread called std::terminate.
TEST(ALICATurtleWorldModel, DeleteAfterInit)
{
    turtlesim::ALICATurtleWorldModel::init();
    ASSERT_NE(turtlesim::ALICATurtleWorldModel::get(), nullptr);

    turtlesim::ALICATurtleWorldModel::del();
    EXPECT_EQ(turtlesim::ALICATurtleWorldModel::get(), nullptr);
}

} // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    // ALICATurtle names its topics after the "name" parameter, as the launch file sets it for the node's process. The default,
    // "Hello!", is not a valid topic name.
    const char* rosArgs[] = {argv[0], "--ros-args", "-p", "name:=turtle1"};
    rclcpp::init(4, rosArgs);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
