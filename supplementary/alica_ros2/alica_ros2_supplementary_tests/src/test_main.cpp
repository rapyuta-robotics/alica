#include <test_supplementary.h>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}
