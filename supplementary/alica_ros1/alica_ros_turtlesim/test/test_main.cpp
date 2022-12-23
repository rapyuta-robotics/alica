#include <geometry_msgs/Pose.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <thread>

void poseCallback(const geometry_msgs::Pose::ConstPtr msg) {}

TEST(AlicaTurtlesimTest, destinationTest)
{
    using namespace std::chrono_literals;

    ros::NodeHandle n;
    ros::Publisher init_pub = n.advertise<std_msgs::Empty>("init", 10);

    ros::Subscriber turtle1_sub = n.subscribe("turtle1/pose", 1, poseCallback);
    ros::Subscriber turtle2_sub = n.subscribe("turtle2/pose", 1, poseCallback);
    ros::Subscriber turtle3_sub = n.subscribe("turtle3/pose", 1, poseCallback);
    ros::Subscriber turtle4_sub = n.subscribe("turtle4/pose", 1, poseCallback);

    std::this_thread::sleep_for(1s);

    EXPECT_EQ(init_pub.getNumSubscribers(), 4U);
    std_msgs::Empty msg;
    for (int i = 0; i < 10; ++i) {
        init_pub.publish(msg);
        ros::spinOnce();
    }
    EXPECT_EQ(init_pub.getNumSubscribers(), 4U);
    std::this_thread::sleep_for(10s);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "turtlesim_tests");
    bool ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
