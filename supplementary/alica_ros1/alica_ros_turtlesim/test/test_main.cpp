#include <std_msgs/Empty.h>
#include <turtlesim/Pose.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <chrono>
#include <thread>

class TurtlePosition;

#define RUN_UNTIL_EXPECT_EQ(left, right, ms)                                                                                                                   \
    using namespace std::chrono;                                                                                                                               \
    {                                                                                                                                                          \
        auto start = system_clock::now();                                                                                                                      \
        do {                                                                                                                                                   \
            ros::spinOnce();                                                                                                                                   \
            if (left == right) {                                                                                                                               \
                EXPECT_EQ(left, right);                                                                                                                        \
                return;                                                                                                                                        \
            }                                                                                                                                                  \
            ros::Duration(0.01).sleep();                                                                                                                       \
        } while (duration_cast<milliseconds>(system_clock::now() - start).count() < ms);                                                                       \
    }                                                                                                                                                          \
    EXPECT_EQ(left, right);

#define EXPECT_POSE_NEAR(curr_pose, desired_pose, delta)                                                                                                       \
    EXPECT_NEAR(curr_pose.x, desired_pose.x, desired_delta.x);                                                                                                 \
    EXPECT_NEAR(curr_pose.y, desired_pose.y, desired_delta.y);

#define RUN_UNTIL_EXPECT_NEAR_POSE(turtle, desired_pose, desired_delta, ms)                                                                                    \
    using namespace std::chrono;                                                                                                                               \
    {                                                                                                                                                          \
        auto start = system_clock::now();                                                                                                                      \
        auto curr_pose = turtle.getPose();                                                                                                                     \
        do {                                                                                                                                                   \
            ros::spinOnce();                                                                                                                                   \
            curr_pose = turtle.getPose();                                                                                                                      \
            if (turtle.isNear(desired_pose, desired_delta)) {                                                                                                  \
                EXPECT_POSE_NEAR(curr_pose, desired_pose, desired_delta);                                                                                      \
                return;                                                                                                                                        \
            }                                                                                                                                                  \
            ros::Duration(0.01).sleep();                                                                                                                       \
        } while (duration_cast<milliseconds>(system_clock::now() - start).count() < ms);                                                                       \
    }                                                                                                                                                          \
    EXPECT_POSE_NEAR(turtle.getPose(), desired_pose, desired_delta);

using namespace std::chrono_literals;

// class to handle callbacks
class TurtlePosition
{
public:
    void poseCallback(const turtlesim::Pose::ConstPtr msg)
    {
        _pose.x = msg->x;
        _pose.y = msg->y;
        _pose.theta = msg->theta;
    }

    turtlesim::Pose getPose() const { return _pose; }

    bool isNear(const turtlesim::Pose& desired_pose, const turtlesim::Pose& desired_delta) const
    {
        assert(desired_delta.x > 0);
        assert(desired_delta.y > 0);
        assert(desired_delta.theta > 0);

        return _pose.x < desired_pose.x + desired_delta.x && _pose.x > desired_pose.x - desired_delta.x && _pose.y < desired_pose.y + desired_delta.y &&
               _pose.y > desired_pose.y - desired_delta.y && _pose.theta < desired_pose.theta + desired_delta.theta &&
               _pose.theta > desired_pose.theta - desired_delta.theta;
    }

private:
    turtlesim::Pose _pose;
};

TEST(AlicaTurtlesimTest, destinationTest)
{
    ros::NodeHandle nh;
    ros::Publisher join_formation_turtle_1_pub = nh.advertise<std_msgs::Empty>("/turtle1/join_formation", 10);
    ros::Publisher join_formation_turtle_2_pub = nh.advertise<std_msgs::Empty>("/turtle2/join_formation", 10);
    ros::Publisher join_formation_turtle_3_pub = nh.advertise<std_msgs::Empty>("/turtle3/join_formation", 10);
    ros::Publisher join_formation_turtle_4_pub = nh.advertise<std_msgs::Empty>("/turtle4/join_formation", 10);

    // wait for subscribers to latch

    RUN_UNTIL_EXPECT_EQ(join_formation_turtle_1_pub.getNumSubscribers(), 1U, 1000);
    RUN_UNTIL_EXPECT_EQ(join_formation_turtle_2_pub.getNumSubscribers(), 1U, 1000);
    RUN_UNTIL_EXPECT_EQ(join_formation_turtle_3_pub.getNumSubscribers(), 1U, 1000);
    RUN_UNTIL_EXPECT_EQ(join_formation_turtle_4_pub.getNumSubscribers(), 1U, 1000);

    turtlesim::Pose desired_delta;
    desired_delta.x = 0.1;
    desired_delta.y = 0.1;
    desired_delta.theta = 0.1;

    TurtlePosition turtle1;
    ros::Subscriber turtle1_sub = nh.subscribe("turtle1/pose", 10, &TurtlePosition::poseCallback, &turtle1);

    turtlesim::Pose desired_pose_turtle_1;
    desired_pose_turtle_1.x = 4.96;
    desired_pose_turtle_1.y = 5.033;

    // send init message to turtles
    std_msgs::Empty msg;
    join_formation_turtle_1_pub.publish(msg);

    RUN_UNTIL_EXPECT_NEAR_POSE(turtle1, desired_pose_turtle_1, desired_delta, 10000);

    // join_formation_turtle_2_pub.publish(msg);
    // join_formation_turtle_3_pub.publish(msg);
    // join_formation_turtle_4_pub.publish(msg);

    // TurtlePosition turtle2;
    // TurtlePosition turtle3;
    // TurtlePosition turtle4;

    // ros::Subscriber turtle1_sub = nh.subscribe("turtle1/pose", 10, &TurtlePosition::poseCallback, &turtle1);
    // ros::Subscriber turtle2_sub = nh.subscribe("turtle2/pose", 10, &TurtlePosition::poseCallback, &turtle2);
    // ros::Subscriber turtle3_sub = nh.subscribe("turtle3/pose", 10, &TurtlePosition::poseCallback, &turtle3);
    // ros::Subscriber turtle4_sub = nh.subscribe("turtle4/pose", 10, &TurtlePosition::poseCallback, &turtle4);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "turtlesim_tests");
    bool ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
