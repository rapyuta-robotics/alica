#include <std_msgs/Empty.h>
#include <turtlesim/Pose.h>

#include <autodiff/AutoDiff.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <chrono>
#include <thread>

class TurtlePosition;

#define EXPECT_POSE_NEAR(curr_pose, desired_pose, delta)                                                                                                       \
    do {                                                                                                                                                       \
        EXPECT_NEAR(curr_pose.x, desired_pose.x, delta.x);                                                                                                     \
        EXPECT_NEAR(curr_pose.y, desired_pose.y, delta.y);                                                                                                     \
    } while (0)

#define RUN_UNTIL_ROBOT_STOP(turtle, ms)                                                                                                                       \
    do {                                                                                                                                                       \
        using namespace std::chrono;                                                                                                                           \
        auto start = system_clock::now();                                                                                                                      \
        auto past_pose = turtle.getPose();                                                                                                                     \
        do {                                                                                                                                                   \
            ros::Duration(0.5).sleep();                                                                                                                        \
            ros::spinOnce();                                                                                                                                   \
            if (turtle.isPoseEqual(past_pose, 0.01)) {                                                                                                         \
                break;                                                                                                                                         \
            }                                                                                                                                                  \
            past_pose = turtle.getPose();                                                                                                                      \
        } while (duration_cast<milliseconds>(system_clock::now() - start).count() < ms);                                                                       \
        EXPECT_TRUE(turtle.isPoseEqual(past_pose, 0.01));                                                                                                      \
    } while (0)

// class to handle callbacks
class TurtlePosition
{
public:
    TurtlePosition() = default;

    TurtlePosition(const double x, const double y, const double theta = 0)
    {
        _pose.x = x;
        _pose.y = y;
        _pose.theta = theta;
    }

    void poseCallback(const turtlesim::Pose::ConstPtr msg)
    {
        _pose.x = msg->x;
        _pose.y = msg->y;
        _pose.theta = msg->theta;
    }

    turtlesim::Pose getPose() const { return _pose; }

    bool isNear(const turtlesim::Pose& desired_pose, const turtlesim::Pose& delta) const
    {
        assert(delta.x > 0);
        assert(delta.y > 0);

        return std::fabs(_pose.x - desired_pose.x) < delta.x && std::fabs(_pose.y - desired_pose.y) < delta.y;
    }

    double distance(const TurtlePosition& turtle) { return sqrt(pow(turtle._pose.x - _pose.x, 2) + pow(turtle._pose.y - _pose.y, 2)); }

    bool isPoseEqual(const turtlesim::Pose& other_pose, double tolerance)
    {
        return std::fabs(other_pose.x - _pose.x) + std::fabs(other_pose.y - _pose.y) + std::fabs(other_pose.theta - _pose.theta) < tolerance;
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
    RUN_UNTIL_EXPECT_EQ(join_formation_turtle_1_pub.getNumSubscribers(), 1U, 10000);
    RUN_UNTIL_EXPECT_EQ(join_formation_turtle_2_pub.getNumSubscribers(), 1U, 10000);
    RUN_UNTIL_EXPECT_EQ(join_formation_turtle_3_pub.getNumSubscribers(), 1U, 10000);
    RUN_UNTIL_EXPECT_EQ(join_formation_turtle_4_pub.getNumSubscribers(), 1U, 10000);

    turtlesim::Pose pose_tolerance;
    pose_tolerance.x = 0.2;
    pose_tolerance.y = 0.2;

    // Turtle 1 test
    TurtlePosition turtle1;
    ros::Subscriber turtle1_sub = nh.subscribe("turtle1/pose", 10, &TurtlePosition::poseCallback, &turtle1);

    turtlesim::Pose desired_pose_turtle_1;
    desired_pose_turtle_1.x = 4.97;
    desired_pose_turtle_1.y = 5.033;

    // send init message to turtles
    std_msgs::Empty msg;
    join_formation_turtle_1_pub.publish(msg);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    RUN_UNTIL_ROBOT_STOP(turtle1, 10000);
    EXPECT_POSE_NEAR(turtle1.getPose(), desired_pose_turtle_1, pose_tolerance);

    // Turtle 2 test
    TurtlePosition turtle2;
    ros::Subscriber turtle2_sub = nh.subscribe("turtle2/pose", 10, &TurtlePosition::poseCallback, &turtle2);

    auto turtle_center = TurtlePosition(5, 5);
    double desired_distance_to_the_center = 2.5;
    double distance_tolerance = 0.3;

    join_formation_turtle_2_pub.publish(msg);

    RUN_UNTIL_ROBOT_STOP(turtle2, 10000);
    EXPECT_NEAR(turtle2.distance(turtle_center), desired_distance_to_the_center, distance_tolerance);

    // Turtle 3
    TurtlePosition turtle3;
    ros::Subscriber turtle3_sub = nh.subscribe("turtle3/pose", 10, &TurtlePosition::poseCallback, &turtle3);

    double distance_between_followers = 2.0 * desired_distance_to_the_center * sin(M_PI / (double) (2)) - distance_tolerance;

    join_formation_turtle_3_pub.publish(msg);

    RUN_UNTIL_ROBOT_STOP(turtle3, 10000);

    EXPECT_NEAR(turtle2.distance(turtle_center), desired_distance_to_the_center, distance_tolerance);
    EXPECT_NEAR(turtle3.distance(turtle_center), desired_distance_to_the_center, distance_tolerance);
    EXPECT_NEAR(turtle3.distance(turtle2), distance_between_followers, distance_tolerance);

    // Turtle 4
    TurtlePosition turtle4;
    ros::Subscriber turtle4_sub = nh.subscribe("turtle4/pose", 10, &TurtlePosition::poseCallback, &turtle4);

    distance_between_followers = 2.0 * desired_distance_to_the_center * sin(M_PI / (double) (3));

    join_formation_turtle_4_pub.publish(msg);

    RUN_UNTIL_ROBOT_STOP(turtle4, 10000);

    EXPECT_NEAR(turtle2.distance(turtle_center), desired_distance_to_the_center, distance_tolerance);
    EXPECT_NEAR(turtle3.distance(turtle_center), desired_distance_to_the_center, distance_tolerance);
    EXPECT_NEAR(turtle4.distance(turtle_center), desired_distance_to_the_center, distance_tolerance);
    EXPECT_NEAR(turtle3.distance(turtle2), distance_between_followers, distance_tolerance);
    EXPECT_NEAR(turtle4.distance(turtle2), distance_between_followers, distance_tolerance);
    EXPECT_NEAR(turtle4.distance(turtle3), distance_between_followers, distance_tolerance);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "turtlesim_tests");
    bool ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
