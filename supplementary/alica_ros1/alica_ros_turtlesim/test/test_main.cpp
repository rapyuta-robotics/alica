#include <std_msgs/Empty.h>
#include <turtlesim/Pose.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <thread>

using namespace std::chrono_literals;

// class to handle callbacks
class TurtlePosition
{
public:
    void poseCallback(const turtlesim::Pose::ConstPtr msg)
    {
        x = msg->x;
        y = msg->y;
    }

    double getX() { return x; }
    double getY() { return y; }

private:
    double x;
    double y;
};

TEST(AlicaTurtlesimTest, destinationTest)
{
    ros::NodeHandle n;
    ros::Publisher init_pub = n.advertise<std_msgs::Empty>("init", 10);

    // wait for subscribers to latch
    std::this_thread::sleep_for(1s);
    EXPECT_EQ(init_pub.getNumSubscribers(), 4U);

    // send init message to turtles
    std_msgs::Empty msg;
    for (int i = 0; i < 10; ++i) {
        init_pub.publish(msg);
        ros::spinOnce();
    }

    // wait for turtles to move
    std::this_thread::sleep_for(10s);

    TurtlePosition turtle1;
    TurtlePosition turtle2;
    TurtlePosition turtle3;
    TurtlePosition turtle4;

    ros::Subscriber turtle1_sub = n.subscribe("turtle1/pose", 10, &TurtlePosition::poseCallback, &turtle1);
    ros::Subscriber turtle2_sub = n.subscribe("turtle2/pose", 10, &TurtlePosition::poseCallback, &turtle2);
    ros::Subscriber turtle3_sub = n.subscribe("turtle3/pose", 10, &TurtlePosition::poseCallback, &turtle3);
    ros::Subscriber turtle4_sub = n.subscribe("turtle4/pose", 10, &TurtlePosition::poseCallback, &turtle4);

    // wait for subscribers to latch
    std::this_thread::sleep_for(1s);

    for (int i = 0; i < 100; ++i) {
        ros::spinOnce();
    }

    // collect distances between turtles and sort in ascending order
    std::vector<double> distances;
    distances.emplace_back(sqrt(pow((turtle1.getX() - turtle2.getX()), 2) + pow((turtle1.getY() - turtle2.getY()), 2)));
    distances.emplace_back(sqrt(pow((turtle1.getX() - turtle3.getX()), 2) + pow((turtle1.getY() - turtle3.getY()), 2)));
    distances.emplace_back(sqrt(pow((turtle1.getX() - turtle4.getX()), 2) + pow((turtle1.getY() - turtle4.getY()), 2)));
    distances.emplace_back(sqrt(pow((turtle2.getX() - turtle3.getX()), 2) + pow((turtle2.getY() - turtle3.getY()), 2)));
    distances.emplace_back(sqrt(pow((turtle2.getX() - turtle4.getX()), 2) + pow((turtle2.getY() - turtle4.getY()), 2)));
    distances.emplace_back(sqrt(pow((turtle3.getX() - turtle4.getX()), 2) + pow((turtle3.getY() - turtle4.getY()), 2)));
    std::sort(distances.begin(), distances.end());

    // should have 3 distances of ~2.6 (outer turtles to center turtle)
    EXPECT_NEAR(distances[0], 2.6, 0.2);
    EXPECT_NEAR(distances[1], 2.6, 0.2);
    EXPECT_NEAR(distances[2], 2.6, 0.2);

    // should have 3 distances of ~4.5 (distances between outer turtles)
    EXPECT_NEAR(distances[3], 4.5, 0.4);
    EXPECT_NEAR(distances[4], 4.5, 0.4);
    EXPECT_NEAR(distances[5], 4.5, 0.4);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "turtlesim_tests");
    bool ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
