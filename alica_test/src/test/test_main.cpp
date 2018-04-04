#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "AlicaEngine");
	bool ret = RUN_ALL_TESTS();
	ros::shutdown();
	return ret;
}
