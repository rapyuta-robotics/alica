#include "SystemConfig.h"
#include <gtest/gtest.h>

using namespace supplementary;

// Declare a test
TEST(SystemConfigBasics, readValues)
{
	// determine the path to the test config
	string path = FileSystem::getSelfPath();
	int place = path.rfind("devel");
	path = path.substr(0, place);
	path = path + "src/supplementary/system_config/test";

	// bring up the SystemConfig with the corresponding path
	SystemConfig* sc = SystemConfig::getInstance();
	sc->setRootPath(path);
	sc->setConfigPath(path + "/etc");

	// read int
	int intTestValue = (*sc)["Test"]->get<int>("intTestValue", NULL);
	EXPECT_EQ(221, intTestValue);

	// read double
	double doubleTestValue = (*sc)["Test"]->get<double>("doubleTestValue", NULL);
	EXPECT_DOUBLE_EQ(0.66234023823, doubleTestValue);

	// read float
	float floatTestValue = (*sc)["Test"]->get<float>("floatTestValue", NULL);
	EXPECT_FLOAT_EQ(1.14f, floatTestValue);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

