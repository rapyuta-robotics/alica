// Bring in my package's API, which is what I'm testing
#include "SystemConfig.h"
// Bring in gtest
#include <gtest/gtest.h>
#include <stdio.h>

using namespace supplementary;
std::string get_selfpath() {
	char buff[1024];
	ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff)-1);
	if (len != -1) {
		buff[len] = '\0';
		return std::string(buff);
	} else {
		/* handle error condition */
	}
}
// Declare a test
TEST(SystemConfigBasics, ownRobotID)
{
	SystemConfigPtr sc = SystemConfig::getInstance();
	int ownID = sc->GetOwnRobotID();
	EXPECT_TRUE(ownID>0);
}

TEST(SystemConfigBasics, ownRobotID2)
{
	SystemConfigPtr sc = SystemConfig::getInstance();

	int ownID = (*sc)["Globals"]->get<int>("Globals.Team", SystemConfig::getHostname().c_str(), "ID", NULL);
	EXPECT_TRUE(ownID>0);
}

TEST(SystemConfigBasics, testOwnEtc)
{
	std::string path = get_selfpath();
	int place = path.rfind("devel");
	path = path.substr(0, place);
	path = path + "src/supplementary/system_config/test";

	SystemConfigPtr sc = SystemConfig::getInstance();
	sc->setRootPath(path);
	sc->setConfigPath(path + "/etc");
	Configuration *conf = (*sc)["Test"];

	std::string testValue1 = conf->get<std::string>("TestValue1", NULL);
	EXPECT_STREQ(testValue1.c_str(),"TestValue1");

	int testValue2 = conf->get<int>("TestValue2", NULL);
	EXPECT_EQ(testValue2,2);

	float testValue3 = conf->get<float>("TestValue3", NULL);
	EXPECT_FLOAT_EQ(testValue3,0.66412);

	bool testValue4 = conf->get<bool>("TestValue4", NULL);
	EXPECT_EQ(testValue4, false);

	bool testValue5 = conf->get<bool>("TestValue5", NULL);
	EXPECT_EQ(testValue5, false);

	bool testValue6 = conf->get<bool>("TestValue6", NULL);
	EXPECT_EQ(testValue6, true);

	bool testValue7 = conf->get<bool>("TestValue7", NULL);
	EXPECT_EQ(testValue7, true);


}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

