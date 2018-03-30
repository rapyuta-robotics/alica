#include "SystemConfig.h"
#include <iostream>
#include <typeinfo>
#include <gtest/gtest.h>

using namespace supplementary;
// Declare a test
TEST(SystemConfigBasics, readValues) {
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
    unsigned short uShortTestValue = (*sc)["Test"]->get<unsigned short>("uShortTestValue", NULL);
    EXPECT_EQ(3, uShortTestValue);

    // read int
    int intTestValue = (*sc)["Test"]->get<int>("intTestValue", NULL);
    EXPECT_EQ(221, intTestValue);

    // read double
    double doubleTestValue = (*sc)["Test"]->get<double>("doubleTestValue", NULL);
    EXPECT_DOUBLE_EQ(0.66234023823, doubleTestValue);

    // read float
    float floatTestValue = (*sc)["Test"]->get<float>("floatTestValue", NULL);
    EXPECT_FLOAT_EQ(1.14f, floatTestValue);

    // read spaceTestValue
    int spaceTestValue = (*sc)["Test"]->get<int>("spaceTestValue", NULL);
    EXPECT_FLOAT_EQ(6, spaceTestValue);

    // read TestSectionValue1
    string testSectionValue1 = (*sc)["Test"]->get<string>("TestSection.TestSectionValue1", NULL);
    EXPECT_STREQ("TestSectionValue1", testSectionValue1.c_str());

    // read TestSectionValue2
    float testSectionValue2 = (*sc)["Test"]->get<float>("TestSection.TestSectionValue2", NULL);
    EXPECT_FLOAT_EQ(0.66412f, testSectionValue2);
}
// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
