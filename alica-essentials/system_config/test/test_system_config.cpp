#include "SystemConfig.h"

#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include <typeinfo>

// Declare a test
TEST(SystemConfigBasics, readValues)
{
    // bring up the SystemConfig with the corresponding path
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    sc.setRootPath(".");
    sc.setConfigPath("./etc");

    // read int
    unsigned short uShortTestValue = sc["Test"]->get<unsigned short>("uShortTestValue", NULL);
    EXPECT_EQ(3, uShortTestValue);

    // read int
    int intTestValue = sc["Test"]->get<int>("intTestValue", NULL);
    EXPECT_EQ(221, intTestValue);

    // read double
    double doubleTestValue = sc["Test"]->get<double>("doubleTestValue", NULL);
    EXPECT_DOUBLE_EQ(0.66234023823, doubleTestValue);

    // read float
    float floatTestValue = sc["Test"]->get<float>("floatTestValue", NULL);
    EXPECT_FLOAT_EQ(1.14f, floatTestValue);

    // read spaceTestValue
    int spaceTestValue = sc["Test"]->get<int>("spaceTestValue", NULL);
    EXPECT_FLOAT_EQ(6, spaceTestValue);

    // read list of strings
    std::vector<std::string> stringListTestValue = sc["Test"]->getList<std::string>("stringListTestValue", NULL);
    std::vector<std::string> referenceStringList{"asdf", "bla", "blub", "hust hust", "möp"};
    for (int i = 0; i < stringListTestValue.size(); i++) {
        EXPECT_STREQ(stringListTestValue[i].c_str(), referenceStringList[i].c_str());
    }

    // read TestSectionValue1
    std::string testSectionValue1 = sc["Test"]->get<std::string>("TestSection.TestSectionValue1", NULL);
    EXPECT_STREQ("TestSectionValue1", testSectionValue1.c_str());

    // read TestSectionValue1 with two conf-path parameters
    std::string testSectionValue11 = sc["Test"]->get<std::string>("TestSection", "TestSectionValue1", NULL);
    EXPECT_STREQ("TestSectionValue1", testSectionValue11.c_str());

    // read TestSectionValue2
    float testSectionValue2 = sc["Test"]->get<float>("TestSection.TestSectionValue2", NULL);
    EXPECT_FLOAT_EQ(0.66412f, testSectionValue2);
}
// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
