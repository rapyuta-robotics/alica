// Bring in my package's API, which is what I'm testing
#include "SystemConfig.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(SystemConfigBasics, ownRobotID)
{
  SystemConfigPtr sc = SystemConfig::getInstance();
  int ownID = sc->GetOwnRobotID();
  EXPECT_TRUE(ownID>0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
