// Bring in my package's API, which is what I'm testing
#include "engine/AlicaEngine.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(SimpleTestSuite, testCase1)
{
EXPECT_TRUE(1==1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
