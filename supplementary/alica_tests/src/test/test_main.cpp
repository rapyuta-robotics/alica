#include <gtest/gtest.h>

int main(int argc, char** argv)
{
    // std::signal(SIGSEGV, signalHandler);
    // std::signal(SIGABRT, signalHandler);
    testing::InitGoogleTest(&argc, argv);
    bool ret = RUN_ALL_TESTS();
    return ret;
}
