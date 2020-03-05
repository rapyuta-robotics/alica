#include <gtest/gtest.h>
#include <string>
#include <thread>

#include <essentials/Timer.h>

class EventTest : public ::testing::Test
{
  public:
    int callbackInt = 0;
    std::condition_variable* cv;
    std::mutex cv_mtx;

    void callback()
    {
        callbackInt++;
        this->cv->notify_one();
        std::cout << "ZÄHLE HOCH " << callbackInt << std::endl;
    }
};

TEST_F(EventTest, timerEvent)
{
    this->cv = new std::condition_variable();
    std::unique_lock<std::mutex> lck(cv_mtx);

    essentials::Timer timerEvent(1000, 1000);
    timerEvent.registerCV(this->cv);
    timerEvent.start();

    cv->wait_for(lck, std::chrono::seconds(5), [this] {
        this->callback();
        std::cout << "callbackInt is " << callbackInt << std::endl;
        return callbackInt == 3;
    });

    timerEvent.stop();

    EXPECT_EQ(3, callbackInt) << "WRONG value of times!" << std::endl;
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
