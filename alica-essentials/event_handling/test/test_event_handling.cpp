/*
 * test_event_handling.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Paul Panin
 */

#include <gtest/gtest.h>
#include <thread>
#include <string>

#include "AutoResetEvent.h"
#include "Timer.h"

using namespace supplementary;

// TODO cleanup this global variable mess :)
AutoResetEvent event;
int callbackInt2;
class EventTest : public ::testing::Test
{
public:
	int callbackInt = 0;
	condition_variable* cv;
	mutex cv_mtx;

	static void otherThread(std::string which)
	{
		event.waitOne();
		std::cout << which << std::endl;
	}
	void callback()
	{
		callbackInt++;
		this->cv->notify_one();
		std::cout << "ZÄHLE HOCH " << callbackInt << std::endl;
	}

	static void callbackTwo()
	{
		callbackInt2++;
		std::cout << "ZÄHLE HOCH " << callbackInt2 << std::endl;
	}
};

TEST_F(EventTest, timerEvent)
{
	this->cv = new condition_variable();
	unique_lock<mutex> lck(cv_mtx);

	Timer timerEvent(1000, 1000);
	timerEvent.registerCV(this->cv);
	timerEvent.start();

	cv->wait_for(lck, chrono::seconds(5), [&]
	{
		this->callback();
		cout << "callbackInt is " << callbackInt << endl;
		return callbackInt == 3;
	});

	timerEvent.stop();

	EXPECT_EQ(3, callbackInt) << "WRONG value of times!" << endl;
}

// TODO cleanup this test...
TEST_F(EventTest, autoResetEvent)
{
	std::thread first(std::bind(otherThread, "first!"));
	//std::cout << "nach FIRST" << std::endl;

	sleep(1);
	while (!event.isThreadWaiting())
	{
		sleep(0.05);
	//	std::cout << "ICH WARTE1 " << std::endl;
	}
	//std::cout << "NACHDEM WARTEN1 " << std::endl;
	event.set();
	//std::cout << "NACHDEM SET1 " << std::endl;

	std::thread second(std::bind(otherThread, "second!"));
	sleep(1);
	//std::cout << "nach SECOND" << std::endl;
	while (!event.isThreadWaiting())
	{
		sleep(0.05);
		//std::cout << "ICH WARTE2 " << std::endl;
	}
	//std::cout << "NACHDEM WARTEN2 " << std::endl;
	event.set();
	//std::cout << "NACHDEM SET2 " << std::endl;
	event.set();

	first.join();
	second.join();
}
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
