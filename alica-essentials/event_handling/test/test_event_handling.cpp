/*
 * testeventhandling.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Paul Panin
 */

#include <gtest/gtest.h>
#include <thread>
#include "../include/AutoResetEvent.h"
#include "../include/TimerCallback.h"
#include "../include/Timer.h"
#include <string>
using namespace supplementary;

AutoResetEvent event;
int callbackInt = 0;

class EventTest : public ::testing::Test
{
public:
	static void otherThread(std::string which)
	{
		event.waitOne();
		std::cout << which << std::endl;
	}
	void callback()
	{
		callbackInt++;
		std::cout << "ZÄHLE HOCH " << callbackInt << std::endl;
	}

	static void callbackTwo()
	{
		callbackInt++;
		std::cout << "ZÄHLE HOCH " << callbackInt << std::endl;
	}
};

TEST_F(EventTest, autoResetEvent)
{
	std::thread first(std::bind(otherThread, "first!"));
	std::cout << "nach FIRST" << std::endl;

	sleep(1);
	while (!event.isThreadWaiting())
	{
		sleep(0.05);
		std::cout << "ICH WARTE1 " << std::endl;
	}
	std::cout << "NACHDEM WARTEN1 " << std::endl;
	event.set();
	std::cout << "NACHDEM SET1 " << std::endl;

	std::thread second(std::bind(otherThread, "second!"));
	sleep(1);
	std::cout << "nach SECOND" << std::endl;
	while (!event.isThreadWaiting())
	{
		sleep(0.05);
		std::cout << "ICH WARTE2 " << std::endl;
	}
	std::cout << "NACHDEM WARTEN2 " << std::endl;
	event.set();
	std::cout << "NACHDEM SET2 " << std::endl;
	event.set();

	first.join();
	second.join();
//
	std::chrono::milliseconds dura(4000);
	std::chrono::milliseconds duraThread(1000);
	std::chrono::milliseconds delayinM(1000);

	TimerCallback<EventTest> h(this, &EventTest::callback, duraThread, true, delayinM);
	h.start();
	std::this_thread::sleep_for(dura);
	h.stop();

	EXPECT_EQ(3, callbackInt) << "WRONG value of times!" << endl;

//	Timer* t = new Timer(&EventTest::callbackTwo, duraThread, true, delayinM);
//	callbackInt = 0;
//	t->start();
//	std::this_thread::sleep_for(dura);
//	t->stop();
//
//	EXPECT_EQ(3, callbackInt) << "WRONG value of times!" << endl;
}
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
