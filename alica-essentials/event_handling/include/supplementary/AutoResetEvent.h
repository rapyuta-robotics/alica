#pragma once

#include <thread>
#include <condition_variable>
#include <mutex>
#include <iostream>

namespace supplementary
{

	class AutoResetEvent
	{
	public:
		explicit AutoResetEvent(bool initial = false);
		~AutoResetEvent();
		void set();
		void reset();
		void waitOne();
		bool isThreadWaiting();
		int waitingThread;

	private:
		AutoResetEvent(const AutoResetEvent&);
		AutoResetEvent& operator=(const AutoResetEvent&); // non-copyable
		bool flag;

		std::mutex protect;
		std::mutex protectWatingThread;
		std::condition_variable signal;
	};

} /* namespace supplementary */
