#include "supplementary/Timer.h"

namespace supplementary
{

	Timer::Timer(long msInterval, long msDelayedStart)
	{
		this->started = true;
		this->running = false;
		this->triggered = false;
		this->msInterval = std::chrono::milliseconds(msInterval);
		this->msDelayedStart = std::chrono::milliseconds(msDelayedStart);
		this->runThread = new std::thread(&Timer::run, this, false);
	}

	Timer::~Timer()
	{
		this->running = true;
		this->started = false;
		cv.notify_one();
		this->runThread->join();
		delete this->runThread;
	}

	void Timer::run(bool notifyAll)
	{
		if (msDelayedStart.count() > 0)
		{
			std::this_thread::sleep_for(msDelayedStart);
		}

		std::unique_lock<std::mutex> lck(cv_mtx);

		while (this->started)
		{
			this->cv.wait(lck, [&]
			{return !this->started || (this->running && this->registeredCVs.size() > 0);});

			if (!this->started) // for destroying the timer
				return;

			std::chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
			this->notifyAll(notifyAll);
			auto dura = std::chrono::high_resolution_clock::now() - start;
//			cout << "TimerEvent: Duration is " << chrono::duration_cast<chrono::nanoseconds>(dura).count()
//					<< " nanoseconds" << endl;
			std::this_thread::sleep_for(msInterval - dura);
		}
	}

	bool Timer::start()
	{
		if (this->started && !this->running)
		{
			this->running = true;
			this->cv.notify_one();
		}
		return this->started && this->running;
	}

	bool Timer::stop()
	{
		if (this->started && this->running)
		{
			this->running = false;
		}
		return this->started && this->running;
	}

	bool Timer::isRunning()
	{
		return this->running;
	}

	bool Timer::isStarted()
	{
		return this->started;
	}

	void Timer::setDelayedStart(long msDelayedStart)
	{
		this->msDelayedStart = std::chrono::milliseconds(msDelayedStart);
	}

	const long Timer::getDelayedStart() const
	{
		return msDelayedStart.count();
	}

	void Timer::setInterval(long msInterval)
	{
		this->msInterval = std::chrono::milliseconds(msInterval);
	}

	const long Timer::getInterval() const
	{
		return msInterval.count();
	}


} /* namespace supplementary */

