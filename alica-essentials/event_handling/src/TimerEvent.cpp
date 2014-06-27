/*
 * TimerEvent.cpp
 *
 *  Created on: Jun 27, 2014
 *      Author: Stephan Opfer
 */

#include "TimerEvent.h"

namespace supplementary
{

	TimerEvent::TimerEvent(long msInterval, long msDelayedStart, bool notifyAll) : notifyAll(notifyAll)
	{
		this->started = false;
		this->running = false;
		this->msInterval = chrono::milliseconds(msInterval);
		this->msDelayedStart = chrono::milliseconds(msDelayedStart);
		this->registeredCVs = vector<condition_variable*>();
		this->runThread = new thread(&TimerEvent::run, this);
	}

	void TimerEvent::registerCV(condition_variable* condVar)
	{
		this->registeredCVs.push_back(condVar);

	}

	void TimerEvent::run()
	{
		if (msDelayedStart.count() > 0)
		{
			this_thread::sleep_for(msDelayedStart);
		}

		unique_lock<mutex> lck(cv_mtx);

		while (this->started)
		{
			this->cv.wait(lck, [&]
			{	return this->running && this->registeredCVs.size() > 0;});

			chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
			for (int i = 0; i < this->registeredCVs.size(); i++)
			{
				if (this->notifyAll)
				{
					registeredCVs[i]->notify_all();
				}
				else
				{
					registeredCVs[i]->notify_one();
				}
			}
			auto dura = std::chrono::high_resolution_clock::now() - start;
			cout << "TimerEvent: Duration is " << chrono::duration_cast<chrono::nanoseconds>(dura).count() << " nanoseconds" << endl;
			this_thread::sleep_for(msInterval - dura);
		}
	}

	void TimerEvent::start()
	{
		this->started = true;
		this->running = true;
		runThread = new thread(&TimerEvent::run, this);
	}

	bool TimerEvent::restart()
	{
		if (this->started && !this->running)
		{
			this->running = true;
			this->cv.notify_one();
			return true;
		}
		else
		{
			return false;
		}
	}

	bool TimerEvent::pause()
	{
		if (this->started && this->running)
		{
			this->running = false;
			return true;
		}
		else
		{
			return false;
		}
	}

	void TimerEvent::stop()
	{
		this->running = true;
		this->started = false;
		cv.notify_one();
	}

	bool TimerEvent::isRunning()
	{
		return this->running;
	}

	bool TimerEvent::isStarted()
	{
		return this->started;
	}

	void TimerEvent::setMsDelayedStart(long msDelayedStart)
	{
		this->msDelayedStart = chrono::milliseconds(msDelayedStart);
	}

	void TimerEvent::setMsInterval(long msInterval)
	{
		this->msInterval = chrono::milliseconds(msInterval);
	}

} /* namespace supplementary */
