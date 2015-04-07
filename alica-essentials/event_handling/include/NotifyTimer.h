/*
 * NotifyTimer.h
 *
 *  Created on: Nov 24, 2014
 *      Author: Philipp
 */

#ifndef NOTIFYTIMER_H_
#define NOTIFYTIMER_H_

#include <vector>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <iostream>
#include "ITrigger.h"

using namespace std;

namespace supplementary
{

	template<class NotificationClass>
	using t_notificationcallback = void (NotificationClass::*)();

	template<class NotificationClass>
	class NotifyTimer : public virtual ITrigger
	{
	public:
		NotifyTimer(long msInterval, t_notificationcallback<NotificationClass> callback, NotificationClass* obj);
		~NotifyTimer();
		bool start();
		bool stop();
		bool isRunning();
		bool isStarted();
		void setInterval(long msInterval);
		const long getInterval() const;
		void run(bool notifyAll = false);
		void registerCV(condition_variable* condVar);


	private:
		thread* runThread;
		chrono::milliseconds msInterval; /** < The time between two fired events */
		bool running, started;
		t_notificationcallback<NotificationClass> callback;
		NotificationClass* obj;

	};

	template<class NotificationClass>
	NotifyTimer<NotificationClass>::NotifyTimer(long msInterval, t_notificationcallback<NotificationClass> callback,
								NotificationClass* obj)
	{
		this->started = true;
		this->running = false;
		this->msInterval = chrono::milliseconds(msInterval);
		this->runThread = new thread(&NotifyTimer::run, this, false);
		this->callback = callback;
		this->obj = obj;
	}

	template<class NotificationClass>
	void NotifyTimer<NotificationClass>::run(bool notifyAll)
	{
		while (this->started)
		{
			if (!this->started) // for destroying the NotifyTimer
				return;

			chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
			if (this->running)
			{
				(obj->*callback)();
			}
			//			XXX: call function here
			auto dura = std::chrono::high_resolution_clock::now() - start;
//			cout << "NotifyTimerEvent: Duration is " << chrono::duration_cast<chrono::nanoseconds>(dura).count()
//					<< " nanoseconds" << endl;
			this_thread::sleep_for(msInterval - dura);
		}
	}

	template<class NotificationClass>
	NotifyTimer<NotificationClass>::~NotifyTimer()
	{
		this->running = false;
		this->started = false;
		this->runThread->join();
		delete this->runThread;
	}

	template<class NotificationClass>
	bool NotifyTimer<NotificationClass>::start()
	{
		if (this->started && !this->running)
		{
			this->running = true;
		}
		return this->started && this->running;
	}

	template<class NotificationClass>
	bool NotifyTimer<NotificationClass>::stop()
	{
		if (this->started && this->running)
		{
			this->running = false;
		}
		return this->started && this->running;
	}

	template<class NotificationClass>
	bool NotifyTimer<NotificationClass>::isRunning()
	{
		return this->running;
	}

	template<class NotificationClass>
	bool NotifyTimer<NotificationClass>::isStarted()
	{
		return this->started;
	}

	template<class NotificationClass>
	void NotifyTimer<NotificationClass>::setInterval(long msInterval)
	{
		this->msInterval = chrono::milliseconds(msInterval);
	}

	template<class NotificationClass>
	const long NotifyTimer<NotificationClass>::getInterval() const
	{
		return msInterval.count();
	}

} /* namespace supplementary */

template<class NotificationClass>
inline void supplementary::NotifyTimer<NotificationClass>::registerCV(condition_variable* condVar)
{
}

#endif /* NOTIFYTIMER_H_ */
