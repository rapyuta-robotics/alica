#pragma once

#include "ITrigger.hpp"

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>
#include <vector>

namespace essentials
{

template <class NotificationClass>
using t_notificationcallback = void (NotificationClass::*)();

/**
 * The NotifyTimer allows to have a callback-member function be called in a fixed
 * interval. The calling is delayed after each start by the given delay.
 * @tparam NotificationClass
 */
template <class NotificationClass>
class NotifyTimer : public ITrigger
{
public:
    NotifyTimer(std::chrono::milliseconds msInterval, std::chrono::milliseconds msDelayedStart, t_notificationcallback<NotificationClass> callback,
            NotificationClass* obj);
    NotifyTimer(std::chrono::milliseconds msInterval, std::chrono::milliseconds msDelayedStart, bool notifyAllThreads);
    ~NotifyTimer() override;
    /**
     * Sets the timer to run.
     * @return True, if the timer will run after this call. False, otherwise.
     */
    bool start();
    /**
     * Sets the timer to stop.
     * @return True, if the timer will anyways run after this call. False, otherwise.
     */
    bool stop();
    /**
     * Allows to get the running state of the notify timer.
     * @return True, if the timer is set to run. False, otherwise.
     */
    bool isStarted();
    std::chrono::milliseconds getDelayedStart() const;
    std::chrono::milliseconds getInterval() const;

private:
    void run(bool notifyAllThreads) override; /** < The method executed by the notify timer thread */
    std::mutex _cv_mtx;
    std::condition_variable _cv;
    t_notificationcallback<NotificationClass> _callback;
    NotificationClass* _obj;
    bool _running;                             /** < Is always true except when the notify timer is shutting down. */
    bool _started;                             /** < True, if the NotifyTimer is active. False, otherwise. */
    std::chrono::milliseconds _msInterval;     /** < The milliseconds between two calls to the callback. */
    std::chrono::milliseconds _msDelayedStart; /** < The milliseconds between (re)starting the NotifyTimer and the callback call. */
    std::unique_ptr<std::thread> _runThread;
};

template <class NotificationClass>
NotifyTimer<NotificationClass>::NotifyTimer(std::chrono::milliseconds msInterval, std::chrono::milliseconds msDelayedStart,
        t_notificationcallback<NotificationClass> callback, NotificationClass* obj)
        : _msInterval(msInterval)
        , _msDelayedStart(msDelayedStart)
        , _running(true)
        , _started(false)
        , _callback(callback)
        , _obj(obj)
{
    /**
     * We initialise the thread in the body of the constructor
     * because this guarantees to have all control variables
     * for this thread initialised properly before it starts.
     */
    _runThread = std::make_unique<std::thread>(&NotifyTimer::run, this, false);
}

template <class NotificationClass>
NotifyTimer<NotificationClass>::NotifyTimer(std::chrono::milliseconds msInterval, std::chrono::milliseconds msDelayedStart, bool notifyAllThreads)
        : _msInterval(msInterval)
        , _msDelayedStart(msDelayedStart)
        , _running(true)
        , _started(false)
        , _obj(nullptr)
{
    _runThread = std::make_unique<std::thread>(&NotifyTimer::run, this, notifyAllThreads);
}

template <class NotificationClass>
void NotifyTimer<NotificationClass>::run(bool notifyAllThreads)
{
    while (_running) {
        {
            std::unique_lock<std::mutex> lck(_cv_mtx);
            _cv.wait(lck, [&] { return !_running || _started; });
        }

        if (!_running) // for destroying the timer
            return;

        if (_msDelayedStart.count() > 0) {
            std::this_thread::sleep_for(_msDelayedStart);
        }

        while (isStarted()) {
            std::chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
            if (_obj != nullptr) {
                (_obj->*_callback)();
            } else {
                notifyEveryCV(notifyAllThreads);
            }
            auto dura = std::chrono::high_resolution_clock::now() - start;
            std::this_thread::sleep_for(_msInterval - dura);
//            std::cout << "[NotifyTimer] Finished one iteration." << std::endl;
        }
    }
}

template <class NotificationClass>
NotifyTimer<NotificationClass>::~NotifyTimer()
{
//    std::cout << "[NotifyTimer] DESTRUCT: _started: " << std::boolalpha << _started << " _running: " << _running << std::endl;
    {
        std::lock_guard<std::mutex> lockGuard(_cv_mtx);
//        std::cout << "[NotifyTimer] DESTRUCT!" << std::endl;
        _started = false;
        _running = false;
    }
    _cv.notify_all();
    _runThread->join();
}

template <class NotificationClass>
bool NotifyTimer<NotificationClass>::start()
{
//    std::cout << "[NotifyTimer] START: _started: " << std::boolalpha << _started << " _running: " << _running << std::endl;
    std::lock_guard<std::mutex> lockGuard(_cv_mtx);
    if (_running && !_started) {
//        std::cout << "[NotifyTimer] START called!" << std::endl;
        _started = true;
    }
    _cv.notify_all();
    return _running && _started;
}

template <class NotificationClass>
bool NotifyTimer<NotificationClass>::stop()
{
//    std::cout << "[NotifyTimer] STOP: _started: " << std::boolalpha << _started << " _running: " << _running << std::endl;
    std::lock_guard<std::mutex> lockGuard(_cv_mtx);
    if (_running && _started) {
//        std::cout << "[NotifyTimer] STOP called!" << std::endl;
        _started = false;
    }

    return _running && _started;
}

template <class NotificationClass>
bool NotifyTimer<NotificationClass>::isStarted()
{
    std::lock_guard<std::mutex> lockGuard(_cv_mtx); // achieves adhearing to memory barries
//    std::cout << "[NotifyTimer] IS-STARTED: _started: " << std::boolalpha << _started << " _running: " << _running << std::endl;
    return _started;
}

template <class NotificationClass>
std::chrono::milliseconds NotifyTimer<NotificationClass>::getDelayedStart() const
{
    return _msDelayedStart;
}

template <class NotificationClass>
std::chrono::milliseconds NotifyTimer<NotificationClass>::getInterval() const
{
    return _msInterval;
}

} /* namespace essentials */