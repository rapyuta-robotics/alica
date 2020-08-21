#include "essentials/Timer.h"

#include <thread>

namespace essentials
{

Timer::Timer(std::chrono::milliseconds msInterval, std::chrono::milliseconds msDelayedStart, bool notifyAllThreads)
        : _msInterval(msInterval)
        , _msDelayedStart(msDelayedStart)
        , _running(true)
        , _started(false)
        , _justStartedAgain(false)
{
    _runThread = new std::thread(&Timer::run, this, notifyAllThreads);
}

Timer::~Timer()
{
    _started = true;
    _running = false;
    _cv.notify_one();
    _runThread->join();
    delete _runThread;
}

void Timer::run(bool notifyAllThreads)
{
    while (_running) {
        {
            std::unique_lock<std::mutex> lck(_cv_mtx);
            _cv.wait(lck, [&] { return !_running || _started && isAnyCVRegistered(); });
        }

        if (!_running) // for destroying the timer
            return;

        if (_justStartedAgain) {
            if (_msDelayedStart.count() > 0) {
                std::this_thread::sleep_for(_msDelayedStart);
            }
            _justStartedAgain = false;
        }

        std::chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
        notifyEveryCV(notifyAllThreads);
        auto dura = std::chrono::high_resolution_clock::now() - start;
        std::this_thread::sleep_for(_msInterval - dura);
    }
}

bool Timer::start()
{
    if (_running && !_started) {
        _started = true;
        _justStartedAgain = true;
        _cv.notify_one();
    }
    return _running && _started;
}

bool Timer::stop()
{
    if (_running && _started) {
        _started = false;
    }
    return _running && _started;
}

bool Timer::isStarted() const
{
    return _started;
}

std::chrono::milliseconds Timer::getDelayedStart() const
{
    return _msDelayedStart;
}

std::chrono::milliseconds Timer::getInterval() const
{
    return _msInterval;
}

} /* namespace essentials */
