#include "essentials/Worker.h"
#include "essentials/Timer.h"

#include <string>
#include <thread>

namespace essentials
{

Worker::Worker(std::string name, std::chrono::milliseconds msInterval, std::chrono::milliseconds msDelayedStart)
        : _name(name)
        , _started(true)
        , _runCV()
{
    _timer = new essentials::Timer(msInterval, msDelayedStart, true);
    _timer->registerCV(&_runCV);
    _runThread = new std::thread(&Worker::runInternal, this);
}

Worker::~Worker()
{
    _started = false;
    _runCV.notify_all();
    _timer->start();
    _runThread->join();
    delete _runThread;
    delete _timer;
}

bool Worker::stop()
{
    return _timer->stop();
}

bool Worker::start()
{
    return _timer->start();
}

bool Worker::isRunning() const
{
    return _timer->isStarted();
}

void Worker::runInternal()
{
    while (_started) {
        {
            std::unique_lock<std::mutex> lck(_runCV_mtx);
            _runCV.wait(lck, [&] {
                // protection against spurious wake-ups
                return !_started || _timer->isNotifyCalled(&_runCV);
            });
        }

        if (!_started)
            return;

        try {
            this->run();
        } catch (std::exception& e) {
            std::cerr << "Exception catched:  " << _name << " - " << e.what() << std::endl;
        }

        _timer->setNotifyCalled(&_runCV, false);
    }
}

std::string Worker::name() const
{
    return _name;
}

} // namespace essentials
