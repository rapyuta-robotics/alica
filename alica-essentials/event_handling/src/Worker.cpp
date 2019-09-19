#include "essentials/Worker.h"
#include "essentials/ITrigger.h"
#include "essentials/Timer.h"

#include <string>

namespace essentials
{

Worker::Worker(std::string name)
        : name(name)
        , started(true)
        , runCV()
{
    this->timer = new essentials::Timer(0, 0);
    this->timer->registerCV(&this->runCV);
    this->runThread = new std::thread(&Worker::runInternal, this);
}

Worker::~Worker()
{
    this->started = false;
    this->runCV.notify_all();
    this->timer->start();
    this->runThread->join();
    delete this->runThread;
    delete this->timer;
}

bool Worker::stop()
{
    return this->timer->stop();
}

bool Worker::start()
{
    return this->timer->start();
}

bool Worker::isRunning() const {
    return this->timer->isRunning();
}

void Worker::setIntervalMS(std::chrono::milliseconds intervalMS)
{
    this->timer->setInterval(intervalMS.count());
}

void Worker::setDelayedStartMS(std::chrono::milliseconds delayedStartMS)
{
    this->timer->setDelayedStart(delayedStartMS.count());
}

void Worker::runInternal()
{
    std::unique_lock<std::mutex> lck(runCV_mtx);
    while (this->started) {
        this->runCV.wait(lck, [&] {
            // protection against spurious wake-ups
            return !this->started || this->timer->isNotifyCalled(&runCV);
        });

        if (!this->started)
            return;

        try {
            this->run();
        } catch (std::exception& e) {
            std::cerr << "Exception catched:  " << this->name << " - " << e.what() << std::endl;
        }

        this->timer->setNotifyCalled(false, &runCV);
    }
}

} // namespace essentials
