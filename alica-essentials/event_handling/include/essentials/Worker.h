#pragma once

#include <chrono>
#include <condition_variable>
#include <string>
#include <thread>

#define WORKER_DEBUG

namespace std
{
class thread;
}

namespace essentials
{
class Timer;
class Worker
{
public:
    Worker(std::string name);
    virtual ~Worker();
    virtual void run() = 0; /** < Meant to be overwritten by derived classes. */
    bool stop();
    bool start();
    void setIntervalMS(std::chrono::milliseconds delay);
    void setDelayedStartMS(std::chrono::milliseconds delayedStartMS);

    std::string name; /** < The name of this worker. */

  protected:
    std::condition_variable runCV;

    bool started; /** < Is always true except when the worker is shutting down. */
    bool running; /** < Tells us whether the worker is currently running (or active). */

    std::thread* runThread;   /** < Executes the runInternal and thereby the abstract run method. */
    essentials::Timer* timer; /** < Triggers the condition_variable of the runThread. */

  private:
    void runInternal();

    std::mutex runCV_mtx;
};

} // namespace essentials
