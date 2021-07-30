#include "engine/scheduler/Scheduler.h"
#include "engine/AlicaEngine.h"

#include <algorithm>

namespace alica
{

namespace scheduler
{

Scheduler::Scheduler(const alica::AlicaEngine* ae, const YAML::Node& config)
        : _ae(ae)
        , _jobID(0)
        , _notifierIsActive(false)
{
    _running = true;
    int threadPoolSize = config["Alica"]["ThreadPoolSize"].as<int, int>(std::max(1u, std::thread::hardware_concurrency()));

    _alicaTimerFactory = std::make_unique<typename alica::AlicaTimerFactory<typename alica::SyncStopTimerRos<ros::CallbackQueue>, ros::CallbackQueue,
            typename alica::ThreadPoolRos<ros::CallbackQueue>>>(threadPoolSize);

    for (int i = 0; i < threadPoolSize; i++) {
        _workers.emplace_back(std::thread(&Scheduler::workerFunction, this));
    }

    _workers.emplace_back(std::thread(&Scheduler::workerNotifier, this));
    _workers.emplace_back(std::thread(&Scheduler::monitorJobs, this));
}

Scheduler::~Scheduler()
{
    if (_running.load()) {
        terminate();
    }
}

void Scheduler::terminate()
{
    _running = false;
    {
        std::unique_lock<std::mutex> lock(_workerMtx);
        _jobQueue.clear();
    }

    for (std::thread& worker : _workers) {
        _workerCV.notify_all();
        _workerNotifierCV.notify_one();
        worker.join();
    }
    _workers.clear();
}

void Scheduler::schedule(std::shared_ptr<Job> job, bool notify)
{
    if (job->scheduledTime.inNanoseconds() == 0) {
        job->scheduledTime = _ae->getAlicaClock().now();
    }

    {
        std::unique_lock<std::mutex> lock(_workerMtx);
        if (job->isRepeated) {
            alica::SyncStopTimerRos<ros::CallbackQueue> *timer = _alicaTimerFactory->createTimer(job->cb, job->repeatInterval);
            timer->start();
            _timers[job->id] = timer;
        } else {
            _jobQueue.insert(std::move(job));
        }
    }

    if (notify) {
        _workerCV.notify_one();
    }
}

int Scheduler::getNextJobID()
{
    return ++_jobID;
}

std::vector<std::weak_ptr<Job>> Scheduler::getPrerequisites(int id)
{
    return _jobQueue.getPrerequisites(id);
}

void Scheduler::workerFunction()
{
    while (_running.load()) {
        std::shared_ptr<Job> job;
        bool executeJob = true;
        {
            std::unique_lock<std::mutex> lock(_workerMtx);
            // wait when no executable job is available. Do no wait when queue has executable jobs or the scheduler has been terminated.
            _workerCV.wait(lock, [this, &job] {
                job = _jobQueue.getAvailableJob(_ae->getAlicaClock().now());
                if (!job && !_jobQueue.isEmpty()) {
                    _notifierIsActive = true;
                    _workerNotifierCV.notify_one();
                }
                return !_running.load() || job;
            });

            if (!_running.load()) {
                continue;
            }
            job->inProgress = true;
        }

        if (executeJob) {
            try {
                std::cerr << "execute job: " << job->id << std::endl;
                job->cb();
                std::cerr << "finished job: " << job->id << std::endl;
                if (job->isRepeated) {
                    job->scheduledTime += job->repeatInterval;
                    job->inProgress = false;
                } else {
                    std::cerr << "reset job with id " << job->id << std::endl;
                    job.reset();
                    _workerCV.notify_one();
                }
            } catch (const std::bad_function_call& e) {
                ALICA_ERROR_MSG("Bad function call");
            }
        } else {
            schedule(std::move(job), false);
        }
    }
}

void Scheduler::workerNotifier()
{
    while (_running.load()) {
        {
            std::unique_lock<std::mutex> lock(_workerNotifierMtx);
            _workerNotifierCV.wait(lock, [this] { return _notifierIsActive || !_running.load(); });

            if (!_running.load()) {
                break;
            }
            _ae->getAlicaClock().sleep(_jobQueue.getLowestScheduledTime() - _ae->getAlicaClock().now());
            _notifierIsActive = false;
            _workerCV.notify_one();
        }
    }
}

void Scheduler::monitorJobs()
{
    while (_running.load()) {
        _jobQueue.detectDelayedJobs(_ae->getAlicaClock().now());
        _ae->getAlicaClock().sleep(alica::AlicaTime::seconds(5));
    }
}
} // namespace scheduler
} // namespace alica
