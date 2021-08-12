#include "engine/BasicPlan.h"

#include "engine/scheduler/Job.h"
#include "engine/AlicaEngine.h"

namespace alica
{

BasicPlan::BasicPlan() {}

void BasicPlan::start()
{
    std::function<void()> cb = std::bind(&BasicPlan::init, this);
    std::shared_ptr<scheduler::Job> initJob = std::make_shared<scheduler::Job>(cb);
    _ae->editScheduler().schedule(std::move(initJob));
}

void BasicPlan::stop()
{
    std::function<void()> cb = std::bind(&BasicPlan::onTermination, this);
    std::shared_ptr<scheduler::Job> terminateJob = std::make_shared<scheduler::Job>(cb);
    _ae->editScheduler().schedule(std::move(terminateJob));
}

} // namespace alica
