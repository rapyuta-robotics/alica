#include "engine/BasicPlan.h"

#include "engine/AlicaEngine.h"
#include "engine/model/Configuration.h"
#include "engine/PlanInterface.h"
#include "engine/scheduler/Scheduler.h"

namespace alica
{

BasicPlan::BasicPlan()
        : _msInterval(AlicaTime::milliseconds(DEFAULT_MS_INTERVAL))
        , _planStarted(false)
        , _activeRunJobId(-1)
{
}

void BasicPlan::doInit()
{
    _planStarted = true;
    init();
    _activeRunJobId =  _ae->editScheduler().schedule(std::bind(&BasicPlan::doRun, this, nullptr), getInterval());
}

void BasicPlan::doRun(void* msg)
{
    run(msg);
}

void BasicPlan::doTerminate()
{
    _ae->editScheduler().cancelJob(_activeRunJobId);
    onTermination();
    _planStarted = false;
}

void BasicPlan::start()
{
    _ae->editScheduler().schedule(std::bind(&BasicPlan::doInit, this));
}

void BasicPlan::stop()
{
    _ae->editScheduler().schedule(std::bind(&BasicPlan::doTerminate, this));
}

void BasicPlan::setConfiguration(const Configuration* conf)
{
    _configuration = conf;
}

ThreadSafePlanInterface BasicPlan::getPlanContext() const { return ThreadSafePlanInterface(isPlanStarted() ? _context : nullptr); }

} // namespace alica
