#include "engine/BasicPlan.h"

#include "engine/AlicaEngine.h"
#include "engine/model/Configuration.h"
#include "engine/PlanInterface.h"

namespace alica
{

BasicPlan::BasicPlan()
        : _planStarted(false)
        , _context(nullptr)
        , _configuration(nullptr)
{
}

void BasicPlan::doInit()
{
    init();
    _planStarted = true;
}

void BasicPlan::doTerminate()
{
    onTermination();
    _planStarted = false;
}

void BasicPlan::start()
{
    std::function<void()> cb = std::bind(&BasicPlan::doInit, this);
    _ae->editScheduler().schedule(cb);
}

void BasicPlan::stop()
{
    std::function<void()> cb = std::bind(&BasicPlan::doTerminate, this);
    _ae->editScheduler().schedule(cb);
}

void BasicPlan::setConfiguration(const Configuration* conf)
{
    _configuration = conf;
}

ThreadSafePlanInterface BasicPlan::getPlanContext() const { return ThreadSafePlanInterface(isPlanStarted() ? _context : nullptr); }

} // namespace alica
