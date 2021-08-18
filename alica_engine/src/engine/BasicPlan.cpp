#include "engine/BasicPlan.h"

#include "engine/AlicaEngine.h"
#include "engine/model/Configuration.h"
#include "engine/PlanInterface.h"

namespace alica
{

BasicPlan::BasicPlan()
        : _context(nullptr)
        , _planStarted(false)
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
