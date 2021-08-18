#include "engine/BasicPlan.h"

#include "engine/AlicaEngine.h"
#include "engine/model/Configuration.h"

namespace alica
{

BasicPlan::BasicPlan()
        : _context(nullptr)
        , _planStarted(false)
        , _configuration(nullptr)
{
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

} // namespace alica
