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
    try {
        onInit();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicPlan] Exception in Plan-INIT" << std::endl << e.what());
    }
    _activeRunJobId =  _ae->editScheduler().schedule(std::bind(&BasicPlan::doRun, this, nullptr), getInterval());
}

void BasicPlan::doRun(void* msg)
{
    try {
        run(msg);
    } catch (const std::exception& e) {
        std::string err = std::string("Exception caught") + std::string(" - ") + std::string(e.what());
        sendLogMessage(4, err);
    }
}

void BasicPlan::doTerminate()
{
    _ae->editScheduler().cancelJob(_activeRunJobId);
    try {
        onTerminate();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicPlan] Exception in Plan-TERMINATE" << std::endl << e.what());
    }
    _planStarted = false;
}

void BasicPlan::sendLogMessage(int level, const std::string& message) const
{
    _ae->getCommunicator().sendLogMessage(level, message);
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