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
        , _signalState(1)
        , _execState(1)
{
}

void BasicPlan::doInit()
{
    ++_execState;
    _planStarted = true;
    try {
        onInit();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicPlan] Exception in Plan-INIT" << std::endl << e.what());
    }
    // Do not schedule runJob when freq is 0.
    if (_msInterval > AlicaTime::milliseconds(0)) {
        _activeRunJobId =  _ae->editScheduler().schedule(std::bind(&BasicPlan::doRun, this, nullptr), getInterval());
    }
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
    if (_activeRunJobId != -1) {
        _ae->editScheduler().cancelJob(_activeRunJobId);
        _activeRunJobId = -1;
    }
    ++_execState;
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

void BasicPlan::start(RunningPlan* rp)
{
    if (isActive(_signalState.load())) {
        return;
    }
    ++_signalState;
    // This has to be done after incrementing _signalState for correct behaviour of getPlanContext()
    _context.store(rp);
    _ae->editScheduler().schedule(std::bind(&BasicPlan::doInit, this));
}

void BasicPlan::stop()
{
    if (!isActive(_signalState.load())) {
        return;
    }
    ++_signalState;
    _ae->editScheduler().schedule(std::bind(&BasicPlan::doTerminate, this));
}

void BasicPlan::setConfiguration(const Configuration* conf)
{
    _configuration = conf;
}

void BasicPlan::setInterval(int32_t msInterval)
{
    _msInterval = AlicaTime::milliseconds(msInterval);
}

ThreadSafePlanInterface BasicPlan::getPlanContext() const { return ThreadSafePlanInterface(isExecutingInContext() ? _context.load() : nullptr); }

} // namespace alica
