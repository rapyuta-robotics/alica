#include "engine/BasicPlan.h"

#include "engine/AlicaEngine.h"
#include "engine/model/Configuration.h"
#include "engine/scheduler/Scheduler.h"

#include "engine/PlanInterface.h"

namespace alica
{

BasicPlan::BasicPlan(IAlicaWorldModel* wm)
        : RunnableObject(wm)
        , _isMasterPlan(false)
{
}

void BasicPlan::stop()
{
    doStop();
    _engine->editScheduler().schedule(std::bind(&BasicPlan::terminateJob, this));
}

void BasicPlan::start(RunningPlan* rp)
{
    doStart(rp);
    _engine->editScheduler().schedule(std::bind(&BasicPlan::initJob, this));
}

void BasicPlan::initJob()
{
    doInit();

    // Do not schedule runJob when freq is 0.
    if (_msInterval > AlicaTime::milliseconds(0)) {
        _activeRunJobId = _engine->editScheduler().schedule(std::bind(&BasicPlan::runJob, this), getInterval());
    }
}

void BasicPlan::runJob()
{
    doRun();
}

void BasicPlan::terminateJob()
{
    if (_activeRunJobId != -1) {
        _engine->editScheduler().cancelJob(_activeRunJobId);
        _activeRunJobId = -1;
    }
    doTerminate();
}

void BasicPlan::onInit_()
{
    if (_trace) {
        _trace->setTag("type", _isMasterPlan ? "master_plan" : "plan");
        if (_isMasterPlan) {
            _trace->finish();
        }
    }
    onInit();
}

void BasicPlan::onRun_()
{
    run(nullptr);
}

void BasicPlan::onTerminate_()
{
    onTerminate();
}

} // namespace alica
