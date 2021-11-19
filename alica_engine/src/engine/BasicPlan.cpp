#include "engine/BasicPlan.h"

#include "engine/AlicaEngine.h"
#include "engine/model/Configuration.h"
#include "engine/model/Plan.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/scheduler/Scheduler.h"

#include "engine/PlanInterface.h"
#include "engine/IPlanAttachmentCreator.h"

namespace alica
{

BasicPlan::BasicPlan()
        : RunnableObject()
{
}

void BasicPlan::doInit()
{
    ++_execState;

    if (!isExecutingInContext()) {
        return;
    }
    setFlags(Flags::INIT_EXECUTED);

    _execContext = _signalContext.exchange(nullptr);

    initTrace();

    try {
        traceInit("Plan");
        onInit();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicPlan] Exception in Plan-INIT" << std::endl << e.what());
    }
    // Do not schedule runJob when freq is 0.
    if (_msInterval > AlicaTime::milliseconds(0)) {
        _activeRunJobId = _engine->editScheduler().schedule(std::bind(&BasicPlan::doRun, this, nullptr), getInterval());
    }
}

void BasicPlan::doRun(void* msg)
{
    try {
        traceRun();
        run(msg);
    } catch (const std::exception& e) {
        std::string err = std::string("Exception caught") + std::string(" - ") + std::string(e.what());
        sendLogMessage(4, err);
    }
}

void BasicPlan::doTerminate()
{
    if (_activeRunJobId != -1) {
        _engine->editScheduler().cancelJob(_activeRunJobId);
        _activeRunJobId = -1;
    }
    setTerminatedState();
    try {
        traceTermination();
        onTerminate();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicPlan] Exception in Plan-TERMINATE" << std::endl << e.what());
    }

    _execContext.store(nullptr);
}

void BasicPlan::createChildAttachments(const Plan* plan, IPlanAttachmentCreator& planAttachmentCreator)
{
    for(const State* state : plan->getStates()) {
        for(const ConfAbstractPlanWrapper* wrapper : state->getConfAbstractPlanWrappers()) {
            _planAttachments.emplace(wrapper->getId(), planAttachmentCreator.createPlanAttachment(wrapper->getId()));
        }
    }
}

} // namespace alica
