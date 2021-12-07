#include "engine/RunnableObject.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanInterface.h"
#include "engine/model/ConfAbstractPlanWrapper.h"

#include <alica_common_config/debug_output.h>

#include <assert.h>
#include <iostream>

namespace alica
{
RunnableObject::RunnableObject(IAlicaWorldModel* wm, const std::string& name)
        : _name(name)
        , _engine(nullptr)
        , _configuration(nullptr)
        , _flags(static_cast<uint8_t>(Flags::TRACING_ENABLED))
        , _msInterval(AlicaTime::milliseconds(DEFAULT_MS_INTERVAL))
        , _requiresParameters(false)
        , _activeRunJobId(-1)
        , _signalContext(nullptr)
        , _execContext(nullptr)
        , _signalState(1)
        , _execState(1)
        , _wm(wm)
{
}

void RunnableObject::sendLogMessage(int level, const std::string& message) const
{
    _engine->getCommunicator().sendLogMessage(level, message);
}

ThreadSafePlanInterface RunnableObject::getPlanContext() const
{
    return ThreadSafePlanInterface(isExecutingInContext() ? _execContext.load() : nullptr);
}

void RunnableObject::stop()
{
    if (!isActive(_signalState.load())) {
        return;
    }
    ++_signalState;
    _engine->editScheduler().schedule(std::bind(&RunnableObject::doTerminate, this));
}

void RunnableObject::start(RunningPlan* rp)
{
    if (isActive(_signalState.load())) {
        return;
    }
    ++_signalState;
    _signalContext.store(rp);
    if(_requiresParameters) {
        assert(rp->getParent());
        assert(rp->getParent()->getBasicPlan());
        const auto& wrappers = rp->getParent()->getActiveState()->getConfAbstractPlanWrappers();
        auto it = std::find_if(wrappers.begin(), wrappers.end(), [this](const auto& wrapper_ptr){
            return wrapper_ptr->getAbstractPlan()->getName() == _name;
        });
        assert(it != wrappers.end());

        int64_t wrapperId = (*it)->getId();

        std::shared_ptr<BlackBoard> parentBlackBoard = rp->getParent()->getBasicPlan()->getBlackBoard();
        auto& planAttachment = rp->getParent()->getBasicPlan()->getPlanAttachment(wrapperId);
        auto initCall = [&](){
            if(!_blackBoard) {
                _blackBoard = std::make_shared<BlackBoard>();
            }
            LockedBlackBoardRW(*_blackBoard).clear();
            if(!planAttachment->setParameters(*parentBlackBoard, *_blackBoard)) {
                std::cerr << "Setting parameters failed, supposedly as the context has already changed.  Plan will not be scheduled" << std::endl;
                return;
            }
            doInit();
        };
        _engine->editScheduler().schedule(initCall);
    } else {

        std::shared_ptr<BlackBoard> parentBlackBoard;
        if(rp->getParent() && rp->getParent()->getBasicPlan()) {
            parentBlackBoard = rp->getParent()->getBasicPlan()->getBlackBoard();

        }
        auto initCall = [this, parentBlackBoard=std::move(parentBlackBoard)](){
            // Share blackboard with parent if we have one, or start fresh otherwise
            if(parentBlackBoard) {
                _blackBoard = parentBlackBoard;
            } else {
                _blackBoard = std::make_shared<BlackBoard>();
            }
            doInit();
        };
        _engine->editScheduler().schedule(initCall);
    }
}

void RunnableObject::setTerminatedState()
{
    ++_execState;

    clearFlags(Flags::RUN_TRACED);
    if (!areFlagsSet(Flags::INIT_EXECUTED)) {
        _execContext.store(nullptr);
        return;
    }
    clearFlags(Flags::INIT_EXECUTED);
}

void RunnableObject::traceTermination()
{
    if (_trace) {
        _trace->setLog({"Terminate", "true"});
        _trace.reset();
    }
}

void RunnableObject::initTrace()
{
    // Get closest parent that has a trace
    if (areFlagsSet(Flags::TRACING_ENABLED) && _engine->getTraceFactory()) {
        auto parent = _execContext.load()->getParent();
        for (; parent && (!parent->getBasicPlan() || !parent->getBasicPlan()->getTraceContext().has_value()); parent = parent->getParent())
            ;
        _trace = _engine->getTraceFactory()->create(_name, parent ? parent->getBasicPlan()->getTraceContext() : std::nullopt);
    }
}

void RunnableObject::traceRun()
{
    if (_trace && !areFlagsSet(Flags::RUN_TRACED)) {
        _trace->setLog({"Run", "true"});
        setFlags(Flags::RUN_TRACED);
    }
}

void RunnableObject::traceInit(const std::string& type)
{
    if (_trace) {
        _trace->setLog({type, "true"});
        _trace->setLog({"Init", "true"});
    }
}

} /* namespace alica */
