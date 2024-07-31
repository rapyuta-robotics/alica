
#include <engine/RuntimePlanFactory.h>

#include "engine/BasicPlan.h"
#include "engine/IPlanCreator.h"
#include "engine/logging/Logging.h"
#include "engine/model/Plan.h"
#include "engine/modelmanagement/factories/Factory.h"
#include <engine/RuntimePlanFactory.h>

namespace alica
{

RuntimePlanFactory::RuntimePlanFactory(std::shared_ptr<Blackboard> globalBlackboard, const IAlicaTraceFactory* traceFactory, const TeamManager& teamManager,
        const IAlicaTimerFactory& timerFactory)
        : _traceFactory(traceFactory)
        , _teamManager(teamManager)
        , _timerFactory(timerFactory)
        , _globalBlackboard(globalBlackboard)
{
}

void RuntimePlanFactory::init(std::unique_ptr<IPlanCreator>&& pc)
{
    {
        std::lock_guard lk(_m);
        _creator = std::move(pc);
        _initialized = true;
    }
    _cv.notify_all();
}

std::unique_ptr<BasicPlan> RuntimePlanFactory::create(int64_t id, const Plan* planModel) const
{
    {
        std::unique_lock lk(_m);
        while (!_initialized) {
            _cv.wait(lk);
        }
    }

    PlanContext ctx{_globalBlackboard, planModel->getName(), planModel, _traceFactory};
    std::unique_ptr<BasicPlan> basicPlan = _creator->createPlan(id, ctx);
    if (!basicPlan) {
        Logging::logError("RuntimePlanFactory") << "Plan creation failed: " << id;
        return nullptr;
    }

    basicPlan->setAlicaTraceFactory(_traceFactory);
    basicPlan->setTeamManager(&_teamManager);
    basicPlan->setAlicaTimerFactory(&_timerFactory);

    return basicPlan;
}

} /* namespace alica */
