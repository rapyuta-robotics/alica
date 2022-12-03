
#include <engine/RuntimePlanFactory.h>

#include "engine/BasicPlan.h"
#include "engine/IPlanCreator.h"
#include "engine/logging/Logging.h"
#include "engine/model/Plan.h"
#include "engine/modelmanagement/factories/Factory.h"
#include <engine/RuntimePlanFactory.h>

namespace alica
{

RuntimePlanFactory::RuntimePlanFactory(
        Blackboard& worldModels, const IAlicaTraceFactory* traceFactory, const TeamManager& teamManager, const IAlicaTimerFactory& timerFactory)
        : _traceFactory(traceFactory)
        , _teamManager(teamManager)
        , _timerFactory(timerFactory)
        , _worldModels(worldModels)
{
}

void RuntimePlanFactory::init(std::unique_ptr<IPlanCreator>&& pc)
{
    _creator = std::move(pc);
}

std::unique_ptr<BasicPlan> RuntimePlanFactory::create(int64_t id, const Plan* planModel) const
{
    PlanContext ctx{_worldModels, planModel->getName(), planModel, _traceFactory};
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
