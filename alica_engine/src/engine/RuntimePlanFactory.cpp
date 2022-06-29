
#include <engine/RuntimePlanFactory.h>

#include "engine/AlicaEngine.h"
#include "engine/BasicPlan.h"
#include "engine/IPlanCreator.h"
#include "engine/model/Plan.h"
#include <alica_common_config/debug_output.h>

namespace alica
{

RuntimePlanFactory::RuntimePlanFactory(
        IAlicaWorldModel* wm, const IAlicaTraceFactory* traceFactory, const TeamManager& teamManager, const IAlicaTimerFactory& timerFactory)
        : _traceFactory(traceFactory)
        , _teamManager(teamManager)
        , _timerFactory(timerFactory)
        , _wm(wm)
{
}

void RuntimePlanFactory::init(std::unique_ptr<IPlanCreator>&& pc)
{
    _creator = std::move(pc);
}

std::unique_ptr<BasicPlan> RuntimePlanFactory::create(int64_t id, const Plan* planModel) const
{
    PlanContext ctx{_wm, planModel->getName(), planModel};
    std::unique_ptr<BasicPlan> basicPlan = _creator->createPlan(id, ctx);
    if (!basicPlan) {
        ALICA_ERROR_MSG("RuntimePlanFactory: Plan creation failed: " << id);
        return nullptr;
    }

    basicPlan->setAlicaTraceFactory(_traceFactory);
    basicPlan->setTeamManager(&_teamManager);
    basicPlan->setAlicaTimerFactory(&_timerFactory);

    return basicPlan;
}

} /* namespace alica */
