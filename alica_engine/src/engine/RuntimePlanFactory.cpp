
#include <engine/RuntimePlanFactory.h>

#include "engine/AlicaEngine.h"
#include "engine/BasicPlan.h"
#include "engine/IPlanCreator.h"
#include "engine/logging/Logging.h"
#include "engine/model/Plan.h"

namespace alica
{

RuntimePlanFactory::RuntimePlanFactory(IAlicaWorldModel* wm, AlicaEngine* engine)
        : _engine(engine)
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
        Logging::logError("RuntimePlanFactory") << "Plan creation failed: " << id;
        return nullptr;
    }

    // TODO Cleanup: get rid of this later, behaviour only needs traceFactory, teamManager and not entire engine
    basicPlan->setAlicaTraceFactory(_engine->getTraceFactory());
    basicPlan->setTeamManager(&_engine->getTeamManager());
    basicPlan->setAlicaTimerFactory(&_engine->getTimerFactory());

    return basicPlan;
}

} /* namespace alica */
