
#include <engine/RuntimePlanFactory.h>

#include "engine/BasicPlan.h"
#include "engine/IPlanCreator.h"
#include "engine/model/Plan.h"

#include <alica_common_config/debug_output.h>

namespace alica
{

RuntimePlanFactory::RuntimePlanFactory(IPlanCreator& pc, IAlicaWorldModel* wm, AlicaEngine* engine)
        : _creator(pc)
        , _engine(engine)
        , _wm(wm)
{
}

std::unique_ptr<BasicPlan> RuntimePlanFactory::create(int64_t id, const Plan* planModel) const
{
    PlanContext ctx{_wm, planModel->getName(), planModel};
    std::unique_ptr<BasicPlan> basicPlan = _creator.createPlan(id, ctx);
    if (!basicPlan) {
        ALICA_ERROR_MSG("RuntimePlanFactory: Plan creation failed: " << id);
        return nullptr;
    }

    // TODO Cleanup: get rid of this later, behaviour only needs traceFactory, teamManager and not entire engine
    basicPlan->setEngine(_engine);
    return basicPlan;
}

} /* namespace alica */
