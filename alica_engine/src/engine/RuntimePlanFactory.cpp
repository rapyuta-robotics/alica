
#include "engine/AlicaEngine.h"
#include "engine/BasicPlan.h"
#include "engine/IPlanCreator.h"
#include "engine/model/Plan.h"
#include <alica_common_config/debug_output.h>
#include <engine/RuntimePlanFactory.h>

namespace alica
{

RuntimePlanFactory::RuntimePlanFactory(ConfigChangeListener& configChangeListener, IAlicaWorldModel* wm, AlicaEngine* engine)
        : _engine(engine)
        , _wm(wm)
{
    auto reloadFunctionPtr = std::bind(&RuntimePlanFactory::reload, this, std::placeholders::_1);
    configChangeListener.subscribe(reloadFunctionPtr);
    reload(configChangeListener.getConfig());
}

void RuntimePlanFactory::reload(const YAML::Node& config)
{
    _customerLibraryFolder = config["Alica"]["CustomerLibrary"]["Folder"].as<std::string>();
    std::cerr << _customerLibraryFolder << "Library folder" << std::endl;
}

void RuntimePlanFactory::init(std::unique_ptr<IPlanCreator>&& pc)
{
    _creator = std::move(pc);
}

std::unique_ptr<BasicPlan> RuntimePlanFactory::create(int64_t id, const Plan* planModel) const
{
    PlanContext ctx{_wm, planModel->getName(), planModel, _customerLibraryFolder};
    std::unique_ptr<BasicPlan> basicPlan = _creator->createPlan(id, ctx);
    if (!basicPlan) {
        ALICA_ERROR_MSG("RuntimePlanFactory: Plan creation failed: " << id);
        return nullptr;
    }

    // TODO Cleanup: get rid of this later, behaviour only needs traceFactory, teamManager and not entire engine
    basicPlan->setAlicaTraceFactory(_engine->getTraceFactory());
    basicPlan->setTeamManager(&_engine->getTeamManager());
    basicPlan->setAlicaTimerFactory(&_engine->getTimerFactory());

    return basicPlan;
}

} /* namespace alica */
