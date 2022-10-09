
#include "engine/BasicPlan.h"
#include "engine/IPlanCreator.h"
#include "engine/logging/Logging.h"
#include "engine/model/Plan.h"
#include "engine/modelmanagement/factories/Factory.h"
#include <engine/RuntimePlanFactory.h>

namespace alica
{

RuntimePlanFactory::RuntimePlanFactory(ConfigChangeListener& configChangeListener, IAlicaWorldModel* wm, const IAlicaTraceFactory* traceFactory,
        const TeamManager& teamManager, const IAlicaTimerFactory& timerFactory)
        : _traceFactory(traceFactory)
        , _teamManager(teamManager)
        , _timerFactory(timerFactory)
        , _wm(wm)
{
    auto reloadFunctionPtr = std::bind(&RuntimePlanFactory::reload, this, std::placeholders::_1);
    configChangeListener.subscribe(reloadFunctionPtr);
    reload(configChangeListener.getConfig());
}

void RuntimePlanFactory::reload(const YAML::Node& config)
{
    if (Factory::isValid(config["Alica"]["CustomerLibrary"]) && Factory::isValid(config["Alica"]["CustomerLibrary"]["Folder"])) {
        _customerLibraryFolder = config["Alica"]["CustomerLibrary"]["Folder"].as<std::string>();
        Logging::logDebug("AE") << "RuntimePlanFactory: Library folder: " << _customerLibraryFolder;
    }
}

void RuntimePlanFactory::init(std::unique_ptr<IPlanCreator>&& pc)
{
    _creator = std::move(pc);
}

std::unique_ptr<BasicPlan> RuntimePlanFactory::create(int64_t id, const Plan* planModel) const
{
    PlanContext ctx{_wm, planModel->getName(), planModel, _customerLibraryFolder, _traceFactory};
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
