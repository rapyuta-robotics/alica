
#include <engine/RuntimeBehaviourFactory.h>

#include "engine/BasicBehaviour.h"
#include "engine/ConfigChangeListener.h"
#include "engine/IBehaviourCreator.h"
#include "engine/logging/Logging.h"
#include "engine/modelmanagement/factories/Factory.h"

namespace alica
{

RuntimeBehaviourFactory::RuntimeBehaviourFactory(ConfigChangeListener& configChangeListener, IAlicaWorldModel* wm, TeamManager& teamManager, PlanBase& planBase,
        const IAlicaCommunication& communication, const IAlicaTraceFactory* traceFactory, const IAlicaTimerFactory& timerFactory)
        : _wm(wm)
        , _teamManager(teamManager)
        , _planBase(planBase)
        , _communication(communication)
        , _traceFactory(traceFactory)
        , _timerFactory(timerFactory)
{
    auto reloadFunctionPtr = std::bind(&RuntimeBehaviourFactory::reload, this, std::placeholders::_1);
    configChangeListener.subscribe(reloadFunctionPtr);
    reload(configChangeListener.getConfig());
}

void RuntimeBehaviourFactory::reload(const YAML::Node& config)
{
    if (Factory::isValid(config["Alica"]["CustomerLibrary"]) && Factory::isValid(config["Alica"]["CustomerLibrary"]["Folder"])) {
        _customerLibraryFolder = config["Alica"]["CustomerLibrary"]["Folder"].as<std::string>();
        Logging::logDebug("AE") << "RuntimeBehaviourFactory: Library folder: " << _customerLibraryFolder;
    }
}

void RuntimeBehaviourFactory::init(std::unique_ptr<IBehaviourCreator>&& bc)
{
    _creator = std::move(bc);
}

std::unique_ptr<BasicBehaviour> RuntimeBehaviourFactory::create(int64_t id, const Behaviour* behaviourModel) const
{
    BehaviourContext ctx{_wm, behaviourModel->getName(), behaviourModel, _customerLibraryFolder, _traceFactory};
    std::unique_ptr<BasicBehaviour> basicBeh = _creator->createBehaviour(id, ctx);
    if (!basicBeh) {
        Logging::logError("RuntimeBehaviourFactory") << "Behaviour creation failed: " << id;
        return nullptr;
    }

    basicBeh->setTeamManager(&_teamManager);
    basicBeh->setPlanBase(&_planBase);
    basicBeh->setAlicaCommunication(&_communication);
    basicBeh->setAlicaTraceFactory(_traceFactory);
    basicBeh->setAlicaTimerFactory(&_timerFactory);
    return basicBeh;
}

} /* namespace alica */
