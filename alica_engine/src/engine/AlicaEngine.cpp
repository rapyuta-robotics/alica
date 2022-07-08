#include "engine/AlicaEngine.h"

#include "engine/IRoleAssignment.h"
#include "engine/RuntimeBehaviourFactory.h"
#include "engine/StaticRoleAssignment.h"
#include "engine/Types.h"
#include "engine/UtilityFunction.h"
#include "engine/constraintmodul/VariableSyncModule.h"
#include "engine/model/Plan.h"
#include "engine/model/RoleSet.h"
#include "engine/modelmanagement/ModelManager.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/syncmodule/SyncModule.h"
#include "engine/teammanager/TeamManager.h"

#include <algorithm>
#include <alica_common_config/debug_output.h>
#include <chrono>
#include <functional>
#include <random>
#include <stdlib.h>

namespace alica
{
/**
 * Abort execution with a message, called if initialization fails.
 * @param msg A string
 */
void AlicaEngine::abort(const std::string& msg)
{
    std::cerr << "ABORT: " << msg << std::endl;
    exit(EXIT_FAILURE);
}

/**
 * The main class.
 */
AlicaEngine::AlicaEngine(AlicaContext& ctx, YAML::Node& config, const AlicaContextParams& alicaContextParams)
        : _configChangeListener(config)
        , _ctx(ctx)
        , _stepCalled(false)
        , _stepEngine(alicaContextParams.stepEngine)
        , _modelManager(_configChangeListener, alicaContextParams.configPath, _planRepository)
        , _masterPlan(_modelManager.loadPlanTree(alicaContextParams.masterPlanName))
        , _roleSet(_modelManager.loadRoleSet(alicaContextParams.roleSetName))
        , _teamManager(_configChangeListener, _modelManager, getPlanRepository(), _ctx.getCommunicator(), _ctx.getAlicaClock(), _log, getVersion(),
                  getMasterPlanId(), _ctx.getLocalAgentName(), alicaContextParams.agentID)
        , _log(_configChangeListener, getTeamManager(), _teamObserver, getPlanRepository(), _ctx.getAlicaClock(), _ctx.getLocalAgentName())
        , _syncModul(_configChangeListener, getTeamManager(), getPlanRepository(), _ctx.getCommunicator(), _ctx.getAlicaClock())
        , _variableSyncModule(std::make_unique<VariableSyncModule>(
                  _configChangeListener, _ctx.getCommunicator(), _ctx.getAlicaClock(), editTeamManager(), _ctx.getTimerFactory()))
        , _auth(_configChangeListener, _ctx.getCommunicator(), _ctx.getAlicaClock(), editTeamManager())
        , _roleAssignment(std::make_unique<StaticRoleAssignment>(_ctx.getCommunicator(), getPlanRepository(), editTeamManager()))
        , _planBase(this)
        , _teamObserver(
                  _configChangeListener, editLog(), editRoleAssignment(), _ctx.getCommunicator(), _ctx.getAlicaClock(), getPlanRepository(), editTeamManager())
{
    auto reloadFunctionPtr = std::bind(&AlicaEngine::reload, this, std::placeholders::_1);
    subscribe(reloadFunctionPtr);
    reload(_ctx.getConfig());

    if (!_planRepository.verifyPlanBase()) {
        AlicaEngine::abort("Error in parsed plans.");
    }

    ALICA_DEBUG_MSG("AE: Constructor finished!");
}

AlicaEngine::~AlicaEngine()
{
    _roleSet = nullptr;
    _masterPlan = nullptr;
}

void AlicaEngine::reload(const YAML::Node& config)
{
    PartialAssignment::allowIdling(config["Alica"]["AllowIdling"].as<bool>());
    _maySendMessages = !config["Alica"]["SilentStart"].as<bool>();
    _useStaticRoles = config["Alica"]["UseStaticRoles"].as<bool>();
    if (!_useStaticRoles) {
        AlicaEngine::abort("Unknown RoleAssignment Type!");
    }
}

/**
 * Initialise the engine
 * @return bool true if everything worked false otherwise
 */
bool AlicaEngine::init(AlicaCreators&& creatorCtx)
{
    if (_initialized) {
        ALICA_WARNING_MSG("AE: Already initialized.");
        return true; // todo false?
    }

    _behaviourFactory = std::make_unique<RuntimeBehaviourFactory>(std::move(creatorCtx.behaviourCreator), _ctx.getWorldModel(), this);
    _planFactory = std::make_unique<RuntimePlanFactory>(std::move(creatorCtx.planCreator), _ctx.getWorldModel(), this);

    _stepCalled = false;
    _roleAssignment->init();

    _expressionHandler.attachAll(this, _planRepository, creatorCtx);
    UtilityFunction::initDataStructures(this);

    RunningPlan::init(_ctx.getConfig());
    _teamManager.init();
    _syncModul.init();
    _variableSyncModule->init();
    _auth.init();

    _initialized = true;
    return true;
}

void AlicaEngine::start()
{
    // TODO: Removing this api need major refactoring of unit tests.
    _planBase.start(_masterPlan, _ctx.getWorldModel());
    ALICA_DEBUG_MSG("AE: Engine started!");
}
/**
 * Closes the engine for good.
 */
void AlicaEngine::terminate()
{
    _maySendMessages = false;
    _planBase.stop();
    _auth.close();
    _syncModul.close();
    _teamObserver.close();
    _log.close();
    _variableSyncModule->close();
    _initialized = false;
}

const IAlicaCommunication& AlicaEngine::getCommunicator() const
{
    return _ctx.getCommunicator();
}

const AlicaClock& AlicaEngine::getAlicaClock() const
{
    return _ctx.getAlicaClock();
}

const IAlicaTraceFactory* AlicaEngine::getTraceFactory() const
{
    return _ctx.getTraceFactory();
}

std::string AlicaEngine::getLocalAgentName() const
{
    return _ctx.getLocalAgentName();
}

IAlicaTimerFactory& AlicaEngine::getTimerFactory() const
{
    return _ctx.getTimerFactory();
}

/**
 * Register with this EngineTrigger to be called after an engine iteration is complete.
 */
void AlicaEngine::iterationComplete()
{
    // TODO: implement the trigger function for iteration complete
}

int AlicaEngine::getVersion() const
{
    return _ctx.getVersion();
}

void AlicaEngine::setStepCalled(bool stepCalled)
{
    _stepCalled = stepCalled;
}

bool AlicaEngine::getStepCalled() const
{
    return _stepCalled;
}

bool AlicaEngine::getStepEngine() const
{
    return _stepEngine;
}

void AlicaEngine::setStepEngine(bool stepEngine)
{
    _stepEngine = stepEngine;
}

const YAML::Node& AlicaEngine::getConfig() const
{
    return _ctx.getConfig();
}

IAlicaWorldModel* AlicaEngine::getWorldModel() const
{
    return _ctx.getWorldModel();
}

void AlicaEngine::subscribe(std::function<void(const YAML::Node& config)> reloadFunction)
{
    _configChangeListener.subscribe(reloadFunction);
};

/**
 * Triggers the engine to run one iteration.
 * Attention: This method call is asynchronous to the triggered iteration.
 * So please wait long enough to let the engine do its stuff of its iteration,
 * before you read values, which will be changed by this iteration.
 */
void AlicaEngine::stepNotify()
{
    setStepCalled(true);
    _planBase.getStepModeCV()->notify_all();
}

void AlicaEngine::reloadConfig(const YAML::Node& config)
{
    _configChangeListener.reloadConfig(config);
}

ConfigChangeListener& AlicaEngine::getConfigChangeListener()
{
    return _configChangeListener;
}
} // namespace alica
