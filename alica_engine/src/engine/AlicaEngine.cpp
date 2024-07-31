#include "engine/AlicaEngine.h"

#include "engine/IRoleAssignment.h"
#include "engine/StaticRoleAssignment.h"
#include "engine/Types.h"
#include "engine/UtilityFunction.h"
#include "engine/constraintmodul/VariableSyncModule.h"
#include "engine/logging/AlicaDefaultLogger.h"
#include "engine/model/Plan.h"
#include "engine/model/RoleSet.h"
#include "engine/model/Transition.h"
#include "engine/model/TransitionCondition.h"
#include "engine/modelmanagement/ModelManager.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/syncmodule/SyncModule.h"
#include "engine/teammanager/TeamManager.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <random>
#include <stdlib.h>

namespace alica
{

/**
 * The main class.
 */
AlicaEngine::AlicaEngine(AlicaContext& ctx, YAML::Node& config, const AlicaContextParams& alicaContextParams)
        : _configChangeListener(config)
        , _ctx(ctx)
        , _stepCalled(false)
        , _stepEngine(alicaContextParams.stepEngine)
        , _modelManager(_configChangeListener, alicaContextParams.configPaths, _planRepository)
        , _masterPlan(_modelManager.loadPlanTree(alicaContextParams.masterPlanName, alicaContextParams.placeholderMapping))
        , _roleSet(_modelManager.loadRoleSet(alicaContextParams.roleSetName))
        , _teamManager(_configChangeListener, getPlanRepository(), _ctx.getCommunicator(), _ctx.getAlicaClock(), getVersion(), getMasterPlanId(),
                  _ctx.getLocalAgentName(), alicaContextParams.agentID)
        , _syncModul(_configChangeListener, getTeamManager(), getPlanRepository(), _ctx.getCommunicator(), _ctx.getAlicaClock())
        , _roleAssignment(std::make_unique<StaticRoleAssignment>(_ctx.getCommunicator(), getPlanRepository(), editTeamManager()))
        , _teamObserver(_configChangeListener, editRoleAssignment(), _ctx.getCommunicator(), _ctx.getAlicaClock(), getPlanRepository(), editTeamManager())
        , _auth(_configChangeListener, _ctx.getCommunicator(), _ctx.getAlicaClock(), editTeamManager())
        , _variableSyncModule(std::make_unique<VariableSyncModule>(
                  _configChangeListener, _ctx.getCommunicator(), _ctx.getAlicaClock(), editTeamManager(), _ctx.getTimerFactory()))
        , _planBase(_configChangeListener, _ctx.getAlicaClock(), _ctx.getCommunicator(), editRoleAssignment(), editSyncModul(), editAuth(), editTeamObserver(),
                  editTeamManager(), getPlanRepository(), _stepEngine, _stepCalled, getGlobalBlackboard(), editResultStore(), _ctx.getSolvers(),
                  getTimerFactory(), getTraceFactory())
{
    auto reloadFunctionPtr = std::bind(&AlicaEngine::reload, this, std::placeholders::_1);
    _configChangeListener.subscribe(reloadFunctionPtr);
    reload(_ctx.getConfig());

    if (!_planRepository.verifyPlanBase()) {
        AlicaEngine::abort(LOGNAME, "Error in parsed plans.");
    }

    Logging::logDebug(LOGNAME) << "Constructor finished!";
}

AlicaEngine::~AlicaEngine()
{
    if (_initialized) {
        terminate();
    }
    _roleSet = nullptr;
    _masterPlan = nullptr;
}

void AlicaEngine::reload(const YAML::Node& config)
{
    PartialAssignment::allowIdling(config["Alica"]["AllowIdling"].as<bool>());
    _maySendMessages = !config["Alica"]["SilentStart"].as<bool>();
    _useStaticRoles = config["Alica"]["UseStaticRoles"].as<bool>();
    if (!_useStaticRoles) {
        AlicaEngine::abort(LOGNAME, "Unknown RoleAssignment Type!");
    }
}

/**
 * Initialise the engine
 * @return bool true if everything worked false otherwise
 */
bool AlicaEngine::init(AlicaCreators&& creatorCtx)
{
    if (_initialized) {
        Logging::logWarn(LOGNAME) << "Already initialized.";
        return true; // todo false?
    }

    _planBase.init(std::move(creatorCtx.behaviourCreator), std::move(creatorCtx.planCreator));

    _roleAssignment->init();

    _expressionHandler.attachAll(_planRepository, creatorCtx);
    UtilityFunction::initDataStructures(getPlanRepository(), getRoleSet(), getRoleAssignment(), getTeamManager());

    RunningPlan::init(_ctx.getConfig());
    _teamManager.init();
    _syncModul.init();
    _variableSyncModule->init();
    _auth.init();
    initTransitionConditions(creatorCtx.transitionConditionCreator.get());

    _initialized = true;
    return true;
}

void AlicaEngine::start()
{
    // TODO: Removing this api need major refactoring of unit tests.
    _planBase.start(_masterPlan);
    Logging::logInfo(LOGNAME) << "Engine started!";
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
    _variableSyncModule->close();
    _initialized = false;
}

void AlicaEngine::initTransitionConditions(ITransitionConditionCreator* creator)
{
    for (const Transition* transition : _planRepository.getTransitions()) {
        TransitionCondition* transitionCondition = transition->getTransitionCondition();

        if (_defaultTransitionConditionCreator.isDefaultTransitionCondition(transitionCondition->getName())) {
            transitionCondition->setEvalCallback(_defaultTransitionConditionCreator.createConditions(transitionCondition->getName()));
        } else {
            TransitionConditionContext ctx{transitionCondition->getName(), transitionCondition->getLibraryName(), transitionCondition->getId()};
            transitionCondition->setEvalCallback(creator->createConditions(ctx.conditionConfId, ctx));
        }
    }
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

const YAML::Node& AlicaEngine::getConfig() const
{
    return _ctx.getConfig();
}

/**
 * Triggers the engine to run one iteration.
 * Attention: This method call is asynchronous to the triggered iteration.
 * So please wait long enough to let the engine do its stuff of its iteration,
 * before you read values, which will be changed by this iteration.
 */
void AlicaEngine::stepNotify()
{
    _planBase.stepNotify();
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
