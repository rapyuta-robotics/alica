
#include "engine/AlicaEngine.h"
#include <SystemConfig.h>

#include "engine/IRoleAssignment.h"
#include "engine/StaticRoleAssignment.h"
#include "engine/UtilityFunction.h"
#include "engine/constraintmodul/VariableSyncModule.h"
#include "engine/model/Plan.h"
#include "engine/model/RoleSet.h"
#include "engine/planselector/PartialAssignment.h"
#include <alica_common_config/debug_output.h>
#include <essentials/AgentIDManager.h>

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
AlicaEngine::AlicaEngine(AlicaContext& ctx, const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine, const AgentID* agentID)
        : _ctx(ctx)
        , _stepCalled(false)
        , _log(this)
        , _stepEngine(stepEngine)
        , _agentIDManager(new essentials::AgentIDFactory())
        , _planParser(&_planRepository)
        , _teamManager(this, (agentID != nullptr ? _agentIDManager.getIDFromBytes(agentID->toByteVector()) : nullptr))
        , _syncModul(this)
        , _behaviourPool(this)
        , _teamObserver(this)
        , _variableSyncModule(std::make_unique<VariableSyncModule>(this))
        , _auth(this)
        , _planBase(this)
        , _masterPlan(_planParser.parsePlanTree(masterPlanName))
        , _roleSet(_planParser.parseRoleSet(roleSetName))
        , _roleAssignment(std::make_unique<StaticRoleAssignment>(this))
{
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    PartialAssignment::allowIdling(sc["Alica"]->get<bool>("Alica.AllowIdling", NULL));
    _maySendMessages = !sc["Alica"]->get<bool>("Alica.SilentStart", NULL);
    _useStaticRoles = sc["Alica"]->get<bool>("Alica.UseStaticRoles", NULL);
    if (!_useStaticRoles) {
        AlicaEngine::abort("Unknown RoleAssignment Type!");
    }

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

/**
 * Initialise the engine
 * @param bc A behaviourcreator
 * @param roleSetName A string, the roleset to be used. If empty, a default roleset is looked for
 * @param masterPlanName A string, the top-level plan to be used
 * @param roleSetDir A string, the directory in which to search for roleSets. If empty, the base role path will be used.
 * @param stepEngine A bool, whether or not the engine should start in stepped mode
 * @return bool true if everything worked false otherwise
 */
bool AlicaEngine::init(AlicaCreators& creatorCtx)
{
    _stepCalled = false;
    bool everythingWorked = true;
    everythingWorked &= _behaviourPool.init(*creatorCtx.behaviourCreator);
    _roleAssignment->init();
    _auth.init();

    _expressionHandler.attachAll(_planRepository, creatorCtx);
    UtilityFunction::initDataStructures(this);
    _syncModul.init();
    _variableSyncModule->init();
    RunningPlan::init();
    _teamManager.init();
    return everythingWorked;
}

void AlicaEngine::start()
{
    // TODO: Removing this api need major refactoring of unit tests.
    _planBase.start(_masterPlan);
    ALICA_DEBUG_MSG("AE: Engine started!");
}
/**
 * Closes the engine for good.
 */
void AlicaEngine::terminate()
{
    _maySendMessages = false;

    _behaviourPool.stopAll();
    _behaviourPool.terminateAll();

    _planBase.stop();
    _auth.close();
    _syncModul.close();
    _teamObserver.close();
    _log.close();
}

const IAlicaCommunication& AlicaEngine::getCommunicator() const
{
    return _ctx.getCommunicator();
}

const AlicaClock& AlicaEngine::getAlicaClock() const
{
    return _ctx.getAlicaClock();
}

std::string AlicaEngine::getRobotName() const
{
    return _ctx.getRobotName();
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

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing a part of a ROS
 * message and receiving a pointer to a corresponding AgentID object.
 */
AgentIDConstPtr AlicaEngine::getIdFromBytes(const std::vector<uint8_t>& idByteVector)
{
    return AgentIDConstPtr(_agentIDManager.getIDFromBytes(idByteVector));
}

} // namespace alica
