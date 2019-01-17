
#include "engine/AlicaEngine.h"
#include "engine/BehaviourPool.h"
#include "engine/IConditionCreator.h"
#include "engine/IRoleAssignment.h"
#include "engine/Logger.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/StaticRoleAssignment.h"
#include "engine/TeamObserver.h"
#include "engine/UtilityFunction.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/constraintmodul/VariableSyncModule.h"
#include "engine/expressionhandler/ExpressionHandler.h"
#include "engine/model/Plan.h"
#include "engine/model/RoleSet.h"
#include "engine/parser/PlanParser.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/teammanager/TeamManager.h"
#include <engine/syncmodule/SyncModule.h>

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
AlicaEngine::AlicaEngine(essentials::AgentIDManager* idManager, const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine)
        : _stepCalled(false)
        , _planBase(nullptr)
        , _communicator(nullptr)
        , _alicaClock(nullptr)
        , _terminating(false)
        , _expressionHandler(nullptr)
        , _log(nullptr)
        , _auth(nullptr)
        , _variableSyncModule(nullptr)
        , _stepEngine(stepEngine)
        , _agentIDManager(idManager)
{
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    _maySendMessages = !sc["Alica"]->get<bool>("Alica.SilentStart", NULL);
    _useStaticRoles = sc["Alica"]->get<bool>("Alica.UseStaticRoles", NULL);
    PartialAssignment::allowIdling(sc["Alica"]->get<bool>("Alica.AllowIdling", NULL));

    _planRepository = new PlanRepository();
    _planParser = new PlanParser(_planRepository);
    _masterPlan = _planParser->parsePlanTree(masterPlanName);
    _roleSet = _planParser->parseRoleSet(roleSetName);
    _teamManager = new TeamManager(this, true);
    _teamManager->init();
    _behaviourPool = new BehaviourPool(this);
    _teamObserver = new TeamObserver(this);
    if (_useStaticRoles) {
        _roleAssignment = new StaticRoleAssignment(this);
    } else {
        AlicaEngine::abort("Unknown RoleAssignment Type!");
    }
    // the communicator is expected to be set before init() is called
    _roleAssignment->setCommunication(_communicator);
    _syncModul = new SyncModule(this);

    if (!_planRepository->verifyPlanBase()) {
        abort("Error in parsed plans.");
    }
    ALICA_DEBUG_MSG("AE: Constructor finished!");
}

AlicaEngine::~AlicaEngine()
{
    if (!_terminating) {
        shutdown();
    }
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
bool AlicaEngine::init(IBehaviourCreator* bc, IConditionCreator* cc, IUtilityCreator* uc, IConstraintCreator* crc)
{
    if (!bc || !cc || !uc || !crc) {
        ALICA_ERROR_MSG("Empty creators!");
        return false;
    }

    if (!_expressionHandler) {
        _expressionHandler = new ExpressionHandler(this);
    }

    _stepCalled = false;
    bool everythingWorked = true;
    everythingWorked &= _behaviourPool->init(*bc);
    _auth = new AuthorityManager(this);
    _log = new Logger(this);
    _roleAssignment->init();
    _auth->init();
    _planBase = new PlanBase(this, _masterPlan);

    _expressionHandler->attachAll(*cc, *uc, *crc);
    UtilityFunction::initDataStructures(this);
    _syncModul->init();
    if (!_variableSyncModule) {
        _variableSyncModule = new VariableSyncModule(this);
    }
    if (_communicator) {
        _communicator->startCommunication();
    }
    if (_variableSyncModule) {
        _variableSyncModule->init();
    }
    RunningPlan::init();
    return everythingWorked;
}

/**
 * Closes the engine for good.
 */
void AlicaEngine::shutdown()
{
    if (_communicator != nullptr) {
        _communicator->stopCommunication();
    }
    _terminating = true;
    _maySendMessages = false;

    if (_behaviourPool != nullptr) {
        _behaviourPool->stopAll();
        _behaviourPool->terminateAll();
        delete _behaviourPool;
        _behaviourPool = nullptr;
    }

    if (_planBase != nullptr) {
        _planBase->stop();
        delete _planBase;
        _planBase = nullptr;
    }

    if (_auth != nullptr) {
        _auth->close();
        delete _auth;
        _auth = nullptr;
    }

    if (_syncModul != nullptr) {
        _syncModul->close();
        delete _syncModul;
        _syncModul = nullptr;
    }

    if (_teamObserver != nullptr) {
        _teamObserver->close();
        delete _teamObserver;
        _teamObserver = nullptr;
    }

    if (_log != nullptr) {
        _log->close();
        delete _log;
        _log = nullptr;
    }

    if (_planRepository != nullptr) {
        delete _planRepository;
        _planRepository = nullptr;
    }

    if (_planParser != nullptr) {
        delete _planParser;
        _planParser = nullptr;
    }

    _roleSet = nullptr;
    _masterPlan = nullptr;

    if (_expressionHandler != nullptr) {
        delete _expressionHandler;
        _expressionHandler = nullptr;
    }

    if (_variableSyncModule != nullptr) {
        delete _variableSyncModule;
        _variableSyncModule = nullptr;
    }
    if (_roleAssignment != nullptr) {
        delete _roleAssignment;
        _roleAssignment = nullptr;
    }

    delete _alicaClock;
    _alicaClock = nullptr;
}

/**
 * Register with this EngineTrigger to be called after an engine iteration is complete.
 */
void AlicaEngine::iterationComplete()
{
    // TODO: implement the trigger function for iteration complete
}

/**
 * Starts the engine.
 */
void AlicaEngine::start()
{
    _planBase->start();
    std::cout << "AE: Engine started" << std::endl;
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

void AlicaEngine::setAlicaClock(AlicaClock* clock)
{
    _alicaClock = clock;
}

void AlicaEngine::setTeamObserver(TeamObserver* teamObserver)
{
    _teamObserver = teamObserver;
}

void AlicaEngine::setSyncModul(SyncModule* syncModul)
{
    _syncModul = syncModul;
}

void AlicaEngine::setAuth(AuthorityManager* auth)
{
    _auth = auth;
}

void AlicaEngine::setRoleAssignment(IRoleAssignment* roleAssignment)
{
    _roleAssignment = roleAssignment;
}

void AlicaEngine::setStepEngine(bool stepEngine)
{
    _stepEngine = stepEngine;
}

/**
 * Gets the robot name, either by access the environment variable "ROBOT", or if that isn't set, the hostname.
 * @return The robot name under which the engine operates, a string
 */
std::string AlicaEngine::getRobotName() const
{
    return essentials::SystemConfig::getInstance().getHostname();
}

void AlicaEngine::setLog(Logger* log)
{
    _log = log;
}

bool AlicaEngine::isTerminating() const
{
    return _terminating;
}

void AlicaEngine::setMaySendMessages(bool maySendMessages)
{
    _maySendMessages = maySendMessages;
}

void AlicaEngine::setCommunicator(IAlicaCommunication* communicator)
{
    _communicator = communicator;
}

void AlicaEngine::setResultStore(VariableSyncModule* resultStore)
{
    _variableSyncModule = resultStore;
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
    getPlanBase()->getStepModeCV()->notify_all();
}

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing a part of a ROS
 * message and receiving a pointer to a corresponding AgentID object.
 */
AgentIDConstPtr AlicaEngine::getIdFromBytes(const std::vector<uint8_t>& idByteVector) const
{
    return AgentIDConstPtr(_agentIDManager->getIDFromBytes(idByteVector));
}

} // namespace alica
