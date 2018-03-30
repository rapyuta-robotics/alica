
#define AE_DEBUG

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
#include "engine/collections/AssignmentCollection.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/constraintmodul/VariableSyncModule.h"
#include "engine/expressionhandler/ExpressionHandler.h"
#include "engine/model/Plan.h"
#include "engine/model/RoleSet.h"
#include "engine/parser/PlanParser.h"
#include "engine/planselector/PartialAssignmentPool.h"
#include "engine/planselector/PlanSelector.h"
#include "engine/teammanager/TeamManager.h"
#include <engine/syncmodule/SyncModule.h>
#include <engine/syncmodule/SyncModule.h>

#include <supplementary/AgentIDManager.h>

using std::cout;
using std::endl;
namespace alica {
/**
 * The main class.
 */
AlicaEngine::AlicaEngine(supplementary::AgentIDManager* idManager, string roleSetName, string masterPlanName,
        string roleSetDir, bool stepEngine)
        : stepCalled(false)
        , planBase(nullptr)
        , planSelector(nullptr)
        , communicator(nullptr)
        , alicaClock(nullptr)
        , sc(supplementary::SystemConfig::getInstance())
        , terminating(false)
        , expressionHandler(nullptr)
        , log(nullptr)
        , auth(nullptr)
        , stepEngine(stepEngine)
        , pap(nullptr)
        , variableSyncModule(nullptr)
        , agentIDManager(idManager) {
#ifdef AE_DEBUG
    cout << "AE: Constructor finished!" << endl;
#endif
    this->maySendMessages = !(*sc)["Alica"]->get<bool>("Alica.SilentStart", NULL);
    this->useStaticRoles = (*sc)["Alica"]->get<bool>("Alica.UseStaticRoles", NULL);
    AssignmentCollection::maxEpsCount = (*this->sc)["Alica"]->get<short>("Alica.MaxEpsPerPlan", NULL);
    AssignmentCollection::allowIdling = (*this->sc)["Alica"]->get<bool>("Alica.AllowIdling", NULL);

    this->planRepository = new PlanRepository();
    this->planParser = new PlanParser(this, this->planRepository);
    this->masterPlan = this->planParser->parsePlanTree(masterPlanName);
    this->roleSet = this->planParser->parseRoleSet(roleSetName, roleSetDir);
    this->teamManager = new TeamManager(this, true);
    this->teamManager->init();
    this->behaviourPool = new BehaviourPool(this);
    this->teamObserver = new TeamObserver(this);
    if (this->useStaticRoles) {
        this->roleAssignment = new StaticRoleAssignment(this);
    } else {
        this->abort("Unknown RoleAssignment Type!");
    }
    // the communicator is expected to be set before init() is called
    this->roleAssignment->setCommunication(communicator);
    this->syncModul = new SyncModule(this);
}

AlicaEngine::~AlicaEngine() {}

/**
 * Initialise the engine
 * @param bc A behaviourcreator
 * @param roleSetName A string, the roleset to be used. If empty, a default roleset is looked for
 * @param masterPlanName A string, the top-level plan to be used
 * @param roleSetDir A string, the directory in which to search for roleSets. If empty, the base role path will be used.
 * @param stepEngine A bool, whether or not the engine should start in stepped mode
 * @return bool true if everything worked false otherwise
 */
bool AlicaEngine::init(IBehaviourCreator* bc, IConditionCreator* cc, IUtilityCreator* uc, IConstraintCreator* crc) {
    if (!this->expressionHandler) {
        this->expressionHandler = new ExpressionHandler(this, cc, uc, crc);
    }

    this->stepCalled = false;
    bool everythingWorked = true;
    everythingWorked &= this->behaviourPool->init(bc);
    this->auth = new AuthorityManager(this);
    this->log = new Logger(this);
    this->roleAssignment->init();
    if (!this->pap) {
        pap = new PartialAssignmentPool();
    }
    if (!planSelector) {
        this->planSelector = new PlanSelector(this, pap);
    }

    this->auth->init();
    this->planBase = new PlanBase(this, this->masterPlan);
    this->expressionHandler->attachAll();
    UtilityFunction::initDataStructures(this);
    this->syncModul->init();
    if (!this->variableSyncModule) {
        this->variableSyncModule = new VariableSyncModule(this);
    }
    if (this->communicator) {
        this->communicator->startCommunication();
    }
    if (this->variableSyncModule) {
        this->variableSyncModule->init();
    }
    return everythingWorked;
}

/**
 * Closes the engine for good.
 */
void AlicaEngine::shutdown() {
    if (this->communicator != nullptr) {
        this->communicator->stopCommunication();
    }
    this->terminating = true;
    this->maySendMessages = false;

    if (this->behaviourPool != nullptr) {
        this->behaviourPool->stopAll();
        delete this->behaviourPool;
        this->behaviourPool = nullptr;
    }

    if (this->planBase != nullptr) {
        this->planBase->stop();
        delete this->planBase;
        this->planBase = nullptr;
    }

    if (this->auth != nullptr) {
        this->auth->close();
        delete this->auth;
        this->auth = nullptr;
    }

    if (this->syncModul != nullptr) {
        this->syncModul->close();
        delete this->syncModul;
        this->syncModul = nullptr;
    }

    if (this->teamObserver != nullptr) {
        this->teamObserver->close();
        delete this->teamObserver;
        this->teamObserver = nullptr;
    }

    if (this->log != nullptr) {
        this->log->close();
        delete this->log;
        this->log = nullptr;
    }

    if (this->planRepository != nullptr) {
        delete this->planRepository;
        this->planRepository = nullptr;
    }

    if (this->planParser != nullptr) {
        delete this->planParser;
        this->planParser = nullptr;
    }

    delete planSelector;
    planSelector = nullptr;

    if (this->pap != nullptr) {
        delete this->pap;
        this->pap = nullptr;
    }

    this->roleSet = nullptr;
    this->masterPlan = nullptr;

    if (this->expressionHandler != nullptr) {
        delete this->expressionHandler;
        this->expressionHandler = nullptr;
    }

    if (this->variableSyncModule != nullptr) {
        delete this->variableSyncModule;
        this->variableSyncModule = nullptr;
    }
    if (this->roleAssignment != nullptr) {
        delete this->roleAssignment;
        this->roleAssignment = nullptr;
    }
}

/**
 * Register with this EngineTrigger to be called after an engine iteration is complete.
 */
void AlicaEngine::iterationComplete() {
    // TODO: implement the trigger function for iteration complete
}

/**
 * Starts the engine.
 */
void AlicaEngine::start() {
    this->planBase->start();
    cout << "AE: Engine started" << endl;
}

void AlicaEngine::setStepCalled(bool stepCalled) {
    this->stepCalled = stepCalled;
}

bool AlicaEngine::getStepCalled() const {
    return this->stepCalled;
}

bool AlicaEngine::getStepEngine() {
    return this->stepEngine;
}

/**
 * Returns the plan repository, which holds the static ALICA program.
 */
PlanRepository* AlicaEngine::getPlanRepository() const {
    return this->planRepository;
}

/**
 * Returns the planselector
 */
PlanSelector* AlicaEngine::getPlanSelector() {
    return this->planSelector;
}
/**
 * Returns the Alica Clock interface
 */
IAlicaClock* AlicaEngine::getIAlicaClock() const {
    return this->alicaClock;
}
void AlicaEngine::setIAlicaClock(IAlicaClock* clock) {
    this->alicaClock = clock;
}

/**
 * Returns the behaviourpool
 */
BehaviourPool* AlicaEngine::getBehaviourPool() {
    return this->behaviourPool;
}

/**
 * Returns the TeamObserver, which handles most communication tasks.
 */
TeamObserver* AlicaEngine::getTeamObserver() const {
    return this->teamObserver;
}
void AlicaEngine::setTeamObserver(TeamObserver* teamObserver) {
    this->teamObserver = teamObserver;
}

/**
 * Gets the SyncModul, which enables synchronized transitions.
 */
SyncModule* AlicaEngine::getSyncModul() {
    return syncModul;
}

void AlicaEngine::setSyncModul(SyncModule* syncModul) {
    this->syncModul = syncModul;
}

/**
 * Gets AuthorityManager, which detects and resolvs conflicts in task allocation.
 */
AuthorityManager* AlicaEngine::getAuth() {
    return auth;
}
void AlicaEngine::setAuth(AuthorityManager* auth) {
    this->auth = auth;
}

/**
 * Gets the RoleAssignment, responsible for allocating roles to robots.
 */
IRoleAssignment* AlicaEngine::getRoleAssignment() {
    return roleAssignment;
}

void AlicaEngine::setRoleAssignment(IRoleAssignment* roleAssignment) {
    this->roleAssignment = roleAssignment;
}

/**
 * Returns the parser which reads ALICAs XML representation
 */
PlanParser* AlicaEngine::getPlanParser() const {
    return planParser;
}

/**
 * Returns the RoleSet in use.
 */
RoleSet* AlicaEngine::getRoleSet() {
    return roleSet;
}

void AlicaEngine::setStepEngine(bool stepEngine) {
    this->stepEngine = stepEngine;
}

/**
 * Abort execution with a message, called if initialization fails.
 * @param msg A string
 */
void AlicaEngine::abort(string msg) const {
    cerr << "ABORT: " << msg << endl;
    exit(EXIT_FAILURE);
}

/**
 * Gets the robot name, either by access the environment variable "ROBOT", or if that isn't set, the hostname.
 * @return The robot name under which the engine operates, a string
 */
string AlicaEngine::getRobotName() const {
    return sc->getHostname();
}

/**
 * Gets the Logger
 */
Logger* AlicaEngine::getLog() {
    return log;
}

void AlicaEngine::setLog(Logger* log) {
    this->log = log;
}

bool AlicaEngine::isTerminating() const {
    return terminating;
}
bool AlicaEngine::isMaySendMessages() const {
    return maySendMessages;
}
void AlicaEngine::setMaySendMessages(bool maySendMessages) {
    this->maySendMessages = maySendMessages;
}

void AlicaEngine::setTerminating(bool terminating) {
    this->terminating = terminating;
}

const IAlicaCommunication* AlicaEngine::getCommunicator() const {
    return communicator;
}

void AlicaEngine::setCommunicator(IAlicaCommunication* communicator) {
    this->communicator = communicator;
}

/**
 * Returns Alica Main clase that manages the current alica state
 */
PlanBase* AlicaEngine::getPlanBase() {
    return planBase;
}

void AlicaEngine::addSolver(int identifier, ISolver* solver) {
    this->solver[identifier] = solver;
}

ISolver* AlicaEngine::getSolver(int identifier) {
    return this->solver[identifier];
}

VariableSyncModule* AlicaEngine::getResultStore() {
    return this->variableSyncModule;
}

void AlicaEngine::setResultStore(VariableSyncModule* resultStore) {
    this->variableSyncModule = resultStore;
}

PartialAssignmentPool* AlicaEngine::getPartialAssignmentPool() const {
    return this->pap;
}

/**
 * Triggers the engine to run one iteration.
 * Attention: This method call is asynchronous to the triggered iteration.
 * So please wait long enough to let the engine do its stuff of its iteration,
 * before you read values, which will be changed by this iteration.
 */
void AlicaEngine::stepNotify() {
    this->setStepCalled(true);
    this->getPlanBase()->getStepModeCV()->notify_all();
}

TeamManager* AlicaEngine::getTeamManager() const {
    return this->teamManager;
}

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing a part of a ROS
 * message and receiving a pointer to a corresponding AgentID object.
 */
const supplementary::AgentID* AlicaEngine::getIDFromBytes(const std::vector<uint8_t>& idByteVector) {
    return this->agentIDManager->getIDFromBytes(idByteVector);
}

}  // namespace alica
