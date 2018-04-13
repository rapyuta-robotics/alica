
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

namespace alica {
/**
 * Abort execution with a message, called if initialization fails.
 * @param msg A string
 */
void AlicaEngine::abort(const std::string& msg) {
    std::cerr << "ABORT: " << msg << std::endl;
    exit(EXIT_FAILURE);
}

/**
 * The main class.
 */
AlicaEngine::AlicaEngine(supplementary::AgentIDManager* idManager, const std::string& roleSetName, const std::string& masterPlanName, const std::string& roleSetDir, bool stepEngine)
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
    _maySendMessages = !(*sc)["Alica"]->get<bool>("Alica.SilentStart", NULL);
    this->useStaticRoles = (*sc)["Alica"]->get<bool>("Alica.UseStaticRoles", NULL);
    AssignmentCollection::maxEpsCount = (*this->sc)["Alica"]->get<short>("Alica.MaxEpsPerPlan", NULL);
    AssignmentCollection::allowIdling = (*this->sc)["Alica"]->get<bool>("Alica.AllowIdling", NULL);

    this->planRepository = new PlanRepository();
    this->planParser = new PlanParser(this->planRepository);
    this->masterPlan = this->planParser->parsePlanTree(masterPlanName);
    this->roleSet = this->planParser->parseRoleSet(roleSetName, roleSetDir);
    this->teamManager = new TeamManager(this, true);
    this->teamManager->init();
    this->behaviourPool = new BehaviourPool(this);
    this->teamObserver = new TeamObserver(this);
    if (this->useStaticRoles) {
        this->roleAssignment = new StaticRoleAssignment(this);
    } else {
        AlicaEngine::abort("Unknown RoleAssignment Type!");
    }
    // the communicator is expected to be set before init() is called
    this->roleAssignment->setCommunication(communicator);
    this->syncModul = new SyncModule(this);

    if (!planRepository->verifyPlanBase()) {
        abort("Error in parsed plans.");
    }
    #ifdef AE_DEBUG
    std::cout << "AE: Constructor finished!" << std::endl;
    #endif
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
    RunningPlan::init();
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
    _maySendMessages = false;

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
    std::cout << "AE: Engine started" << std::endl;
}

void AlicaEngine::setStepCalled(bool stepCalled) {
    this->stepCalled = stepCalled;
}

bool AlicaEngine::getStepCalled() const {
    return this->stepCalled;
}

bool AlicaEngine::getStepEngine() const {
    return this->stepEngine;
}

void AlicaEngine::setIAlicaClock(IAlicaClock* clock) {
    this->alicaClock = clock;
}

void AlicaEngine::setTeamObserver(TeamObserver* teamObserver) {
    this->teamObserver = teamObserver;
}

void AlicaEngine::setSyncModul(SyncModule* syncModul) {
    this->syncModul = syncModul;
}

void AlicaEngine::setAuth(AuthorityManager* auth) {
    this->auth = auth;
}

void AlicaEngine::setRoleAssignment(IRoleAssignment* roleAssignment) {
    this->roleAssignment = roleAssignment;
}

void AlicaEngine::setStepEngine(bool stepEngine) {
    this->stepEngine = stepEngine;
}

/**
 * Gets the robot name, either by access the environment variable "ROBOT", or if that isn't set, the hostname.
 * @return The robot name under which the engine operates, a string
 */
std::string AlicaEngine::getRobotName() const {
    return sc->getHostname();
}

void AlicaEngine::setLog(Logger* log) {
    this->log = log;
}

bool AlicaEngine::isTerminating() const {
    return terminating;
}

void AlicaEngine::setMaySendMessages(bool maySendMessages) {
    _maySendMessages = maySendMessages;
}

void AlicaEngine::setCommunicator(IAlicaCommunication* communicator) {
    this->communicator = communicator;
}


void AlicaEngine::setResultStore(VariableSyncModule* resultStore) {
    this->variableSyncModule = resultStore;
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
