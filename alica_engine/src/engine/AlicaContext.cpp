#include "engine/AlicaContext.h"
#include "engine/AlicaEngine.h"
#include "engine/AlicaTimer.h"
#include "engine/Types.h"
#include "engine/constraintmodul/VariableSyncModule.h"
#include "engine/logging/AlicaDefaultLogger.h"
#include "engine/modelmanagement/factories/BlackboardBlueprintFactory.h"

#include <engine/FileSystem.h>
#include <engine/modelmanagement/Strings.h>
#include <memory>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

namespace alica
{

constexpr int ALICA_VERSION_MAJOR = 1;
constexpr int ALICA_VERSION_MINOR = 0;
constexpr int ALICA_VERSION_PATCH = 0;
constexpr int ALICA_VERSION = (ALICA_VERSION_MAJOR * 10000) + (ALICA_VERSION_MINOR * 100) + ALICA_VERSION_PATCH;
constexpr uint32_t ALICA_CTX_GOOD = 0xaac0ffee;
constexpr uint32_t ALICA_CTX_BAD = 0xdeaddead;
constexpr int ALICA_LOOP_TIME_ESTIMATE = 33; // ms

std::atomic<int> AlicaContext::_processLoggerRefCount{0};

AlicaContext::AlicaContext(const AlicaContextParams& alicaContextParams)
        /* Install a default logger before anything else, so that logs emitted during construction are
           reported - specifically the logs while reading the config. The verbosity is INFO initially
           because the config file has not been parsed yet; the ctor body upgrades it once it has.

           setIfUnset, not set: the logger is process-wide, so a second AlicaContext must not take it
           away from a context that is already running. Only the context that installs it re-configures
           it below, which means that in a multi-context process the logger's verbosity comes from the
           FIRST context's config. */
        : _ownsProcessLogger(AlicaLogger::setIfUnset<AlicaDefaultLogger>(Verbosity::INFO, alicaContextParams.agentName))
        , _alicaContextParams(alicaContextParams)
        , _localAgentName(alicaContextParams.agentName)
        , _configRootNode(initConfig(alicaContextParams.configPaths, alicaContextParams.agentName))
        , _validTag(ALICA_CTX_GOOD)
        , _clock(std::make_unique<AlicaClock>())
        , _globalBlackboard(std::make_shared<Blackboard>(loadGlobalBlackboardBlueprint().get()))
{
    // Counted here rather than in the member-init list so that a throwing member initializer leaves
    // the count balanced: no increment happened, and the destructor will not run.
    ++_processLoggerRefCount;
    if (_ownsProcessLogger) {
        // reset the logger but this time the config is read
        setLogger<AlicaDefaultLogger>();
    }
}

std::unique_ptr<BlackboardBlueprint> AlicaContext::loadGlobalBlackboardBlueprint()
{
    std::string filePath;
    for (const auto& folder : _alicaContextParams.configPaths) {
        if (essentials::FileSystem::findFile(folder, _alicaContextParams.masterPlanName + alica::Strings::plan_extension, filePath)) {
            break;
        }
    }
    if (filePath.empty()) {
        AlicaEngine::abort(LOGNAME, "Master file not found, aborting");
    }
    YAML::Node masterPlanNode;
    try {
        masterPlanNode = YAML::LoadFile(filePath);
    } catch (YAML::BadFile& badFile) {
        AlicaEngine::abort(LOGNAME, "Could not load master file: ", filePath, ", error details- ", badFile.msg, ", aborting");
    }
    auto inheritBlackboard = Factory::getValue<bool>(masterPlanNode, alica::Strings::inheritBlackboard, false);
    if (inheritBlackboard && Factory::isValid(masterPlanNode[alica::Strings::blackboard])) {
        return BlackboardBlueprintFactory::create(masterPlanNode[alica::Strings::blackboard]);
    }
    return BlackboardBlueprintFactory::createEmpty();
}

AlicaContext::~AlicaContext()
{
    if (_initialized) {
        terminate();
    }
    _validTag = ALICA_CTX_BAD;
    // The logger is process-wide, so it must outlive every context that is still using it. Only the
    // last context to go tears it down; destroying it here unconditionally would leave any surviving
    // context logging into a null logger for the rest of its life.
    if (--_processLoggerRefCount == 0) {
        AlicaLogger::destroy();
    }
}

int AlicaContext::init(AlicaCreators& creatorCtx)
{
    AlicaCreators creators(std::move(creatorCtx.conditionCreator), std::move(creatorCtx.utilityCreator), std::move(creatorCtx.constraintCreator),
            std::move(creatorCtx.behaviourCreator), std::move(creatorCtx.planCreator), std::move(creatorCtx.transitionConditionCreator));
    return init(std::move(creators));
}

int AlicaContext::init(AlicaCreators&& creatorCtx, bool delayStart)
{
    if (_initialized) {
        Logging::logWarn(LOGNAME) << "Context already initialized.";
        return -1;
    }

    if (!_communicator) {
        AlicaEngine::abort(LOGNAME, "Communicator not set");
    }
    if (!_timerFactory) {
        AlicaEngine::abort(LOGNAME, "TimerFactory not set");
    }
    if (!_engineTimerFactory) {
        Logging::logWarn(LOGNAME) << "Engine TimerFactory not set, defaulting to AlicaSystemTimerFactory";
        _engineTimerFactory = std::make_unique<alica::AlicaSystemTimerFactory>();
    }

    _engine = std::make_unique<AlicaEngine>(*this, _configRootNode, _alicaContextParams);

    _communicator->startCommunication();

    if (_engine->init(std::move(creatorCtx))) {
        {
            LockedBlackboardRW gbb(*_globalBlackboard);
            gbb.set("agentName", _engine->getLocalAgentName());
            gbb.set("agentId", _engine->getTeamManager().getLocalAgentID());
        }
        if (!delayStart) {
            _engine->start();
        } else {
            Logging::logInfo(LOGNAME) << "engine start delayed";
        }
        _initialized = true;

        Logging::logInfo(LOGNAME) << "context initialized";
        return 0;
    }
    return -1;
}

int AlicaContext::terminate()
{
    if (_communicator) {
        _communicator->stopCommunication();
    }
    if (_engine) {
        _engine->terminate();
        _engine.reset();
    }
    _initialized = false;
    Logging::logInfo(LOGNAME) << "context terminated";
    // TODO: Fix this (add proper return code in engine shutdown)
    return 0;
}

const std::unordered_map<std::string, Verbosity> AlicaContext::_verbosityStringToVerbosityMap = {{"DEBUG", alica::Verbosity::DEBUG},
        {"INFO", alica::Verbosity::INFO}, {"WARNING", alica::Verbosity::WARNING}, {"ERROR", alica::Verbosity::ERROR}, {"FATAL", alica::Verbosity::FATAL}};

bool AlicaContext::isValid() const
{
    return _validTag == ALICA_CTX_GOOD;
}

void AlicaContext::stepEngine()
{
    _engine->stepNotify();
    constexpr const auto timeout = std::chrono::seconds(2);
    auto start = std::chrono::system_clock::now();
    bool stuck = false;
    do {
        _engine->getAlicaClock().sleep(alica::AlicaTime::milliseconds(ALICA_LOOP_TIME_ESTIMATE));
        if (std::chrono::system_clock::now() > start + timeout && !stuck) {
            // avoid spamming by logging only the first step we're stuck
            Logging::logWarn(LOGNAME) << "Got stuck trying to step engine for too long";
            stuck = true;
        }
    } while (!_engine->editPlanBase().isWaiting());
}

AgentId AlicaContext::getLocalAgentId() const
{
    return _engine->getTeamManager().getLocalAgentID();
}

std::string AlicaContext::getLocalAgentName() const
{
    return _localAgentName;
}

VariableSyncModule& AlicaContext::editSyncModule()
{
    return _engine->editResultStore();
}

YAML::Node AlicaContext::initConfig(const std::vector<std::string>& configPaths, const std::string& agentName)
{
    std::string configFileSearchPath, configFilePath;
    for (const auto& folder : configPaths) {
        configFileSearchPath = essentials::FileSystem::combinePaths(folder, agentName);
        if (essentials::FileSystem::findFile(configFileSearchPath, "Alica.yaml", configFilePath)) {
            try {
                return YAML::LoadFile(configFilePath);
            } catch (YAML::BadFile& badFile) {
                Logging::logWarn(LOGNAME) << "Agent wise config file could not be parsed, file: " << configFilePath << ", error details- " << badFile.msg
                                          << ", will continue the search in the next config directory";
            }
        }
    }

    Logging::logInfo(LOGNAME) << "Agent wise config file could not be used, will look for the global config file instead";

    for (const auto& configFileSearchPath : configPaths) {
        if (essentials::FileSystem::findFile(configFileSearchPath, "Alica.yaml", configFilePath)) {
            try {
                return YAML::LoadFile(configFilePath);
            } catch (YAML::BadFile& badFile) {
                Logging::logWarn(LOGNAME) << "Global config file could not be parsed, file: " << configFilePath << ", error details- " << badFile.msg
                                          << ", will continue the search in the next config directory";
            }
        }
    }
    Logging::logError(LOGNAME) << "Could not parse/find the agent wise config file or the global config file";

    return YAML::Node{};
}

void AlicaContext::getVersion(int& major, int& minor, int& patch)
{
    major = ALICA_VERSION_MAJOR;
    minor = ALICA_VERSION_MINOR;
    patch = ALICA_VERSION_PATCH;
}

int AlicaContext::getVersion()
{
    return ALICA_VERSION;
}

void AlicaContext::reloadConfig()
{
    _engine->reloadConfig(_configRootNode);
}

AlicaCommunicationHandlers AlicaContext::getCommunicationHandlers()
{
    return AlicaCommunicationHandlers{[this](std::shared_ptr<SyncTalk> st) {
                                          if (_engine)
                                              _engine->editSyncModul().onSyncTalk(st);
                                      },
            [this](std::shared_ptr<SyncReady> sr) {
                if (_engine)
                    _engine->editSyncModul().onSyncReady(sr);
            },
            [this](const AllocationAuthorityInfo& aai) {
                if (_engine)
                    _engine->editAuth().handleIncomingAuthorityMessage(aai);
            },
            [this](std::shared_ptr<PlanTreeInfo> st) {
                if (_engine)
                    _engine->editTeamObserver().handlePlanTreeInfo(st);
            },
            [this](const SolverResult& sr) {
                if (_engine)
                    _engine->editResultStore().onSolverResult(sr);
            },
            [this](const AgentQuery& pq) {
                if (_engine)
                    _engine->getTeamManager().handleAgentQuery(pq);
            },
            [this](const AgentAnnouncement& pa) {
                if (_engine)
                    _engine->editTeamManager().handleAgentAnnouncement(pa);
            }};
}

ISolverBase& AlicaContext::getSolverBase(const std::type_info& solverType) const
{
    auto cit = _solvers.find(solverType.hash_code());
    assert(cit != _solvers.end());
    return (*(cit->second));
}

const Blackboard& AlicaContext::getGlobalBlackboard() const
{
    return *_globalBlackboard;
}

Blackboard& AlicaContext::editGlobalBlackboard()
{
    return *_globalBlackboard;
}

const std::shared_ptr<Blackboard> AlicaContext::getGlobalBlackboardShared()
{
    return _globalBlackboard;
}

} // namespace alica
