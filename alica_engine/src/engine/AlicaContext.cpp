#include "engine/AlicaContext.h"
#include "engine/AlicaEngine.h"
#include "engine/Types.h"
#include "engine/constraintmodul/VariableSyncModule.h"
#include "engine/logging/AlicaDefaultLogger.h"

#include <engine/FileSystem.h>

namespace alica
{

constexpr int ALICA_VERSION_MAJOR = 0;
constexpr int ALICA_VERSION_MINOR = 9;
constexpr int ALICA_VERSION_PATCH = 0;
constexpr int ALICA_VERSION = (ALICA_VERSION_MAJOR * 10000) + (ALICA_VERSION_MINOR * 100) + ALICA_VERSION_PATCH;
constexpr uint32_t ALICA_CTX_GOOD = 0xaac0ffee;
constexpr uint32_t ALICA_CTX_BAD = 0xdeaddead;
constexpr int ALICA_LOOP_TIME_ESTIMATE = 33; // ms

AlicaContext::AlicaContext(const AlicaContextParams& alicaContextParams)
        : _validTag(ALICA_CTX_GOOD)
        , _configRootNode(initConfig(alicaContextParams.configPaths, alicaContextParams.agentName))
        , _alicaContextParams(alicaContextParams)
        , _clock(std::make_unique<AlicaClock>())
        , _localAgentName(alicaContextParams.agentName)
{
}

AlicaContext::~AlicaContext()
{
    if (_initialized) {
        terminate();
    }
    _validTag = ALICA_CTX_BAD;
}

int AlicaContext::init(AlicaCreators& creatorCtx)
{
    AlicaCreators creators(std::move(creatorCtx.conditionCreator), std::move(creatorCtx.utilityCreator), std::move(creatorCtx.constraintCreator),
            std::move(creatorCtx.behaviourCreator), std::move(creatorCtx.planCreator), std::move(creatorCtx.transitionConditionCreator));
    return init(std::move(creators));
}

int AlicaContext::init(AlicaCreators&& creatorCtx, bool delayStart)
{
    if (!Logging::isInitialized()) {
        setLogger<AlicaDefaultLogger>();
    }

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

    _engine = std::make_unique<AlicaEngine>(*this, _configRootNode, _alicaContextParams);

    _communicator->startCommunication();

    if (_engine->init(std::move(creatorCtx))) {
        LockedBlackboardRW gbb(_globalBlackboard);
        gbb.set("agentName", _engine->getLocalAgentName());
        gbb.set("agentId", _engine->getTeamManager().getLocalAgentID());
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
    do {
        _engine->getAlicaClock().sleep(alica::AlicaTime::milliseconds(ALICA_LOOP_TIME_ESTIMATE));
        if (std::chrono::system_clock::now() > start + timeout) {
            throw std::runtime_error("Got stuck trying to step engine");
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
    return _globalBlackboard;
}

Blackboard& AlicaContext::editGlobalBlackboard()
{
    return _globalBlackboard;
}

} // namespace alica
