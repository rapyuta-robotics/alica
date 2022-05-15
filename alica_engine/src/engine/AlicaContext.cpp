#include "engine/AlicaContext.h"
#include "engine/AlicaEngine.h"
#include "engine/Types.h"
#include "engine/constraintmodul/VariableSyncModule.h"

#include <essentials/FileSystem.h>

namespace alica
{

constexpr int ALICA_VERSION_MAJOR = 0;
constexpr int ALICA_VERSION_MINOR = 9;
constexpr int ALICA_VERSION_PATCH = 0;
constexpr int ALICA_VERSION = (ALICA_VERSION_MAJOR * 10000) + (ALICA_VERSION_MINOR * 100) + ALICA_VERSION_PATCH;
constexpr uint32_t ALICA_CTX_GOOD = 0xaac0ffee;
constexpr uint32_t ALICA_CTX_BAD = 0xdeaddead;
constexpr int ALICA_LOOP_TIME_ESTIMATE = 33; // ms

AlicaContext::AlicaContext(const AlicaContextParams alicaContextParams)
        : _validTag(ALICA_CTX_GOOD)
        , _configRootNode(initConfig(alicaContextParams.configPath, alicaContextParams.agentName))
        , _worldModel(nullptr)
        , _alicaContextParams(alicaContextParams)
        , _clock(std::make_unique<AlicaClock>())
{
}

AlicaContext::~AlicaContext()
{
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
    if (_initialized) {
        ALICA_WARNING_MSG("AC: Context already initialized.");
        return -1;
    }

    if (!_communicator) {
        AlicaEngine::abort("AC: Communicator not seted");
    }
    if (!_timerFactory) {
        AlicaEngine::abort("AC: TimerFactory not seted");
    }

    _engine = std::make_unique<AlicaEngine>(*this, _configRootNode, _alicaContextParams);

    if (_communicator) {
        _communicator->startCommunication();
    }

    _engine = std::make_unique<AlicaEngine>(*this, _configRootNode, _alicaContextParams);

    _communicator->startCommunication();

    if (_engine->init(std::move(creatorCtx))) {
        if (!delayStart) {
            _engine->start();
        }
        _initialized = true;

        return 0;
    }
    return -1;
}

int AlicaContext::terminate()
{
    if (_communicator) {
        _communicator->stopCommunication();
    }
    _engine->terminate();
    _initialized = false;
    // TODO: Fix this (add proper return code in engine shutdown)
    return 0;
}

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
    } while (!_engine->getPlanBase().isWaiting());
}

AgentId AlicaContext::getLocalAgentId() const
{
    return _engine->getTeamManager().getLocalAgentID();
}

std::string AlicaContext::getLocalAgentName() const
{
    return _localAgentName;
}

YAML::Node AlicaContext::initConfig(const std::string& configPath, const std::string& agentName)
{
    YAML::Node node;
    std::string configFile;
    try {
        configFile = essentials::FileSystem::combinePaths(configPath, agentName);
        configFile = essentials::FileSystem::combinePaths(configFile, "Alica.yaml");
        node = YAML::LoadFile(configFile);
        return node;
    } catch (YAML::BadFile& badFile) {
        ALICA_WARNING_MSG("AC: Could not parse file: " << configFile << " - " << badFile.msg);
    }

    try {
        configFile = essentials::FileSystem::combinePaths(configPath, "Alica.yaml");
        node = YAML::LoadFile(configFile);
    } catch (YAML::BadFile& badFile) {
        AlicaEngine::abort("AC: Could not parse file: ", configFile + " - " + badFile.msg);
    }

    return node;
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
    return AlicaCommunicationHandlers{[this](std::shared_ptr<SyncTalk> st) { _engine->editSyncModul().onSyncTalk(st); },
            [this](std::shared_ptr<SyncReady> sr) { _engine->editSyncModul().onSyncReady(sr); },
            [this](const AllocationAuthorityInfo& aai) { _engine->editAuth().handleIncomingAuthorityMessage(aai); },
            [this](std::shared_ptr<PlanTreeInfo> st) { _engine->editTeamObserver().handlePlanTreeInfo(st); },
            [this](const SolverResult& sr) { _engine->editResultStore().onSolverResult(sr); },
            [this](const AgentQuery& pq) { _engine->getTeamManager().handleAgentQuery(pq); },
            [this](const AgentAnnouncement& pa) { _engine->editTeamManager().handleAgentAnnouncement(pa); }};
}

} // namespace alica
