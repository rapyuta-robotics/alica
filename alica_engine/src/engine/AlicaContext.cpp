#include "engine/AlicaContext.h"
#include "engine/AlicaEngine.h"
#include "engine/Types.h"

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

AlicaContext::AlicaContext(const AlicaContextParams& alicaContextParams)
        : _validTag(ALICA_CTX_GOOD)
        , _configRootNode(initConfig(alicaContextParams.configPath, alicaContextParams.agentName))
        , _engine(std::make_unique<AlicaEngine>(*this, alicaContextParams.configPath, alicaContextParams.roleSetName, alicaContextParams.masterPlanName,
                  alicaContextParams.stepEngine, alicaContextParams.agentID))
        , _clock(std::make_unique<AlicaClock>())
        , _communicator(nullptr)
        , _worldModel(nullptr)
{
}

AlicaContext::~AlicaContext()
{
    _validTag = ALICA_CTX_BAD;
}

int AlicaContext::init(AlicaCreators& creatorCtx)
{
    if (_initialized) {
        ALICA_WARNING_MSG("AC: Context already initialized.");
        return -1;
    }

    if (_communicator) {
        _communicator->startCommunication();
    }

    if (_engine->init(creatorCtx)) {
        _engine->start();
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
    do {
        _engine->getAlicaClock().sleep(alica::AlicaTime::milliseconds(ALICA_LOOP_TIME_ESTIMATE));
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

} // namespace alica
