#include "engine/AlicaContext.h"
#include "engine/AlicaEngine.h"

#include <essentials/IdentifierConstPtr.h>

namespace alica
{

constexpr int ALICA_VERSION_MAJOR = 0;
constexpr int ALICA_VERSION_MINOR = 9;
constexpr int ALICA_VERSION_PATCH = 0;
constexpr int ALICA_VERSION = (ALICA_VERSION_MAJOR * 10000) + (ALICA_VERSION_MINOR * 100) + ALICA_VERSION_PATCH;
constexpr uint32_t ALICA_CTX_GOOD = 0xaac0ffee;
constexpr uint32_t ALICA_CTX_BAD = 0xdeaddead;
constexpr int ALICA_LOOP_TIME_ESTIMATE = 33; // ms

AlicaContext::AlicaContext(const std::string& agentName, const std::string& configPath,
                           const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine,
                           const essentials::Identifier& agentID)
        : _validTag(ALICA_CTX_GOOD)
        , _configRootNode(initConfig(configPath, agentName))
        , _configPath(configPath)
        , _engine(std::make_unique<AlicaEngine>(*this, agentName, configPath, roleSetName, masterPlanName, stepEngine, agentID))
        , _clock(std::make_unique<AlicaClock>())
        , _communicator(nullptr)
        , _idManager(std::make_unique<essentials::IDManager>())
{
//    _configRootNode = initConfig(configPath, agentName);
//    _clock = std::make_unique<AlicaClock>();
//    _communicator = nullptr;
//    _idManager = std::make_unique<essentials::IDManager>();
//    _engine = std::make_unique<AlicaEngine>(*this, agentName, configPath, roleSetName, masterPlanName, stepEngine, agentID);
}

AlicaContext::~AlicaContext()
{
    _validTag = ALICA_CTX_BAD;
}

int AlicaContext::init(AlicaCreators& creatorCtx)
{
    if (_initialized) {
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

essentials::IdentifierConstPtr AlicaContext::getLocalAgentId() const
{
    return _engine->getTeamManager().getLocalAgentID();
}

std::string AlicaContext::getLocalAgentName() const
{
    return _localAgentName;
}

void AlicaContext::setLocalAgentName(const std::string& name)
{
    _localAgentName = name;
}

YAML::Node AlicaContext::initConfig(const std::string& configPath, const std::string& agentName)
{
    YAML::Node node;
    try {
        node = YAML::LoadFile(configPath + agentName + "/Alica.yaml");
    } catch (YAML::BadFile& badFile) {
        AlicaEngine::abort("AC: Could not parse file: ", badFile.msg);
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

} // namespace alica