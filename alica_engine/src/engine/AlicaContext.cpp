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

AlicaContext::AlicaContext(const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine,
                           const std::string& fullConfigPath, const essentials::Identifier& agentID)
        : _validTag(ALICA_CTX_GOOD)
{
    initConfig(fullConfigPath);
    _clock = std::make_unique<AlicaClock>();
    _communicator = nullptr;
    _idManager = std::make_unique<essentials::IDManager>();
    _engine = std::make_unique<AlicaEngine>(*this, roleSetName, masterPlanName, stepEngine, agentID);
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

    if (!_configRootNode) {
        initConfig();
    }

    if (_communicator) {
        _communicator->startCommunication();
    }

    if (_engine->init(creatorCtx)) {
        _engine->start();
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

void AlicaContext::initConfig(const std::string configPath)
{
    try {
        _configRootNode = YAML::LoadFile(configPath);
        size_t index = configPath.find_last_of("/");
        if (index == std::string::npos) {
            std::cerr << "AC: Error setting configPath for: " << configPath << std::endl;
            return;
        }
        _configPath = configPath.substr(0, index + 1);
    } catch (YAML::BadFile& badFile) {
        AlicaEngine::abort("AC: Could not parse file: ", badFile.msg);
    }
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

void AlicaContext::setInitialized(bool initialized)
{
    _initialized = initialized;
}

void AlicaContext::reloadAll()
{
    for (auto it = _configChangeListeners.begin(); it != _configChangeListeners.end(); ++it) {
        (*it)->reload(getConfig());
    }
}

void AlicaContext::subscribe(ConfigChangeListener *listener)
{
    _configChangeListeners.push_back(listener);
}

void AlicaContext::subscribe(std::function<void(const YAML::Node& config)> reloadFunctionPtr)
{
    _reloadFunctionPtrs.push_back(reloadFunctionPtr);
};

void AlicaContext::unsubscribe(ConfigChangeListener *listener)
{

    auto it = _configChangeListeners.begin();
    for (; it != _configChangeListeners.end(); ++it) {
        if ((*it) == listener) {
            break;
        }
    }
    if (it != _configChangeListeners.end()) {
        _configChangeListeners.erase(it);
    }
}

} // namespace alica