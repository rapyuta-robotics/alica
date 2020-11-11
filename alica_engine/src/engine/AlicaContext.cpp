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
                           const essentials::Identifier& agentID)
        : _validTag(ALICA_CTX_GOOD)
        , _engine(std::make_unique<AlicaEngine>(*this, roleSetName, masterPlanName, stepEngine, agentID))
        , _clock(std::make_unique<AlicaClock>())
        , _communicator(nullptr)
        , _idManager(std::make_unique<essentials::IDManager>())
{
}

AlicaContext::AlicaContext(const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine,
                           const std::string& fullConfigPath, const essentials::Identifier& agentID)
        : _validTag(ALICA_CTX_GOOD)
//        , _engine(std::make_unique<AlicaEngine>(*this, roleSetName, masterPlanName, stepEngine, agentID))
//        , _clock(std::make_unique<AlicaClock>())
//        , _communicator(nullptr)
//        , _idManager(std::make_unique<essentials::IDManager>())
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
    essentials::SystemConfig::getInstance().shutdown();
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

/**
 * Method is deprecated and will be removed soon. Use
 * getLocalAgentName() instead.
 * @return
 */
std::string AlicaContext::getRobotName()
{
    return getLocalAgentName();
}

std::string AlicaContext::getLocalAgentName()
{
    return essentials::SystemConfig::getInstance().getHostname();
}

void AlicaContext::setLocalAgentName(const std::string& name)
{
    essentials::SystemConfig::getInstance().setHostname(name);
}

void AlicaContext::setRootPath(const std::string& path)
{
    essentials::SystemConfig::getInstance().setRootPath(path);
}

void AlicaContext::setConfigPath(const std::string& path)
{
    essentials::SystemConfig::getInstance().setConfigPath(path);
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

} // namespace alica