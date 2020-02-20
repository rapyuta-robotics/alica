#include <stdio.h>
#include "engine/AlicaContext.h"
#include "engine/AlicaEngine.h"

namespace alica
{

constexpr int ALICA_VERSION_MAJOR = 0;
constexpr int ALICA_VERSION_MINOR = 9;
constexpr int ALICA_VERSION_PATCH = 0;
constexpr int ALICA_VERSION = (ALICA_VERSION_MAJOR * 10000) + (ALICA_VERSION_MINOR * 100) + ALICA_VERSION_PATCH;
constexpr uint32_t ALICA_CTX_GOOD = 0xaac0ffee;
constexpr uint32_t ALICA_CTX_BAD = 0xdeaddead;
constexpr int ALICA_LOOP_TIME_ESTIMATE = 33; // ms

AlicaContext::AlicaContext(const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine, AgentIDConstPtr agentID)
        : _validTag(ALICA_CTX_GOOD)
        , _engine(std::make_unique<AlicaEngine>(*this, roleSetName, masterPlanName, stepEngine, agentID))
        , _clock(std::make_unique<AlicaClock>())
{
}

AlicaContext::~AlicaContext()
{
    _validTag = ALICA_CTX_BAD;
}

int AlicaContext::init(AlicaCreators& creatorCtx)
{
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

bool AlicaContext::isValid()
{
    return _validTag == ALICA_CTX_GOOD;
}

bool AlicaContext::isStateActiveHelper(const RunningPlan* rp, int64_t id)
{
    if (!rp) {
        return false;
    }
    const State* activeState = rp->getActiveState();
    if (activeState && activeState->getId() == id) {
        return true;
    }
    const std::vector<RunningPlan*>& children = rp->getChildren();
    for (int i = 0; i < static_cast<int>(children.size()); ++i) {
        if (isStateActiveHelper(children[i], id)) {
            return true;
        }
    }
    return false;
}

bool AlicaContext::isStateActive(int64_t id) const
{
    // This method should be used only when engine is trigger based
    assert(_engine->getStepEngine());
    return isStateActiveHelper(_engine->getPlanBase().getRootNode(), id);
}

void AlicaContext::stepEngine()
{
    _engine->stepNotify();
    do {
        _engine->getAlicaClock().sleep(alica::AlicaTime::milliseconds(ALICA_LOOP_TIME_ESTIMATE));
    } while (!_engine->getPlanBase().isWaiting());
}

essentials::AgentID AlicaContext::getLocalAgentId() const
{
    return *(_engine->getTeamManager().getLocalAgentID());
}

std::string AlicaContext::getRobotName()
{
    return essentials::SystemConfig::getInstance().getHostname();
}

void AlicaContext::setRobotName(const std::string& name)
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
}