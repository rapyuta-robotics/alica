#include "engine/teammanager/TeamManager.h"

#include "engine/ConfigChangeListener.h"
#include "engine/IAlicaCommunication.h"
#include "engine/IRoleAssignment.h"
#include "engine/Logger.h"
#include "engine/PlanRepository.h"
#include "engine/Types.h"
#include "engine/collections/RobotProperties.h"
#include "engine/containers/AgentQuery.h"
#include "engine/logging/LoggingUtil.h"

#include <chrono>
#include <functional>
#include <limits>
#include <random>
#include <utility>

namespace alica
{

namespace
{
constexpr int DEFAULT_AGENT_ID_SIZE = 8;
}

AgentsCache::AgentsCache()
        : _agents(std::make_shared<AgentMap>())
{
}

AgentsCache::~AgentsCache()
{
    for (auto& agentEntry : *_agents) {
        delete agentEntry.second;
    }
}

const std::shared_ptr<AgentsCache::AgentMap>& AgentsCache::get() const
{
    static thread_local std::shared_ptr<alica::AgentsCache::AgentMap> s_agents;
    if (s_agents != _agents) {
        std::lock_guard<std::mutex> guard(_agentsMutex);
        s_agents = _agents;
    }
    return s_agents;
}

bool AgentsCache::addAgent(Agent* agent)
{
    // Mutate agent cache and add new agent
    std::lock_guard<std::mutex> guard(_agentsMutex);
    std::shared_ptr<AgentMap> newAgentsMap = std::make_shared<AgentMap>(*_agents);
    auto ret = newAgentsMap->emplace(agent->getId(), agent);
    _agents = std::move(newAgentsMap);
    return ret.second;
}

TeamManager::TeamManager(ConfigChangeListener& configChangeListener, const ModelManager& modelManager, const PlanRepository& planRepository,
        const IAlicaCommunication& communicator, const AlicaClock& clock, Logger& log, int version, uint64_t masterPlanId, const std::string& localAgentName,
        AgentId agentID)
        : _localAgent(nullptr)
        , _modelManager(modelManager)
        , _planRepository(planRepository)
        , _communicator(communicator)
        , _clock(clock)
        , _log(log)
        , _agentAnnouncementTimeInterval(AlicaTime::zero())
        , _timeLastAnnouncement(AlicaTime::zero())
        , _announcementRetries(0)
        , _version(version)
        , _masterPlanId(masterPlanId)
        , _localAgentName(localAgentName)
        , _localAgentID(agentID)
{
    auto reloadFunctionPtr = std::bind(&TeamManager::reload, this, std::placeholders::_1);
    configChangeListener.subscribe(reloadFunctionPtr);
    reload(configChangeListener.getConfig());
    Logging::LoggingUtil::logInfo() << "[TeamManager] Own ID is " << _localAnnouncement.senderID;
}

TeamManager::~TeamManager() {}

void TeamManager::reload(const YAML::Node& config)
{
    _teamTimeOut = AlicaTime::milliseconds(config["Alica"]["TeamTimeOut"].as<unsigned long>());
    _useAutoDiscovery = config["Alica"]["AutoDiscovery"].as<bool>();
    if (_useAutoDiscovery) {
        _agentAnnouncementTimeInterval = AlicaTime::seconds(config["Alica"]["AgentAnnouncementTimeInterval"].as<unsigned long>());
        _announcementRetries = config["Alica"]["AnnouncementRetries"].as<int>();
    }
    readSelfFromConfig(config);
}

void TeamManager::setTeamTimeout(AlicaTime t)
{
    AgentsCache::AgentMap& agents = *_agentsCache.get();
    _teamTimeOut = t;

    for (auto& agentEntry : agents) {
        agentEntry.second->setTimeout(t);
    }
}

void TeamManager::readSelfFromConfig(const YAML::Node& config)
{
    if (_localAgent) {
        return;
    }

    if (_localAgentID == InvalidAgentID) {
        uint64_t id = config["Local"]["ID"].as<uint64_t>(InvalidAgentID);
        if (id != InvalidAgentID) {
            _localAnnouncement.senderID = id;
        } else {
            _localAnnouncement.senderID = generateID();
            Logging::LoggingUtil::logDebug() << "TM: Auto generated id " << _localAnnouncement.senderID;
        }
    } else {
        _localAnnouncement.senderID = _localAgentID;
    }

    std::random_device rd;
    _localAnnouncement.token = rd();
    _localAnnouncement.senderName = _localAgentName;
    _localAnnouncement.senderSdk = _version;
    _localAnnouncement.planHash = _masterPlanId;

    const std::string myRole = config["Local"]["DefaultRole"].as<std::string>();
    const PlanRepository::Accessor<Role>& roles = _planRepository.getRoles();
    for (const Role* role : roles) {
        if (role->getName() == myRole) {
            _localAnnouncement.roleId = role->getId();
        }
    }

    const YAML::Node localConfigNode = config["Local"];
    for (YAML::const_iterator it = localConfigNode.begin(); it != localConfigNode.end(); ++it) {
        std::string key = it->first.as<std::string>();
        YAML::Node value = it->second;
        if (key == "ID" || key == "DefaultRole") {
            continue;
        }
        std::string svalue = localConfigNode[key].as<std::string>();
        _localAnnouncement.capabilities.emplace_back(key, svalue);
    }

    _localAgent = new Agent(_modelManager, _planRepository, _clock, _teamTimeOut, myRole, _localAnnouncement);
    _localAgent->setLocal(true);
    _agentsCache.addAgent(_localAgent);
}

ActiveAgentIdView TeamManager::getActiveAgentIds() const
{
    return ActiveAgentIdView(_agentsCache.get());
}

ActiveAgentView TeamManager::getActiveAgents() const
{
    return ActiveAgentView(_agentsCache.get());
}

int TeamManager::getTeamSize() const
{
    return ActiveAgentIdView(_agentsCache.get()).size();
}

const Agent* TeamManager::getAgentByID(AgentId agentId) const
{
    return getAgent(agentId);
}

Agent* TeamManager::getAgent(AgentId agentId) const
{
    AgentsCache::AgentMap& agents = *_agentsCache.get();
    auto agentEntry = agents.find(agentId);
    if (agentEntry != agents.end()) {
        return agentEntry->second;
    }

    return nullptr;
}

AgentId TeamManager::getLocalAgentID() const
{
    return _localAgent->getId();
}

void TeamManager::setTimeLastMsgReceived(AgentId agentId, AlicaTime timeLastMsgReceived)
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setTimeLastMsgReceived(timeLastMsgReceived);
    }
}

bool TeamManager::isAgentActive(AgentId agentId) const
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        return agent->isActive();
    }
    return false;
}

/**
 * Checks if an agent is ignored
 * @param agentId an essentials::AgentID identifying the agent
 */
bool TeamManager::isAgentIgnored(AgentId agentId) const
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        return agent->isIgnored();
    }
    // Ignore agents that are not announced yet
    return true;
}

void TeamManager::setAgentIgnored(AgentId agentId, const bool ignored) const
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setIgnored(ignored);
    }
}

bool TeamManager::setSuccess(AgentId agentId, const AbstractPlan* plan, const EntryPoint* entryPoint)
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setSuccess(plan, entryPoint);
        return true;
    }
    return false;
}

bool TeamManager::setSuccessMarks(AgentId agentId, const IdGrp& suceededEps)
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setSuccessMarks(suceededEps);
        return true;
    }
    return false;
}

const DomainVariable* TeamManager::getDomainVariable(AgentId agentId, const std::string& sort) const
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        return agent->getDomainVariable(sort);
    }
    return nullptr;
}

bool TeamManager::updateAgents(AgentGrp& deactivatedAgents)
{
    AgentsCache::AgentMap& agents = *_agentsCache.get();
    bool changedSomeAgent = false;
    for (const auto& agent : agents) {
        bool changedCurrentAgent = agent.second->update();
        if (changedCurrentAgent && !agent.second->isActive()) {
            deactivatedAgents.push_back(agent.second->getId());
        }
        changedSomeAgent |= changedCurrentAgent;
    }
    return changedSomeAgent;
}

void TeamManager::handleAgentQuery(const AgentQuery& aq) const
{
    if (!_useAutoDiscovery || aq.senderID == _localAgent->getId()) {
        return;
    }

    const Agent* ag = getAgent(aq.senderID);
    if (ag && ag->isIgnored()) {
        // if agent is already discovered and ignored
        return;
    }

    // TODO: Add sdk compatibility check with comparing major version numbers
    if (aq.senderSdk != _localAgent->getSdk() || aq.planHash != _localAgent->getPlanHash()) {
        Logging::LoggingUtil::logWarn() << "TM: Version mismatch ignoring: " << aq.senderID << " sdk: " << aq.senderSdk << " ph: " << aq.planHash;
        return;
    }

    Logging::LoggingUtil::logDebug() << "TM: Responding to agent: " << aq.senderID;
    announcePresence();
}

void TeamManager::handleAgentAnnouncement(const AgentAnnouncement& aa)
{
    if (aa.senderID == _localAgent->getId()) {
        if (aa.token != _localAgent->getToken()) {
            // Shall abort ?
            Logging::LoggingUtil::logError() << "Duplicate Agent(" << aa.senderID << ") discovered";
        }
        return;
    }

    if (!_useAutoDiscovery) {
        return;
    }

    // TODO: Add sdk compatibility check with comparing major version numbers
    if (aa.senderSdk != _localAgent->getSdk() || aa.planHash != _localAgent->getPlanHash()) {
        Logging::LoggingUtil::logWarn() << "TM: Version mismatch ignoring: " << aa.senderID << " sdk: " << aa.senderSdk << " ph: " << aa.planHash;
        return;
    }

    // Check if agent already exists
    Agent* agentInfo = getAgent(aa.senderID);
    if (agentInfo) {
        return;
    }

    std::string agentRole;
    const PlanRepository::Accessor<Role>& roles = _planRepository.getRoles();
    for (const Role* role : roles) {
        if (role->getId() == aa.roleId) {
            agentRole = role->getName();
        }
    }

    agentInfo = new Agent(_modelManager, _planRepository, _clock, _teamTimeOut, agentRole, aa);
    agentInfo->setTimeLastMsgReceived(_clock.now());
    _log.eventOccurred("New Agent(", aa.senderID, ")");
    if (!_agentsCache.addAgent(agentInfo)) {
        // already existed
        delete agentInfo;
    }
}

void TeamManager::init()
{
    if (_useAutoDiscovery) {
        _timeLastAnnouncement = _clock.now();
        announcePresence();
        queryPresence();
    }
}

void TeamManager::announcePresence() const
{
    Logging::LoggingUtil::logDebug() << "TM: Announcing presence " << _localAnnouncement.senderID;
    for (int i = 0; i < _announcementRetries; ++i) {
        _communicator.sendAgentAnnouncement(_localAnnouncement);
    }
}

void TeamManager::queryPresence() const
{
    AgentQuery pqMessage;
    pqMessage.senderID = _localAgent->getId();
    pqMessage.senderSdk = _localAgent->getSdk();
    pqMessage.planHash = _localAgent->getPlanHash();
    for (int i = 0; i < _announcementRetries; ++i) {
        _communicator.sendAgentQuery(pqMessage);
    }
}

void TeamManager::tick()
{
    if (!_useAutoDiscovery) {
        return;
    }

    // check whether its time for new announcement
    AlicaTime now = _clock.now();
    if (_timeLastAnnouncement + _agentAnnouncementTimeInterval <= now) {
        _timeLastAnnouncement = now;
        announcePresence();
    }
}

/**
 * Generates random ID.
 * @return The ID
 */

alica::AgentId TeamManager::generateID()
{
    std::random_device device;
    std::uniform_int_distribution<int32_t> distribution(1, std::numeric_limits<int32_t>::max());
    uint64_t id = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    id = (id << 32) | (distribution(device));
    return id;
}

} /* namespace alica */
