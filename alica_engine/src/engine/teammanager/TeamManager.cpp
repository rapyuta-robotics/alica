#include "engine/teammanager/TeamManager.h"

#include "engine/AlicaEngine.h"
#include "engine/IRoleAssignment.h"
#include "engine/Logger.h"
#include "engine/collections/RobotProperties.h"
#include "engine/containers/AgentQuery.h"

#include <alica_common_config/debug_output.h>

#include <limits>
#include <random>
#include <utility>
#include <functional>

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

TeamManager::TeamManager(AlicaEngine* engine, essentials::IdentifierConstPtr agentID)
        : _localAgent(nullptr)
        , _localAgentID(agentID)
        , _engine(engine)
        , _agentAnnouncementTimeInterval(AlicaTime::zero())
        , _timeLastAnnouncement(AlicaTime::zero())
        , _announcementRetries(0)
{
    std::function<void(const YAML::Node& config)> reloadFunctionPtr = std::bind(&TeamManager::reload, this, std::placeholders::_1);
    _engine->subscribe(reloadFunctionPtr);
    reload(_engine->getConfig());
    std::cout << "[TeamManager] Own ID is " << _localAnnouncement.senderID << std::endl;
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
    const std::string localAgentName = _engine->getLocalAgentName();

    if (_localAgentID == nullptr) {
        constexpr auto notAValidID = std::numeric_limits<uint64_t>::max();
        uint64_t id = config["Local"]["ID"].as<uint64_t>(notAValidID);
        if (id != notAValidID) {
            _localAnnouncement.senderID = _engine->getID(id);
        } else {
            _localAnnouncement.senderID = _engine->generateID(DEFAULT_AGENT_ID_SIZE);
            ALICA_DEBUG_MSG("TM: Auto generated id " << _localAnnouncement.senderID);
        }
    } else {
        _localAnnouncement.senderID = _localAgentID;
    }

    std::random_device rd;
    _localAnnouncement.token = rd();
    _localAnnouncement.senderName = localAgentName;
    _localAnnouncement.senderSdk = _engine->getVersion();
    // TODO: add plan hash
    _localAnnouncement.planHash = 0;

    const std::string myRole = config["Local"]["DefaultRole"].as<std::string>();
    const PlanRepository::Accessor<Role>& roles = _engine->getPlanRepository().getRoles();
    for (const Role* role : roles) {
        if (role->getName() == myRole) {
            _localAnnouncement.roleId = role->getId();
        }
    }

    const YAML::Node localConfigNode = config["Local"];
    _localAnnouncement.capabilities.clear(); //remove old CapabilityPairs to prevent duplicates / outdated values

    for (YAML::const_iterator it = localConfigNode.begin(); it != localConfigNode.end(); ++it) {
        std::string key = it->first.as<std::string>();
        YAML::Node value = it->second;
        if (key == "ID" || key == "DefaultRole") {
            continue;
        }
        std::string svalue = localConfigNode[key].as<std::string>();
        _localAnnouncement.capabilities.emplace_back(key, svalue);
    }

    _localAgent = new Agent(_engine, _teamTimeOut, myRole, _localAnnouncement);
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

const Agent* TeamManager::getAgentByID(essentials::IdentifierConstPtr agentId) const
{
    return getAgent(agentId);
}

Agent* TeamManager::getAgent(essentials::IdentifierConstPtr agentId) const
{
    AgentsCache::AgentMap& agents = *_agentsCache.get();
    auto agentEntry = agents.find(agentId);
    if (agentEntry != agents.end()) {
        return agentEntry->second;
    }

    return nullptr;
}

essentials::IdentifierConstPtr TeamManager::getLocalAgentID() const
{
    return _localAgent->getId();
}

void TeamManager::setTimeLastMsgReceived(essentials::IdentifierConstPtr agentId, AlicaTime timeLastMsgReceived)
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setTimeLastMsgReceived(timeLastMsgReceived);
    }
}

bool TeamManager::isAgentActive(essentials::IdentifierConstPtr agentId) const
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
bool TeamManager::isAgentIgnored(essentials::IdentifierConstPtr agentId) const
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        return agent->isIgnored();
    }
    // Ignore agents that are not announced yet
    return true;
}

void TeamManager::setAgentIgnored(essentials::IdentifierConstPtr agentId, const bool ignored) const
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setIgnored(ignored);
    }
}

bool TeamManager::setSuccess(essentials::IdentifierConstPtr agentId, const AbstractPlan* plan, const EntryPoint* entryPoint)
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setSuccess(plan, entryPoint);
        return true;
    }
    return false;
}

bool TeamManager::setSuccessMarks(essentials::IdentifierConstPtr agentId, const IdGrp& suceededEps)
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setSuccessMarks(suceededEps);
        return true;
    }
    return false;
}

const DomainVariable* TeamManager::getDomainVariable(essentials::IdentifierConstPtr agentId, const std::string& sort) const
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

    const Agent* ag = _engine->getTeamManager().getAgent(aq.senderID);
    if (ag && ag->isIgnored()) {
        // if agent is already discovered and ignored
        return;
    }

    // TODO: Add sdk compatibility check with comparing major version numbers
    if (aq.senderSdk != _localAgent->getSdk() || aq.planHash != _localAgent->getPlanHash()) {
        ALICA_WARNING_MSG("TM: Version mismatch ignoring: " << aq.senderID << " sdk: " << aq.senderSdk << " ph: " << aq.planHash);
        return;
    }

    ALICA_DEBUG_MSG("TM: Responding to agent: " << aq.senderID);
    announcePresence();
}

void TeamManager::handleAgentAnnouncement(const AgentAnnouncement& aa)
{
    if (aa.senderID == _localAgent->getId()) {
        if (aa.token != _localAgent->getToken()) {
            // Shall abort ?
            ALICA_ERROR_MSG("Duplicate Agent(" << aa.senderID << ") discovered");
        }
        return;
    }

    if (!_useAutoDiscovery) {
        return;
    }

    // TODO: Add sdk compatibility check with comparing major version numbers
    if (aa.senderSdk != _localAgent->getSdk() || aa.planHash != _localAgent->getPlanHash()) {
        ALICA_WARNING_MSG("TM: Version mismatch ignoring: " << aa.senderID << " sdk: " << aa.senderSdk << " ph: " << aa.planHash);
        return;
    }

    // Check if agent already exists
    Agent* agentInfo = getAgent(aa.senderID);
    if (agentInfo) {
        return;
    }

    std::string agentRole;
    const PlanRepository::Accessor<Role>& roles = _engine->getPlanRepository().getRoles();
    for (const Role* role : roles) {
        if (role->getId() == aa.roleId) {
            agentRole = role->getName();
        }
    }


    agentInfo = new Agent(_engine, _teamTimeOut, agentRole, aa);
    agentInfo->setTimeLastMsgReceived(_engine->getAlicaClock().now());
    _engine->editLog().eventOccurred("New Agent(", aa.senderID, ")");
    if (!_agentsCache.addAgent(agentInfo)) {
        // already existed
        delete agentInfo;
    }
}

void TeamManager::init()
{
    if (_useAutoDiscovery) {
        _timeLastAnnouncement = _engine->getAlicaClock().now();
        announcePresence();
        queryPresence();
    }
}

void TeamManager::announcePresence() const
{
    ALICA_DEBUG_MSG("TM: Announcing presence " << _localAnnouncement.senderID);
    for (int i = 0; i < _announcementRetries; ++i) {
        _engine->getCommunicator().sendAgentAnnouncement(_localAnnouncement);
    }
}

void TeamManager::queryPresence() const
{
    AgentQuery pqMessage;
    pqMessage.senderID = _localAgent->getId();
    pqMessage.senderSdk = _localAgent->getSdk();
    pqMessage.planHash = _localAgent->getPlanHash();
    for (int i = 0; i < _announcementRetries; ++i) {
        _engine->getCommunicator().sendAgentQuery(pqMessage);
    }
}

void TeamManager::tick()
{
    if (!_useAutoDiscovery) {
        return;
    }

    // check whether its time for new announcement
    AlicaTime now = _engine->getAlicaClock().now();
    if (_timeLastAnnouncement + _agentAnnouncementTimeInterval <= now) {
        _timeLastAnnouncement = now;
        announcePresence();
    }
}
} /* namespace alica */
