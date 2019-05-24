#include "engine/teammanager/TeamManager.h"

#include "engine/AlicaEngine.h"
#include "engine/IRoleAssignment.h"
#include "engine/Logger.h"
#include "engine/collections/RobotProperties.h"
#include "engine/containers/AgentQuery.h"
#include "essentials/AgentIDFactory.h"
#include <alica_common_config/debug_output.h>

#include <SystemConfig.h>
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

TeamManager::TeamManager(AlicaEngine* engine)
        : _localAgent(nullptr)
        , _engine(engine)
        , _agentAnnouncementTimeInterval(AlicaTime::zero())
        , _timeLastAnnouncement(AlicaTime::zero())
        , _announcementRetries(0)
{
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    _teamTimeOut = AlicaTime::milliseconds(sc["Alica"]->get<unsigned long>("Alica.TeamTimeOut", NULL));
    _useAutoDiscovery = sc["Alica"]->get<bool>("Alica.AutoDiscovery", NULL);
    if (_useAutoDiscovery) {
        _agentAnnouncementTimeInterval = AlicaTime::seconds(sc["Alica"]->get<unsigned long>("Alica.AgentAnnouncementTimeInterval", NULL));
        _announcementRetries = sc["Alica"]->get<int>("Alica.AnnouncementRetries", NULL);
    }
    readSelfFromConfig();
}

TeamManager::~TeamManager() {}

void TeamManager::setTeamTimeout(AlicaTime t)
{
    AgentsCache::AgentMap& agents = *_agentsCache.get();
    _teamTimeOut = t;

    for (auto& agentEntry : agents) {
        agentEntry.second->setTimeout(t);
    }
}

void TeamManager::readSelfFromConfig()
{
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    const std::string localAgentName = _engine->getRobotName();
    int id = sc["Local"]->tryGet<int>(-1, "Local", localAgentName.c_str(), "ID", NULL);
    if (id != -1) {
        _localAnnouncement.senderID = _engine->getId(id);
    } else {
        _localAnnouncement.senderID = _engine->generateId(DEFAULT_AGENT_ID_SIZE);
        ALICA_DEBUG_MSG("tm: Auto generated id " << _localAnnouncement.senderID);
    }

    std::random_device rd;
    _localAnnouncement.token = rd();
    _localAnnouncement.senderName = localAgentName;
    _localAnnouncement.senderSdk = _engine->getVersion();
    // TODO: add plan hash
    _localAnnouncement.planHash = 0;
    const std::string myRole = sc["Local"]->get<std::string>("Local", localAgentName.c_str(), "DefaultRole", NULL);
    const PlanRepository::Accessor<Role>& roles = _engine->getPlanRepository().getRoles();
    for (const Role* role : roles) {
        if (role->getName() == myRole) {
            _localAnnouncement.roleId = role->getId();
        }
    }

    std::shared_ptr<std::vector<std::string>> properties = sc["Local"]->getNames("Local", localAgentName.c_str(), NULL);
    for (const std::string& s : *properties) {
        if (s == "ID" || s == "DefaultRole") {
            continue;
        }

        std::string svalue = sc["Local"]->get<std::string>("Local", localAgentName.c_str(), s.c_str(), NULL);
        _localAnnouncement.capabilities.emplace_back(s, svalue);
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
    AgentsCache::AgentMap& agents = *_agentsCache.get();
    int teamSize = 0;
    for (auto& agentEntry : agents) {
        if (agentEntry.second->isActive()) {
            ++teamSize;
        }
    }
    return teamSize;
}

const Agent* TeamManager::getAgentByID(AgentIDConstPtr agentId) const
{
    return getAgent(agentId);
}

Agent* TeamManager::getAgent(AgentIDConstPtr agentId) const
{
    AgentsCache::AgentMap& agents = *_agentsCache.get();
    auto agentEntry = agents.find(agentId);
    if (agentEntry != agents.end()) {
        return agentEntry->second;
    }

    return nullptr;
}

AgentIDConstPtr TeamManager::getLocalAgentID() const
{
    return _localAgent->getId();
}

void TeamManager::setTimeLastMsgReceived(AgentIDConstPtr agentId, AlicaTime timeLastMsgReceived)
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setTimeLastMsgReceived(timeLastMsgReceived);
    }
}

bool TeamManager::isAgentActive(AgentIDConstPtr agentId) const
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
bool TeamManager::isAgentIgnored(AgentIDConstPtr agentId) const
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        return agent->isIgnored();
    }
    // Ignore agents that are not announced yet
    return true;
}

void TeamManager::setAgentIgnored(AgentIDConstPtr agentId, const bool ignored) const
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setIgnored(ignored);
    }
}

bool TeamManager::setSuccess(AgentIDConstPtr agentId, const AbstractPlan* plan, const EntryPoint* entryPoint)
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setSuccess(plan, entryPoint);
        return true;
    }
    return false;
}

bool TeamManager::setSuccessMarks(AgentIDConstPtr agentId, const IdGrp& suceededEps)
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setSuccessMarks(suceededEps);
        return true;
    }
    return false;
}

const DomainVariable* TeamManager::getDomainVariable(AgentIDConstPtr agentId, const std::string& sort) const
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
        ALICA_WARNING_MSG("tm: Version mismatch ignoring: " << aq.senderID << " sdk: " << aq.senderSdk << " ph: " << aq.planHash);
        return;
    }

    ALICA_DEBUG_MSG("tm: Responding to agent: " << aq.senderID);
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
        ALICA_WARNING_MSG("tm: Version mismatch ignoring: " << aa.senderID << " sdk: " << aa.senderSdk << " ph: " << aa.planHash);
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
    ALICA_DEBUG_MSG("tm: Announcing presence");
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
