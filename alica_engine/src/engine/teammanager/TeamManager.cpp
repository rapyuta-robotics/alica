#include "engine/teammanager/TeamManager.h"

#include "engine/AlicaEngine.h"
#include "engine/IRoleAssignment.h"
#include "engine/collections/RobotProperties.h"
#include "engine/containers/AgentAnnouncement.h"
#include "engine/containers/AgentQuery.h"
#include "essentials/AgentIDFactory.h"

#include <SystemConfig.h>
#include <iostream>
#include <utility>

namespace
{
thread_local std::shared_ptr<alica::AgentsCache::AgentMap> s_agents;
}

namespace alica
{

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
    if (s_agents != _agents) {
        std::lock_guard<std::mutex> guard(_agentsMutex);
        s_agents = _agents;
    }
    return s_agents;
}

void AgentsCache::addAgent(Agent* agent)
{
    // Mutate agent cache and add new agent
    std::lock_guard<std::mutex> guard(_agentsMutex);
    std::shared_ptr<AgentMap> newAgentsMap = std::make_shared<AgentMap>(*_agents);
    newAgentsMap->emplace(std::make_pair(agent->getId(), agent));
    _agents = std::move(newAgentsMap);
}

TeamManager::TeamManager(AlicaEngine* engine)
        : _localAgent(nullptr)
        , _engine(engine)
        , _agentAnnouncementTimeInterval(AlicaTime::zero())
        , _timeLastAnnouncement(AlicaTime::zero())
{
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    _teamTimeOut = AlicaTime::milliseconds(sc["Alica"]->get<unsigned long>("Alica.TeamTimeOut", NULL));
    _useAutoDiscovery = sc["Alica"]->get<bool>("Alica.AutoDiscovery", NULL);
    if (_useAutoDiscovery) {
        _agentAnnouncementTimeInterval = AlicaTime::milliseconds(sc["Alica"]->get<unsigned long>("Alica.AgentAnnouncementTimeInterval", NULL));
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
    std::string localAgentName = _engine->getRobotName();
    int id = sc["Local"]->tryGet<int>(-1, "Local", localAgentName.c_str(), "ID", NULL);
    if (id == -1) {
        //@759 TODO auto generate id
    }

    AgentAnnouncement aa;
    aa.senderID = _engine->getId(id);
    aa.senderName = localAgentName;
    aa.role = sc["Local"]->get<std::string>("Local", localAgentName.c_str(), "DefaultRole", NULL);
    // @759: TODO: get plan hash and sdk here
    // aa.planHash =
    // aa.senderSdk =
    // Add capabilities
    std::shared_ptr<std::vector<std::string>> properties = sc["Local"]->getNames("Local", localAgentName.c_str(), NULL);
    for (const std::string& s : *properties) {
        if (s == "ID" || s == "DefaultRole") {
            continue;
        }

        std::string svalue = sc["Local"]->get<std::string>("Local", localAgentName.c_str(), s.c_str(), NULL);
        aa.capabilities.emplace_back(s, svalue);
    }

    _localAgent = new Agent(_engine, _teamTimeOut, aa);
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
    } else {
        return nullptr;
    }
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
        agent->isIgnored();
    }
    return false;
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
    }
    return false;
}

bool TeamManager::setSuccessMarks(AgentIDConstPtr agentId, const IdGrp& suceededEps)
{
    Agent* agent = getAgent(agentId);
    if (agent) {
        agent->setSuccessMarks(suceededEps);
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

AgentGrp TeamManager::updateAgents(bool changedSomeAgent)
{
    AgentsCache::AgentMap& agents = *_agentsCache.get();
    changedSomeAgent = false;
    AgentGrp deactivatedAgentIds;
    for (const auto& agent : agents) {
        bool changedCurrentAgent = agent.second->update();
        if (changedCurrentAgent && !agent.second->isActive()) {
            deactivatedAgentIds.push_back(agent.second->getId());
        }
        changedSomeAgent |= changedCurrentAgent;
    }
    return deactivatedAgentIds;
}

void TeamManager::handleAgentQuery(const AgentQuery& pq) const
{
    //@759 check plan hash and sdk version before responding and log query here
    if (!_useAutoDiscovery) {
        return;
    }
    announcePresence();
}

void TeamManager::handleAgentAnnouncement(const AgentAnnouncement& aa)
{
    if (!_useAutoDiscovery) {
        return;
    }

    // Check if agent already exists
    Agent* agentInfo = getAgent(aa.senderID);
    if (agentInfo) {
        return;
    }

    //@759 verify plan hash and sdk version before adding and log query here
    agentInfo = new Agent(_engine, _teamTimeOut, aa);
    _agentsCache.addAgent(agentInfo);
    _engine->editRoleAssignment().update();
    _engine->editRoleAssignment().tick();
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
    //@759 TODO: cache this message
    AgentAnnouncement paMessage;
    paMessage.senderID = _localAgent->getId();
    paMessage.senderName = _localAgent->getName();
    paMessage.senderSdk = _localAgent->getSdk();
    paMessage.planHash = _localAgent->getPlanHash();
    paMessage.role = _localAgent->getProperties().getDefaultRole();
    // @759 TODO: add capabilities
    // paMessage.capabilities =
    for (int i = 0; i < _announcementRetries; ++i) {
        _engine->getCommunicator().sendAgentAnnouncement(paMessage);
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
    if (_timeLastAnnouncement + _agentAnnouncementTimeInterval > now) {
        _timeLastAnnouncement = now;
        announcePresence();
    }
}
} /* namespace alica */
