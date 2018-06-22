#include "engine/teammanager/TeamManager.h"

#include "engine/AlicaEngine.h"
#include "engine/collections/RobotProperties.h"
#include "supplementary/AgentIDFactory.h"

#include <SystemConfig.h>
#include <iostream>
#include <utility>

namespace alica
{

TeamManager::TeamManager(AlicaEngine* engine, bool useConfigForTeam = true)
        : localAgent(nullptr)
        , useConfigForTeam(useConfigForTeam)
        , engine(engine)
{
}

TeamManager::~TeamManager()
{
    for (auto& agentEntry : _agents) {
        delete agentEntry.second;
    }
}

void TeamManager::init()
{
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    this->teamTimeOut = AlicaTime::milliseconds((*sc)["Alica"]->get<unsigned long>("Alica.TeamTimeOut", NULL));

    if (useConfigForTeam) {
        this->readTeamFromConfig(sc);
    }
}

void TeamManager::readTeamFromConfig(supplementary::SystemConfig* sc)
{
    std::string localAgentName = this->engine->getRobotName();
    std::shared_ptr<std::vector<std::string>> agentNames = (*sc)["Globals"]->getSections("Globals.Team", NULL);

    Agent* agent;
    bool foundSelf = false;
    for (const std::string& agentName : *agentNames) {
        int id = (*sc)["Globals"]->tryGet<int>(-1, "Globals", "Team", agentName.c_str(), "ID", NULL);

        agent = new Agent(this->engine, this->teamTimeOut, this->engine->getID(id), agentName);
        if (!foundSelf && agentName.compare(localAgentName) == 0) {
            foundSelf = true;
            this->localAgent = agent;
            this->localAgent->setLocal(true);
        } else {
            for (auto& agentEntry : _agents) {
                if (*(agentEntry.first) == *(agent->getID())) {
                    AlicaEngine::abort("TM: Two robots with the same ID in Globals.conf. ID: ", agent->getID());
                }
            }
        }
        _agents.emplace(agent->getID(), agent);
    }
    if (!foundSelf) {
        AlicaEngine::abort("TM: Could not find own agent name in Globals Id = " + localAgentName);
    }

    if ((*sc)["Alica"]->get<bool>("Alica.TeamBlackList.InitiallyFull", NULL)) {
        for (auto& agentEntry : _agents) {
            agentEntry.second->setIgnored(true);
        }
    }
}

ActiveAgentIdView TeamManager::getActiveAgentIds() const
{
    return ActiveAgentIdView(_agents);
}

ActiveAgentView TeamManager::getActiveAgents() const
{
    return ActiveAgentView(_agents);
}

std::unique_ptr<std::list<const RobotProperties*>> TeamManager::getActiveAgentProperties() const
{
    auto agentProperties = std::unique_ptr<std::list<const RobotProperties*>>(new std::list<const RobotProperties*>());
    for (auto& agentEntry : _agents) {
        if (agentEntry.second->isActive()) {
            agentProperties->push_back(agentEntry.second->getProperties());
        }
    }
    return std::move(agentProperties);
}

int TeamManager::getTeamSize() const
{
    int teamSize = 0;
    for (auto& agentEntry : _agents) {
        if (agentEntry.second->isActive()) {
            teamSize++;
        }
    }
    return teamSize;
}

const Agent* TeamManager::getAgentByID(AgentIDConstPtr agentId) const
{
    auto agentEntry = _agents.find(agentId);
    if (agentEntry != _agents.end()) {
        return agentEntry->second;
    } else {
        return nullptr;
    }
}

AgentIDConstPtr TeamManager::getLocalAgentID() const
{
    return this->localAgent->getID();
}

void TeamManager::setTimeLastMsgReceived(AgentIDConstPtr robotID, AlicaTime timeLastMsgReceived)
{
    auto mapIter = _agents.find(robotID);
    if (mapIter != _agents.end()) {
        mapIter->second->setTimeLastMsgReceived(timeLastMsgReceived);
    } else {
        // TODO alex robot properties protokoll anstoßen
        Agent* agent = new Agent(this->engine, this->teamTimeOut, robotID);
        agent->setTimeLastMsgReceived(timeLastMsgReceived);
        _agents.emplace(robotID, agent);
    }
}

bool TeamManager::isAgentActive(AgentIDConstPtr agentId) const
{
    auto agentEntry = _agents.find(agentId);
    if (agentEntry != _agents.end()) {
        return agentEntry->second->isActive();
    } else {
        return false;
    }
}

/**
 * Checks if an agent is ignored
 * @param agentId an supplementary::AgentID identifying the agent
 */
bool TeamManager::isAgentIgnored(AgentIDConstPtr agentId) const
{
    auto agentEntry = _agents.find(agentId);
    if (agentEntry != _agents.end()) {
        return agentEntry->second->isIgnored();
    } else {
        return false;
    }
}

void TeamManager::setAgentIgnored(AgentIDConstPtr agentId, const bool ignored) const
{
    auto agentEntry = _agents.find(agentId);
    if (agentEntry != _agents.end()) {
        agentEntry->second->setIgnored(ignored);
    }
}

bool TeamManager::setSuccess(AgentIDConstPtr agentId, const AbstractPlan* plan, const EntryPoint* entryPoint)
{
    auto agentEntry = _agents.find(agentId);
    if (agentEntry != _agents.end()) {
        agentEntry->second->setSuccess(plan, entryPoint);
        return true;
    }
    return false;
}

bool TeamManager::setSuccessMarks(AgentIDConstPtr agentId, std::shared_ptr<SuccessMarks> successMarks)
{
    auto agentEntry = _agents.find(agentId);
    if (agentEntry != _agents.end()) {
        agentEntry->second->setSuccessMarks(successMarks);
        return true;
    }
    return false;
}

const DomainVariable* TeamManager::getDomainVariable(AgentIDConstPtr agentId, const std::string& sort) const
{
    auto agentEntry = _agents.find(agentId);
    if (agentEntry != _agents.end()) {
        return agentEntry->second->getDomainVariable(sort);
    }
    return nullptr;
}

} /* namespace alica */
