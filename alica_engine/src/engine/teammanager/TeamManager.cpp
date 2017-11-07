#include "engine/teammanager/TeamManager.h"

#include "engine/AlicaEngine.h"
#include "engine/collections/RobotProperties.h"
#include "supplementary/IAgentIDFactory.h"

#include <SystemConfig.h>
#include <iostream>
#include <utility>

namespace alica
{

TeamManager::TeamManager(AlicaEngine *engine, bool useConfigForTeam = true)
    : ITeamManager(engine)
    , localAgent(nullptr)
    , teamTimeOut(0)
    , useConfigForTeam(useConfigForTeam)
{
}

TeamManager::~TeamManager()
{
    for (auto &agentEntry : this->agents)
    {
        delete agentEntry.second;
    }
}

void TeamManager::init()
{
    supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();
    this->teamTimeOut = (*sc)["Alica"]->get<unsigned long>("Alica.TeamTimeOut", NULL) * 1000000; // ms to ns

    if (useConfigForTeam)
    {
        this->readTeamFromConfig(sc);
    }
}

void TeamManager::readTeamFromConfig(supplementary::SystemConfig *sc)
{
    string localAgentName = this->engine->getRobotName();
    shared_ptr<vector<string>> agentNames = (*sc)["Globals"]->getSections("Globals.Team", NULL);

    Agent *agent;
    bool foundSelf = false;
    for (auto &agentName : *agentNames)
    {
        int id = (*sc)["Globals"]->tryGet<int>(-1, "Globals", "Team", agentName.c_str(), "ID", NULL);

        agent = new Agent(this->engine, this->teamTimeOut, this->engine->getID(id), agentName);
        if (!foundSelf && agentName.compare(localAgentName) == 0)
        {
            foundSelf = true;
            this->localAgent = agent;
            this->localAgent->setLocal(true);
        }
        else
        {
            for (auto &agentEntry : this->agents)
            {
                if (*(agentEntry.first) == *(agent->getID()))
                {
                    this->engine->abort("TM: Two robots with the same ID in Globals.conf. ID: ", agent->getID());
                }
            }
        }
        this->agents.emplace(agent->getID(), agent);
    }
    if (!foundSelf)
    {
        this->engine->abort("TM: Could not find own agent name in Globals Id = " + localAgentName);
    }

    if ((*sc)["Alica"]->get<bool>("Alica.TeamBlackList.InitiallyFull", NULL))
    {
        for (auto &agentPair : this->agents)
        {
            this->ignoredAgents.insert(agentPair.first);
        }
    }
}

std::unique_ptr<std::list<const supplementary::IAgentID *>> TeamManager::getActiveAgentIDs() const
{
    auto activeAgentIDs = unique_ptr<std::list<const supplementary::IAgentID *>>(new list<const supplementary::IAgentID *>());
    for (auto &agentEntry : this->agents)
    {
        if (agentEntry.second->isActive())
        {
            activeAgentIDs->push_back(agentEntry.first);
        }
    }
    return std::move(activeAgentIDs);
}

std::unique_ptr<std::list<Agent *>> TeamManager::getAllAgents()
{
    auto agentList = unique_ptr<std::list<Agent *>>(new list<Agent *>());
    for (auto &agentEntry : this->agents)
    {
        agentList->push_back(agentEntry.second);
    }
    return std::move(agentList);
}

std::unique_ptr<std::list<Agent *>> TeamManager::getActiveAgents()
{
    auto agentList = unique_ptr<std::list<Agent *>>(new list<Agent *>());
    for (auto &agentEntry : this->agents)
    {
        if (agentEntry.second->isActive())
        {
            agentList->push_back(agentEntry.second);
        }
    }
    return std::move(agentList);
}

std::unique_ptr<std::list<const RobotProperties *>> TeamManager::getActiveAgentProperties() const
{
    auto agentProperties = unique_ptr<std::list<const RobotProperties *>>(new list<const RobotProperties *>());
    for (auto &agentEntry : this->agents)
    {
        if (agentEntry.second->isActive())
        {
            agentProperties->push_back(agentEntry.second->getProperties());
        }
    }
    return std::move(agentProperties);
}

int TeamManager::getTeamSize() const
{
    int teamSize = 0;
    for (auto &agentEntry : this->agents)
    {
        if (agentEntry.second->isActive())
        {
            teamSize++;
        }
    }
    return teamSize;
}

const Agent *TeamManager::getAgentByID(const supplementary::IAgentID *agentId) const
{
    auto agentEntry = this->agents.find(agentId);
    if (agentEntry != this->agents.end())
    {
        return agentEntry->second;
    }
    else
    {
        return nullptr;
    }
}

const supplementary::IAgentID *TeamManager::getLocalAgentID() const
{
    return this->localAgent->getID();
}

void TeamManager::setTimeLastMsgReceived(const supplementary::IAgentID *robotID, AlicaTime timeLastMsgReceived)
{
    auto mapIter = this->agents.find(robotID);
    if (mapIter != this->agents.end())
    {
        mapIter->second->setTimeLastMsgReceived(timeLastMsgReceived);
    }
    else
    {
        // TODO alex robot properties protokoll anstoßen
        Agent *agent = new Agent(this->engine, this->teamTimeOut, robotID);
        agent->setTimeLastMsgReceived(timeLastMsgReceived);
        auto mapEntry = this->agents.emplace(robotID, agent);
    }
}

bool TeamManager::isAgentActive(const supplementary::IAgentID *agentId) const
{
    auto agentEntry = this->agents.find(agentId);
    if (agentEntry != this->agents.end())
    {
        return agentEntry->second->isActive();
    }
    else
    {
        return false;
    }
}

/**
 * Checks if an agent is ignored
 * @param agentId an supplementary::IAgentID identifying the agent
 */
bool TeamManager::isAgentIgnored(const supplementary::IAgentID *agentId) const
{
    return std::find_if(this->ignoredAgents.begin(), this->ignoredAgents.end(), [&agentId](const supplementary::IAgentID *id) { return *agentId == *id; }) !=
           this->ignoredAgents.end();
}

void TeamManager::ignoreAgent(const supplementary::IAgentID *agentId)
{
    if (find_if(ignoredAgents.begin(), ignoredAgents.end(), [&agentId](const supplementary::IAgentID *id) { return *agentId == *id; }) != ignoredAgents.end())
    {
        return;
    }
    this->ignoredAgents.insert(agentId);
}

void TeamManager::unIgnoreAgent(const supplementary::IAgentID *agentId)
{
    this->ignoredAgents.erase(agentId);
}

bool TeamManager::setSuccess(const supplementary::IAgentID *agentId, AbstractPlan *plan, EntryPoint *entryPoint)
{
    auto agentEntry = this->agents.find(agentId);
    if (agentEntry != this->agents.end())
    {
        agentEntry->second->setSuccess(plan, entryPoint);
        return true;
    }
    return false;
}

bool TeamManager::setSuccessMarks(const supplementary::IAgentID *agentId, std::shared_ptr<SuccessMarks> successMarks)
{
    auto agentEntry = this->agents.find(agentId);
    if (agentEntry != this->agents.end())
    {
        agentEntry->second->setSuccessMarks(successMarks);
        return true;
    }
    return false;
}

Variable *TeamManager::getDomainVariable(const supplementary::IAgentID *agentId, string sort)
{
    auto agentEntry = this->agents.find(agentId);
    if (agentEntry != this->agents.end())
    {
        return agentEntry->second->getDomainVariable(sort);
    }
    return nullptr;
}

} /* namespace alica */
