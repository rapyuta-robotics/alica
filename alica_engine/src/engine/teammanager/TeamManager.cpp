#include "engine/teammanager/TeamManager.h"

#include "engine/AlicaEngine.h"
#include "engine/IRobotIDFactory.h"
#include "engine/collections/RobotProperties.h"

#include <SystemConfig.h>
#include <iostream>
#include <utility>

namespace alica
{

TeamManager::TeamManager(const AlicaEngine *engine, bool useConfigForTeam = true)
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
        int tmpID = (*sc)["Globals"]->tryGet<int>(-1, "Globals", "Team", agentName.c_str(), "ID", NULL);

        std::vector<uint8_t> robotId;

        for(int i = 0; i < sizeof(int); i++) {
        	robotId.push_back( *(((uint8_t*)&tmpID) + i) );
        }

        agent = new Agent(this->engine, this->teamTimeOut,
                          this->engine->getRobotIDFactory()->create(robotId), agentName);
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

std::unique_ptr<std::list<const IRobotID *>> TeamManager::getActiveAgentIDs() const
{
    auto activeAgentIDs = unique_ptr<std::list<const IRobotID *>>(new list<const IRobotID *>());
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

const Agent *TeamManager::getAgentByID(const IRobotID *agentId) const
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

const IRobotID *TeamManager::getLocalAgentID() const
{
    return this->localAgent->getID();
}

void TeamManager::setTimeLastMsgReceived(const IRobotID *robotID, AlicaTime timeLastMsgReceived)
{
    auto mapIter = this->agents.find(robotID);
    if (mapIter != this->agents.end())
    {
        mapIter->second->setTimeLastMsgReceived(timeLastMsgReceived);
    }
    else
    {
        Agent *agent = new Agent(this->engine, this->teamTimeOut, robotID);
        agent->setTimeLastMsgReceived(timeLastMsgReceived);
        auto mapEntry = this->agents.emplace(robotID, agent);
    }
}

bool TeamManager::isAgentActive(const IRobotID *agentId) const
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
 * @param agentId an IRobotID identifying the agent
 */
bool TeamManager::isAgentIgnored(const IRobotID *agentId) const
{
    return std::find(this->ignoredAgents.begin(), this->ignoredAgents.end(), agentId) != this->ignoredAgents.end();
}

void TeamManager::ignoreAgent(const alica::IRobotID *agentId)
{
    if (find(ignoredAgents.begin(), ignoredAgents.end(), agentId) != ignoredAgents.end())
    {
        return;
    }
    this->ignoredAgents.insert(agentId);
}

void TeamManager::unIgnoreAgent(const alica::IRobotID *agentId)
{
    this->ignoredAgents.erase(agentId);
}

bool TeamManager::setSuccess(const IRobotID *agentId, AbstractPlan *plan, EntryPoint *entryPoint)
{
    auto agentEntry = this->agents.find(agentId);
    if (agentEntry != this->agents.end())
    {
        agentEntry->second->setSuccess(plan, entryPoint);
        return true;
    }
    return false;
}

bool TeamManager::setSuccessMarks(const IRobotID *agentId, std::shared_ptr<SuccessMarks> successMarks)
{
	auto agentEntry = this->agents.find(agentId);
	if (agentEntry != this->agents.end())
	{
		agentEntry->second->setSuccessMarks(successMarks);
		return true;
	}
	return false;
}

Variable *TeamManager::getDomainVariable(const IRobotID *agentId, string sort)
{
    auto agentEntry = this->agents.find(agentId);
    if (agentEntry != this->agents.end())
    {
        return agentEntry->second->getDomainVariable(sort);
    }
    return nullptr;
}

} /* namespace alica */
