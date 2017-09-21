#include "engine/teammanager/TeamManager.h"

#include "engine/AlicaEngine.h"
#include "engine/collections/RobotProperties.h"

#include <SystemConfig.h>
#include <utility>
#include <iostream>

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
        int tmpID = (*sc)["Globals"]->tryGet<int>(-1, "Globals", "Team", name.c_str(), "ID", NULL);
        agent = new Agent(this->engine, this->teamTimeOut,
                          this->engine->getRobotIDFactory()->create((uint8_t *)&tmpID, sizeof(int)), localAgentName);
        if (!foundSelf && agentName.compare(localAgentName) == 0)
        {
            foundSelf = true;
            this->localAgent = agent;
            this->localAgent->setActive(true);
        }
        else
        {
            for (auto& agentEntry : this->agents)
            {
                if (*(agentEntry.first) == *(agent->getID()))
                {
                    stringstream ss;
                    ss << "TO: Found twice Robot ID " << agent->getID() << "in globals team section" << std::endl;
                    this->engine->abort(ss.str());
                }
            }
        }
        this->agents.insert(agent->getID(), agent);
    }
    if (!foundSelf)
    {
        this->engine->abort("TM: Could not find own agent name in Globals Id = " + localAgentName);
    }

    if ((*sc)["Alica"]->get<bool>("Alica.TeamBlackList.InitiallyFull", NULL))
    {
        for (auto &agentPair : this->agents)
        {
            this->ignoredRobots.insert(agentPair.first);
        }
    }
}

void TeamManager::tick()
{
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
        auto mapEntry = this->agents.emplace(robotID, timeLastMsgReceived);
    }
}

} /* namespace alica */
