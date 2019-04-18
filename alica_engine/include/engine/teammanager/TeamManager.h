#pragma once


#include <engine/AlicaClock.h>
#include <engine/teammanager/Agent.h>

#include <essentials/AgentIDConstPtr.h>

#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>

namespace alica
{

class AlicaEngine;
class DomainVariable;
class Variable;
class SuccessMarks;

class ActiveAgentIdView;
class ActiveAgentIdIterator;
class ActiveAgentView;
class ActiveAgentIterator;

class TeamManager
{
public:
    using AgentMap = std::map<essentials::AgentIDConstPtr, Agent*>;

    TeamManager(AlicaEngine* engine, bool useConfigForTeam);
    virtual ~TeamManager();

    void init();

    const AgentMap& getAllAgents() const { return _agents; }

    essentials::AgentIDConstPtr getLocalAgentID() const;
    const Agent* getLocalAgent() const { return localAgent; }
    Agent* editLocalAgent() { return localAgent; }

    ActiveAgentIdView getActiveAgentIds() const;
    ActiveAgentView getActiveAgents() const;

    int getTeamSize() const;
    const Agent* getAgentByID(essentials::AgentIDConstPtr agentId) const;

    void setTimeLastMsgReceived(essentials::AgentIDConstPtr agendId, AlicaTime timeLastMsgReceived);
    bool isAgentIgnored(essentials::AgentIDConstPtr agentId) const;
    bool isAgentActive(essentials::AgentIDConstPtr agentId) const;
    void setAgentIgnored(essentials::AgentIDConstPtr, bool) const;
    bool setSuccess(essentials::AgentIDConstPtr agentId, const AbstractPlan* plan, const EntryPoint* entryPoint);
    bool setSuccessMarks(essentials::AgentIDConstPtr agentId, const IdGrp& suceededEps);
    const DomainVariable* getDomainVariable(essentials::AgentIDConstPtr agentId, const std::string& sort) const;

private:
    AlicaTime teamTimeOut;
    Agent* localAgent;
    AlicaEngine* engine;
    AgentMap _agents;
    bool useConfigForTeam;

    void readTeamFromConfig(essentials::SystemConfig* sc);
};

class ActiveAgentBaseIterator : public std::iterator<std::forward_iterator_tag, essentials::AgentIDConstPtr>
{
public:
    ActiveAgentBaseIterator(TeamManager::AgentMap::const_iterator it, const TeamManager::AgentMap& map)
            : _it(it)
            , _map(map)
    {
        toNextValid();
    }
    ActiveAgentBaseIterator& operator++()
    {
        ++_it;
        toNextValid();
        return *this;
    }
    bool operator==(const ActiveAgentBaseIterator& o) const { return _it == o._it; }
    bool operator!=(const ActiveAgentBaseIterator& o) const { return !(*this == o); }

protected:
    void toNextValid()
    {
        while (_it != _map.end()) {
            if (_it->second->isActive()) {
                return;
            }
            ++_it;
        }
    }
    TeamManager::AgentMap::const_iterator _it;
    const TeamManager::AgentMap& _map;
};

class ActiveAgentIdIterator : public ActiveAgentBaseIterator
{
public:
    ActiveAgentIdIterator(TeamManager::AgentMap::const_iterator it, const TeamManager::AgentMap& map)
            : ActiveAgentBaseIterator(it, map)
    {
    }
    essentials::AgentIDConstPtr operator*() const { return _it->first; }
};

class ActiveAgentIterator : public ActiveAgentBaseIterator
{
public:
    ActiveAgentIterator(TeamManager::AgentMap::const_iterator it, const TeamManager::AgentMap& map)
            : ActiveAgentBaseIterator(it, map)
    {
    }
    const Agent* operator*() const { return _it->second; }
};

class ActiveAgentBaseView
{
public:
    ActiveAgentBaseView(const TeamManager::AgentMap& map)
            : _map(map)
    {
    }

protected:
    const TeamManager::AgentMap& _map;
};

class ActiveAgentIdView : public ActiveAgentBaseView
{
public:
    ActiveAgentIdView(const TeamManager::AgentMap& map)
            : ActiveAgentBaseView(map)
    {
    }
    ActiveAgentIdIterator begin() const { return ActiveAgentIdIterator(_map.begin(), _map); }
    ActiveAgentIdIterator end() const { return ActiveAgentIdIterator(_map.end(), _map); }
    int size() const { return std::distance(begin(), end()); }
    bool empty() const { return begin() == end(); }
};

class ActiveAgentView : public ActiveAgentBaseView
{
public:
    ActiveAgentView(const TeamManager::AgentMap& map)
            : ActiveAgentBaseView(map)
    {
    }
    ActiveAgentIterator begin() const { return ActiveAgentIterator(_map.begin(), _map); }
    ActiveAgentIterator end() const { return ActiveAgentIterator(_map.end(), _map); }
    int size() const { return std::distance(begin(), end()); }
    bool empty() const { return begin() == end(); }
};

} /* namespace alica */
