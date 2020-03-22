#pragma once


#include <engine/AlicaClock.h>
#include <engine/containers/AgentAnnouncement.h>
#include <engine/teammanager/Agent.h>

#include <essentials/IdentifierConstPtr.h>

#include <list>
#include <map>
#include <memory>
#include <string>

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
struct AgentQuery;

// A read optimized cache for multi writer and readers.
class AgentsCache
{
public:
    using AgentMap = std::map<essentials::IdentifierConstPtr, Agent*>;

    AgentsCache();
    ~AgentsCache();
    const std::shared_ptr<AgentMap>& get() const;
    bool addAgent(Agent* agent);

private:
    // TODO: split active and inactive agents in different maps.
    // Most of the operations are only on active agents.
    std::shared_ptr<AgentMap> _agents;
    mutable std::mutex _agentsMutex;
};

class TeamManager
{
public:
    TeamManager(AlicaEngine* engine);
    virtual ~TeamManager();

    essentials::IdentifierConstPtr getLocalAgentID() const;
    const Agent* getLocalAgent() const { return _localAgent; }
    Agent* editLocalAgent() { return _localAgent; }

    ActiveAgentIdView getActiveAgentIds() const;
    ActiveAgentView getActiveAgents() const;

    int getTeamSize() const;
    const Agent* getAgentByID(essentials::IdentifierConstPtr agentId) const;

    void setTimeLastMsgReceived(essentials::IdentifierConstPtr agendId, AlicaTime timeLastMsgReceived);
    bool isAgentIgnored(essentials::IdentifierConstPtr agentId) const;
    bool isAgentActive(essentials::IdentifierConstPtr agentId) const;
    void setAgentIgnored(essentials::IdentifierConstPtr, bool) const;
    bool setSuccess(essentials::IdentifierConstPtr agentId, const AbstractPlan* plan, const EntryPoint* entryPoint);
    bool setSuccessMarks(essentials::IdentifierConstPtr agentId, const IdGrp& suceededEps);
    const DomainVariable* getDomainVariable(essentials::IdentifierConstPtr agentId, const std::string& sort) const;

    void setTeamTimeout(AlicaTime t);
    bool updateAgents(AgentGrp& deactivatedAgents);
    void handleAgentQuery(const AgentQuery& pq) const;
    void handleAgentAnnouncement(const AgentAnnouncement& aa);
    void init();
    void tick();

private:
    void readSelfFromConfig();
    void announcePresence() const;
    void queryPresence() const;
    Agent* getAgent(essentials::IdentifierConstPtr agentId) const;

    AlicaTime _teamTimeOut;
    AlicaTime _agentAnnouncementTimeInterval;
    AlicaTime _timeLastAnnouncement;
    int _announcementRetries;
    AgentAnnouncement _localAnnouncement;

    Agent* _localAgent;
    AgentsCache _agentsCache;
    AlicaEngine* _engine;
    bool _useAutoDiscovery;
};

class ActiveAgentBaseIterator : public std::iterator<std::forward_iterator_tag, essentials::IdentifierConstPtr>
{
public:
    ActiveAgentBaseIterator(AgentsCache::AgentMap::const_iterator it, const AgentsCache::AgentMap& map)
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
    AgentsCache::AgentMap::const_iterator _it;
    const AgentsCache::AgentMap& _map;
};

class ActiveAgentIdIterator : public ActiveAgentBaseIterator
{
public:
    ActiveAgentIdIterator(AgentsCache::AgentMap::const_iterator it, const AgentsCache::AgentMap& map)
            : ActiveAgentBaseIterator(it, map)
    {
    }
    essentials::IdentifierConstPtr operator*() const { return _it->first; }
};

class ActiveAgentIterator : public ActiveAgentBaseIterator
{
public:
    ActiveAgentIterator(AgentsCache::AgentMap::const_iterator it, const AgentsCache::AgentMap& map)
            : ActiveAgentBaseIterator(it, map)
    {
    }
    const Agent* operator*() const { return _it->second; }
};

class ActiveAgentBaseView
{
public:
    ActiveAgentBaseView(const std::shared_ptr<AgentsCache::AgentMap>& map)
            : _map(map)
    {
    }

protected:
    const std::shared_ptr<AgentsCache::AgentMap> _map;
};

class ActiveAgentIdView : public ActiveAgentBaseView
{
public:
    ActiveAgentIdView(const std::shared_ptr<AgentsCache::AgentMap> map)
            : ActiveAgentBaseView(map)
    {
    }
    ActiveAgentIdIterator begin() const { return ActiveAgentIdIterator(_map->begin(), *_map); }
    ActiveAgentIdIterator end() const { return ActiveAgentIdIterator(_map->end(), *_map); }
    int size() const { return std::distance(begin(), end()); }
    bool empty() const { return begin() == end(); }
};

class ActiveAgentView : public ActiveAgentBaseView
{
public:
    ActiveAgentView(const std::shared_ptr<AgentsCache::AgentMap> map)
            : ActiveAgentBaseView(map)
    {
    }
    ActiveAgentIterator begin() const { return ActiveAgentIterator(_map->begin(), *_map); }
    ActiveAgentIterator end() const { return ActiveAgentIterator(_map->end(), *_map); }
    int size() const { return std::distance(begin(), end()); }
    bool empty() const { return begin() == end(); }
};

} /* namespace alica */
