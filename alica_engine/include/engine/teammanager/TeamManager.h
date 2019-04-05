#pragma once

#include <engine/AgentIDConstPtr.h>
#include <engine/AlicaClock.h>
#include <engine/teammanager/Agent.h>

#include <list>
#include <map>
#include <memory>
#include <shared_mutex>
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
struct AgentQuery;
struct AgentAnnouncement;

// A read optimized cache for multi writer and readers.
class AgentsCache
{
public:
    using AgentMap = std::map<AgentIDConstPtr, Agent*>;

    AgentsCache();
    ~AgentsCache();
    const std::shared_ptr<AgentMap>& get() const;
    void addAgent(Agent* agent);

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

    AgentIDConstPtr getLocalAgentID() const;
    const Agent* getLocalAgent() const { return _localAgent; }
    Agent* editLocalAgent() { return _localAgent; }

    ActiveAgentIdView getActiveAgentIds() const;
    ActiveAgentView getActiveAgents() const;

    int getTeamSize() const;
    const Agent* getAgentByID(AgentIDConstPtr agentId) const;

    void setTimeLastMsgReceived(AgentIDConstPtr agendId, AlicaTime timeLastMsgReceived);
    bool isAgentIgnored(AgentIDConstPtr agentId) const;
    bool isAgentActive(AgentIDConstPtr agentId) const;
    void setAgentIgnored(AgentIDConstPtr, bool) const;
    bool setSuccess(AgentIDConstPtr agentId, const AbstractPlan* plan, const EntryPoint* entryPoint);
    bool setSuccessMarks(AgentIDConstPtr agentId, const IdGrp& suceededEps);
    const DomainVariable* getDomainVariable(AgentIDConstPtr agentId, const std::string& sort) const;

    void setTeamTimeout(AlicaTime t);
    AgentGrp updateAgents(bool changedSomeAgent);
    void handleAgentQuery(const AgentQuery& pq) const;
    void handleAgentAnnouncement(const AgentAnnouncement& aa);
    void init();
    void tick();

private:
    AlicaTime _teamTimeOut;
    AlicaTime _agentAnnouncementTimeInterval;
    AlicaTime _timeLastAnnouncement;
    int _announcementRetries;

    Agent* _localAgent;
    AgentsCache _agentsCache;
    AlicaEngine* _engine;
    bool _useAutoDiscovery;

    void readSelfFromConfig();
    void announcePresence() const;
    void queryPresence() const;
    Agent* getAgent(AgentIDConstPtr agentId) const;
};

class ActiveAgentBaseIterator : public std::iterator<std::forward_iterator_tag, AgentIDConstPtr>
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
    AgentIDConstPtr operator*() const { return _it->first; }
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
