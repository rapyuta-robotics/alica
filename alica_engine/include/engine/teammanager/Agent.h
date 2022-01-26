#pragma once

#include "engine/AlicaClock.h"
#include "engine/Types.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/RobotProperties.h"

#include <list>
#include <memory>
#include <string>

namespace alica
{

class AlicaEngine;
class TeamManager;
class TeamObserver;
class AbstractPlan;
class EntryPoint;
class DomainVariable;
class SuccessMarks;
struct AgentAnnouncement;

class Agent
{
    // allows the TeamManager to call setTimeLastMsgReceived(..)
    friend ::alica::TeamManager;
    friend ::alica::TeamObserver;

public:
    ~Agent();

    AgentId getId() const { return _id; }
    const std::string& getName() const { return _name; }
    const RobotProperties& getProperties() const { return _properties; }
    const RobotEngineData& getEngineData() const { return _engineData; }
    uint32_t getToken() const { return _token; }
    uint32_t getSdk() const { return _sdk; }
    uint64_t getPlanHash() const { return _planHash; }
    RobotEngineData& editEngineData() { return _engineData; }
    bool isActive() const { return _active; }
    bool isIgnored() const { return _ignored; }
    void setTimeout(AlicaTime t);

private:
    Agent(const AlicaEngine* engine, AlicaTime timeout, const std::string& defaultRole, const AgentAnnouncement& aa);

    const AlicaEngine* _engine;
    AgentId _id;
    bool _active;
    bool _ignored;
    bool _local;
    AlicaTime _timeout;
    AlicaTime _timeLastMsgReceived;
    RobotProperties _properties;
    RobotEngineData _engineData;
    std::string _name;
    uint32_t _token;
    uint32_t _sdk;
    uint64_t _planHash;

    void setLocal(bool local);
    void setIgnored(const bool ignored) { _ignored = ignored; }
    void setTimeLastMsgReceived(AlicaTime timeLastMsgReceived) { _timeLastMsgReceived = timeLastMsgReceived; }
    void setSuccess(const AbstractPlan* plan, const EntryPoint* entryPoint);
    void setSuccessMarks(const IdGrp& suceededEps);
    const DomainVariable* getDomainVariable(const std::string& sort) const;
    const EntryPointGrp* getSucceededEntryPoints(const AbstractPlan* plan) const;
    bool update();
};

} /* namespace alica */
