#pragma once

#include "engine/IAlicaClock.h"

#include <memory>
#include <list>

namespace alica
{

class IRobotID;
class AlicaEngine;
class Agent;
class RobotProperties;
class RobotEngineData;
class AbstractPlan;
class EntryPoint;
class Variable;
class SuccessMarks;

class ITeamManager
{
  public:
    ITeamManager(const AlicaEngine *engine)
        : engine(engine){};
    virtual ~ITeamManager();

    virtual void init() = 0;
    virtual const IRobotID *getLocalAgentID() const = 0;
    virtual std::unique_ptr<std::list<Agent*>> getAllAgents() = 0;
    virtual std::unique_ptr<std::list<Agent*>> getActiveAgents() = 0;
    virtual std::unique_ptr<std::list<const IRobotID *>> getActiveAgentIDs() const = 0;
    virtual std::unique_ptr<std::list<const RobotProperties*>> getActiveAgentProperties() const = 0;
    virtual int getTeamSize() const = 0;

    virtual const Agent* getAgentByID(const IRobotID* agentId) const = 0;
    virtual bool isAgentIgnored(const IRobotID* agentId) const = 0;
    virtual bool isAgentActive(const IRobotID* agentId) const = 0;

    virtual void setTimeLastMsgReceived(const IRobotID *, AlicaTime) = 0;
    virtual bool setSuccess(const IRobotID* agentId, AbstractPlan* plan, EntryPoint* entryPoint) = 0;
    virtual bool setSuccessMarks(const IRobotID *agentId, std::shared_ptr<SuccessMarks> successMarks) = 0;
    virtual Variable* getDomainVariable(const IRobotID* robot, std::string ident) = 0;

    const AlicaEngine *engine;
};
}
