#pragma once

#include "engine/IAlicaClock.h"

#include <memory>
#include <list>

namespace supplementary {
	class IAgentID;
}

namespace alica
{

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
    virtual const supplementary::IAgentID *getLocalAgentID() const = 0;
    virtual std::unique_ptr<std::list<Agent*>> getAllAgents() = 0;
    virtual std::unique_ptr<std::list<Agent*>> getActiveAgents() = 0;
    virtual std::unique_ptr<std::list<const supplementary::IAgentID *>> getActiveAgentIDs() const = 0;
    virtual std::unique_ptr<std::list<const RobotProperties*>> getActiveAgentProperties() const = 0;
    virtual int getTeamSize() const = 0;

    virtual const Agent* getAgentByID(const supplementary::IAgentID* agentId) const = 0;
    virtual bool isAgentIgnored(const supplementary::IAgentID* agentId) const = 0;
    virtual bool isAgentActive(const supplementary::IAgentID* agentId) const = 0;

    virtual void setTimeLastMsgReceived(const supplementary::IAgentID *, AlicaTime) = 0;
    virtual bool setSuccess(const supplementary::IAgentID* agentId, AbstractPlan* plan, EntryPoint* entryPoint) = 0;
    virtual bool setSuccessMarks(const supplementary::IAgentID *agentId, std::shared_ptr<SuccessMarks> successMarks) = 0;
    virtual Variable* getDomainVariable(const supplementary::IAgentID* robot, std::string ident) = 0;

    const AlicaEngine *engine;
};
}
