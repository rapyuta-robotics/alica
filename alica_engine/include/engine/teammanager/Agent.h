#pragma once

#include "engine/IAlicaClock.h"

#include <string>
#include <memory>
#include <list>

namespace supplementary {
	class IAgentID;
}

namespace alica
{

class AlicaEngine;
class RobotProperties;
class RobotEngineData;
class TeamManager;
class TeamObserver;
class AbstractPlan;
class EntryPoint;
class Variable;
class SuccessMarks;

class Agent
{
    // allows the TeamManager to call setTimeLastMsgReceived(..)
    friend ::alica::TeamManager;
    friend ::alica::TeamObserver;

  public:
    virtual ~Agent();

    const supplementary::IAgentID *getID();
    std::string getName();
    const RobotProperties *getProperties() const;
    const RobotEngineData *getEngineData() const;
    bool isActive();

  protected:
    Agent(const AlicaEngine *engine, AlicaTime timeout, const supplementary::IAgentID *id);
    Agent(const AlicaEngine *engine, AlicaTime timeout, const supplementary::IAgentID *id, std::string name);

    const supplementary::IAgentID *id;
    std::string name;
    bool active;
    bool local;
    AlicaTime timeout;
    AlicaTime timeLastMsgReceived;
    RobotProperties *properties;
    RobotEngineData *engineData;

    void setLocal(bool local);
    void setTimeLastMsgReceived(AlicaTime timeLastMsgReceived);
    void setSuccess(AbstractPlan* plan, EntryPoint* entryPoint);
    void setSuccessMarks(std::shared_ptr<SuccessMarks> successMarks);
    Variable* getDomainVariable(std::string sort);
    std::shared_ptr<std::list<EntryPoint*>> getSucceededEntryPoints(AbstractPlan* plan);
    bool update();

    const AlicaEngine *engine;
};

} /* namespace alica */
