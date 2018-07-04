#pragma once

#include <engine/AgentIDConstPtr.h>
#include <engine/Types.h>

#include <map>
#include <memory>
#include <string>

namespace alica
{
class AlicaEngine;
class SuccessMarks;

/**
 * Basic plan execution information relating to a robot within the team.
 */
class RobotEngineData
{
public:
    RobotEngineData(const AlicaEngine* engine, AgentIDConstPtr agentId);
    virtual ~RobotEngineData();
    virtual void initDomainVariables();

    std::shared_ptr<SuccessMarks> getSuccessMarks() const { return _successMarks; }
    void setSuccessMarks(std::shared_ptr<SuccessMarks> successMarks);
    void clearSuccessMarks();

    const DomainVariable* getDomainVariable(const Variable* templateVar) const;
    const DomainVariable* getDomainVariable(const std::string& name) const;

protected:
    const AlicaEngine* _engine;
    AgentIDConstPtr _agentId;
    /**
     * The SuccessMarks of the robot, indicating which EntryPoints are completed.
     */
    std::shared_ptr<SuccessMarks> _successMarks;
    /**
     * The domain variables (a.k.a. quantified variables) are held in a map: TemplateVariable -> DomainVariable
     */
    std::map<const Variable*, const DomainVariable*> _domainVariables;
    /**
     * Creates a hopefully unique id, in order to make the variable
     * string "X" (specified in the Plan Designer) unique in the team.
     */
    int64_t makeUniqueId(const std::string& s) const;
};

} /* namespace alica */
