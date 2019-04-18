#pragma once

#include <engine/Types.h>
#include <engine/collections/SuccessMarks.h>

#include <essentials/AgentIDConstPtr.h>

#include <map>
#include <memory>
#include <string>

namespace alica
{
class AlicaEngine;

/**
 * Basic plan execution information relating to a robot within the team.
 */
class RobotEngineData
{
public:
    RobotEngineData(const AlicaEngine* engine, essentials::AgentIDConstPtr agentId);
    ~RobotEngineData();
    void initDomainVariables();

    const SuccessMarks& getSuccessMarks() const { return _successMarks; }
    SuccessMarks& editSuccessMarks() { return _successMarks; }
    void updateSuccessMarks(const IdGrp& succeededEps);
    void clearSuccessMarks();

    const DomainVariable* getDomainVariable(const Variable* templateVar) const;
    const DomainVariable* getDomainVariable(const std::string& name) const;

private:
    /**
     * Creates a hopefully unique id, in order to make the variable
     * string "X" (specified in the Plan Designer) unique in the team.
     */
    int64_t makeUniqueId(const std::string& s) const;

    const AlicaEngine* _engine;
    essentials::AgentIDConstPtr _agentId;
    /**
     * The SuccessMarks of the robot, indicating which EntryPoints are completed.
     */
    SuccessMarks _successMarks;
    /**
     * The domain variables (a.k.a. quantified variables) are held in a map: TemplateVariable -> DomainVariable
     */
    std::map<const Variable*, std::unique_ptr<const DomainVariable>> _domainVariables;
};

} /* namespace alica */
