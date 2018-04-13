#pragma once

#include <map>
#include <memory>
#include <string>

namespace supplementary {
class AgentID;
}

namespace alica {
class SuccessMarks;
class Variable;
class AlicaEngine;

using std::map;
using std::shared_ptr;
using std::string;

/**
 * Basic plan execution information relating to a robot within the team.
 */
class RobotEngineData {
public:
    RobotEngineData(const AlicaEngine* engine, const supplementary::AgentID* agentId);
    virtual ~RobotEngineData();
    virtual void initDomainVariables();

    shared_ptr<SuccessMarks> getSuccessMarks() const;
    void setSuccessMarks(shared_ptr<SuccessMarks> successMarks);
    void clearSuccessMarks();

    const Variable* getDomainVariable(const std::string& sort) const;

protected:
    const AlicaEngine* engine;
    const supplementary::AgentID* agentId;
    /**
     * The SuccessMarks of the robot, indicating which EntryPoints are completed.
     */
    std::shared_ptr<SuccessMarks> successMarks;
    /**
     * The domain variables (a.k.a. quantified variables) are hold in a map: "X" -> Variable
     */
    std::map<std::string, const Variable*> domainVariables;
    /**
     * Creates a hopefully unique id, in order to make the variable
     * string "X" (specified in the Plan Designer) unique in the team.
     */
    int64_t makeUniqueId(const std::string& s) const;
};

} /* namespace alica */
