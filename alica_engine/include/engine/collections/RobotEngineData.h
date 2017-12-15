#pragma once

#include <map>
#include <memory>
#include <typeinfo>
#include <string>

namespace supplementary{
	class AgentID;
}

namespace alica
{
class SuccessMarks;
class Variable;
class AlicaEngine;

using std::string;
using std::map;
using std::shared_ptr;

/**
 * Basic plan execution information relating to a robot within the team.
 */
class RobotEngineData
{
  public:
	RobotEngineData(const AlicaEngine *engine, const supplementary::AgentID *agentId);
    virtual ~RobotEngineData();
    virtual void initDomainVariables();

    shared_ptr<SuccessMarks> getSuccessMarks() const;
    void setSuccessMarks(shared_ptr<SuccessMarks> successMarks);
    void clearSuccessMarks();

    Variable *getDomainVariable(string sort) const;

  protected:
    const AlicaEngine *engine;
    const supplementary::AgentID * agentId;
    /**
     * The SuccessMarks of the robot, indicating which EntryPoints are completed.
     */
    shared_ptr<SuccessMarks> successMarks;
    /**
     * The domain variables (a.k.a. quantified variables) are hold in a map: "X" -> Variable
     */
    map<string, Variable *> domainVariables;
    /**
     * Creates a hopefully unique id, in order to make the variable
     * string "X" (specified in the Plan Designer) unique in the team.
     */
    long makeUniqueId(string s);
};

} /* namespace alica */
