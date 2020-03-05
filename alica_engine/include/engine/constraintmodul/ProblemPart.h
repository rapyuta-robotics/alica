#pragma once

#include <engine/Types.h>
#include <engine/collections/AgentVariables.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/constraintmodul/UniqueVarStore.h>

#include <memory>
#include <vector>

namespace alica
{
class Condition;
class RunningPlan;
class DomainVariable;
class ISolverBase;
class SolverContext;

class ProblemPart
{
public:
    ProblemPart(const Condition* con, const RunningPlan* rp);

    bool hasVariable(const DomainVariable* v) const;

    const Condition* getCondition() const { return _condition; }
    const RunningPlan* getRunningPlan() const { return _runningPlan; }
    const std::vector<AgentVariables>& getAllVariables() const { return _vars; }
    const AgentVariables& getVarsOfAgent(essentials::IdentifierConstPtr id) const;

    // TODO: get rid of the shared ptr
    std::shared_ptr<ProblemDescriptor> generateProblemDescriptor(ISolverBase& solver, const UniqueVarStore& uvs, SolverContext* ctx);

    ProblemPart(const ProblemPart&) = delete;
    ProblemPart& operator=(const ProblemPart&) = delete;

    ProblemPart(ProblemPart&& o);
    ProblemPart& operator=(ProblemPart&& o);

private:
    std::vector<AgentVariables> _vars;
    const Condition* _condition;
    const RunningPlan* _runningPlan;
    std::shared_ptr<ProblemDescriptor> _descriptor;
};

} /* namespace alica */
