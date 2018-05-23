#pragma once

#include "engine/collections/AgentVariables.h"
#include "supplementary/AgentID.h"
#include <alica_solver_interface/Interval.h>
#include <memory>
#include <utility>
#include <vector>
namespace alica
{
class SolverContext;
class SolverTerm;
class SolverVariable;

class ProblemPart;

class ProblemDescriptor
{
  public:
    explicit ProblemDescriptor(SolverContext* ctx);
    SolverTerm* getConstraint() const { return _constraint; }
    SolverTerm* getUtility() const { return _utility; }
    const std::vector<SolverVariable*>& getStaticVars() const { return _staticVars; }

    const std::vector<AgentSolverVariables>& getDomainVars() const { return _domainVars; }
    std::vector<AgentSolverVariables>& editDomainVars() { return _domainVars; }

    const std::vector<SolverVariable*>& getAllVariables() const { return _allVars; }

    bool isSettingUtilitySignificanceThreshold() const { return _setsUtilitySignificanceThreshold; }
    double getUtilitySignificanceThreshold() const { return _utilitySignificanceThreshold; }
    double getUtilitySufficiencyThreshold() const { return _utilitySufficiencyThreshold; }

    void setConstraint(SolverTerm* value) { _constraint = value; }
    void setUtility(SolverTerm* value) { _utility = value; }

    void setUtilitySignificanceThreshold(double value);
    void setUtilitySufficiencyThreshold(double value);

    ProblemDescriptor(const ProblemDescriptor&) = delete;
    ProblemDescriptor& operator=(const ProblemDescriptor&) = delete;

    SolverContext* getContext() { return _context; }

  private:
    friend ProblemPart;

    void clear();
    void prepForUsage();

    SolverTerm* _constraint;
    SolverTerm* _utility;

    std::vector<SolverVariable*> _staticVars;
    std::vector<AgentSolverVariables> _domainVars;
    std::vector<SolverVariable*> _allVars;

    SolverContext* _context;

    double _utilitySignificanceThreshold; /*<< minimum delta for adapting a better utility */
    double _utilitySufficiencyThreshold;
    int _dim;
    bool _setsUtilitySignificanceThreshold;
};

} // namespace alica
