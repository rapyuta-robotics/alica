#pragma once

#include "engine/collections/AgentVariables.h"
#include "supplementary/AgentID.h"
#include <memory>
#include <utility>
#include <vector>
namespace alica
{
class SolverTerm;
class SolverVariable;
class ProblemPart;

class ProblemDescriptor
{
  public:
    ProblemDescriptor();
    std::shared_ptr<SolverTerm> getConstraint() const { return _constraint; }
    std::shared_ptr<SolverTerm> getUtility() const { return _utility; }
    const std::vector<std::shared_ptr<SolverVariable>>& getStaticVars() const { return _staticVars; }
    const std::vector<std::pair<double, double>>& getStaticRanges() const { return _staticRanges; }

    std::vector<std::pair<double, double>>& editStaticRanges() { return _staticRanges; }

    const std::vector<AgentSolverVariables>& getDomainVars() const { return _domainVars; }
    std::vector<AgentSolverVariables>& editDomainVars() { return _domainVars; }

    const std::vector<std::shared_ptr<SolverVariable>>& getAllVariables() const { return _allVars; }
    const std::vector<std::pair<double, double>>& getAllRanges();

    bool isSettingUtilitySignificanceThreshold() const { return _setsUtilitySignificanceThreshold; }
    double getUtilitySignificanceThreshold() const { return _utilitySignificanceThreshold; }
    double getUtilitySufficiencyThreshold() const { return _utilitySufficiencyThreshold; }

    void setConstraint(std::shared_ptr<SolverTerm> value);
    void setUtility(std::shared_ptr<SolverTerm> value);

    void setUtilitySignificanceThreshold(double value);
    void setUtilitySufficiencyThreshold(double value);

    ProblemDescriptor(const ProblemDescriptor&) = delete;
    ProblemDescriptor& operator=(const ProblemDescriptor&) = delete;

  private:
    friend ProblemPart;

    void clear();
    void prepForUsage();

    std::shared_ptr<SolverTerm> _constraint;
    std::shared_ptr<SolverTerm> _utility;

    std::vector<std::shared_ptr<SolverVariable>> _staticVars;
    std::vector<AgentSolverVariables> _domainVars;
    std::vector<std::shared_ptr<SolverVariable>> _allVars;

    std::vector<std::pair<double, double>> _staticRanges;
    std::vector<std::pair<double, double>> _allRanges;

    double _utilitySignificanceThreshold; /*<< minimum delta for adapting a better utility */
    double _utilitySufficiencyThreshold;
    int _dim;
    bool _setsUtilitySignificanceThreshold;
};

} // namespace alica
