#pragma once

#include "supplementary/AgentID.h"

#include <memory>
#include <vector>

namespace alica {
class SolverTerm;
class SolverVariable;


class ProblemDescriptor : public ProblemDescriptor {
public:

    std::shared_ptr<SolverTerm> getConstraint() const;
    std::shared_ptr<SolverTerm> getUtility() const;
    const std::vector<SolverVariable>& getStaticVars() const;
    const std::vector<std::pair<double>>& getStaticRanges() const;

    std::vector<std::pair<double>>& editStaticRanges();

    const std::vector<AgentSolverVariables>& getDomainVars() const;

    const std::vector<shared_ptr<SolverVariable>>& getAllVariables() const;
    const std::vector<std::pair<double>>& getAllRanges() const;

    bool getSetsUtilitySignificanceThreshold() const;
    double getUtilitySignificanceThreshold() const;
    double getUtilitySufficiencyThreshold() const;

    void setConstraint(std::shared_ptr<SolverTerm> value);
    void setUtility(std::shared_ptr<SolverTerm> value);



    void setSetsUtilitySignificanceThreshold(bool value);
    void setUtilitySignificanceThreshold(double value);
    void setUtilitySufficiencyThreshold(double value);



private:
    friend ProblemPart;
    ProblemDescriptor();

    int dim;
    static constexpr double minExpressible = -10E29;
    static constexpr double maxExpressible = 10E29;

    double utilitySignificanceThreshold = 1E-22; /*<< minimum delta for adapting a better utility */
    bool setsUtilitySignificanceThreshold;

    std::shared_ptr<SolverTerm> constraint;
    std::shared_ptr<SolverTerm> utility;
    double utilitySufficiencyThreshold;
    shared_ptr<vector<shared_ptr<SolverVariable>>> staticVars;
    shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverVariable>>>>>>> domainVars;
    shared_ptr<vector<shared_ptr<vector<const supplementary::AgentID*>>>> agentsInScope;
    shared_ptr<vector<shared_ptr<SolverVariable>>> allVars;

    std::vector<std::vector<std::pair<double>>> domainRanges;
    std::vector<std::pair<double>> _staticRanges;
};

}  // namespace alica
