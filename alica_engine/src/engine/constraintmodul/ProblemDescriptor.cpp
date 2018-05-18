#include "engine/constraintmodul/ProblemDescriptor.h"

#include <alica_solver_interface/SolverContext.h>
#include <alica_solver_interface/SolverTerm.h>
#include <alica_solver_interface/SolverVariable.h>

#include <iostream>
#include <limits>

namespace alica
{

ProblemDescriptor::ProblemDescriptor(SolverContext* ctx)
    : _context(ctx)
    , _constraint(nullptr)
    , _utility(nullptr)
{
    clear();
}

void ProblemDescriptor::setUtilitySignificanceThreshold(double value)
{
    _utilitySignificanceThreshold = value;
    _setsUtilitySignificanceThreshold = true;
}

void ProblemDescriptor::setUtilitySufficiencyThreshold(double value)
{
    _utilitySufficiencyThreshold = value;
}

const std::vector<Interval<double>>& ProblemDescriptor::getAllRanges()
{
    if (_allRanges.empty()) {
        _allRanges.insert(_allRanges.end(), _staticRanges.begin(), _staticRanges.end());
        for (const AgentSolverVariables& avars : _domainVars) {
            for (const RangedVariable& rav : avars.getVars()) {
                _allRanges.push_back(rav.range);
            }
        }
    }
    return _allRanges;
}

void ProblemDescriptor::clear()
{
    _utilitySignificanceThreshold = 1E-22;
    _utilitySufficiencyThreshold = std::numeric_limits<double>::max();
    _dim = 0;
    _setsUtilitySignificanceThreshold = false;

    _constraint = nullptr;
    _utility = nullptr;

    _staticVars.clear();
    _domainVars.clear();
    _allVars.clear();

    _staticRanges.clear();
    _allRanges.clear();
    _context->clear();
}

void ProblemDescriptor::prepForUsage()
{
    _staticRanges.resize(_staticVars.size(), Interval<double>(SolverVariable::minExpressibleValue, SolverVariable::maxExpressibleValue));
    _allVars.reserve(_dim);
    _allRanges.reserve(_dim);
    _allVars.insert(_allVars.end(), _staticVars.begin(), _staticVars.end());

    for (const AgentSolverVariables& avars : _domainVars) {
        for (const RangedVariable& rav : avars.getVars()) {
            _allVars.push_back(rav.var);
        }
    }
}

} // namespace alica
