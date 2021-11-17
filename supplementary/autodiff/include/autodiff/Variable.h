#pragma once

#include "Term.h"
#include <alica_solver_interface/Interval.h>
#include <alica_solver_interface/SolverVariable.h>

namespace autodiff
{
class Variable : public Term, public alica::SolverVariable
{
  public:
    static constexpr double maxExpressibleValue = std::numeric_limits<double>::max() / 2;
    static constexpr double minExpressibleValue = std::numeric_limits<double>::lowest() / 2;

    int accept(ITermVisitor* visitor) override;
    void acceptRecursive(ITermVisitor* visitor) override;

    TermPtr aggregateConstants() override;
    TermPtr derivative(VarPtr v) const override;

    std::string toString() const override;
    static void Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim);

    void setVarIdx(int id) { _varIdx = id; }

    virtual EvalFunction getEvalFunction() const override { return &Eval; }
    virtual void fillParameters(Parameter* params) const override { params[0].asIdx = _varIdx; }

    // This is the total range of possible values of this variable
    alica::Interval<double> getRange() const { return _globalRange; }
    alica::Interval<double>& editRange() { return _globalRange; }

  private:
    friend TermHolder;
    Variable(TermHolder* owner, int64_t id);
    alica::Interval<double> _globalRange;
    int _varIdx;
};
} /* namespace autodiff */
