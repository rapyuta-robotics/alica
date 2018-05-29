#pragma once

#include "ITermVisitor.h"
#include "TermPtr.h"
#include "Types.h"
#include <alica_solver_interface/Interval.h>
#include <alica_solver_interface/SolverTerm.h>

#include <string>
#include <vector>

namespace autodiff
{

class TermHolder;
class TermList;

enum AndType
{
    MIN,
    AND
};
enum OrType
{
    MAX,
    OR
};

class Term : public alica::SolverTerm
{
  public:
    virtual ~Term();

    /**
     * Accepts a term visitor
     *
     * @param visitor The term visitor to accept
     */
    virtual void acceptRecursive(ITermVisitor* visitor) = 0;
    virtual int accept(ITermVisitor* visitor) = 0;

    static AndType getAnd();
    static void setAnd(AndType a);
    static OrType getOr();
    static void setOr(OrType o);
    static inline constexpr double getConstraintSteepness() { return CONSTRAINTSTEEPNESS; }

    virtual TermPtr aggregateConstants() = 0;
    virtual TermPtr derivative(VarPtr v) const = 0;
    virtual TermPtr negate() const;

    virtual std::string toString() const = 0;
    virtual bool isConstant() const { return false; }

    TermHolder* getOwner() const { return _owner; }

    void setTapeIdx(int idx) { _tapeIdx = idx; }
    int getTapeIdx() const { return _tapeIdx; }
    virtual EvalFunction getEvalFunction() const = 0;
    virtual void fillParameters(Parameter* params) const = 0;

    alica::Interval<double> getLocalRange() const { return _localRange; }
    alica::Interval<double>& editLocalRange() { return _localRange; }
    double getMax() const { return _localRange.getMax(); }
    double getMin() const { return _localRange.getMin(); }

    void setMax(double m) { _localRange.setMax(m); }
    void setMin(double m) { _localRange.setMin(m); }

    const std::vector<TermPtr>& getParents() const { return _parents; }
    std::vector<TermPtr>& editParents() { return _parents; }

  protected:
    static constexpr double EPSILON = 1e-10;
    static constexpr double CONSTRAINTSTEEPNESS = 0.01;
    friend TermList;
    friend TermHolder;
    Term(TermHolder* owner);
    // backptr to owner object:
    TermHolder* _owner;
    // intrusive list for tape generation & interval propagation
    TermPtr _next;
    // index to tape representation
    int _tapeIdx;
    // intentional padding
    // DAG backptrs for interval propgation
    std::vector<TermPtr> _parents;
    // Interval values
    alica::Interval<double> _localRange;

  private:
    static OrType _orop;
    static AndType _andop;
};

TermPtr operator+(const TermPtr left, const TermPtr right);
TermPtr operator*(const TermPtr left, const TermPtr right);
TermPtr operator/(const TermPtr numerator, const TermPtr denominator);
TermPtr operator-(const TermPtr left, const TermPtr right);

TermPtr operator+(const double left, const TermPtr right);
TermPtr operator*(const double left, const TermPtr right);
TermPtr operator/(const double numerator, const TermPtr denominator);
TermPtr operator-(const double left, const TermPtr right);

TermPtr operator+(const TermPtr left, const double right);
TermPtr operator*(const TermPtr left, const double right);
TermPtr operator/(const TermPtr numerator, const double denominator);
TermPtr operator-(const TermPtr left, const double right);

TermPtr operator-(const TermPtr term);

TermPtr operator!(const TermPtr term);
TermPtr operator&(const TermPtr left, const TermPtr right);
TermPtr operator|(const TermPtr left, const TermPtr right);

TermPtr operator>(const TermPtr left, const TermPtr right);
TermPtr operator<(const TermPtr left, const TermPtr right);
TermPtr operator<=(const TermPtr left, const TermPtr right);
TermPtr operator>=(const TermPtr left, const TermPtr right);

TermPtr operator&=(const TermPtr left, const TermPtr right);

} /* namespace autodiff */
