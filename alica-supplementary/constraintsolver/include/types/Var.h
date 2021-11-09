#pragma once

#include "types/Assignment.h"

#include <autodiff/Tape.h>
#include <autodiff/TermHolder.h>
#include <autodiff/TermPtr.h>
#include <autodiff/Types.h>

#include <memory>

namespace alica
{
namespace reasoner
{
namespace cnsat
{
class Clause;
class DecisionLevel;
class Watcher;

class Var
{
  public:
    Var(int index, bool prefSign = true);
    ~Var();

    void reset();
    std::shared_ptr<Clause> getReason() const;
    void setReason(std::shared_ptr<Clause> reason);

    void print() const;
    std::string toString() const;

    std::shared_ptr<std::vector<Watcher*>> watchList;
    int index;
    int activity;
    int negActivity;
    bool locked;
    Assignment assignment;
    bool seen;
    bool preferedSign;
    std::shared_ptr<DecisionLevel> decisionLevel;

    int positiveAppearance;
    int negativeAppearance;

    autodiff::TermPtr _term;
    std::shared_ptr<std::vector<std::shared_ptr<std::vector<double>>>> positiveRanges;
    std::shared_ptr<std::vector<std::shared_ptr<std::vector<double>>>> negativeRanges;

    double positiveRangeSize;
    double negativeRangeSize;

    autodiff::Tape _positiveTerm;
    autodiff::Tape _negativeTerm;
    autodiff::Tape* _curTerm;
    void setToPositive()
    {
        if (_curTerm != &_positiveTerm) {
            if (!_positiveTerm.isSet()) {
                _positiveTerm = _term->getOwner()->compileSeparately(_term);
            }
            _curTerm = &_positiveTerm;
        }
    }
    void setToNegative()
    {
        if (_curTerm != &_negativeTerm) {
            if (!_negativeTerm.isSet()) {
                _negativeTerm = _term->getOwner()->compileSeparately(_term->negate());
            }
            _curTerm = &_negativeTerm;
        }
    }

  private:
    std::shared_ptr<Clause> reason;
};

} /* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */
