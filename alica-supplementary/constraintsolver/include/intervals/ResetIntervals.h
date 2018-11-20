/*
 * ResetIntervals.h
 *
 *  Created on: Dec 11, 2014
 *      Author: Philipp
 */

#ifndef RESETINTERVALS_H_
#define RESETINTERVALS_H_

#include <autodiff/ITermVisitor.h>
#include <autodiff/TermPtr.h>
#include <autodiff/Types.h>

namespace alica
{
namespace reasoner
{
namespace intervalpropagation
{

class ResetIntervals : public autodiff::ITermVisitor
{
  public:
    ResetIntervals();

    int visit(autodiff::Abs* abs);
    int visit(autodiff::And* and_);
    int visit(autodiff::Atan2* atan2);
    int visit(autodiff::Constant* constant);
    int visit(autodiff::ConstPower* intPower);
    int visit(autodiff::ConstraintUtility* cu);
    int visit(autodiff::Cos* cos);
    int visit(autodiff::Exp* exp);
    int visit(autodiff::LinSigmoid* sigmoid);
    int visit(autodiff::Log* log);
    int visit(autodiff::LTConstraint* constraint);
    int visit(autodiff::LTEConstraint* constraint);
    int visit(autodiff::Max* max);
    int visit(autodiff::Min* min);
    int visit(autodiff::Or* or_);
    int visit(autodiff::Product* product);
    int visit(autodiff::Reification* dis);
    int visit(autodiff::Sigmoid* sigmoid);
    int visit(autodiff::Sin* sin);
    int visit(autodiff::Sum* sum);
    int visit(autodiff::TermPower* power);
    int visit(autodiff::Variable* var);

  private:
    void updateInterval(autodiff::TermPtr t, double min, double max);
};

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */

#endif /* RESETINTERVALS_H_ */
