#include "TermHolder.h"

#include "Abs.h"
#include "And.h"
#include "Atan2.h"
#include "ConstPower.h"
#include "Constant.h"
#include "ConstraintUtility.h"
#include "Cos.h"
#include "Exp.h"
#include "LTConstraint.h"
#include "LTEConstraint.h"
#include "LinSigmoid.h"
#include "Log.h"
#include "Max.h"
#include "Min.h"
#include "Or.h"
#include "Product.h"
#include "Reification.h"
#include "Sigmoid.h"
#include "Sin.h"
#include "Sum.h"
#include "TermPower.h"
#include "TermPtr.h"
#include "Variable.h"

#include <limits>

namespace autodiff
{

TermHolder::TermHolder()
    : _true(new Constant(1.0, this))
    , _zero(new Constant(0.0, this))
    , _false(new Constant(std::numeric_limits<double>::min(), this))
{
}

TermHolder::TermHolder(TermHolder&& o)
    : _true(std::move(o._true))
    , _zero(std::move(o._zero))
    , _false(std::move(o._false))
    , _vars(std::move(o._vars))
    , _terms(std::move(o._terms))
    , _tape(std::move(o._tape))
{
    for (std::unique_ptr<Term>& t : _terms) {
        t->_owner = this;
    }
    _true->_owner = this;
    _zero->_owner = this;
    _false->_owner = this;
}
TermHolder& TermHolder::operator=(TermHolder&& o)
{
    _true = std::move(o._true);
    _zero = std::move(o._zero);
    _false = std::move(o._false);
    _vars = std::move(o._vars);
    _terms = std::move(o._terms);
    _tape = std::move(o._tape);
    for (std::unique_ptr<Term>& t : _terms) {
        t->_owner = this;
    }
    _true->_owner = this;
    _zero->_owner = this;
    _false->_owner = this;
    return *this;
}

TermHolder::~TermHolder() {}

VarPtr TermHolder::createVariable(int64_t id)
{
    VarPtr ret = new Variable(this, id);
    _vars.push_back(ret);
    handleNewTerm(ret);
    return ret;
}

TermPtr TermHolder::constant(double v)
{
    return createTerm<Constant>(v);
}

TermPtr TermHolder::sum(TermPtr left, TermPtr right)
{
    return createTerm<Sum>(left, right);
}

TermPtr TermHolder::product(TermPtr left, TermPtr right)
{
    return createTerm<Product>(left, right);
}

TermPtr TermHolder::min(TermPtr left, TermPtr right)
{
    return createTerm<Min>(left, right);
}

TermPtr TermHolder::max(TermPtr left, TermPtr right)
{
    return createTerm<Max>(left, right);
}

TermPtr TermHolder::and_(TermPtr left, TermPtr right)
{
    return createTerm<And>(left, right);
}

TermPtr TermHolder::or_(TermPtr left, TermPtr right)
{
    return createTerm<Or>(left, right);
}

TermPtr TermHolder::lessThan(TermPtr left, TermPtr right)
{
    return createTerm<LTConstraint>(left, right);
}

TermPtr TermHolder::lessThanEqual(TermPtr left, TermPtr right)
{
    return createTerm<LTEConstraint>(left, right);
}

TermPtr TermHolder::reify(TermPtr arg)
{
    return createTerm<Reification>(arg);
}

TermPtr TermHolder::abs(TermPtr arg)
{
    return createTerm<Abs>(arg);
}

TermPtr TermHolder::constPower(TermPtr arg, double exponent)
{
    return createTerm<ConstPower>(arg, exponent);
}

TermPtr TermHolder::termPower(TermPtr arg, TermPtr exponent)
{
    return createTerm<TermPower>(arg, exponent);
}

TermPtr TermHolder::sigmoid(TermPtr arg, double steepness)
{
    return createTerm<Sigmoid>(arg, steepness);
}

TermPtr TermHolder::sigmoid(TermPtr arg, TermPtr mid, double steepness)
{
    return sigmoid(arg - mid, steepness);
}

TermPtr TermHolder::linSigmoid(TermPtr arg)
{
    return createTerm<LinSigmoid>(arg);
}

TermPtr TermHolder::sin(TermPtr arg)
{
    return createTerm<Sin>(arg);
}

TermPtr TermHolder::cos(TermPtr arg)
{
    return createTerm<Cos>(arg);
}

TermPtr TermHolder::exp(TermPtr arg)
{
    return createTerm<Exp>(arg);
}

TermPtr TermHolder::log(TermPtr arg)
{
    return createTerm<Log>(arg);
}

TermPtr TermHolder::atan2(TermPtr left, TermPtr right)
{
    return createTerm<Atan2>(left, right);
}

TermPtr TermHolder::constraintUtility(TermPtr constraint, TermPtr utility)
{
    return createTerm<ConstraintUtility>(constraint, utility);
}

void TermHolder::handleNewTerm(TermPtr t)
{
    _terms.emplace_back(t.get());
}

void TermHolder::clear()
{
    _terms.clear();
    _vars.clear();
    _tape = Tape();
}
}
