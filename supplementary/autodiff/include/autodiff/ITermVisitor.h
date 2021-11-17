#pragma once

namespace autodiff
{
class Abs;
class And;
class Atan2;
class Constant;
class ConstPower;
class ConstraintUtility;
class Cos;
class Exp;
class LinSigmoid;
class Log;
class LTConstraint;
class LTEConstraint;
class Max;
class Min;
class Or;
class Product;
class Reification;
class Sigmoid;
class Sin;
class Sum;
class TermPower;
class Variable;

class ITermVisitor
{
  public:
    virtual ~ITermVisitor() {}

    virtual int visit(Abs* elem) = 0;
    virtual int visit(And* elem) = 0;
    virtual int visit(Atan2* elem) = 0;
    virtual int visit(Constant* elem) = 0;
    virtual int visit(ConstPower* elem) = 0;
    virtual int visit(ConstraintUtility* elem) = 0;
    virtual int visit(Cos* elem) = 0;
    virtual int visit(Exp* elem) = 0;
    virtual int visit(LinSigmoid* elem) = 0;
    virtual int visit(Log* elem) = 0;
    virtual int visit(LTConstraint* elem) = 0;
    virtual int visit(LTEConstraint* elem) = 0;
    virtual int visit(Max* elem) = 0;
    virtual int visit(Min* elem) = 0;
    virtual int visit(Or* elem) = 0;
    virtual int visit(Product* elem) = 0;
    virtual int visit(Reification* elem) = 0;
    virtual int visit(Sigmoid* elem) = 0;
    virtual int visit(Sin* elem) = 0;
    virtual int visit(Sum* elem) = 0;
    virtual int visit(TermPower* elem) = 0;
    virtual int visit(Variable* var) = 0;
};

} /* namespace autodiff */
