#pragma once

namespace autodiff
{
class Term;
class TermList;
class TermPtr;
class Variable;
class Tape;

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
class LTEConstraint;
class LTConstraint;
class Max;
class Min;
class Or;
class Product;
class Reification;
class Sigmoid;
class Sin;
class Sum;
class TermPower;

using VarPtr = Variable*;

union Parameter
{
    double asDouble;
    int asIdx;
};

typedef void (*EvalFunction)(const Tape&, const Parameter*, double*, const double*, int);
} // namespace autodiff