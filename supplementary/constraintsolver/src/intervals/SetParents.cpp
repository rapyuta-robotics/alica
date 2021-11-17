
#include "intervals/SetParents.h"

#include <autodiff/AutoDiff.h>

//#define SetParentsDEBUG

namespace alica
{
namespace reasoner
{
namespace intervalpropagation
{

using namespace autodiff;

SetParents::SetParents() {}

SetParents::~SetParents() {}

int SetParents::visit(Abs* abs)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Abs* abs)" << std::endl;
#endif
    abs->getArg()->editParents().push_back(abs);
    return false;
}

int SetParents::visit(And* and_)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(And* and_)" << std::endl;
#endif
    and_->getLeft()->editParents().push_back(and_);
    and_->getRight()->editParents().push_back(and_);
    return false;
}

int SetParents::visit(Atan2* atan2)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Atan2* atan2)" << std::endl;
#endif
    atan2->getLeft()->editParents().push_back(atan2);
    atan2->getRight()->editParents().push_back(atan2);
    return false;
}

int SetParents::visit(Constant* constant)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Constant* constant)" << std::endl;
#endif
    return false;
}

int SetParents::visit(ConstPower* intPower)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(ConstPower* intPower)" << std::endl;
#endif
    intPower->getBase()->editParents().push_back(intPower);
    return false;
}

int SetParents::visit(ConstraintUtility* cu)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(ConstraintUtility* cu)" << std::endl;
#endif
    cu->getLeft()->editParents().push_back(cu);
    cu->getRight()->editParents().push_back(cu);
    return false;
}

int SetParents::visit(Cos* cos)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Cos* cos)" << std::endl;
#endif
    cos->getArg()->editParents().push_back(cos);
    return false;
}

int SetParents::visit(Exp* exp)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Exp* exp)" << std::endl;
#endif
    exp->getArg()->editParents().push_back(exp);
    return false;
}

int SetParents::visit(LinSigmoid* sigmoid)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(LinSigmoid* sigmoid)" << std::endl;
#endif
    sigmoid->getArg()->editParents().push_back(sigmoid);
    return false;
}

int SetParents::visit(Log* log)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Log* log)" << std::endl;
#endif
    log->getArg()->editParents().push_back(log);
    return false;
}

int SetParents::visit(LTConstraint* constraint)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(LTConstraint* constraint)" << std::endl;
#endif
    constraint->getLeft()->editParents().push_back(constraint);
    constraint->getRight()->editParents().push_back(constraint);
    return false;
}

int SetParents::visit(LTEConstraint* constraint)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(LTEConstraint* constraint)" << std::endl;
#endif
    constraint->getLeft()->editParents().push_back(constraint);
    constraint->getRight()->editParents().push_back(constraint);
    return false;
}

int SetParents::visit(Max* max)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Max* max)" << std::endl;
#endif
    max->getLeft()->editParents().push_back(max);
    max->getRight()->editParents().push_back(max);
    return false;
}

int SetParents::visit(Min* min)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Min* min)" << std::endl;
#endif
    min->getLeft()->editParents().push_back(min);
    min->getRight()->editParents().push_back(min);
    return false;
}

int SetParents::visit(Or* or_)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Or* or_)" << std::endl;
#endif
    or_->getLeft()->editParents().push_back(or_);
    or_->getRight()->editParents().push_back(or_);
    return false;
}

int SetParents::visit(Product* product)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Product* product)" << std::endl;
#endif
    product->getLeft()->editParents().push_back(product);
    product->getRight()->editParents().push_back(product);
    return false;
}

int SetParents::visit(Reification* reif)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Reification* reif)" << std::endl;
#endif
    reif->getLeft()->editParents().push_back(reif);
    reif->getRight()->editParents().push_back(reif);
    return false;
}

int SetParents::visit(Sigmoid* sigmoid)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Sigmoid* sigmoid)" << std::endl;
#endif
    sigmoid->getArg()->editParents().push_back(sigmoid);
    return false;
}

int SetParents::visit(Sin* sin)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Sin* sin)" << std::endl;
#endif
    sin->getArg()->editParents().push_back(sin);
    return false;
}

int SetParents::visit(Sum* sum)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Sum* sum)" << std::endl;
#endif
    sum->getLeft()->editParents().push_back(sum);
    sum->getRight()->editParents().push_back(sum);
    return false;
}

int SetParents::visit(TermPower* power)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(TermPower* power)" << std::endl;
#endif
    power->getLeft()->editParents().push_back(power);
    power->getRight()->editParents().push_back(power);
    return false;
}

int SetParents::visit(Variable* /* var */)
{
#ifdef SetParentsDEBUG
    std::cout << "SetParents::visit(Variable* var)" << std::endl;
#endif
    return false;
}

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */
