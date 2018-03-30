/*
 * SetParents.cpp
 *
 *  Created on: Dec 12, 2014
 *      Author: Philipp
 */

#include "intervals/SetParents.h"

//#define SetParentsDEBUG

#include <vector>

namespace alica {
namespace reasoner {
namespace intervalpropagation {

SetParents::SetParents() {}

SetParents::~SetParents() {}

int SetParents::visit(shared_ptr<Abs> abs) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Abs> abs)" << endl;
#endif
    abs->arg->parents.push_back(abs);
    // abs->arg->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<And> and_) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<And> and_)" << endl;
#endif
    and_->left->parents.push_back(and_);
    and_->right->parents.push_back(and_);
    // and_->left->accept(this);
    // and_->right->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Atan2> atan2) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Atan2> atan2)" << endl;
#endif
    atan2->left->parents.push_back(atan2);
    atan2->right->parents.push_back(atan2);
    // atan2->left->accept(this);
    // atan2->right->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Constant> constant) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Constant> constant)" << endl;
#endif
    return false;
    // return UpdateInterval(constant,constant.Value,constant.Value);
}

int SetParents::visit(shared_ptr<ConstPower> intPower) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<ConstPower> intPower)" << endl;
#endif
    intPower->base->parents.push_back(intPower);
    // intPower->base->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<ConstraintUtility> cu) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<ConstraintUtility> cu)" << endl;
#endif
    cu->constraint->parents.push_back(cu);
    cu->utility->parents.push_back(cu);
    // cu->constraint->accept(this);
    // cu->utility->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Cos> cos) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Cos> cos)" << endl;
#endif
    cos->arg->parents.push_back(cos);
    // cos->arg->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Exp> exp) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Exp> exp)" << endl;
#endif
    exp->arg->parents.push_back(exp);
    // exp->arg->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Gp> gp) {
    throw "Not implemented yet";
    return false;
}

int SetParents::visit(shared_ptr<LinSigmoid> sigmoid) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<LinSigmoid> sigmoid)" << endl;
#endif
    sigmoid->arg->parents.push_back(sigmoid);
    // sigmoid->arg->accept(this);
    // sigmoid->mid->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Log> log) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Log> log)" << endl;
#endif
    log->arg->parents.push_back(log);
    // log->arg->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<LTConstraint> constraint) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<LTConstraint> constraint)" << endl;
#endif
    constraint->left->parents.push_back(constraint);
    constraint->right->parents.push_back(constraint);
    // constraint->left->accept(this);
    // constraint->right->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<LTEConstraint> constraint) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<LTEConstraint> constraint)" << endl;
#endif
    constraint->left->parents.push_back(constraint);
    constraint->right->parents.push_back(constraint);
    // constraint->left->accept(this);
    // constraint->right->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Max> max) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Max> max)" << endl;
#endif
    max->left->parents.push_back(max);
    max->right->parents.push_back(max);
    // max->left->accept(this);
    // max->right->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Min> min) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Min> min)" << endl;
#endif
    min->left->parents.push_back(min);
    min->right->parents.push_back(min);

    // min->left->accept(this);
    // min->right->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Or> or_) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Or> or_)" << endl;
#endif
    or_->left->parents.push_back(or_);
    or_->right->parents.push_back(or_);
    // or_->left->accept(this);
    // or_->right->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Product> product) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Product> product)" << endl;
#endif
    product->left->parents.push_back(product);
    product->right->parents.push_back(product);

    // product->left->accept(this);
    // product->right->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Reification> reif) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Reification> reif)" << endl;
#endif
    reif->condition->parents.push_back(reif);
    // reif->condition->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Sigmoid> sigmoid) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Sigmoid> sigmoid)" << endl;
#endif
    sigmoid->arg->parents.push_back(sigmoid);
    sigmoid->mid->parents.push_back(sigmoid);
    // sigmoid->arg->accept(this);
    // sigmoid->mid->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Sin> sin) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Sin> sin)" << endl;
#endif
    sin->arg->parents.push_back(sin);
    // sin->arg->accept(this);
    return false;
}

int SetParents::visit(shared_ptr<Sum> sum) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Sum> sum)" << endl;
#endif
    for (shared_ptr<Term> t : sum->terms) {
        t->parents.push_back(sum);
        // t->accept(this);
    }
    return false;
}

int SetParents::visit(shared_ptr<TermPower> power) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<TermPower> power)" << endl;
#endif
    power->base->parents.push_back(power);
    power->exponent->parents.push_back(power);
    return false;
}

int SetParents::visit(shared_ptr<Variable> var) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Variable> var)" << endl;
#endif
    return false;
}

int SetParents::visit(shared_ptr<Zero> zero) {
#ifdef SetParentsDEBUG
    cout << "SetParents::visit(shared_ptr<Zero> zero)" << endl;
#endif
    return false;
    // return UpdateInterval(zero,0,0);
}

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */
