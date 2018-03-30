/*
 * FormulaTransform.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#include "FormulaTransform.h"
//#define FORMULATRANS_DEBUG
//#define USE_EXTENDED_EQUALITY

#include "types/Clause.h"
#include "types/Lit.h"
#include "types/Var.h"
#include "CNSat.h"

#include <algorithm>

#ifdef USE_EXTENDED_EQUALITY
#include "TermEquality.h"
#endif

#include <iostream>

namespace alica {
namespace reasoner {
namespace cnsat {

FormulaTransform::FormulaTransform() {
#ifdef USE_EXTENDED_EQUALITY
    this->te = make_shared<TermEquality>();
#endif
    this->atomOccurrence = 0;
}

FormulaTransform::~FormulaTransform() {
    // TODO Auto-generated destructor stub
}

shared_ptr<list<shared_ptr<Clause>>> FormulaTransform::transformToCNF(
        shared_ptr<Term> formula, shared_ptr<CNSat> solver) {
    this->solver = solver;
    reset();
    shared_ptr<list<shared_ptr<Clause>>> clauses = make_shared<list<shared_ptr<Clause>>>();
    shared_ptr<Clause> initial = make_shared<Clause>();
    shared_ptr<Lit> lit = make_shared<Lit>(formula, Assignment::UNASSIGNED, true);
    initial->literals->push_back(lit);

    clauses->push_back(initial);

    doTransform(clauses);

#ifdef FORMULATRANS_DEBUG
    cout << "Clauses: " << clauses->size() << " Lits: " << atomOccurrence << " Vars: " << solver->variables->size()
         << "" << endl;
#endif

    return clauses;
}

void FormulaTransform::reset() {
    this->atoms.clear();
    this->atomOccurrence = 0;
}

shared_ptr<Var> FormulaTransform::getAtoms(int term_id) {
    return this->atoms[term_id];
}

int FormulaTransform::getAtomOccurrence() {
    return atomOccurrence;
}

void FormulaTransform::doTransform(shared_ptr<list<shared_ptr<Clause>>>& clauses) {
    shared_ptr<Clause> curClause = nullptr;
    shared_ptr<Lit> curLit = nullptr;
    int j = 0;
    auto clauseIter = clauses->begin();
    while (clauseIter != clauses->end()) {
        shared_ptr<Clause> tmpClause = *clauseIter;
        if (!tmpClause->isFinished) {
            bool finished = true;
            for (j = 0; j < tmpClause->literals->size(); ++j) {
                if (tmpClause->literals->at(j)->isTemporary) {
                    finished = false;
                    curClause = tmpClause;
                    curLit = curClause->literals->at(j);
                    break;
                }
            }
            if (!finished) {
                //							bool wasempty=false;
                //							clauseIter = clauses->erase(clauseIter);
                //							if(clauses->size()==0) {
                //								wasempty = true;
                //							}

                curClause->literals->erase(curClause->literals->begin() + j);
                shared_ptr<Clause> nc1 = nullptr, nc2 = nullptr;
                performStep(curClause, curLit, nc1, nc2);
                if (nc1 != nullptr) {
                    clauses->push_back(nc1);
                }
                if (nc2 != nullptr) {
                    clauses->push_back(nc2);
                }

                //							if(wasempty && clauses->size() > 0) {
                //								clauseIter = clauses->begin();
                //							} else if(wasempty && clauses->size() == 0) {
                //								clauseIter = clauses->end();
                //							}
                clauseIter = clauses->erase(clauseIter);

                //							clauseIter =
                //clauses->erase(find(clauses->begin(),  clauses->end(), tmpClause));
            } else {
                tmpClause->isFinished = true;
                clauseIter++;
            }
        } else {
            clauseIter++;
        }
    }
}

void FormulaTransform::performStep(
        shared_ptr<Clause>& c, shared_ptr<Lit>& lit, shared_ptr<Clause>& newClause1, shared_ptr<Clause>& newClause2) {
    shared_ptr<Term> formula = lit->atom;
    if (dynamic_pointer_cast<Max>(formula) != 0) {
        shared_ptr<Max> m = dynamic_pointer_cast<Max>(formula);
        shared_ptr<Lit> l = make_shared<Lit>(m->left, Assignment::UNASSIGNED, true);
        shared_ptr<Lit> r = make_shared<Lit>(m->right, Assignment::UNASSIGNED, true);
        c->addChecked(l);
        c->addChecked(r);
        newClause1 = c;
        newClause2 = nullptr;
        return;
    }
    if (dynamic_pointer_cast<And>(formula) != 0) {
        shared_ptr<And> a = dynamic_pointer_cast<And>(formula);
        shared_ptr<Lit> l = make_shared<Lit>(a->left, Assignment::UNASSIGNED, true);
        shared_ptr<Lit> r = make_shared<Lit>(a->right, Assignment::UNASSIGNED, true);
        shared_ptr<Clause> c2 = c->clone();
        c->addChecked(l);
        c2->addChecked(r);
        newClause1 = c;
        newClause2 = c2;
        return;
    }
    if (dynamic_pointer_cast<Or>(formula) != 0) {
        shared_ptr<Or> o = dynamic_pointer_cast<Or>(formula);
        shared_ptr<Lit> l = make_shared<Lit>(o->left, Assignment::UNASSIGNED, true);
        shared_ptr<Lit> r = make_shared<Lit>(o->right, Assignment::UNASSIGNED, true);
        c->addChecked(l);
        c->addChecked(r);
        newClause1 = c;
        newClause2 = nullptr;
        return;
    }
    if (dynamic_pointer_cast<Min>(formula) != 0) {
        shared_ptr<Min> m = dynamic_pointer_cast<Min>(formula);
        shared_ptr<Lit> l = make_shared<Lit>(m->left, Assignment::UNASSIGNED, true);
        shared_ptr<Lit> r = make_shared<Lit>(m->right, Assignment::UNASSIGNED, true);
        shared_ptr<Clause> c2 = c->clone();
        c->addChecked(l);
        c2->addChecked(r);
        newClause1 = c;
        newClause2 = c2;
        return;
    }
    if (dynamic_pointer_cast<LTConstraint>(formula) != 0) {
        lit->isTemporary = false;
        lit->computeVariableCount();
        lit->sign = Assignment::TRUE;
        this->atomOccurrence++;
        shared_ptr<Var> v = nullptr;
#ifdef USE_EXTENDED_EQUALITY
        if (this->tryGetVar(lit->atom, &v)) {
#else
        if (this->atoms.find(lit->atom->getId()) != atoms.end()) {
#endif
            lit->var = atoms[lit->atom->getId()];
        } else {
            lit->var = solver->newVar();
            lit->var->term = lit->atom;
            this->atoms[lit->atom->getId()] = lit->var;
        }
        c->addChecked(lit);
        newClause1 = c;
        newClause2 = nullptr;
        return;
    }
    if (dynamic_pointer_cast<LTEConstraint>(formula) != 0) {
        lit->isTemporary = false;
        lit->computeVariableCount();
        lit->sign = Assignment::FALSE;
        this->atomOccurrence++;
        shared_ptr<Term> p = dynamic_pointer_cast<LTEConstraint>(formula)->negate();
        lit->atom = p;
        shared_ptr<Var> v = nullptr;
#ifdef USE_EXTENDED_EQUALITY
        if (this->tryGetVar(p, &v)) {
#else
        if (this->atoms.find(p->getId()) != atoms.end()) {
#endif
            lit->var = atoms[p->getId()];
        } else {
            lit->var = solver->newVar();
            lit->var->term = p;
            this->atoms[p->getId()] = lit->var;
        }
        c->addChecked(lit);
        newClause1 = c;
        newClause2 = nullptr;
        return;
    }
    if (dynamic_pointer_cast<Constant>(formula) != 0) {
        if (dynamic_pointer_cast<Constant>(formula)->value <= 0.0) {
            newClause1 = c;
        } else
            newClause1 = nullptr;
        newClause2 = nullptr;
        return;
    }
    cerr << "Unknown constraint in transformation:  " << typeid(formula).name() << endl;
    throw "Unknown constraint in transformation!";
}

bool FormulaTransform::tryGetVar(shared_ptr<Term> t, shared_ptr<Var> v) {
    cout << "FormulaTransform::tryGetVar not implemented yet!" << endl;
    throw "FormulaTransform::tryGetVar not implemented yet!";
}

} /* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */
