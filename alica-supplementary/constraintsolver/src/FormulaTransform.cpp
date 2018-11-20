/*
 * FormulaTransform.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#include "FormulaTransform.h"

#include "CNSat.h"
#include "types/Clause.h"
#include "types/Lit.h"
#include "types/Var.h"

#include <autodiff/AutoDiff.h>

#include <algorithm>

#include <iostream>

//#define FORMULATRANS_DEBUG
//#define USE_EXTENDED_EQUALITY

#ifdef USE_EXTENDED_EQUALITY
#include "TermEquality.h"
#endif

namespace alica
{
namespace reasoner
{
namespace cnsat
{

using autodiff::TermPtr;

FormulaTransform::FormulaTransform()
{
#ifdef USE_EXTENDED_EQUALITY
    this->te = std::make_shared<TermEquality>();
#endif
    this->atomOccurrence = 0;
}

FormulaTransform::~FormulaTransform() {}

std::shared_ptr<std::list<std::shared_ptr<Clause>>> FormulaTransform::transformToCNF(TermPtr formula, std::shared_ptr<CNSat> solver)
{
    this->solver = solver;
    reset();
    std::shared_ptr<std::list<std::shared_ptr<Clause>>> clauses = std::make_shared<std::list<std::shared_ptr<Clause>>>();
    std::shared_ptr<Clause> initial = std::make_shared<Clause>();
    std::shared_ptr<Lit> lit = std::make_shared<Lit>(formula, Assignment::UNASSIGNED, true);
    initial->literals->push_back(lit);

    clauses->push_back(initial);

    doTransform(clauses);

#ifdef FORMULATRANS_DEBUG
    std::cout << "Clauses: " << clauses->size() << " Lits: " << atomOccurrence << " Vars: " << solver->variables->size() << "" << std::endl;
#endif

    return clauses;
}

void FormulaTransform::reset()
{
    this->atoms.clear();
    this->atomOccurrence = 0;
}

int FormulaTransform::getAtomOccurrence() const
{
    return atomOccurrence;
}

void FormulaTransform::doTransform(std::shared_ptr<std::list<std::shared_ptr<Clause>>>& clauses)
{
    std::shared_ptr<Clause> curClause = nullptr;
    std::shared_ptr<Lit> curLit = nullptr;
    int j = 0;
    auto clauseIter = clauses->begin();
    while (clauseIter != clauses->end()) {
        std::shared_ptr<Clause> tmpClause = *clauseIter;
        if (!tmpClause->isFinished) {
            bool finished = true;
            for (j = 0; j < static_cast<int>(tmpClause->literals->size()); ++j) {
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
                std::shared_ptr<Clause> nc1 = nullptr, nc2 = nullptr;
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
                // clauses->erase(find(clauses->begin(),  clauses->end(), tmpClause));
            } else {
                tmpClause->isFinished = true;
                clauseIter++;
            }
        } else {
            clauseIter++;
        }
    }
}

void FormulaTransform::performStep(std::shared_ptr<Clause>& c, std::shared_ptr<Lit>& lit, std::shared_ptr<Clause>& newClause1,
                                   std::shared_ptr<Clause>& newClause2)
{
    TermPtr formula = lit->_atom;
    autodiff::Max* asMax = dynamic_cast<autodiff::Max*>(formula.get());
    if (asMax != nullptr) {
        std::shared_ptr<Lit> l = std::make_shared<Lit>(asMax->getLeft(), Assignment::UNASSIGNED, true);
        std::shared_ptr<Lit> r = std::make_shared<Lit>(asMax->getRight(), Assignment::UNASSIGNED, true);
        c->addChecked(l);
        c->addChecked(r);
        newClause1 = c;
        newClause2 = nullptr;
        return;
    }
    autodiff::And* asAnd = dynamic_cast<autodiff::And*>(formula.get());
    if (asAnd != nullptr) {
        std::shared_ptr<Lit> l = std::make_shared<Lit>(asAnd->getLeft(), Assignment::UNASSIGNED, true);
        std::shared_ptr<Lit> r = std::make_shared<Lit>(asAnd->getRight(), Assignment::UNASSIGNED, true);
        std::shared_ptr<Clause> c2 = c->clone();
        c->addChecked(l);
        c2->addChecked(r);
        newClause1 = c;
        newClause2 = c2;
        return;
    }
    autodiff::Or* asOr = dynamic_cast<autodiff::Or*>(formula.get());
    if (asOr != nullptr) {
        std::shared_ptr<Lit> l = std::make_shared<Lit>(asOr->getLeft(), Assignment::UNASSIGNED, true);
        std::shared_ptr<Lit> r = std::make_shared<Lit>(asOr->getRight(), Assignment::UNASSIGNED, true);
        c->addChecked(l);
        c->addChecked(r);
        newClause1 = c;
        newClause2 = nullptr;
        return;
    }
    autodiff::Min* asMin = dynamic_cast<autodiff::Min*>(formula.get());
    if (asMin != nullptr) {
        std::shared_ptr<Lit> l = std::make_shared<Lit>(asMin->getLeft(), Assignment::UNASSIGNED, true);
        std::shared_ptr<Lit> r = std::make_shared<Lit>(asMin->getRight(), Assignment::UNASSIGNED, true);
        std::shared_ptr<Clause> c2 = c->clone();
        c->addChecked(l);
        c2->addChecked(r);
        newClause1 = c;
        newClause2 = c2;
        return;
    }
    if (dynamic_cast<autodiff::LTConstraint*>(formula.get()) != nullptr) {
        lit->isTemporary = false;
        lit->computeVariableCount();
        lit->sign = Assignment::TRUE;
        this->atomOccurrence++;
        std::shared_ptr<Var> v = nullptr;
#ifdef USE_EXTENDED_EQUALITY
        if (this->tryGetVar(lit->_atom, &v)) {
#else
        if (this->atoms.find(lit->_atom) != atoms.end()) {
#endif
            lit->var = atoms[lit->_atom];
        } else {
            lit->var = solver->newVar();
            lit->var->_term = lit->_atom;
            this->atoms[lit->_atom] = lit->var;
        }
        c->addChecked(lit);
        newClause1 = c;
        newClause2 = nullptr;
        return;
    }

    if (dynamic_cast<autodiff::LTEConstraint*>(formula.get()) != nullptr) {
        lit->isTemporary = false;
        lit->computeVariableCount();
        lit->sign = Assignment::FALSE;
        this->atomOccurrence++;
        autodiff::TermPtr p = dynamic_cast<autodiff::LTEConstraint*>(formula.get())->negate();
        lit->_atom = p;
        std::shared_ptr<Var> v = nullptr;
#ifdef USE_EXTENDED_EQUALITY
        if (this->tryGetVar(p, &v)) {
#else
        if (this->atoms.find(p) != atoms.end()) {
#endif
            lit->var = atoms[p];
        } else {
            lit->var = solver->newVar();
            lit->var->_term = p;
            this->atoms[p] = lit->var;
        }
        c->addChecked(lit);
        newClause1 = c;
        newClause2 = nullptr;
        return;
    }
    autodiff::Constant* asConst = dynamic_cast<autodiff::Constant*>(formula.get());
    if (asConst != nullptr) {
        if (asConst->getValue() <= 0.0) {
            newClause1 = c;
        } else {
            newClause1 = nullptr;
        }
        newClause2 = nullptr;
        return;
    }
    std::cerr << "Unknown constraint in transformation:  " << typeid(formula.get()).name() << std::endl;
    throw "Unknown constraint in transformation!";
}

bool FormulaTransform::tryGetVar(autodiff::TermPtr t, std::shared_ptr<Var> v)
{
    std::cout << "FormulaTransform::tryGetVar not implemented yet!" << std::endl;
    throw "FormulaTransform::tryGetVar not implemented yet!";
}

} /* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */
