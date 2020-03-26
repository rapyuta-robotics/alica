#include "constraintsolver/CNSMTGSolver.h"
//#define CNSMTGSOLVER_LOG
//#define DO_PREPROPAGATION
//#define CNSMTGSolver_DEBUG

#include "essentials/Configuration.h"
#include "essentials/SystemConfig.h"

#include "constraintsolver/CNSat.h"
#include "constraintsolver/FormulaTransform.h"
#include "intervals/IntervalPropagator.h"
#include "types/Assignment.h"
#include "types/Clause.h"
#include "types/Var.h"

#include <autodiff/Constant.h>
#include <autodiff/ConstraintUtility.h>
#include <autodiff/Tape.h>
#include <autodiff/TermHolder.h>
#include <autodiff/TermPtr.h>

#include <cmath>
#include <limits>
#include <sstream>
#include <stdlib.h> /* srand, rand */
#include <string>
#include <time.h> /* time */

#include <iostream>

namespace alica
{
namespace reasoner
{

using autodiff::TermPtr;
using std::cout;
using std::endl;
using std::list;
using std::make_shared;
using std::shared_ptr;
using std::vector;

int CNSMTGSolver::fcounter = 0;

CNSMTGSolver::CNSMTGSolver()
        : dim(0)
        , utilityThreshold(1.0)
        , begin()
        , runs(0)
        , fevals(0)
        , seedWithUtilOptimum(true)
{
    autodiff::Term::setAnd(autodiff::AndType::AND);
    autodiff::Term::setOr(autodiff::OrType::MAX);

    ft = make_shared<cnsat::FormulaTransform>();

    ip = make_shared<intervalpropagation::IntervalPropagator>();

    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    maxfevals = sc["Alica"]->get<int>("Alica", "CSPSolving", "MaxFunctionEvaluations", NULL);
    maxSolveTime = AlicaTime::milliseconds(sc["Alica"]->get<int>("Alica", "CSPSolving", "MaxSolveTime", NULL));
    rPropConvergenceStepSize = 0;
    useIntervalProp = true;
    optimize = false;

    this->lastSeed = nullptr;
}

AlicaTime CNSMTGSolver::getTime() const
{
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return AlicaTime::nanoseconds(std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count());
}

CNSMTGSolver::~CNSMTGSolver() {}

int CNSMTGSolver::RpropResult::compareTo(shared_ptr<RpropResult> other)
{
    return finalUtil > other->finalUtil ? -1 : 1;
}

void CNSMTGSolver::initLog()
{
    std::stringstream ss;
    ss << "/tmp/test" << (fcounter++) << ".dbg";
    std::string logFile = ss.str();
    sw.open(logFile);
}

void CNSMTGSolver::log(double util, shared_ptr<vector<double>>& val)
{
    sw << util;
    sw << "\t";
    for (int i = 0; i < dim; ++i) {
        sw << val->at(i);
        sw << "\t";
    }
    sw << endl;
}

void CNSMTGSolver::logStep()
{
    sw << endl;
    sw << endl;
}

void CNSMTGSolver::closeLog()
{
    sw.close();
}

shared_ptr<vector<double>> CNSMTGSolver::solve(
        autodiff::TermPtr equation, autodiff::TermHolder& holder, shared_ptr<vector<shared_ptr<vector<double>>>>& limits, double& util)
{
    return solve(equation, holder, limits, nullptr, std::numeric_limits<double>::max(), util);
}

shared_ptr<vector<double>> CNSMTGSolver::solve(autodiff::TermPtr equation, autodiff::TermHolder& holder, shared_ptr<vector<shared_ptr<vector<double>>>>& limits,
        shared_ptr<vector<shared_ptr<vector<double>>>> seeds, double sufficientUtility, double& util)
{
#ifdef CNSMTGSolver_DEBUG
    cout << "CNSMTGSolver::solve() " << useIntervalProp << endl;
    cout << equation->toString() << endl;
#endif
    lastSeed = nullptr;
    probeCount = 0;
    successProbeCount = 0;
    intervalCount = 0;
    successIntervalCount = 0;
    fevalsCount = 0;
    runCount = 0;
    this->begin = this->getTime();

    util = 0;
#ifdef CNSMTGSOLVER_LOG
    initLog();
#endif
    rResults.clear();

    this->dim = equation->getOwner()->getDim();
    this->limits = limits;
    this->ranges = vector<double>(dim);
    this->rpropStepWidth = vector<double>(dim);
    this->rpropStepConvergenceThreshold = vector<double>(dim);

    equation = equation->aggregateConstants();

    autodiff::ConstraintUtility* cu = static_cast<autodiff::ConstraintUtility*>(equation.get());

    bool constraintIsConstant = cu->getConstraint()->isConstant();
    if (constraintIsConstant) {
        autodiff::Constant* constraint = static_cast<autodiff::Constant*>(cu->getConstraint().get());
        if (constraint->getValue() < 0.25) {
            util = constraint->getValue();
            shared_ptr<vector<double>> ret = make_shared<vector<double>>(dim);
            for (int i = 0; i < dim; ++i) {
                ret->at(i) = (limits->at(i)->at(1) + limits->at(i)->at(0)) / 2.0;
            }
            return ret;
        }
    }
    ss = make_shared<cnsat::CNSat>();

    ss->useIntervalProp = this->useIntervalProp;
    shared_ptr<list<shared_ptr<cnsat::Clause>>> cnf = ft->transformToCNF(cu->getConstraint(), ss);
    /*for (shared_ptr<cnsat::Clause> c : *cnf)
     {
     c->print();
     }*/

    if (this->useIntervalProp) {
        ip->setGlobalRanges(*equation->getOwner(), limits, ss);
    }

    for (std::shared_ptr<cnsat::Clause> c : *cnf) {
        if (!c->isTautologic) {
            if (c->literals->size() == 0) {
                util = std::numeric_limits<double>::lowest();
                std::shared_ptr<std::vector<double>> ret = std::make_shared<std::vector<double>>(dim);
                for (int i = 0; i < dim; ++i) {
                    ret->at(i) = (limits->at(i)->at(1) + limits->at(i)->at(0)) / 2.0;
                }
                return ret;
            }
            ss->addBasicClause(c);
        }
    }
    ss->cnsmtGSolver = this;

    ss->init();
    if (this->useIntervalProp) {
#ifdef DO_PREPROPAGATION
        if (!ip->prePropagate(ss->variables)) {
            cout << "Unsatisfiable (unit propagation)" << endl;
            return nullptr;
        }
#endif
    }

    bool solutionFound = false;

    ss->unitDecissions = ss->decisions->size();

    do {
        if (!solutionFound) {
            ss->emptySATClause();
            ss->emptyTClause();
            ss->backTrack(ss->unitDecissions);
        }
        solutionFound = ss->solve(getTime() + AlicaTime::minutes(1));
        if (this->optimize) {
            r1 = rPropOptimizeFeasible(ss->decisions, cu->getUtility(), r1->finalValue, false);
        }
        if (!solutionFound && r1->finalUtil > 0) {
            r1->finalUtil = -1;
        }
        util = r1->finalUtil;
        if (!this->optimize && solutionFound) {
            return r1->finalValue;
        } else if (this->optimize) {
            rResults.push_back(r1);
            shared_ptr<cnsat::Clause> c = make_shared<cnsat::Clause>();
            for (shared_ptr<cnsat::Var> v : *(ss->decisions)) {
                cnsat::Assignment ass = v->assignment == cnsat::Assignment::TRUE ? cnsat::Assignment::FALSE : cnsat::Assignment::TRUE;
                shared_ptr<cnsat::Lit> lit = make_shared<cnsat::Lit>(v, ass);
                c->add(lit);
            }
            ss->addIClause(c);
            ss->backTrack(ss->decisions->at(ss->decisions->size() - 1)->decisionLevel);
            solutionFound = false;
        }
    } while (!solutionFound && this->begin + this->maxSolveTime > getTime());

    if (rResults.size() > 0) {
        for (shared_ptr<RpropResult> rp : rResults) {
            if (rp->finalUtil > util) {
                util = rp->finalUtil;
                r1 = rp;
            }
            return r1->finalValue;
        }
    }
    //			cout << "Unsatisfiable" << endl;

    return nullptr;
}

bool CNSMTGSolver::intervalPropagate(shared_ptr<vector<shared_ptr<cnsat::Var>>> decisions, shared_ptr<vector<shared_ptr<vector<double>>>>& curRanges)
{
#ifdef CNSMTGSolver_DEBUG
    cout << "CNSMTGSolver::intervalPropagate()" << endl;
#endif
    this->intervalCount++;
    shared_ptr<vector<shared_ptr<cnsat::Var>>> offending = nullptr;
    if (!ip->propagate(decisions, curRanges, offending)) {
        cout << "yo1" << endl;
        if (offending != nullptr) {
            cout << "yo2" << endl;
            shared_ptr<cnsat::Clause> learnt = make_shared<cnsat::Clause>();
            for (shared_ptr<cnsat::Var>& v : *offending) {
                shared_ptr<cnsat::Lit> lit =
                        make_shared<cnsat::Lit>(v, v->assignment == cnsat::Assignment::TRUE ? cnsat::Assignment::FALSE : cnsat::Assignment::TRUE);
                learnt->add(lit);
            }
            ss->addIClause(learnt);
        }
        return false;
    }
    this->limits = curRanges;
    this->successIntervalCount++;
    return true;
}

bool CNSMTGSolver::probeForSolution(std::shared_ptr<std::vector<std::shared_ptr<cnsat::Var>>> decisions, std::shared_ptr<std::vector<double>> solution)
{
#ifdef CNSMTGSolver_DEBUG
    cout << "CNSMTGSolver::probeForSolution()" << endl;
    cout << "\tDecisions:" << endl;
    for (int i = 0; i < decisions->size(); ++i) {
        auto decision = decisions->at(i);
        cout << "\t\t" << decision->toString() << endl;
    }
    cout << "\t------------------------" << endl;
#endif
    this->probeCount++;
    solution = nullptr;
    for (int i = 0; i < dim; ++i) {
        double upper = this->limits->at(i)->at(1);
        double lower = this->limits->at(i)->at(0);
        this->ranges.at(i) = (upper - lower);
    }

    for (int i = 0; i < static_cast<int>(decisions->size()); ++i) {
        auto decision = decisions->at(i);
        decision->_curTerm = decision->assignment == cnsat::Assignment::TRUE ? &decision->_positiveTerm : &decision->_negativeTerm;
    }

    r1 = rPropFindFeasible(decisions, lastSeed);
    if (r1->finalUtil < 0.5) {
        r1 = rPropFindFeasible(decisions, nullptr);
    }
    if (r1->finalUtil < 0.5) {
        shared_ptr<cnsat::Clause> learnt = make_shared<cnsat::Clause>();
        for (shared_ptr<cnsat::Var> v : *(ss->decisions)) {
            shared_ptr<cnsat::Lit> lit =
                    make_shared<cnsat::Lit>(v, v->assignment == cnsat::Assignment::TRUE ? cnsat::Assignment::FALSE : cnsat::Assignment::TRUE);
            learnt->add(lit);
        }
        ss->addTClause(learnt);

#ifdef CNSMTGSolver_DEBUG
        cout << "\treturn FALSE" << endl;
#endif
        return false;
    }

    lastSeed = make_shared<vector<double>>(*r1->finalValue);
    solution = make_shared<vector<double>>(*r1->finalValue);
    successProbeCount++;
#ifdef CNSMTGSolver_DEBUG
    cout << "\treturn TRUE" << endl;
#endif
    return true;
}

long CNSMTGSolver::getRuns()
{
    return this->runs;
}

long CNSMTGSolver::getFEvals()
{
    return this->fevals;
}

double CNSMTGSolver::getRPropConvergenceStepSize()
{
    return this->rPropConvergenceStepSize;
}

shared_ptr<cnsat::CNSat> CNSMTGSolver::getCNSatSolver()
{
    return this->ss;
}

void CNSMTGSolver::setUseIntervalProp(bool useIntervalProp)
{
    this->useIntervalProp = useIntervalProp;
}

shared_ptr<CNSMTGSolver::RpropResult> CNSMTGSolver::rPropFindFeasible(shared_ptr<vector<shared_ptr<cnsat::Var>>> constraints, shared_ptr<vector<double>> seed)
{
#ifdef CNSMTGSolver_DEBUG
    cout << "CNSMTGSolver::rPropFindFeasible()" << endl;
#endif
    runCount++;
    initializeStepSize();
    shared_ptr<vector<double>> curGradient;
    shared_ptr<RpropResult> ret = make_shared<RpropResult>();

    if (seed != nullptr) {
        curGradient = initialPointFromSeed(constraints, ret, seed);
    } else {
        curGradient = initialPoint(constraints, ret);
    }

    double curUtil = ret->initialUtil;
    if (curUtil > 0.5) {
        return ret;
    }

    shared_ptr<vector<double>> formerGradient = make_shared<vector<double>>(dim);
    shared_ptr<vector<double>> curValue = make_shared<vector<double>>(dim);

    *curValue = *ret->finalValue;
    *formerGradient = *curGradient;

    int itcounter = 0;
    int badcounter = 0;

#ifdef CNSMTGSOLVER_LOG
    log(curUtil, curValue);
#endif

    int maxIter = 60;
    int maxBad = 30;
    double minStep = 1E-11;

    while (itcounter++ < maxIter && badcounter < maxBad) {
        for (int i = 0; i < dim; ++i) {
            if (curGradient->at(i) * formerGradient->at(i) > 0) {
                rpropStepWidth.at(i) *= 1.3;
            } else if (curGradient->at(i) * formerGradient->at(i) < 0) {
                rpropStepWidth.at(i) *= 0.5;
            }
            rpropStepWidth.at(i) = std::max(minStep, rpropStepWidth.at(i));
            if (curGradient->at(i) > 0) {
                curValue->at(i) += rpropStepWidth.at(i);
            } else if (curGradient->at(i) < 0) {
                curValue->at(i) -= rpropStepWidth.at(i);
            }

            if (curValue->at(i) > limits->at(i)->at(1)) {
                curValue->at(i) = limits->at(i)->at(1);
            } else if (curValue->at(i) < limits->at(i)->at(0)) {
                curValue->at(i) = limits->at(i)->at(0);
            }
        }
        this->fevalsCount++;
        *formerGradient = *curGradient;
        differentiate(constraints, curValue, curGradient, curUtil);

        bool allZero = true;
        for (int i = 0; i < dim; ++i) {
            if (std::isnan(curGradient->at(i))) {
                ret->aborted = true;
#ifdef CNSMTGSOLVER_LOG
                logStep();
#endif
                return ret;
            }
            allZero &= (curGradient->at(i) == 0);
        }

#ifdef CNSMTGSOLVER_LOG
        log(curUtil, curValue);
#endif

        if (curUtil > ret->finalUtil) {
            badcounter = 0;

            ret->finalUtil = curUtil;
            *ret->finalValue = *curValue;
            if (curUtil > 0.75) {
                return ret;
            }
        } else {
            badcounter++;
        }
        if (allZero) {
            ret->aborted = false;
#ifdef CNSMTGSOLVER_LOG
            logStep();
#endif
            return ret;
        }
    }
#ifdef CNSMTGSOLVER_LOG
    logStep();
#endif
    ret->aborted = false;
    return ret;
}

shared_ptr<CNSMTGSolver::RpropResult> CNSMTGSolver::rPropOptimizeFeasible(shared_ptr<vector<shared_ptr<cnsat::Var>>> constraints, TermPtr ut,
        // shared_ptr<vector<shared_ptr<autodiff::Variable>>> args,
        shared_ptr<vector<double>>& seed, bool precise)
{
#ifdef CNSMTGSolver_DEBUG
    cout << "CNSMTGSolver::rPropOptimizeFeasible()" << endl;
#endif

    TermPtr constr = ut->getOwner()->trueConstant();
    for (shared_ptr<cnsat::Var> v : *constraints) {
        if (v->assignment == cnsat::Assignment::TRUE) {
            constr = constr & v->_term;
        } else {
            constr = constr & v->_term->negate();
        }
    }
    TermPtr cu = ut->getOwner()->constraintUtility(constr, ut);
    cu->getOwner()->compile(cu);
    const int dim = cu->getOwner()->getDim();
    std::vector<double> evaluation_result(dim + 1);

    runCount++;
    initializeStepSize();
    shared_ptr<vector<double>> curGradient;
    shared_ptr<RpropResult> ret = make_shared<RpropResult>();

    if (seed != nullptr) {
        curGradient = initialPointFromSeed(constraints, ret, seed);
    } else {
        curGradient = initialPoint(constraints, ret);
    }
    double curUtil = ret->initialUtil;

    shared_ptr<vector<double>> formerGradient = make_shared<vector<double>>(dim);
    shared_ptr<vector<double>> curValue = make_shared<vector<double>>(dim);

    *curValue = *ret->initialValue;
    *formerGradient = *curGradient;

    int itcounter = 0;
    int badcounter = 0;

#ifdef CNSMTGSOLVER_LOG
    log(curUtil, curValue);
#endif

    int maxIter = 60;
    int maxBad = 30;
    double minStep = 1E-11;
    if (precise) {
        maxIter = 120;   // 110
        maxBad = 60;     // 60
        minStep = 1E-15; // 15
    }
    int convergendDims = 0;

    while (itcounter++ < maxIter && badcounter < maxBad) {
        convergendDims = 0;
        for (int i = 0; i < dim; ++i) {
            if (curGradient->at(i) * formerGradient->at(i) > 0) {
                rpropStepWidth.at(i) *= 1.3;
            } else if (curGradient->at(i) * formerGradient->at(i) < 0) {
                rpropStepWidth.at(i) *= 0.5;
            }
            rpropStepWidth.at(i) = std::max(minStep, rpropStepWidth.at(i));
            if (curGradient->at(i) > 0) {
                curValue->at(i) += rpropStepWidth.at(i);
            } else if (curGradient->at(i) < 0) {
                curValue->at(i) -= rpropStepWidth.at(i);
            }

            if (curValue->at(i) > limits->at(i)->at(1)) {
                curValue->at(i) = limits->at(i)->at(1);
            } else if (curValue->at(i) < limits->at(i)->at(0)) {
                curValue->at(i) = limits->at(i)->at(0);
            }

            if (rpropStepWidth.at(i) < rpropStepConvergenceThreshold.at(i)) {
                ++convergendDims;
            }
        }

        if (!precise && convergendDims >= dim) {
            return ret;
        }
        this->fevalsCount++;
        *formerGradient = *curGradient;
        cu->getOwner()->evaluate(&(*curValue)[0], &evaluation_result[0]);

        bool allZero = true;
        for (int i = 0; i < dim; ++i) {
            if (std::isnan(evaluation_result[i + 1])) {
                ret->aborted = false; // true; // HACK!
#ifdef CNSMTGSOLVER_LOG
                logStep();
#endif
                return ret;
            }
            allZero = allZero && (evaluation_result[i + 1] == 0);
        }

        curUtil = evaluation_result[0];
        *formerGradient = *curGradient;
        std::copy(evaluation_result.begin() + 1, evaluation_result.end(), curGradient->begin());
#ifdef CNSMTGSOLVER_LOG
        log(curUtil, curValue);
#endif

        if (curUtil > ret->finalUtil) {
            badcounter = 0;

            ret->finalUtil = curUtil;
            *ret->finalValue = *curValue;
            if (curUtil > 0.75) {
                return ret;
            }
        } else {
            badcounter++;
        }
        if (allZero) {
            ret->aborted = false;
#ifdef CNSMTGSOLVER_LOG
            logStep();
#endif
            return ret;
        }
#ifdef CNSMTGSOLVER_LOG
        logStep();
#endif
        ret->aborted = false;
    }
    return ret;
}

void CNSMTGSolver::differentiate(
        shared_ptr<vector<shared_ptr<cnsat::Var>>> constraints, shared_ptr<vector<double>>& val, shared_ptr<vector<double>>& gradient, double& util)
{
#ifdef CNSMTGSolver_DEBUG
    cout << "CNSMTGSolver::differentiate()" << endl;
#endif
    std::vector<double> evaluation_result(val->size() + 1);
    constraints->at(0)->_curTerm->evaluate(&(*val)[0], &evaluation_result[0]);
    gradient->resize(val->size());
    std::copy(evaluation_result.begin() + 1, evaluation_result.end(), gradient->begin());
    util = evaluation_result[0];
    for (int i = 1; i < static_cast<int>(constraints->size()); ++i) {
        constraints->at(i)->_curTerm->evaluate(&(*val)[0], &evaluation_result[0]);
        if (evaluation_result[0] <= 0) {
            if (util > 0) {
                util = evaluation_result[0];
            } else {
                util += evaluation_result[0];
            }
            for (int j = 0; j < dim; ++j) {
                gradient->at(j) += evaluation_result[j + 1];
            }
        }
    }
#ifdef CNSMTGSolver_DEBUG
    cout << "\tutil = " << util << endl;
#endif
}

shared_ptr<vector<double>> CNSMTGSolver::initialPointFromSeed(
        shared_ptr<vector<shared_ptr<cnsat::Var>>> constraints, shared_ptr<CNSMTGSolver::RpropResult> res, shared_ptr<vector<double>>& seed)
{
#ifdef CNSMTGSolver_DEBUG
    cout << "CNSMTGSolver::initialPointFromSeed()" << endl;
#endif
    std::vector<double> evaluation_result(dim + 1);
    bool found = true;
    res->initialValue = make_shared<vector<double>>(dim);
    res->finalValue = make_shared<vector<double>>(dim);
    shared_ptr<vector<double>> gradient;
    do {
        gradient = make_shared<vector<double>>(dim);
        found = true;
        res->initialUtil = 1;
        for (int i = 0; i < dim; ++i) {
            if (std::isnan(seed->at(i))) {
#ifdef CNSMTGSolver_DEBUG
                cout << "\tstd::isnan(seed->at(i) => THEN" << endl;
#endif
                res->initialValue->at(i) = ((double) rand() / RAND_MAX) * ranges[i] + limits->at(i)->at(0);
            } else {
#ifdef CNSMTGSolver_DEBUG
                cout << "\tstd::isnan(seed->at(i) => ELSE" << endl;
#endif
                res->initialValue->at(i) = std::min(std::max(seed->at(i), limits->at(i)->at(0)), limits->at(i)->at(1));
            }
        }

        this->fevalsCount++;
        for (int i = 0; i < static_cast<int>(constraints->size()); ++i) {
#ifdef CNSMTGSolver_DEBUG
            cout << "\tconstraints => " << i << endl;
#endif
            if (constraints->at(i)->assignment == cnsat::Assignment::TRUE) {
#ifdef CNSMTGSolver_DEBUG
                cout << "\tconstraints->at(i)->assignment == cnsat::Assignment::TRUE" << endl;
#endif
                constraints->at(i)->setToPositive();
            } else {
#ifdef CNSMTGSolver_DEBUG
                cout << "\tconstraints->at(i)->assignment != cnsat::Assignment::TRUE" << endl;
#endif
                constraints->at(i)->setToNegative();
            }
            constraints->at(i)->_curTerm->evaluate(&(*res->initialValue)[0], &evaluation_result[0]);
            for (int j = 0; j < dim; ++j) {
                if (std::isnan(evaluation_result[j + 1])) {
#ifdef CNSMTGSolver_DEBUG
                    cout << "\t" << j << " std::isnan(tup.first->at(j))" << endl;
#endif
                    found = false;
                    break;
                }
#ifdef CNSMTGSolver_DEBUG
                cout << "\t" << j << " !std::isnan(tup.first->at(j))" << endl;
#endif
                gradient->at(j) += evaluation_result[j + 1];
            }
            if (!found) {
                break;
            }
            if (evaluation_result[0] <= 0) {
                if (res->initialUtil > 0) {
                    res->initialUtil = evaluation_result[0];
                } else {
                    res->initialUtil += evaluation_result[0];
                }
            }
        }
    } while (!found);
    res->finalUtil = res->initialUtil;

    *res->finalValue = *res->initialValue;

    return gradient;
}

shared_ptr<vector<double>> CNSMTGSolver::initialPoint(shared_ptr<vector<shared_ptr<cnsat::Var>>> constraints, shared_ptr<CNSMTGSolver::RpropResult> res)
{
#ifdef CNSMTGSolver_DEBUG
    cout << "CNSMTGSolver::initialPoint()" << endl;
#endif
    std::vector<double> evaluation_result(dim + 1);
    bool found = true;
    res->initialValue = make_shared<vector<double>>(dim);
    res->finalValue = make_shared<vector<double>>(dim);
    shared_ptr<vector<double>> gradient;
    do {
        gradient = make_shared<vector<double>>(dim);
        found = true;
        res->initialUtil = 1;
        for (int i = 0; i < dim; ++i) {
            res->initialValue->at(i) = ((double) rand() / RAND_MAX) * ranges[i] + limits->at(i)->at(0);
            //					cout << "if (TMPVAR == " << TMPVAR++ << ") { res.initialValue[i] = " <<
            // res->initialValue->at(i)
            //<< "; }" << endl;
        }

        this->fevalsCount++;
        for (int i = 0; i < static_cast<int>(constraints->size()); ++i) {
            if (constraints->at(i)->assignment == cnsat::Assignment::TRUE) {
                constraints->at(i)->setToPositive();
            } else {
                constraints->at(i)->setToNegative();
            }
            constraints->at(i)->_curTerm->evaluate(&(*res->initialValue)[0], &evaluation_result[0]);
            for (int j = 0; j < dim; ++j) {
                if (std::isnan(evaluation_result[j + 1])) {
                    found = false;
                    break;
                }
                gradient->at(j) += evaluation_result[j + 1];
            }
            if (!found) {
                break;
            }
            if (evaluation_result[0] <= 0) {
                if (res->initialUtil > 0) {
                    res->initialUtil = evaluation_result[0];
                } else {
                    res->initialUtil += evaluation_result[0];
                }
            }
        }
    } while (!found);
    res->finalUtil = res->initialUtil;

    *res->finalValue = *res->initialValue;

    return gradient;
}

void CNSMTGSolver::initializeStepSize()
{
#ifdef CNSMTGSolver_DEBUG
    cout << "CNSMTGSolver::initializeStepSize()" << endl;
#endif
    for (int i = 0; i < dim; ++i) {
        this->rpropStepWidth.at(i) = initialStepSize * ranges.at(i);
        this->rpropStepConvergenceThreshold.at(i) = rpropStepWidth.at(i) * this->rPropConvergenceStepSize;
    }
}

} /* namespace reasoner */
} /* namespace alica */
