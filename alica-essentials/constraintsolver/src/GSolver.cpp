/*
 * GSolver.cpp
 *
 *  Created on: Aug 12, 2014
 *      Author: psp
 */

#include "GSolver.h"
#define GSOLVER_LOG
//#define ALWAYS_CHECK_THRESHOLD
#define AGGREGATE_CONSTANTS

#include "Configuration.h"
#include "SystemConfig.h"

#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>

#include <cmath>
#include <limits>
#include <sstream>
#include <stdlib.h> /* srand, rand */
#include <string>
#include <time.h> /* time */

#include <iostream>

namespace alica {
namespace reasoner {
int GSolver::_fcounter = 0;

GSolver::GSolver() {
    Term::setAnd(AndType::AND);
    Term::setOr(OrType::MAX);

    _rResults = vector<shared_ptr<RpropResult>>();

    _seedWithUtilOptimum = true;
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    _maxfevals = (*sc)["Alica"]->get<int>("Alica", "CSPSolving", "MaxFunctionEvaluations", NULL);
    _maxSolveTime = AlicaTime::milliseconds((*sc)["Alica"]->get<int>("Alica", "CSPSolving", "MaxSolveTime", NULL));
    _rPropConvergenceStepSize = 1E-2;

    _alicaClock = new alica::AlicaClock();
}

GSolver::~GSolver() {
    delete _alicaClock;
}

void GSolver::initLog() {
    std::stringstream ss;
    ss << "/tmp/test" << (_fcounter++) << ".dbg";
    string logFile = ss.str();
    sw.open(logFile);
}

void GSolver::log(double util, shared_ptr<vector<double>>& val) {
    sw << util;
    sw << "\t";
    for (int i = 0; i < _dim; ++i) {
        sw << val->at(i);
        sw << "\t";
    }
    sw << endl;
}

void GSolver::logStep() {
    sw << endl;
    sw << endl;
}

void GSolver::closeLog() {
    sw.close();
}

std::shared_ptr<vector<double>> GSolver::solve(std::shared_ptr<Term> equation,
        shared_ptr<vector<shared_ptr<autodiff::Variable>>> args, const std::vector<double>& limits, double* util) {
    return solve(equation, args, limits, make_shared<vector<shared_ptr<vector<double>>>>(),
            numeric_limits<double>::max(), util);
}

bool GSolver::solveSimple(shared_ptr<Term> equation, shared_ptr<vector<shared_ptr<autodiff::Variable>>> args,
        const std::vector<double>& limits) {
    return solveSimple(equation, args, limits, make_shared<vector<shared_ptr<vector<double>>>>());
}

shared_ptr<vector<double>> GSolver::solve(shared_ptr<Term> equation,
        shared_ptr<vector<shared_ptr<autodiff::Variable>>> args, const std::vector<double>& limits,
        shared_ptr<vector<shared_ptr<vector<double>>>> seeds, double sufficientUtility, double* util) {
    //			cout << "GSolver" << endl;
    //			cout << equation->toString() << endl;
    //			cout << args->size() << endl;
    //			for(auto& a : *limits) {
    //				cout << a->at(0) << " " << a->at(1) << endl;
    //			}

    _fevals = 0;
    _runs = 0;
    *util = 0;
    _utilityThreshold = sufficientUtility;

#ifdef GSOLVER_LOG
    initLog();
#endif

    _rResults.clear();
    AlicaTime begin = _alicaClock->now();

    _dim = args->size();

    _ranges = vector<double>(_dim);

    for (int i = 0; i < _dim; ++i) {
        _ranges[i] = (limits[2 * i + 1] - limits[2 * i]);
    }
#ifdef AGGREGATE_CONSTANTS
    equation = equation->aggregateConstants();
#endif
    _term = TermUtils::compile(equation, args);
    shared_ptr<ConstraintUtility> cu = dynamic_pointer_cast<ConstraintUtility>(equation);
    bool utilIsConstant = dynamic_pointer_cast<Constant>(cu->utility) != 0;
    if (utilIsConstant) {
        _utilityThreshold = 0.75;
    }
    bool constraintIsConstant = dynamic_pointer_cast<Constant>(cu->constraint) != 0;
    if (constraintIsConstant) {
        shared_ptr<Constant> constraint = dynamic_pointer_cast<Constant>(cu->constraint);
        if (constraint->value < 0.25) {
            *util = constraint->value;
            auto ret = make_shared<vector<double>>(_dim);
            for (int i = 0; i < _dim; ++i) {
                ret->at(i) = _ranges[i] / 2.0 + limits[2 * i];
            }
            cout << "GSolver ConstraintConstant" << endl;
            return ret;
        }
    }

    // Optimize given seeds
    _rpropStepWidth = vector<double>(_dim);
    _rpropStepConvergenceThreshold = vector<double>(_dim);
    if (seeds->size() > 0) {
        _runs++;
        // Run with prefered cached seed
        shared_ptr<RpropResult> rpfirst = rPropLoop(seeds->at(0), true, limits);
        if (rpfirst->_finalUtil > _utilityThreshold) {
            *util = rpfirst->_finalUtil;
            //					cout << "GSolver: Using Solution from Seed!" << endl;
            return rpfirst->_finalValue;
        }
        _rResults.push_back(rpfirst);
        // run with seeds of all other agends
        for (int i = 1; i < seeds->size(); ++i) {
            if (begin + _maxSolveTime < _alicaClock->now() || _fevals > _maxfevals) {
                break;  // do not check any further seeds
            }
            _runs++;
            shared_ptr<RpropResult> rp = rPropLoop(seeds->at(i), false, limits);
            if (rp->_finalUtil > _utilityThreshold) {
                *util = rp->_finalUtil;
                return rp->_finalValue;
            }
            _rResults.push_back(rp);
        }
    }

    // Here: Ignore all constraints search optimum
    if (begin + _maxSolveTime > _alicaClock->now() && _fevals < _maxfevals) {
        // if time allows, do an unconstrained run
        if (!constraintIsConstant && !utilIsConstant && _seedWithUtilOptimum) {
            shared_ptr<ICompiledTerm> curProb = _term;
            _term = TermUtils::compile(cu->utility, args);
            _runs++;
            shared_ptr<vector<double>> utilitySeed = rPropLoop(make_shared<vector<double>>(), limits)->_finalValue;
            _term = curProb;
            // Take result and search with constraints
            shared_ptr<RpropResult> ru = rPropLoop(utilitySeed, false, limits);
            _rResults.push_back(ru);
        }
    }

    //			if(seeds->size() == 0) {
    do {  // Do runs until termination criteria, running out of time, or too many function evaluations
        _runs++;
        shared_ptr<RpropResult> rp = rPropLoop(make_shared<vector<double>>(), false, limits);
        if (rp->_finalUtil > _utilityThreshold) {
            *util = rp->_finalUtil;
            return rp->_finalValue;
        }
        _rResults.push_back(rp);
    } while (begin + _maxSolveTime > _alicaClock->now() && _fevals < _maxfevals);
    //			}

    // return best result
    int resIdx = 0;
    shared_ptr<RpropResult> res = _rResults[0];
    for (int i = 1; i < _rResults.size(); ++i) {
        if (std::isnan(res->_finalUtil) || _rResults[i]->_finalUtil > res->_finalUtil) {
            if (resIdx == 0 && seeds->size() > 0 && !std::isnan(res->_finalUtil)) {
                if (_rResults[i]->_finalUtil - res->_finalUtil > _utilitySignificanceThreshold &&
                        _rResults[i]->_finalUtil > 0.75) {
                    res = _rResults[i];
                    resIdx = i;
                }
            } else {
                res = _rResults[i];
                resIdx = i;
            }
        }
    }

//			cout << "GSolver: Using Result Nr. " << resIdx << endl;
//			for(int i=0; i < seeds->size(); i++) {
//				cout << "Seed: ";
//				for (double v : *seeds->at(i)) {
//					cout << "\t" << v;
//				}
//				cout << endl;
//			}
#ifdef GSOLVER_LOG
    closeLog();
#endif

    *util = res->_finalUtil;
    //			cout << "GSolver Afterall " << res->_finalUtil << " runs " << _runs << endl;
    return res->_finalValue;
}

bool GSolver::solveSimple(shared_ptr<Term> equation, shared_ptr<vector<shared_ptr<autodiff::Variable>>> args,
        const std::vector<double>& limits, shared_ptr<vector<shared_ptr<vector<double>>>> seeds) {
    _rResults.clear();

    _dim = args->size();

    _ranges = vector<double>(_dim);
    for (int i = 0; i < _dim; ++i) {
        _ranges[i] = limits[2 * i + 1] - limits[2 * i];
    }
    equation = equation->aggregateConstants();

    _term = TermUtils::compile(equation, args);

    _rpropStepWidth = vector<double>(_dim);
    _rpropStepConvergenceThreshold = vector<double>(_dim);
    if (seeds->size() > 0) {
        for (int i = 0; i < seeds->size(); ++i) {
            shared_ptr<RpropResult> r = rPropLoop(seeds->at(i), limits);
            _rResults.push_back(r);
            if (r->_finalUtil > 0.75) {
                return true;
            }
        }
    }
    int runs = 2 * _dim - seeds->size();
    for (int i = 0; i < runs; ++i) {
        shared_ptr<RpropResult> r = rPropLoop(make_shared<vector<double>>(), limits);
        _rResults.push_back(r);
        if (r->_finalUtil > 0.75) {
            return true;
        }
    }
    int adit = 0;
    while (!evalResults() && adit++ < 20) {
        shared_ptr<RpropResult> r = rPropLoop(make_shared<vector<double>>(), limits);
        _rResults.push_back(r);
        if (r->_finalUtil > 0.75) {
            return true;
        }
    }
    if (adit > 20) {
        cerr << "Failed to satisfy heuristic!" << endl;
    }

    return false;
}

shared_ptr<vector<double>> GSolver::solveTest(shared_ptr<Term> equation,
        shared_ptr<vector<shared_ptr<autodiff::Variable>>> args, const std::vector<double>& limits) {
#ifdef GSOLVER_LOG
    initLog();
#endif

    _rResults.clear();
    shared_ptr<vector<double>> res;

    _dim = args->size();
    _ranges = vector<double>(_dim);
    for (int i = 0; i < _dim; ++i) {
        _ranges[i] = limits[2 * i + 1] - limits[2 * i];
    }

    _term = TermUtils::compile(equation, args);

    _rpropStepWidth = vector<double>(_dim);
    _rpropStepConvergenceThreshold = vector<double>(_dim);
    _runs = 0;
    _fevals = 0;
    // int runs = 1000000;
    while (true) {
        _runs++;
        // shared_ptr<RpropResult> r = rPropLoopSimple(vector<double>());
        shared_ptr<RpropResult> r = rPropLoop(make_shared<vector<double>>(), false, limits);
        if (r->_finalUtil > 0.75) {
            res = r->_finalValue;
            break;
        }
    }

#ifdef GSOLVER_LOG
    closeLog();
#endif

    return res;
}

shared_ptr<vector<double>> GSolver::solveTest(shared_ptr<Term> equation,
        shared_ptr<vector<shared_ptr<autodiff::Variable>>> args, const std::vector<double>& limits, int maxRuns,
        bool* found) {
#ifdef GSOLVER_LOG
    initLog();
#endif
    *found = false;
    _rResults.clear();
    shared_ptr<vector<double>> res;

    _dim = args->size();

    _ranges = vector<double>(_dim);
    for (int i = 0; i < _dim; ++i) {
        _ranges[i] = limits[2 * i + 1] - limits[2 * i];
    }

    _term = TermUtils::compile(equation, args);

    _rpropStepWidth = vector<double>(_dim);
    _rpropStepConvergenceThreshold = vector<double>(_dim);
    _runs = 0;
    _fevals = 0;

    while (_runs < maxRuns) {
        _runs++;
        // shared_ptr<RpropResult> r = rPropLoopSimple(vector<double>());
        shared_ptr<RpropResult> r = rPropLoop(make_shared<vector<double>>(), false, limits);
        if (r->_finalUtil > 0.75) {
            res = r->_finalValue;
            *found = true;
            break;
        }
    }

#ifdef GSOLVER_LOG
    closeLog();
#endif

    return res;
}

long GSolver::getRuns() {
    return _runs;
}

void GSolver::setRuns(long runs) {
    _runs = runs;
}

long GSolver::getFEvals() {
    return _fevals;
}

void GSolver::setFEvals(long fevals) {
    _fevals = fevals;
}

long GSolver::getMaxFEvals() {
    return _maxfevals;
}

void GSolver::setMaxFEvals(long maxfevals) {
    _maxfevals = maxfevals;
}

double GSolver::getRPropConvergenceStepSize() {
    return _rPropConvergenceStepSize;
}

void GSolver::setRPropConvergenceStepSize(double rPropConvergenceStepSize) {
    _rPropConvergenceStepSize = rPropConvergenceStepSize;
}

void GSolver::setUtilitySignificanceThreshold(double utilitySignificanceThreshold) {
    _utilitySignificanceThreshold = utilitySignificanceThreshold;
}

AlicaClock* GSolver::getAlicaClock() {
    return _alicaClock;
}

void GSolver::setAlicaClock(AlicaClock* clock) {
    _alicaClock = clock;
}

shared_ptr<vector<double>> GSolver::initialPointFromSeed(
        shared_ptr<RpropResult>& res, shared_ptr<vector<double>>& seed, const std::vector<double>& limits) {
    std::pair<shared_ptr<vector<double>>, double> tup;

    res->_initialValue = make_shared<vector<double>>(_dim);
    res->_finalValue = make_shared<vector<double>>(_dim);
    for (int i = 0; i < _dim; ++i) {
        if (std::isnan(seed->at(i))) {
            res->_initialValue->at(i) = ((double) rand() / RAND_MAX) * _ranges[i] + limits[2 * i];
        } else {
            res->_initialValue->at(i) = min(max(seed->at(i), limits[2 * i]), limits[2 * i + 1]);
        }
    }
    tup = _term->differentiate(res->_initialValue);
    res->_initialUtil = tup.second;
    res->_finalUtil = tup.second;

    res->_finalValue = res->_initialValue;

    return tup.first;
}

shared_ptr<vector<double>> GSolver::initialPoint(shared_ptr<RpropResult>& res, const std::vector<double>& limits) {
    pair<shared_ptr<vector<double>>, double> tup;
    bool found = true;
    res->_initialValue = make_shared<vector<double>>(_dim);
    res->_finalValue = make_shared<vector<double>>(_dim);
    do {
        for (int i = 0; i < _dim; ++i) {
            res->_initialValue->at(i) = ((double) rand() / RAND_MAX) * _ranges[i] + limits[2 * i];
            //					cout << " << i << "\t" << _ranges[i] << "\t" << _limits->at(i)->at(0) <<
            //"\t"
            //<<  res->_initialValue->at(i) << endl;
            //					cout << "if (TMPVAR == " << TMPVAR++ << ") { res.initialValue[i] = " <<
            // res->_initialValue->at(i) << "; }" << endl;
        }
        _fevals++;
        tup = _term->differentiate(res->_initialValue);
        for (int i = 0; i < _dim; ++i) {
            if (std::isnan(tup.first->at(i))) {
                found = false;
                break;
            } else {
                found = true;
            }
        }
    } while (!found);
    res->_initialUtil = tup.second;
    res->_finalUtil = tup.second;

    res->_finalValue = res->_initialValue;

    return tup.first;
}

shared_ptr<GSolver::RpropResult> GSolver::rPropLoop(
        shared_ptr<vector<double>> seed, const std::vector<double>& limits) {
    return rPropLoop(seed, false, limits);
}

shared_ptr<GSolver::RpropResult> GSolver::rPropLoop(
        shared_ptr<vector<double>> seed, bool precise, const std::vector<double>& limits) {
    initialStepSize();

    shared_ptr<vector<double>> curGradient;

    shared_ptr<RpropResult> ret = make_shared<RpropResult>();

    if (seed->size() > 0) {
        curGradient = initialPointFromSeed(ret, seed, limits);
    } else {
        curGradient = initialPoint(ret, limits);
    }
    double curUtil = ret->_initialUtil;

    auto formerGradient = make_shared<vector<double>>(_dim);
    auto curValue = make_shared<vector<double>>(_dim);

    pair<shared_ptr<vector<double>>, double> tup;

    curValue = ret->_initialValue;

    formerGradient = curGradient;

    int itcounter = 0;
    int badcounter = 0;

#ifdef GSOLVER_LOG
    log(curUtil, curValue);
#endif

    int maxIter = 60;
    int maxBad = 30;
    double minStep = 1E-11;
    if (precise) {
        maxIter = 120;    // 110
        maxBad = 60;      // 60
        minStep = 1E-15;  // 15
    }
    int convergendDims = 0;

    while (itcounter++ < maxIter && badcounter < maxBad) {
        convergendDims = 0;
        for (int i = 0; i < _dim; ++i) {
            if (curGradient->at(i) * formerGradient->at(i) > 0)
                _rpropStepWidth[i] *= 1.3;
            else if (curGradient->at(i) * formerGradient->at(i) < 0)
                _rpropStepWidth[i] *= 0.5;
            _rpropStepWidth[i] = max(minStep, _rpropStepWidth[i]);
            if (curGradient->at(i) > 0)
                curValue->at(i) += _rpropStepWidth[i];
            else if (curGradient->at(i) < 0)
                curValue->at(i) -= _rpropStepWidth[i];

            if (curValue->at(i) > limits[2 * i + 1])
                curValue->at(i) = limits[2 * i + 1];
            else if (curValue->at(i) < limits[2 * i])
                curValue->at(i) = limits[2 * i];
            if (_rpropStepWidth[i] < _rpropStepConvergenceThreshold[i])
                ++convergendDims;
        }

        if (!precise && convergendDims >= _dim) {
            if (curUtil > ret->_finalUtil) {
                ret->_finalUtil = curUtil;
                ret->_finalValue = curValue;
            }

            return ret;
        }

        _fevals++;
        tup = _term->differentiate(curValue);
        bool allZero = true;
        for (int i = 0; i < _dim; ++i) {
            if (std::isnan(tup.first->at(i))) {
                ret->_aborted = true;
#ifdef GSOLVER_LOG
                logStep();
#endif
                return ret;
            }
            allZero &= (tup.first->at(i) == 0);
        }

        curUtil = tup.second;
        formerGradient = curGradient;
        curGradient = tup.first;
#ifdef GSOLVER_LOG
        log(curUtil, curValue);
#endif

        if (curUtil > ret->_finalUtil) {
            badcounter = 0;

            ret->_finalUtil = curUtil;
            ret->_finalValue = curValue;

#ifdef ALWAYS_CHECK_THRESHOLD
            if (curUtil > _utilityThreshold) {
                return ret;
            }
#endif
        } else {
            badcounter++;
        }
        if (allZero) {
            ret->_aborted = false;
#ifdef GSOLVER_LOG
            logStep();
#endif
            return ret;
        }
    }
#ifdef GSOLVER_LOG
    logStep();
#endif
    ret->_aborted = false;
    return ret;
}

shared_ptr<GSolver::RpropResult> GSolver::rPropLoopSimple(
        shared_ptr<vector<double>> seed, const std::vector<double>& limits) {
    initialStepSize();
    shared_ptr<vector<double>> curGradient;

    shared_ptr<RpropResult> ret = make_shared<RpropResult>();

    if (seed->size() > 0) {
        curGradient = initialPointFromSeed(ret, seed, limits);
    } else {
        curGradient = initialPoint(ret, limits);
    }
    double curUtil = ret->_initialUtil;

    if (ret->_initialUtil > 0.75) {
        return ret;
    }

    auto formerGradient = make_shared<vector<double>>(_dim);
    auto curValue = make_shared<vector<double>>(_dim);

    pair<shared_ptr<vector<double>>, double> tup;

    curValue = ret->_initialValue;

    formerGradient = curGradient;

    int itcounter = 0;
    int badcounter = 0;

    while (itcounter++ < 40 && badcounter < 6) {
        for (int i = 0; i < _dim; ++i) {
            if (curGradient->at(i) * formerGradient->at(i) > 0)
                _rpropStepWidth[i] *= 1.3;
            else if (curGradient->at(i) * formerGradient->at(i) < 0)
                _rpropStepWidth[i] *= 0.5;
            _rpropStepWidth[i] = max(0.0001, _rpropStepWidth[i]);
            if (curGradient->at(i) > 0)
                curValue->at(i) += _rpropStepWidth[i];
            else if (curGradient->at(i) < 0)
                curValue->at(i) -= _rpropStepWidth[i];

            if (curValue->at(i) > limits[2 * i + 1])
                curValue->at(i) = limits[2 * i + 1];
            else if (curValue->at(i) < limits[2 * i])
                curValue->at(i) = limits[2 * i];
        }

        tup = _term->differentiate(curValue);
        bool allZero = true;

        for (int i = 0; i < _dim; ++i) {
            if (std::isnan(tup.first->at(i))) {
                ret->_aborted = false;
                return ret;
            }
            allZero &= (tup.first->at(i) == 0);
        }

        curUtil = tup.second;
        formerGradient = curGradient;
        curGradient = tup.first;

        if (curUtil > ret->_finalUtil) {
            badcounter = 0;

            ret->_finalUtil = curUtil;
            ret->_finalValue = curValue;

            if (curUtil > 0.75) {
                return ret;
            }
        } else {
            badcounter++;
        }
        if (allZero) {
            ret->_aborted = false;
            return ret;
        }
    }

    ret->_aborted = false;
    return ret;
}

void GSolver::initialStepSize() {
    for (int i = 0; i < _dim; ++i) {
        _rpropStepWidth[i] = _initialStepSize * _ranges[i];
        _rpropStepConvergenceThreshold[i] = _rpropStepWidth[i] * _rPropConvergenceStepSize;
    }
}

bool GSolver::evalResults() {
    /*
     * TODO: Compare Average Distance between start and end point to
     * Average distance between any two arbitrary points.
     *
     * Latter one can be found in http://www.math.uni-muenster.de/reine/u/burgstal/d18.pdf :
     * maxDist = sqrt(dim)
     * avgDist <= 1 / sqrt(6) * sqrt((1+2*sqrt(1-3/(5*dim)))/3) * maxDist
     *
     * aprox: (http://www.jstor.org/pss/1427094)
     *
     * avgDist = sqrt(dim/3) * (1 - 1/(10*k) - 13/(280*k^2) - 101/ (2800*k^3) - 37533 / (1232000 k^4) * O(sqrt(k)) (?)
     * */
    int count = _rResults.size();

    int abortedCount = 0;

    vector<double> midInitValue = vector<double>(_dim);

    vector<double> valueInitDev = vector<double>(_dim);

    for (int i = 0; i < count; ++i) {
        if (_rResults[i]->_aborted) {
            abortedCount++;
        } else {
            for (int j = 0; j < _dim; ++j) {
                midInitValue[j] += _rResults[i]->_initialValue->at(j);
            }
        }
    }
    if (count - abortedCount < _dim) {
        return false;
    }
    for (int j = 0; j < _dim; ++j) {
        midInitValue[j] /= count - abortedCount;
    }
    for (int i = 0; i < count; ++i) {
        if (_rResults[i]->_aborted) {
            continue;
        }
        for (int j = 0; j < _dim; ++j) {
            valueInitDev[j] += pow((_rResults[i]->_initialValue->at(j) - midInitValue[j]) / _ranges[j], 2);
        }
    }
    for (int j = 0; j < _dim; ++j) {
        valueInitDev[j] /= count - abortedCount;
        if (valueInitDev[j] < 0.0441) {
            return false;
        }
    }

    return true;
}

int GSolver::RpropResult::compareTo(shared_ptr<RpropResult> other) {
    return _finalUtil > other->_finalUtil ? -1 : 1;
}

double GSolver::RpropResult::distanceTraveled() {
    double ret = 0;
    for (int i = 0; i < _initialValue->size(); ++i) {
        ret += (_initialValue->at(i) - _finalValue->at(i)) * (_initialValue->at(i) - _finalValue->at(i));
    }
    return sqrt(ret);
}

double GSolver::RpropResult::distanceTraveledNormed(vector<double> ranges) {
    double ret = 0;
    for (int i = 0; i < _initialValue->size(); ++i) {
        ret += (_initialValue->at(i) - _finalValue->at(i)) * (_initialValue->at(i) - _finalValue->at(i)) /
               (ranges[i] * ranges[i]);
    }
    return sqrt(ret);
}

//		bool GSolver::RpropResult::operator<(const shared_ptr<GSolver::RpropResult>& left, const
// shared_ptr<GSolver::RpropResult>& right)
//		{
//			return left->_finalUtil < right->_finalUtil;
//		}
//
//		bool GSolver::RpropResult::operator>(const shared_ptr<GSolver::RpropResult>& left, const
// shared_ptr<GSolver::RpropResult>& right)
//		{
//			return left->_finalUtil > right->_finalUtil;
//		}
} /* namespace reasoner */
} /* namespace alica */
