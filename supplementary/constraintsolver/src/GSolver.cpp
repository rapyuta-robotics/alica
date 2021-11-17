
#include "constraintsolver/GSolver.h"
//#define GSOLVER_LOG
//#define ALWAYS_CHECK_THRESHOLD
#define AGGREGATE_CONSTANTS

//#include "essentials/Configuration.h"
//#include "essentials/SystemConfig.h"

#include <engine/AlicaClock.h>

#include <autodiff/Constant.h>
#include <autodiff/ConstraintUtility.h>
#include <autodiff/Tape.h>
#include <autodiff/Term.h>
#include <autodiff/TermPtr.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdlib.h> /* srand, rand */
#include <string>

#include <iostream>

namespace alica
{
namespace reasoner
{
using autodiff::Constant;
using autodiff::ConstraintUtility;
using autodiff::Tape;
#ifdef GSOLVER_LOG
int GSolver::_fcounter = 0;
#endif

GSolver::GSolver(YAML::Node config)
        : _seedWithUtilOptimum(true)
        , _rPropConvergenceStepSize(1E-2)
        , _utilitySignificanceThreshold(1E-22)
        , _initialStepSize(0.005)
        , _utilityThreshold(1.0)
        , _runs(0)
        , _fevals(0)
{
    autodiff::Term::setAnd(autodiff::AndType::AND);
    autodiff::Term::setOr(autodiff::OrType::MAX);

//    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    _maxfevals = config["Alica"]["CSPSolving"]["MaxFunctionEvaluations"].as<int>();
//    _maxfevals = sc["Alica"]->get<int>("Alica", "CSPSolving", "MaxFunctionEvaluations", NULL);
    _maxSolveTime = AlicaTime::milliseconds(config["Alica"]["CSPSolving"]["MaxSolveTime"].as<int>());
//    _maxSolveTime = AlicaTime::milliseconds(sc["Alica"]->get<int>("Alica", "CSPSolving", "MaxSolveTime", NULL));
}

GSolver::~GSolver() {}
#ifdef GSOLVER_LOG
void GSolver::initLog()
{
    std::stringstream ss;
    ss << "/tmp/test" << (_fcounter++) << ".dbg";
    std::string logFile = ss.str();
    sw.open(logFile);
}

void GSolver::log(double util, const double* val)
{
    sw << util;
    sw << "\t";
    for (int i = 0; i < val.dim(); ++i) {
        sw << val[i];
        sw << "\t";
    }
    sw << std::endl;
}

void GSolver::logStep()
{
    sw << std::endl;
    sw << std::endl;
}

void GSolver::closeLog()
{
    sw.close();
}
#endif

bool GSolver::solve(autodiff::TermPtr equation, autodiff::TermHolder& holder, const std::vector<Interval<double>>& limits, double& out_util,
        std::vector<double>& o_solution)
{
    return solve(equation, holder, limits, std::vector<double>(), std::numeric_limits<double>::max(), out_util, o_solution);
}

bool GSolver::solve(autodiff::TermPtr equation, autodiff::TermHolder& holder, const std::vector<Interval<double>>& limits, const std::vector<double>& seeds,
        double sufficientUtility, double& out_util, std::vector<double>& o_solution)
{
    _fevals = 0;
    _runs = 0;
    out_util = 0;
    _utilityThreshold = sufficientUtility;

#ifdef GSOLVER_LOG
    initLog();
#endif

    _results.clear();
    AlicaTime begin = _alicaClock.now();

    const int dim = holder.getDim();

#ifdef AGGREGATE_CONSTANTS
    equation = equation->aggregateConstants();
#endif
    holder.compile(equation);

    assert(dynamic_cast<ConstraintUtility*>(equation.get()) != nullptr);

    ConstraintUtility* cu = static_cast<ConstraintUtility*>(equation.get());

    const bool utilIsConstant = cu->getRight()->isConstant();
    if (utilIsConstant) {
        _utilityThreshold = 0.75;
    }
    const bool constraintIsConstant = cu->getLeft()->isConstant();
    if (constraintIsConstant) {
        Constant* constraint = static_cast<Constant*>(cu->getLeft().get());
        if (constraint->getValue() < 0.25) {
            out_util = constraint->getValue();
            o_solution.resize(dim);
            for (int i = 0; i < dim; ++i) {
                o_solution[i] = limits[i].getMidPoint();
            }
            // std::cout << "GSolver ConstraintConstant" << std::endl;
            return false;
        }
    }

    // Optimize given seeds
    _rpropStepWidth.resize(dim);
    _rpropStepConvergenceThreshold.resize(dim);
    if (!seeds.empty()) {
        const int seedCount = seeds.size() / dim;
        ensureResultSpace(seedCount, dim);
        ++_runs;
        // Run with prefered cached seed
        ResultView rpfirst = getResultView(0, dim);
        rPropLoop(holder.getTape(), &seeds[0], limits, rpfirst, true);
        if (rpfirst.getUtil() > _utilityThreshold) {
            out_util = rpfirst.getUtil();
            writeSolution(rpfirst, o_solution);
            return true;
        }

        // run with seeds of all other agends

        for (int i = 1; i < seedCount; ++i) {
            if (begin + _maxSolveTime < _alicaClock.now() || _fevals > _maxfevals) {
                break; // do not check any further seeds
            }
            ResultView res = getResultView(i, dim);
            ++_runs;
            rPropLoop(holder.getTape(), &seeds[i * dim], limits, res, false);
            if (res.getUtil() > _utilityThreshold) {
                out_util = res.getUtil();
                writeSolution(res, o_solution);
                return true;
            }
        }
    }

    // Here: Ignore all constraints search optimum
    if (begin + _maxSolveTime > _alicaClock.now() && _fevals < _maxfevals) {
        ensureResultSpace(_runs + 2, dim); // for the next two runs
        // if time allows, do an unconstrained run
        if (!constraintIsConstant && !utilIsConstant && _seedWithUtilOptimum) {
            Tape curProb = holder.compileSeparately(cu->getRight());
            ResultView res = getResultView(_runs, dim);
            ++_runs;
            rPropLoop(curProb, nullptr, limits, res);
            // Take result and search with constraints
            rPropLoop(holder.getTape(), res.getPoint(), limits, res, false);
        }
    }

    do { // Do runs until termination criteria, running out of time, or too many function evaluations
        ensureResultSpace(_runs + 1, dim);
        ResultView res = getResultView(_runs, dim);
        ++_runs;

        rPropLoop(holder.getTape(), nullptr, limits, res, false);
        if (res.getUtil() > _utilityThreshold) {
            out_util = res.getUtil();
            writeSolution(res, o_solution);
            return true;
        }
    } while (begin + _maxSolveTime > _alicaClock.now() && _fevals < _maxfevals);

    // find the best result
    int resIdx = 0;
    ResultView res = getResultView(0, dim); // take the first result
    for (int i = 1; i < _runs; ++i) {
        ResultView next = getResultView(i, dim);
        if (std::isnan(res.getUtil()) || next.getUtil() > res.getUtil()) {     // better one is found
            if (resIdx == 0 && !seeds.empty() && !std::isnan(res.getUtil())) { // hysteresis towards first result
                if (next.getUtil() - res.getUtil() > _utilitySignificanceThreshold && next.getUtil() > 0.75) {
                    res = next;
                    resIdx = i;
                }
            } else {
                res = next;
                resIdx = i;
            }
        }
    }

#ifdef GSOLVER_LOG
    closeLog();
#endif

    out_util = res.getUtil();
    writeSolution(res, o_solution);
    return res.getUtil() > 0.75;
}
bool GSolver::solveSimple(autodiff::TermPtr equation, autodiff::TermHolder& holder, const std::vector<Interval<double>>& limits)
{
    return solveSimple(equation, holder, limits, std::vector<double>());
}

bool GSolver::solveSimple(
        autodiff::TermPtr equation, autodiff::TermHolder& holder, const std::vector<Interval<double>>& limits, const std::vector<double>& seeds)
{
    _results.clear();

    const int dim = holder.getDim();

#ifdef AGGREGATE_CONSTANTS
    equation = equation->aggregateConstants();
#endif

    holder.compile(equation);

    _rpropStepWidth.resize(dim);
    _rpropStepConvergenceThreshold.resize(dim);
    const int seedCount = seeds.size() / dim;
    if (!seeds.empty()) {

        ensureResultSpace(seedCount, dim);
        for (int i = 0; i < seedCount; ++i) {
            ResultView res = getResultView(i, dim);
            rPropLoop(holder.getTape(), &seeds[i * dim], limits, res);
            if (res.getUtil() > 0.75) {
                return true;
            }
        }
    }
    int runs = 2 * dim;
    ensureResultSpace(runs, dim);
    for (int i = seedCount; i < runs; ++i) {
        ResultView res = getResultView(i, dim);
        rPropLoop(holder.getTape(), nullptr, limits, res);
        if (res.getUtil() > 0.75) {
            return true;
        }
    }
    int adit = 0;

    while (!evalResults(runs, dim, limits) && ++adit < 21) {
        ++runs;
        ensureResultSpace(runs, dim);
        ResultView res = getResultView(runs - 1, dim);
        rPropLoop(holder.getTape(), nullptr, limits, res);
        if (res.getUtil() > 0.75) {
            return true;
        }
    }
    if (adit > 20) {
        std::cerr << "Failed to satisfy heuristic!" << std::endl;
    }

    return false;
}
inline void GSolver::ensureResultSpace(int count, int dim)
{
    int required = ResultView::getRequiredSize(dim) * count;
    if (static_cast<int>(_results.size()) < required) {
        _results.resize(required);
    }
}
void GSolver::writeSolution(ResultView result, std::vector<double>& o_solution) const
{
    o_solution.resize(result.dim());
    std::copy(result.getPoint(), result.getPoint() + result.dim(), o_solution.begin());
}

inline GSolver::ResultView GSolver::getResultView(int num, int dim)
{
    int idx = ResultView::getRequiredSize(dim) * num;
    assert(static_cast<int>(_results.size()) >= idx + ResultView::getRequiredSize(dim));
    return ResultView(&_results[idx], dim);
}

void GSolver::initialPointFromSeed(
        const autodiff::Tape& tape, const double* seed, ResultView o_res, const std::vector<Interval<double>>& limits, double* o_value) const
{
    for (int i = 0; i < o_res.dim(); ++i) {
        if (std::isnan(seed[i])) {
            o_res.editPoint()[i] = ((double) rand() / RAND_MAX) * limits[i].size() + limits[i].getMin();
        } else {
            o_res.editPoint()[i] = limits[i].clamp(seed[i]);
        }
    }
    tape.evaluate(o_res.getPoint(), o_value);
    o_res.setUtil(o_value[0]);
}

void GSolver::initialPoint(const autodiff::Tape& tape, ResultView o_res, const std::vector<Interval<double>>& limits, double* o_value)
{
    const int dim = o_res.dim();
    bool found;
    do {
        for (int i = 0; i < dim; ++i) {
            o_res.editPoint()[i] = ((double) rand() / RAND_MAX) * limits[i].size() + limits[i].getMin();
        }
        ++_fevals;
        tape.evaluate(o_res.getPoint(), o_value);
        found = true;
        for (int i = 0; i < dim + 1; ++i) {
            if (std::isnan(o_value[i])) {
                found = false;
                break;
            }
        }
    } while (!found);
    o_res.setUtil(o_value[0]);
}

void GSolver::rPropLoop(const autodiff::Tape& tape, const double* seed, const std::vector<Interval<double>>& limits, ResultView o_result)
{
    return rPropLoop(tape, seed, limits, o_result, false);
}

void GSolver::rPropLoop(const autodiff::Tape& tape, const double* seed, const std::vector<Interval<double>>& limits, ResultView o_result, bool precise)
{
    const int dim = o_result.dim();
    initialStepSize(dim, limits);

    double* curGradient = static_cast<double*>(alloca(sizeof(double) * (dim + 1)));
    double* formerGradient = static_cast<double*>(alloca(sizeof(double) * (dim + 1)));

    double* pointBuffer = static_cast<double*>(alloca(sizeof(double) * dim));

    if (seed != nullptr) {
        initialPointFromSeed(tape, seed, o_result, limits, curGradient);
    } else {
        initialPoint(tape, o_result, limits, curGradient);
    }

    memcpy(formerGradient, curGradient, sizeof(double) * (dim + 1));
    memcpy(pointBuffer, o_result.getPoint(), sizeof(double) * dim);
    double curUtil = o_result.getUtil();
    int itcounter = 0;
    int badcounter = 0;

#ifdef GSOLVER_LOG
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

    while (itcounter++ < maxIter && badcounter < maxBad) {
        int convergedDims = movePoint(dim, minStep, pointBuffer, curGradient, formerGradient, limits);

        if (!precise && convergedDims >= dim) {
            if (curUtil > o_result.getUtil()) {
                o_result.setUtil(curUtil);
                memcpy(o_result.editPoint(), pointBuffer, sizeof(double) * dim);
            }
            return;
        }

        ++_fevals;
        std::swap(curGradient, formerGradient);
        tape.evaluate(pointBuffer, curGradient);
        bool allZero = true;
        for (int i = 1; i <= dim; ++i) {
            if (std::isnan(curGradient[i])) {
                o_result.setAborted();
#ifdef GSOLVER_LOG
                logStep();
#endif
                return;
            }
            allZero = allZero && (curGradient[i] == 0);
        }
        curUtil = curGradient[0];

#ifdef GSOLVER_LOG
        log(curUtil, pointBuffer);
#endif
        if (curUtil > o_result.getUtil()) {
            badcounter = 0;
            o_result.setUtil(curUtil);
            memcpy(o_result.editPoint(), pointBuffer, sizeof(double) * dim);

#ifdef ALWAYS_CHECK_THRESHOLD
            if (curUtil > _utilityThreshold) {
                return;
            }
#endif
        } else {
            ++badcounter;
        }
        if (allZero) {
            o_result.setAborted();
#ifdef GSOLVER_LOG
            logStep();
#endif
            return;
        }
    }
#ifdef GSOLVER_LOG
    logStep();
#endif
    o_result.setOk();
}

void GSolver::initialStepSize(int dim, const std::vector<Interval<double>>& limits)
{
    for (int i = 0; i < dim; ++i) {
        _rpropStepWidth[i] = _initialStepSize * limits[i].size();
        _rpropStepConvergenceThreshold[i] = _rpropStepWidth[i] * _rPropConvergenceStepSize;
    }
}

bool GSolver::evalResults(int numResults, int dim, const std::vector<Interval<double>>& limits)
{

    int nonaborted = numResults;

    double* midValue = static_cast<double*>(alloca(sizeof(double) * dim));
    double* valueDev = static_cast<double*>(alloca(sizeof(double) * dim));

    memset(midValue, 0, sizeof(double) * dim);
    memset(valueDev, 0, sizeof(double) * dim);

    for (int i = 0; i < numResults; ++i) {
        ResultView r = getResultView(i, dim);
        if (r.isAborted()) {
            --nonaborted;
        } else {
            for (int j = 0; j < dim; ++j) {
                midValue[j] += r.getPoint()[j];
            }
        }
    }

    if (nonaborted < dim) {
        return false;
    }

    for (int j = 0; j < dim; ++j) {
        midValue[j] /= nonaborted;
    }
    for (int i = 0; i < numResults; ++i) {
        ResultView r = getResultView(i, dim);
        if (r.isAborted()) {
            continue;
        }
        for (int j = 0; j < dim; ++j) {
            valueDev[j] += pow((r.getPoint()[j] - midValue[j]) / limits.size(), 2);
        }
    }
    for (int j = 0; j < dim; ++j) {
        valueDev[j] /= nonaborted;
        if (valueDev[j] < 0.0441) {
            return false;
        }
    }

    return true;
}

int GSolver::movePoint(
        int dim, double minStep, double* pointBuffer, const double* curGradient, const double* oldGradient, const std::vector<Interval<double>>& limits)
{
    int converged = 0;
    for (int i = 0; i < dim; ++i) {
        if (curGradient[i + 1] * oldGradient[i + 1] > 0) {
            _rpropStepWidth[i] *= 1.3;
        } else if (curGradient[i + 1] * oldGradient[i + 1] < 0) {
            _rpropStepWidth[i] *= 0.5;
            _rpropStepWidth[i] = std::max(minStep, _rpropStepWidth[i]);
        }
        if (curGradient[i + 1] > 0) {
            pointBuffer[i] += _rpropStepWidth[i];
        } else if (curGradient[i + 1] < 0) {
            pointBuffer[i] -= _rpropStepWidth[i];
        }
        pointBuffer[i] = limits[i].clamp(pointBuffer[i]);
        if (_rpropStepWidth[i] < _rpropStepConvergenceThreshold[i]) {
            ++converged;
        }
    }
    return converged;
}

} /* namespace reasoner */
} /* namespace alica */
