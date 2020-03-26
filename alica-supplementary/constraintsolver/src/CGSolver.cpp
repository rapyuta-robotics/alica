#include "constraintsolver/CGSolver.h"

#include "constraintsolver/GSolver.h"

#include <engine/AlicaEngine.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/constraintmodul/VariableSyncModule.h>
#include <engine/model/Variable.h>

#include <autodiff/Term.h>
#include <autodiff/TermHolder.h>
#include <autodiff/Variable.h>

#include <limits>

#include <iostream>

namespace alica
{
namespace reasoner
{
using alica::VariableGrp;
using autodiff::TermHolder;
using autodiff::TermPtr;

CGSolver::CGSolver(AlicaEngine* ae)
        : ISolver(ae)
        , _lastUtil(0.0)
        , _lastFEvals(0.0)
        , _lastRuns(0.0)
{
    autodiff::Term::setAnd(autodiff::AndType::AND);
    autodiff::Term::setOr(autodiff::OrType::MAX);
}

CGSolver::~CGSolver() {}

bool CGSolver::existsSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls)
{
    TermHolder* holder = static_cast<TermHolder*>(ctx);
    TermPtr constraint = holder->trueConstant();
    const int dim = holder->getDim();

    std::vector<Interval<double>> ranges(dim, Interval<double>(std::numeric_limits<double>::lowest() / 2, std::numeric_limits<double>::max() / 2));

    int i = 0;
    for (const autodiff::Variable* v : holder->getVariables()) {
        if (!v->getRange().isValid()) {
            return false;
        }
        ranges[i] = v->getRange();
        ++i;
    }

    for (const std::shared_ptr<ProblemDescriptor>& c : calls) {
        if (dynamic_cast<autodiff::Term*>(c->getConstraint()) == nullptr) {
            std::cerr << "CGSolver: Constraint type not compatible with selected solver" << std::endl;
            return false;
        }
        constraint = constraint & TermPtr(static_cast<autodiff::Term*>(c->getConstraint()));
    }

    std::vector<Variant> serial_seeds;
    int seed_num = getAlicaEngine()->getResultStore().getSeeds(holder->getVariables(), ranges, serial_seeds);

    std::vector<double> seeds;
    seeds.reserve(seed_num * dim);
    std::transform(serial_seeds.begin(), serial_seeds.end(), std::back_inserter(seeds),
            [](Variant v) -> double { return v.isDouble() ? v.getDouble() : std::numeric_limits<double>::quiet_NaN(); });

    return _sgs.solveSimple(constraint, *holder, ranges, seeds);
}

bool CGSolver::getSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<double>& results)
{
    TermHolder* holder = static_cast<TermHolder*>(ctx);
    TermPtr constraint = holder->trueConstant();
    TermPtr utility = holder->constant(1);
    const int dim = holder->getDim();

    // create lists of constraint variables and corresponding ranges

    std::vector<Interval<double>> ranges(dim, Interval<double>(std::numeric_limits<double>::lowest() / 2, std::numeric_limits<double>::max() / 2));

    int i = 0;
    for (const autodiff::Variable* v : holder->getVariables()) {
        if (!v->getRange().isValid()) {
            std::cerr << "CGSolver: Ranges do not allow a solution!" << std::endl;
            return false;
        }
        ranges[i] = v->getRange();
        ++i;
    }
    // get some utility significance threshold value if one exists
    double usigVal = calls[0]->getUtilitySignificanceThreshold();
    for (int i = 1; i < static_cast<int>(calls.size()); ++i) {
        // TODO: fixed Values
        if (calls.at(i)->isSettingUtilitySignificanceThreshold()) {
            usigVal = calls[i]->getUtilitySignificanceThreshold();
        }
    }

    double sufficientUtility = 0.0;

    for (const std::shared_ptr<ProblemDescriptor>& c : calls) {
        TermPtr constraintTerm = dynamic_cast<autodiff::Term*>(c->getConstraint());
        if (constraintTerm.get() == nullptr) {
            std::cerr << "CGSolver: Constraint type not compatible with selected solver" << std::endl;
            return false;
        }
        constraint = constraint & constraintTerm;

        TermPtr utilityTerm = dynamic_cast<autodiff::Term*>(c->getUtility());
        if (utilityTerm.get() == nullptr) {
            std::cerr << "CGSolver: Utility type not compatible with selected solver" << std::endl;
            return false;
        }
        utility = utility + utilityTerm;

        sufficientUtility += c->getUtilitySufficiencyThreshold();
    }
    TermPtr all = holder->constraintUtility(constraint, utility);

    std::vector<Variant> serial_seeds;
    int seed_num = getAlicaEngine()->getResultStore().getSeeds(holder->getVariables(), ranges, serial_seeds);

    std::vector<double> seeds;
    seeds.reserve(seed_num * dim);
    std::transform(serial_seeds.begin(), serial_seeds.end(), std::back_inserter(seeds),
            [](Variant v) -> double { return v.isDouble() ? v.getDouble() : std::numeric_limits<double>::quiet_NaN(); });

#ifdef CGSolver_DEBUG
    for (int i = 0; i < seeds.size(); i += dim) {
        std::cout << "----CGS: seed " << (i / dim) << " ";
        for (int j = 0; j < dim; ++j) {
            std::cout << seeds[i + j] << " ";
        }
        std::cout << endl;
    }
#endif

    double util = 0;
    bool solved;
    { // for lock_guard
        std::lock_guard<std::mutex> lock(_mtx);
        _gs.setUtilitySignificanceThreshold(usigVal);
        solved = _gs.solve(all, *holder, ranges, seeds, sufficientUtility, util, results);
    }

    _lastUtil = util;
    _lastFEvals = _gs.getFEvals();
    _lastRuns = _gs.getRuns();
#ifdef CGSolver_DEBUG
    std::cout << "CGS: result ";
    for (int i = 0; i < results.size(); ++i) {
        std::cout << results[i] << " ";
    }
    std::cout << std::endl;
#endif
    return solved;
}

SolverVariable* CGSolver::createVariable(int64_t id, SolverContext* ctx)
{
    return static_cast<TermHolder*>(ctx)->createVariable(id);
}
std::unique_ptr<SolverContext> CGSolver::createSolverContext()
{
    return std::unique_ptr<SolverContext>(new TermHolder());
}
} // namespace reasoner
} // namespace alica
