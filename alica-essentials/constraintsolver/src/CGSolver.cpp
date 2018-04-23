#include "CGSolver.h"

#include <GSolver.h>
#include <engine/AlicaEngine.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/constraintmodul/VariableSyncModule.h>
#include <engine/model/Variable.h>
#include <limits>

#include <iostream>

namespace alica
{
namespace reasoner
{
using alica::VariableGrp;

CGSolver::CGSolver(AlicaEngine* ae)
    : ISolver(ae)
    , _lastUtil(0.0)
    , _lastFEvals(0.0)
    , _lastRuns(0.0)
{
    Term::setAnd(AndType::AND);
    Term::setOr(OrType::MAX);
}

CGSolver::~CGSolver() {}

bool CGSolver::existsSolutionImpl(const VariableGrp& vars, const std::vector<shared_ptr<ProblemDescriptor>>& calls)
{
    std::shared_ptr<Term> constraint = ConstraintBuilder::TRUE;
    const int dim = vars.size();

    auto cVars = make_shared<vector<shared_ptr<autodiff::Variable>>>(dim);
    std::vector<double> ranges(dim * 2, std::numeric_limits<double>::max() / 2);
    for (int i = 0; i < dim; ++i) {
        ranges[2 * i] = std::numeric_limits<double>::lowest() / 2;
        cVars->at(i) = dynamic_pointer_cast<autodiff::Variable>(vars.at(i)->getSolverVar());
    }

    // TODO: fixed Values

    for (auto c : calls) {
        if (dynamic_pointer_cast<autodiff::Term>(c->getConstraint()).get() == nullptr) {
            cerr << "CGSolver: Constrainttype not compatible with selected solver" << endl;
            return false;
        }
        constraint = constraint & dynamic_pointer_cast<autodiff::Term>(c->getConstraint());
        const std::vector<std::pair<double, double>>& allRanges = c->getAllRanges();
        for (int i = 0; i < c->getAllVariables().size(); ++i) {
            for (int j = 0; j < dim; ++j) {
                if (dynamic_pointer_cast<autodiff::Term>(c->getAllVariables()[j]).get() == nullptr) {
                    cerr << "CGSolver: Variabletype not compatible with selected solver" << endl;
                    return false;
                }
                if (cVars->at(j) == dynamic_pointer_cast<autodiff::Term>(c->getAllVariables()[j])) {
                    ranges[j * 2] = std::max(ranges[2 * j], allRanges[i].first);
                    ranges[j * 2 + 1] = std::min(ranges[2 * j + 1], allRanges[i].second);
                    if (ranges[j * 2] > ranges[j * 2 + 1]) {
                        return false;
                    }
                    break;
                }
            }
        }
    }
    std::vector<Variant> serial_seeds;
    int seed_num = getAlicaEngine()->getResultStore()->getSeeds(vars, ranges, serial_seeds);

    shared_ptr<vector<shared_ptr<vector<double>>>> seeds = make_shared<vector<shared_ptr<vector<double>>>>();
    seeds->reserve(seed_num);
    for (int i = 0; i < serial_seeds.size(); i += dim) {
        shared_ptr<vector<double>> singleseed = make_shared<vector<double>>();
        singleseed->reserve(dim);
        // Fill a single seed with information from the combined seed vector by resolving the variants to doubles or
        // NaNs.
        std::transform(serial_seeds.begin() + i, serial_seeds.begin() + i + dim, std::back_inserter(*singleseed),
                       [](Variant v) -> double { return v.isDouble() ? v.getDouble() : std::numeric_limits<double>::quiet_NaN(); });
        seeds->push_back(std::move(singleseed));
    }

    return _sgs.solveSimple(constraint, cVars, ranges, seeds);
}

bool CGSolver::getSolutionImpl(const VariableGrp& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<double>& results)
{
    shared_ptr<Term> constraint = ConstraintBuilder::TRUE;
    shared_ptr<Term> utility = TermBuilder::constant(1);
    int dim = vars.size();

    // create lists of constraint variables and corresponding ranges
    auto constraintVariables = make_shared<vector<shared_ptr<autodiff::Variable>>>(dim);
    std::vector<double> ranges(dim * 2, std::numeric_limits<double>::max() / 2);
    for (int i = 0; i < dim; ++i) {
        ranges[2 * i] = std::numeric_limits<double>::lowest() / 2;
        constraintVariables->at(i) = dynamic_pointer_cast<autodiff::Variable>(vars.at(i)->getSolverVar());
    }

    // get some utility significance threshold value if one exists
    double usigVal = calls[0]->getUtilitySignificanceThreshold();
    for (int i = 0; i < calls.size(); ++i) {
        // TODO: fixed Values
        if (calls.at(i)->isSettingUtilitySignificanceThreshold()) {
            usigVal = calls[i]->getUtilitySignificanceThreshold();
        }
    }

    // TODO: fixed Values

    double sufficientUtility = 0.0;

    for (auto& c : calls) {
        std::shared_ptr<autodiff::Term> constraintTerm = dynamic_pointer_cast<autodiff::Term>(c->getConstraint());
        if (constraintTerm.get() == nullptr) {
            std::cerr << "CGSolver: Constraint type not compatible with selected solver" << std::endl;
            return false;
        }
        constraint = constraint & constraintTerm;

        std::shared_ptr<autodiff::Term> utilityTerm = dynamic_pointer_cast<autodiff::Term>(c->getUtility());
        if (utilityTerm.get() == nullptr) {
            std::cerr << "CGSolver: Utility type not compatible with selected solver" << std::endl;
            return false;
        }
        utility = utility + utilityTerm;

        sufficientUtility += c->getUtilitySufficiencyThreshold();

        // limit ranges according to the ranges of the given calls
        const std::vector<std::pair<double, double>>& allRanges = c->getAllRanges();
        for (int i = 0; i < c->getAllVariables().size(); ++i) {
            std::shared_ptr<autodiff::Term> variableTerm = dynamic_pointer_cast<autodiff::Term>(c->getAllVariables()[i]);
            if (variableTerm.get() == nullptr) {
                std::cerr << "CGSolver: Variable is not of Type autodiff::Term!" << std::endl;
                return false;
            }

            for (int j = 0; j < constraintVariables->size(); ++j) {
                if (constraintVariables->at(j) == variableTerm) {
                    ranges[j * 2] = std::max(ranges[2 * j], allRanges[i].first);
                    ranges[j * 2 + 1] = std::min(ranges[2 * j + 1], allRanges[i].second);
                    if (ranges[j * 2] > ranges[j * 2 + 1]) {
                        std::cerr << "CGSolver: Ranges do not allow a solution!" << std::endl;
                        return false;
                    }
                    break;
                }
            }
        }
    }
    shared_ptr<Term> all = make_shared<ConstraintUtility>(constraint, utility);

    std::vector<Variant> serial_seeds;
    int seed_num = getAlicaEngine()->getResultStore()->getSeeds(vars, ranges, serial_seeds);

    shared_ptr<vector<shared_ptr<vector<double>>>> seeds = make_shared<vector<shared_ptr<vector<double>>>>();
    seeds->reserve(seed_num);
    for (int i = 0; i < serial_seeds.size(); i += dim) {
        shared_ptr<vector<double>> singleseed = make_shared<vector<double>>();
        singleseed->reserve(dim);
        std::transform(serial_seeds.begin() + i, serial_seeds.begin() + i + dim, std::back_inserter(*singleseed),
                       [](Variant v) -> double { return v.isDouble() ? v.getDouble() : std::numeric_limits<double>::quiet_NaN(); });
        seeds->push_back(std::move(singleseed));
    }

#ifdef CGSolver_DEBUG
    for (int i = 0; i < seeds->size(); ++i) {
        std::cout << "----CGS: seed " << i << " ";
        for (int j = 0; j < seeds->at(i)->size(); j++) {
            std::cout << seeds->at(i)->at(j) << " ";
        }
        std::cout << endl;
    }
#endif

    shared_ptr<vector<double>> gresults;
    double util = 0;
    { // for lock_guard
        lock_guard<std::mutex> lock(_mtx);
        _gs.setUtilitySignificanceThreshold(usigVal);
        gresults = _gs.solve(all, constraintVariables, ranges, seeds, sufficientUtility, &util);
    }
    if (gresults->size() > 0) {
        for (int i = 0; i < dim; ++i) {
            results.push_back(gresults->at(i));
        }
    }
    _lastUtil = util;
    _lastFEvals = _gs.getFEvals();
    _lastRuns = _gs.getRuns();
#ifdef CGSolver_DEBUG
    cout << "CGS: result ";
    for (int i = 0; i < gresults->size(); ++i) {
        cout << gresults->at(i) << " ";
    }
    cout << endl;
#endif
    return util > 0.75;
}

shared_ptr<SolverVariable> CGSolver::createVariable(int64_t id)
{
    return make_shared<autodiff::Variable>();
}

} // namespace reasoner
} // namespace alica
