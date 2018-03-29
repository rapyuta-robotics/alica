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

CGSolver::CGSolver(AlicaEngine *ae)
    : ISolver(ae)
    , lastUtil(0.0)
    , lastFEvals(0.0)
    , lastRuns(0.0)
{
    Term::setAnd(AndType::AND);
    Term::setOr(OrType::MAX);
    gs = make_shared<GSolver>();
    sgs = make_shared<GSolver>();
}

CGSolver::~CGSolver()
{
}

bool CGSolver::existsSolution(vector<Variable *> &vars, vector<shared_ptr<ProblemDescriptor>> &calls)
{
    shared_ptr<Term> constraint = ConstraintBuilder::TRUE;
    int dim = vars.size();

    auto cVars = make_shared<vector<shared_ptr<autodiff::Variable>>>(dim);
    auto ranges = make_shared<vector<shared_ptr<vector<double>>>>(dim);
    for (int i = 0; i < vars.size(); ++i)
    {
        ranges->at(i) = make_shared<vector<double>>(2);
        ranges->at(i)->at(0) = std::numeric_limits<double>::lowest();
        ranges->at(i)->at(1) = std::numeric_limits<double>::max();
        cVars->at(i) = dynamic_pointer_cast<autodiff::Variable>(vars.at(i)->getSolverVar());
    }

    // TODO: fixed Values

    for (auto c : calls)
    {
        if (!(dynamic_pointer_cast<autodiff::Term>(c->getConstraint()) != 0))
        {
            cerr << "CGSolver: Constrainttype not compatible with selected solver" << endl;
            return false;
        }
        constraint = constraint & dynamic_pointer_cast<autodiff::Term>(c->getConstraint());
        shared_ptr<vector<vector<double>>> allRanges = c->allRanges();
        for (int i = 0; i < c->getAllVars()->size(); ++i)
        {
            for (int j = 0; j < cVars->size(); ++j)
            {
                if (!(dynamic_pointer_cast<autodiff::Term>(c->getAllVars()->at(j)) != 0))
                {
                    cerr << "CGSolver: Variabletype not compatible with selected solver" << endl;
                    return false;
                }
                if (cVars->at(j) == dynamic_pointer_cast<autodiff::Term>(c->getAllVars()->at(j)))
                {
                    ranges->at(j)->at(0) = min(ranges->at(j)->at(0), allRanges->at(i).at(0));
                    ranges->at(j)->at(1) = min(ranges->at(j)->at(1), allRanges->at(i).at(1));
                    if (ranges->at(j)->at(0) > ranges->at(j)->at(1))
                    {
                        return false;
                    }
                    break;
                }
            }
        }
    }
    auto serial_seeds = ae->getResultStore()->getSeeds(std::make_shared<std::vector<Variable *>>(vars), ranges);
    // Deserialize seeds
    shared_ptr<vector<shared_ptr<vector<double>>>> seeds =
        make_shared<vector<shared_ptr<vector<double>>>>(serial_seeds->size());
    for (auto &serialseed : *serial_seeds)
    {
        shared_ptr<vector<double>> singleseed = make_shared<vector<double>>(serialseed->size());
        for (auto &serialvalue : *serialseed)
        {
            if (serialvalue != nullptr)
            {
                double v;
                uint8_t *pointer = (uint8_t *)&v;
                if (serialvalue->size() == sizeof(double))
                {
                    for (int k = 0; k < sizeof(double); k++)
                    {
                        *pointer = serialvalue->at(k);
                        pointer++;
                    }
                    singleseed->push_back(v);
                }
                else
                {
                    cerr << "CGS: Received Seed that is not size of double" << endl;
                    break;
                }
            }
            else
            {
                singleseed->push_back(std::numeric_limits<double>::max());
            }
        }
        seeds->push_back(singleseed);
    }

    return sgs->solveSimple(constraint, cVars, ranges, seeds);
}

bool CGSolver::getSolution(vector<Variable *> &vars, vector<shared_ptr<ProblemDescriptor>> &calls,
                           vector<void *> &results)
{
    shared_ptr<Term> constraint = ConstraintBuilder::TRUE;
    shared_ptr<Term> utility = TermBuilder::constant(1);
    int dim = vars.size();

    // create lists of constraint variables and corresponding ranges
    auto constraintVariables = make_shared<vector<shared_ptr<autodiff::Variable>>>(dim);
    auto ranges = make_shared<vector<shared_ptr<vector<double>>>>(dim);
    for (int i = 0; i < vars.size(); ++i)
    {
        ranges->at(i) = make_shared<vector<double>>(2);
        ranges->at(i)->at(0) = std::numeric_limits<double>::lowest();
        ranges->at(i)->at(1) = std::numeric_limits<double>::max();
        constraintVariables->at(i) = dynamic_pointer_cast<autodiff::Variable>(vars.at(i)->getSolverVar());
    }

    // get some utility significance threshold value if one exists
    double usigVal = calls[0]->getUtilitySignificanceThreshold();
    for (int i = 0; i < calls.size(); ++i)
    {
        // TODO: fixed Values
        if (calls.at(i)->getSetsUtilitySignificanceThreshold())
        {
            usigVal = calls[i]->getUtilitySignificanceThreshold();
        }
    }

    // TODO: fixed Values

    double sufficientUtility = 0.0;

    for (auto &c : calls)
    {
        std::shared_ptr<autodiff::Term> constraintTerm = dynamic_pointer_cast<autodiff::Term>(c->getConstraint());
        if (constraintTerm.get() == nullptr)
        {
            std::cerr << "CGSolver: Constraint type not compatible with selected solver" << std::endl;
            return false;
        }
        constraint = constraint & constraintTerm;


        std::shared_ptr<autodiff::Term> utilityTerm = dynamic_pointer_cast<autodiff::Term>(c->getUtility());
        if (utilityTerm.get() == nullptr)
        {
            std::cerr << "CGSolver: Utility type not compatible with selected solver" << std::endl;
            return false;
        }
        utility = utility + utilityTerm;

        sufficientUtility += c->getUtilitySufficiencyThreshold();

        // limit ranges according to the ranges of the given calls
        auto allRanges = c->allRanges();
        for (int i = 0; i < c->getAllVars()->size(); ++i)
        {
            std::shared_ptr<autodiff::Term> variableTerm = dynamic_pointer_cast<autodiff::Term>(c->getAllVars()->at(i));
            if (variableTerm.get() == nullptr)
            {
                std::cerr << "CGSolver: Variable is not of Type autodiff::Term!" << std::endl;
                return false;
            }

            for (int j = 0; j < constraintVariables->size(); ++j)
            {
                if (constraintVariables->at(j) == variableTerm)
                {
                    ranges->at(j)->at(0) = max(ranges->at(j)->at(0), allRanges->at(i).at(0));
                    ranges->at(j)->at(1) = min(ranges->at(j)->at(1), allRanges->at(i).at(1));
                    if (ranges->at(j)->at(0) > ranges->at(j)->at(1))
                    {
                        std::cerr << "CGSolver: Ranges do not allow a solution!" << std::endl;
                        return false;
                    }
                    break;
                }
            }
        }
    }
    shared_ptr<Term> all = make_shared<ConstraintUtility>(constraint, utility);

    auto tmp = make_shared<vector<Variable *>>(vars);
    auto serial_seeds = this->ae->getResultStore()->getSeeds(tmp, ranges);
    // Deserialize seeds
    auto seeds = make_shared<vector<shared_ptr<vector<double>>>>();
    seeds->reserve(serial_seeds->size());
    for (int i = 0; i < serial_seeds->size(); i++)
    {
        auto &serialSeed = serial_seeds->at(i);
        shared_ptr<vector<double>> singleSeed = make_shared<vector<double>>();
        singleSeed->reserve(serialSeed->size());
        for (auto &serialvalue : *serialSeed)
        {
            if (serialvalue != nullptr)
            {
                double v;
                uint8_t *pointer = (uint8_t *)&v;
                if (serialvalue->size() == sizeof(double))
                {
                    for (int k = 0; k < sizeof(double); k++)
                    {
                        *pointer = serialvalue->at(k);
                        pointer++;
                    }
                    singleSeed->push_back(v);
                }
                else
                {
                    cerr << "CGS: Received Seed that is not size of double" << endl;
                    break;
                }
            }
            else
            {
                singleSeed->push_back(std::numeric_limits<double>::quiet_NaN());
            }
        }
        seeds->push_back(singleSeed);
    }

#ifdef CGSolver_DEBUG
    for (int i = 0; i < seeds->size(); i++)
    {
        cout << "----CGS: seed " << i << " ";
        for (int j = 0; j < seeds->at(i)->size(); j++)
        {
            cout << seeds->at(i)->at(j) << " ";
        }
        cout << endl;
    }
#endif

    shared_ptr<vector<double>> gresults;
    double util = 0;
    { // for lock_guard
        lock_guard<std::mutex> lock(this->mtx);
        this->gs->setUtilitySignificanceThreshold(usigVal);
        gresults = this->gs->solve(all, constraintVariables, ranges, seeds, sufficientUtility, &util);
    }
    if (gresults->size() > 0)
    {
        for (int i = 0; i < dim; ++i)
        {
            double *rVal = new double{gresults->at(i)};
            results.push_back(rVal);
        }
    }
    this->lastUtil = util;
    this->lastFEvals = this->gs->getFEvals();
    this->lastRuns = this->gs->getRuns();
#ifdef CGSolver_DEBUG
    cout << "CGS: result ";
    for (int i = 0; i < gresults->size(); i++)
    {
        cout << gresults->at(i) << " ";
    }
    cout << endl;
#endif
    return util > 0.75;
}

shared_ptr<SolverVariable> CGSolver::createVariable(long id)
{
    return make_shared<autodiff::Variable>();
}

} /* namespace Reasoner */
} /* namespace Alica */
