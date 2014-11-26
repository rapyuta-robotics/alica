/*
 * CGSolver.cpp
 *
 *  Created on: Oct 10, 2014
 *      Author: Philipp
 */

#include "CGSolver.h"

#include <engine/AlicaEngine.h>
#include <engine/constraintmodul/ConstraintDescriptor.h>
#include <engine/constraintmodul/IResultStore.h>
#include <engine/model/Variable.h>
#include <GSolver.h>
#include <limits>

#include <iostream>

namespace alica
{
	namespace reasoner
	{

		CGSolver::CGSolver(AlicaEngine* ae) :
				IConstraintSolver(ae)
		{
			Term::setAnd(AndType::AND);
			Term::setOr(OrType::MAX);
			gs = make_shared<GSolver>();
			sgs = make_shared<GSolver>();
		}

		CGSolver::~CGSolver()
		{
			// TODO Auto-generated destructor stub
		}

		bool CGSolver::existsSolution(vector<Variable*>& vars, vector<shared_ptr<ConstraintDescriptor>>& calls)
		{
			shared_ptr<Term> constraint = ConstraintBuilder::TRUE;
			int dim = vars.size();

			auto cVars = make_shared<vector<shared_ptr<autodiff::Variable>>>(dim);
			auto ranges = make_shared<vector<shared_ptr<vector<double>>> >(dim);
			for (int i = 0; i < vars.size(); ++i)
			{
				ranges->at(i) = make_shared<vector<double>>(2);
				ranges->at(i)->at(0) = std::numeric_limits<double>::min();
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
			auto seeds = ae->getResultStore()->getSeeds(make_shared<vector<Variable*>>(vars), ranges);

			return sgs->solveSimple(constraint, cVars, ranges, seeds);
		}

		bool CGSolver::getSolution(vector<Variable*>& vars, vector<shared_ptr<ConstraintDescriptor>>& calls,
									shared_ptr<vector<double>>& results)
		{
			shared_ptr<Term> constraint = ConstraintBuilder::TRUE;
			shared_ptr<Term> utility = TermBuilder::constant(1);
			results = nullptr;
			int dim = vars.size();

			auto cVars = make_shared<vector<shared_ptr<autodiff::Variable>>>(dim);
			auto ranges = make_shared<vector<shared_ptr<vector<double>>> >(dim);
			for (int i = 0; i < vars.size(); ++i)
			{
				ranges->at(i) = make_shared<vector<double>>(2);
				ranges->at(i)->at(0) = std::numeric_limits<double>::min();
				ranges->at(i)->at(1) = std::numeric_limits<double>::max();
				cVars->at(i) = dynamic_pointer_cast<autodiff::Variable>(vars.at(i)->getSolverVar());
			}
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

			double sufficientUtility = 0;

			for (auto c : calls)
			{
				if (!(dynamic_pointer_cast<autodiff::Term>(c->getConstraint()) != 0))
				{
					cerr << "CGSolver: Constrainttype not compatible with selected solver" << endl;
					return false;
				}
				constraint = constraint & dynamic_pointer_cast<autodiff::Term>(c->getConstraint());
				if (!(dynamic_pointer_cast<autodiff::Term>(c->getUtility()) != 0))
				{
					cerr << "CGSolver: Utilitytype not compatible with selected solver" << endl;
					return false;
				}
				utility = utility + dynamic_pointer_cast<autodiff::Term>(c->getUtility());
				sufficientUtility += c->getUtilitySufficiencyThreshold();
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
			shared_ptr<Term> all = make_shared<ConstraintUtility>(constraint, utility);

			auto seeds = ae->getResultStore()->getSeeds(make_shared<vector<Variable*>>(vars), ranges);

			double util = 0;
			{ // for lock_guard
				lock_guard<std::mutex> lock(mtx);
				gs->setUtilitySignificanceThreshold(usigVal);
				results = gs->solve(all, cVars, ranges, seeds, sufficientUtility, &util);
			}
			if (results != nullptr)
			{
				for (int i = 0; i < dim; ++i)
				{
					ae->getResultStore()->postResult(vars[i]->getId(), results->at(i));
				}
			}
			lastUtil = util;
			lastFEvals = gs->getFEvals();
			lastRuns = gs->getRuns();
			return util > 0.75;
		}

	} /* namespace Reasoner */
} /* namespace Alica */
