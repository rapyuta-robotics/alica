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
									vector<double>& results)
		{
			std::cout << "CGSolver::getSolution is called" << std::endl;
			return false;
		}

	} /* namespace Reasoner */
} /* namespace Alica */
