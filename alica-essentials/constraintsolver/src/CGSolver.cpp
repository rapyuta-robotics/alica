/*
 * CGSolver.cpp
 *
 *  Created on: Oct 10, 2014
 *      Author: Philipp
 */

#include "CGSolver.h"

#include "ResultStore.h"

namespace alica
{
	namespace reasoner
	{

		CGSolver::CGSolver()
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

		bool CGSolver::existsSolution(vector<Variable*> vars, vector<shared_ptr<ConstraintDescriptor>> calls)
		{
			return false;
		}

		bool CGSolver::getSolution(vector<Variable*> vars, vector<shared_ptr<ConstraintDescriptor>> calls,
							vector<double>& results)
		{
			return false;
		}

	} /* namespace Reasoner */
} /* namespace Alica */
