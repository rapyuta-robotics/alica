/*
 * ConstraintHelper.cpp
 *
 *  Created on: Oct 17, 2014
 *      Author: Philipp
 */

#include "engine/constraintmodul/ConstraintHelper.h"

#include "engine/constraintmodul/IConstraintSolver.h"

namespace alica
{
	shared_ptr<IConstraintSolver> ConstraintHelper::solver;

	/**
	 * Init the ConstraintHelper, called once during Initialisation
	 *
	 * @param theSolver A IConstraintSolver
	 *
	 * @return A bool
	 */
	bool ConstraintHelper::init(shared_ptr<IConstraintSolver> theSolver)
	{
		solver = theSolver;
		return true;
	}

	/**
	 * Returns the constraint solver used
	 */
	shared_ptr<IConstraintSolver> ConstraintHelper::getSolver()
	{
		return solver;
	}
} /* namespace alica */
