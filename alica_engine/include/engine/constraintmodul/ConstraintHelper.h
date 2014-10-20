/*
 * ConstraintHelper.h
 *
 *  Created on: Oct 17, 2014
 *      Author: Philipp
 */

#ifndef CONSTRAINTHELPER_H_
#define CONSTRAINTHELPER_H_

#include <memory>

using namespace std;

namespace alica
{
	class IConstraintSolver;

	/**
	 * The Constraint Helper offers methods to query for variables for valid values and test if solutions exists
	 */
	class ConstraintHelper
	{
	public:
		static bool init(shared_ptr<IConstraintSolver> theSolver);
		static shared_ptr<IConstraintSolver> getSolver();

	private:
		static shared_ptr<IConstraintSolver> solver;
	};

} /* namespace alica */

#endif /* CONSTRAINTHELPER_H_ */
