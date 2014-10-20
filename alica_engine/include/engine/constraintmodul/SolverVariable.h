/*
 * SolverVariable.h
 *
 *  Created on: Oct 20, 2014
 *      Author: Philipp
 */

#ifndef SOLVERVARIABLE_H_
#define SOLVERVARIABLE_H_

#include "engine/constraintmodul/SolverTerm.h"

namespace alica
{

	class SolverVariable : public SolverTerm
	{
	public:
		SolverVariable();
		virtual ~SolverVariable();
	};

} /* namespace alica */

#endif /* SOLVERVARIABLE_H_ */
