/*
 * CGSolver.h
 *
 *  Created on: Oct 10, 2014
 *      Author: Philipp
 */

#ifndef CGSOLVER_H_
#define CGSOLVER_H_

//#include "IConstraintSolver.h"
#include <engine/constraintmodul/IConstraintSolver.h>

#include <GSolver.h>

#include <memory>

namespace alica
{
	namespace reasoner
	{
		class ResultStore;

		class CGSolver : public IConstraintSolver
		{
		public:
			CGSolver();
			virtual ~CGSolver();

			bool existsSolution(vector<Variable*> vars, vector<shared_ptr<ConstraintDescriptor>> calls);
			bool getSolution(vector<Variable*> vars, vector<shared_ptr<ConstraintDescriptor>> calls, vector<double>& results);

			shared_ptr<ResultStore> resultCache;

			shared_ptr<GSolver> gs;
			shared_ptr<GSolver> sgs;

//			public double LastUtil {get; private set;}
//			public double LastRuns {get; private set;}
//			public double LastFEvals {get; private set;}
		};

	} /* namespace Reasoner */
} /* namespace Alica */

#endif /* CGSOLVER_H_ */
