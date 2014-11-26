/*
 * SolverResult.h
 *
 *  Created on: Nov 24, 2014
 *      Author: Philipp
 */

#ifndef SOLVERRESULT_H_
#define SOLVERRESULT_H_

#include "SolverVar.h"

namespace alica
{
	struct SolverResult
	{
		int senderID;
		vector<SolverVar*> vars;

		~SolverResult()
		{
			for (auto sv : vars) {
				delete sv;
			}
		}
	};
} /* namespace alica */

#endif /* SOLVERRESULT_H_ */
