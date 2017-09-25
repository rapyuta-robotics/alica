#pragma once
#include "SolverVar.h"
#include "engine/IRobotID.h"

namespace alica
{
	struct SolverResult
	{
		const alica::IRobotID* senderID;
		vector<SolverVar*> vars;

		~SolverResult()
		{
			for (auto sv : vars) {
				delete sv;
			}
		}
	};
} /* namespace alica */
