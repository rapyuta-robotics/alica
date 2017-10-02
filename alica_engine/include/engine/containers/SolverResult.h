#pragma once
#include "SolverVar.h"
#include "supplementary/IAgentID.h"

namespace alica
{
	struct SolverResult
	{
		const supplementary::IAgentID* senderID;
		vector<SolverVar*> vars;

		~SolverResult()
		{
			for (auto sv : vars) {
				delete sv;
			}
		}
	};
} /* namespace alica */
