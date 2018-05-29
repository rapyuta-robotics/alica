#pragma once
#include "SolverVar.h"
#include "supplementary/AgentID.h"

#include <vector>

namespace alica {
struct SolverResult {
    const supplementary::AgentID* senderID;
    std::vector<SolverVar> vars;
};
} /* namespace alica */
