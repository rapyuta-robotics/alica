#pragma once
#include "SolverVar.h"

#include <vector>

namespace alica
{
struct SolverResult
{
    alica::AgentId senderID;
    std::vector<SolverVar> vars;
};
} /* namespace alica */
