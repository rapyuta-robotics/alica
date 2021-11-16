#pragma once
#include "engine/Types.h"
#include "SolverVar.h"

#include <vector>

namespace alica
{
struct SolverResult
{
    AgentId senderID;
    std::vector<SolverVar> vars;
};
} /* namespace alica */
