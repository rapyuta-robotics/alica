#pragma once
#include "SolverVar.h"
#include "engine/Types.h"

#include <vector>

namespace alica
{
struct SolverResult
{
    AgentId senderID;
    std::vector<SolverVar> vars;
};
} /* namespace alica */
