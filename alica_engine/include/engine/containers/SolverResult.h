#pragma once
#include "SolverVar.h"
#include "engine/AgentIDConstPtr.h"

#include <vector>

namespace alica
{
struct SolverResult
{
    AgentIDConstPtr senderID;
    std::vector<SolverVar> vars;
};
} /* namespace alica */
