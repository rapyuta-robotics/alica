#pragma once
#include "SolverVar.h"
#include <essentials/AgentIDConstPtr.h>

#include <vector>

namespace alica
{
struct SolverResult
{
    essentials::AgentIDConstPtr senderID;
    std::vector<SolverVar> vars;
};
} /* namespace alica */
