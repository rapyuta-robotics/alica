#pragma once
#include "SolverVar.h"

#include <vector>

namespace alica
{
struct SolverResult
{
    uint64_t senderID;
    std::vector<SolverVar> vars;
};
} /* namespace alica */
