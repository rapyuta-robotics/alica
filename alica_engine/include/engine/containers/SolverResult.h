#pragma once
#include "SolverVar.h"
#include <essentials/IdentifierConstPtr.h>

#include <vector>

namespace alica
{
struct SolverResult
{
    essentials::IdentifierConstPtr senderID;
    std::vector<SolverVar> vars;
};
} /* namespace alica */
