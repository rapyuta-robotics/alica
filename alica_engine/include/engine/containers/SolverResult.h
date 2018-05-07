#pragma once
#include "SolverVar.h"
#include "supplementary/AgentID.h"

namespace alica {
struct SolverResult {
    const supplementary::AgentID* senderID;
    vector<SolverVar*> vars;

    ~SolverResult() {
        for (auto sv : vars) {
            delete sv;
        }
    }
};
} /* namespace alica */
