
#pragma once
#include "Types.h"
#include <alica_common_config/debug_output.h>
#include <supplementary/AgentID.h>

#include <ostream>

namespace alica
{
std::ostream& operator<<(std::ostream& out, const AgentGrp& ag)
{
    for (const AgentIDConstPtr id : ag) {
        out << *id << " ";
    }
    // std::copy(names.begin(), names.end(), std::ostream_iterator<AgentIDConstPtr>(os, " "));
    return out;
}
}