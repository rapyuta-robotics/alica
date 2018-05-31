
#pragma once
#include "Types.h"
#include <ostream>

#define ALICA_DEBUG_LEVEL_WARNING

#include <alica_common_config/debug_output.h>

std::ostream& operator<<(std::ostream& out, const AgentGrp& ag)
{
    if (!out.good()) {
        return out;
    }
    for (const AgentIDConstPtr id : ag) {
        out << *id << " ";
    }
    // std::copy(names.begin(), names.end(), std::ostream_iterator<AgentIDConstPtr>(os, " "));
    return os;
}
