
#pragma once
#include "Types.h"
#include <alica_common_config/debug_output.h>
#include <engine/AgentIDConstPtr.h>

#include <iterator>
#include <ostream>

namespace alica
{

std::ostream& operator<<(std::ostream& out, const AgentGrp& ag)
{
    std::copy(ag.begin(), ag.end(), std::ostream_iterator<AgentIDConstPtr>(out, " "));
    return out;
}
}