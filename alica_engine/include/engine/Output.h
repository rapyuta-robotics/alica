
#pragma once
#include "Types.h"

#include <iterator>
#include <ostream>

namespace alica
{

inline std::ostream& operator<<(std::ostream& out, const AgentGrp& ag)
{
    std::copy(ag.begin(), ag.end(), std::ostream_iterator<AgentId>(out, " "));
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const IdGrp& ig)
{
    std::copy(ig.begin(), ig.end(), std::ostream_iterator<int64_t>(out, " "));
    return out;
}
} // namespace alica