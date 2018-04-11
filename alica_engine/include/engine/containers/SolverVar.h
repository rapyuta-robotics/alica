#pragma once
#include <stdint.h>
#include <vector>

namespace alica {
struct SolverVar {
    int64_t id;
    std::vector<uint8_t> value;
};

} /* namespace alica */
