#pragma once
#include <stdint.h>
#include <vector>

namespace alica {
struct SolverVar {
    int64_t id;
    uint8_t[Variant::kVariantSize] value;
};

} /* namespace alica */
