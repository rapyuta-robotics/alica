#pragma once

#include "engine/collections/Variant.h"
#include <stdint.h>
#include <vector>

namespace alica {
struct SolverVar {
    int64_t id;
    uint8_t value[Variant::kVariantSize];
};

} /* namespace alica */
