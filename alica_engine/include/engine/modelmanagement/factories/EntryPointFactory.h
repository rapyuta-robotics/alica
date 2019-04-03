#pragma once

#include "Factory.h"

#include <vector>

namespace alica {
    class EntryPointFactory : public Factory {
    public:
        static std::vector<EntryPoint*> create(const YAML::Node& entryPoints);
        static void attachReferences();
    };
}
