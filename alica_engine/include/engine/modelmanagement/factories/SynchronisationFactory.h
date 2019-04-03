#pragma once

#include "Factory.h"

namespace alica {
    class Synchronisation;
    class SynchronisationFactory: public Factory {
    public:
        static Synchronisation* create(const YAML::Node& synchronisationNode, Plan* plan);
        static void attachReferences();
    };
}
