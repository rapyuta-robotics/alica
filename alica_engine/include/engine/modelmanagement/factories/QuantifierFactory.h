#pragma once

#include "Factory.h"

namespace alica {
    class Quantifier;
    class QuantifierFactory: public Factory {
    public:
        static Quantifier* create(const YAML::Node& quantifierNode);
    };
}
