#pragma once

#include "engine/model/VariableBinding.h"
#include "engine/modelmanagement/factories/Factory.h"

namespace alica {
    class VariableBindingFactory : public Factory {
    public:
        static VariableBinding* create(const YAML::Node& node);
    };
}
