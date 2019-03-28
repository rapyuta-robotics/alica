#include "engine/modelmanagement/factories/VariableBindingFactory.h"

namespace alica {
    VariableBinding* VariableBindingFactory::create(const YAML::Node& node) {
        VariableBinding* variableBinding = new VariableBinding();
        // TODO not ready in plan designer, yet
        return variableBinding;
    }
}
