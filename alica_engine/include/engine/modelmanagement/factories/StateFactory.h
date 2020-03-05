#pragma once

#include "engine/modelmanagement/factories/Factory.h"

namespace alica {
    class State;
    class StateFactory : public Factory {
    public :
        static State* create(const YAML::Node& stateNode);
        static void attachReferences();
    };
}
