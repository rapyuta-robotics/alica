#pragma once

#include "engine/modelmanagement/factories/Factory.h"

namespace alica {
    class StateFactory : public Factory {
    public :
        static State* create(const YAML::Node& stateNode);
        static void attachReferences();
    };
}
