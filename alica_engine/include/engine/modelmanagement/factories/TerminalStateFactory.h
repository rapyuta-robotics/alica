#pragma once

#include "engine/modelmanagement/factories/Factory.h"

namespace alica {
    class TerminalState;
    class TerminalStateFactory : public Factory {
    public :
        static TerminalState* create(const YAML::Node& terminalStateNode, AbstractPlan* plan);
    };
}
