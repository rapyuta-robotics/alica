#include "engine/model/Plan.h"

#include <yaml-cpp/yaml.h>

#include <iostream>

namespace YAML {
    template<>
    struct convert<alica::Plan*> {
        static Node encode(const alica::Plan* rhs) {
            Node node;

            return node;
        }

        static bool decode(const Node& node, alica::Plan* plan) {
            plan = new alica::Plan(1);
            return true;
        }
    };
}