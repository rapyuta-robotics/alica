#pragma once

#include "engine/BasicCondition.h"
#include "engine/RunningPlan.h"
#include <boost/dll/alias.hpp>

class IAlicaWorldModel;

namespace alica
{
class CircleRuntimeCondition : public BasicCondition
{
public:
    CircleRuntimeCondition();
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
    // Factory method
    static std::shared_ptr<CircleRuntimeCondition> create()
    {
        std::cerr << "Debug:"
                  << "CircleRuntimeCondition created static" << std::endl;

        return std::shared_ptr<CircleRuntimeCondition>(new CircleRuntimeCondition());
    }
};
BOOST_DLL_ALIAS(alica::CircleRuntimeCondition::create, CircleRuntimeCondition)
} // namespace alica
