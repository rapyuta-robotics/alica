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
    CircleRuntimeCondition()
    {
        std::cerr << "Debug:"
                  << "CircleRuntimeCondition created" << std::endl;
    }

    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
    // Factory method
    static std::unique_ptr<CircleRuntimeCondition> create() { 
                std::cerr << "Debug:"
                  << "CircleRuntimeCondition created static" << std::endl;

        return std::unique_ptr<CircleRuntimeCondition>(new CircleRuntimeCondition()); }
};
BOOST_DLL_ALIAS(alica::CircleRuntimeCondition::create, CircleRuntimeCondition)
} // namespace alica
