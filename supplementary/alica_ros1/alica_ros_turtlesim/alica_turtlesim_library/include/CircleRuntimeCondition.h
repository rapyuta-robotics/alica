#pragma once

#include "engine/BasicCondition.h"
#include "engine/BasicConstraint.h"
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
    static std::shared_ptr<CircleRuntimeCondition> create(ConditionContext& context)
    {
        (void) context;
        std::cerr << "Debug:"
                  << "CircleRuntimeCondition created static" << std::endl;

        return std::shared_ptr<CircleRuntimeCondition>(new CircleRuntimeCondition());
    }
};
BOOST_DLL_ALIAS(alica::CircleRuntimeCondition::create, CircleRuntimeCondition)

class CircleRuntimeConditionConstraint : public BasicConstraint
{
public:
    CircleRuntimeConditionConstraint();
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp) override;
    // Factory method
    static std::shared_ptr<CircleRuntimeConditionConstraint> create(ConstraintContext& context)
    {
        (void) context;
        return std::make_shared<CircleRuntimeConditionConstraint>();
    }
};
BOOST_DLL_ALIAS(alica::CircleRuntimeConditionConstraint::create, CircleRuntimeConditionConstraint)
} // namespace alica
