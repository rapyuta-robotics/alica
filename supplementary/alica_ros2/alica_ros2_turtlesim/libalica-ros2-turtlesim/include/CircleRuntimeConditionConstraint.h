#pragma once

#include <engine/BasicConstraint.h>
#include <memory>

#include <boost/dll/alias.hpp>

namespace turtlesim
{
class ProblemDescriptor;
class RunningPlan;
class CircleRuntimeConditionConstraint : public alica::BasicConstraint
{
public:
    void getConstraint(std::shared_ptr<alica::ProblemDescriptor> c, std::shared_ptr<alica::RunningPlan> rp);
    static std::shared_ptr<CircleRuntimeConditionConstraint> create(alica::ConstraintContext&);
};
BOOST_DLL_ALIAS(turtlesim::CircleRuntimeConditionConstraint::create, CircleRuntimeConditionConstraint)
} /* namespace turtlesim */
