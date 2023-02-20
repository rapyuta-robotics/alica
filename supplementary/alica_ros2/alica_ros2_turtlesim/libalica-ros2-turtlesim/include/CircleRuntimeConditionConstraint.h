#pragma once

#include <engine/BasicConstraint.h>
#include <iostream>
#include <memory>

#include <boost/dll/alias.hpp>

namespace alica
{
class ProblemDescriptor;
class RunningPlan;
class CircleRuntimeConditionConstraint : public BasicConstraint
{
    public:
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
    static std::shared_ptr<CircleRuntimeConditionConstraint> create(alica::ConstraintContext&);
};
BOOST_DLL_ALIAS(alica::CircleRuntimeConditionConstraint::create, CircleRuntimeConditionConstraint)
} /* namespace alica */
