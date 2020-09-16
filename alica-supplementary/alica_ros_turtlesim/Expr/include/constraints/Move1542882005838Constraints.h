#pragma once

#include <engine/BasicConstraint.h>
#include <iostream>
#include <memory>

namespace alica
{
class ProblemDescriptor;
class RunningPlan;
class Constraint1543284793605 : public BasicConstraint
{
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
