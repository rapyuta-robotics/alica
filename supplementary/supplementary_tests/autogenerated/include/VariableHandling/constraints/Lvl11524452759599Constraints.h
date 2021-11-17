#pragma once

#include <engine/BasicConstraint.h>
#include <iostream>
#include <memory>

namespace alica
{
class ProblemDescriptor;
class RunningPlan;
class Constraint1524453470580 : public BasicConstraint
{
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
};
class Constraint1524453491764 : public BasicConstraint
{
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
