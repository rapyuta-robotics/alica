#pragma once

#include <engine/BasicConstraint.h>
#include <iostream>
#include <memory>

namespace alica
{
class ProblemDescriptor;
class RunningPlan;
class Constraint1402489460549 : public BasicConstraint
{
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
};
class Constraint1402489462088 : public BasicConstraint
{
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
