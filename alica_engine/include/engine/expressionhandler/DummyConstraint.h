#pragma once

#include "engine/BasicConstraint.h"

#include <memory>

namespace alica
{
class ProblemDescriptor;
class RunningPlan;

class DummyConstraint : public BasicConstraint
{
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
