#pragma once

#include <memory>
#include <string>

namespace alica
{
class ProblemDescriptor;
class RunningPlan;

struct ConstraintContext
{
    const std::string name;
    const std::string libraryName;
    int64_t constraintConfId;
};

class BasicConstraint
{
public:
    virtual ~BasicConstraint() {}

    virtual void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp) = 0;
};

} /* namespace alica */
