#pragma once

#include <memory>

namespace alica
{
class RunningPlan;

class IPlanTreeVisitor
{
public:
    virtual ~IPlanTreeVisitor() {}
    virtual void visit(RunningPlan& r) = 0;
};

} /* namespace alica */
