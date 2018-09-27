#pragma once

#include <memory>

namespace alica
{
class RunningPlan;

class IPlanTreeVisitor
{
  public:
    virtual ~IPlanTreeVisitor() {}
    virtual void visit(std::shared_ptr<RunningPlan> r) = 0;
};

} /* namespace alica */
