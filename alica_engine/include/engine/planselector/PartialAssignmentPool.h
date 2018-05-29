#include <vector>

#pragma once

namespace alica
{
class PartialAssignment;
class EntryPoint;
class Task;

class PartialAssignmentPool
{
  public:
    PartialAssignmentPool();
    virtual ~PartialAssignmentPool();
    int curIndex;
    const static int maxCount;
    const EntryPoint* idleEP;
    const Task* idleTask;
    std::vector<PartialAssignment*> daPAs;
};

} // namespace alica
