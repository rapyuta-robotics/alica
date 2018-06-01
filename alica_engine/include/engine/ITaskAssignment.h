#pragma once

#include <memory>

namespace alica
{

class IAssignment;
class Assignment;

class ITaskAssignment
{
public:
    virtual ~ITaskAssignment() {}
    /**
     * Returns the best possible assignment for a plan, taking similarities to the old assignment into account.
     * @param oldAss The old IAssignment,  possibly null in case of a completely new assignment problem.
     * @return The new Assignment
     */
    virtual Assignment getNextBestAssignment(const Assignment* oldAss) = 0;
};

} /* namespace alica */
