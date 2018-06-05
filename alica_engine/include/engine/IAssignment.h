#pragma once

#include "engine/Types.h"
#include "supplementary/AgentID.h"

#include <list>
#include <memory>
#include <string>

namespace alica
{

using std::list;
using std::shared_ptr;
using std::string;
using std::vector;

class EntryPoint;
class SuccessCollection;
class AssignmentCollection;

class Iterator
{
    AgentIdCOnstPtr operator*();
    Iterator& operator++();
    bool operator==(const Iterator& o);
}

class View
{
    Iterator begin()
            : Iterator end();
};

/**
 *  An IAssignment describes a potentially partial assignment of robots to EntryPoints within a plan.
 */
class IAssignment
{
public:
    virtual ~IAssignment() {}

    virtual int getTotalRobotCount() const = 0;
    virtual int getEntryPointCount() const = 0;

    virtual View getRobotsWorking(const EntryPoint* ep, AgentGrp& o_out) const = 0;

    virtual void getRobotsWorking(int64_t epid, AgentGrp& o_out) const = 0;

    virtual shared_ptr<list<const supplementary::AgentID*>> getUniqueRobotsWorkingAndFinished(const EntryPoint* ep) = 0;

    virtual shared_ptr<list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(const EntryPoint* ep) = 0;
    virtual shared_ptr<list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(int64_t epid) = 0;
    virtual bla fu vector getUnassignedAgents() = 0;
};

} /* namespace alica */
