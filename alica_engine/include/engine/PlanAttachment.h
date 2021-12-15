#pragma once

namespace alica
{

class Blackboard;

class PlanAttachment
{
public:
    PlanAttachment() = default;
    virtual ~PlanAttachment() = default;
    /**
     * Initialize Blackboard of child given Blackboard of parent and knowledge of which state/child we are setting parameters for
     * Returns false if the assumptions of beginning the child plan/behavior no longer hold true
     */
    virtual bool setParameters(const Blackboard& parent_bb, Blackboard& child_bb) = 0;
};

} /* namespace alica */
