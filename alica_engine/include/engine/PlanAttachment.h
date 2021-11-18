#pragma once

#include "engine/blackboard/BlackBoard.h"

namespace alica
{

class PlanAttachment
{
public:
    PlanAttachment();
    virtual ~PlanAttachment();
    /**
     * Initialize blackboard of child given blackboard of parent and knowledge of which state/child we are setting parameters for
     * Returns false if the assumptions of beginning the child plan/behavior no longer hold true
     */
    virtual bool setParameters(const BlackBoard& parent_bb, BlackBoard& child_bb) = 0;
};

} /* namespace alica */
