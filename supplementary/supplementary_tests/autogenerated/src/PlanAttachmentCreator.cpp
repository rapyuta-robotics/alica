#include "PlanAttachmentCreator.h"
#include "GSolver/GSolverMaster1417423751087.h"
#include "GSolver/GSolverTestPlan1417423757243.h"
#include "ProblemModule/ProbBuildingLevel11479557378264.h"
#include "ProblemModule/ProbBuildingLevel1_11479557664989.h"
#include "ProblemModule/ProblemBuildingMaster1479556022226.h"
#include "ProblemModule/QueryPlan11479556074049.h"
#include "ProblemModule/QueryPlan21479718449392.h"
#include "VariableHandling/Lvl11524452759599.h"
#include "VariableHandling/Lvl21524452793378.h"
#include "VariableHandling/Lvl31524452836022.h"
#include "VariableHandling/VHMaster1524452721452.h"
#include "engine/PlanAttachment.h"

namespace alica
{

class DefaultPlanAttachment : public PlanAttachment
{
    bool setParameters(const BlackBoard& parent_bb, BlackBoard& child_bb) { return true; }
};

PlanAttachmentCreator::PlanAttachmentCreator() {}
PlanAttachmentCreator::~PlanAttachmentCreator() {}

std::unique_ptr<PlanAttachment> PlanAttachmentCreator::createPlanAttachment(int64_t attachmentWrapperConfId)
{
    switch (attachmentWrapperConfId) {
    default:
        return std::make_unique<DefaultPlanAttachment>();
    }
}
} // namespace alica
