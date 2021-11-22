#include "PlanAttachmentCreator.h"
#include "Master2425328142973735249.h"
#include "Move1889749086610694100.h"
#include "engine/PlanAttachment.h"

namespace alica
{

PlanAttachmentCreator::PlanAttachmentCreator() {}
PlanAttachmentCreator::~PlanAttachmentCreator() {}

std::unique_ptr<PlanAttachment> PlanAttachmentCreator::createPlanAttachment(int64_t attachmentWrapperConfId)
{
    switch (attachmentWrapperConfId) {
    default:
        std::cerr << "ConditionCreator: Unknown plan attachment id requested: " << attachmentWrapperConfId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
