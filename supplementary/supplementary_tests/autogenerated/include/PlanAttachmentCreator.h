#pragma once

#include <engine/IPlanAttachmentCreator.h>

#include <memory>

namespace alica
{
class PlanAttachment;

class PlanAttachmentCreator : public IPlanAttachmentCreator
{
public:
    PlanAttachmentCreator();
    virtual ~PlanAttachmentCreator();
    std::unique_ptr<PlanAttachment> createPlanAttachment(int64_t attachmentWrapperConfId) override;
};

} /* namespace alica */
