#pragma once

#include <memory>

namespace alica
{
    class PlanAttachment;

    class IPlanAttachmentCreator
    {
    public:
        virtual ~IPlanAttachmentCreator() {}
        virtual std::unique_ptr<PlanAttachment> createPlanAttachment(int64_t attachmentWrapperConfId) = 0;
    };

} /* namespace alica */
