#pragma once

#include <memory>

namespace alica
{
    class AlicaEngine;
    class BasicPlan;
    class Configuration;

    class IPlanCreator
    {
    public:
        virtual ~IPlanCreator() {}
        virtual std::unique_ptr<BasicPlan> createPlan(int64_t planId) = 0;
        virtual std::unique_ptr<PlanAttachment> createPlanAttachment(int64_t attachmentWrapperConfId) = 0;
    };

} /* namespace alica */
