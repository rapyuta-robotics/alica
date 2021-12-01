#pragma once

#include <memory>
#include "engine/IAlicaWorldModel.h"
#include "engine/PlanAttachment.h"

namespace alica
{
class AlicaEngine;
class BasicPlan;
class Configuration;


class DefaultPlanAttachment : public PlanAttachment
{
bool setParameters(const BlackBoard& parent_bb, BlackBoard& child_bb){
        std::cerr << "DefaultPlanAttachment: this function should not be called" << std::endl;
        throw new std::exception();
        return false;
}
};

class IPlanCreator
{
public:
virtual ~IPlanCreator() {}
virtual std::unique_ptr<PlanAttachment> createPlanAttachment(int64_t attachmentWrapperConfId) = 0;
virtual std::unique_ptr<BasicPlan> createPlan(int64_t planId, IAlicaWorldModel* wm) = 0;
};

} /* namespace alica */
