#pragma once

#include <memory>
#include "engine/IAlicaWorldModel.h"
#include "engine/PlanAttachment.h"

namespace alica
{
class AlicaEngine;
class BasicPlan;
class Configuration;

class IPlanCreator
{
public:
virtual ~IPlanCreator() {}
virtual std::unique_ptr<PlanAttachment> createPlanAttachment(int64_t attachmentWrapperConfId) = 0;
virtual std::unique_ptr<BasicPlan> createPlan(int64_t planId, IAlicaWorldModel* wm) = 0;
};

} /* namespace alica */
