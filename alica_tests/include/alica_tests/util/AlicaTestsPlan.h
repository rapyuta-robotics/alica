#pragma once

#include <engine/BasicPlan.h>

namespace alica
{

template <class Plan>
class AlicaTestsPlan : public alica::BasicPlan
{
public:
    AlicaTestsPlan(alica::PlanContext& context)
            : alica::BasicPlan(context)
    {
    }
    static std::unique_ptr<Plan> create(alica::PlanContext& context) { return std::make_unique<Plan>(context); }
};

} // namespace alica
