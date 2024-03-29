#pragma once

#include "AbstractPlan.h"
#include "engine/Types.h"

#include <list>
#include <sstream>
#include <string>

namespace alica
{
class Plan;
class VariableBinding;
class PlanTypeFactory;

class PlanType : public AbstractPlan
{
public:
    PlanType();
    virtual ~PlanType();

    std::string toString(std::string indent = "") const override;

    const VariableBindingGrp& getVariableBindings() const { return _variableBindings; }
    const PlanGrp& getPlans() const { return _plans; }
    const Plan* getPlanById(int64_t id) const;

private:
    friend PlanTypeFactory;

    PlanGrp _plans;
    VariableBindingGrp _variableBindings;
};

} // namespace alica
