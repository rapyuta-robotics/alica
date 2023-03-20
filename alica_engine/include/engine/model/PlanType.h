#pragma once

#include "AbstractPlan.h"
#include "engine/Types.h"

#include <string>

namespace alica
{
class Plan;
class VariableBinding;

class PlanType : public AbstractPlan
{
public:
    PlanType();

    std::string toString(std::string indent = "") const override;
    void addVariableBinding(const VariableBinding* v);
    const VariableBindingGrp& getVariableBindings() const { return _variableBindings; }
    void addPlan(const Plan* p);
    const PlanGrp& getPlans() const { return _plans; }
    const Plan* getPlanById(int64_t id) const;

private:
    PlanGrp _plans;
    VariableBindingGrp _variableBindings;
};

} // namespace alica
