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
class ModelFactory;

class PlanType : public AbstractPlan
{
public:
    PlanType();
    virtual ~PlanType();

    std::string toString() const override;

    const VariableBindingGrp& getParametrisation() const { return _parametrisation; }
    const PlanGrp& getPlans() const { return _plans; }
    const Plan* getPlanById(int64_t id) const;

private:
    friend ModelFactory;
    void setParametrisation(const VariableBindingGrp& parametrisation);
    void setPlans(const PlanGrp& plans);

    PlanGrp _plans;
    VariableBindingGrp _parametrisation;
};

} // namespace alica
