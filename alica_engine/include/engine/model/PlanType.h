#pragma once

#include "AbstractPlan.h"
#include "Plan.h"
#include "engine/Types.h"

#include <list>
#include <sstream>
#include <string>

namespace alica
{
class Plan;
class Parametrisation;
class ModelFactory;

class PlanType : public AbstractPlan
{
public:
    PlanType();
    virtual ~PlanType();

    std::string toString() const override;

    const ParametrisationGrp& getParametrisation() const { return _parametrisation; }
    const PlanGrp& getPlans() const { return _plans; }
    const Plan* getPlanById(int64_t id) const;

private:
    friend ModelFactory;
    void setParametrisation(const ParametrisationGrp& parametrisation);
    void setPlans(const PlanGrp& plans);

    PlanGrp _plans;
    ParametrisationGrp _parametrisation;
};

} // namespace alica
