#pragma once

#include "engine/BasicConstraint.h"
#include "engine/BasicPlan.h"
#include "engine/BasicUtilityFunction.h"
#include <boost/dll/alias.hpp>

namespace alica
{
class AcmePlan : public BasicPlan
{
public:
    AcmePlan(PlanContext& context);
    void run() override{};

    // Factory method
    static std::unique_ptr<AcmePlan> create(PlanContext& context) { return std::make_unique<AcmePlan>(context); }
};

class AcmeRuntimeConditionConstraint : public alica::BasicConstraint
{
public:
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp) override{};
    // Factory method
    static std::shared_ptr<AcmeRuntimeConditionConstraint> create(alica::ConstraintContext& context)
    {
        return std::make_shared<AcmeRuntimeConditionConstraint>();
    }
};

BOOST_DLL_ALIAS(alica::AcmePlan::create, AcmePlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AcmePlanUtilityFunction)
BOOST_DLL_ALIAS(alica::AcmeRuntimeConditionConstraint::create, AcmeRuntimeConditionConstraint)
} // namespace alica
