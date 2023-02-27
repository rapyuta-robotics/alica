#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MasterPlanTaskAssignment1407152758497 : public DomainPlan
{
public:
    MasterPlanTaskAssignment1407152758497(PlanContext& context);
    virtual ~MasterPlanTaskAssignment1407152758497();
};

class UtilityFunction1407152758497 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MasterPlanTaskAssignment1407152758497)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MasterPlanTaskAssignment1407152758497UtilityFunction)
} /* namespace alica */
