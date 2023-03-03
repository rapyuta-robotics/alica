#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <libalica-tests/util/AlicaTestsUtilityFunction.h>

namespace alica
{
class MasterPlanTaskAssignment : public DomainPlan
{
public:
    MasterPlanTaskAssignment(PlanContext& context);
};

class MasterPlanTaskAssignmentUtilityFunction : public AlicaTestsUtilityFunction<MasterPlanTaskAssignmentUtilityFunction>
{
public:
    MasterPlanTaskAssignmentUtilityFunction(UtilityFunctionContext& context)
            : AlicaTestsUtilityFunction(context){};
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MasterPlanTaskAssignment)
BOOST_DLL_ALIAS(alica::MasterPlanTaskAssignmentUtilityFunction::create, MasterPlanTaskAssignmentUtilityFunction)
} /* namespace alica */
