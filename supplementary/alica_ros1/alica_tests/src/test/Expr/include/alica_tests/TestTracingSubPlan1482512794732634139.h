#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestTracingSubPlan1482512794732634139 : public DomainPlan
{
public:
    TestTracingSubPlan1482512794732634139(PlanContext& context);
    virtual ~TestTracingSubPlan1482512794732634139();
};

class UtilityFunction1482512794732634139 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestTracingSubPlan1482512794732634139)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestTracingSubPlan1482512794732634139UtilityFunction)

} /* namespace alica */
