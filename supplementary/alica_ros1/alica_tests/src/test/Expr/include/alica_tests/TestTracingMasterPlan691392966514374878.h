#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestTracingMasterPlan691392966514374878 : public DomainPlan
{
public:
    TestTracingMasterPlan691392966514374878(PlanContext& context);
    virtual ~TestTracingMasterPlan691392966514374878();
};

class UtilityFunction691392966514374878 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1840401110297459509 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestTracingMasterPlan691392966514374878)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestTracingMasterPlan691392966514374878UtilityFunction)

} /* namespace alica */
