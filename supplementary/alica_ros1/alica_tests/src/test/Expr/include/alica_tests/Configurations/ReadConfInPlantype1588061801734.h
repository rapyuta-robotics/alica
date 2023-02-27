#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ReadConfInPlantype1588061801734 : public DomainPlan
{
public:
    ReadConfInPlantype1588061801734(PlanContext& context);
    virtual ~ReadConfInPlantype1588061801734();
};

class UtilityFunction1588061801734 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1588246141557 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1588246144841 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ReadConfInPlantype1588061801734)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ReadConfInPlantype1588061801734UtilityFunction)
} /* namespace alica */
