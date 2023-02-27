#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class OrderedSchedulingTestPlan1629895582410 : public DomainPlan
{
public:
    OrderedSchedulingTestPlan1629895582410(PlanContext& context);
    virtual ~OrderedSchedulingTestPlan1629895582410();
};

class UtilityFunction1629895582410 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1629895758612 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1629895768182 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, OrderedSchedulingTestPlan1629895582410)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, OrderedSchedulingTestPlan1629895582410UtilityFunction)
} /* namespace alica */
