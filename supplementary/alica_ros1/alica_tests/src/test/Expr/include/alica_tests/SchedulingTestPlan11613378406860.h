#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SchedulingTestPlan11613378406860 : public DomainPlan
{
public:
    SchedulingTestPlan11613378406860(PlanContext& context);
    virtual ~SchedulingTestPlan11613378406860();

protected:
    virtual void onInit();
    virtual void onTerminate();
};

class UtilityFunction1613378406860 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1614960055821 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1614960063843 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SchedulingTestPlan11613378406860)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingTestPlan11613378406860UtilityFunction)
} /* namespace alica */
