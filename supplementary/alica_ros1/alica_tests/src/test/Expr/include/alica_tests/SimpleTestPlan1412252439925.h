#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SimpleTestPlan1412252439925 : public DomainPlan
{
public:
    SimpleTestPlan1412252439925(PlanContext& context);
    virtual ~SimpleTestPlan1412252439925();

protected:
    virtual void onInit() override;
};

class UtilityFunction1412252439925 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1412781707952 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class RunTimeCondition1412781693884 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1412761926856 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SimpleTestPlan1412252439925)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SimpleTestPlan1412252439925UtilityFunction)

} /* namespace alica */
