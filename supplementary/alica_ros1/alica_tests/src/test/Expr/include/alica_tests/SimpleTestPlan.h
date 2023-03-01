#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <libalica-tests/util/AlicaTestsPlan.h>

namespace alica
{
class SimpleTestPlan : public AlicaTestsPlan<SimpleTestPlan>
{
public:
    SimpleTestPlan(PlanContext& context);

protected:
    virtual void onInit() override;
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

BOOST_DLL_ALIAS(alica::SimpleTestPlan::create, SimpleTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SimpleTestPlanUtilityFunction)

} /* namespace alica */
