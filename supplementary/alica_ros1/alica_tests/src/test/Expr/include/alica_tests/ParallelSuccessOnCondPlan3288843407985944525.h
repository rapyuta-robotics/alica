#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ParallelSuccessOnCondPlan3288843407985944525 : public DomainPlan
{
public:
    ParallelSuccessOnCondPlan3288843407985944525(PlanContext& context);
    virtual ~ParallelSuccessOnCondPlan3288843407985944525();
    virtual void onInit() override;
};

class UtilityFunction3288843407985944525 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition2767999024419231358 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1470823850869867131 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ParallelSuccessOnCondPlan3288843407985944525)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ParallelSuccessOnCondPlan3288843407985944525UtilityFunction)

} /* namespace alica */
