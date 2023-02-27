#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class AdjacentSuccessSubPlan1682631238618360548 : public DomainPlan
{
public:
    AdjacentSuccessSubPlan1682631238618360548(PlanContext& context);
    virtual ~AdjacentSuccessSubPlan1682631238618360548();
};

class UtilityFunction1682631238618360548 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1067314038887345208 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition597347780541336226 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AdjacentSuccessSubPlan1682631238618360548)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AdjacentSuccessSubPlan1682631238618360548UtilityFunction)
} /* namespace alica */
