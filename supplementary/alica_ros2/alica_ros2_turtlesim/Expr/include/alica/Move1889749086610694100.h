#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicCondition.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class Move1889749086610694100 : public BasicPlan
{
public:
    Move1889749086610694100(PlanContext& context);
    virtual ~Move1889749086610694100();
    static std::unique_ptr<Move1889749086610694100> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::Move1889749086610694100::create, Move1889749086610694100)

class UtilityFunction1889749086610694100 : public BasicUtilityFunction
{
public:
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    static std::shared_ptr<UtilityFunction1889749086610694100> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::UtilityFunction1889749086610694100::create, UtilityFunction1889749086610694100)
class RunTimeCondition1288817888979746811 : public BasicCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
} /* namespace alica */
