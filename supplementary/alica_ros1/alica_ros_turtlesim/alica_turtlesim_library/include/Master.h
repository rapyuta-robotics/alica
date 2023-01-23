#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

namespace alica
{

class Master : public BasicPlan
{
public:
    Master(PlanContext& context);
    virtual ~Master();
    void onInit() override;
    // Factory method
    static std::unique_ptr<Master> create(PlanContext& context) { return std::make_unique<Master>(context); }
};
BOOST_DLL_ALIAS(alica::Master::create, Master)

class MasterUtilityFunction : public BasicUtilityFunction
{
public:
    MasterUtilityFunction() = default;
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan) override;
    // Factory method
    static std::shared_ptr<MasterUtilityFunction> create(UtilityFunctionContext&) { return std::make_shared<MasterUtilityFunction>(); }
};
BOOST_DLL_ALIAS(alica::MasterUtilityFunction::create, MasterUtilityFunction)

bool conditionMove2Init(const Blackboard* input, const RunningPlan* rp, const Blackboard* globalBlackboard);
BOOST_DLL_ALIAS(alica::conditionMove2Init, Move2Init)

bool conditionInit2Move(const Blackboard* input, const RunningPlan* rp, const Blackboard* globalBlackboard);
BOOST_DLL_ALIAS(alica::conditionInit2Move, Init2Move)

} /* namespace alica */
