#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class HandleFailExplicit1530004915640 : public DomainPlan
{
public:
    HandleFailExplicit1530004915640(PlanContext& context);
    virtual ~HandleFailExplicit1530004915640();

protected:
    virtual void onInit() override;
};

class UtilityFunction1530004915640 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1530004993493 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1530004994611 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1532424093178 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1532424113475 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, HandleFailExplicit1530004915640)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, HandleFailExplicit1530004915640UtilityFunction)
} /* namespace alica */
