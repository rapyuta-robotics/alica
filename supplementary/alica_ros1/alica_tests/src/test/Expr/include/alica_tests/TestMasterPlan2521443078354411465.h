#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestMasterPlan2521443078354411465 : public DomainPlan
{
public:
    TestMasterPlan2521443078354411465(PlanContext& context);
    virtual ~TestMasterPlan2521443078354411465();

protected:
    virtual void onInit() override;
};

class UtilityFunction2521443078354411465 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1879497210052616817 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition3883605426713053219 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition2733591692277574870 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestMasterPlan2521443078354411465)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestMasterPlan2521443078354411465UtilityFunction)

} /* namespace alica */
