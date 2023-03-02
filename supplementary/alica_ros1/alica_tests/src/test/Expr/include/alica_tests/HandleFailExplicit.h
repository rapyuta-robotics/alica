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
class HandleFailExplicit : public AlicaTestsPlan<HandleFailExplicit>
{
public:
    HandleFailExplicit(PlanContext& context);

protected:
    virtual void onInit() override;
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

BOOST_DLL_ALIAS(alica::HandleFailExplicit::create, HandleFailExplicit)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, HandleFailExplicitUtilityFunction)
} /* namespace alica */
