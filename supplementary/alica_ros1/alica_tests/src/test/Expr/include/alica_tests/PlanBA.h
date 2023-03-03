#pragma once

#include <alica_tests/DomainCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <libalica-tests/util/AlicaTestsPlan.h>

namespace alica_test
{
class SchedWM;
}

namespace alica
{
class PlanBA : public AlicaTestsPlan<PlanBA>
{
public:
    PlanBA(PlanContext& context);

protected:
    virtual void onInit() override;
    virtual void onTerminate() override;

private:
    std::shared_ptr<alica_test::SchedWM> _wm;
};

BOOST_DLL_ALIAS(alica::PlanBA::create, PlanBA)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanBAUtilityFunction)
} /* namespace alica */
