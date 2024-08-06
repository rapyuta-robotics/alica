#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestGlobalBlackboardMaster : public AlicaTestsPlan<TestGlobalBlackboardMaster>
{
public:
    TestGlobalBlackboardMaster(PlanContext& context);

protected:
    virtual void run() override;
    virtual void onInit() override;
};

BOOST_DLL_ALIAS(alica::TestGlobalBlackboardMaster::create, TestGlobalBlackboardMaster)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestGlobalBlackboardMasterUtilityFunction)

} /* namespace alica */
