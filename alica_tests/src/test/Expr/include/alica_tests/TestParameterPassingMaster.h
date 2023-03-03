#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <libalica-tests/util/AlicaTestsPlan.h>

namespace alica
{
class TestParameterPassingMaster : public AlicaTestsPlan<TestParameterPassingMaster>
{
public:
    TestParameterPassingMaster(PlanContext& context);

private:
    virtual void onInit() override;
};
BOOST_DLL_ALIAS(alica::TestParameterPassingMaster::create, TestParameterPassingMaster)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestParameterPassingMasterUtilityFunction)

} /* namespace alica */
