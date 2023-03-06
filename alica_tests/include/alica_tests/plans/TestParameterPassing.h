#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestParameterPassing : public AlicaTestsPlan<TestParameterPassing>
{
public:
    TestParameterPassing(PlanContext& context);

protected:
    virtual void onInit() override;
};

BOOST_DLL_ALIAS(alica::TestParameterPassing::create, TestParameterPassing)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestParameterPassingUtilityFunction)

} /* namespace alica */
